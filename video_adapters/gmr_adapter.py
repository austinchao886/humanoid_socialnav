#!/usr/bin/env python3
"""Retarget canonical SMPL to Unitree G1 with the pinned official GMR library."""
from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview", type=Path, required=True)
    args = parser.parse_args()
    from general_motion_retargeting import GeneralMotionRetargeting as GMR
    from general_motion_retargeting.utils.smpl import get_gvhmr_data_offline_fast

    gmr_root = Path(os.environ.get("GMR_ROOT", "/opt/GMR"))
    body_models = Path(os.environ.get("SMPLX_MODEL_DIR", gmr_root / "assets/body_models"))
    with np.load(args.input, allow_pickle=False) as data, tempfile.TemporaryDirectory(prefix="gmr-") as temp:
        betas = np.asarray(data["betas"])
        if betas.ndim == 2:
            betas = betas.mean(0)
        source = Path(temp) / "canonical_smplx.npz"
        np.savez(source, pose_body=data["body_pose"], root_orient=data["global_orient"],
                 trans=data["transl"], betas=betas[:10], gender="neutral",
                 mocap_frame_rate=np.asarray(float(np.asarray(data["fps"]).reshape(-1)[0])))
        smplx_data, body_model, smplx_output, human_height = load_smplx_compatible(
            source, body_models
        )
        frames, fps = get_gvhmr_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=30)
    retarget = GMR(actual_human_height=human_height, src_human="smplx",
                   tgt_robot="unitree_g1", use_velocity_limit=True)
    qpos = []
    for frame in frames:
        qpos.append(np.asarray(retarget.retarget(frame), dtype=np.float64))
    values = np.stack(qpos)
    if values.shape[1] != 36:
        raise RuntimeError(f"GMR Unitree G1 output must have 36 qpos values, got {values.shape}")
    mjcf = Path(os.environ.get("G1_MJCF", gmr_root / "assets/unitree_g1/g1_mocap_29dof.xml"))
    values, contacts, contact_metrics = condition_feet(values, float(fps), mjcf)

    render_preview(values, float(fps), mjcf, args.preview)
    np.savez_compressed(args.output, qpos=values, fps=np.asarray([fps]),
                        foot_contacts=contacts, contact_metrics=np.asarray(contact_metrics))


def load_smplx_compatible(source: Path, body_models: Path):
    """Load a multi-frame SMPL-X sequence without the upstream batch mismatch."""
    import smplx
    import torch

    data = np.load(source, allow_pickle=True)
    frames = int(data["pose_body"].shape[0])
    body_model = smplx.create(
        body_models, "smplx", gender=str(data["gender"]), use_pca=False
    )
    zeros_45 = torch.zeros(frames, 45, dtype=torch.float32)
    zeros_3 = torch.zeros(frames, 3, dtype=torch.float32)
    shape_betas = torch.as_tensor(data["betas"], dtype=torch.float32).reshape(1, -1)
    shape_betas = shape_betas[:, :body_model.num_betas]
    output = body_model(
        betas=shape_betas,
        global_orient=torch.as_tensor(data["root_orient"], dtype=torch.float32),
        body_pose=torch.as_tensor(data["pose_body"], dtype=torch.float32),
        transl=torch.as_tensor(data["trans"], dtype=torch.float32),
        left_hand_pose=zeros_45,
        right_hand_pose=zeros_45,
        jaw_pose=zeros_3,
        leye_pose=zeros_3,
        reye_pose=zeros_3,
        expression=torch.zeros(frames, 10, dtype=torch.float32),
        return_full_pose=True,
    )
    human_height = 1.66 + 0.1 * float(np.asarray(data["betas"]).reshape(-1)[0])
    return data, body_model, output, human_height


def condition_feet(qpos: np.ndarray, fps: float, mjcf: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Ground-align G1 FK, infer contacts, and reduce stance-foot sliding."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    if model.nq != 36:
        raise RuntimeError(f"G1 retarget MJCF must have nq=36, got {model.nq}")
    foot_ids = [model.body(name).id for name in ("left_ankle_roll_link", "right_ankle_roll_link")]
    data = mujoco.MjData(model)

    def fk(values: np.ndarray) -> np.ndarray:
        positions = np.empty((len(values), 2, 3), dtype=np.float64)
        for index, value in enumerate(values):
            data.qpos[:] = value
            mujoco.mj_forward(model, data)
            positions[index] = data.xpos[foot_ids]
        return positions

    result = np.asarray(qpos, dtype=np.float64).copy()
    feet = fk(result)
    # Ankle-roll origins sit about 5 cm above the sole in the pinned G1 MJCF.
    result[:, 2] += 0.05 - float(np.min(feet[:, :, 2]))
    feet = fk(result)
    speed = np.linalg.norm(np.gradient(feet[:, :, :2], 1.0 / fps, axis=0), axis=2)
    low = feet[:, :, 2] <= float(np.min(feet[:, :, 2])) + 0.035
    contacts = low & (speed <= 0.12)

    correction = np.zeros((len(result), 2), dtype=np.float64)
    for foot in range(2):
        for start, end in _true_runs(contacts[:, foot]):
            if end - start < 3:
                continue
            anchor = feet[start, foot, :2]
            correction[start:end] += anchor[None, :] - feet[start:end, foot, :2]
    active = np.maximum(contacts.sum(axis=1), 1)[:, None]
    correction /= active
    correction = np.clip(correction, -0.08, 0.08)
    if len(correction) > 1:
        kernel = np.asarray([1, 4, 6, 4, 1], dtype=np.float64) / 16.0
        padded = np.pad(correction, ((2, 2), (0, 0)), mode="edge")
        correction = np.stack([np.convolve(padded[:, axis], kernel, mode="valid")
                               for axis in range(2)], axis=1)
    result[:, :2] += correction
    feet = fk(result)
    speed = np.linalg.norm(np.gradient(feet[:, :, :2], 1.0 / fps, axis=0), axis=2)
    contacts = (feet[:, :, 2] <= float(np.min(feet[:, :, 2])) + 0.035) & (speed <= 0.12)
    contact_speed = speed[contacts]
    metrics = np.asarray([
        float(np.min(feet[:, :, 2])),
        float(np.max(contact_speed)) if contact_speed.size else 0.0,
        float(np.mean(contacts[:, 0])), float(np.mean(contacts[:, 1])),
    ])
    return result, contacts.astype(np.uint8), metrics


def _true_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    padded = np.pad(np.asarray(mask, dtype=np.int8), (1, 1))
    changes = np.diff(padded)
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def render_preview(qpos: np.ndarray, fps: float, mjcf: Path, output: Path) -> None:
    """Render the exact conditioned reference without requiring an X display."""
    import imageio.v2 as imageio
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    # Match the fixed Isaac replay camera: eye=(2.8, -3.0, 1.75),
    # lookat=(0.0, 0.0, 0.82), rendered at 1280x720.
    renderer = mujoco.Renderer(model, height=720, width=1280)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    # MuJoCo's default vertical FOV is wider than the Isaac replay camera;
    # 3.1 m gives equivalent on-screen G1 framing to Isaac's 4.2 m eye radius.
    camera.distance = 3.1
    camera.azimuth = -45.0
    camera.elevation = -13.0
    writer = imageio.get_writer(str(output), fps=fps)
    try:
        for value in qpos:
            data.qpos[:] = value
            mujoco.mj_forward(model, data)
            camera.lookat[:] = data.xpos[model.body("pelvis").id]
            renderer.update_scene(data, camera=camera)
            writer.append_data(renderer.render())
    finally:
        writer.close()
        renderer.close()


if __name__ == "__main__":
    main()
