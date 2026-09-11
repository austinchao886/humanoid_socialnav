from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from motion_contracts.artifact import (
    MUJOCO_TO_ISAACLAB,
    SONIC_BODY_INDEXES,
    SONIC_BODY_NAMES,
    SONIC_OFFICIAL_NEUTRAL_ISAACLAB,
    _mujoco_body_kinematics,
    _quat_angular_velocity,
    _write_csv,
    render_artifact_preview,
)
from motion_contracts.validator import validate_artifact


FPS = 50.0
NEUTRAL_HOLD_S = 1.0
TRANSITION_S = 1.5
KINDS = ("neutral", "arm_raise", "shallow_squat")


def deterministic_trajectory(kind: str, fps: float = FPS) -> np.ndarray:
    """Return an absolute 36-qpos G1 trajectory in MuJoCo joint order."""
    if kind not in KINDS:
        raise ValueError(f"unknown deterministic motion: {kind}")
    hold_frames = round(NEUTRAL_HOLD_S * fps)
    transition_frames = round(TRANSITION_S * fps)
    neutral = _qpos(SONIC_OFFICIAL_NEUTRAL_ISAACLAB, root_height=0.76)

    if kind == "neutral":
        return np.repeat(neutral[None, :], round(4.0 * fps), axis=0)

    target_joints = SONIC_OFFICIAL_NEUTRAL_ISAACLAB.copy()
    target_height = 0.76
    if kind == "arm_raise":
        # A slow, left-arm-only reach.  Lower body and floating base remain at
        # the trained neutral pose so this isolates upper-body tracking.
        target_joints[11] = -0.90  # left shoulder pitch
        target_joints[15] = 0.35   # left shoulder roll
        target_joints[21] = 0.40   # left elbow
    else:
        # Symmetric shallow squat, deliberately milder than the official
        # squat acceptance reference.
        target_joints[[0, 1]] = -0.45
        target_joints[[9, 10]] = 0.90
        target_joints[[13, 14]] = -0.45
        target_height = 0.70
    target = _qpos(target_joints, root_height=target_height)

    alpha = np.arange(1, transition_frames + 1, dtype=np.float64)
    alpha /= transition_frames
    alpha = alpha * alpha * (3.0 - 2.0 * alpha)
    outward = (1.0 - alpha[:, None]) * neutral + alpha[:, None] * target
    inward = (1.0 - alpha[:, None]) * target + alpha[:, None] * neutral
    return np.vstack(
        (
            np.repeat(neutral[None, :], hold_frames, axis=0),
            outward,
            np.repeat(target[None, :], hold_frames, axis=0),
            inward,
            np.repeat(neutral[None, :], hold_frames, axis=0),
        )
    )


def create_deterministic_artifact(
    artifact_dir: Path,
    *,
    kind: str,
    request_id: str,
    motion_id: str,
    mjcf: Path,
    render_preview: bool = True,
) -> dict:
    qpos = deterministic_trajectory(kind)
    artifact_dir.mkdir(parents=True, exist_ok=False)
    joint_pos = qpos[:, 7:][:, MUJOCO_TO_ISAACLAB]
    joint_vel = np.gradient(joint_pos, 1.0 / FPS, axis=0, edge_order=1)
    body_pos, body_quat = _mujoco_body_kinematics(qpos, mjcf)
    body_lin_vel = np.gradient(body_pos, 1.0 / FPS, axis=0, edge_order=1)
    body_ang_vel = _quat_angular_velocity(body_quat, FPS)

    _write_csv(
        artifact_dir / "joint_pos.csv",
        joint_pos,
        [f"joint_{index}" for index in range(29)],
    )
    _write_csv(
        artifact_dir / "joint_vel.csv",
        joint_vel,
        [f"joint_vel_{index}" for index in range(29)],
    )
    _write_csv(
        artifact_dir / "body_pos.csv",
        body_pos.reshape(len(qpos), -1),
        [f"body_{index}_{axis}" for index in range(14) for axis in "xyz"],
    )
    _write_csv(
        artifact_dir / "body_quat.csv",
        body_quat.reshape(len(qpos), -1),
        [f"body_{index}_{axis}" for index in range(14) for axis in "wxyz"],
    )
    _write_csv(
        artifact_dir / "body_lin_vel.csv",
        body_lin_vel.reshape(len(qpos), -1),
        [f"body_{index}_vel_{axis}" for index in range(14) for axis in "xyz"],
    )
    _write_csv(
        artifact_dir / "body_ang_vel.csv",
        body_ang_vel.reshape(len(qpos), -1),
        [f"body_{index}_angvel_{axis}" for index in range(14) for axis in "xyz"],
    )
    (artifact_dir / "metadata.txt").write_text(
        f"Metadata for: {motion_id}\n"
        "==============================\n\n"
        f"Body part indexes:\n{SONIC_BODY_INDEXES}\n\n"
        f"Total timesteps: {len(qpos)}\n"
    )
    digest = hashlib.sha256(qpos.tobytes()).hexdigest()
    manifest = {
        "schema_version": 1,
        "request_id": request_id,
        "motion_id": motion_id,
        "source": "deterministic_v1",
        "model": None,
        "prompt": f"deterministic {kind}",
        "seed": 0,
        "fps": FPS,
        "num_frames": len(qpos),
        "joint_order": "g1_29dof_isaaclab",
        "quaternion_order": "wxyz",
        "body_names": SONIC_BODY_NAMES,
        "kinematics_source": "mujoco_fk",
        "deterministic_motion": kind,
        "trajectory_sha256": digest,
        "execution_contract": {
            "asset": "sonic_official_g1",
            "tracker": "gear_sonic",
            "requires_frame_zero_settle": True,
            "frame_zero_settle_s": 3.0,
            "requires_bootstrap_root_support": True,
            "neutral_pose": "sonic_official_g1_default",
            "neutral_transition_s": TRANSITION_S,
            "neutral_hold_s": NEUTRAL_HOLD_S,
        },
        "created_at": datetime.now(timezone.utc).isoformat(),
    }
    (artifact_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    if render_preview:
        render_artifact_preview(artifact_dir, artifact_dir / "preview.mp4", FPS)
    return manifest


def _qpos(joint_pos_isaaclab: np.ndarray, *, root_height: float) -> np.ndarray:
    qpos = np.zeros(36, dtype=np.float64)
    qpos[2] = root_height
    qpos[3] = 1.0
    qpos[7:][MUJOCO_TO_ISAACLAB] = joint_pos_isaaclab
    return qpos


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create validated deterministic SONIC reference artifacts"
    )
    parser.add_argument("--exchange", type=Path, default=Path("/motion_exchange"))
    parser.add_argument("--kind", choices=(*KINDS, "all"), default="all")
    parser.add_argument(
        "--mjcf",
        type=Path,
        default=Path(
            os.getenv(
                "SONIC_G1_MJCF",
                "/sonic_robot_description/mjcf/g1_29dof_rev_1_0.xml",
            )
        ),
    )
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args()
    kinds = KINDS if args.kind == "all" else (args.kind,)
    summary = []
    args.exchange.mkdir(parents=True, exist_ok=True)
    for kind in kinds:
        motion_id = f"isaac-{kind.replace('_', '-')}-v1"
        request_id = motion_id
        destination = args.exchange / motion_id
        if destination.exists():
            raise FileExistsError(
                f"immutable artifact already exists; refusing overwrite: {destination}"
            )
        with tempfile.TemporaryDirectory(
            dir=args.exchange, prefix=".deterministic-"
        ) as temporary:
            staging = Path(temporary) / motion_id
            manifest = create_deterministic_artifact(
                staging,
                kind=kind,
                request_id=request_id,
                motion_id=motion_id,
                mjcf=args.mjcf,
                render_preview=not args.no_preview,
            )
            validation = validate_artifact(staging)
            os.rename(staging, destination)
        summary.append(
            {
                "motion_id": motion_id,
                "request_id": request_id,
                "state": (
                    "READY_FOR_APPROVAL"
                    if validation.valid
                    else "INVALID_REFERENCE"
                ),
                "artifact_path": str(destination),
                "num_frames": manifest["num_frames"],
                "validation": validation.__dict__,
            }
        )
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if all(x["validation"]["valid"] for x in summary) else 2)


if __name__ == "__main__":
    main()
