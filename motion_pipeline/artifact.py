from __future__ import annotations

import csv
import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

# qpos from Kimodo is MuJoCo order; SONIC reference CSV is IsaacLab order.
MUJOCO_TO_ISAACLAB = np.asarray(
    [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
)
SONIC_OFFICIAL_NEUTRAL_ISAACLAB = np.asarray([
    -0.312, -0.312, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.669, 0.669, 0.2, 0.2, -0.363, -0.363, 0.2, -0.2, 0.0,
    0.0, 0.0, 0.0, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
], dtype=np.float64)
SONIC_BODY_NAMES = [
    "pelvis", "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
    "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link", "torso_link",
    "left_shoulder_roll_link", "left_elbow_link", "left_wrist_yaw_link",
    "right_shoulder_roll_link", "right_elbow_link", "right_wrist_yaw_link",
]
SONIC_BODY_INDEXES = [0, 4, 10, 18, 5, 11, 19, 9, 16, 22, 28, 17, 23, 29]
SONIC_PREVIEW_EDGES = [
    (0, 1), (1, 2), (2, 3),
    (0, 4), (4, 5), (5, 6),
    (0, 7),
    (7, 8), (8, 9), (9, 10),
    (7, 11), (11, 12), (12, 13),
]


def convert_kimodo_qpos(
    qpos_csv: Path, artifact_dir: Path, *, request_id: str, motion_id: str,
    prompt: str, seed: int, model: str, fps: float = 50.0,
    source_fps: float = 30.0, smoothing_sigma_frames: float = 0.75,
    neutral_transition_s: float = 2.0, neutral_hold_s: float = 1.0,
) -> dict:
    return convert_g1_qpos(
        qpos_csv, artifact_dir, request_id=request_id, motion_id=motion_id,
        source="kimodo", model=model, prompt=prompt, seed=seed, fps=fps,
        source_fps=source_fps, smoothing_sigma_frames=smoothing_sigma_frames,
        neutral_transition_s=neutral_transition_s, neutral_hold_s=neutral_hold_s,
    )


def convert_g1_qpos(
    qpos_csv: Path,
    artifact_dir: Path,
    *,
    request_id: str,
    motion_id: str,
    source: str,
    model: str,
    prompt: str = "",
    seed: int | None = None,
    fps: float = 50.0,
    source_fps: float = 30.0,
    smoothing_sigma_frames: float = 0.75,
    neutral_transition_s: float = 2.0,
    neutral_hold_s: float = 1.0,
    source_metadata: dict | None = None,
    source_sha256: str | None = None,
    source_contacts_csv: Path | None = None,
) -> dict:
    qpos = _load_kimodo_csv(qpos_csv)
    if qpos.ndim == 1:
        qpos = qpos[None, :]
    if qpos.shape[1] != 36:
        raise ValueError(f"Kimodo G1 qpos must have 36 columns; got {qpos.shape[1]}")

    source_frames = len(qpos)
    source_contacts = None
    if source_contacts_csv is not None:
        source_contacts = np.loadtxt(source_contacts_csv, delimiter=",", skiprows=1)
        source_contacts = source_contacts[None, :] if source_contacts.ndim == 1 else source_contacts
        if source_contacts.shape != (source_frames, 2) or not np.isin(source_contacts, (0, 1)).all():
            raise ValueError("source foot contacts must be binary [source_frames, 2]")
    qpos = _condition_qpos(qpos, source_fps, fps, smoothing_sigma_frames)
    conditioned_frames = len(qpos)
    if source_contacts is not None:
        indices = np.rint(np.linspace(0, source_frames - 1, conditioned_frames)).astype(int)
        conditioned_contacts = source_contacts[indices].astype(np.uint8)
    qpos = _add_neutral_transitions(
        qpos, fps, neutral_transition_s, neutral_hold_s
    )
    artifact_dir.mkdir(parents=True, exist_ok=False)
    root_pos = qpos[:, :3]
    root_quat = qpos[:, 3:7]
    joint_pos = qpos[:, 7:][:, MUJOCO_TO_ISAACLAB]
    joint_vel = np.gradient(joint_pos, 1.0 / fps, axis=0, edge_order=1)

    _write_csv(artifact_dir / "joint_pos.csv", joint_pos, [f"joint_{i}" for i in range(29)])
    _write_csv(artifact_dir / "joint_vel.csv", joint_vel, [f"joint_vel_{i}" for i in range(29)])
    if source_contacts is not None:
        edge_frames = (len(qpos) - conditioned_frames) // 2
        contacts = np.vstack((np.ones((edge_frames, 2), dtype=np.uint8),
                              conditioned_contacts,
                              np.ones((len(qpos) - conditioned_frames - edge_frames, 2), dtype=np.uint8)))
        _write_csv(artifact_dir / "foot_contacts.csv", contacts,
                   ["left_contact", "right_contact"])
    mjcf = Path(os.getenv("G1_MJCF", os.getenv("KIMODO_G1_MJCF", "/workspace/kimodo/kimodo/assets/skeletons/g1skel34/xml/g1.xml")))
    try:
        body_pos, body_quat = _mujoco_body_kinematics(qpos, mjcf)
        kinematics_source = "mujoco_fk"
    except (ImportError, FileNotFoundError):
        # Dependency-light unit-test fallback. The generator service refuses to
        # publish this fallback as executable output.
        body_pos = np.repeat(root_pos[:, None, :], len(SONIC_BODY_NAMES), axis=1)
        body_quat = np.repeat(root_quat[:, None, :], len(SONIC_BODY_NAMES), axis=1)
        kinematics_source = "root_only_test_fallback"
    body_lin_vel = np.gradient(body_pos, 1.0 / fps, axis=0, edge_order=1)
    body_ang_vel = _quat_angular_velocity(body_quat, fps)
    _write_csv(artifact_dir / "body_pos.csv", body_pos.reshape(len(qpos), -1), [f"body_{i}_{axis}" for i in range(14) for axis in "xyz"])
    _write_csv(artifact_dir / "body_quat.csv", body_quat.reshape(len(qpos), -1), [f"body_{i}_{axis}" for i in range(14) for axis in "wxyz"])
    _write_csv(artifact_dir / "body_lin_vel.csv", body_lin_vel.reshape(len(qpos), -1), [f"body_{i}_vel_{axis}" for i in range(14) for axis in "xyz"])
    _write_csv(artifact_dir / "body_ang_vel.csv", body_ang_vel.reshape(len(qpos), -1), [f"body_{i}_angvel_{axis}" for i in range(14) for axis in "xyz"])
    metadata = (
        f"Metadata for: {motion_id}\n==============================\n\n"
        f"Body part indexes:\n{SONIC_BODY_INDEXES}\n\nTotal timesteps: {len(qpos)}\n"
    )
    (artifact_dir / "metadata.txt").write_text(metadata)
    manifest = {
        "schema_version": 1,
        "request_id": request_id,
        "motion_id": motion_id,
        "source": source,
        "model": model,
        "prompt": prompt,
        "seed": seed,
        "fps": fps,
        "source_fps": source_fps,
        "source_num_frames": source_frames,
        "conditioned_motion_frames": conditioned_frames,
        "temporal_conditioning": {
            "joint_gaussian_sigma_source_frames": smoothing_sigma_frames,
            "resampling": "linear_position_nlerp_quaternion",
            "neutral_transition_s_each_end": neutral_transition_s,
            "neutral_hold_s_each_end": neutral_hold_s,
            "neutral_transition": "cubic_smoothstep",
        },
        "num_frames": len(qpos),
        "joint_order": "g1_29dof_isaaclab",
        "quaternion_order": "wxyz",
        "body_names": SONIC_BODY_NAMES,
        "kinematics_source": kinematics_source,
        "execution_contract": {
            "asset": "sonic_official_g1",
            "tracker": "gear_sonic",
            "requires_frame_zero_settle": True,
            "frame_zero_settle_s": 3.0,
            "requires_bootstrap_root_support": True,
            "neutral_pose": "sonic_official_g1_default",
            "neutral_transition_s": neutral_transition_s,
            "neutral_hold_s": neutral_hold_s,
        },
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_sha256": source_sha256 or hashlib.sha256(qpos_csv.read_bytes()).hexdigest(),
    }
    if source_metadata:
        manifest["source_metadata"] = source_metadata
    (artifact_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return manifest


def _load_kimodo_csv(path: Path) -> np.ndarray:
    """Load official Kimodo CSVs, which are headerless.

    Older unit fixtures included a header, so detect it instead of blindly
    dropping row zero.  The old converter's unconditional ``skiprows=1`` lost
    the first real Kimodo frame.
    """
    first = path.read_text().splitlines()[0]
    try:
        first_values = np.asarray([float(value) for value in first.split(",")])
        # Some early fixtures used a purely numeric 0..35 header, which is
        # syntactically indistinguishable from a data row unless we recognize
        # that exact sentinel.  Official Kimodo output remains headerless.
        skiprows = int(
            len(first_values) == 36
            and np.array_equal(first_values, np.arange(36, dtype=np.float64))
        )
    except ValueError:
        skiprows = 1
    value = np.loadtxt(path, delimiter=",", skiprows=skiprows)
    return value[None, :] if value.ndim == 1 else value


def _condition_qpos(qpos: np.ndarray, source_fps: float, target_fps: float, sigma: float) -> np.ndarray:
    """Condition model output before validation without changing safety gates.

    Kimodo-G1 emits 30 Hz references.  We apply a small documented Gaussian
    anti-alias filter to the 1-DoF joints, then resample to SONIC's 50 Hz. Root
    translation is linearly interpolated and root quaternion uses normalized
    shortest-arc interpolation.  This repairs model-frame jitter; validation
    still evaluates the resulting reference with its original thresholds.
    """
    if source_fps <= 0 or target_fps <= 0:
        raise ValueError("source and target fps must be positive")
    result = qpos.astype(np.float64, copy=True)
    if sigma > 0 and len(result) > 1:
        radius = int(np.ceil(4.0 * sigma))
        offsets = np.arange(-radius, radius + 1, dtype=np.float64)
        kernel = np.exp(-0.5 * (offsets / sigma) ** 2)
        kernel /= kernel.sum()
        padded = np.pad(result[:, 7:], ((radius, radius), (0, 0)), mode="edge")
        result[:, 7:] = np.stack(
            [np.convolve(padded[:, joint], kernel, mode="valid") for joint in range(29)], axis=1
        )
    if np.isclose(source_fps, target_fps) or len(result) < 2:
        return result
    source_t = np.arange(len(result), dtype=np.float64) / source_fps
    target_t = np.arange(0.0, source_t[-1] + 1e-12, 1.0 / target_fps)
    output = np.empty((len(target_t), 36), dtype=np.float64)
    for column in list(range(3)) + list(range(7, 36)):
        output[:, column] = np.interp(target_t, source_t, result[:, column])
    indices = np.minimum(np.searchsorted(source_t, target_t, side="right") - 1, len(result) - 2)
    indices = np.maximum(indices, 0)
    alpha = ((target_t - source_t[indices]) * source_fps)[:, None]
    qa, qb = result[indices, 3:7], result[indices + 1, 3:7]
    qb = np.where(np.sum(qa * qb, axis=1, keepdims=True) < 0.0, -qb, qb)
    quat = (1.0 - alpha) * qa + alpha * qb
    output[:, 3:7] = quat / np.maximum(np.linalg.norm(quat, axis=1, keepdims=True), 1e-12)
    return output


def _add_neutral_transitions(
    qpos: np.ndarray,
    fps: float,
    transition_s: float,
    hold_s: float,
) -> np.ndarray:
    """Enter and leave model motion from SONIC's exact trained neutral pose."""
    if transition_s < 1.0:
        raise ValueError("neutral transition must be at least 1.0 second")
    if hold_s < 1.0:
        raise ValueError("neutral hold must be at least 1.0 second")
    if len(qpos) == 0:
        raise ValueError("cannot pad an empty motion")
    transition_frames = max(1, round(transition_s * fps))
    hold_frames = max(1, round(hold_s * fps))

    neutral_start = _neutral_qpos(qpos[0])
    neutral_end = _neutral_qpos(qpos[-1])
    pre_alpha = np.arange(transition_frames, dtype=np.float64) / transition_frames
    post_alpha = np.arange(1, transition_frames + 1, dtype=np.float64) / transition_frames
    pre = _blend_qpos(neutral_start, qpos[0], pre_alpha)
    post = _blend_qpos(qpos[-1], neutral_end, post_alpha)
    return np.vstack(
        (
            np.repeat(neutral_start[None, :], hold_frames, axis=0),
            pre,
            qpos,
            post,
            np.repeat(neutral_end[None, :], hold_frames, axis=0),
        )
    )


def _neutral_qpos(anchor: np.ndarray) -> np.ndarray:
    neutral = np.zeros(36, dtype=np.float64)
    neutral[:2] = anchor[:2]
    neutral[2] = 0.76
    neutral[3:7] = _heading_quaternion(anchor[3:7])
    # convert_kimodo_qpos selects qpos[:, 7:][:, MUJOCO_TO_ISAACLAB],
    # so assign the inverse relation here.
    neutral[7:][MUJOCO_TO_ISAACLAB] = SONIC_OFFICIAL_NEUTRAL_ISAACLAB
    return neutral


def _heading_quaternion(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64)
    q = q / max(float(np.linalg.norm(q)), 1e-12)
    w, x, y, z = q
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.asarray([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def _blend_qpos(start: np.ndarray, end: np.ndarray, alpha: np.ndarray) -> np.ndarray:
    eased = (alpha * alpha * (3.0 - 2.0 * alpha))[:, None]
    output = (1.0 - eased) * start[None, :] + eased * end[None, :]
    qa = start[3:7]
    qb = end[3:7]
    if float(np.dot(qa, qb)) < 0.0:
        qb = -qb
    quat = (1.0 - eased) * qa[None, :] + eased * qb[None, :]
    output[:, 3:7] = quat / np.maximum(
        np.linalg.norm(quat, axis=1, keepdims=True), 1e-12
    )
    return output


def _mujoco_body_kinematics(qpos: np.ndarray, mjcf: Path) -> tuple[np.ndarray, np.ndarray]:
    if not mjcf.is_file():
        raise FileNotFoundError(mjcf)
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    if model.nq != qpos.shape[1]:
        raise ValueError(f"G1 MJCF nq={model.nq}, Kimodo qpos={qpos.shape[1]}")
    ids = []
    for name in SONIC_BODY_NAMES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"G1 MJCF is missing tracked body {name}")
        ids.append(body_id)
    data = mujoco.MjData(model)
    positions = np.empty((len(qpos), len(ids), 3), dtype=np.float64)
    quaternions = np.empty((len(qpos), len(ids), 4), dtype=np.float64)
    for frame, values in enumerate(qpos):
        data.qpos[:] = values
        mujoco.mj_forward(model, data)
        positions[frame] = data.xpos[ids]
        quaternions[frame] = data.xquat[ids]  # MuJoCo is wxyz.
    return positions, quaternions


def _quat_angular_velocity(quat: np.ndarray, fps: float) -> np.ndarray:
    result = np.zeros(quat.shape[:-1] + (3,), dtype=np.float64)
    if len(quat) < 2:
        return result
    # q_delta = q_next * conjugate(q_current), wxyz; use shortest arc.
    a, b = quat[:-1], quat[1:]
    aw, av = a[..., :1], a[..., 1:]
    bw, bv = b[..., :1], b[..., 1:]
    dw = bw * aw + np.sum(bv * av, axis=-1, keepdims=True)
    dv = -bw * av + aw * bv - np.cross(bv, av)
    sign = np.where(dw < 0.0, -1.0, 1.0)
    dv, dw = dv * sign, np.clip(dw * sign, -1.0, 1.0)
    angle = 2.0 * np.arctan2(np.linalg.norm(dv, axis=-1, keepdims=True), dw)
    axis = dv / np.maximum(np.linalg.norm(dv, axis=-1, keepdims=True), 1e-9)
    result[1:] = axis * angle * fps
    result[0] = result[1]
    return result


def render_preview(npz_path: Path, output_path: Path, fps: float = 50.0) -> None:
    """Render a dependency-light diagnostic preview from Kimodo global joints."""
    import av
    from PIL import Image, ImageDraw

    with np.load(npz_path, allow_pickle=False) as data:
        joints = np.asarray(data["posed_joints"])
    width, height = 640, 480
    container = av.open(str(output_path), mode="w")
    stream = container.add_stream("h264", rate=int(fps))
    stream.width, stream.height, stream.pix_fmt = width, height, "yuv420p"
    points = joints[..., [0, 2]]
    center = np.mean(points.reshape(-1, 2), axis=0)
    extent = max(float(np.max(np.abs(points - center))), 0.5)
    for frame_points in points:
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)
        normalized = (frame_points - center) / (2.2 * extent)
        pixels = np.column_stack((width * (0.5 + normalized[:, 0]), height * (0.8 - normalized[:, 1])))
        for x, y in pixels:
            draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill="black")
        frame = av.VideoFrame.from_image(image)
        for packet in stream.encode(frame):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()


def render_artifact_preview(
    artifact_dir: Path, output_path: Path, fps: float = 50.0
) -> None:
    """Render the exact body trajectory sent to SONIC, including transitions."""
    import av
    from PIL import Image, ImageDraw

    body_pos = np.loadtxt(
        artifact_dir / "body_pos.csv", delimiter=",", skiprows=1
    ).reshape(-1, len(SONIC_BODY_NAMES), 3)
    width, height = 640, 480
    container = av.open(str(output_path), mode="w")
    stream = container.add_stream("h264", rate=int(fps))
    stream.width, stream.height, stream.pix_fmt = width, height, "yuv420p"
    points = body_pos[..., [0, 2]]
    center = np.mean(points.reshape(-1, 2), axis=0)
    extent = max(float(np.max(np.abs(points - center))), 0.5)
    for frame_points in points:
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)
        normalized = (frame_points - center) / (2.2 * extent)
        pixels = np.column_stack(
            (width * (0.5 + normalized[:, 0]), height * (0.8 - normalized[:, 1]))
        )
        for start, end in SONIC_PREVIEW_EDGES:
            draw.line(
                (pixels[start, 0], pixels[start, 1], pixels[end, 0], pixels[end, 1]),
                fill="gray",
                width=2,
            )
        for x, y in pixels:
            draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill="black")
        frame = av.VideoFrame.from_image(image)
        for packet in stream.encode(frame):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()


def _write_csv(path: Path, values: np.ndarray, headers: list[str]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)
        writer.writerows(values.tolist())
