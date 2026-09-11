from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

REQUIRED = (
    "manifest.json",
    "joint_pos.csv",
    "joint_vel.csv",
    "body_pos.csv",
    "body_quat.csv",
    "body_lin_vel.csv",
    "body_ang_vel.csv",
    "metadata.txt",
)

# The CSVs consumed by SONIC use the articulation order produced by its pinned
# IsaacLab URDF.  Keep the limits beside that contract so an artifact cannot
# pass a single global +/-3.2 rad check while violating a specific G1 joint.
G1_ISAACLAB_JOINT_NAMES = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_ankle_roll_joint",
    "right_ankle_roll_joint", "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint",
    "right_wrist_roll_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
)
G1_LOWER = np.asarray([
    -2.5307, -2.5307, -2.618, -0.5236, -2.9671, -0.52, -2.7576, -2.7576,
    -0.52, -0.087267, -0.087267, -3.0892, -3.0892, -0.87267, -0.87267,
    -1.5882, -2.2515, -0.2618, -0.2618, -2.618, -2.618, -1.0472, -1.0472,
    -1.972222054, -1.972222054, -1.614429558, -1.614429558, -1.614429558,
    -1.614429558,
])
G1_UPPER = np.asarray([
    2.8798, 2.8798, 2.618, 2.9671, 0.5236, 0.52, 2.7576, 2.7576, 0.52,
    2.8798, 2.8798, 2.6704, 2.6704, 0.5236, 0.5236, 2.2515, 1.5882,
    0.2618, 0.2618, 2.618, 2.618, 2.0944, 2.0944, 1.972222054, 1.972222054,
    1.614429558, 1.614429558, 1.614429558, 1.614429558,
])
G1_VELOCITY = np.asarray([
    32.0, 32.0, 32.0, 20.0, 20.0, 37.0, 32.0, 32.0, 37.0, 20.0, 20.0,
    37.0, 37.0, 37.0, 37.0, 37.0, 37.0, 37.0, 37.0, 37.0, 37.0, 37.0,
    37.0, 37.0, 37.0, 22.0, 22.0, 22.0, 22.0,
])
SONIC_OFFICIAL_NEUTRAL = np.asarray([
    -0.312, -0.312, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.669, 0.669, 0.2, 0.2, -0.363, -0.363, 0.2, -0.2, 0.0,
    0.0, 0.0, 0.0, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
])
MAX_NEUTRAL_JOINT_DELTA_RAD = 0.25
VIDEO_REQUIRED = (
    "normalized.mp4", "canonical_smpl.npz", "tracking_report.json",
    "smpl_overlay.mp4", "smpl_world.mp4", "g1_motion.npz", "retarget_preview.mp4",
    "foot_contacts.csv",
)


@dataclass
class ValidationResult:
    valid: bool
    errors: list[str]
    warnings: list[str]
    metrics: dict[str, float | int]


def validate_artifact(path: Path) -> ValidationResult:
    errors: list[str] = []
    warnings: list[str] = []
    missing = [name for name in REQUIRED if not (path / name).is_file()]
    if missing:
        return ValidationResult(False, [f"missing files: {', '.join(missing)}"], [], {})
    try:
        manifest = json.loads((path / "manifest.json").read_text())
        arrays = {
            name: _load(path / name)
            for name in (
                "joint_pos.csv", "joint_vel.csv", "body_pos.csv", "body_quat.csv",
                "body_lin_vel.csv", "body_ang_vel.csv",
            )
        }
    except (ValueError, json.JSONDecodeError) as exc:
        return ValidationResult(False, [f"parse error: {exc}"], [], {})

    if manifest.get("source") == "video":
        missing_video = [name for name in VIDEO_REQUIRED if not (path / name).is_file()]
        if not any((path / name).is_file() for name in ("source.mp4", "source.mov")):
            missing_video.append("source.mp4|source.mov")
        if missing_video:
            errors.append(f"missing video provenance: {', '.join(missing_video)}")
        source_metadata = manifest.get("source_metadata")
        if not isinstance(source_metadata, dict):
            errors.append("video artifact requires source_metadata")
        else:
            video_metadata = source_metadata.get("video", {})
            if video_metadata.get("sha256") != manifest.get("source_sha256"):
                errors.append("video checksum does not match manifest source_sha256")
            if source_metadata.get("static_camera") is not True:
                errors.append("v1 video artifact requires static_camera=true")
            if source_metadata.get("canonical_smpl_schema") != 1:
                errors.append("video artifact requires canonical_smpl_schema=1")
            contact_metrics = source_metadata.get("contact_metrics", {})
            if float(contact_metrics.get("min_ankle_height_m", -1.0)) < -0.01:
                errors.append("video reference penetrates the ground plane")
            if float(contact_metrics.get("max_contact_foot_speed_m_s", 999.0)) > 0.12:
                errors.append("video stance-foot sliding exceeds 0.12 m/s")
        try:
            contacts = _load(path / "foot_contacts.csv")
            if contacts.shape != (len(arrays["joint_pos.csv"]), 2):
                errors.append("foot_contacts.csv must have [num_frames, 2] values")
            elif not np.isin(contacts, (0, 1)).all():
                errors.append("foot_contacts.csv must contain only 0 or 1")
        except ValueError as exc:
            errors.append(f"invalid foot_contacts.csv: {exc}")

    jp, jv, bp, bq, blv, bav = (
        arrays[n]
        for n in (
            "joint_pos.csv", "joint_vel.csv", "body_pos.csv", "body_quat.csv",
            "body_lin_vel.csv", "body_ang_vel.csv",
        )
    )
    frames = {len(v) for v in arrays.values()}
    if len(frames) != 1:
        errors.append(f"frame counts differ: {sorted(frames)}")
    if jp.shape[1] != 29 or jv.shape[1] != 29:
        errors.append("joint_pos and joint_vel must each have 29 columns")
    if bp.shape[1] != 42 or bq.shape[1] != 56:
        errors.append("SONIC body_pos/body_quat must have 14x3/14x4 columns")
    if blv.shape[1] != 42 or bav.shape[1] != 42:
        errors.append("SONIC body_lin_vel/body_ang_vel must each have 14x3 columns")
    for name, value in arrays.items():
        if not np.isfinite(value).all():
            errors.append(f"{name} contains NaN or infinity")

    fps = float(manifest.get("fps", 0))
    if not math.isclose(fps, 50.0):
        errors.append(f"fps must be 50, got {fps}")
    if manifest.get("joint_order") != "g1_29dof_isaaclab":
        errors.append("joint_order must be g1_29dof_isaaclab")
    if manifest.get("quaternion_order") != "wxyz":
        errors.append("quaternion_order must be wxyz")
    if manifest.get("kinematics_source") != "mujoco_fk":
        errors.append("kinematics_source must be mujoco_fk")
    execution_contract = manifest.get("execution_contract", {})
    if execution_contract.get("asset") != "sonic_official_g1":
        errors.append("execution contract must target sonic_official_g1")
    if execution_contract.get("tracker") != "gear_sonic":
        errors.append("execution contract must use gear_sonic")
    if not execution_contract.get("requires_frame_zero_settle"):
        errors.append("execution contract must require frame-zero settling")
    if float(execution_contract.get("frame_zero_settle_s", 0.0)) < 3.0:
        errors.append("frame-zero settling must be at least 3.0 seconds")
    if not execution_contract.get("requires_bootstrap_root_support"):
        errors.append("execution contract must require bootstrap root support")
    if execution_contract.get("neutral_pose") != "sonic_official_g1_default":
        errors.append("execution contract must use sonic_official_g1_default neutral pose")
    if float(execution_contract.get("neutral_transition_s", 0.0)) < 1.0:
        errors.append("neutral transition must be at least 1.0 second")
    neutral_hold_s = float(execution_contract.get("neutral_hold_s", 0.0))
    if neutral_hold_s < 1.0:
        errors.append("neutral hold must be at least 1.0 second")
    if manifest.get("num_frames") != len(jp):
        errors.append("manifest num_frames does not match CSV data")

    if bq.shape[1] == 56:
        norms = np.linalg.norm(bq.reshape(len(bq), 14, 4), axis=2)
        if np.max(np.abs(norms - 1.0)) > 1e-2:
            errors.append("body quaternion is not normalized")
    max_abs_pos = float(np.max(np.abs(jp)))
    max_abs_vel = float(np.max(np.abs(jv)))
    max_acc = float(np.max(np.abs(np.diff(jv, axis=0) * fps))) if len(jv) > 1 else 0.0
    max_step = float(np.max(np.abs(np.diff(jp, axis=0)))) if len(jp) > 1 else 0.0
    if jp.shape[1] == len(G1_ISAACLAB_JOINT_NAMES):
        lower_violation = G1_LOWER[None, :] - jp
        upper_violation = jp - G1_UPPER[None, :]
        pos_violation = np.maximum(lower_violation, upper_violation)
        if float(np.max(pos_violation)) > 0.0:
            frame, joint = np.unravel_index(int(np.argmax(pos_violation)), pos_violation.shape)
            errors.append(
                "joint position limit violation: "
                f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, frame={frame}, "
                f"value={jp[frame, joint]:.4f} rad, "
                f"limit=[{G1_LOWER[joint]:.4f}, {G1_UPPER[joint]:.4f}]"
            )
        velocity_violation = np.abs(jv) - G1_VELOCITY[None, :]
        if float(np.max(velocity_violation)) > 0.0:
            frame, joint = np.unravel_index(int(np.argmax(velocity_violation)), velocity_violation.shape)
            errors.append(
                "joint velocity limit violation: "
                f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, frame={frame}, "
                f"value={jv[frame, joint]:.4f} rad/s, "
                f"limit={G1_VELOCITY[joint]:.4f} rad/s"
            )
    if max_step > 0.35:
        errors.append(f"joint discontinuity exceeds 0.35 rad/frame: {max_step:.3f}")
    if max_acc > 500.0:
        errors.append(f"joint acceleration exceeds conservative limit: {max_acc:.3f} rad/s^2")
    if bp.shape[1] == 42:
        root_pos = bp.reshape(len(bp), 14, 3)[:, 0]
        if np.min(root_pos[:, 2]) < 0.35 or np.max(root_pos[:, 2]) > 1.5:
            errors.append("root height leaves [0.35, 1.5] m")
        horizontal = np.linalg.norm(root_pos[:, :2] - root_pos[0, :2], axis=1)
        if float(np.max(horizontal)) > 2.0:
            errors.append("root displacement exceeds 2.0 m v1 workspace")
        if manifest.get("source") == "video" and float(np.max(horizontal)) > 0.5:
            errors.append("video root displacement exceeds 0.5 m stationary-motion workspace")
    else:
        horizontal = np.asarray([math.inf])
    if bq.shape[1] == 56:
        root_quat = bq.reshape(len(bq), 14, 4)[:, 0]
        # For wxyz, the world-up component of the rotated local z axis is
        # 1-2(x^2+y^2).  Yaw is unrestricted; tilt beyond 60 degrees is not a
        # standing/short-step v1 reference.
        root_up_z = 1.0 - 2.0 * (root_quat[:, 1] ** 2 + root_quat[:, 2] ** 2)
        if float(np.min(root_up_z)) < 0.5:
            errors.append(
                f"root orientation exceeds 60 degree tilt: min_up_z={float(np.min(root_up_z)):.4f}"
            )
    endpoint_delta = np.abs(jp[-1] - jp[0])
    max_endpoint_delta = float(np.max(endpoint_delta))
    if max_endpoint_delta > 0.35:
        joint = int(np.argmax(endpoint_delta))
        errors.append(
            "motion endpoint is discontinuous with playback reset: "
            f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, delta={max_endpoint_delta:.4f} rad"
        )
    start_neutral_delta = np.abs(jp[0] - SONIC_OFFICIAL_NEUTRAL)
    end_neutral_delta = np.abs(jp[-1] - SONIC_OFFICIAL_NEUTRAL)
    max_start_neutral_delta = float(np.max(start_neutral_delta))
    max_end_neutral_delta = float(np.max(end_neutral_delta))
    if max_start_neutral_delta > MAX_NEUTRAL_JOINT_DELTA_RAD:
        joint = int(np.argmax(start_neutral_delta))
        errors.append(
            "frame 0 is discontinuous from SONIC official neutral: "
            f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, "
            f"delta={max_start_neutral_delta:.4f} rad"
        )
    if max_end_neutral_delta > MAX_NEUTRAL_JOINT_DELTA_RAD:
        joint = int(np.argmax(end_neutral_delta))
        errors.append(
            "final frame is discontinuous from SONIC official neutral: "
            f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, "
            f"delta={max_end_neutral_delta:.4f} rad"
        )
    neutral_hold_frames = min(len(jp), max(1, math.ceil(neutral_hold_s * fps)))
    start_hold_delta = np.abs(
        jp[:neutral_hold_frames] - SONIC_OFFICIAL_NEUTRAL[None, :]
    )
    end_hold_delta = np.abs(
        jp[-neutral_hold_frames:] - SONIC_OFFICIAL_NEUTRAL[None, :]
    )
    max_start_hold_delta = float(np.max(start_hold_delta))
    max_end_hold_delta = float(np.max(end_hold_delta))
    if max_start_hold_delta > 0.02:
        frame, joint = np.unravel_index(
            int(np.argmax(start_hold_delta)), start_hold_delta.shape
        )
        errors.append(
            "initial neutral hold is not constant: "
            f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, frame={frame}, "
            f"delta={max_start_hold_delta:.4f} rad"
        )
    if max_end_hold_delta > 0.02:
        frame, joint = np.unravel_index(
            int(np.argmax(end_hold_delta)), end_hold_delta.shape
        )
        absolute_frame = len(jp) - neutral_hold_frames + frame
        errors.append(
            "final neutral hold is not constant: "
            f"joint={G1_ISAACLAB_JOINT_NAMES[joint]}, frame={absolute_frame}, "
            f"delta={max_end_hold_delta:.4f} rad"
        )
    if bp.shape[1] == 42:
        first_root_height = float(root_pos[0, 2])
        final_root_height = float(root_pos[-1, 2])
        if abs(first_root_height - 0.76) > 0.05:
            errors.append(
                f"frame 0 root height is discontinuous from neutral: {first_root_height:.4f} m"
            )
        if abs(final_root_height - 0.76) > 0.05:
            errors.append(
                f"final root height is discontinuous from neutral: {final_root_height:.4f} m"
            )

    metrics = {
        "frames": len(jp),
        "duration_s": len(jp) / fps if fps else 0.0,
        "max_abs_joint_position_rad": max_abs_pos,
        "max_abs_joint_velocity_rad_s": max_abs_vel,
        "max_joint_acceleration_rad_s2": max_acc,
        "max_joint_step_rad": max_step,
        "max_root_displacement_m": float(np.max(horizontal)),
        "min_root_up_z": float(np.min(root_up_z)) if bq.shape[1] == 56 else -1.0,
        "max_endpoint_joint_delta_rad": max_endpoint_delta,
        "max_start_neutral_joint_delta_rad": max_start_neutral_delta,
        "max_end_neutral_joint_delta_rad": max_end_neutral_delta,
        "max_start_neutral_hold_delta_rad": max_start_hold_delta,
        "max_end_neutral_hold_delta_rad": max_end_hold_delta,
    }
    result = ValidationResult(not errors, errors, warnings, metrics)
    (path / "validation.json").write_text(json.dumps(asdict(result), indent=2, sort_keys=True) + "\n")
    return result


def _load(path: Path) -> np.ndarray:
    value = np.loadtxt(path, delimiter=",", skiprows=1)
    return value[None, :] if value.ndim == 1 else value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact", type=Path)
    args = parser.parse_args()
    result = validate_artifact(args.artifact)
    print(json.dumps(asdict(result), indent=2))
    raise SystemExit(0 if result.valid else 2)


if __name__ == "__main__":
    main()
