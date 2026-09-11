"""Explicit G1 asset compatibility profiles for the SONIC execution bridge."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


UNITREE_G1_MOTOR_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)


@dataclass(frozen=True)
class AssetProfile:
    profile_id: str
    isaac_task: str
    source_asset: str
    qualification: str
    contract_to_asset_joint: dict[str, str]
    joint_axis_sign: dict[str, int]
    joint_effort_scale: dict[str, float]
    body_mass_override_kg: dict[str, float]
    allowed_extra_joints: tuple[str, ...]
    physics_dt_s: float
    solver_position_iteration_count: int
    solver_velocity_iteration_count: int
    notes: tuple[str, ...]

    @property
    def qualified(self) -> bool:
        return self.qualification == "qualified"

    def assert_task(self, task: str) -> None:
        if task != self.isaac_task:
            raise ValueError(
                f"asset profile {self.profile_id} requires task {self.isaac_task}, got {task}"
            )

    def assert_articulation(self, joint_names: list[str]) -> None:
        mapped = [self.contract_to_asset_joint[name] for name in UNITREE_G1_MOTOR_JOINTS]
        if len(set(mapped)) != len(mapped):
            raise ValueError(f"asset profile {self.profile_id} maps multiple motors to one joint")
        missing = [name for name in mapped if name not in joint_names]
        if missing:
            raise ValueError(f"asset profile {self.profile_id} is missing joints: {missing}")
        extras = [name for name in joint_names if name not in mapped]
        unexpected = [name for name in extras if name not in self.allowed_extra_joints]
        absent_expected = [name for name in self.allowed_extra_joints if name not in extras]
        if unexpected or absent_expected:
            raise ValueError(
                f"asset profile {self.profile_id} extra-joint mismatch: "
                f"unexpected={unexpected}, absent_expected={absent_expected}"
            )

    def runtime_contract(self) -> dict:
        return {
            "profile_id": self.profile_id,
            "qualified": self.qualified,
            "qualification": self.qualification,
            "contract_to_asset_joint": dict(self.contract_to_asset_joint),
            "joint_axis_sign": dict(self.joint_axis_sign),
            "joint_effort_scale": dict(self.joint_effort_scale),
            "body_mass_override_kg": dict(self.body_mass_override_kg),
            "solver_position_iteration_count": self.solver_position_iteration_count,
            "solver_velocity_iteration_count": self.solver_velocity_iteration_count,
        }


def load_asset_profile(profile: str, config_dir: Path) -> AssetProfile:
    """Load and strictly validate a named JSON profile or explicit JSON path."""

    path = Path(profile)
    if not path.suffix:
        path = config_dir / f"{profile}.json"
    path = path.resolve()
    config_root = config_dir.resolve()
    if not path.is_relative_to(config_root):
        raise ValueError(f"asset profile must be under {config_root}: {path}")
    try:
        raw = json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise ValueError(f"unknown asset profile: {profile}") from exc
    if raw.get("schema_version") != 1:
        raise ValueError(f"unsupported asset profile schema: {raw.get('schema_version')}")

    expected = set(UNITREE_G1_MOTOR_JOINTS)
    mapping = raw.get("contract_to_asset_joint")
    signs = raw.get("joint_axis_sign")
    if not isinstance(mapping, dict) or set(mapping) != expected:
        raise ValueError("contract_to_asset_joint must define exactly the 29 Unitree motors")
    if not isinstance(signs, dict) or set(signs) != expected:
        raise ValueError("joint_axis_sign must define exactly the 29 Unitree motors")
    if any(value not in (-1, 1) for value in signs.values()):
        raise ValueError("every joint_axis_sign must be -1 or 1")
    raw_effort_scale = raw.get("joint_effort_scale")
    if raw_effort_scale is None:
        effort_scale = {name: 1.0 for name in UNITREE_G1_MOTOR_JOINTS}
    else:
        if not isinstance(raw_effort_scale, dict) or set(raw_effort_scale) != expected:
            raise ValueError("joint_effort_scale must define exactly the 29 Unitree motors")
        effort_scale = {
            str(name): float(raw_effort_scale[name])
            for name in UNITREE_G1_MOTOR_JOINTS
        }
        if any(
            not math.isfinite(value) or not 0.5 <= value <= 1.5
            for value in effort_scale.values()
        ):
            raise ValueError("every joint_effort_scale must be finite and within [0.5, 1.5]")
    qualification = str(raw.get("qualification"))
    if qualification not in {"qualified", "pending_calibration", "rejected"}:
        raise ValueError(f"invalid asset qualification: {qualification}")

    raw_body_mass_override = raw.get("body_mass_override_kg", {})
    if not isinstance(raw_body_mass_override, dict):
        raise ValueError("body_mass_override_kg must be an object")
    body_mass_override = {
        str(name): float(value) for name, value in raw_body_mass_override.items()
    }
    if any(not name for name in body_mass_override):
        raise ValueError("body_mass_override_kg body names must be non-empty")
    if any(
        not math.isfinite(value) or not 0.0001 <= value <= 20.0
        for value in body_mass_override.values()
    ):
        raise ValueError(
            "every body_mass_override_kg value must be finite and within [0.0001, 20.0]"
        )

    result = AssetProfile(
        profile_id=str(raw["profile_id"]),
        isaac_task=str(raw["isaac_task"]),
        source_asset=str(raw["source_asset"]),
        qualification=qualification,
        contract_to_asset_joint={str(k): str(v) for k, v in mapping.items()},
        joint_axis_sign={str(k): int(v) for k, v in signs.items()},
        joint_effort_scale=effort_scale,
        body_mass_override_kg=body_mass_override,
        allowed_extra_joints=tuple(str(value) for value in raw.get("allowed_extra_joints", ())),
        physics_dt_s=float(raw.get("physics_dt_s", 0.005)),
        solver_position_iteration_count=int(
            raw.get("solver_position_iteration_count", 8)
        ),
        solver_velocity_iteration_count=int(
            raw.get("solver_velocity_iteration_count", 4)
        ),
        notes=tuple(str(value) for value in raw.get("notes", ())),
    )
    if result.profile_id != path.stem:
        raise ValueError(
            f"profile_id {result.profile_id!r} does not match filename {path.stem!r}"
        )
    if result.physics_dt_s != 0.005:
        raise ValueError("SONIC LowCmd asset profiles must use 0.005 s physics dt")
    if not 4 <= result.solver_position_iteration_count <= 8:
        raise ValueError("solver_position_iteration_count must be within [4, 8]")
    if not 1 <= result.solver_velocity_iteration_count <= 4:
        raise ValueError("solver_velocity_iteration_count must be within [1, 4]")
    return result
