#!/usr/bin/env python3
"""Isolated official MuJoCo endpoint for SONIC unsupported acceptance tests."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time
import uuid

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.utils.mujoco_sim.base_sim import BaseSimulator
import mujoco
import numpy as np


RUNTIME = Path("/motion_exchange/.runtime")
COMMAND_PATH = RUNTIME / "native_mujoco_command.json"
STATUS_PATH = RUNTIME / "native_mujoco_status.json"


def read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def main() -> int:
    loop_config = SimLoopConfig(
        # Keep the official sim semantics.  A non-loopback interface is
        # classified as ENV_TYPE=real and applies real-robot damping offsets
        # (including making the sim waist-pitch Kd negative).
        interface="sim",
        enable_onscreen=False,
        enable_offscreen=False,
        verbose=False,
    )
    config = loop_config.load_wbc_yaml()
    config["DOMAIN_ID"] = 73
    config["PRINT_SCENE_INFORMATION"] = False
    config["ENABLE_ELASTIC_BAND"] = True
    simulator = BaseSimulator(config=config, onscreen=False, offscreen=False)
    simulator.start_as_thread()

    session_id = f"native-mujoco-{uuid.uuid4().hex}"
    released = False
    supported_diagnostic = False
    release_time = None
    command_seen_time = None
    max_velocity_ratio = 0.0
    max_velocity_joint = None
    joint_names = [
        mujoco.mj_id2name(
            simulator.sim_env.mj_model,
            mujoco.mjtObj.mjOBJ_JOINT,
            int(index),
        )
        for index in simulator.sim_env.body_joint_index
    ]
    velocity_by_name = {
        "left_hip_pitch_joint": 20.0,
        "left_hip_roll_joint": 20.0,
        "left_hip_yaw_joint": 32.0,
        "left_knee_joint": 20.0,
        "left_ankle_pitch_joint": 37.0,
        "left_ankle_roll_joint": 37.0,
        "right_hip_pitch_joint": 20.0,
        "right_hip_roll_joint": 20.0,
        "right_hip_yaw_joint": 32.0,
        "right_knee_joint": 20.0,
        "right_ankle_pitch_joint": 37.0,
        "right_ankle_roll_joint": 37.0,
        "waist_yaw_joint": 32.0,
        "waist_roll_joint": 37.0,
        "waist_pitch_joint": 37.0,
        "left_shoulder_pitch_joint": 37.0,
        "left_shoulder_roll_joint": 37.0,
        "left_shoulder_yaw_joint": 37.0,
        "left_elbow_joint": 37.0,
        "left_wrist_roll_joint": 37.0,
        "left_wrist_pitch_joint": 22.0,
        "left_wrist_yaw_joint": 22.0,
        "right_shoulder_pitch_joint": 37.0,
        "right_shoulder_roll_joint": 37.0,
        "right_shoulder_yaw_joint": 37.0,
        "right_elbow_joint": 37.0,
        "right_wrist_roll_joint": 37.0,
        "right_wrist_pitch_joint": 22.0,
        "right_wrist_yaw_joint": 22.0,
    }
    velocity_limits = np.asarray(
        [velocity_by_name[name] for name in joint_names], dtype=float
    )
    floor_geom_id = mujoco.mj_name2id(
        simulator.sim_env.mj_model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )
    foot_body_ids = {
        side: mujoco.mj_name2id(
            simulator.sim_env.mj_model,
            mujoco.mjtObj.mjOBJ_BODY,
            f"{side}_ankle_roll_link",
        )
        for side in ("left", "right")
    }
    result = "READY"
    reason = None
    expected_reference_root_height = None
    max_reference_root_height_error = 0.0
    next_status_wall = 0.0

    try:
        while simulator._running:
            now = time.monotonic()
            data = simulator.sim_env.mj_data
            sim_time = float(data.time)
            root_height = float(data.qpos[2])
            root_linear_velocity = np.asarray(data.qvel[:3], dtype=float)
            root_angular_velocity = np.asarray(data.qvel[3:6], dtype=float)
            root_quat = np.asarray(data.qpos[3:7], dtype=float)
            upright_cosine = float(
                1.0 - 2.0 * (root_quat[1] ** 2 + root_quat[2] ** 2)
            )
            root_tilt_rad = float(math.acos(np.clip(upright_cosine, -1.0, 1.0)))
            foot_contact = {"left": False, "right": False}
            for contact_index in range(int(data.ncon)):
                contact = data.contact[contact_index]
                geom1 = int(contact.geom1)
                geom2 = int(contact.geom2)
                if floor_geom_id not in (geom1, geom2):
                    continue
                other_geom = geom2 if geom1 == floor_geom_id else geom1
                other_body = int(simulator.sim_env.mj_model.geom_bodyid[other_geom])
                for side, body_id in foot_body_ids.items():
                    if other_body == body_id:
                        foot_contact[side] = True
            body_indices = simulator.sim_env.body_joint_index
            body_dq = np.asarray(
                data.qvel[body_indices + simulator.sim_env.qvel_offset - 1], dtype=float
            )
            ratio = np.abs(body_dq) / velocity_limits
            ratio_index = int(np.argmax(ratio))
            if float(ratio[ratio_index]) > max_velocity_ratio:
                max_velocity_ratio = float(ratio[ratio_index])
                max_velocity_joint = joint_names[ratio_index]

            low_cmd = simulator.unitree_bridge.low_cmd
            command_seen = bool(
                low_cmd is not None
                and any(
                    abs(float(low_cmd.motor_cmd[index].kp)) > 1.0e-6
                    for index in range(simulator.unitree_bridge.num_body_motor)
                )
            )
            if command_seen and command_seen_time is None:
                command_seen_time = sim_time

            control = read_json(COMMAND_PATH)
            control_reference_height = control.get("expected_reference_root_height_m")
            if control_reference_height is not None:
                expected_reference_root_height = float(control_reference_height)
            reference_root_height_error = (
                expected_reference_root_height - root_height
                if expected_reference_root_height is not None
                else None
            )
            if reference_root_height_error is not None:
                max_reference_root_height_error = max(
                    max_reference_root_height_error, reference_root_height_error
                )
            if (
                control.get("session_id") == session_id
                and control.get("state") == "RELEASE"
                and not released
            ):
                simulator.sim_env.elastic_band.enable = False
                released = True
                release_time = sim_time
                result = "EXECUTING"
                print(f"[native-mujoco] elastic band released at t={sim_time:.3f}s", flush=True)
            if (
                control.get("session_id") == session_id
                and control.get("state") == "SUPPORT_DIAGNOSTIC"
                and not released
                and not supported_diagnostic
            ):
                supported_diagnostic = True
                result = "EXECUTING_SUPPORTED"
                print(
                    f"[native-mujoco] fixed-root diagnostic started at t={sim_time:.3f}s",
                    flush=True,
                )
            if control.get("session_id") == session_id and control.get("state") == "STOP":
                if released:
                    result = "COMPLETED"
                elif supported_diagnostic:
                    result = "COMPLETED_DIAGNOSTIC"
                else:
                    result = "ABORTED"
                break

            finite = bool(
                np.isfinite(data.qpos).all()
                and np.isfinite(data.qvel).all()
                and math.isfinite(root_height)
            )
            if not finite:
                result = "UNSAFE"
                reason = "non-finite MuJoCo state"
                break
            if (
                released
                and reference_root_height_error is not None
                and reference_root_height_error > 0.25
            ):
                result = "UNSAFE"
                reason = (
                    "reference-relative fall threshold crossed: "
                    f"reference_root_height={expected_reference_root_height:.4f}m, "
                    f"actual_root_height={root_height:.4f}m, "
                    f"error={reference_root_height_error:.4f}m"
                )
                break
            if released and root_tilt_rad > 0.80:
                result = "UNSAFE"
                reason = f"root tilt threshold crossed: tilt={root_tilt_rad:.4f}rad"
                break
            if (released or supported_diagnostic) and float(ratio[ratio_index]) > 1.05:
                result = "UNSAFE"
                reason = (
                    "actuator velocity limit crossed: "
                    f"joint={joint_names[ratio_index]}, dq={abs(body_dq[ratio_index]):.4f}rad/s, "
                    f"limit={velocity_limits[ratio_index]:.4f}rad/s"
                )
                break

            if now >= next_status_wall:
                write_json(
                    STATUS_PATH,
                    {
                        "schema_version": 1,
                        "session_id": session_id,
                        "state": result,
                        "simulation_time_s": sim_time,
                        "root_height_m": root_height,
                        "root_linear_velocity_m_s": root_linear_velocity.tolist(),
                        "root_angular_velocity_rad_s": root_angular_velocity.tolist(),
                        "root_tilt_rad": root_tilt_rad,
                        "left_foot_contact": foot_contact["left"],
                        "right_foot_contact": foot_contact["right"],
                        "expected_reference_root_height_m": expected_reference_root_height,
                        "reference_root_height_error_m": reference_root_height_error,
                        "max_reference_root_height_error_m": max_reference_root_height_error,
                        "command_seen": command_seen,
                        "command_seen_simulation_time_s": command_seen_time,
                        "support_released": released,
                        "supported_diagnostic": supported_diagnostic,
                        "release_simulation_time_s": release_time,
                        "max_velocity_ratio": max_velocity_ratio,
                        "max_velocity_joint": max_velocity_joint,
                        "updated_epoch_s": time.time(),
                    },
                )
                next_status_wall = now + 0.1
            time.sleep(0.01)
    except BaseException as exc:
        result = "FAILED"
        reason = str(exc)
        raise
    finally:
        simulator.close()
        if simulator.sim_thread is not None:
            simulator.sim_thread.join(timeout=5.0)
        final_data = simulator.sim_env.mj_data
        write_json(
            STATUS_PATH,
            {
                "schema_version": 1,
                "session_id": session_id,
                "state": result,
                "reason": reason,
                "simulation_time_s": float(final_data.time),
                "root_height_m": float(final_data.qpos[2]),
                "root_linear_velocity_m_s": np.asarray(
                    final_data.qvel[:3], dtype=float
                ).tolist(),
                "root_angular_velocity_rad_s": np.asarray(
                    final_data.qvel[3:6], dtype=float
                ).tolist(),
                "root_tilt_rad": root_tilt_rad,
                "left_foot_contact": foot_contact["left"],
                "right_foot_contact": foot_contact["right"],
                "expected_reference_root_height_m": expected_reference_root_height,
                "reference_root_height_error_m": reference_root_height_error,
                "max_reference_root_height_error_m": max_reference_root_height_error,
                "command_seen": command_seen,
                "command_seen_simulation_time_s": command_seen_time,
                "support_released": released,
                "supported_diagnostic": supported_diagnostic,
                "release_simulation_time_s": release_time,
                "max_velocity_ratio": max_velocity_ratio,
                "max_velocity_joint": max_velocity_joint,
                "updated_epoch_s": time.time(),
            },
        )
        print(f"[native-mujoco] result={result} reason={reason}", flush=True)
    return 0 if result in {"COMPLETED", "COMPLETED_DIAGNOSTIC"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
