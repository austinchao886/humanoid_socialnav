#!/usr/bin/env python3
"""Drive SONIC against the isolated official MuJoCo endpoint."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import time

import pexpect


RUNTIME = Path("/motion_exchange/.runtime")
STATUS_PATH = RUNTIME / "native_mujoco_status.json"
COMMAND_PATH = RUNTIME / "native_mujoco_command.json"
RESULT_PATH = Path(
    os.environ.get(
        "SONIC_DIAGNOSTIC_RESULT",
        "/motion_exchange/diagnostics/native_mujoco_release_result.json",
    )
)
TRACKER_LOG = Path(
    os.environ.get(
        "SONIC_DIAGNOSTIC_TRACKER_LOG",
        "/motion_exchange/diagnostics/native_mujoco_release_sonic.log",
    )
)
CSV_DIR = Path(
    os.environ.get(
        "SONIC_DIAGNOSTIC_CSV_DIR",
        "/motion_exchange/diagnostics/native_mujoco_release_csv_008",
    )
)
MOTION_ID = os.environ.get("SONIC_NATIVE_MOTION_ID", "squat_001__A359")
REFERENCE_DIR = Path("/reference") / MOTION_ID


def load_reference_root_heights() -> list[float]:
    lines = (REFERENCE_DIR / "body_pos.csv").read_text().splitlines()[1:]
    return [float(line.split(",")[2]) for line in lines if line.strip()]


def last_logged_reference_root_height(default: float) -> float:
    path = CSV_DIR / "target_motion.csv"
    try:
        lines = path.read_text().splitlines()
        if lines:
            return float(lines[-1].split(",")[2])
    except (FileNotFoundError, IndexError, ValueError):
        pass
    return default


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


def drain(child: pexpect.spawn, timeout_s: float = 0.1) -> None:
    child.expect([pexpect.EOF, pexpect.TIMEOUT], timeout=timeout_s)


def wait_ready(timeout_s: float = 60.0) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = read_json(STATUS_PATH)
        if status.get("state") == "READY" and time.time() - status.get("updated_epoch_s", 0) < 2:
            return status
        time.sleep(0.1)
    raise TimeoutError("official MuJoCo endpoint did not become READY")


def main() -> int:
    mode = os.environ.get("SONIC_NATIVE_MODE", "unsupported_quickstart")
    if mode not in {
        "supported_diagnostic",
        "unsupported_quickstart",
        "unsupported_contact_gate",
    }:
        raise ValueError(f"unsupported SONIC_NATIVE_MODE: {mode}")
    ready = wait_ready()
    reference_root_heights = load_reference_root_heights()
    frame_count = len(reference_root_heights)
    session_id = ready["session_id"]
    # These explicit files are opened before StateLogger creates --logs-dir.
    CSV_DIR.mkdir(parents=True, exist_ok=True)
    command = (
        "/sonic/gear_sonic_deploy/target/release/g1_deploy_onnx_ref lo "
        "/sonic/gear_sonic_deploy/policy/release/model_decoder.onnx /reference "
        "--obs-config /sonic/gear_sonic_deploy/policy/release/observation_config.yaml "
        "--encoder-file /sonic/gear_sonic_deploy/policy/release/model_encoder.onnx "
        "--planner-file /sonic/gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx "
        "--input-type keyboard --output-type all --zmq-host localhost --disable-crc-check "
        f"--enable-csv-logs --logs-dir {CSV_DIR} "
        f"--target-motion-logfile {CSV_DIR / 'target_motion.csv'} "
        f"--planner-motion-logfile {CSV_DIR / 'planner_motion.csv'} "
        f"--policy-input-logfile {CSV_DIR / 'policy_input.csv'}"
    )
    child = pexpect.spawn("/bin/bash", ["-lc", command], encoding="utf-8", timeout=180)
    result = {
        "schema_version": 1,
        "motion_id": MOTION_ID,
        "frames": frame_count,
        "reference_hz": 50,
        "test_mode": mode,
        "startup_sequence": "init_then_control_then_play",
        "unsupported_precontrol_hold_s": 0.0,
        "unsupported_preplay_hold_s": 0.0,
        "native_mujoco_session_id": session_id,
        "diagnostic_csv_dir": str(CSV_DIR),
        "dds_interface": "lo",
        "environment_type": "sim",
        "tracker_binary_variant": os.environ.get(
            "SONIC_TRACKER_BINARY_VARIANT", "host-mounted"
        ),
        "started_epoch_s": time.time(),
        "result": "FAILED",
        "reason": None,
    }
    TRACKER_LOG.parent.mkdir(parents=True, exist_ok=True)
    with TRACKER_LOG.open("w", buffering=1) as tracker_log:
        child.logfile_read = tracker_log
        try:
            child.expect_exact("Init Done", timeout=180)
            child.send("]")
            child.expect("transitioning to CONTROL state", timeout=15)
            status = read_json(STATUS_PATH)
            result["control_start_simulation_time_s"] = float(
                status.get("simulation_time_s", 0.0)
            )
            print("[native-driver] SONIC entered CONTROL", flush=True)

            control_state = (
                "SUPPORT_DIAGNOSTIC" if mode == "supported_diagnostic" else "RELEASE"
            )
            write_json(
                COMMAND_PATH,
                {
                    "session_id": session_id,
                    "state": control_state,
                    "expected_reference_root_height_m": reference_root_heights[0],
                },
            )
            expected_state = (
                "EXECUTING_SUPPORTED" if mode == "supported_diagnostic" else "EXECUTING"
            )
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                drain(child)
                status = read_json(STATUS_PATH)
                if status.get("state") == expected_state:
                    break
                if status.get("state") in {"UNSAFE", "FAILED"}:
                    raise RuntimeError(f"MuJoCo rejected control transition: {status}")
            else:
                raise TimeoutError(f"MuJoCo did not enter {expected_state}")

            if mode == "unsupported_contact_gate":
                stable_since = None
                deadline = time.monotonic() + 60
                while time.monotonic() < deadline:
                    drain(child)
                    status = read_json(STATUS_PATH)
                    if status.get("state") in {"UNSAFE", "FAILED"}:
                        raise RuntimeError(
                            f"MuJoCo terminated while waiting for ground contact: {status}"
                        )
                    velocity = status.get("root_linear_velocity_m_s", [999, 999, 999])
                    grounded = bool(
                        status.get("left_foot_contact")
                        and status.get("right_foot_contact")
                        and abs(float(velocity[2])) <= 0.15
                        and float(status.get("root_tilt_rad", 99)) <= 0.25
                        and 0.70 <= float(status.get("root_height_m", 0)) <= 0.90
                    )
                    if grounded:
                        if stable_since is None:
                            stable_since = float(status["simulation_time_s"])
                        if float(status["simulation_time_s"]) - stable_since >= 0.30:
                            break
                    else:
                        stable_since = None
                else:
                    raise TimeoutError(
                        f"unsupported robot did not reach the playback gate: {status}"
                    )
                result["playback_gate"] = {
                    "left_foot_contact": status["left_foot_contact"],
                    "right_foot_contact": status["right_foot_contact"],
                    "root_height_m": status["root_height_m"],
                    "root_linear_velocity_m_s": status["root_linear_velocity_m_s"],
                    "root_tilt_rad": status["root_tilt_rad"],
                    "stable_duration_s": float(status["simulation_time_s"]) - stable_since,
                }

            playback_start = float(status["simulation_time_s"])
            result["playback_start_simulation_time_s"] = playback_start
            child.send("T")
            print(f"[native-driver] playing {MOTION_ID} in {mode}", flush=True)
            completion_text = f"Motion index: 0 : {MOTION_ID} completed."
            deadline = time.monotonic() + max(180, frame_count / 50 * 15)
            motion_completed = False
            while time.monotonic() < deadline:
                match = child.expect_exact(
                    [completion_text, pexpect.EOF, pexpect.TIMEOUT], timeout=0.1
                )
                if match == 0:
                    motion_completed = True
                elif match == 1:
                    raise RuntimeError("SONIC exited before reference completion")
                status = read_json(STATUS_PATH)
                write_json(
                    COMMAND_PATH,
                    {
                        "session_id": session_id,
                        "state": control_state,
                        "expected_reference_root_height_m":
                            last_logged_reference_root_height(reference_root_heights[0]),
                    },
                )
                if status.get("state") in {"UNSAFE", "FAILED"}:
                    raise RuntimeError(f"MuJoCo terminated during playback: {status}")
                if motion_completed:
                    break
            else:
                raise TimeoutError(f"{MOTION_ID} did not complete in MuJoCo")

            result["motion_completed_simulation_time_s"] = float(
                status.get("simulation_time_s", 0.0)
            )
            # A printed "motion completed" only means the reference cursor
            # reached the last frame. Require the unsupported robot to recover
            # to a stable two-foot stance before declaring acceptance.
            stable_since = None
            deadline = time.monotonic() + 180
            while time.monotonic() < deadline:
                drain(child)
                expected_height = last_logged_reference_root_height(
                    reference_root_heights[0]
                )
                write_json(
                    COMMAND_PATH,
                    {
                        "session_id": session_id,
                        "state": control_state,
                        "expected_reference_root_height_m": expected_height,
                    },
                )
                status = read_json(STATUS_PATH)
                if status.get("state") in {"UNSAFE", "FAILED"}:
                    raise RuntimeError(
                        f"MuJoCo terminated during post-motion recovery: {status}"
                    )
                linear = status.get("root_linear_velocity_m_s", [999, 999, 999])
                angular = status.get("root_angular_velocity_rad_s", [999, 999, 999])
                stable = bool(
                    status.get("left_foot_contact")
                    and status.get("right_foot_contact")
                    and math.sqrt(sum(float(value) ** 2 for value in linear)) <= 0.20
                    and math.sqrt(sum(float(value) ** 2 for value in angular)) <= 1.00
                    and float(status.get("root_tilt_rad", 99)) <= 0.25
                    and abs(float(status.get("reference_root_height_error_m", 99)))
                        <= 0.15
                )
                if stable:
                    if stable_since is None:
                        stable_since = float(status["simulation_time_s"])
                    if float(status["simulation_time_s"]) - stable_since >= 0.50:
                        break
                else:
                    stable_since = None
            else:
                raise TimeoutError(
                    f"{MOTION_ID} did not recover to stable two-foot stance: {status}"
                )
            result["post_motion_stability_gate"] = {
                "left_foot_contact": status["left_foot_contact"],
                "right_foot_contact": status["right_foot_contact"],
                "root_height_m": status["root_height_m"],
                "root_linear_velocity_m_s": status["root_linear_velocity_m_s"],
                "root_angular_velocity_rad_s": status["root_angular_velocity_rad_s"],
                "root_tilt_rad": status["root_tilt_rad"],
                "reference_root_height_error_m":
                    status["reference_root_height_error_m"],
                "stable_duration_s": float(status["simulation_time_s"]) - stable_since,
            }

            # Freeze the accepted simulation state before stopping the tracker;
            # otherwise MuJoCo continues briefly with stale/no LowCmd while the
            # C++ process tears down and the final report looks like a fall.
            write_json(COMMAND_PATH, {"session_id": session_id, "state": "STOP"})
            expected_final = (
                "COMPLETED_DIAGNOSTIC"
                if mode == "supported_diagnostic"
                else "COMPLETED"
            )
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                status = read_json(STATUS_PATH)
                if status.get("state") == expected_final:
                    break
                time.sleep(0.1)
            if status.get("state") != expected_final:
                raise RuntimeError(f"MuJoCo did not complete cleanly: {status}")
            child.send("O")
            child.expect(pexpect.EOF, timeout=30)
            result["result"] = expected_final
            result["native_mujoco_status"] = status
            result["finished_epoch_s"] = time.time()
            write_json(RESULT_PATH, result)
            print(f"[native-driver] {expected_final}", flush=True)
            return 0
        except BaseException as exc:
            result["reason"] = str(exc)
            result["native_mujoco_status"] = read_json(STATUS_PATH)
            result["finished_epoch_s"] = time.time()
            write_json(RESULT_PATH, result)
            if child.isalive():
                child.send("O")
                try:
                    child.expect(pexpect.EOF, timeout=15)
                except (pexpect.TIMEOUT, pexpect.EOF):
                    child.terminate(force=True)
            write_json(COMMAND_PATH, {"session_id": session_id, "state": "STOP"})
            print(f"[native-driver] FAILED: {exc}", file=sys.stderr, flush=True)
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
