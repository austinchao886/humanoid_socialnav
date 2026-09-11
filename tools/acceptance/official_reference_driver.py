#!/usr/bin/env python3
"""Run one pinned SONIC reference against the isolated Isaac simulator endpoint.

This is a staged acceptance driver, not a general motion execution endpoint.  It
requires an already READY Isaac session, keeps the simulator-side bootstrap
support in charge of release safety, and only starts reference playback after
Isaac reports that support has actually been released.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

import pexpect


TERMINAL_STATES = {
    "ABORTED",
    "COMPLETED",
    "FAILED",
    "STABLE",
    "UNSAFE",
}


def reference_root_heights(path: Path) -> list[float]:
    lines = path.read_text().splitlines()[1:]
    values = [float(line.split(",")[2]) for line in lines if line.strip()]
    if not values:
        raise RuntimeError(f"reference has no root-height samples: {path}")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion-id", required=True)
    parser.add_argument("--selector-dir", type=Path, required=True)
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--frames", type=int, required=True)
    parser.add_argument("--reference-hz", type=float, default=50.0)
    parser.add_argument(
        "--isaac-reference-root",
        type=Path,
        default=Path(
            "/socialnav_humanoid_ws/motion_pipeline/vendor/"
            "GR00T-WholeBodyControl/gear_sonic_deploy/reference/example"
        ),
        help="Pinned official reference root as mounted inside the Isaac container",
    )
    parser.add_argument(
        "--interface",
        default=os.getenv("SONIC_INTERFACE", "wlp69s0"),
        help="DDS interface used by the low-level SONIC process",
    )
    parser.add_argument("--post-hold-s", type=float, default=2.0)
    parser.add_argument("--status-path", type=Path, default=Path("/motion_exchange/.runtime/isaac_status.json"))
    parser.add_argument("--request-path", type=Path, default=Path("/motion_exchange/.runtime/request.json"))
    parser.add_argument("--tracker-log", type=Path, required=True)
    parser.add_argument("--result-path", type=Path, required=True)
    parser.add_argument(
        "--csv-dir",
        type=Path,
        help="Optional SONIC split-CSV diagnostic directory",
    )
    parser.add_argument("--ready-max-age-s", type=float, default=5.0)
    parser.add_argument("--init-timeout-s", type=float, default=180.0)
    parser.add_argument("--release-timeout-s", type=float, default=420.0)
    parser.add_argument("--playback-timeout-s", type=float, default=900.0)
    return parser.parse_args()


def read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def drain(child: pexpect.spawn, timeout_s: float = 0.25) -> None:
    child.expect([pexpect.EOF, pexpect.TIMEOUT], timeout=timeout_s)


def wait_for_state(
    child: pexpect.spawn,
    status_path: Path,
    wanted: set[str],
    timeout_s: float,
) -> dict:
    deadline = time.monotonic() + timeout_s
    last_state = None
    while time.monotonic() < deadline:
        drain(child)
        status = read_json(status_path)
        state = status.get("state")
        if state != last_state:
            print(f"[official-driver] Isaac state={state}", flush=True)
            last_state = state
        if state in wanted or state in TERMINAL_STATES:
            return status
        if not child.isalive():
            raise RuntimeError("SONIC tracker exited before Isaac reached the requested state")
    raise TimeoutError(
        f"timed out after {timeout_s:.1f}s waiting for Isaac state in {sorted(wanted)}"
    )


def main() -> int:
    args = parse_args()
    ready = read_json(args.status_path)
    if ready.get("state") != "READY":
        raise RuntimeError(f"Isaac endpoint is not READY: {ready.get('state')}")
    heartbeat_age = time.time() - float(ready.get("updated_epoch_s", 0.0))
    if heartbeat_age > args.ready_max_age_s:
        raise RuntimeError(f"Isaac READY heartbeat is stale: age={heartbeat_age:.2f}s")
    if not (args.selector_dir / args.motion_id).is_dir():
        raise RuntimeError(
            f"motion selector does not contain {args.motion_id}: {args.selector_dir}"
        )

    request = {
        "schema_version": 1,
        "state": "STARTING",
        "request_id": args.request_id,
        "motion_id": args.motion_id,
        "isaac_session_id": ready["session_id"],
        "approved_epoch_s": time.time(),
        "source": "pinned_sonic_official_reference",
        "reference_hz": args.reference_hz,
        "reference_root_heights_m": reference_root_heights(
            args.selector_dir / args.motion_id / "body_pos.csv"
        ),
        "reference_joint_pos_path": str(
            args.isaac_reference_root / args.motion_id / "joint_pos.csv"
        ),
    }
    atomic_write_json(args.request_path, request)

    command = [
        "target/release/g1_deploy_onnx_ref",
        args.interface,
        "policy/release/model_decoder.onnx",
        str(args.selector_dir),
        "--obs-config",
        "policy/release/observation_config.yaml",
        "--encoder-file",
        "policy/release/model_encoder.onnx",
        "--planner-file",
        "planner/target_vel/V2/planner_sonic.onnx",
        "--input-type",
        "keyboard",
        "--output-type",
        "all",
        "--zmq-host",
        "localhost",
    ]
    if args.csv_dir is not None:
        args.csv_dir.mkdir(parents=True, exist_ok=True)
        command.extend(["--enable-csv-logs", "--logs-dir", str(args.csv_dir)])
    args.tracker_log.parent.mkdir(parents=True, exist_ok=True)
    child = pexpect.spawn(
        command[0],
        command[1:],
        cwd="/sonic/gear_sonic_deploy",
        encoding="utf-8",
        codec_errors="replace",
        timeout=args.init_timeout_s,
    )
    result = {
        "schema_version": 1,
        "request_id": args.request_id,
        "motion_id": args.motion_id,
        "frames": args.frames,
        "reference_hz": args.reference_hz,
        "started_epoch_s": time.time(),
        "isaac_session_id": ready["session_id"],
        "tracker_log": str(args.tracker_log),
        "result": "FAILED",
        "reason": None,
    }

    with args.tracker_log.open("w", buffering=1) as tracker_log:
        child.logfile_read = tracker_log
        try:
            child.expect_exact("Init Done", timeout=args.init_timeout_s)
            print("[official-driver] SONIC initialized; entering CONTROL", flush=True)
            child.send("]")
            child.expect("transitioning to CONTROL state", timeout=10.0)
            request["state"] = "SETTLING"
            request["control_epoch_s"] = time.time()
            atomic_write_json(args.request_path, request)

            status = wait_for_state(
                child,
                args.status_path,
                {"EXECUTING"},
                args.release_timeout_s,
            )
            if status.get("state") != "EXECUTING":
                raise RuntimeError(
                    f"Isaac rejected support release: {status.get('state')}: {status.get('reason')}"
                )

            playback_start_sim_s = float(status.get("simulation_time_s", 0.0))
            required_sim_s = args.frames / args.reference_hz + args.post_hold_s
            result["playback_start_simulation_time_s"] = playback_start_sim_s
            result["required_playback_simulation_time_s"] = required_sim_s
            print(
                "[official-driver] support released; starting reference "
                f"({args.frames} frames at {args.reference_hz:g} Hz)",
                flush=True,
            )
            child.send("T")
            request["state"] = "PLAYING"
            request["playback_epoch_s"] = time.time()
            atomic_write_json(args.request_path, request)

            deadline = time.monotonic() + args.playback_timeout_s
            last_reported_second = -1
            while time.monotonic() < deadline:
                drain(child)
                status = read_json(args.status_path)
                state = status.get("state")
                if state in TERMINAL_STATES:
                    raise RuntimeError(
                        f"Isaac terminated during playback: {state}: {status.get('reason')}"
                    )
                simulation_time_s = float(status.get("simulation_time_s", 0.0))
                elapsed_sim_s = simulation_time_s - playback_start_sim_s
                elapsed_second = int(max(0.0, elapsed_sim_s))
                if elapsed_second != last_reported_second:
                    print(
                        f"[official-driver] playback simulation time "
                        f"{elapsed_sim_s:.2f}/{required_sim_s:.2f}s",
                        flush=True,
                    )
                    last_reported_second = elapsed_second
                if elapsed_sim_s >= required_sim_s:
                    break
                if not child.isalive():
                    raise RuntimeError("SONIC tracker exited during reference playback")
            else:
                raise TimeoutError("reference playback did not finish before wall-time timeout")

            print("[official-driver] reference window complete; stopping LowCmd stream", flush=True)
            request["state"] = "STOPPING"
            request["stopping_epoch_s"] = time.time()
            atomic_write_json(args.request_path, request)
            child.send("O")
            child.expect(pexpect.EOF, timeout=30.0)
            final_status = wait_for_state(
                child,
                args.status_path,
                {"COMPLETED"},
                60.0,
            )
            result["isaac_status"] = final_status
            if final_status.get("state") != "COMPLETED":
                raise RuntimeError(
                    f"Isaac did not complete cleanly: {final_status.get('state')}: "
                    f"{final_status.get('reason')}"
                )
            result["result"] = "COMPLETED"
            result["finished_epoch_s"] = time.time()
            atomic_write_json(args.result_path, result)
            print(f"[official-driver] COMPLETED: {args.result_path}", flush=True)
            return 0
        except BaseException as exc:
            result["reason"] = str(exc)
            result["finished_epoch_s"] = time.time()
            result["isaac_status"] = read_json(args.status_path)
            atomic_write_json(args.result_path, result)
            if child.isalive():
                child.send("O")
                try:
                    child.expect(pexpect.EOF, timeout=15.0)
                except (pexpect.TIMEOUT, pexpect.EOF):
                    child.terminate(force=True)
            print(f"[official-driver] FAILED: {exc}", file=sys.stderr, flush=True)
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
