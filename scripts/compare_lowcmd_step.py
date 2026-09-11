#!/usr/bin/env python3
"""Compare two fixed-root Unitree LowCmd step-response logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--joint", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-relative-peak-velocity-error", type=float, default=0.10)
    parser.add_argument("--max-relative-rise-time-error", type=float, default=0.15)
    parser.add_argument("--max-relative-response-error", type=float, default=0.10)
    return parser.parse_args()


def _load(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]
    if len(rows) < 10:
        raise RuntimeError(f"LowCmd step log is too short: {path}")
    return rows


def _metrics(path: Path) -> dict:
    rows = _load(path)
    target = [row["q_des"] for row in rows]
    initial_target = target[0]
    target_delta = max(target, key=lambda value: abs(value - initial_target)) - initial_target
    if abs(target_delta) < 1.0e-6:
        raise RuntimeError(f"LowCmd target has no step: {path}")
    active = [
        index
        for index, value in enumerate(target)
        if abs(value - initial_target) >= 0.5 * abs(target_delta)
    ]
    if not active:
        raise RuntimeError(f"cannot locate LowCmd step interval: {path}")
    start, stop = active[0], active[-1]
    initial_window = rows[max(0, start - 10):start]
    if not initial_window:
        raise RuntimeError(f"LowCmd log has no pre-step samples: {path}")
    initial_q = sum(row["q"] for row in initial_window) / len(initial_window)
    response = [rows[index]["q"] - initial_q for index in range(start, stop + 1)]
    commanded_sign = 1.0 if target_delta > 0.0 else -1.0
    signed_response = [commanded_sign * value for value in response]
    peak = max(signed_response)
    threshold = 0.9 * abs(target_delta)
    rise_index = next(
        (index for index, value in enumerate(signed_response) if value >= threshold),
        None,
    )
    rise_time = None
    if rise_index is not None:
        rise_time = rows[start + rise_index]["elapsed_sim_s"] - rows[start]["elapsed_sim_s"]
    tail_count = max(5, len(response) // 5)
    steady = sum(signed_response[-tail_count:]) / tail_count
    return {
        "path": str(path),
        "samples": len(rows),
        "command_delta_rad": target_delta,
        "response_direction_matches": peak > 0.0,
        "peak_response_rad": peak,
        "overshoot_rad": max(0.0, peak - abs(target_delta)),
        "steady_response_rad": steady,
        "rise_time_90_s": rise_time,
        "max_abs_velocity_rad_s": max(abs(row["dq"]) for row in rows),
        "max_abs_torque_nm": max(abs(row["tau_est"]) for row in rows),
    }


def _relative_error(a: float, b: float) -> float:
    return abs(a - b) / max(abs(a), 1.0e-9)


def main() -> int:
    args = parse_args()
    baseline = _metrics(args.baseline)
    candidate = _metrics(args.candidate)
    velocity_error = _relative_error(
        baseline["max_abs_velocity_rad_s"], candidate["max_abs_velocity_rad_s"]
    )
    peak_response_error = _relative_error(
        baseline["peak_response_rad"], candidate["peak_response_rad"]
    )
    steady_response_error = _relative_error(
        baseline["steady_response_rad"], candidate["steady_response_rad"]
    )
    baseline_reached_90 = baseline["rise_time_90_s"] is not None
    candidate_reached_90 = candidate["rise_time_90_s"] is not None
    if not baseline_reached_90 or not candidate_reached_90:
        rise_error = None
    else:
        rise_error = _relative_error(
            baseline["rise_time_90_s"], candidate["rise_time_90_s"]
        )
    if baseline_reached_90 and candidate_reached_90:
        response_shape_pass = bool(
            rise_error is not None
            and rise_error <= args.max_relative_rise_time_error
        )
        rise_time_mode = "both_reached"
    elif not baseline_reached_90 and not candidate_reached_90:
        # A gravity-loaded joint may settle below 90% even in the authoritative
        # baseline.  In that case the missing rise time is not itself a
        # candidate failure: require both peak and steady displacement parity
        # instead.  If only one asset reaches 90%, the response shapes differ
        # and the comparison still fails.
        response_shape_pass = bool(
            peak_response_error <= args.max_relative_response_error
            and steady_response_error <= args.max_relative_response_error
        )
        rise_time_mode = "both_unreached"
    else:
        response_shape_pass = False
        rise_time_mode = "reachability_mismatch"
    direction_pass = bool(candidate["response_direction_matches"])
    parity_pass = bool(
        direction_pass
        and velocity_error <= args.max_relative_peak_velocity_error
        and response_shape_pass
    )
    payload = {
        "schema_version": 1,
        "joint": args.joint,
        "root_support": "fixed",
        "diagnostic_only": True,
        "baseline": baseline,
        "candidate": candidate,
        "comparison": {
            "direction_pass": direction_pass,
            "dynamic_parity_pass": parity_pass,
            "relative_peak_velocity_error": velocity_error,
            "relative_peak_response_error": peak_response_error,
            "relative_steady_response_error": steady_response_error,
            "relative_rise_time_error": rise_error,
            "rise_time_mode": rise_time_mode,
            "thresholds": {
                "max_relative_peak_velocity_error": args.max_relative_peak_velocity_error,
                "max_relative_rise_time_error": args.max_relative_rise_time_error,
                "max_relative_response_error": args.max_relative_response_error,
            },
        },
    }
    numbers = (velocity_error, baseline["max_abs_torque_nm"], candidate["max_abs_torque_nm"])
    if not all(math.isfinite(value) for value in numbers):
        raise RuntimeError("non-finite LowCmd comparison metric")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, args.output)
    print(
        f"joint={args.joint} direction_pass={direction_pass} "
        f"dynamic_parity_pass={parity_pass} velocity_error={velocity_error:.3%}"
    )
    return 0 if direction_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
