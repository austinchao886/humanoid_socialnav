#!/usr/bin/env python3
"""Summarize SONIC CSV logs without changing acceptance thresholds."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import numpy as np


HARDWARE_JOINTS = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll", "right_hip_pitch",
    "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch",
    "right_ankle_roll", "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]


def prefixed_csv(path: Path, prefix: str) -> np.ndarray:
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        columns = [index for index, name in enumerate(header) if name.startswith(prefix)]
        if not columns:
            raise ValueError(f"no {prefix!r} columns in {path}")
        return np.asarray(
            [[float(row[index]) for index in columns] for row in reader],
            dtype=np.float64,
        )


def numeric_csv(path: Path) -> np.ndarray:
    values = np.genfromtxt(path, delimiter=",")
    if values.ndim == 1:
        values = values[None, :]
    return values[:, ~np.isnan(values).all(axis=0)]


def aligned_error(actual: np.ndarray, target: np.ndarray, shift: int) -> np.ndarray:
    if shift >= 0:
        count = min(len(actual), len(target) - shift)
        return actual[:count] - target[shift:shift + count]
    count = min(len(target), len(actual) + shift)
    return actual[-shift:-shift + count] - target[:count]


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_sonic_tracking.py LOG_DIR")
    log_dir = Path(sys.argv[1])
    actual_q = prefixed_csv(log_dir / "q.csv", "q_")[:, :29]
    actual_dq = prefixed_csv(log_dir / "dq.csv", "dq_")[:, :29]
    torque = prefixed_csv(log_dir / "motor_torque.csv", "tau_")[:, :29]
    playing = prefixed_csv(log_dir / "motion_playing.csv", "playing_")[:, 0] > 0.5
    target = numeric_csv(log_dir / "target_motion.csv")[:, 7:36]

    candidates = {}
    for shift in range(-3, 4):
        error = aligned_error(actual_q, target, shift)
        candidates[shift] = float(np.sqrt(np.mean(error * error)))
    best_shift = min(candidates, key=candidates.get)
    error = aligned_error(actual_q, target, best_shift)
    count = len(error)
    play_mask = playing[:count]
    if best_shift < 0:
        play_mask = playing[-best_shift:-best_shift + count]
    play_error = error[play_mask] if play_mask.any() else error
    joint_rmse = np.sqrt(np.mean(play_error * play_error, axis=0))
    joint_max = np.max(np.abs(play_error), axis=0)
    worst_rmse = int(np.argmax(joint_rmse))
    worst_max = np.unravel_index(np.argmax(np.abs(play_error)), play_error.shape)

    dq_abs = np.abs(actual_dq)
    torque_abs = np.abs(torque)
    dq_index = np.unravel_index(np.argmax(dq_abs), dq_abs.shape)
    torque_index = np.unravel_index(np.argmax(torque_abs), torque_abs.shape)
    report = {
        "schema_version": 1,
        "log_dir": str(log_dir),
        "samples": {
            "q": int(len(actual_q)),
            "target": int(len(target)),
            "playing": int(playing.sum()),
        },
        "best_target_shift_frames": int(best_shift),
        "candidate_rmse_rad": {str(key): value for key, value in candidates.items()},
        "playing_tracking": {
            "rmse_all_rad": float(np.sqrt(np.mean(play_error * play_error))),
            "mae_all_rad": float(np.mean(np.abs(play_error))),
            "max_abs_rad": float(np.abs(play_error[worst_max])),
            "max_abs_joint": HARDWARE_JOINTS[int(worst_max[1])],
            "max_abs_playing_sample": int(worst_max[0]),
            "worst_joint_rmse_rad": float(joint_rmse[worst_rmse]),
            "worst_joint_rmse_name": HARDWARE_JOINTS[worst_rmse],
            "per_joint_rmse_rad": dict(zip(HARDWARE_JOINTS, joint_rmse.tolist())),
            "per_joint_max_abs_rad": dict(zip(HARDWARE_JOINTS, joint_max.tolist())),
        },
        "observed_max_abs_velocity_rad_s": {
            "value": float(dq_abs[dq_index]),
            "joint": HARDWARE_JOINTS[int(dq_index[1])],
            "sample": int(dq_index[0]),
        },
        "observed_max_abs_torque_nm": {
            "value": float(torque_abs[torque_index]),
            "joint": HARDWARE_JOINTS[int(torque_index[1])],
            "sample": int(torque_index[0]),
        },
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
