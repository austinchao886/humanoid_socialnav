#!/usr/bin/env python3
"""Reconstruct G1-mode encoder inputs and compare ONNX CPU tokens to SONIC logs."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import numpy as np
import onnxruntime as ort


ISAACLAB_TO_MUJOCO = np.asarray(
    [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
     11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28],
    dtype=int,
)


def csv_values(path: Path, prefix: str) -> np.ndarray:
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        start = next(i for i, name in enumerate(header) if name.startswith(prefix))
        width = sum(name.startswith(prefix) for name in header)
        return np.asarray(
            [[float(value) for value in row[start:start + width]] for row in reader],
            dtype=np.float64,
        )


def plain_csv(path: Path, skip_header: bool) -> np.ndarray:
    return np.genfromtxt(path, delimiter=",", skip_header=int(skip_header))[:, :]


def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.asarray([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def qconj(q: np.ndarray) -> np.ndarray:
    return q * np.asarray([1.0, -1.0, -1.0, -1.0])


def heading(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.asarray([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def rot6(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    matrix = np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return matrix[:, :2].reshape(-1)


def main() -> int:
    if len(sys.argv) != 4:
        raise SystemExit("usage: compare_encoder_backend.py MODEL REF_DIR LOG_DIR")
    model_path, ref_dir, log_dir = map(Path, sys.argv[1:])
    ref_q = plain_csv(ref_dir / "joint_pos.csv", True)[:, :29]
    ref_dq = plain_csv(ref_dir / "joint_vel.csv", True)[:, :29]
    ref_root_q = plain_csv(ref_dir / "body_quat.csv", True)[:, :4]
    target = plain_csv(log_dir / "target_motion.csv", False)
    target = target[:, ~np.isnan(target).all(axis=0)]
    target_hw = target[:, 7:36]
    expected_hw = ref_q[:, ISAACLAB_TO_MUJOCO]
    frame_ids = np.asarray([
        int(np.argmin(np.max(np.abs(expected_hw - row), axis=1))) for row in target_hw
    ])
    frame_errors = np.asarray([
        np.max(np.abs(expected_hw[frame] - row)) for frame, row in zip(frame_ids, target_hw)
    ])

    base_q = csv_values(log_dir / "base_quat.csv", "base_q")
    playing = csv_values(log_dir / "motion_playing.csv", "playing_")[:, 0] > 0.5
    logged_tokens = csv_values(log_dir / "token_state.csv", "token_")
    rows = min(len(frame_ids), len(base_q), len(playing), len(logged_tokens))
    init_base_heading = heading(base_q[0])
    init_ref_heading_inv = qconj(heading(ref_root_q[0]))
    apply_heading = qmul(init_base_heading, init_ref_heading_inv)

    encoder_inputs = np.zeros((rows, 1762), dtype=np.float32)
    for row_index in range(rows):
        current = int(frame_ids[row_index])
        encoder_inputs[row_index, 0] = 0.0
        for lookahead in range(10):
            frame = current + (lookahead * 5 if playing[row_index] else 0)
            frame = min(frame, len(ref_q) - 1)
            encoder_inputs[row_index, 4 + 29*lookahead:4 + 29*(lookahead+1)] = ref_q[frame]
            if playing[row_index]:
                encoder_inputs[row_index, 294 + 29*lookahead:294 + 29*(lookahead+1)] = ref_dq[frame]
            new_ref = qmul(apply_heading, ref_root_q[frame])
            relative = qmul(qconj(base_q[row_index]), new_ref)
            encoder_inputs[row_index, 601 + 6*lookahead:601 + 6*(lookahead+1)] = rot6(relative)

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_meta = session.get_inputs()[0]
    output_meta = session.get_outputs()[0]
    predicted = np.concatenate([
        session.run([output_meta.name], {input_meta.name: row[None, :]})[0]
        for row in encoder_inputs
    ], axis=0)

    shifts = {}
    for shift in range(-3, 4):
        if shift >= 0:
            lhs = predicted[: min(len(predicted), len(logged_tokens) - shift)]
            rhs = logged_tokens[shift:shift + len(lhs)]
        else:
            rhs = logged_tokens[: min(len(logged_tokens), len(predicted) + shift)]
            lhs = predicted[-shift:-shift + len(rhs)]
        error = np.abs(lhs - rhs)
        shifts[str(shift)] = {
            "samples": int(len(lhs)),
            "mae": float(error.mean()),
            "max_abs": float(error.max()),
        }

    print(json.dumps({
        "frames": frame_ids[:rows].tolist(),
        "max_target_frame_match_error_rad": float(frame_errors[:rows].max()),
        "reconstructed_input_shape": list(encoder_inputs.shape),
        "logged_token_shape": list(logged_tokens.shape),
        "candidate_shifts_token_index_minus_input_index": shifts,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
