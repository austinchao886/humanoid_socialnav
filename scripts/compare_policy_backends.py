#!/usr/bin/env python3
"""Compare logged SONIC policy inputs against ONNX Runtime decoder outputs."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import numpy as np
import onnxruntime as ort


def load_actions(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        action_start = header.index("act_0")
        return np.asarray(
            [[float(value) for value in row[action_start:action_start + 29]] for row in reader],
            dtype=np.float32,
        )


def main() -> int:
    if len(sys.argv) != 4:
        raise SystemExit("usage: compare_policy_backends.py MODEL POLICY_INPUT ACTION_CSV")
    model_path, input_path, action_path = map(Path, sys.argv[1:])
    inputs = np.genfromtxt(input_path, delimiter=",", dtype=np.float32)
    if inputs.ndim == 1:
        inputs = inputs[None, :]
    # The native logger terminates each row with a comma.
    inputs = inputs[:, ~np.isnan(inputs).all(axis=0)]
    actions = load_actions(action_path)

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_meta = session.get_inputs()[0]
    output_meta = session.get_outputs()[0]
    predicted = np.concatenate(
        [session.run([output_meta.name], {input_meta.name: row[None, :]})[0] for row in inputs],
        axis=0,
    )

    shifts = {}
    for shift in range(-3, 4):
        if shift >= 0:
            lhs = predicted[: min(len(predicted), len(actions) - shift)]
            rhs = actions[shift:shift + len(lhs)]
        else:
            rhs = actions[: min(len(actions), len(predicted) + shift)]
            lhs = predicted[-shift:-shift + len(rhs)]
        error = np.abs(lhs - rhs)
        shifts[str(shift)] = {
            "samples": int(len(lhs)),
            "mae": float(error.mean()),
            "max_abs": float(error.max()),
        }

    best_shift = min(shifts, key=lambda key: shifts[key]["mae"])
    shift = int(best_shift)
    if shift >= 0:
        lhs = predicted[: min(len(predicted), len(actions) - shift)]
        rhs = actions[shift:shift + len(lhs)]
    else:
        rhs = actions[: min(len(actions), len(predicted) + shift)]
        lhs = predicted[-shift:-shift + len(rhs)]
    per_joint = np.abs(lhs - rhs).max(axis=0)

    print(json.dumps({
        "model_input": {"name": input_meta.name, "shape": input_meta.shape},
        "model_output": {"name": output_meta.name, "shape": output_meta.shape},
        "logged_input_shape": list(inputs.shape),
        "logged_action_shape": list(actions.shape),
        "candidate_shifts_action_index_minus_input_index": shifts,
        "best_shift": shift,
        "best_shift_max_abs_by_joint": per_joint.tolist(),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
