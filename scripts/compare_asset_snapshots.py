#!/usr/bin/env python3
"""Compare an unqualified G1 asset snapshot with the SONIC baseline."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pose-tolerance-rad", type=float, default=0.02)
    parser.add_argument("--relative-tolerance", type=float, default=0.05)
    return parser.parse_args()


def _relative_error(a: float, b: float) -> float:
    scale = max(abs(a), abs(b), 1.0e-9)
    return abs(a - b) / scale


def _joint_values(snapshot: dict, field: str) -> dict[str, Any]:
    return dict(zip(snapshot["joint_names"], snapshot[field], strict=True))


def _body_values(snapshot: dict, field: str) -> dict[str, Any]:
    return dict(zip(snapshot["body_names"], snapshot[field], strict=True))


def _changed_scalars(
    names: list[str], baseline: dict[str, float], candidate: dict[str, float], tolerance: float
) -> list[dict[str, float | str]]:
    changed = []
    for name in names:
        a = float(baseline[name])
        b = float(candidate[name])
        error = _relative_error(a, b)
        if error > tolerance:
            changed.append(
                {"name": name, "baseline": a, "candidate": b, "relative_error": error}
            )
    return changed


def _vector_norm(value: Any) -> float:
    if isinstance(value, list):
        return math.sqrt(sum(_vector_norm(item) ** 2 for item in value))
    return abs(float(value))


def _vector_difference(a: Any, b: Any) -> float:
    if isinstance(a, list) and isinstance(b, list):
        return math.sqrt(
            sum(_vector_difference(x, y) ** 2 for x, y in zip(a, b, strict=True))
        )
    return abs(float(a) - float(b))


def main() -> int:
    args = parse_args()
    baseline = json.loads(args.baseline.read_text())
    candidate = json.loads(args.candidate.read_text())
    baseline_joints = list(baseline["joint_names"])
    candidate_joints = list(candidate["joint_names"])
    missing_joints = [name for name in baseline_joints if name not in candidate_joints]
    extra_joints = [name for name in candidate_joints if name not in baseline_joints]
    shared_joints = [name for name in baseline_joints if name in candidate_joints]

    baseline_default = _joint_values(baseline, "default_joint_pos")
    candidate_default = _joint_values(candidate, "default_joint_pos")
    pose_differences = []
    for name in shared_joints:
        delta = float(candidate_default[name]) - float(baseline_default[name])
        if abs(delta) > args.pose_tolerance_rad:
            pose_differences.append(
                {
                    "joint": name,
                    "baseline_rad": float(baseline_default[name]),
                    "candidate_rad": float(candidate_default[name]),
                    "delta_rad": delta,
                }
            )

    scalar_fields = (
        "joint_effort_limits",
        "joint_velocity_limits",
        "joint_damping",
        "joint_armature",
        "joint_friction",
    )
    joint_differences = {}
    for field in scalar_fields:
        a = _joint_values(baseline, field)
        b = _joint_values(candidate, field)
        joint_differences[field] = _changed_scalars(
            shared_joints, a, b, args.relative_tolerance
        )

    position_limit_differences = []
    a_limits = _joint_values(baseline, "joint_position_limits")
    b_limits = _joint_values(candidate, "joint_position_limits")
    for name in shared_joints:
        baseline_range = [float(value) for value in a_limits[name]]
        candidate_range = [float(value) for value in b_limits[name]]
        maximum_delta = max(
            abs(a - b) for a, b in zip(baseline_range, candidate_range, strict=True)
        )
        if maximum_delta > args.pose_tolerance_rad:
            position_limit_differences.append(
                {
                    "joint": name,
                    "baseline_rad": baseline_range,
                    "candidate_rad": candidate_range,
                    "max_delta_rad": maximum_delta,
                }
            )

    baseline_bodies = list(baseline["body_names"])
    candidate_bodies = list(candidate["body_names"])
    shared_bodies = [name for name in baseline_bodies if name in candidate_bodies]
    baseline_mass = _body_values(baseline, "body_mass")
    candidate_mass = _body_values(candidate, "body_mass")
    mass_differences = _changed_scalars(
        shared_bodies, baseline_mass, candidate_mass, args.relative_tolerance
    )
    baseline_inertia = _body_values(baseline, "body_inertia")
    candidate_inertia = _body_values(candidate, "body_inertia")
    inertia_differences = []
    for name in shared_bodies:
        a = baseline_inertia[name]
        b = candidate_inertia[name]
        error = _vector_difference(a, b) / max(
            _vector_norm(a), _vector_norm(b), 1.0e-12
        )
        if error > args.relative_tolerance:
            inertia_differences.append(
                {
                    "name": name,
                    "baseline": a,
                    "candidate": b,
                    "relative_frobenius_error": error,
                }
            )
    baseline_total_mass = sum(float(value) for value in baseline["body_mass"])
    candidate_total_mass = sum(float(value) for value in candidate["body_mass"])

    blockers = []
    if missing_joints:
        blockers.append("missing SONIC motor joints")
    if pose_differences:
        blockers.append("default body pose differs from the trained SONIC pose")
    if joint_differences["joint_armature"]:
        blockers.append("joint armature differs from the SONIC dynamics contract")
    if position_limit_differences:
        blockers.append("joint position limits differ")
    if joint_differences["joint_effort_limits"]:
        blockers.append("joint effort limits differ")
    if joint_differences["joint_velocity_limits"]:
        blockers.append("joint velocity limits differ")
    if mass_differences or _relative_error(baseline_total_mass, candidate_total_mass) > args.relative_tolerance:
        blockers.append("body mass distribution differs")
    if inertia_differences:
        blockers.append("body inertia distribution differs")
    simulation_contract_differences = {}
    for section, baseline_values in baseline.get("simulation_contract", {}).items():
        candidate_values = candidate.get("simulation_contract", {}).get(section, {})
        differences = {
            key: {"baseline": value, "candidate": candidate_values.get(key)}
            for key, value in baseline_values.items()
            if candidate_values.get(key) != value
        }
        if differences:
            simulation_contract_differences[section] = differences
    if simulation_contract_differences:
        blockers.append("simulation/contact properties differ")
    # A static snapshot cannot prove joint-axis direction.  Qualification must
    # retain this blocker until the fixed-root signed response probe is compared.
    blockers.append("joint axis signs require fixed-root LowCmd response verification")

    payload = {
        "schema_version": 1,
        "baseline_profile_id": baseline["profile_id"],
        "candidate_profile_id": candidate["profile_id"],
        "qualification": "pending_calibration" if blockers else "qualified",
        "blocking_reasons": blockers,
        "joint_topology": {
            "baseline_count": len(baseline_joints),
            "candidate_count": len(candidate_joints),
            "missing": missing_joints,
            "extra": extra_joints,
        },
        "body_topology": {
            "baseline_count": len(baseline_bodies),
            "candidate_count": len(candidate_bodies),
            "missing": [name for name in baseline_bodies if name not in candidate_bodies],
            "extra": [name for name in candidate_bodies if name not in baseline_bodies],
        },
        "default_pose_differences": pose_differences,
        "joint_property_differences": joint_differences,
        "joint_position_limit_differences": position_limit_differences,
        "mass": {
            "baseline_total_kg": baseline_total_mass,
            "candidate_total_kg": candidate_total_mass,
            "relative_total_error": _relative_error(
                baseline_total_mass, candidate_total_mass
            ),
            "shared_body_differences": mass_differences,
            "shared_body_inertia_differences": inertia_differences,
        },
        "simulation_contract_differences": simulation_contract_differences,
        "axis_response_probe": {
            "status": "not_run",
            "required": True,
        },
    }
    if not all(
        math.isfinite(value)
        for value in (baseline_total_mass, candidate_total_mass)
    ):
        raise RuntimeError("non-finite mass in asset snapshots")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, args.output)
    print(
        f"candidate={candidate['profile_id']} qualification={payload['qualification']} "
        f"blockers={len(blockers)} output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
