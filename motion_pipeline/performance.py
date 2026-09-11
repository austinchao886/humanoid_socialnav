from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import statistics
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .latency import LATENCY_BUDGETS, evaluate_stage, read_timing


IMMUTABLE_ARTIFACT_FILES = (
    "manifest.json",
    "joint_pos.csv",
    "joint_vel.csv",
    "body_pos.csv",
    "body_quat.csv",
    "metadata.txt",
    "validation.json",
    "preview.mp4",
)

REFERENCE_TRAJECTORY_FILES = (
    "joint_pos.csv",
    "joint_vel.csv",
    "body_pos.csv",
    "body_quat.csv",
)

GENERATION_STAGES = {
    "dds_generate_to_received",
    "kimodo_generation",
    "reference_conversion_fk",
    "preview_rendering",
    "validation",
    "validation_to_ready",
}


def artifact_checksum(artifact: Path) -> str:
    """Hash only the immutable reference contract, never execution metadata."""

    digest = hashlib.sha256()
    for name in IMMUTABLE_ARTIFACT_FILES:
        path = artifact / name
        if not path.is_file():
            continue
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as handle:
            while chunk := handle.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def trajectory_checksum(artifact: Path) -> str:
    """Hash generated numeric reference files, excluding per-request metadata."""

    digest = hashlib.sha256()
    for name in REFERENCE_TRAJECTORY_FILES:
        path = artifact / name
        if not path.is_file():
            raise FileNotFoundError(f"reference trajectory file is missing: {path}")
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as handle:
            while chunk := handle.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def _stage_samples(timing: dict[str, Any], name: str) -> list[dict[str, Any]]:
    history = timing.get("stage_history", {}).get(name)
    if isinstance(history, list):
        samples = [sample for sample in history if isinstance(sample, dict)]
        if name == "offline_video_output":
            # Older supervisors recorded a near-zero finalization stage even
            # when video capture was disabled. A video performance sample is
            # real only when it names an emitted artifact.
            samples = [
                sample
                for sample in samples
                if sample.get("details", {}).get("video_path")
            ]
        return samples
    latest = timing.get("stages", {}).get(name)
    if not isinstance(latest, dict):
        return []
    if name == "offline_video_output" and not latest.get("details", {}).get(
        "video_path"
    ):
        return []
    return [latest]


def aggregate_stage(
    samples: list[dict[str, Any]], required_runs: int = 3
) -> dict[str, Any]:
    warm_samples = [
        sample
        for sample in samples
        if not sample.get("excluded_from_gate", False)
        and not sample.get("details", {}).get("cold_bootstrap", False)
    ]
    selected = warm_samples[-required_runs:]
    durations = [
        float(sample["duration_s"])
        for sample in selected
        if sample.get("duration_s") is not None
    ]
    realtime_factors = [
        float(sample["realtime_factor"])
        for sample in selected
        if sample.get("realtime_factor") is not None
    ]
    duration_median = statistics.median(durations) if durations else None
    realtime_median = (
        statistics.median(realtime_factors) if realtime_factors else None
    )
    return {
        "sample_count": len(warm_samples),
        "required_runs": required_runs,
        "selected_samples": selected,
        "median_duration_s": duration_median,
        "median_realtime_factor": realtime_median,
        "enough_runs": len(selected) >= required_runs,
    }


def _hardware_summary() -> dict[str, Any]:
    summary: dict[str, Any] = {
        "platform": platform.platform(),
        "cpu_count": os.cpu_count(),
    }
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=index,name,driver_version,memory.total",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        summary["gpus"] = [
            line.strip() for line in result.stdout.splitlines() if line.strip()
        ]
    except (FileNotFoundError, subprocess.SubprocessError):
        summary["gpus"] = []
    return summary


def _nested(payload: dict[str, Any], *path: str) -> Any:
    value: Any = payload
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _median_nested(
    reports: list[dict[str, Any]], path: tuple[str, ...]
) -> float | None:
    values = []
    for report in reports:
        value = _nested(report, *path)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return statistics.median(values) if values else None


def _load_execution_reports(
    exchange: Path, timing: dict[str, Any], required_runs: int
) -> tuple[list[dict[str, Any]], list[str]]:
    samples = _stage_samples(timing, "report_trace_finalization")[-required_runs:]
    reports = []
    paths = []
    exchange_resolved = exchange.resolve()
    for sample in samples:
        raw_path = sample.get("details", {}).get("report")
        if not raw_path:
            continue
        path = Path(raw_path)
        if path.is_absolute() and path.parts[:2] == ("/", "motion_exchange"):
            path = exchange / Path(*path.parts[2:])
        elif not path.is_absolute():
            path = exchange / path
        path = path.resolve()
        if path != exchange_resolved and exchange_resolved not in path.parents:
            continue
        try:
            payload = json.loads(path.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            continue
        if isinstance(payload, dict):
            reports.append(payload)
            paths.append(str(path))
    return reports, paths


def aggregate_execution_metrics(
    exchange: Path, timing: dict[str, Any], required_runs: int
) -> dict[str, Any]:
    reports, paths = _load_execution_reports(exchange, timing, required_runs)
    phase = {}
    for name in (
        "physics_step",
        "lowstate_bridge",
        "lowcmd_action",
        "critical_monitor",
        "loop",
    ):
        phase[name] = {
            percentile: _median_nested(
                reports, ("performance", "phase_latency", name, percentile)
            )
            for percentile in ("p50_ms", "p95_ms", "p99_ms")
        }
    cuda_allocated = [
        _nested(report, "performance", "resources", "cuda_max_memory_allocated_bytes")
        for report in reports
    ]
    cuda_reserved = [
        _nested(report, "performance", "resources", "cuda_max_memory_reserved_bytes")
        for report in reports
    ]
    cuda_devices = sorted(
        {
            str(value)
            for report in reports
            if (value := _nested(report, "performance", "resources", "cuda_device"))
        }
    )
    render_intervals = sorted(
        {
            int(value)
            for report in reports
            if isinstance(
                (value := _nested(
                    report,
                    "performance",
                    "resources",
                    "render_interval_physics_steps",
                )),
                (int, float),
            )
        }
    )
    return {
        "sample_count": len(reports),
        "required_runs": required_runs,
        "enough_runs": len(reports) >= required_runs,
        "report_paths": paths,
        "unsupported_playback_realtime_factor_median": _median_nested(
            reports, ("performance", "unsupported_playback_realtime_factor")
        ),
        "phase_latency_median": phase,
        "lowstate_publish_rate_hz_median": _median_nested(
            reports, ("performance", "dds", "lowstate_publish_hz")
        ),
        "lowcmd_receive_rate_hz_median": _median_nested(
            reports, ("performance", "lowcmd", "lowcmd_receive_rate_hz")
        ),
        "lowcmd_age_p95_ms_median": _median_nested(
            reports, ("performance", "lowcmd", "lowcmd_age_p95_ms")
        ),
        "sonic": {
            "inference_p50_ms_median": _median_nested(
                reports, ("performance", "sonic", "inference_p50_ms")
            ),
            "inference_p95_ms_median": _median_nested(
                reports, ("performance", "sonic", "inference_p95_ms")
            ),
            "control_p50_ms_median": _median_nested(
                reports, ("performance", "sonic", "control_p50_ms")
            ),
            "control_p95_ms_median": _median_nested(
                reports, ("performance", "sonic", "control_p95_ms")
            ),
            "control_p95_under_20ms_all_runs": bool(reports)
            and all(
                _nested(report, "performance", "sonic", "control_p95_under_20ms")
                is True
                for report in reports
            ),
        },
        "resources": {
            "average_cpu_cores_median": _median_nested(
                reports, ("performance", "resources", "average_cpu_cores")
            ),
            "cuda_devices": cuda_devices,
            "cuda_max_memory_allocated_bytes": max(
                (int(value) for value in cuda_allocated if isinstance(value, (int, float))),
                default=None,
            ),
            "cuda_max_memory_reserved_bytes": max(
                (int(value) for value in cuda_reserved if isinstance(value, (int, float))),
                default=None,
            ),
            "render_interval_physics_steps": render_intervals,
            "video_capture_enabled_any": any(
                _nested(report, "performance", "resources", "video_capture_enabled")
                is True
                for report in reports
            ),
        },
    }


def build_performance_report(
    exchange: Path,
    motion_ids: list[str],
    *,
    required_runs: int = 3,
    image_versions: dict[str, str] | None = None,
    asset_profile: str = "sonic_official_g1",
    generation_motion_ids: list[str] | None = None,
) -> dict[str, Any]:
    timings = []
    artifacts = []
    for motion_id in motion_ids:
        artifact = (exchange / motion_id).resolve()
        if artifact.parent != exchange.resolve() or not artifact.is_dir():
            raise FileNotFoundError(f"unknown motion artifact: {motion_id}")
        timings.append(read_timing(artifact / "timing.json"))
        artifacts.append(
            {
                "motion_id": motion_id,
                "path": str(artifact),
                "reference_checksum_sha256": artifact_checksum(artifact),
            }
        )

    # Never pool runs from different motions to satisfy the three-run gate.
    # A stage is ready only when every requested motion has enough warm samples.
    motion_results: dict[str, Any] = {}
    blocking_failures: list[str] = []
    incomplete_stages: list[str] = []
    for motion_id, timing in zip(motion_ids, timings):
        per_motion_stages: dict[str, Any] = {}
        for name, budget in LATENCY_BUDGETS.items():
            aggregate = aggregate_stage(
                _stage_samples(timing, name), required_runs=required_runs
            )
            if not aggregate["enough_runs"]:
                gate_result = "INSUFFICIENT_RUNS"
                if budget.get("blocking", True) and not (
                    generation_motion_ids and name in GENERATION_STAGES
                ):
                    incomplete_stages.append(f"{motion_id}:{name}")
            else:
                gate_result, _ = evaluate_stage(
                    name,
                    duration_s=aggregate["median_duration_s"],
                    realtime_factor=aggregate["median_realtime_factor"],
                )
                if gate_result == "SLOW" and budget.get("blocking", True):
                    blocking_failures.append(f"{motion_id}:{name}")
            per_motion_stages[name] = {
                **aggregate,
                "budget": budget,
                "gate_result": gate_result,
            }
        motion_results[motion_id] = {
            "stages": per_motion_stages,
            "execution_metrics": aggregate_execution_metrics(
                exchange, timing, required_runs
            ),
        }

    generation_benchmark = None
    generation_stage_results: dict[str, Any] = {}
    if generation_motion_ids:
        generation_artifacts = []
        generation_timings = []
        for motion_id in generation_motion_ids:
            artifact = (exchange / motion_id).resolve()
            if artifact.parent != exchange.resolve() or not artifact.is_dir():
                raise FileNotFoundError(
                    f"unknown generation benchmark artifact: {motion_id}"
                )
            timing = read_timing(artifact / "timing.json")
            generation_timings.append(timing)
            validation = json.loads((artifact / "validation.json").read_text())
            generation_artifacts.append(
                {
                    "motion_id": motion_id,
                    "path": str(artifact),
                    "artifact_checksum_sha256": artifact_checksum(artifact),
                    "trajectory_checksum_sha256": trajectory_checksum(artifact),
                    "validation_valid": validation.get("valid") is True,
                }
            )
        for name in GENERATION_STAGES:
            # Each artifact is one independently dispatched fixed-seed run.
            # Select only its latest stage so a retried request cannot silently
            # contribute multiple samples to the explicit run group.
            samples = []
            for timing in generation_timings:
                sample = timing.get("stages", {}).get(name)
                if isinstance(sample, dict):
                    samples.append(sample)
            aggregate = aggregate_stage(samples, required_runs=required_runs)
            budget = LATENCY_BUDGETS[name]
            if not aggregate["enough_runs"]:
                gate_result = "INSUFFICIENT_RUNS"
                if budget.get("blocking", True):
                    incomplete_stages.append(f"generation_benchmark:{name}")
            else:
                gate_result, _ = evaluate_stage(
                    name,
                    duration_s=aggregate["median_duration_s"],
                    realtime_factor=aggregate["median_realtime_factor"],
                )
                if gate_result == "SLOW" and budget.get("blocking", True):
                    blocking_failures.append(f"generation_benchmark:{name}")
            generation_stage_results[name] = {
                **aggregate,
                "budget": budget,
                "gate_result": gate_result,
            }
        trajectory_checksums = {
            value["trajectory_checksum_sha256"] for value in generation_artifacts
        }
        all_valid = all(value["validation_valid"] for value in generation_artifacts)
        generation_benchmark = {
            "motion_ids": generation_motion_ids,
            "artifacts": generation_artifacts,
            "fixed_seed_reproducible": len(trajectory_checksums) == 1,
            "all_validation_valid": all_valid,
            "stages": generation_stage_results,
            "result": (
                "PASS"
                if all_valid
                and len(trajectory_checksums) == 1
                and all(
                    value["gate_result"] == "PASS"
                    for value in generation_stage_results.values()
                )
                else "NOT_READY"
            ),
        }
        if not all_valid:
            blocking_failures.append("generation_benchmark:validation_result")
        if len(trajectory_checksums) != 1:
            blocking_failures.append("generation_benchmark:reproducibility")

    # Keep the top-level stage view for existing report consumers, but derive
    # it from per-motion gates instead of concatenating their sample histories.
    stage_results: dict[str, Any] = {}
    for name, budget in LATENCY_BUDGETS.items():
        if name in generation_stage_results:
            stage_results[name] = {
                **generation_stage_results[name],
                "source": "generation_benchmark",
            }
            continue
        per_motion = {
            motion_id: motion_results[motion_id]["stages"][name]
            for motion_id in motion_ids
        }
        gate_values = [value["gate_result"] for value in per_motion.values()]
        if "SLOW" in gate_values:
            gate_result = "SLOW"
        elif "INSUFFICIENT_RUNS" in gate_values:
            gate_result = "INSUFFICIENT_RUNS"
        else:
            gate_result = "PASS"
        duration_medians = [
            value["median_duration_s"]
            for value in per_motion.values()
            if value["median_duration_s"] is not None
        ]
        realtime_medians = [
            value["median_realtime_factor"]
            for value in per_motion.values()
            if value["median_realtime_factor"] is not None
        ]
        stage_results[name] = {
            "sample_count": sum(value["sample_count"] for value in per_motion.values()),
            "required_runs_per_motion": required_runs,
            "median_duration_s": (
                statistics.median(duration_medians) if duration_medians else None
            ),
            "median_realtime_factor": (
                statistics.median(realtime_medians) if realtime_medians else None
            ),
            "enough_runs": all(value["enough_runs"] for value in per_motion.values()),
            "per_motion": per_motion,
            "budget": budget,
            "gate_result": gate_result,
        }

    gate_passed = not blocking_failures and not incomplete_stages
    return {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "warm_runtime": True,
        "required_warm_runs": required_runs,
        "asset_profile": asset_profile,
        "image_versions": image_versions or {},
        "hardware": _hardware_summary(),
        "artifacts": artifacts,
        "generation_benchmark": generation_benchmark,
        "motions": motion_results,
        "stages": stage_results,
        "blocking_slow_stages": blocking_failures,
        "incomplete_stages": incomplete_stages,
        "web_ui_latency_gate_passed": gate_passed,
        "result": "PASS" if gate_passed else "NOT_READY",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate warm motion latency runs")
    parser.add_argument("motion_id", nargs="+")
    parser.add_argument(
        "--exchange", type=Path, default=Path("/motion_exchange")
    )
    parser.add_argument("--required-runs", type=int, default=3)
    parser.add_argument("--asset-profile", default="sonic_official_g1")
    parser.add_argument(
        "--generation-run",
        action="append",
        default=[],
        metavar="MOTION_ID",
        help="Fixed-seed generation artifact; repeat once per warm run",
    )
    parser.add_argument(
        "--image",
        action="append",
        default=[],
        metavar="NAME=VERSION",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    images = dict(value.split("=", 1) for value in args.image)
    report = build_performance_report(
        args.exchange,
        args.motion_id,
        required_runs=args.required_runs,
        image_versions=images,
        asset_profile=args.asset_profile,
        generation_motion_ids=args.generation_run,
    )
    output = args.output
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = args.exchange / "diagnostics/performance" / f"latency_{stamp}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(output)
    raise SystemExit(0 if report["web_ui_latency_gate_passed"] else 2)


if __name__ == "__main__":
    main()
