import json

from motion_pipeline.latency import initialize_timing, record_stage
from motion_pipeline.performance import (
    aggregate_execution_metrics,
    artifact_checksum,
    build_performance_report,
)


def test_artifact_checksum_ignores_execution_metadata(tmp_path):
    artifact = tmp_path / "motion"
    artifact.mkdir()
    (artifact / "manifest.json").write_text("reference")
    before = artifact_checksum(artifact)
    (artifact / "timing.json").write_text("changed")
    (artifact / "sonic_execution.log").write_text("changed")
    assert artifact_checksum(artifact) == before


def test_offline_video_gate_ignores_legacy_no_video_samples(tmp_path, monkeypatch):
    artifact = tmp_path / "motion"
    artifact.mkdir()
    (artifact / "manifest.json").write_text("reference")
    timing = artifact / "timing.json"
    initialize_timing(timing, "request", "motion")
    record_stage(
        timing,
        "offline_video_output",
        duration_s=0.0,
        details={"video_path": None},
    )
    record_stage(
        timing,
        "offline_video_output",
        duration_s=30.0,
        details={"video_path": "/motion_exchange/executions/replay.mp4"},
    )
    monkeypatch.setattr(
        "motion_pipeline.performance.LATENCY_BUDGETS",
        {
            "offline_video_output": {
                "kind": "max_duration_s",
                "value": 60.0,
                "blocking": False,
            }
        },
    )

    report = build_performance_report(tmp_path, ["motion"])
    stage = report["stages"]["offline_video_output"]
    assert stage["sample_count"] == 1
    assert stage["per_motion"]["motion"]["median_duration_s"] == 30.0


def test_three_run_median_controls_gate(tmp_path, monkeypatch):
    artifact = tmp_path / "motion"
    artifact.mkdir()
    (artifact / "manifest.json").write_text("reference")
    timing = artifact / "timing.json"
    initialize_timing(timing, "request", "motion")
    for duration in (2.0, 4.0, 8.0):
        record_stage(timing, "validation", duration_s=duration)
    monkeypatch.setattr(
        "motion_pipeline.performance.LATENCY_BUDGETS",
        {"validation": {"kind": "max_duration_s", "value": 5.0}},
    )

    report = build_performance_report(tmp_path, ["motion"])
    assert report["stages"]["validation"]["median_duration_s"] == 4.0
    assert report["stages"]["validation"]["gate_result"] == "PASS"
    assert report["web_ui_latency_gate_passed"] is True


def test_incomplete_required_stage_blocks_gate(tmp_path, monkeypatch):
    artifact = tmp_path / "motion"
    artifact.mkdir()
    (artifact / "manifest.json").write_text("reference")
    initialize_timing(artifact / "timing.json", "request", "motion")
    record_stage(artifact / "timing.json", "validation", duration_s=1.0)
    monkeypatch.setattr(
        "motion_pipeline.performance.LATENCY_BUDGETS",
        {"validation": {"kind": "max_duration_s", "value": 5.0}},
    )

    report = build_performance_report(tmp_path, ["motion"])
    assert report["stages"]["validation"]["gate_result"] == "INSUFFICIENT_RUNS"
    assert report["web_ui_latency_gate_passed"] is False


def test_runs_from_different_motions_are_not_pooled(tmp_path, monkeypatch):
    monkeypatch.setattr(
        "motion_pipeline.performance.LATENCY_BUDGETS",
        {"validation": {"kind": "max_duration_s", "value": 5.0}},
    )
    for motion_id, run_count in (("motion-a", 2), ("motion-b", 1)):
        artifact = tmp_path / motion_id
        artifact.mkdir()
        (artifact / "manifest.json").write_text("reference")
        timing = artifact / "timing.json"
        initialize_timing(timing, f"request-{motion_id}", motion_id)
        for _ in range(run_count):
            record_stage(timing, "validation", duration_s=1.0)

    report = build_performance_report(tmp_path, ["motion-a", "motion-b"])
    stage = report["stages"]["validation"]
    assert stage["sample_count"] == 3
    assert stage["enough_runs"] is False
    assert stage["per_motion"]["motion-a"]["sample_count"] == 2
    assert stage["per_motion"]["motion-b"]["sample_count"] == 1
    assert report["web_ui_latency_gate_passed"] is False


def test_explicit_generation_run_group_can_supply_generation_gate(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(
        "motion_pipeline.performance.LATENCY_BUDGETS",
        {"validation": {"kind": "max_duration_s", "value": 5.0}},
    )
    monkeypatch.setattr(
        "motion_pipeline.performance.GENERATION_STAGES", {"validation"}
    )
    execution = tmp_path / "execution"
    execution.mkdir()
    (execution / "manifest.json").write_text("reference")
    initialize_timing(execution / "timing.json", "request", "execution")

    generation_ids = []
    for index in range(3):
        motion_id = f"generation-{index}"
        generation_ids.append(motion_id)
        artifact = tmp_path / motion_id
        artifact.mkdir()
        (artifact / "manifest.json").write_text(f"request-{index}")
        for name in ("joint_pos.csv", "joint_vel.csv", "body_pos.csv", "body_quat.csv"):
            (artifact / name).write_text("same-reference")
        (artifact / "validation.json").write_text(json.dumps({"valid": True}))
        timing = artifact / "timing.json"
        initialize_timing(timing, f"request-{index}", motion_id)
        record_stage(timing, "validation", duration_s=1.0 + index)

    report = build_performance_report(
        tmp_path,
        ["execution"],
        generation_motion_ids=generation_ids,
    )
    assert report["generation_benchmark"]["fixed_seed_reproducible"] is True
    assert report["generation_benchmark"]["result"] == "PASS"
    assert report["stages"]["validation"]["median_duration_s"] == 2.0
    assert report["stages"]["validation"]["source"] == "generation_benchmark"
    assert report["web_ui_latency_gate_passed"] is True


def test_execution_metrics_aggregate_last_three_reports(tmp_path):
    artifact = tmp_path / "motion"
    artifact.mkdir()
    timing = artifact / "timing.json"
    initialize_timing(timing, "request", "motion")
    for index, physics_p95 in enumerate((6.0, 7.0, 8.0)):
        report = tmp_path / f"report-{index}.json"
        report.write_text(
            json.dumps(
                {
                    "performance": {
                        "unsupported_playback_realtime_factor": 0.8 + index * 0.01,
                        "phase_latency": {
                            "physics_step": {
                                "p50_ms": 4.0,
                                "p95_ms": physics_p95,
                                "p99_ms": 9.0,
                            }
                        },
                        "lowcmd": {
                            "lowcmd_receive_rate_hz": 60.0,
                            "lowcmd_age_p95_ms": 20.0 + index,
                        },
                        "sonic": {
                            "control_p95_ms": 0.4,
                            "control_p95_under_20ms": True,
                        },
                        "resources": {
                            "average_cpu_cores": 1.5,
                            "cuda_device": "cuda:0",
                            "cuda_max_memory_allocated_bytes": 1024,
                            "cuda_max_memory_reserved_bytes": 2048,
                            "render_interval_physics_steps": 20,
                            "video_capture_enabled": False,
                        },
                    }
                }
            )
        )
        record_stage(
            timing,
            "report_trace_finalization",
            duration_s=0.01,
            details={"report": str(report)},
        )

    metrics = aggregate_execution_metrics(tmp_path, json.loads(timing.read_text()), 3)
    assert metrics["enough_runs"] is True
    assert metrics["phase_latency_median"]["physics_step"]["p95_ms"] == 7.0
    assert metrics["lowcmd_age_p95_ms_median"] == 21.0
    assert metrics["sonic"]["control_p95_under_20ms_all_runs"] is True
