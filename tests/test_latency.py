import json

from motion_pipeline.latency import (
    evaluate_stage,
    initialize_timing,
    record_stage,
    timing_summary,
)


def test_latency_budget_marks_slow_and_pass():
    assert evaluate_stage("validation", duration_s=4.0)[0] == "PASS"
    assert evaluate_stage("validation", duration_s=5.1)[0] == "SLOW"
    assert (
        evaluate_stage("unsupported_playback", realtime_factor=0.8)[0]
        == "PASS"
    )
    assert (
        evaluate_stage("unsupported_playback", realtime_factor=0.79)[0]
        == "SLOW"
    )


def test_excluded_human_stage_never_blocks(tmp_path):
    path = tmp_path / "timing.json"
    initialize_timing(path, "request", "motion")
    record_stage(
        path,
        "human_approval_wait",
        duration_s=3600.0,
        excluded=True,
    )

    value = json.loads(path.read_text())
    assert value["stages"]["human_approval_wait"]["result"] == "EXCLUDED"
    assert value["blocking_slow_stages"] == []


def test_nonblocking_video_budget_does_not_block_gate(tmp_path):
    path = tmp_path / "timing.json"
    initialize_timing(path, "request", "motion")
    record_stage(path, "offline_video_output", duration_s=61.0)

    summary = timing_summary(path)
    assert summary["stages"]["offline_video_output"]["result"] == "SLOW"
    assert summary["blocking_slow_stages"] == []


def test_atomic_updates_preserve_prior_stages(tmp_path):
    path = tmp_path / "timing.json"
    initialize_timing(path, "request", "motion")
    record_stage(path, "kimodo_generation", duration_s=12.0)
    record_stage(path, "validation", duration_s=1.0)

    value = json.loads(path.read_text())
    assert set(value["stages"]) == {"kimodo_generation", "validation"}
    assert value["request_id"] == "request"
    assert value["motion_id"] == "motion"


def test_repeated_runs_keep_stage_history(tmp_path):
    path = tmp_path / "timing.json"
    initialize_timing(path, "request", "motion")
    record_stage(path, "unsupported_playback", duration_s=2.0, realtime_factor=1.0)
    record_stage(path, "unsupported_playback", duration_s=4.0, realtime_factor=0.5)

    value = json.loads(path.read_text())
    history = value["stage_history"]["unsupported_playback"]
    assert [entry["realtime_factor"] for entry in history] == [1.0, 0.5]
    assert value["stages"]["unsupported_playback"]["result"] == "SLOW"
