import json

import pytest

from sonic_tracker.supervisor import (
    enrich_execution_report,
    parse_sonic_timing_log,
)


def test_parse_sonic_timing_log_reports_percentiles(tmp_path):
    log = tmp_path / "sonic.log"
    log.write_text(
        "Loop timing - LowState age: 4.0ms, Obs: 200us, Policy: 100us, "
        "Obs 2 Motor Command: 300us, Post processing: 20us\n"
        "Loop timing - LowState age: 8.0ms, Obs: 400us, Policy: 200us, "
        "Obs 2 Motor Command: 600us, Post processing: 20us\n"
    )

    result = parse_sonic_timing_log(log)
    assert result["sample_count"] == 2
    assert result["inference_p50_ms"] == pytest.approx(0.15)
    assert result["control_p50_ms"] == pytest.approx(0.45)
    assert result["control_p95_under_20ms"] is True


def test_enrich_execution_report_preserves_existing_performance(tmp_path):
    report = tmp_path / "report.json"
    report.write_text(json.dumps({"performance": {"physics": {"p50_ms": 4.0}}}))

    enrich_execution_report(report, {"control_p95_ms": 0.4})

    payload = json.loads(report.read_text())
    assert payload["performance"]["physics"]["p50_ms"] == 4.0
    assert payload["performance"]["sonic"]["control_p95_ms"] == 0.4
