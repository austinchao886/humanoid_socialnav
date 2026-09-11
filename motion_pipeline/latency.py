from __future__ import annotations

import fcntl
import json
import math
import os
import tempfile
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator


SCHEMA_VERSION = 1

# Warm steady-state budgets.  Human approval and cold bootstrap/download time
# are recorded separately and never fail the automated latency gate.
LATENCY_BUDGETS: dict[str, dict[str, Any]] = {
    "dds_generate_to_received": {"kind": "max_duration_s", "value": 1.0},
    "kimodo_generation": {"kind": "max_duration_s", "value": 60.0},
    "reference_conversion_fk": {"kind": "max_duration_s", "value": 10.0},
    "preview_rendering": {"kind": "max_duration_s", "value": 10.0},
    "validation": {"kind": "max_duration_s", "value": 5.0},
    "validation_to_ready": {"kind": "max_duration_s", "value": 1.0},
    "approval_to_supervisor": {"kind": "max_duration_s", "value": 1.0},
    "isaac_runner_startup": {"kind": "max_duration_s", "value": 30.0},
    "sonic_initialization": {"kind": "max_duration_s", "value": 20.0},
    "approval_to_playback": {"kind": "max_duration_s", "value": 30.0},
    "settling_grounding": {"kind": "max_duration_s", "value": 15.0},
    "unsupported_playback": {"kind": "min_realtime_factor", "value": 0.8},
    "post_hold": {"kind": "max_duration_s", "value": 3.0},
    "report_trace_finalization": {"kind": "max_duration_s", "value": 5.0},
    "offline_video_output": {
        "kind": "max_duration_s",
        "value": 60.0,
        "blocking": False,
    },
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ResourceLock:
    """Cross-container advisory lock used to serialize GPU-1 workloads."""

    def __init__(self, path: Path):
        self.path = path
        self.handle = None
        self.wait_s = 0.0
        self.wait_started_at = None

    def acquire(self) -> "ResourceLock":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a+")
        self.wait_started_at = utc_now()
        started = time.monotonic()
        fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX)
        self.wait_s = time.monotonic() - started
        return self

    def release(self) -> None:
        if self.handle is None:
            return
        fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
        self.handle.close()
        self.handle = None


def new_timing(request_id: str, motion_id: str) -> dict[str, Any]:
    now = utc_now()
    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id,
        "motion_id": motion_id,
        "created_at": now,
        "updated_at": now,
        "stages": {},
        "stage_history": {},
        "blocking_slow_stages": [],
    }


def evaluate_stage(
    name: str,
    *,
    duration_s: float | None = None,
    realtime_factor: float | None = None,
    excluded: bool = False,
) -> tuple[str, dict[str, Any] | None]:
    budget = LATENCY_BUDGETS.get(name)
    if excluded:
        return "EXCLUDED", budget
    if budget is None:
        return "RECORDED", None
    kind = budget["kind"]
    threshold = float(budget["value"])
    if kind == "max_duration_s":
        passed = duration_s is not None and math.isfinite(duration_s) and duration_s <= threshold
    elif kind == "min_realtime_factor":
        passed = (
            realtime_factor is not None
            and math.isfinite(realtime_factor)
            and realtime_factor >= threshold
        )
    else:
        raise ValueError(f"unknown latency budget kind: {kind}")
    return ("PASS" if passed else "SLOW"), budget


def initialize_timing(path: Path, request_id: str, motion_id: str) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return read_timing(path)
    payload = new_timing(request_id, motion_id)
    _atomic_write(path, payload)
    return payload


def read_timing(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except FileNotFoundError:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"timing metadata must be a JSON object: {path}")
    return value


def record_stage(
    path: Path,
    name: str,
    *,
    started_at: str | None = None,
    finished_at: str | None = None,
    duration_s: float | None = None,
    realtime_factor: float | None = None,
    excluded: bool = False,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    result, budget = evaluate_stage(
        name,
        duration_s=duration_s,
        realtime_factor=realtime_factor,
        excluded=excluded,
    )
    stage: dict[str, Any] = {
        "started_at": started_at,
        "finished_at": finished_at or utc_now(),
        "duration_s": None if duration_s is None else round(float(duration_s), 6),
        "realtime_factor": (
            None if realtime_factor is None else round(float(realtime_factor), 6)
        ),
        "result": result,
        "excluded_from_gate": bool(excluded),
    }
    if budget is not None:
        stage["budget"] = budget
    if details:
        stage["details"] = details

    lock_path = path.with_name(path.name + ".lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        payload = read_timing(path)
        if not payload:
            raise FileNotFoundError(f"timing metadata is not initialized: {path}")
        payload.setdefault("stages", {})[name] = stage
        payload.setdefault("stage_history", {}).setdefault(name, []).append(stage)
        payload["updated_at"] = utc_now()
        payload["blocking_slow_stages"] = sorted(
            stage_name
            for stage_name, value in payload["stages"].items()
            if value.get("result") == "SLOW"
            and value.get("budget", {}).get("blocking", True)
        )
        _atomic_write(path, payload)
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
    return stage


def timing_summary(path: Path) -> dict[str, Any]:
    payload = read_timing(path)
    if not payload:
        return {}
    return {
        "timing_path": str(path),
        "blocking_slow_stages": payload.get("blocking_slow_stages", []),
        "stages": payload.get("stages", {}),
    }


@contextmanager
def timed_stage(
    path: Path,
    name: str,
    *,
    excluded: bool = False,
    details: dict[str, Any] | None = None,
) -> Iterator[None]:
    start = time.monotonic()
    started_at = utc_now()
    try:
        yield
    finally:
        record_stage(
            path,
            name,
            started_at=started_at,
            duration_s=time.monotonic() - start,
            excluded=excluded,
            details=details,
        )


def _atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
