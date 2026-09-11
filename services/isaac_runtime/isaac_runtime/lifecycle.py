"""Shared state names and readiness rules for the persistent Isaac runtime."""

from __future__ import annotations

from enum import Enum
from typing import Any, Mapping


class RuntimeState(str, Enum):
    """Externally visible lifecycle states written to ``isaac_status.json``."""

    STARTING = "STARTING"
    SUPPORTED_BOOTSTRAP = "SUPPORTED_BOOTSTRAP"
    READY = "READY"  # Compatibility: SONIC has not taken over yet.
    READY_STANDING = "READY_STANDING"
    INTERACTIVE = "INTERACTIVE"
    SETTLING = "SETTLING"
    GROUNDING = "GROUNDING"
    EXECUTING = "EXECUTING"
    POST_HOLD_COMPLETE = "POST_HOLD_COMPLETE"
    RETURN_TO_STAND = "RETURN_TO_STAND"
    COMPLETED = "COMPLETED"
    SAFE_STOP = "SAFE_STOP"
    UNSAFE = "UNSAFE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


EXECUTION_READY_STATES = frozenset(
    {
        RuntimeState.READY.value,
        RuntimeState.READY_STANDING.value,
        RuntimeState.INTERACTIVE.value,
    }
)
TERMINAL_FAILURE_STATES = frozenset(
    {
        RuntimeState.SAFE_STOP.value,
        RuntimeState.UNSAFE.value,
        RuntimeState.FAILED.value,
        RuntimeState.ABORTED.value,
    }
)


def require_execution_ready(
    status: Mapping[str, Any],
    *,
    now_epoch_s: float,
    max_age_s: float = 5.0,
) -> None:
    """Raise ``RuntimeError`` unless a status can accept a new reference."""

    state = status.get("state")
    if state not in EXECUTION_READY_STATES:
        raise RuntimeError(f"official Isaac endpoint is not ready: {state}")
    try:
        age_s = now_epoch_s - float(status.get("updated_epoch_s", 0.0))
    except (TypeError, ValueError) as exc:
        raise RuntimeError("official Isaac readiness heartbeat is invalid") from exc
    if age_s > max_age_s:
        raise RuntimeError(
            f"official Isaac readiness heartbeat is stale: age={age_s:.2f}s"
        )
    if not status.get("session_id"):
        raise RuntimeError("official Isaac runtime status has no session_id")
