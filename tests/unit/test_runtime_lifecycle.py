import time

import pytest

from isaac_runtime.lifecycle import require_execution_ready


@pytest.mark.parametrize("state", ["READY", "READY_STANDING"])
def test_execution_ready_accepts_cold_and_persistent_idle_states(state):
    status = {
        "state": state,
        "session_id": "same-process",
        "updated_epoch_s": time.time(),
    }

    require_execution_ready(status, now_epoch_s=time.time())


def test_execution_ready_rejects_busy_or_failed_runtime():
    status = {
        "state": "SAFE_STOP",
        "session_id": "same-process",
        "updated_epoch_s": time.time(),
    }

    with pytest.raises(RuntimeError, match="not ready"):
        require_execution_ready(status, now_epoch_s=time.time())


def test_execution_ready_rejects_stale_heartbeat():
    status = {
        "state": "READY_STANDING",
        "session_id": "same-process",
        "updated_epoch_s": 10.0,
    }

    with pytest.raises(RuntimeError, match="stale"):
        require_execution_ready(status, now_epoch_s=20.0, max_age_s=5.0)
