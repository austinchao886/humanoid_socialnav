import csv
import json
import signal
import time

import numpy as np

from sonic_tracker.supervisor import (
    SonicSupervisor,
    motion_completion_timeout_s,
    prepare_sonic_reference_view,
    reference_root_heights,
)


def test_sonic_reference_view_preserves_absolute_joint_positions(tmp_path):
    artifact = tmp_path / "artifact"
    artifact.mkdir()
    absolute = np.linspace(-0.7, 0.7, 29, dtype=np.float64)
    rows = np.vstack((absolute, absolute + 0.125))
    np.savetxt(
        artifact / "joint_pos.csv",
        rows,
        delimiter=",",
        header=",".join(f"joint_{index}" for index in range(29)),
        comments="",
    )
    (artifact / "metadata.txt").write_text("immutable\n")

    destination = tmp_path / "view"
    prepare_sonic_reference_view(artifact, destination)

    converted = np.loadtxt(destination / "joint_pos.csv", delimiter=",", skiprows=1)
    assert np.allclose(converted, rows)
    assert (destination / "joint_pos.csv").is_symlink()
    assert (destination / "metadata.txt").is_symlink()
    assert (artifact / "metadata.txt").read_text() == "immutable\n"


def test_sonic_reference_view_rejects_wrong_width(tmp_path):
    artifact = tmp_path / "artifact"
    artifact.mkdir()
    with (artifact / "joint_pos.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["joint_0"])
        writer.writerow([0.0])

    import pytest

    with pytest.raises(Exception, match="29 columns"):
        prepare_sonic_reference_view(artifact, tmp_path / "view")


def test_sonic_reference_view_rejects_non_finite_values(tmp_path):
    artifact = tmp_path / "artifact"
    artifact.mkdir()
    with (artifact / "joint_pos.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([f"joint_{index}" for index in range(29)])
        writer.writerow([0.0] * 28 + [float("nan")])

    import pytest

    with pytest.raises(Exception, match="non-finite"):
        prepare_sonic_reference_view(artifact, tmp_path / "view")


def test_reference_root_heights_uses_pelvis_z_column(tmp_path):
    path = tmp_path / "body_pos.csv"
    path.write_text(
        "body_0_x,body_0_y,body_0_z,body_1_x\n"
        "0.0,0.0,0.79,1.0\n"
        "0.1,0.0,0.31,1.1\n"
    )

    assert reference_root_heights(path) == [0.79, 0.31]


def test_reference_root_heights_rejects_empty_reference(tmp_path):
    path = tmp_path / "body_pos.csv"
    path.write_text("body_0_x,body_0_y,body_0_z\n")

    import pytest

    with pytest.raises(RuntimeError, match="no root-height samples"):
        reference_root_heights(path)


def test_motion_completion_timeout_accounts_for_slow_gui_rtf():
    timeout = motion_completion_timeout_s(549, 50.0, 0.066)

    assert timeout > 360.0


def test_motion_completion_timeout_keeps_legacy_guard_without_rtf():
    assert motion_completion_timeout_s(549, 50.0, None) == 139.8


def test_supervisor_requires_exact_qualified_non_diagnostic_asset_profile(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SONIC_ASSET_PROFILE", "sonic_official_g1")
    monkeypatch.setenv("SONIC_ISAAC_TASK", "Isaac-Flat-G129-SONIC-Official")
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())
    status = {
        "task": "Isaac-Flat-G129-SONIC-Official",
        "asset_profile": "sonic_official_g1",
        "asset_profile_qualified": True,
        "diagnostic_only": False,
        "state": "READY",
        "updated_epoch_s": time.time(),
        "session_id": "session",
    }
    supervisor.isaac_status_path.write_text(json.dumps(status))
    assert supervisor._require_isaac_ready()["session_id"] == "session"

    import pytest

    status["asset_profile"] = "g1_dex1_wholebody"
    supervisor.isaac_status_path.write_text(json.dumps(status))
    with pytest.raises(RuntimeError, match="wrong Isaac asset profile"):
        supervisor._require_isaac_ready()

    status["asset_profile"] = "sonic_official_g1"
    status["diagnostic_only"] = True
    supervisor.isaac_status_path.write_text(json.dumps(status))
    with pytest.raises(RuntimeError, match="diagnostic-only"):
        supervisor._require_isaac_ready()


def test_supervisor_accepts_persistent_ready_standing(tmp_path, monkeypatch):
    monkeypatch.setenv("SONIC_ASSET_PROFILE", "sonic_official_g1")
    monkeypatch.setenv("SONIC_ISAAC_TASK", "Isaac-Flat-G129-SONIC-Official")
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())
    supervisor.isaac_status_path.write_text(
        json.dumps(
            {
                "task": "Isaac-Flat-G129-SONIC-Official",
                "asset_profile": "sonic_official_g1",
                "asset_profile_qualified": True,
                "diagnostic_only": False,
                "state": "READY_STANDING",
                "updated_epoch_s": time.time(),
                "session_id": "persistent-session",
            }
        )
    )

    assert supervisor._require_isaac_ready()["session_id"] == "persistent-session"


def test_supervisor_accepts_interactive_runtime_for_reference_preemption(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SONIC_ASSET_PROFILE", "sonic_official_g1")
    monkeypatch.setenv("SONIC_ISAAC_TASK", "Isaac-Flat-G129-SONIC-Official")
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())
    supervisor.isaac_status_path.write_text(
        json.dumps(
            {
                "task": "Isaac-Flat-G129-SONIC-Official",
                "asset_profile": "sonic_official_g1",
                "asset_profile_qualified": True,
                "diagnostic_only": False,
                "state": "INTERACTIVE",
                "updated_epoch_s": time.time(),
                "session_id": "interactive-session",
            }
        )
    )

    assert supervisor._require_isaac_ready()["session_id"] == "interactive-session"


def test_supervisor_enters_native_joystick_planner_mode(tmp_path, monkeypatch):
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())

    class FakeChild:
        def __init__(self):
            self.sent = []
            self.pid = 4242

        def isalive(self):
            return True

        def send(self, value):
            self.sent.append(value)

    child = FakeChild()
    supervisor.child = child
    expected_patterns = []
    written = []
    waited = []
    monkeypatch.setattr(
        supervisor,
        "_expect_or_abort",
        lambda patterns, timeout: expected_patterns.append((patterns, timeout)),
    )
    monkeypatch.setattr(
        supervisor, "_write_runtime_request", lambda payload: written.append(dict(payload))
    )
    monkeypatch.setattr(
        supervisor,
        "_wait_for_isaac_runtime_mode",
        lambda session_id, state, timeout: waited.append((session_id, state, timeout)),
    )
    sent_signals = []
    monkeypatch.setattr(
        "sonic_tracker.supervisor.os.kill",
        lambda pid, requested_signal: sent_signals.append((pid, requested_signal)),
    )

    request = {
        "state": "IDLE",
        "request_id": "request",
        "motion_id": "motion",
    }
    supervisor._enter_joystick_locomotion(request, "same-session")

    assert child.sent == []
    assert sent_signals == [(4242, signal.SIGUSR2)]
    assert supervisor.runtime_mode == "JOYSTICK_LOCOMOTION"
    assert written[-1]["state"] == "INTERACTIVE"
    assert written[-1]["interactive_source"] == "unitree_wireless_remote"
    assert waited == [("same-session", "INTERACTIVE", 180.0)]
    assert len(expected_patterns) == 2


def test_select_loaded_motion_materializes_same_index_reference(tmp_path, monkeypatch):
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())

    class FakeChild:
        def __init__(self):
            self.sent = []

        def send(self, value):
            self.sent.append(value)

    child = FakeChild()
    supervisor.child = child
    supervisor.loaded_motion_indexes = {"neutral": 0}
    supervisor.current_motion_index = 0
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(supervisor, "_expect_or_abort", lambda *_args, **_kwargs: 0)

    supervisor._select_loaded_motion("neutral")

    assert child.sent == ["U"]
    assert supervisor.current_motion_index == 0


def test_standing_flush_plays_concrete_neutral_reference(tmp_path, monkeypatch):
    artifact = tmp_path / "neutral"
    artifact.mkdir()
    (artifact / "manifest.json").write_text(
        json.dumps({"num_frames": 200, "fps": 50.0})
    )
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())

    class FakeChild:
        def __init__(self):
            self.sent = []

        def send(self, value):
            self.sent.append(value)

    child = FakeChild()
    supervisor.child = child
    supervisor.loaded_motion_indexes = {"neutral": 0}
    supervisor.current_motion_index = 0
    expected = []
    monkeypatch.setattr(supervisor, "_standing_motion_id", lambda: "neutral")
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        supervisor,
        "_expect_or_abort",
        lambda patterns, timeout: expected.append((patterns, timeout)),
    )

    supervisor._play_standing_reference()

    assert child.sent == ["U", "T"]
    assert len(expected) == 3
    assert "Materialized motion" in expected[0][0][0]
    assert "200 total frames" in expected[1][0][0]
    assert "neutral" in expected[2][0][0]


def test_interactive_bootstrap_ignores_pre_start_isaac_heartbeat(
    tmp_path, monkeypatch
):
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())
    statuses = iter(
        [
            {
                "session_id": "old-session",
                "updated_epoch_s": supervisor.started_epoch_s - 1.0,
            },
            {
                "session_id": "new-session",
                "updated_epoch_s": supervisor.started_epoch_s + 1.0,
            },
        ]
    )
    monkeypatch.setattr(supervisor, "_require_isaac_ready", lambda: next(statuses))
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)

    assert supervisor._wait_for_isaac_ready(timeout=1.0)["session_id"] == "new-session"


def test_pre_planner_gate_requires_quiet_ready_standing(tmp_path, monkeypatch):
    supervisor = SonicSupervisor(tmp_path, tmp_path, object())
    monkeypatch.setattr(
        supervisor,
        "_read_isaac_status",
        lambda: {
            "session_id": "same-session",
            "state": "READY_STANDING",
            "root_height_m": 0.78,
            "root_tilt_rad": 0.02,
            "max_joint_velocity_rad_s": 0.8,
        },
    )

    status = supervisor._wait_for_stable_standing(
        "same-session", stable_duration=0.0, timeout=1.0
    )

    assert status["state"] == "READY_STANDING"
