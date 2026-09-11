from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import re
import shutil
import signal
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path

import pexpect

from motion_contracts.protocol import CONTROL_TOPIC, STATUS_TOPIC, ControlCommand, ProtocolError, State, status_json
from motion_contracts.validator import validate_artifact
from motion_pipeline.core.latency import (
    ResourceLock,
    initialize_timing,
    read_timing,
    record_stage,
    timing_summary,
    utc_now,
)
from motion_pipeline.runtime.dds_transport import JsonDDS
from motion_pipeline.runtime_lifecycle import (
    TERMINAL_FAILURE_STATES,
    require_execution_ready,
)


SONIC_LOOP_TIMING_RE = re.compile(
    r"LowState age:\s*([0-9.]+)ms.*?"
    r"Obs:\s*([0-9.]+)us,\s*Policy:\s*([0-9.]+)us,\s*"
    r"Obs 2 Motor Command:\s*([0-9.]+)us"
)


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def parse_sonic_timing_log(path: Path) -> dict[str, object]:
    """Extract control/inference latency without instrumenting SONIC's hot loop."""

    policy_ms: list[float] = []
    control_ms: list[float] = []
    observation_ms: list[float] = []
    lowstate_age_ms: list[float] = []
    try:
        lines = path.read_text(errors="replace").splitlines()
    except FileNotFoundError:
        lines = []
    for line in lines:
        match = SONIC_LOOP_TIMING_RE.search(line)
        if match is None:
            continue
        lowstate_age_ms.append(float(match.group(1)))
        observation_ms.append(float(match.group(2)) / 1000.0)
        policy_ms.append(float(match.group(3)) / 1000.0)
        control_ms.append(float(match.group(4)) / 1000.0)
    return {
        "sample_count": len(control_ms),
        "inference_p50_ms": _percentile(policy_ms, 0.50),
        "inference_p95_ms": _percentile(policy_ms, 0.95),
        "control_p50_ms": _percentile(control_ms, 0.50),
        "control_p95_ms": _percentile(control_ms, 0.95),
        "observation_p50_ms": _percentile(observation_ms, 0.50),
        "observation_p95_ms": _percentile(observation_ms, 0.95),
        "lowstate_age_p50_ms": _percentile(lowstate_age_ms, 0.50),
        "lowstate_age_p95_ms": _percentile(lowstate_age_ms, 0.95),
        "control_p95_under_20ms": (
            bool(control_ms) and _percentile(control_ms, 0.95) < 20.0
        ),
    }


def enrich_execution_report(path: Path, sonic: dict[str, object]) -> None:
    try:
        payload = json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return
    payload.setdefault("performance", {})["sonic"] = sonic
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def reference_root_heights(path: Path) -> list[float]:
    """Load the pelvis/root z contract carried in column 2 of body_pos.csv."""

    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        next(reader)
        values = [float(row[2]) for row in reader if row]
    if not values:
        raise RuntimeError(f"reference has no root-height samples: {path}")
    return values


def motion_completion_timeout_s(
    num_frames: int,
    reference_hz: float,
    release_realtime_factor: object,
) -> float:
    """Size the wall-clock liveness guard from Isaac's measured sim rate."""
    if num_frames <= 0 or reference_hz <= 0.0:
        raise ValueError("motion completion timeout requires positive frames and fps")
    duration_s = num_frames / reference_hz
    legacy_guard_s = duration_s * 10.0 + 30.0
    try:
        measured_rtf = float(release_realtime_factor)
    except (TypeError, ValueError):
        measured_rtf = 0.0
    measured_guard_s = 0.0
    if math.isfinite(measured_rtf) and measured_rtf > 0.0:
        # The timeout is only a liveness guard. Runtime fall, NaN, tracking,
        # DDS-loss, and abort checks remain active in _expect_or_abort.
        measured_guard_s = duration_s / measured_rtf * 2.0 + 30.0
    return max(60.0, legacy_guard_s, measured_guard_s)


class SonicSupervisor:
    def __init__(self, exchange: Path, sonic_root: Path, dds: JsonDDS):
        self.exchange = exchange.resolve()
        self.sonic_root = sonic_root
        self.dds = dds
        self.started_epoch_s = time.time()
        self.queue: queue.Queue[tuple[str, float, str]] = queue.Queue()
        self.child: pexpect.spawn | None = None
        self.execution_thread: threading.Thread | None = None
        self.abort_event = threading.Event()
        self.active_command: ControlCommand | None = None
        self.lock = threading.Lock()
        self.runtime_dir = self.exchange / ".runtime"
        self.runtime_dir.mkdir(exist_ok=True)
        self.isaac_status_path = self.runtime_dir / "isaac_status.json"
        self.runtime_request_path = self.runtime_dir / "request.json"
        self.required_asset_profile = os.getenv(
            "SONIC_ASSET_PROFILE", "sonic_official_g1"
        )
        self.required_isaac_task = os.getenv(
            "SONIC_ISAAC_TASK", "Isaac-Flat-G129-SONIC-Official"
        )
        self.required_reference_contract = os.getenv(
            "SONIC_REFERENCE_CONTRACT", "sonic_official_g1"
        )
        self.persistent_process = os.getenv(
            "SONIC_PERSISTENT_PROCESS", "1"
        ) not in {"0", "false", "False"}
        self.loaded_motion_indexes: dict[str, int] = {}
        self.current_motion_index = 0
        self.control_started = False
        self.runtime_mode = "STOPPED"
        self.child_log_handle = None

    def run(self) -> None:
        def receive(raw: str) -> None:
            request_id, motion_id = _ids(raw)
            print(
                "[sonic-supervisor] received control command "
                f"request_id={request_id} motion_id={motion_id}",
                flush=True,
            )
            self.queue.put((raw, time.monotonic(), utc_now()))

        self.dds.subscribe(CONTROL_TOPIC, receive)
        status = {
            "schema_version": 1,
            "state": "READY",
            "dds_domain": int(os.getenv("DDS_DOMAIN", "0")),
            "dds_interface": os.getenv("DDS_INTERFACE") or None,
            "asset_profile": self.required_asset_profile,
            "isaac_task": self.required_isaac_task,
            "reference_contract": self.required_reference_contract,
            "updated_epoch_s": time.time(),
        }
        status_path = self.runtime_dir / "sonic_supervisor_status.json"
        temporary = status_path.with_name(f".{status_path.name}.{os.getpid()}.tmp")
        temporary.write_text(json.dumps(status, indent=2, sort_keys=True) + "\n")
        os.replace(temporary, status_path)
        print(
            "[sonic-supervisor] READY "
            f"domain={status['dds_domain']} interface={status['dds_interface']} "
            f"asset_profile={self.required_asset_profile} ",
            flush=True,
        )
        if (
            self.persistent_process
            and os.getenv("SONIC_INTERACTIVE_RUNTIME", "1")
            not in {"0", "false", "False"}
            and os.getenv("SONIC_ENABLE_PLANNER", "0")
            not in {"0", "false", "False"}
        ):
            try:
                self._bootstrap_interactive_runtime()
            except Exception as exc:
                # Keep the DDS supervisor available. A later approve_execute
                # can still cold-start the controller, while the bootstrap
                # failure remains explicit in logs instead of being hidden.
                print(
                    "[sonic-supervisor] interactive bootstrap failed: "
                    f"{exc}",
                    flush=True,
                )
                self._stop()
        while True:
            raw, received_monotonic, received_at = self.queue.get()
            queue_wait_s = time.monotonic() - received_monotonic
            try:
                command = ControlCommand.parse(raw)
                self._handle(
                    command,
                    received_monotonic=received_monotonic,
                    received_at=received_at,
                    queue_wait_s=queue_wait_s,
                )
            except Exception as exc:
                request_id, motion_id = _ids(raw)
                self.dds.publish(STATUS_TOPIC, status_json(request_id, State.FAILED, motion_id=motion_id, error={"message": str(exc)}))

    def _handle(
        self,
        command: ControlCommand,
        *,
        received_monotonic: float | None = None,
        received_at: str | None = None,
        queue_wait_s: float = 0.0,
    ) -> None:
        if command.action == "approve_execute":
            self._start_execution(
                command,
                received_monotonic=received_monotonic or time.monotonic(),
                received_at=received_at or utc_now(),
                queue_wait_s=queue_wait_s,
            )
        elif command.action == "reject":
            self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.REJECTED, motion_id=command.motion_id))
        elif command.action in {"abort", "reset"}:
            with self.lock:
                active = self.active_command
            if active is not None and command.motion_id != active.motion_id:
                raise ProtocolError(
                    f"active motion is {active.motion_id}, not {command.motion_id}"
                )
            if active is not None:
                self._write_runtime_request(
                    {
                        "schema_version": 1,
                        "state": "ABORTED",
                        "request_id": active.request_id,
                        "motion_id": active.motion_id,
                        "aborted_epoch_s": time.time(),
                    }
                )
            self.abort_event.set()
            self._stop()
            self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.ABORTED, motion_id=command.motion_id))

    def _start_execution(
        self,
        command: ControlCommand,
        *,
        received_monotonic: float,
        received_at: str,
        queue_wait_s: float,
    ) -> None:
        with self.lock:
            if self.execution_thread is not None and self.execution_thread.is_alive():
                raise ProtocolError("a motion is already executing")
            self.abort_event.clear()
            self.active_command = command
            self.execution_thread = threading.Thread(
                target=self._execute_guarded,
                args=(command, received_monotonic, received_at, queue_wait_s),
                name=f"sonic-{command.motion_id}",
                daemon=True,
            )
            self.execution_thread.start()

    def _execute_guarded(
        self,
        command: ControlCommand,
        approval_received_monotonic: float,
        approval_received_at: str,
        queue_wait_s: float,
    ) -> None:
        try:
            self._execute(
                command,
                approval_received_monotonic=approval_received_monotonic,
                approval_received_at=approval_received_at,
                queue_wait_s=queue_wait_s,
            )
        except Exception as exc:
            if not self.abort_event.is_set():
                self.dds.publish(
                    STATUS_TOPIC,
                    status_json(
                        command.request_id,
                        State.FAILED,
                        motion_id=command.motion_id,
                        error={"message": str(exc)},
                    ),
                )
            # A failed/safety-aborted execution may have left controller state
            # inconsistent.  Never reuse that process for another motion.
            self._stop()
        finally:
            if not self.persistent_process:
                self._stop()
            with self.lock:
                self.active_command = None

    def _execute(
        self,
        command: ControlCommand,
        *,
        approval_received_monotonic: float,
        approval_received_at: str,
        queue_wait_s: float,
    ) -> None:
        artifact = (self.exchange / command.motion_id).resolve()
        if artifact.parent != self.exchange or not artifact.is_dir():
            raise ProtocolError(f"unknown motion_id: {command.motion_id}")
        timing_path = artifact / "timing.json"
        initialize_timing(timing_path, command.request_id, command.motion_id)
        record_stage(
            timing_path,
            "approval_to_supervisor",
            started_at=approval_received_at,
            duration_s=queue_wait_s,
            details={"server_received_at": approval_received_at},
        )
        timing_payload = read_timing(timing_path)
        ready_stage = timing_payload.get("stages", {}).get("validation_to_ready", {})
        ready_finished_at = ready_stage.get("finished_at")
        if ready_finished_at:
            try:
                ready_epoch_s = datetime.fromisoformat(ready_finished_at).timestamp()
                record_stage(
                    timing_path,
                    "human_approval_wait",
                    started_at=ready_finished_at,
                    duration_s=max(0.0, time.time() - ready_epoch_s),
                    excluded=True,
                )
            except (TypeError, ValueError):
                pass

        validation_started_at = utc_now()
        validation_started = time.monotonic()
        result = validate_artifact(artifact)
        record_stage(
            timing_path,
            "execution_gate_validation",
            started_at=validation_started_at,
            duration_s=time.monotonic() - validation_started,
            details={"valid": result.valid},
        )
        if not result.valid:
            raise ProtocolError("artifact failed validation at execution gate")
        manifest = json.loads((artifact / "manifest.json").read_text())
        if manifest.get("request_id") != command.request_id:
            raise ProtocolError("request_id does not own motion_id")
        reference_contract = manifest.get("execution_contract", {}).get("asset")
        if reference_contract != self.required_reference_contract:
            raise ProtocolError(
                "reference asset contract mismatch: "
                f"expected={self.required_reference_contract}, "
                f"actual={reference_contract}"
            )

        isaac_ready_started_at = utc_now()
        isaac_ready_started = time.monotonic()
        isaac_ready = self._require_isaac_ready()
        record_stage(
            timing_path,
            "isaac_ready_check",
            started_at=isaac_ready_started_at,
            duration_s=time.monotonic() - isaac_ready_started,
            details={"session_id": isaac_ready.get("session_id")},
        )
        isaac_ready_performance = isaac_ready.get("performance") or {}
        runner_startup_s = isaac_ready_performance.get("isaac_runner_startup_s")
        if runner_startup_s is not None:
            record_stage(
                timing_path,
                "isaac_runner_startup",
                started_at=isaac_ready_performance.get("runner_started_at"),
                finished_at=isaac_ready_performance.get("runner_ready_at"),
                duration_s=float(runner_startup_s),
                details={
                    "session_id": isaac_ready.get("session_id"),
                    "warm_runtime": True,
                },
            )

        approval = {
            "schema_version": 1,
            "request_id": command.request_id,
            "motion_id": command.motion_id,
            "action": command.action,
            "approved_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "validation": result.__dict__,
            "execution_adapter": {
                "version": 2,
                "joint_position_input": "absolute_isaaclab",
                "joint_position_sonic": "absolute_isaaclab",
                "joint_position_conversion": "identity",
                "asset_profile": self.required_asset_profile,
                "reference_contract": self.required_reference_contract,
            },
        }
        (artifact / "approval.json").write_text(
            json.dumps(approval, indent=2, sort_keys=True) + "\n"
        )

        post_hold = float(os.getenv("SONIC_POST_MOTION_HOLD_S", "2.0"))
        runtime_request = {
            "schema_version": 1,
            "state": "STARTING",
            "request_id": command.request_id,
            "motion_id": command.motion_id,
            "isaac_session_id": isaac_ready["session_id"],
            "asset_profile": self.required_asset_profile,
            "reference_contract": self.required_reference_contract,
            "approved_epoch_s": time.time(),
            "reference_hz": float(manifest["fps"]),
            "post_hold_s": post_hold,
            "reference_root_heights_m": reference_root_heights(
                artifact / "body_pos.csv"
            ),
            "reference_joint_pos_path": str(artifact / "joint_pos.csv"),
        }
        self._write_runtime_request(runtime_request)

        self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.EXECUTING, motion_id=command.motion_id, artifact_path=str(artifact)))
        executable = self.sonic_root / "target/release/g1_deploy_onnx_ref"
        if not executable.is_file():
            raise RuntimeError(f"SONIC executable is missing: {executable}")
        log = (
            self.runtime_dir / "sonic_persistent.log"
            if self.persistent_process
            else artifact / "sonic_execution.log"
        )
        selection = None
        gpu_lock = ResourceLock(self.exchange / ".runtime/gpu1.lock").acquire()
        started = time.monotonic()
        try:
            record_stage(
                timing_path,
                "gpu1_resource_queue_wait",
                started_at=gpu_lock.wait_started_at,
                duration_s=gpu_lock.wait_s,
                excluded=True,
                details={"resource": "gpu1", "owner": "sonic"},
            )
            child_alive = bool(self.child is not None and self.child.isalive())
            if child_alive and command.motion_id not in self.loaded_motion_indexes:
                # A newly validated motion was not part of the prior preload.
                # Rebuild only while idle; this run is explicitly cold and the
                # next runs reuse the enlarged pool.
                self._stop()
                child_alive = False
            reused_process = child_alive
            sonic_initialization_started_at = utc_now()
            sonic_initialization_started = time.monotonic()
            planner_enabled = os.getenv(
                "SONIC_ENABLE_PLANNER", "0"
            ) not in {"0", "false", "False"}
            if not reused_process:
                selection = self._spawn_controller(
                    command.motion_id,
                    artifact=artifact,
                    log=log,
                    planner_enabled=planner_enabled,
                )
            else:
                # Output accumulated while Isaac was between runner sessions is
                # idle telemetry, not an active execution failure. Drain it
                # before establishing the new STARTING/SETTLING boundary.
                while True:
                    try:
                        self.child.read_nonblocking(size=4096, timeout=0)
                    except pexpect.TIMEOUT:
                        break
                    except pexpect.EOF as exc:
                        raise RuntimeError(
                            "persistent SONIC exited while idle"
                        ) from exc
            initialization_s = time.monotonic() - sonic_initialization_started
            cold_bootstrap = bool(
                not reused_process and self.persistent_process
            )
            record_stage(
                timing_path,
                "sonic_initialization",
                started_at=sonic_initialization_started_at,
                duration_s=initialization_s,
                excluded=cold_bootstrap,
                details={
                    "executable": str(executable),
                    "persistent_reuse": reused_process,
                    "cold_bootstrap": cold_bootstrap,
                    "preloaded_motion_count": len(self.loaded_motion_indexes),
                    "planner_enabled": planner_enabled,
                },
            )
            settling_started_at = utc_now()
            settling_started = time.monotonic()
            if not reused_process:
                # Init Done is the end of SONIC's physical initial-pose ramp,
                # not model loading.  Account for it under settling/grounding.
                self._expect_or_abort(["Init Done"], timeout=30)
            if command.motion_id not in self.loaded_motion_indexes:
                raise RuntimeError(
                    f"SONIC did not preload approved motion: {command.motion_id}"
                )
            if planner_enabled and self.runtime_mode == "JOYSTICK_LOCOMOTION":
                # SIGUSR1 asks InterfaceManager's 100 Hz input loop to leave
                # gamepad/planner mode through its safety reset before any
                # offline reference is selected. This control-plane signal is
                # deterministic even when the pseudo-terminal is long-lived.
                self._signal_runtime_mode(signal.SIGUSR1)
                self._expect_or_abort(
                    [r"\[InterfaceManager\] Runtime mode: REFERENCE"],
                    timeout=5,
                )
                self._expect_or_abort(
                    [r"Safety reset: Returned to reference motion at frame 0"],
                    timeout=5,
                )
                self.runtime_mode = "REFERENCE"
                # Planner locomotion leaves recurrent policy history in a
                # different distribution from an offline reference. Flush it
                # through the explicit neutral trajectory before selecting the
                # approved motion; switching the index directly can produce a
                # discontinuous target and tip an unsupported robot.
                self._play_standing_reference()
                self._wait_for_stable_standing(
                    isaac_ready["session_id"],
                    stable_duration=float(
                        os.getenv("SONIC_PRE_REFERENCE_STABLE_S", "3.0")
                    ),
                    timeout=float(
                        os.getenv("SONIC_PRE_REFERENCE_TIMEOUT_S", "60.0")
                    ),
                    accepted_states={"INTERACTIVE"},
                )
            self._select_loaded_motion(command.motion_id)
            if not self.control_started:
                self.child.send("]")
                self._expect_or_abort([r"transitioning to CONTROL state"], timeout=10)
                self.control_started = True
            runtime_request["state"] = "SETTLING"
            runtime_request["control_epoch_s"] = time.time()
            self._write_runtime_request(runtime_request)
            executing_status = self._wait_for_isaac_state(
                command,
                isaac_ready["session_id"],
                "EXECUTING",
                timeout=180.0,
            )
            record_stage(
                timing_path,
                "settling_grounding",
                started_at=settling_started_at,
                duration_s=time.monotonic() - settling_started,
                details={
                    "release_realtime_factor": executing_status.get(
                        "release_realtime_factor"
                    )
                },
            )
            self.child.send("T")
            runtime_request["state"] = "PLAYING"
            runtime_request["playback_epoch_s"] = time.time()
            self._write_runtime_request(runtime_request)
            self._expect_or_abort(
                [rf"Playing motion .*\({int(manifest['num_frames'])} total frames\)"],
                timeout=10,
            )
            playback_started_at = utc_now()
            playback_started = time.monotonic()
            record_stage(
                timing_path,
                "approval_to_playback",
                started_at=approval_received_at,
                duration_s=playback_started - approval_received_monotonic,
                details={"cold_bootstrap": cold_bootstrap},
            )
            playback_timeout = motion_completion_timeout_s(
                int(manifest["num_frames"]),
                float(manifest["fps"]),
                executing_status.get("release_realtime_factor"),
            )
            print(
                "[sonic-supervisor] waiting for motion completion; "
                f"timeout={playback_timeout:.1f}s, "
                f"release_rtf={executing_status.get('release_realtime_factor')}",
                flush=True,
            )
            self._expect_or_abort(
                [rf"Motion index: .* : {re_escape(command.motion_id)} completed\."],
                timeout=playback_timeout,
            )
            playback_wall_s = time.monotonic() - playback_started
            reference_duration_s = float(manifest["num_frames"]) / float(
                manifest["fps"]
            )
            record_stage(
                timing_path,
                "sonic_reference_playback",
                started_at=playback_started_at,
                duration_s=playback_wall_s,
                realtime_factor=(
                    reference_duration_s / playback_wall_s
                    if playback_wall_s > 0.0
                    else 0.0
                ),
                details={
                    "reference_duration_s": reference_duration_s,
                    "frames": int(manifest["num_frames"]),
                },
            )

            post_hold_started_at = utc_now()
            post_hold_started = time.monotonic()
            self._wait_for_isaac_state(
                command,
                isaac_ready["session_id"],
                "POST_HOLD_COMPLETE",
                timeout=max(60.0, post_hold * 20.0),
            )
            record_stage(
                timing_path,
                "post_hold",
                started_at=post_hold_started_at,
                duration_s=time.monotonic() - post_hold_started,
                details={
                    "simulation_duration_s": post_hold,
                    "source": "isaac_physics_post_hold_complete_state",
                },
            )
            runtime_request["state"] = "STOPPING"
            runtime_request["stopping_epoch_s"] = time.time()
            self._write_runtime_request(runtime_request)
            if not self.persistent_process:
                self.child.send("O")
                self.child.expect(pexpect.EOF, timeout=10)
            isaac_result = self._wait_for_isaac_completion(
                command, isaac_ready["session_id"], timeout=10.0
            )
            if self.persistent_process:
                runtime_request["state"] = "IDLE"
                runtime_request["idle_epoch_s"] = time.time()
                self._write_runtime_request(runtime_request)
                self._wait_for_isaac_idle(
                    isaac_ready["session_id"], command.motion_id, timeout=10.0
                )
                if planner_enabled:
                    # Do not initialize the locomotion planner from residual
                    # reference-policy history. Play the explicit neutral
                    # reference once to flush that history, wait until the
                    # unsupported robot is observably quiet, and only then let
                    # the planner take over.
                    self._play_standing_reference()
                    self._wait_for_stable_standing(
                        isaac_ready["session_id"],
                        stable_duration=float(
                            os.getenv("SONIC_PRE_PLANNER_STABLE_S", "3.0")
                        ),
                        timeout=float(
                            os.getenv("SONIC_PRE_PLANNER_TIMEOUT_S", "60.0")
                        ),
                    )
                    self._enter_joystick_locomotion(
                        runtime_request, isaac_ready["session_id"]
                    )
            isaac_performance = isaac_result.get("performance") or {}
            if self.child_log_handle is not None:
                self.child_log_handle.flush()
            sonic_performance = parse_sonic_timing_log(log)
            isaac_performance = dict(isaac_performance)
            isaac_performance["sonic"] = sonic_performance
            report_path = isaac_result.get("report_path")
            if report_path:
                enrich_execution_report(Path(report_path), sonic_performance)
            record_stage(
                timing_path,
                "sonic_control_metrics",
                duration_s=(
                    float(sonic_performance["control_p95_ms"]) / 1000.0
                    if sonic_performance.get("control_p95_ms") is not None
                    else None
                ),
                details=sonic_performance,
            )
            isaac_playback_rtf = isaac_performance.get(
                "unsupported_playback_realtime_factor"
            )
            isaac_playback_wall_s = isaac_performance.get(
                "reference_playback_wall_s"
            )
            if isaac_playback_rtf is not None and isaac_playback_wall_s is not None:
                record_stage(
                    timing_path,
                    "unsupported_playback",
                    duration_s=float(isaac_playback_wall_s),
                    realtime_factor=float(isaac_playback_rtf),
                    details={
                        "reference_duration_s": isaac_performance.get(
                            "reference_duration_s"
                        ),
                        "frames": int(manifest["num_frames"]),
                        "source": "isaac_physics_clock",
                    },
                )
            report_finalization_s = isaac_performance.get(
                "report_trace_finalization_s"
            )
            if report_finalization_s is not None:
                record_stage(
                    timing_path,
                    "report_trace_finalization",
                    duration_s=float(report_finalization_s),
                    details={"report": isaac_result.get("report_path")},
                )
            if (
                isaac_result.get("video_path")
                and isaac_performance.get("video_finalization_s") is not None
            ):
                record_stage(
                    timing_path,
                    "offline_video_output",
                    duration_s=float(isaac_performance["video_finalization_s"]),
                    details={
                        "non_blocking_budget": True,
                        "video_path": isaac_result.get("video_path"),
                    },
                )
            self.dds.publish(
                STATUS_TOPIC,
                status_json(
                    command.request_id,
                    State.COMPLETED,
                    motion_id=command.motion_id,
                    execution={
                        "log": str(log),
                        "elapsed_s": round(time.monotonic() - started, 3),
                        "frames": int(manifest["num_frames"]),
                        "isaac_report": isaac_result.get("report_path"),
                        "isaac_trace": isaac_result.get("trace_path"),
                        "release_realtime_factor": isaac_result.get(
                            "release_realtime_factor"
                        ),
                        "performance": isaac_performance,
                    },
                    timing=timing_summary(timing_path),
                ),
            )
        finally:
            gpu_lock.release()
            if selection is not None and not self.persistent_process:
                shutil.rmtree(selection, ignore_errors=True)

    def _read_isaac_status(self) -> dict:
        try:
            value = json.loads(self.isaac_status_path.read_text())
        except FileNotFoundError as exc:
            raise RuntimeError("official Isaac execution endpoint is not running") from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError("official Isaac runtime status is invalid JSON") from exc
        if not isinstance(value, dict):
            raise RuntimeError("official Isaac runtime status must be an object")
        return value

    def _write_runtime_request(self, payload: dict) -> None:
        temporary = self.runtime_request_path.with_name(".request.json.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        os.replace(temporary, self.runtime_request_path)

    def _require_isaac_ready(self) -> dict:
        status = self._read_isaac_status()
        if status.get("task") != self.required_isaac_task:
            raise RuntimeError(f"wrong Isaac execution task: {status.get('task')}")
        if status.get("asset_profile") != self.required_asset_profile:
            raise RuntimeError(
                "wrong Isaac asset profile: "
                f"expected={self.required_asset_profile}, "
                f"actual={status.get('asset_profile')}"
            )
        if not status.get("asset_profile_qualified"):
            raise RuntimeError(
                f"Isaac asset profile {status.get('asset_profile')} is not qualified"
            )
        if status.get("diagnostic_only"):
            raise RuntimeError("Isaac endpoint is diagnostic-only and cannot execute motion")
        require_execution_ready(status, now_epoch_s=time.time())
        return status

    def _runtime_failure(self) -> str | None:
        command = self.active_command
        if command is None:
            return None
        try:
            status = self._read_isaac_status()
        except RuntimeError as exc:
            return str(exc)
        if status.get("motion_id") not in {None, command.motion_id}:
            return None
        if status.get("state") in TERMINAL_FAILURE_STATES:
            return str(status.get("reason") or f"Isaac state={status.get('state')}")
        return None

    def _wait_for_isaac_idle(
        self, session_id: str, last_motion_id: str, timeout: float
    ) -> dict:
        """Wait until the same simulator session is standing and reusable."""

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self._read_isaac_status()
            if status.get("session_id") != session_id:
                raise RuntimeError("Isaac execution session changed while returning to stand")
            if (
                status.get("state") == "READY_STANDING"
                and status.get("last_motion_id") == last_motion_id
            ):
                return status
            if status.get("state") in TERMINAL_FAILURE_STATES:
                raise RuntimeError(
                    str(status.get("reason") or f"Isaac state={status.get('state')}")
                )
            time.sleep(0.1)
        raise RuntimeError("timed out waiting for persistent Isaac standing state")

    def _wait_for_isaac_state(
        self,
        command: ControlCommand,
        session_id: str,
        desired_state: str,
        timeout: float,
    ) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                raise RuntimeError("execution aborted")
            status = self._read_isaac_status()
            if status.get("session_id") != session_id:
                raise RuntimeError("Isaac execution session changed during motion")
            if status.get("motion_id") == command.motion_id:
                if status.get("state") == desired_state:
                    return status
                if status.get("state") in {"UNSAFE", "FAILED", "ABORTED"}:
                    raise RuntimeError(
                        str(status.get("reason") or f"Isaac state={status.get('state')}")
                    )
            time.sleep(0.1)
        raise RuntimeError(f"timed out waiting for Isaac state={desired_state}")

    def _wait_for_isaac_sim_duration(
        self,
        command: ControlCommand,
        session_id: str,
        duration_s: float,
        timeout: float,
    ) -> None:
        if duration_s <= 0.0:
            return
        first = self._wait_for_isaac_state(
            command, session_id, "EXECUTING", timeout=min(timeout, 10.0)
        )
        deadline = time.monotonic() + timeout
        start_value = first.get("simulation_time_s")
        while start_value is None and time.monotonic() < deadline:
            time.sleep(0.1)
            first = self._read_isaac_status()
            start_value = first.get("simulation_time_s")
        if start_value is None:
            raise RuntimeError("Isaac status did not report simulation_time_s")
        start_sim_s = float(start_value)
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                raise RuntimeError("execution aborted")
            status = self._read_isaac_status()
            if status.get("session_id") != session_id:
                raise RuntimeError("Isaac execution session changed during motion")
            if status.get("motion_id") == command.motion_id:
                if status.get("state") in {"UNSAFE", "FAILED", "ABORTED"}:
                    raise RuntimeError(
                        str(status.get("reason") or f"Isaac state={status.get('state')}")
                    )
                if float(status.get("simulation_time_s", 0.0)) - start_sim_s >= duration_s:
                    return
            time.sleep(0.1)
        raise RuntimeError(
            f"timed out waiting for {duration_s:.3f}s of Isaac simulation time"
        )

    def _wait_for_isaac_completion(
        self, command: ControlCommand, session_id: str, timeout: float
    ) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                raise RuntimeError("execution aborted")
            status = self._read_isaac_status()
            if status.get("session_id") != session_id:
                raise RuntimeError("Isaac execution session changed during motion")
            if status.get("motion_id") == command.motion_id:
                if status.get("state") == "COMPLETED":
                    report_path = status.get("report_path")
                    if not report_path or not Path(report_path).is_file():
                        raise RuntimeError("Isaac completed without a readable execution report")
                    return status
                if status.get("state") in {"UNSAFE", "FAILED"}:
                    raise RuntimeError(
                        str(status.get("reason") or f"Isaac state={status.get('state')}")
                    )
            time.sleep(0.1)
        raise RuntimeError("timed out waiting for Isaac execution completion")

    def _expect_or_abort(self, patterns: list[str], timeout: float) -> int:
        if self.child is None:
            raise RuntimeError("SONIC process was not started")
        errors = r"\[ERROR\]|\bNaN\b|Safety check failed|Lost LowState|fall"
        deadline = time.monotonic() + timeout
        while True:
            if self.abort_event.is_set():
                raise RuntimeError("execution aborted")
            runtime_failure = self._runtime_failure()
            if runtime_failure is not None:
                raise RuntimeError(f"Isaac runtime failure: {runtime_failure}")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(f"SONIC timed out waiting for: {patterns[0]}")
            index = self.child.expect(patterns + [errors, pexpect.EOF, pexpect.TIMEOUT], timeout=min(0.2, remaining))
            if index < len(patterns):
                return index
            if index == len(patterns):
                raise RuntimeError(f"SONIC safety error: {self.child.after}")
            if index == len(patterns) + 1:
                raise RuntimeError("SONIC exited before execution completed")

    def _bootstrap_interactive_runtime(self) -> None:
        """Start one persistent controller and enter native joystick mode."""

        isaac_ready = self._wait_for_isaac_ready(
            timeout=float(os.getenv("SONIC_INTERACTIVE_STARTUP_TIMEOUT_S", "600"))
        )
        standing_motion_id = self._standing_motion_id()
        artifact = self.exchange / standing_motion_id
        runtime_request = {
            "schema_version": 1,
            "state": "STARTING",
            "request_id": "interactive-bootstrap",
            "motion_id": standing_motion_id,
            "isaac_session_id": isaac_ready["session_id"],
            "asset_profile": self.required_asset_profile,
            "reference_contract": self.required_reference_contract,
            "interactive_source": "unitree_wireless_remote",
            "approved_epoch_s": time.time(),
        }
        self._write_runtime_request(runtime_request)
        log = self.runtime_dir / "sonic_persistent.log"
        self._spawn_controller(
            standing_motion_id,
            artifact=artifact,
            log=log,
            planner_enabled=True,
        )
        self._expect_or_abort(["Init Done"], timeout=30)
        self._select_loaded_motion(standing_motion_id)
        self.child.send("]")
        self._expect_or_abort([r"transitioning to CONTROL state"], timeout=10)
        self.control_started = True
        self._enter_joystick_locomotion(
            runtime_request, isaac_ready["session_id"], timeout=180.0
        )
        print(
            "[sonic-supervisor] INTERACTIVE joystick runtime ready "
            f"session={isaac_ready['session_id']} standing={standing_motion_id}",
            flush=True,
        )

    def _wait_for_isaac_ready(self, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        last_error = "Isaac runtime has not reported readiness"
        while time.monotonic() < deadline:
            try:
                status = self._require_isaac_ready()
                if float(status.get("updated_epoch_s", 0.0)) < self.started_epoch_s:
                    raise RuntimeError(
                        "Isaac readiness belongs to a session heartbeat from "
                        "before this supervisor started"
                    )
                return status
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(1.0)
        raise RuntimeError(
            f"timed out waiting for Isaac runtime startup: {last_error}"
        )

    def _standing_motion_id(self) -> str:
        preferred = os.getenv("SONIC_STANDING_MOTION_ID", "isaac-neutral-v1")
        candidates = [preferred]
        candidates.extend(
            artifact.name
            for artifact in sorted(self.exchange.iterdir(), key=lambda path: path.name)
            if artifact.is_dir() and not artifact.name.startswith(".")
        )
        seen = set()
        for motion_id in candidates:
            if motion_id in seen:
                continue
            seen.add(motion_id)
            artifact = self.exchange / motion_id
            manifest_path = artifact / "manifest.json"
            try:
                manifest = json.loads(manifest_path.read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                continue
            if (
                manifest.get("execution_contract", {}).get("asset")
                != self.required_reference_contract
            ):
                continue
            if validate_artifact(artifact).valid:
                return motion_id
        raise RuntimeError(
            "interactive runtime needs one validated standing-compatible motion; "
            f"preferred={preferred}"
        )

    def _spawn_controller(
        self,
        required_motion_id: str,
        *,
        artifact: Path,
        log: Path,
        planner_enabled: bool,
    ) -> Path:
        executable = self.sonic_root / "target/release/g1_deploy_onnx_ref"
        if not executable.is_file():
            raise RuntimeError(f"SONIC executable is missing: {executable}")
        if self.persistent_process:
            selection = self._prepare_persistent_reference_pool(required_motion_id)
        else:
            selection_root = self.exchange / ".sonic-selection"
            selection_root.mkdir(exist_ok=True)
            selection = Path(tempfile.mkdtemp(prefix="run-", dir=selection_root))
            prepare_sonic_reference_view(artifact, selection / required_motion_id)
        command_values = [
            executable,
            os.getenv("SONIC_INTERFACE", "lo"),
            self.sonic_root / "policy/release/model_decoder.onnx",
            selection,
            "--obs-config",
            self.sonic_root / "policy/release/observation_config.yaml",
            "--encoder-file",
            self.sonic_root / "policy/release/model_encoder.onnx",
        ]
        if planner_enabled:
            command_values.extend(
                (
                    "--planner-file",
                    self.sonic_root / "planner/target_vel/V2/planner_sonic.onnx",
                )
            )
        command_values.extend(
            (
                "--input-type",
                "manager" if planner_enabled else "keyboard",
                "--output-type",
                os.getenv("SONIC_OUTPUT_TYPE", "all"),
                "--zmq-host",
                "localhost",
            )
        )
        self.child_log_handle = log.open(
            "a" if self.persistent_process else "w", buffering=1
        )
        child_env = os.environ.copy()
        child_env.setdefault("SONIC_SIM_HISTORY_WARMUP_TICKS", "10")
        self.child = pexpect.spawn(
            str(command_values[0]),
            [str(value) for value in command_values[1:]],
            encoding="utf-8",
            timeout=180,
            env=child_env,
        )
        self.child.logfile = self.child_log_handle
        loaded_names = []
        while True:
            index = self._expect_or_abort(
                [
                    r"✓ Loaded ([^\r\n]+) \(",
                    r"\[DEBUG\] G1Deploy object created successfully!",
                ],
                timeout=120,
            )
            if index == 1:
                break
            loaded_names.append(self.child.match.group(1).strip())
        self.loaded_motion_indexes = {
            name: index for index, name in enumerate(loaded_names)
        }
        self.current_motion_index = 0
        self.control_started = False
        self.runtime_mode = "REFERENCE"
        return selection

    def _select_loaded_motion(self, motion_id: str) -> None:
        if self.child is None:
            raise RuntimeError("SONIC process was not started")
        if motion_id not in self.loaded_motion_indexes:
            raise RuntimeError(f"SONIC did not preload approved motion: {motion_id}")
        target_index = self.loaded_motion_indexes[motion_id]
        motion_count = len(self.loaded_motion_indexes)
        forward = (target_index - self.current_motion_index) % motion_count
        backward = (self.current_motion_index - target_index) % motion_count
        key, count = ("N", forward) if forward <= backward else ("P", backward)
        # Interface safety reset may leave current_motion pointing at a
        # temporary planner snapshot even though current_motion_index still
        # names this target. U asks the keyboard interface to materialize the
        # indexed motion directly, avoiding a transient adjacent reference.
        if count == 0:
            self.child.send("U")
            self._expect_or_abort(
                [rf"Materialized motion .* : {re_escape(motion_id)} at frame 0"],
                timeout=5,
            )
        else:
            self.child.send("R")
            for _ in range(count):
                self.child.send(key)
                time.sleep(0.03)
        self.current_motion_index = target_index

    def _signal_runtime_mode(self, requested_signal: signal.Signals) -> None:
        if self.child is None or not self.child.isalive():
            raise RuntimeError("SONIC process was not started")
        os.kill(self.child.pid, requested_signal)

    def _play_standing_reference(self) -> None:
        standing_motion_id = self._standing_motion_id()
        manifest = json.loads(
            (self.exchange / standing_motion_id / "manifest.json").read_text()
        )
        frames = int(manifest["num_frames"])
        fps = float(manifest["fps"])
        self._select_loaded_motion(standing_motion_id)
        self.child.send("T")
        self._expect_or_abort(
            [rf"Playing motion .*\({frames} total frames\)"], timeout=10
        )
        self._expect_or_abort(
            [rf"Motion index: .* : {re_escape(standing_motion_id)} completed\."],
            timeout=motion_completion_timeout_s(frames, fps, None),
        )

    def _wait_or_abort(self, duration: float, watch_child: bool = False) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.abort_event.wait(timeout=min(0.1, max(0.0, deadline - time.monotonic()))):
                raise RuntimeError("execution aborted")
            runtime_failure = self._runtime_failure()
            if runtime_failure is not None:
                raise RuntimeError(f"Isaac runtime failure: {runtime_failure}")
            if watch_child and self.child is not None:
                try:
                    self._expect_or_abort([r"$^"], timeout=0.01)
                except RuntimeError as exc:
                    if "timed out waiting" not in str(exc):
                        raise

    def _stop(self) -> None:
        child = self.child
        if child is not None:
            try:
                alive = child.isalive()
            except Exception:
                # Abort handling and the execution thread can both observe the
                # same child exit. pexpect's waitpid state is not idempotent.
                alive = False
            if alive:
                try:
                    child.kill(signal.SIGTERM)
                    child.close(force=True)
                except Exception:
                    try:
                        child.kill(signal.SIGKILL)
                    except Exception:
                        pass
        self.child = None
        if self.child_log_handle is not None:
            try:
                self.child_log_handle.close()
            except Exception:
                pass
        self.child_log_handle = None
        self.loaded_motion_indexes = {}
        self.current_motion_index = 0
        self.control_started = False
        self.runtime_mode = "STOPPED"

    def _enter_joystick_locomotion(
        self, runtime_request: dict, session_id: str, timeout: float = 180.0
    ) -> None:
        """Hand LowCmd generation to the native Unitree gamepad planner."""

        if self.child is None or not self.child.isalive():
            raise RuntimeError("cannot enter joystick mode without a live SONIC process")
        # SIGUSR2 asks InterfaceManager to switch delegates, perform its
        # safety reset, and request planner activation after that reset has
        # been consumed by Gamepad::update().
        self._signal_runtime_mode(signal.SIGUSR2)
        self._expect_or_abort(
            [r"\[InterfaceManager\] Runtime mode: JOYSTICK_PLANNER"],
            timeout=5,
        )
        self._expect_or_abort([r"\[Gamepad\] Runtime joystick standby: ready"], timeout=5)
        self.runtime_mode = "JOYSTICK_LOCOMOTION"
        runtime_request["state"] = "INTERACTIVE"
        runtime_request["interactive_source"] = "unitree_wireless_remote"
        runtime_request["interactive_epoch_s"] = time.time()
        self._write_runtime_request(runtime_request)
        self._wait_for_isaac_runtime_mode(session_id, "INTERACTIVE", timeout=timeout)

    def _wait_for_isaac_runtime_mode(
        self, session_id: str, desired_state: str, timeout: float
    ) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self._read_isaac_status()
            if status.get("session_id") != session_id:
                raise RuntimeError("Isaac session changed during runtime mode switch")
            if status.get("state") == desired_state:
                return status
            if status.get("state") in TERMINAL_FAILURE_STATES:
                raise RuntimeError(
                    str(status.get("reason") or f"Isaac state={status.get('state')}")
                )
            time.sleep(0.1)
        raise RuntimeError(f"timed out waiting for Isaac state={desired_state}")

    def _wait_for_stable_standing(
        self,
        session_id: str,
        *,
        stable_duration: float,
        timeout: float,
        accepted_states: set[str] | None = None,
    ) -> dict:
        if accepted_states is None:
            accepted_states = {"READY_STANDING"}
        deadline = time.monotonic() + timeout
        max_joint_velocity = float(
            os.getenv("SONIC_PRE_PLANNER_MAX_JOINT_VELOCITY", "0.9")
        )
        stable_since = None
        latest = None
        while time.monotonic() < deadline:
            latest = self._read_isaac_status()
            if latest.get("session_id") != session_id:
                raise RuntimeError("Isaac session changed while stabilizing neutral stand")
            if latest.get("state") in TERMINAL_FAILURE_STATES:
                raise RuntimeError(
                    str(latest.get("reason") or f"Isaac state={latest.get('state')}")
                )
            try:
                root_height = float(latest.get("root_height_m"))
                root_tilt = float(latest.get("root_tilt_rad"))
                max_dq = float(latest.get("max_joint_velocity_rad_s"))
            except (TypeError, ValueError):
                quiet = False
            else:
                quiet = bool(
                    latest.get("state") in accepted_states
                    and 0.70 <= root_height <= 0.90
                    and root_tilt <= 0.10
                    and max_dq <= max_joint_velocity
                )
            if quiet:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= stable_duration:
                    return latest
            else:
                stable_since = None
            time.sleep(0.1)
        raise RuntimeError(
            "neutral reference did not reach stable standing before planner takeover"
        )

    def _prepare_persistent_reference_pool(self, required_motion_id: str) -> Path:
        """Build a stable SONIC view containing every validated compatible motion."""

        selection_root = self.exchange / ".sonic-selection"
        selection_root.mkdir(exist_ok=True)
        selection = selection_root / "persistent"
        shutil.rmtree(selection, ignore_errors=True)
        selection.mkdir()
        included = []
        for artifact in sorted(self.exchange.iterdir(), key=lambda value: value.name):
            if not artifact.is_dir() or artifact.name.startswith("."):
                continue
            validation_path = artifact / "validation.json"
            manifest_path = artifact / "manifest.json"
            try:
                validation = json.loads(validation_path.read_text())
                manifest = json.loads(manifest_path.read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                continue
            if not validation.get("valid", False):
                continue
            if (
                manifest.get("execution_contract", {}).get("asset")
                != self.required_reference_contract
            ):
                continue
            prepare_sonic_reference_view(artifact, selection / artifact.name)
            included.append(artifact.name)
        if required_motion_id not in included:
            raise RuntimeError(
                f"approved motion was not eligible for SONIC preload: {required_motion_id}"
            )
        return selection


def prepare_sonic_reference_view(artifact: Path, destination: Path) -> None:
    """Create a validated, immutable SONIC view without changing joint values.

    SONIC's C++ deployment code passes ``joint_pos.csv`` directly into the
    policy observation.  The values are absolute IsaacLab-order joint
    positions, not offsets from the trained default pose.
    """
    destination.mkdir()
    for source in artifact.iterdir():
        if source.name == "joint_pos.csv":
            continue
        (destination / source.name).symlink_to(source)

    with (artifact / "joint_pos.csv").open(newline="") as source_handle:
        reader = csv.reader(source_handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ProtocolError("joint_pos.csv is empty") from exc
        if len(header) != 29:
            raise ProtocolError("joint_pos.csv must have exactly 29 columns")
        frame_count = 0
        for frame_index, row in enumerate(reader):
            if len(row) != 29:
                raise ProtocolError(
                    f"joint_pos.csv frame {frame_index} must have exactly 29 columns"
                )
            try:
                values = [float(value) for value in row]
            except ValueError as exc:
                raise ProtocolError(
                    f"joint_pos.csv frame {frame_index} contains a non-numeric value"
                ) from exc
            if not all(math.isfinite(value) for value in values):
                raise ProtocolError(
                    f"joint_pos.csv frame {frame_index} contains a non-finite value"
                )
            frame_count += 1
        if frame_count == 0:
            raise ProtocolError("joint_pos.csv contains no reference frames")

    # Symlink the already-validated artifact so there is no second, converted
    # trajectory whose semantics can diverge from the approved reference.
    (destination / "joint_pos.csv").symlink_to(artifact / "joint_pos.csv")


def _ids(raw: str) -> tuple[str, str]:
    try:
        value = json.loads(raw)
        return str(value.get("request_id", "unknown")), str(value.get("motion_id", "unknown"))
    except Exception:
        return "unknown", "unknown"


def re_escape(value: str) -> str:
    """Regex-escape a protocol identifier used in pexpect patterns."""
    import re

    return re.escape(value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exchange", type=Path, default=Path(os.getenv("MOTION_EXCHANGE", "/motion_exchange")))
    parser.add_argument("--sonic-root", type=Path, default=Path(os.getenv("SONIC_ROOT", "/sonic/gear_sonic_deploy")))
    parser.add_argument("--dds-domain", type=int, default=int(os.getenv("DDS_DOMAIN", "0")))
    parser.add_argument("--dds-interface", default=os.getenv("DDS_INTERFACE") or None)
    args = parser.parse_args()
    SonicSupervisor(args.exchange, args.sonic_root, JsonDDS(args.dds_domain, args.dds_interface)).run()


if __name__ == "__main__":
    main()
