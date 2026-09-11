from __future__ import annotations

import argparse
import json
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path

from motion_contracts.artifact import convert_kimodo_qpos
from motion_contracts.protocol import GENERATE_TOPIC, STATUS_TOPIC, GenerateCommand, ProtocolError, State, status_json
from motion_contracts.validator import validate_artifact
from motion_pipeline.core.latency import (
    ResourceLock,
    initialize_timing,
    record_stage,
    timed_stage,
    timing_summary,
    utc_now,
)
from motion_pipeline.runtime.dds_transport import JsonDDS


class GeneratorService:
    def __init__(self, exchange: Path, model: str, dds: JsonDDS):
        self.exchange = exchange
        self.model = model
        self.dds = dds
        self.queue: queue.Queue[tuple[str, float, str]] = queue.Queue()
        self.seen: set[str] = set()

    def on_generate(self, raw: str) -> None:
        self.queue.put((raw, time.monotonic(), utc_now()))

    def run(self) -> None:
        self.exchange.mkdir(parents=True, exist_ok=True)
        self.dds.subscribe(GENERATE_TOPIC, self.on_generate)
        while True:
            raw, received_monotonic, received_at = self.queue.get()
            dequeued_monotonic = time.monotonic()
            try:
                command = GenerateCommand.parse(raw)
                if command.request_id in self.seen:
                    raise ProtocolError(f"duplicate request_id: {command.request_id}")
                self.seen.add(command.request_id)
                self._generate(
                    command,
                    received_monotonic=received_monotonic,
                    received_at=received_at,
                    queue_wait_s=dequeued_monotonic - received_monotonic,
                )
            except Exception as exc:
                request_id = _request_id(raw)
                self.dds.publish(STATUS_TOPIC, status_json(request_id, State.FAILED, error={"message": str(exc)}))

    def _generate(
        self,
        command: GenerateCommand,
        *,
        received_monotonic: float,
        received_at: str,
        queue_wait_s: float,
    ) -> None:
        motion_id = f"{command.request_id}-{uuid.uuid4().hex[:8]}"
        self.dds.publish(
            STATUS_TOPIC,
            status_json(
                command.request_id,
                State.RECEIVED,
                motion_id=motion_id,
                timing={
                    "server_received_at": received_at,
                    "queue_wait_s": round(queue_wait_s, 6),
                },
            ),
        )
        with tempfile.TemporaryDirectory(dir=self.exchange, prefix=".generating-") as temp:
            staging = Path(temp)
            output = staging / "kimodo"
            self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.GENERATING, motion_id=motion_id))
            generation_started_at = utc_now()
            gpu_lock = ResourceLock(self.exchange / ".runtime/gpu1.lock").acquire()
            generation_started = time.monotonic()
            try:
                subprocess.run(
                    [
                        "kimodo_gen",
                        command.prompt,
                        "--model",
                        self.model,
                        "--duration",
                        str(command.duration_s),
                        "--seed",
                        str(command.seed),
                        "--num_samples",
                        "1",
                        "--output",
                        str(output),
                    ],
                    check=True,
                    timeout=600,
                )
            finally:
                gpu_lock.release()
            generation_duration_s = time.monotonic() - generation_started
            artifact_staging = staging / motion_id
            conversion_started_at = utc_now()
            conversion_started = time.monotonic()
            manifest = convert_kimodo_qpos(
                output.with_suffix(".csv"),
                artifact_staging,
                request_id=command.request_id,
                motion_id=motion_id,
                prompt=command.prompt,
                seed=command.seed,
                model=self.model,
            )
            conversion_duration_s = time.monotonic() - conversion_started
            timing_path = artifact_staging / "timing.json"
            initialize_timing(timing_path, command.request_id, motion_id)
            record_stage(
                timing_path,
                "gpu1_resource_queue_wait",
                started_at=gpu_lock.wait_started_at,
                duration_s=gpu_lock.wait_s,
                excluded=True,
                details={"resource": "gpu1", "owner": "kimodo"},
            )
            record_stage(
                timing_path,
                "dds_generate_to_received",
                started_at=received_at,
                duration_s=queue_wait_s,
                details={
                    "measurement": "server_callback_to_received_status",
                    "client_dispatch_not_in_command_schema": True,
                },
            )
            record_stage(
                timing_path,
                "request_queue_wait",
                started_at=received_at,
                duration_s=queue_wait_s,
            )
            record_stage(
                timing_path,
                "kimodo_generation",
                started_at=generation_started_at,
                duration_s=generation_duration_s,
                details={"model": self.model, "warm_runtime": True},
            )
            record_stage(
                timing_path,
                "reference_conversion_fk",
                started_at=conversion_started_at,
                duration_s=conversion_duration_s,
                details={"kinematics_source": manifest.get("kinematics_source")},
            )
            if manifest.get("kinematics_source") != "mujoco_fk":
                raise RuntimeError("MuJoCo FK was unavailable; refusing non-executable root-only reference")
            source_npz = output.with_suffix(".npz")
            if source_npz.exists():
                shutil.copy2(source_npz, artifact_staging / "kimodo_source.npz")
                from motion_contracts.artifact import render_artifact_preview

                with timed_stage(timing_path, "preview_rendering"):
                    render_artifact_preview(
                        artifact_staging,
                        artifact_staging / "preview.mp4",
                        fps=float(manifest["fps"]),
                    )
            else:
                record_stage(
                    timing_path,
                    "preview_rendering",
                    duration_s=0.0,
                    details={"skipped": True, "reason": "kimodo source NPZ missing"},
                )
            self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.VALIDATING, motion_id=motion_id))
            validation_started_at = utc_now()
            validation_started = time.monotonic()
            result = validate_artifact(artifact_staging)
            validation_finished = time.monotonic()
            record_stage(
                timing_path,
                "validation",
                started_at=validation_started_at,
                duration_s=validation_finished - validation_started,
                details={"valid": result.valid},
            )
            final = self.exchange / motion_id
            os.rename(artifact_staging, final)
            final_timing_path = final / "timing.json"
            record_stage(
                final_timing_path,
                "validation_to_ready",
                started_at=utc_now(),
                duration_s=time.monotonic() - validation_finished,
                details={"terminal_state": (
                    State.READY_FOR_APPROVAL.value
                    if result.valid
                    else State.INVALID_REFERENCE.value
                )},
            )
            timing = timing_summary(final_timing_path)
            if not result.valid:
                self.dds.publish(
                    STATUS_TOPIC,
                    status_json(
                        command.request_id,
                        State.INVALID_REFERENCE,
                        motion_id=motion_id,
                        artifact_path=str(final),
                        validation=result.__dict__,
                        timing=timing,
                    ),
                )
                return
            self.dds.publish(
                STATUS_TOPIC,
                status_json(
                    command.request_id,
                    State.READY_FOR_APPROVAL,
                    motion_id=motion_id,
                    artifact_path=str(final),
                    validation=result.__dict__,
                    timing=timing,
                ),
            )


def _request_id(raw: str) -> str:
    try:
        value = json.loads(raw).get("request_id", "unknown")
        return str(value)
    except Exception:
        return "unknown"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exchange", type=Path, default=Path(os.getenv("MOTION_EXCHANGE", "/motion_exchange")))
    parser.add_argument("--model", default=os.getenv("KIMODO_MODEL", "nvidia/Kimodo-G1-RP-v1"))
    parser.add_argument("--dds-domain", type=int, default=int(os.getenv("DDS_DOMAIN", "0")))
    parser.add_argument("--dds-interface", default=os.getenv("DDS_INTERFACE") or None)
    args = parser.parse_args()
    GeneratorService(args.exchange, args.model, JsonDDS(args.dds_domain, args.dds_interface)).run()


if __name__ == "__main__":
    main()
