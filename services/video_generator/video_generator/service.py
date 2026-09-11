from __future__ import annotations

import json
import math
import os
import queue
import shutil
import tempfile
import uuid
from pathlib import Path

from motion_contracts.artifact import convert_g1_qpos
from motion_contracts.protocol import STATUS_TOPIC, VIDEO_GENERATE_TOPIC, ProtocolError, State, VideoGenerateCommand, status_json
from motion_contracts.validator import validate_artifact
from motion_pipeline.runtime.dds_transport import JsonDDS
from .assets import validate_video_assets
from .pipeline import OfflineVideoPipeline, inspect_video


class VideoGeneratorService:
    """Dedicated worker for offline video requests; never consumes Kimodo work."""

    def __init__(self, exchange: Path, dds: JsonDDS):
        self.exchange = exchange
        self.dds = dds
        self.queue: queue.Queue[str] = queue.Queue()
        self.seen: set[str] = set()

    def run(self) -> None:
        validate_video_assets(full_hash=True)
        self.exchange.mkdir(parents=True, exist_ok=True)
        self.dds.subscribe(VIDEO_GENERATE_TOPIC, self.queue.put)
        while True:
            raw = self.queue.get()
            try:
                command = VideoGenerateCommand.parse(raw)
                if command.request_id in self.seen:
                    raise ProtocolError(f"duplicate request_id: {command.request_id}")
                self.seen.add(command.request_id)
                self._generate(command)
            except Exception as exc:
                self.dds.publish(STATUS_TOPIC, status_json(_request_id(raw), State.FAILED,
                                                          error={"message": str(exc)}))

    def _generate(self, command: VideoGenerateCommand) -> None:
        motion_id = f"{command.request_id}-{uuid.uuid4().hex[:8]}"
        self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.RECEIVED,
                                                  motion_id=motion_id))
        with tempfile.TemporaryDirectory(dir=self.exchange, prefix=".video-generating-") as temp:
            staging = Path(temp)
            video_work = staging / "video"
            video_work.mkdir()
            input_info = inspect_video(Path(command.video_path))
            if input_info.sha256 != command.video_sha256:
                raise RuntimeError("video checksum changed after request publication")
            if not math.isclose(input_info.fps, command.original_fps, rel_tol=0.0, abs_tol=0.01):
                raise RuntimeError("video FPS changed after request publication")
            configured_version = os.getenv(
                "GEM_MODEL_VERSION" if command.model == "gem" else "GVHMR_MODEL_VERSION", "unknown")
            if command.model_version != configured_version:
                raise RuntimeError(
                    f"requested {command.model} version {command.model_version} does not match "
                    f"worker version {configured_version}")
            pipeline = OfflineVideoPipeline(
                {"gem": os.getenv("GEM_ADAPTER_COMMAND", ""),
                 "gvhmr": os.getenv("GVHMR_ADAPTER_COMMAND", "")},
                os.getenv("GMR_ADAPTER_COMMAND", ""),
                model_versions={"gem": os.getenv("GEM_MODEL_VERSION", "unknown"),
                                "gvhmr": os.getenv("GVHMR_MODEL_VERSION", "unknown")},
                retarget_version=os.getenv("GMR_VERSION", "unknown"),
            )
            self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.GENERATING,
                                                      motion_id=motion_id))
            qpos, metadata = pipeline.run(Path(command.video_path), video_work,
                                          model=command.model,
                                          static_camera=command.static_camera)
            artifact_staging = staging / motion_id
            manifest = convert_g1_qpos(
                qpos, artifact_staging, request_id=command.request_id, motion_id=motion_id,
                source="video", model=command.model,
                source_fps=float(metadata["source_fps"]), source_metadata=metadata,
                source_sha256=str(metadata["video"]["sha256"]),
                source_contacts_csv=video_work / "foot_contacts.csv",
            )
            if manifest.get("kinematics_source") != "mujoco_fk":
                raise RuntimeError("G1 MuJoCo FK was unavailable; refusing video reference")
            for name in ("source.mp4", "source.mov", "normalized.mp4", "canonical_smpl.npz",
                         "tracking_report.json", "smpl_overlay.mp4", "smpl_world.mp4",
                         "g1_motion.npz", "retarget_preview.mp4"):
                source = video_work / name
                if source.is_file():
                    shutil.copy2(source, artifact_staging / name)
            self.dds.publish(STATUS_TOPIC, status_json(command.request_id, State.VALIDATING,
                                                      motion_id=motion_id))
            result = validate_artifact(artifact_staging)
            final = self.exchange / motion_id
            os.rename(artifact_staging, final)
            state = State.READY_FOR_APPROVAL if result.valid else State.INVALID_REFERENCE
            self.dds.publish(STATUS_TOPIC, status_json(
                command.request_id, state, motion_id=motion_id, artifact_path=str(final),
                validation=result.__dict__,
            ))


def _request_id(raw: str) -> str:
    try:
        return str(json.loads(raw).get("request_id", "unknown"))
    except Exception:
        return "unknown"


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--exchange", type=Path,
                        default=Path(os.getenv("MOTION_EXCHANGE", "/motion_exchange")))
    parser.add_argument("--dds-domain", type=int, default=int(os.getenv("DDS_DOMAIN", "0")))
    parser.add_argument("--dds-interface", default=os.getenv("DDS_INTERFACE") or None)
    args = parser.parse_args()
    VideoGeneratorService(args.exchange, JsonDDS(args.dds_domain, args.dds_interface)).run()


if __name__ == "__main__":
    main()
