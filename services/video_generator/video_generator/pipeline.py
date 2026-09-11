from __future__ import annotations

import hashlib
import json
import os
import shlex
import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


class VideoPipelineError(RuntimeError):
    pass


@dataclass(frozen=True)
class VideoInfo:
    duration_s: float
    fps: float
    width: int
    height: int
    codec: str
    sha256: str


def inspect_video(path: Path) -> VideoInfo:
    if path.suffix.lower() not in {".mp4", ".mov"}:
        raise VideoPipelineError("video must be an MP4 or MOV file")
    if not path.is_file() or path.stat().st_size == 0:
        raise VideoPipelineError(f"video is missing or empty: {path}")
    command = [
        os.getenv("FFPROBE", "ffprobe"), "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=codec_name,width,height,avg_frame_rate:format=duration",
        "-of", "json", str(path),
    ]
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
        value = json.loads(result.stdout)
        stream = value["streams"][0]
        numerator, denominator = stream["avg_frame_rate"].split("/", 1)
        fps = float(numerator) / float(denominator)
        duration = float(value["format"]["duration"])
    except (subprocess.SubprocessError, KeyError, ValueError, ZeroDivisionError, json.JSONDecodeError) as exc:
        raise VideoPipelineError(f"unable to inspect video: {exc}") from exc
    if not 3.0 <= duration <= 10.0:
        raise VideoPipelineError(f"video duration must be in [3, 10] seconds; got {duration:.3f}")
    if fps <= 0 or int(stream["width"]) <= 0 or int(stream["height"]) <= 0:
        raise VideoPipelineError("video has invalid frame rate or dimensions")
    return VideoInfo(duration, fps, int(stream["width"]), int(stream["height"]),
                     str(stream.get("codec_name", "unknown")), _sha256(path))


def normalize_video(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    command = [
        os.getenv("FFMPEG", "ffmpeg"), "-nostdin", "-v", "error", "-y", "-i", str(source),
        "-map", "0:v:0", "-vf", "fps=30", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(destination),
    ]
    try:
        subprocess.run(command, check=True, timeout=180)
    except subprocess.SubprocessError as exc:
        raise VideoPipelineError(f"video normalization failed: {exc}") from exc


class OfflineVideoPipeline:
    """Run isolated pose-estimation and retarget adapters as subprocesses."""

    def __init__(self, model_commands: dict[str, str], retarget_command: str, *,
                 model_versions: dict[str, str] | None = None,
                 retarget_version: str = "unknown"):
        self.model_commands = model_commands
        self.retarget_command = retarget_command
        self.model_versions = model_versions or {}
        self.retarget_version = retarget_version

    def run(self, source: Path, work: Path, *, model: str,
            static_camera: bool) -> tuple[Path, dict]:
        info = inspect_video(source)
        original = work / f"source{source.suffix.lower()}"
        shutil.copy2(source, original)
        normalized = work / "normalized.mp4"
        normalize_video(original, normalized)
        canonical = work / "canonical_smpl.npz"
        tracking = work / "tracking_report.json"
        overlay = work / "smpl_overlay.mp4"
        world = work / "smpl_world.mp4"
        command = self.model_commands.get(model)
        if not command:
            raise VideoPipelineError(f"{model} adapter is not configured")
        self._run_adapter(command, {
            "video": normalized, "canonical": canonical, "tracking": tracking,
            "overlay": overlay, "world": world,
            "static_camera": "1" if static_camera else "0",
        }, timeout=1200)
        _validate_tracking_report(tracking)
        _validate_canonical_smpl(canonical)
        robot = work / "g1_motion.npz"
        preview = work / "retarget_preview.mp4"
        self._run_adapter(self.retarget_command, {
            "canonical": canonical, "robot": robot, "preview": preview,
        }, timeout=600)
        qpos_csv, source_fps, contact_metrics = _validate_and_export_robot_motion(
            robot, work / "g1_qpos.csv", work / "foot_contacts.csv")
        metadata = {
            "video": asdict(info), "model": model,
            "model_version": self.model_versions.get(model, "unknown"),
            "retargeter": "gmr", "retargeter_version": self.retarget_version,
            "static_camera": static_camera,
            "capture_assumptions": {"camera": "fixed_monocular", "single_person": True,
                                    "full_body_visible": True, "feet_visible": True},
            "canonical_smpl_schema": 1,
            "tracking_report": json.loads(tracking.read_text()),
            "source_fps": source_fps,
            "contact_metrics": contact_metrics,
            "conditioning": {
                "target_fps": 50.0, "ground_alignment": "gmr_adapter",
                "foot_contact_correction": "gmr_adapter",
                "neutral_transition": "artifact_converter",
                "joint_smoothing": "artifact_converter_gaussian_0.75_source_frames",
                "limits": "artifact_validator_and_gmr_adapter",
            },
        }
        return qpos_csv, metadata

    @staticmethod
    def _run_adapter(template: str, values: dict[str, object], *, timeout: int) -> None:
        if not template.strip():
            raise VideoPipelineError("adapter command is not configured")
        argv = [part.format_map({key: str(value) for key, value in values.items()})
                for part in shlex.split(template)]
        try:
            subprocess.run(argv, check=True, timeout=timeout)
        except (subprocess.SubprocessError, OSError, KeyError) as exc:
            raise VideoPipelineError(f"adapter failed ({argv[0] if argv else 'empty command'}): {exc}") from exc


def _validate_tracking_report(path: Path) -> None:
    try:
        report = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise VideoPipelineError(f"model did not produce a valid tracking report: {exc}") from exc
    checks = (
        (report.get("person_count") == 1, "video must contain exactly one tracked person"),
        (float(report.get("single_person_frame_fraction", 0)) >= 0.95,
         "exactly one person must be detected in at least 95% of frames"),
        (float(report.get("full_body_visible_fraction", 0)) >= 0.95,
         "full body must be visible in at least 95% of frames"),
        (float(report.get("feet_visible_fraction", 0)) >= 0.95,
         "both feet must be visible in at least 95% of frames"),
        (float(report.get("longest_tracking_gap_s", 999)) <= 0.2,
         "person tracking gap exceeds 0.2 seconds"),
    )
    for passed, message in checks:
        if not passed:
            raise VideoPipelineError(message)


def _validate_canonical_smpl(path: Path) -> None:
    try:
        with np.load(path, allow_pickle=False) as data:
            required = {"global_orient", "body_pose", "transl", "betas", "joints_world",
                        "camera_intrinsics", "incam_global_orient", "incam_body_pose",
                        "incam_transl", "fps"}
            missing = required - set(data.files)
            if missing:
                raise VideoPipelineError(f"canonical SMPL is missing: {', '.join(sorted(missing))}")
            frames = len(data["global_orient"])
            expected = {
                "global_orient": (frames, 3), "body_pose": (frames, 63),
                "transl": (frames, 3), "joints_world": (frames, 24, 3),
                "camera_intrinsics": (frames, 3, 3), "incam_global_orient": (frames, 3),
                "incam_body_pose": (frames, 63), "incam_transl": (frames, 3),
            }
            for name, shape in expected.items():
                if data[name].shape != shape:
                    raise VideoPipelineError(f"canonical SMPL {name} has invalid shape")
            if data["betas"].shape not in {(10,), (frames, 10)}:
                raise VideoPipelineError("canonical SMPL betas must be [10] or [frames,10]")
            if frames < 2 or not all(np.isfinite(data[name]).all() for name in required - {"fps"}):
                raise VideoPipelineError("canonical SMPL contains too few frames or non-finite values")
    except OSError as exc:
        raise VideoPipelineError(f"model did not produce canonical SMPL: {exc}") from exc


def _validate_and_export_robot_motion(path: Path, output: Path,
                                      contacts_output: Path) -> tuple[Path, float, dict]:
    try:
        with np.load(path, allow_pickle=False) as data:
            qpos = np.asarray(data["qpos"], dtype=np.float64)
            fps = float(np.asarray(data["fps"]).reshape(-1)[0])
            contacts = np.asarray(data["foot_contacts"], dtype=np.uint8)
            raw_metrics = np.asarray(data["contact_metrics"], dtype=np.float64)
    except (OSError, KeyError, ValueError) as exc:
        raise VideoPipelineError(f"retargeter did not produce valid G1 motion: {exc}") from exc
    if qpos.ndim != 2 or qpos.shape[1] != 36 or len(qpos) < 2 or not np.isfinite(qpos).all():
        raise VideoPipelineError("G1 retarget output qpos must be finite [frames, 36]")
    if not 1.0 <= fps <= 240.0:
        raise VideoPipelineError(f"invalid retarget frame rate: {fps}")
    if contacts.shape != (len(qpos), 2) or not np.isin(contacts, (0, 1)).all():
        raise VideoPipelineError("foot contacts must be binary [frames, 2]")
    if raw_metrics.shape != (4,) or not np.isfinite(raw_metrics).all():
        raise VideoPipelineError("retarget contact metrics must contain four finite values")
    np.savetxt(output, qpos, delimiter=",")
    np.savetxt(contacts_output, contacts, delimiter=",", fmt="%d",
               header="left_contact,right_contact", comments="")
    metrics = {"min_ankle_height_m": float(raw_metrics[0]),
               "max_contact_foot_speed_m_s": float(raw_metrics[1]),
               "left_contact_fraction": float(raw_metrics[2]),
               "right_contact_fraction": float(raw_metrics[3])}
    return output, fps, metrics


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
