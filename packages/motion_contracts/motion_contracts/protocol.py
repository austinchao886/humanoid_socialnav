from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any

SCHEMA_VERSION = 1
GENERATE_TOPIC = "rt/motion/generate/cmd"
VIDEO_GENERATE_TOPIC = "rt/motion/generate/video/cmd"
CONTROL_TOPIC = "rt/motion/control/cmd"
STATUS_TOPIC = "rt/motion/status"


class State(str, Enum):
    RECEIVED = "RECEIVED"
    GENERATING = "GENERATING"
    VALIDATING = "VALIDATING"
    READY_FOR_APPROVAL = "READY_FOR_APPROVAL"
    EXECUTING = "EXECUTING"
    COMPLETED = "COMPLETED"
    REJECTED = "REJECTED"
    INVALID_REFERENCE = "INVALID_REFERENCE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class GenerateCommand:
    request_id: str
    prompt: str
    seed: int
    duration_s: float
    schema_version: int = SCHEMA_VERSION

    @classmethod
    def parse(cls, raw: str) -> "GenerateCommand":
        data = _object(raw)
        _version(data)
        allowed = {"schema_version", "request_id", "prompt", "seed", "duration_s"}
        _unknown(data, allowed)
        try:
            cmd = cls(
                request_id=_nonempty(data["request_id"], "request_id"),
                prompt=_nonempty(data["prompt"], "prompt"),
                seed=int(data["seed"]),
                duration_s=float(data["duration_s"]),
            )
        except KeyError as exc:
            raise ProtocolError(f"missing field: {exc.args[0]}") from exc
        if not 1.0 <= cmd.duration_s <= 10.0:
            raise ProtocolError("duration_s must be in [1.0, 10.0]")
        if len(cmd.prompt) > 500:
            raise ProtocolError("prompt exceeds 500 characters")
        return cmd


@dataclass(frozen=True)
class VideoGenerateCommand:
    request_id: str
    video_path: str
    video_sha256: str
    original_fps: float
    model: str = "gem"
    model_version: str = "unknown"
    static_camera: bool = True
    schema_version: int = SCHEMA_VERSION

    @classmethod
    def parse(cls, raw: str) -> "VideoGenerateCommand":
        data = _object(raw)
        _version(data)
        _unknown(data, {"schema_version", "request_id", "video_path", "video_sha256",
                        "original_fps", "model", "model_version", "static_camera"})
        try:
            cmd = cls(
                request_id=_nonempty(data["request_id"], "request_id"),
                video_path=_nonempty(data["video_path"], "video_path"),
                video_sha256=_nonempty(data["video_sha256"], "video_sha256").lower(),
                original_fps=float(data["original_fps"]),
                model=_nonempty(data.get("model", "gem"), "model").lower(),
                model_version=_nonempty(data["model_version"], "model_version"),
                static_camera=data.get("static_camera", True),
            )
        except KeyError as exc:
            raise ProtocolError(f"missing field: {exc.args[0]}") from exc
        if cmd.model not in {"gem", "gvhmr"}:
            raise ProtocolError("model must be gem or gvhmr")
        if len(cmd.video_sha256) != 64 or any(c not in "0123456789abcdef" for c in cmd.video_sha256):
            raise ProtocolError("video_sha256 must be a lowercase SHA-256 digest")
        if not 1.0 <= cmd.original_fps <= 240.0:
            raise ProtocolError("original_fps must be in [1, 240]")
        if not isinstance(cmd.static_camera, bool):
            raise ProtocolError("static_camera must be a boolean")
        return cmd


@dataclass(frozen=True)
class ControlCommand:
    request_id: str
    motion_id: str
    action: str
    schema_version: int = SCHEMA_VERSION

    @classmethod
    def parse(cls, raw: str) -> "ControlCommand":
        data = _object(raw)
        _version(data)
        allowed = {"schema_version", "request_id", "motion_id", "action"}
        _unknown(data, allowed)
        try:
            cmd = cls(
                request_id=_nonempty(data["request_id"], "request_id"),
                motion_id=_nonempty(data["motion_id"], "motion_id"),
                action=_nonempty(data["action"], "action"),
            )
        except KeyError as exc:
            raise ProtocolError(f"missing field: {exc.args[0]}") from exc
        if cmd.action not in {"approve_execute", "reject", "abort", "reset"}:
            raise ProtocolError(f"unknown action: {cmd.action}")
        return cmd


def status_json(request_id: str, state: State, **kwargs: Any) -> str:
    payload = {"schema_version": SCHEMA_VERSION, "request_id": request_id, "state": state.value, **kwargs}
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def to_json(command: GenerateCommand | VideoGenerateCommand | ControlCommand) -> str:
    return json.dumps(asdict(command), separators=(",", ":"), sort_keys=True)


def _object(raw: str) -> dict[str, Any]:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc.msg}") from exc
    if not isinstance(data, dict):
        raise ProtocolError("payload must be a JSON object")
    return data


def _version(data: dict[str, Any]) -> None:
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ProtocolError(f"schema_version must be {SCHEMA_VERSION}")


def _unknown(data: dict[str, Any], allowed: set[str]) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ProtocolError(f"unknown fields: {', '.join(unknown)}")


def _nonempty(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError(f"{name} must be a non-empty string")
    return value.strip()
