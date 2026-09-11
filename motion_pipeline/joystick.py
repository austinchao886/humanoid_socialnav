"""Validated joystick protocol shared by the Mac client and simulator bridge."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import struct
from typing import Any


PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 4096
DEFAULT_STALE_AFTER_S = 0.2


@dataclass(frozen=True)
class JoystickCommand:
    sequence: int
    deadman: bool
    emergency_stop: bool
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    l2: float = 0.0
    r2: float = 0.0

    def to_wire_dict(self) -> dict[str, Any]:
        return {"version": PROTOCOL_VERSION, **asdict(self)}


def _axis(payload: dict[str, Any], name: str, *, trigger: bool = False) -> float:
    value = float(payload.get(name, 0.0))
    lower = 0.0 if trigger else -1.0
    if not math.isfinite(value) or not lower <= value <= 1.0:
        raise ValueError(f"{name} must be finite and in [{lower}, 1.0]")
    return value


def parse_command(payload: dict[str, Any]) -> JoystickCommand:
    if int(payload.get("version", -1)) != PROTOCOL_VERSION:
        raise ValueError("unsupported joystick protocol version")
    sequence = int(payload["sequence"])
    if sequence < 0:
        raise ValueError("sequence must be non-negative")
    return JoystickCommand(
        sequence=sequence,
        deadman=bool(payload.get("deadman", False)),
        emergency_stop=bool(payload.get("emergency_stop", False)),
        left_x=_axis(payload, "left_x"),
        left_y=_axis(payload, "left_y"),
        right_x=_axis(payload, "right_x"),
        right_y=_axis(payload, "right_y"),
        l2=_axis(payload, "l2", trigger=True),
        r2=_axis(payload, "r2", trigger=True),
    )


def decode_line(line: bytes) -> JoystickCommand:
    if not line or len(line) > MAX_FRAME_BYTES:
        raise ValueError("invalid joystick frame size")
    payload = json.loads(line.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("joystick frame must be a JSON object")
    return parse_command(payload)


def encode_line(command: JoystickCommand) -> bytes:
    return (json.dumps(command.to_wire_dict(), separators=(",", ":")) + "\n").encode()


def encode_unitree_remote(command: JoystickCommand | None) -> bytes:
    """Encode logical controls as the 40-byte Unitree wireless remote packet."""
    if command is None:
        return bytes(40)
    buttons = 0
    if command.deadman:
        buttons |= 1 << 7  # F2: locomotion deadman
    if command.emergency_stop:
        buttons |= 1 << 3  # Select: hard stop
    if command.l2 > 0.5:
        buttons |= 1 << 5
    if command.r2 > 0.5:
        buttons |= 1 << 4
    # SDL reports up as negative Y; Unitree's forward direction is positive ly.
    return struct.pack(
        "<2sH5f16s",
        b"\x00\x00",
        buttons,
        command.left_x,
        command.right_x,
        -command.right_y,
        command.l2,
        -command.left_y,
        bytes(16),
    )
