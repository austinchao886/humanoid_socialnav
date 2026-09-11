"""Fixed-size, read-only visualization snapshots for a separate Isaac viewer.

The authoritative simulator writes one snapshot alongside each 50 Hz LowState
sample.  The viewer only reads this file and never participates in DDS control.
"""

from __future__ import annotations

import math
import mmap
import os
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


MAGIC = b"G1VSTATE"
VERSION = 1
JOINT_COUNT = 29
ROOT_POSE_COUNT = 7
HEADER = struct.Struct("<8sI4xQQQ")
PAYLOAD = struct.Struct(f"<{ROOT_POSE_COUNT + 2 * JOINT_COUNT}f")
FILE_SIZE = HEADER.size + PAYLOAD.size
SEQUENCE_OFFSET = 16


@dataclass(frozen=True)
class VisualStateSnapshot:
    sequence: int
    written_monotonic_ns: int
    simulation_step: int
    root_pose_wxyz: tuple[float, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]

    @property
    def age_s(self) -> float:
        return max(0.0, (time.monotonic_ns() - self.written_monotonic_ns) / 1.0e9)


def _finite_values(values: Sequence[float], expected: int, label: str) -> tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != expected:
        raise ValueError(f"{label} must contain {expected} values, got {len(converted)}")
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} contains a non-finite value")
    return converted


class VisualStateWriter:
    """Single-writer mmap using an even/odd sequence lock."""

    def __init__(self, path: str | os.PathLike[str]):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w+b")
        self._handle.truncate(FILE_SIZE)
        self._mapping = mmap.mmap(self._handle.fileno(), FILE_SIZE, access=mmap.ACCESS_WRITE)
        HEADER.pack_into(self._mapping, 0, MAGIC, VERSION, 0, 0, 0)
        self._sequence = 0

    def write(
        self,
        *,
        simulation_step: int,
        root_pose_wxyz: Sequence[float],
        joint_positions: Sequence[float],
        joint_velocities: Sequence[float],
    ) -> int:
        root_pose = _finite_values(root_pose_wxyz, ROOT_POSE_COUNT, "root pose")
        positions = _finite_values(joint_positions, JOINT_COUNT, "joint positions")
        velocities = _finite_values(joint_velocities, JOINT_COUNT, "joint velocities")
        quaternion_norm = math.sqrt(sum(value * value for value in root_pose[3:7]))
        if not 0.99 <= quaternion_norm <= 1.01:
            raise ValueError(f"root quaternion norm is invalid: {quaternion_norm:.6f}")

        odd_sequence = self._sequence + 1
        even_sequence = odd_sequence + 1
        struct.pack_into("<Q", self._mapping, SEQUENCE_OFFSET, odd_sequence)
        struct.pack_into("<Q", self._mapping, 24, time.monotonic_ns())
        struct.pack_into("<Q", self._mapping, 32, max(0, int(simulation_step)))
        PAYLOAD.pack_into(self._mapping, HEADER.size, *(root_pose + positions + velocities))
        struct.pack_into("<Q", self._mapping, SEQUENCE_OFFSET, even_sequence)
        self._sequence = even_sequence
        return even_sequence

    def close(self) -> None:
        mapping = getattr(self, "_mapping", None)
        if mapping is not None:
            mapping.close()
            self._mapping = None
        handle = getattr(self, "_handle", None)
        if handle is not None:
            handle.close()
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.close()


class VisualStateReader:
    """Non-blocking reader that returns only internally consistent snapshots."""

    def __init__(self, path: str | os.PathLike[str]):
        self.path = Path(path)
        self._handle = None
        self._mapping = None

    def _open_if_ready(self) -> bool:
        if self._mapping is not None:
            return True
        try:
            if self.path.stat().st_size != FILE_SIZE:
                return False
            self._handle = self.path.open("rb")
            self._mapping = mmap.mmap(
                self._handle.fileno(), FILE_SIZE, access=mmap.ACCESS_READ
            )
            return True
        except (FileNotFoundError, OSError):
            self.close()
            return False

    def read(self) -> VisualStateSnapshot | None:
        if not self._open_if_ready():
            return None
        try:
            sequence_before = struct.unpack_from("<Q", self._mapping, SEQUENCE_OFFSET)[0]
            if sequence_before == 0 or sequence_before & 1:
                return None
            magic, version, sequence, written_ns, simulation_step = HEADER.unpack_from(
                self._mapping, 0
            )
            values = PAYLOAD.unpack_from(self._mapping, HEADER.size)
            sequence_after = struct.unpack_from("<Q", self._mapping, SEQUENCE_OFFSET)[0]
        except (BufferError, OSError, ValueError):
            self.close()
            return None
        if (
            magic != MAGIC
            or version != VERSION
            or sequence != sequence_before
            or sequence_after != sequence_before
            or sequence_after & 1
        ):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        root_pose = tuple(values[:ROOT_POSE_COUNT])
        quaternion_norm = math.sqrt(sum(value * value for value in root_pose[3:7]))
        if not 0.99 <= quaternion_norm <= 1.01:
            return None
        joint_start = ROOT_POSE_COUNT
        velocity_start = joint_start + JOINT_COUNT
        return VisualStateSnapshot(
            sequence=sequence,
            written_monotonic_ns=written_ns,
            simulation_step=simulation_step,
            root_pose_wxyz=root_pose,
            joint_positions=tuple(values[joint_start:velocity_start]),
            joint_velocities=tuple(values[velocity_start:]),
        )

    def close(self) -> None:
        mapping = self._mapping
        self._mapping = None
        if mapping is not None:
            mapping.close()
        handle = self._handle
        self._handle = None
        if handle is not None:
            handle.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.close()
