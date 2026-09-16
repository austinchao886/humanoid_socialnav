"""Supervisor-owned immutable preparation cache; never an approval authority.

Execution callers must still run artifact validation and live safety checks.
Lookup verifies exact source bytes without re-parsing joint CSVs.
"""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import threading

from motion_contracts.protocol import ControlCommand, ProtocolError
from .prepared_gesture import prepare_gesture
from .gesture_runtime import GesturePlan

SOURCE_FILES = ("manifest.json", "validation.json", "joint_pos.csv", "joint_vel.csv")


def source_identity(artifact):
    digest = hashlib.sha256()
    for name in SOURCE_FILES:
        value = (artifact / name).read_bytes()
        digest.update(name.encode() + b"\0" + len(value).to_bytes(8, "big") + value)
    return digest.hexdigest()


@dataclass(frozen=True)
class PreparedEntry:
    plan: GesturePlan
    artifact: Path
    request_id: str
    motion_id: str
    reference_contract: str
    source_identity: str


class PreparedPlanStore:
    def __init__(self, exchange: Path, *, capacity=16):
        if not isinstance(capacity, int) or isinstance(capacity, bool) or capacity < 1:
            raise ValueError("capacity must be a positive integer")
        self.exchange = exchange.resolve()
        self.capacity = capacity
        self.entries = {}
        self.lock = threading.Lock()

    def _artifact(self, motion_id):
        path = (self.exchange / motion_id).resolve()
        if path.parent != self.exchange or not path.is_dir() or path.name != motion_id:
            raise ProtocolError("unknown or escaping motion_id")
        return path

    def prepare(self, motion_id, *, amplitude, time_scale,
                start_offset_s, end_offset_s, entry_s, exit_s):
        artifact = self._artifact(motion_id)
        before = source_identity(artifact)
        manifest = json.loads((artifact / "manifest.json").read_bytes())
        if manifest.get("motion_id") != motion_id:
            raise ProtocolError("source motion identity mismatch")
        request_id = manifest.get("request_id")
        if not isinstance(request_id, str) or not request_id.strip():
            raise ProtocolError("missing source request identity")
        contract = manifest.get("execution_contract", {}).get("asset")
        if contract != "sonic_official_g1":
            raise ProtocolError("unsupported reference contract")
        gesture = prepare_gesture(artifact, amplitude=amplitude, time_scale=time_scale)
        plan = GesturePlan(gesture, start_offset_s, end_offset_s, entry_s, exit_s)
        if source_identity(artifact) != before:
            raise ProtocolError("source changed during preparation")
        entry = PreparedEntry(plan, artifact, request_id, motion_id, contract, before)
        with self.lock:
            if plan.plan_id not in self.entries and len(self.entries) >= self.capacity:
                raise ProtocolError("prepared plan cache full; explicit discard required")
            self.entries[plan.plan_id] = entry
        return entry

    def resolve(self, command: ControlCommand, *, required_reference_contract):
        if command.action != "approve_execute" or command.prepared_plan_id is None:
            raise ProtocolError("explicit prepared-plan approval required")
        with self.lock:
            entry = self.entries.get(command.prepared_plan_id)
        if entry is None:
            raise ProtocolError("prepared plan unavailable; prepare before approval")
        if (entry.request_id != command.request_id or entry.motion_id != command.motion_id
                or entry.reference_contract != required_reference_contract):
            raise ProtocolError("approval owner, motion or reference contract mismatch")
        if (self._artifact(command.motion_id) != entry.artifact
                or source_identity(entry.artifact) != entry.source_identity):
            raise ProtocolError("prepared source is stale; prepare and approve again")
        return entry.plan

    def discard(self, plan_id):
        with self.lock:
            self.entries.pop(plan_id, None)
