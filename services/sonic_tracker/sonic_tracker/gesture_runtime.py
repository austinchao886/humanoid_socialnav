"""Reference-only gesture lifecycle; supervisor owns authorization and safety.

Not a DDS endpoint. A caller must validate approval and live state before creating
this runtime. A session mismatch raises an error for the supervisor's safe path;
it must never be handled by silently reconnecting or replaying approval.
"""
from dataclasses import dataclass
import hashlib
import json
import math

from .composer import JointReference, compose_right_arm, transition_weight
from .prepared_gesture import PreparedGesture


@dataclass(frozen=True)
class GesturePlan:
    gesture: PreparedGesture
    start_offset_s: float
    end_offset_s: float
    entry_s: float
    exit_s: float

    def __post_init__(self):
        values = (self.start_offset_s, self.end_offset_s, self.entry_s, self.exit_s)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("nonfinite plan")
        if not 0 <= self.start_offset_s < self.end_offset_s <= self.gesture.duration_s:
            raise ValueError("invalid prepared-clip segment")
        if min(self.entry_s, self.exit_s) <= 0 or self.entry_s+self.exit_s > self.duration_s:
            raise ValueError("entry/exit must fit within the segment")

    @property
    def duration_s(self):
        return self.end_offset_s-self.start_offset_s

    @property
    def plan_id(self):
        description = dict(schema=1, content_id=self.gesture.content_id,
                           start=self.start_offset_s, end=self.end_offset_s,
                           entry=self.entry_s, exit=self.exit_s, mask="right_arm")
        return hashlib.sha256(json.dumps(description, sort_keys=True).encode()).hexdigest()


class GestureRuntime:
    def __init__(self, plan: GesturePlan, *, approved_plan_id: str,
                 session_id: str, start_simulation_s: float):
        if approved_plan_id != plan.plan_id:
            raise ValueError("approval does not match exact gesture plan")
        if not session_id or not math.isfinite(start_simulation_s):
            raise ValueError("invalid session/start time")
        self.plan = plan
        self.session_id = session_id
        self.start_s = start_simulation_s
        self.last_s = start_simulation_s
        self.cancel_s = None

    def cancel(self, simulation_s: float):
        if not math.isfinite(simulation_s) or simulation_s < self.last_s:
            raise ValueError("cancel time precedes last sample")
        if self.cancel_s is None:
            self.cancel_s = simulation_s  # repeated requests must not restart fade

    def envelope(self, simulation_s: float):
        elapsed = simulation_s-self.start_s
        incoming, incoming_rate = transition_weight(elapsed, self.plan.entry_s)
        outgoing, outgoing_rate = transition_weight(
            elapsed-(self.plan.duration_s-self.plan.exit_s), self.plan.exit_s)
        weight = incoming*(1-outgoing)
        rate = incoming_rate*(1-outgoing)-incoming*outgoing_rate
        if self.cancel_s is not None:
            fade, fade_rate = transition_weight(simulation_s-self.cancel_s, self.plan.exit_s)
            rate = rate*(1-fade)-weight*fade_rate
            weight *= 1-fade
        # Multiplication/subtraction may round an almost-endpoint to exactly
        # zero/one. Keep the representable envelope and derivative consistent.
        return weight, 0.0 if weight in (0.0, 1.0) else rate

    def sample(self, base: JointReference, *, session_id: str) -> JointReference:
        return self.sample_window((base,), session_id=session_id)[0]

    def sample_window(self, bases: tuple[JointReference, ...], *,
                      session_id: str) -> tuple[JointReference, ...]:
        """Compose a planner horizon atomically; only its origin advances time.

        Caller supplies actual planner reference samples, not replicated current
        robot measurements. This method does not extrapolate missing base data.
        Previewing a horizon must not consume it or prevent a next-tick cancel.
        The native adapter must also preserve root/contact reference channels.
        """
        if session_id != self.session_id:
            raise ValueError("session changed: discard this execution")
        if not bases:
            raise ValueError("empty reference window")
        if bases[0].time_s < self.last_s:
            raise ValueError("simulation clock regressed")
        if any(b.time_s <= a.time_s for a, b in zip(bases, bases[1:])):
            raise ValueError("reference window must have increasing timestamps")
        results = tuple(self._compose(base) for base in bases)
        self.last_s = bases[0].time_s
        return results

    def _compose(self, base: JointReference) -> JointReference:
        weight, rate = self.envelope(base.time_s)
        if weight == 0 and rate == 0:
            return base
        elapsed = base.time_s-self.start_s+self.plan.start_offset_s
        gesture = self.plan.gesture.sample(elapsed, base.time_s)
        return compose_right_arm(base, gesture, weight, rate)
