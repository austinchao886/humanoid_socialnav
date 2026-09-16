"""Reference-space composition primitives. No transport or actuator commands.

This kernel establishes kinematic continuity, not dynamic feasibility or approval.
The caller must validate source identity, authorization, freshness and contacts.
"""
from dataclasses import dataclass
import math

JOINT_ORDER = "g1_29dof_isaaclab"
RIGHT_ARM = (12, 16, 20, 22, 24, 26, 28)


@dataclass(frozen=True)
class JointReference:
    q: tuple[float, ...]
    dq: tuple[float, ...]
    time_s: float
    joint_order: str = JOINT_ORDER

    def __post_init__(self):
        object.__setattr__(self, "q", tuple(self.q))
        object.__setattr__(self, "dq", tuple(self.dq))
        if len(self.q) != 29 or len(self.dq) != 29:
            raise ValueError("expected 29 paired joint positions and velocities")
        if self.joint_order != JOINT_ORDER:
            raise ValueError("unsupported joint order")
        if not all(math.isfinite(x) for x in (*self.q, *self.dq, self.time_s)):
            raise ValueError("nonfinite reference")


def transition_weight(elapsed_s: float, duration_s: float) -> tuple[float, float]:
    """Quintic 0→1 envelope and its time derivative (1/s).

    Zero first/second envelope derivatives at endpoints avoid an added velocity
    or acceleration step. They cannot repair discontinuities in either input.
    """
    if not math.isfinite(elapsed_s) or not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("finite elapsed time and positive duration required")
    if elapsed_s <= 0:
        return 0.0, 0.0
    if elapsed_s >= duration_s:
        return 1.0, 0.0
    u = elapsed_s / duration_s
    # Evaluate near one via the symmetric tail to avoid polynomial overshoot.
    v = min(u, 1-u)
    tail = v*v*v*(10 + v*(-15 + 6*v))
    weight = tail if u <= .5 else 1-tail
    rate = 30*u*u*(1-u)*(1-u)/duration_s
    return weight, 0.0 if weight in (0.0, 1.0) else rate


def compose_right_arm(base: JointReference, gesture: JointReference,
                      weight: float, weight_rate: float) -> JointReference:
    """Replace only the right-arm reference with a differentiable blend.

    Both inputs must be sampled at the SAME simulation time and in the same
    joint convention. Entry uses increasing weight; exit decreasing weight.
    dq includes w_dot*(q_gesture-q_base); blending dq alone is inconsistent.
    Lower body, waist and left arm remain exactly those of the base reference.
    """
    if base.time_s != gesture.time_s:
        raise ValueError("reference sample times must match")
    if not math.isfinite(weight) or not 0 <= weight <= 1 or not math.isfinite(weight_rate):
        raise ValueError("invalid blend envelope")
    if weight in (0, 1) and weight_rate != 0:
        raise ValueError("endpoint weight must have zero derivative")
    q, dq = list(base.q), list(base.dq)
    for i in RIGHT_ARM:
        q[i] = base.q[i] + weight * (gesture.q[i] - base.q[i])
        dq[i] = ((1-weight)*base.dq[i] + weight*gesture.dq[i]
                 + weight_rate*(gesture.q[i]-base.q[i]))
    return JointReference(tuple(q), tuple(dq), base.time_s)
