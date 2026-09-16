"""Immutable, content-addressed gesture preparation; never grants execution approval."""
from dataclasses import dataclass
import csv
import hashlib
import io
import json
import math
from pathlib import Path

from .composer import JointReference, RIGHT_ARM, JOINT_ORDER


@dataclass(frozen=True)
class PreparedGesture:
    content_id: str
    source_motion_id: str
    fps: float
    amplitude: float
    time_scale: float
    positions: tuple
    velocities: tuple

    @property
    def duration_s(self):
        return (len(self.positions)-1) / self.fps * self.time_scale

    def sample(self, elapsed_s: float, simulation_time_s: float) -> JointReference:
        """Cubic Hermite interpolation with analytic dq; outside clip holds endpoints.

        Time scaling slows motion. Arm amplitude scales about the source first
        frame, not about zero. Caller combines with live planner using composer.
        """
        q, dq = self._sample_joints(elapsed_s, range(29))
        return JointReference(q, dq, simulation_time_s)

    def sample_right_arm(self, elapsed_s: float):
        """Same Hermite reference, computing only the seven transmitted joints."""
        return self._sample_joints(elapsed_s, RIGHT_ARM)

    def _sample_joints(self, elapsed_s, joints):
        if not math.isfinite(elapsed_s):
            raise ValueError("invalid elapsed time")
        coordinate = max(0., min(len(self.positions)-1., elapsed_s/self.time_scale*self.fps))
        index = min(int(coordinate), len(self.positions)-2)
        u = coordinate-index
        dt = 1/self.fps
        # Shared polynomial basis, once per sample instead of once per joint.
        h0, h1 = 2*u**3-3*u*u+1, (u**3-2*u*u+u)*dt
        h2, h3 = -2*u**3+3*u*u, (u**3-u*u)*dt
        d0, d1 = (6*u*u-6*u)/dt, 3*u*u-4*u+1
        d2, d3 = (-6*u*u+6*u)/dt, 3*u*u-2*u
        outside = elapsed_s < 0 or elapsed_s > self.duration_s
        q, dq = [], []
        for j in joints:
            a, b = self.positions[index][j], self.positions[index+1][j]
            va, vb = self.velocities[index][j], self.velocities[index+1][j]
            value = h0*a + h1*va + h2*b + h3*vb
            speed = (d0*a + d1*va + d2*b + d3*vb)/self.time_scale
            if outside:
                speed = 0.
            scale = self.amplitude if j in RIGHT_ARM else 1.
            q.append(self.positions[0][j] + scale*(value-self.positions[0][j]))
            dq.append(scale*speed)
        return tuple(q), tuple(dq)


def prepare_gesture(artifact: Path, *, amplitude: float, time_scale: float) -> PreparedGesture:
    if not math.isfinite(amplitude) or not 0 < amplitude <= 1:
        raise ValueError("amplitude must be in (0,1]")
    if not math.isfinite(time_scale) or time_scale < 1:
        raise ValueError("time_scale must be >=1; preparation must not accelerate source")
    names = ("manifest.json", "validation.json", "joint_pos.csv", "joint_vel.csv")
    blobs = {name: (Path(artifact)/name).read_bytes() for name in names}
    manifest = json.loads(blobs["manifest.json"])
    validation = json.loads(blobs["validation.json"])
    if validation.get("valid") is not True or validation.get("errors"):
        raise ValueError("source validation failed")
    if manifest.get("joint_order") != JOINT_ORDER:
        raise ValueError("unsupported joint order")
    fps = manifest.get("fps")
    if isinstance(fps, bool) or not isinstance(fps, (float,int)) or not math.isfinite(fps) or fps <= 0:
        raise ValueError("invalid fps")
    motion_id = manifest.get("motion_id")
    if not isinstance(motion_id, str) or not motion_id:
        raise ValueError("missing source identity")
    arrays = []
    for name in names[2:]:
        reader = csv.reader(io.StringIO(blobs[name].decode()))
        prefix = "joint_vel" if name == "joint_vel.csv" else "joint"
        if next(reader, []) != [f"{prefix}_{i}" for i in range(29)]:
            raise ValueError("invalid joint column order")
        values = tuple(tuple(float(v) for v in row) for row in reader)
        if len(values) < 2 or len(values) != manifest.get("num_frames"):
            raise ValueError("frame count mismatch")
        if any(len(row) != 29 or not all(math.isfinite(v) for v in row) for row in values):
            raise ValueError("invalid joint samples")
        arrays.append(values)
    if any(abs(v) > 1e-6 for row in (arrays[1][0], arrays[1][-1]) for v in row):
        raise ValueError("source must have zero-velocity endpoint holds")
    # Hash exact bytes plus transformation settings; approval.json is deliberately
    # excluded: preparation neither inherits nor creates execution authorization.
    digest = hashlib.sha256()
    for name in names:
        digest.update(name.encode()+b"\0"+len(blobs[name]).to_bytes(8,"big")+blobs[name])
    digest.update(json.dumps({"schema":1,"amplitude":amplitude,"time_scale":time_scale}, sort_keys=True).encode())
    return PreparedGesture(digest.hexdigest(), motion_id, float(fps), amplitude,
                           time_scale, arrays[0], arrays[1])
