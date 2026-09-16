"""Build a bounded native gesture buffer; does not publish or authorize it."""
import math
from .composer import RIGHT_ARM
from .gesture_runtime import GestureRuntime

PHYSICS_DT = .005
FRAME_COUNT = 256


def build_snapshot(runtime: GestureRuntime, *, session_id: str, origin_tick: int,
                   origin_simulation_s: float, sequence: int) -> dict:
    if session_id != runtime.session_id:
        raise ValueError("session mismatch")
    if type(origin_tick) is not int or not 0 <= origin_tick <= 2**32-1:
        raise ValueError("invalid simulation tick")
    if type(sequence) is not int or not 0 <= sequence < 2**53:
        raise ValueError("invalid sequence")
    if not math.isfinite(origin_simulation_s) or origin_simulation_s < runtime.last_s:
        raise ValueError("invalid/regressing snapshot time")
    frames=[]
    for index in range(FRAME_COUNT):
        simulation_s=origin_simulation_s+index*PHYSICS_DT
        weight,rate=runtime.envelope(simulation_s)
        q,dq=runtime.plan.gesture.sample_right_arm(
            simulation_s-runtime.start_s+runtime.plan.start_offset_s)
        frames.append([*q,*dq,weight,rate])
    return dict(schema_version=1,session_id=session_id,plan_id=runtime.plan.plan_id,
                sequence=sequence,origin_sim_tick=origin_tick,physics_dt_s=PHYSICS_DT,
                joint_indices=list(RIGHT_ARM),frames=frames)
