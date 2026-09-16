"""Conservative experiment envelope; not a replacement for native safety.

Height, vertical velocity and tilt use the runner's unsupported-ground gate.
Planar .5 m/s bounds the initial slow-walk experiment (command cap .45 m/s).
Torque ratio must remain within the existing actuator limits. The 1.5-second
status bound permits the current 1-Hz status publisher; native protections
and private-channel freshness checks remain responsible for fast failures.
"""
import math


def require_composition_envelope(status, *, now_epoch_s, gait_window=False):
    def scalar(name):
        value=status.get(name)
        if type(value) not in (int,float) or not math.isfinite(value):
            raise RuntimeError(f"invalid composition measurement: {name}")
        return value
    age=now_epoch_s-scalar("updated_epoch_s")
    if not math.isfinite(age) or not 0<=age<=1.5:
        raise RuntimeError("composition status is stale or future-dated")
    if not .70<=scalar("root_height_m")<=.90:
        raise RuntimeError("composition root height outside experiment envelope")
    if not 0<=scalar("root_tilt_rad")<=(.20 if gait_window else .25):
        raise RuntimeError("composition root tilt outside experiment envelope")
    velocity=status.get("root_linear_velocity_m_s")
    if (not isinstance(velocity,(list,tuple)) or len(velocity)!=3
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in velocity)):
        raise RuntimeError("invalid composition root velocity")
    if abs(velocity[2])>(.20 if gait_window else .15) or math.hypot(*velocity[:2])>(.65 if gait_window else .5):
        raise RuntimeError("composition root velocity outside experiment envelope")
    if not 0<=scalar("max_torque_limit_ratio")<=1.:
        raise RuntimeError("composition torque exceeds actuator envelope")
    if gait_window:
        require_gait_window(status)


def require_gait_window(status):
    """Opt-in simulation experiment, NOT a certified dynamical envelope.

    Baseline instantaneous planar/vertical peaks .556/.155 exceeded the old
    command-derived gate; half-second mean path speed stayed below .444.
    Keep mean<=.5, bound transients<=.65/.20, and tighten whole-window tilt
    to .20. These margins are experimental and must be tested, not called safe
    merely because a baseline fits. Native/controller protections stay active.
    """
    w=status.get('kinematic_window')
    if not isinstance(w,dict) or type(w.get('schema_version')) is not int or w.get('schema_version')!=1 or w.get('ready') is not True or w.get('error') is not None:
        raise RuntimeError('gait requires a complete physics-rate window')
    if not status.get('session_id') or w.get('session_id')!=status['session_id']:
        raise RuntimeError('gait window session mismatch')
    def number(key):
        v=w.get(key)
        if type(v) not in (int,float) or not math.isfinite(v):
            raise RuntimeError('invalid gait window: '+key)
        return v
    count=w.get('sample_count')
    if type(count) is not int or not 190<=count<=210:
        raise RuntimeError('invalid gait sample count')
    span=number('end_simulation_s')-number('start_simulation_s')
    current=status.get('simulation_time_s')
    if type(current) not in (int,float) or not math.isfinite(current):
        raise RuntimeError('invalid status simulation time')
    if (not .995-1e-8<=span<=1.005 or abs(span-number('span_s'))>1e-8
            or not 0<number('max_sample_gap_s')<=.006
            or not -1e-8<=current-number('end_simulation_s')<=.01+1e-8
            or number('mean_window_s')!=.5):
        raise RuntimeError('gait window cadence or freshness mismatch')
    for key,lo,hi in (('mean_planar_speed_m_s',0.,.5),('peak_planar_speed_m_s',0.,.65),
                      ('peak_abs_vertical_speed_m_s',0.,.20),('min_height_m',.70,.90),
                      ('max_height_m',.70,.90),('peak_tilt_rad',0.,.20),('peak_torque_ratio',0.,1.)):
        if not lo<=number(key)<=hi:raise RuntimeError('gait window outside experiment envelope: '+key)
    if number('min_height_m')>number('max_height_m') or number('mean_planar_speed_m_s')>number('peak_planar_speed_m_s')+1e-8:
        raise RuntimeError('inconsistent gait window extrema')
