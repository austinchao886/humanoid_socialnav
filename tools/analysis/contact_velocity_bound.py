"""Rigid collision-point planar speed lower bound, NOT a slip classifier.

For any point within R of the measured link origin, v_p = v_o + omega x r.
Thus ||v_p_xy|| >= max(0, ||v_o_xy|| - ||omega|| R). R must bound the
actual collision geometry in metres. Net force does not identify contact point
or distinguish speculative contact, impact and stance. No acceptance pass here.
"""
import math


def point_speed_lower_bound(linear, angular, radius_m):
    for value in (linear, angular):
        if (not isinstance(value, (list, tuple)) or len(value) != 3 or
                any(type(x) not in (float, int) or not math.isfinite(x) for x in value)):
            raise ValueError('finite three-vector required')
    if type(radius_m) not in (float, int) or not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError('positive audited collision radius required')
    return max(0., math.hypot(*linear[:2])-math.hypot(*angular)*radius_m)


def contact_summary(samples, radius_m, force_threshold_n):
    """Samples are (simulation_time, 16-column trace foot row), contiguous 200Hz.

    Integral is a lower bound on material-point travel over adjacent qualified
    samples, not net foot displacement or measured ground slip distance.
    """
    if not math.isfinite(force_threshold_n) or force_threshold_n <= 0:
        raise ValueError('invalid force threshold')
    values=[];integral=0.;elapsed=0.;previous=None
    for stamp,row in samples:
        if (not math.isfinite(stamp) or len(row)!=16 or
                any(not math.isfinite(x) for x in row)):
            raise ValueError('invalid foot trace row')
        bound=point_speed_lower_bound(row[7:10],row[10:13],radius_m)
        active=row[15]>=force_threshold_n
        if previous is not None:
            t0,b0,a0=previous;dt=stamp-t0
            if not 0<dt<=.006:raise ValueError('trace gap or nonmonotonic time')
            if active and a0:
                integral+=(b0+bound)*dt/2
                elapsed+=dt
        if active:values.append(bound)
        previous=(stamp,bound,active)
    values.sort()
    return dict(force_threshold_n=force_threshold_n,qualified_samples=len(values),
                adjacent_qualified_time_s=elapsed,point_speed_lower_bound_max_m_s=max(values,default=None),
                point_speed_lower_bound_p95_m_s=values[int(.95*(len(values)-1))] if values else None,
                material_point_travel_lower_bound_m=integral,
                scope='rigid_point_bound_not_ground_slip_measurement')
