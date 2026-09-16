"""Diagnose failed centering trials; never issue a dynamic-quality pass.

Times are observed status bounds, not exact command or planner timestamps.
Raw LowCmd targets are NOT the planner reference. Foot force occupancy is NOT
sole slip. Exclude all samples at/after rejection (harness releases deadman).
"""
import argparse
import json
import math
from pathlib import Path


def vector(value, size):
    if (not isinstance(value, list) or len(value) != size or
            any(type(x) not in (float, int) or not math.isfinite(x) for x in value)):
        raise ValueError('invalid finite vector')
    return value


def summarize(rows):
    if len(rows) < 3:
        raise ValueError('insufficient trace coverage')
    times = [x['simulation_time_s'] for x in rows]
    dt = [b-a for a, b in zip(times, times[1:])]
    if not all(0 < x <= .006 for x in dt):
        raise ValueError('nonmonotonic or incomplete physics trace')
    target = [vector(x['desired_joint_pos_unitree_order'], 29) for x in rows]
    actual = [vector(x['joint_pos_unitree_order'], 29) for x in rows]
    velocity = [vector(x['joint_vel_unitree_order'], 29) for x in rows]
    peak_rate = [0.] * 29
    peak_step = [0.] * 29
    for a, b, interval in zip(target, target[1:], dt):
        for j in range(29):
            jump = abs(b[j]-a[j])
            peak_step[j] = max(peak_step[j], jump)
            peak_rate[j] = max(peak_rate[j], jump/interval)
    contact = []
    for x in rows:
        measurement = x['foot_contact_measurements']
        feet = measurement['rows']
        if len(feet) != 2:
            raise ValueError('expected two feet')
        contact.append(tuple(vector(f, 16)[15] >= 20 for f in feet))
    tilt = vector([x['root_tilt_rad'] for x in rows], len(rows))
    age = vector([x['lowcmd_age_s'] for x in rows], len(rows))
    if min(age) < 0:
        raise ValueError('negative command age')
    return dict(samples=len(rows), simulation_window=[times[0], times[-1]],
                max_trace_gap_s=max(dt), max_tilt_rad=max(tilt),
                peak_tilt_simulation_s=times[tilt.index(max(tilt))],
                max_lowcmd_age_s=max(age),
                raw_target_step_max_rad_by_joint=peak_step,
                sampled_target_step_over_physics_dt_max_rad_s_by_joint=peak_rate,
                actual_joint_speed_max_rad_s_by_joint=[max(abs(v[j]) for v in velocity) for j in range(29)],
                target_error_max_rad_by_joint=[max(abs(a[j]-b[j]) for a,b in zip(target,actual)) for j in range(29)],
                force_contact_sample_counts={str(n):sum(sum(c)==n for c in contact) for n in (0,1,2)},
                force_contact_state_changes=sum(a != b for a,b in zip(contact,contact[1:])),
                joint_order='unitree', contact_threshold_world_vertical_n=20)


def analyze(report, trace):
    if report.get('completed') is not False or not report.get('error'):
        raise ValueError('expected failed trial')
    events = report['events']
    if not events or events[-1]['name'] != 'decelerate' or 'after' in events[-1]:
        raise ValueError('expected incomplete deceleration')
    before, rejected = events[-1]['before'], report['last_observed_status']
    session = report['session_id']
    for s in (before, rejected):
        if s['session_id'] != session or s['state'] != 'INTERACTIVE':
            raise ValueError('session/state mismatch')
        if Path(s['trace_path']).name != trace.name:
            raise ValueError('wrong trace')
    start, end = vector([before['simulation_time_s'], rejected['simulation_time_s']], 2)
    if not start < end:
        raise ValueError('invalid failure interval')
    windows = [(start-2., start), (start, end)]
    buckets = [[], []]
    with trace.open() as stream:
        for line in stream:
            x = json.loads(line)
            t = vector([x['simulation_time_s']], 1)[0]
            if t >= end:
                break
            for (a,b), bucket in zip(windows, buckets):
                if a <= t < b:
                    bucket.append(x)
    for (a,b), rows in zip(windows,buckets):
        if not rows or rows[0]['simulation_time_s']-a > .006 or b-rows[-1]['simulation_time_s'] > .006:
            raise ValueError('missing interval boundary coverage')
    return dict(session_id=session, rejection_simulation_s=end,
                scope='failed_trial_diagnostics_not_quality_or_planner_causality',
                target_rate_note='LowCmd is sampled/held; step divided by physics dt is not commanded joint velocity or planner reference derivative.',
                pre_stop=summarize(buckets[0]), before_rejection=summarize(buckets[1]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('trace', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(json.loads(args.report.read_text()), args.trace), indent=2))
