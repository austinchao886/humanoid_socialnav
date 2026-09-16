"""Phase metrics for walking_gesture_trial reports; no automatic quality pass.

Observed status timestamps bound phases at 1 Hz, not exact command instants.
Contact-link origin speed is a proxy, not sole slip. Joint speed is not an
oscillation metric. A completed command sequence is not dynamic qualification.
"""
import argparse
import json
import math
from pathlib import Path


def analyze(report, trace):
    if not report.get('completed'):raise ValueError('incomplete command sequence')
    events=report['events']
    if not events:raise ValueError('missing phases')
    for e in events:
        for endpoint in ('before','after'):
            s=e[endpoint]
            if s['session_id']!=report['session_id'] or s['state']!='INTERACTIVE':
                raise ValueError('session/state mismatch')
            if Path(s['trace_path']).name!=trace.name:raise ValueError('wrong trace')
        if e['after']['simulation_time_s']<=e['before']['simulation_time_s']:
            raise ValueError('invalid phase interval')
    buckets=[[] for _ in events]
    with trace.open() as stream:
        for line in stream:
            x=json.loads(line);t=x['simulation_time_s']
            for e,rows in zip(events,buckets):
                if e['before']['simulation_time_s']<=t<e['after']['simulation_time_s']:
                    rows.append(x)
            if t>events[-1]['after']['simulation_time_s']:break
    results=[]
    for e,rows in zip(events,buckets):
        if len(rows)<3:raise ValueError('insufficient phase coverage')
        dt=[y['simulation_time_s']-x['simulation_time_s'] for x,y in zip(rows,rows[1:])]
        if min(dt)<=0:raise ValueError('nonmonotonic trace')
        planar=[math.hypot(*x['root_state_w'][7:9]) for x in rows]
        vertical=[abs(x['root_state_w'][9]) for x in rows]
        ages=[x['lowcmd_age_s'] for x in rows]
        if not all(math.isfinite(v) for v in planar+vertical+ages):raise ValueError('nonfinite trace')
        feet=[]
        for i in (0,1):
            speeds=[math.hypot(*x['foot_contact_measurements']['rows'][i][7:9])
                    for x in rows if x['foot_contact_measurements']['rows'][i][15]>=20]
            feet.append(sorted(speeds)[int(.95*(len(speeds)-1))] if speeds else None)
        c0=rows[0]['command_timing_cumulative'];c1=rows[-1]['command_timing_cumulative']
        results.append(dict(phase=e['name'],samples=len(rows),
            simulation_window=[rows[0]['simulation_time_s'],rows[-1]['simulation_time_s']],
            max_trace_gap_s=max(dt),planar_speed_max_m_s=max(planar),vertical_speed_max_m_s=max(vertical),
            max_lowcmd_age_s=max(ages),max_tilt_rad=max(x['root_tilt_rad'] for x in rows),
            max_torque_ratio=max(x['max_torque_limit_ratio'] for x in rows),
            max_joint_speed_rad_s=max(max(abs(v) for v in x['joint_vel_unitree_order']) for x in rows),
            contact_link_speed_p95_m_s=feet,
            interval_counts_over_delta=[b-a for a,b in zip(c0['interval_counts_over'],c1['interval_counts_over'])],
            planar_bound_exceeded_samples=sum(v>.5 for v in planar),
            vertical_bound_exceeded_samples=sum(v>.15 for v in vertical),
            root_displacement_xy_m=[rows[-1]['root_state_w'][i]-rows[0]['root_state_w'][i] for i in (0,1)]))
    return dict(session_id=report['session_id'],with_gesture=report['with_gesture'],
                scope='phase_diagnostics_not_dynamic_qualification',phases=results)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('report',type=Path);p.add_argument('trace',type=Path)
    args=p.parse_args()
    print(json.dumps(analyze(json.loads(args.report.read_text()),args.trace),indent=2))


if __name__=='__main__':main()
