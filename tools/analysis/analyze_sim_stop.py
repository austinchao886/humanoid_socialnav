"""Analyze read-only DDS stop captures using LowState ticks for joint dq."""
import argparse
import json
from pathlib import Path
import struct
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('capture', type=Path)
args = parser.parse_args()
rows = [json.loads(line) for line in args.capture.open()]
states = [r for r in rows if r['kind'] == 'state']
commands = [r for r in rows if r['kind'] == 'command']
stops = []
previous_moving = False
for r in states:
    _, buttons, lx, rx, ry, l2, ly, _ = struct.unpack('<2sH5f16s', bytes(r['remote']))
    moving = bool(buttons & (1 << 7)) and np.hypot(lx, ly) > .05
    if previous_moving and not moving:
        stops.append({'epoch_s': r['epoch_s'], 'kind': 'center' if buttons & (1 << 7) else 'release'})
    if moving and not previous_moving and stops:
        stops[-1]['next_motion_epoch_s'] = r['epoch_s']
    previous_moving = moving
for stop in stops:
    stop['windows'] = []
    # A diagnostic joint-speed-only settling measure, not the full root gate.
    stop['joint_speed_settled_after_wall_s'] = None
    below_since = None
    previous_epoch = None
    for row in states:
        if not stop['epoch_s'] <= row['epoch_s'] < min(stop['epoch_s']+20, stop.get('next_motion_epoch_s', float('inf'))):
            continue
        if previous_epoch is not None and row['epoch_s']-previous_epoch > .2:
            below_since = None
        previous_epoch = row['epoch_s']
        if all(np.isfinite(row['dq'])) and max(abs(v) for v in row['dq']) <= .9:
            if below_since is None:
                below_since = row['epoch_s']
            if row['epoch_s']-below_since >= 3:
                stop['joint_speed_settled_after_wall_s'] = row['epoch_s']-stop['epoch_s']
                break
        else:
            below_since = None
    for a, b in [(0, 2), (2, 5), (5, 10), (10, 20)]:
        start, end = stop['epoch_s']+a, stop['epoch_s']+b
        if end > min(stop.get('next_motion_epoch_s', float('inf')), states[-1]['epoch_s']):
            continue
        selected = [r for r in states if start <= r['epoch_s'] < end]
        if len(selected) < 3:
            continue
        ticks = np.array([r['tick'] for r in selected], dtype=np.int64)
        # Exclude duplicates and session/tick resets from derivative estimates.
        mask = np.diff(ticks)>0
        if not mask.all():
            raise ValueError('capture window has duplicate/non-monotonic ticks')
        q = np.array([r['q'] for r in selected])
        dq = np.array([r['dq'] for r in selected])
        target = np.array([r['q'] for r in commands if start <= r['epoch_s'] < end])
        if not np.isfinite(q).all() or not np.isfinite(dq).all() or not np.isfinite(target).all():
            raise ValueError('non-finite joint measurements')
        position_dq = np.diff(q, axis=0)/(np.diff(ticks)*.005)[:, None]
        physical_rms = np.sqrt(np.mean(position_dq**2, axis=0))
        top = np.argsort(np.mean(dq*dq, axis=0))[-4:][::-1]
        stop['windows'].append({'wall_window_s': [a,b], 'samples':len(selected),
          'max_dq_p95': float(np.quantile(np.max(np.abs(dq), axis=1), .95)),
          'tick_gap_max':int(np.max(np.diff(ticks))),
          'max_position_dq_rms':float(physical_rms.max()),
          'physical_peak_joint':int(np.argmax(physical_rms)),
          'max_position_range_rad':float(np.ptp(q, axis=0).max()),
          'joints':[{'index':int(j),'dq_rms':float(np.sqrt(np.mean(dq[:,j]**2))),
                     'dq_mean':float(dq[:,j].mean()),'q_range':float(np.ptp(q[:,j])),
                     'position_dq_rms':float(np.sqrt(np.mean(position_dq[:,j]**2))),
                     'target_range':float(np.ptp(target[:,j])) if len(target) else None} for j in top]})
print(json.dumps({'capture':str(args.capture),'states':len(states),'commands':len(commands),'stops':stops},indent=2))
