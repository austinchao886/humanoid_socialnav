"""Simulation-only comparison of stick-centering and deadman-release stops.

Does not change control parameters. Requires no connected PS4 client and a
fresh unsupported INTERACTIVE runtime. Reports unique simulator observations,
not independent 50 Hz measurements. All durations are wall-clock seconds.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time

repo = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(repo))
from motion_pipeline.joystick import JoystickCommand, encode_line

runtime = repo.parent / 'motion_exchange' / '.runtime'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--walk-seconds', type=int, nargs='+', choices=[3, 4, 5], default=[3, 4, 5])
parser.add_argument('--forward', type=float, choices=[.5, 1.0], default=.5,
                    help='Normalized stick input; 1.0 requests the existing 0.45 m/s cap.')
args = parser.parse_args()
if len(args.walk_seconds)>3 or len(set(args.walk_seconds))!=len(args.walk_seconds):
    parser.error('choose each walking duration at most once')
output = repo / '.build' / time.strftime('stop-transients-%Y%m%dT%H%M%S.json')
report = {'completed': False, 'normalized_forward': args.forward,
          'scope': 'Same-session command delivery only; inspect window metrics for stopping quality.',
          'trials': []}
session = None

def status():
    d = json.loads((runtime / 'isaac_status.json').read_text())
    request = json.loads((runtime / 'request.json').read_text())
    assert 0 <= time.time()-d['updated_epoch_s'] < 3, 'stale status'
    assert d['state'] == request['state'] == 'INTERACTIVE', 'not joystick-owned INTERACTIVE'
    assert d['elastic_support_scale'] == d['elastic_support_attitude_scale'] == 0, 'support active'
    assert session is None or d['session_id'] == session, 'session changed'
    return d

def summarize(samples):
    windows = []
    for start, end in [(0, 2), (2, 5), (5, 10), (10, 20)]:
        rows = [r for r in samples if start <= r['elapsed_wall_s'] < end]
        if not rows:
            continue
        windows.append({'wall_window_s': [start, end], 'samples': len(rows),
                        **{f'max_{key}': max(r[key] for r in rows)
                           for key in ['max_joint_velocity_rad_s', 'root_tilt_rad', 'planar_speed_m_s']}})
    return windows

try:
    assert not any('16042' in line and 'ESTAB' in line for line in
                   subprocess.check_output(['ss', '-tn'], text=True).splitlines()), 'Disconnect PS4 client'
    session = status()['session_id']
    report['session_id'] = session
    print('REPORT', output, 'SESSION', session, flush=True)
    with socket.create_connection(('127.0.0.1', 16042), timeout=5) as sock:
        seq = 0
        def drive(seconds, deadman, forward, samples=None):
            global seq
            start = time.monotonic()
            next_check = 0
            last_sim = None
            while time.monotonic()-start < seconds:
                sock.sendall(encode_line(JoystickCommand(seq, deadman, False, 0, -forward, 0, 0)))
                seq += 1
                now = time.monotonic()
                if now >= next_check:
                    d = status()
                    next_check = now + .05
                    if samples is not None and d['simulation_time_s'] != last_sim:
                        last_sim = d['simulation_time_s']
                        samples.append({'elapsed_wall_s': now-start, 'epoch_s': time.time(),
                                        'planar_speed_m_s': math.hypot(*d['root_linear_velocity_m_s'][:2]),
                                        **{k: d[k] for k in ['simulation_time_s', 'trace_path',
                                            'max_joint_velocity_rad_s', 'root_tilt_rad',
                                            'root_angular_velocity_rad_s']}})
                time.sleep(.02)
        try:
            drive(10, False, 0)
            # Reverse the order in the middle pair to reduce simple order bias.
            for duration, modes in [(3, ['release', 'center']), (4, ['center', 'release']), (5, ['release', 'center'])]:
                if duration not in args.walk_seconds:
                    continue
                for mode in modes:
                    drive(duration, True, args.forward)
                    trial = {'stop': mode, 'walk_wall_s': duration, 'before_stop': status(), 'samples': []}
                    report['trials'].append(trial)
                    trial['stop_command_epoch_s'] = time.time()
                    drive(20, mode == 'center', 0, trial['samples'])
                    trial['windows'] = summarize(trial['samples'])
                    print(json.dumps({k:v for k,v in trial.items() if k not in ['samples', 'before_stop']}), flush=True)
                    output.write_text(json.dumps(report, indent=2)+'\n')
                    drive(3, False, 0)
        finally:
            sock.sendall(encode_line(JoystickCommand(seq, False, False, 0, 0, 0, 0)))
    report['completed'] = True
except Exception as error:
    report['error'] = f'{type(error).__name__}: {error}'
    raise
finally:
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
