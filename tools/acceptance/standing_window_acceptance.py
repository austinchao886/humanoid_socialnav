"""Read-only, bounded standing qualification; never commands or rearms a robot.

Run on the simulation host after motion tests, with the joystick disconnected.
Uses the existing planner-hold limits at 2 Hz; this is not a 200 Hz safety
monitor or proof that physical joint oscillation is zero.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=300)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 600:
        parser.error('--seconds must be within [5, 600]')
    repo = Path(__file__).resolve().parents[2]
    runtime = repo.parent / 'motion_exchange' / '.runtime'
    output = repo / '.build' / time.strftime('standing-window-%Y%m%dT%H%M%S.json')
    output.parent.mkdir(exist_ok=True)
    report = dict(passed=False, duration_s=args.seconds, samples=[])
    print('REPORT', output, flush=True)
    try:
        assert not any('16042' in line and 'ESTAB' in line for line in
                       subprocess.check_output(['ss', '-tn'], text=True).splitlines()), 'Disconnect PS4 client'
        deadline = time.monotonic() + args.seconds
        session = None
        while time.monotonic() < deadline:
            d = json.loads((runtime / 'isaac_status.json').read_text())
            request = json.loads((runtime / 'request.json').read_text())
            session = session or d['session_id']
            row = {k:d[k] for k in ('session_id', 'state', 'updated_epoch_s',
                   'simulation_time_s', 'root_height_m', 'root_tilt_rad',
                   'max_joint_velocity_rad_s', 'elastic_support_scale',
                   'elastic_support_attitude_scale')}
            row['planar_speed_m_s'] = math.hypot(*d['root_linear_velocity_m_s'][:2])
            row['yaw_rate_rad_s'] = abs(d['root_angular_velocity_rad_s'][2])
            report['samples'].append(row)
            assert 0 <= time.time()-row['updated_epoch_s'] < 3, 'stale/future status'
            assert row['session_id'] == session, 'session changed'
            assert row['state'] == request['state'] == 'INTERACTIVE', 'not joystick-owned standing'
            assert row['elastic_support_scale'] == row['elastic_support_attitude_scale'] == 0, 'support active'
            assert .70 <= row['root_height_m'] <= .90, 'height outside hold gate'
            assert 0 <= row['root_tilt_rad'] <= .10, 'tilt outside hold gate'
            assert 0 <= row['max_joint_velocity_rad_s'] <= .9, 'joint speed outside hold gate'
            assert row['planar_speed_m_s'] <= .15, 'translation outside hold gate'
            assert row['yaw_rate_rad_s'] <= .20, 'yaw outside hold gate'
            time.sleep(.5)
        report['passed'] = True
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        output.write_text(json.dumps(report, indent=2)+'\n')
        print('PASS' if report['passed'] else 'FAIL', 'samples', len(report['samples']), flush=True)


if __name__ == '__main__':
    main()
