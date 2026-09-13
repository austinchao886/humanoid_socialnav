import json
import socket
import subprocess
import time
import sys
import os
from pathlib import Path

from motion_pipeline.joystick import JoystickCommand, encode_line

root = Path(os.environ.get('MOTION_EXCHANGE', '../motion_exchange')).resolve()
status_path = root / '.runtime/isaac_status.json'
request_path = root / '.runtime/request.json'
def status():
    d = json.loads(status_path.read_text())
    assert time.time() - d['updated_epoch_s'] < 4, 'stale Isaac status'
    return d

initial = status()
request_id, motion_id = sys.argv[1:3] if len(sys.argv) == 3 else ('phone-action1-r4-20260825','phone-action1-r4-20260825-a42efb29')
assert initial['state'] == 'INTERACTIVE', initial
session = initial['session_id']
assert session == os.environ.get('EXPECTED_SESSION', session), 'acceptance session changed'
turn = float(os.environ.get('ACCEPTANCE_TURN', '0'))
sequence = 0
events = []
def record(label, d):
    row = {'event': label, 'epoch_s':time.time(), **{k:d.get(k) for k in ('state','session_id','root_height_m','root_tilt_rad','max_joint_velocity_rad_s','root_linear_velocity_m_s','reason')}}
    events.append(row)
    print(json.dumps(row), flush=True)

with socket.create_connection(('127.0.0.1',16042), timeout=5) as connection:
    def drive(seconds, deadman, forward=0.0, yaw=0.0):
        global sequence
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            connection.sendall(encode_line(JoystickCommand(sequence,deadman,False,0.0,-forward,yaw,0.0)))
            sequence += 1
            time.sleep(.02)
    try:
        record('baseline',initial)
        drive(4, True, .5, turn)
        record('walking_before_approval',status())
        publisher = subprocess.Popen(['docker','exec','sonic-tracker','motion-cli','--domain','42','--interface','lo','control','approve_execute',request_id,motion_id])
        deadline = time.monotonic() + 120
        admission_deadline = time.monotonic() + 5
        next_sample = 0
        departed = False
        last = None
        while time.monotonic() < deadline:
            d = status()
            assert d['session_id'] == session, 'Isaac session changed'
            assert d['state'] not in ('UNSAFE','SAFE_STOP','ABORTED'), d
            req = json.loads(request_path.read_text()).get('state')
            key = (d['state'],req)
            if key != last:
                record('transition_request_'+str(req),d)
                last = key
            departed |= d['state'] != 'INTERACTIVE'
            if not departed and time.monotonic() > admission_deadline:
                raise TimeoutError('approval not admitted within 5s; release joystick')
            if time.monotonic() >= next_sample:
                record('telemetry', d)
                next_sample = time.monotonic() + 1
            if departed and d['state'] == 'INTERACTIVE':
                record('returned_to_interactive',d)
                break
            # Keep walking input through approval admission; release once
            # preemption is acknowledged, so replay return cannot auto-walk.
            drive(.1, not departed, .5 if not departed else 0, turn if not departed else 0)
        else:
            raise TimeoutError('reference did not return to INTERACTIVE')
        drive(3,True,.5)
        record('walking_after_reference',status())
        drive(4,False)
        final = status()
        assert final['session_id'] == session and final['state'] == 'INTERACTIVE', final
        record('stopped_after_round_trip',final)
        assert publisher.wait(timeout=5) == 0
        print('ROUND_TRIP_PASS',flush=True)
    finally:
        drive(.3,False)
