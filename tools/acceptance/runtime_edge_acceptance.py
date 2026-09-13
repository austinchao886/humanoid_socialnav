"""Simulation-only deadman, disconnect, and current abort behavior checks."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

repo = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(repo))
from motion_pipeline.joystick import JoystickCommand,encode_line

exchange = Path(os.environ.get('MOTION_EXCHANGE',str(repo.parent/'motion_exchange')))
output = repo/'.build'/time.strftime('runtime-edge-%Y%m%dT%H%M%S.json')
events=[]
def read(*, allow_recovery_gap=False):
    d=json.loads((exchange/'.runtime/isaac_status.json').read_text())
    if time.time()-d['updated_epoch_s']>=3:
        if allow_recovery_gap:
            return None
        raise AssertionError('stale runtime status')
    return d
def record(label,d):
    item=dict(event=label,epoch_s=time.time(),**{k:d.get(k) for k in ['state','session_id','root_height_m','root_tilt_rad','root_linear_velocity_m_s','reason']})
    events.append(item);print(json.dumps(item),flush=True)
    output.write_text(json.dumps(events,indent=2)+'\n')
def send(s,seconds,dm,forward=.5):
    end=time.monotonic()+seconds
    seq=0
    while time.monotonic()<end:
        s.sendall(encode_line(JoystickCommand(time.monotonic_ns(),dm,False,0,-forward,0,0)))
        time.sleep(.02)
def intact(session):
    d=read();assert d['session_id']==session and d['state']=='INTERACTIVE',d
    return d

assert read()['state']=='INTERACTIVE'
assert not any('16042' in line and 'ESTAB' in line for line in subprocess.check_output(['ss','-tn'],text=True).splitlines()),'Disconnect PS4 client first'
session=read()['session_id']
with socket.create_connection(('127.0.0.1',16042),timeout=5) as s:
    try:
        send(s,3,True)
        record('before_L1_release',intact(session))
        send(s,5,False)
        record('after_L1_release',intact(session))
    finally:
        send(s,.3,False,0)
with socket.create_connection(('127.0.0.1',16042),timeout=5) as s:
    send(s,3,True)
    record('before_disconnect',intact(session))
# Intentionally no release packet: the bridge must expire/release the input.
for _ in range(50):
    intact(session);time.sleep(.1)
record('after_disconnect',intact(session))

request='phone-action1-r4-20260825'
motion=request+'-a42efb29'
def command(action):
    return subprocess.Popen(['docker','exec','sonic-tracker','motion-cli','--domain','42','--interface','lo','control',action,request,motion])
approval=command('approve_execute')
deadline=time.monotonic()+90
while time.monotonic()<deadline:
    intact_status=read()
    assert intact_status['session_id']==session and intact_status['state'] not in ['UNSAFE','SAFE_STOP'],intact_status
    req=json.loads((exchange/'.runtime/request.json').read_text())
    if req.get('state')=='PLAYING' and req.get('motion_id')==motion:break
    time.sleep(.1)
else: raise TimeoutError('reference never began playback')
time.sleep(2)
record('before_abort',read())
aborting=command('abort')
deadline=time.monotonic()+150
last=None
while time.monotonic()<deadline:
    # A hard abort terminates Isaac. Only this bounded recovery loop permits
    # a stale status; stale INTERACTIVE must never count as recovery.
    d=read(allow_recovery_gap=True)
    if d is None:
        time.sleep(.1)
        continue
    key=(d['state'],d['session_id'])
    if key!=last:record('abort_transition',d);last=key
    if d['state']=='READY' and d['session_id']!=session:
        record('abort_recovered_supported_requires_rearm',d)
        break
    if d['state']=='INTERACTIVE':
        record('abort_recovered_same_session' if d['session_id']==session else 'abort_recovered_new_session',d)
        break
    time.sleep(.1)
else:raise TimeoutError('abort did not recover')
assert approval.wait(timeout=5)==0 and aborting.wait(timeout=5)==0
print('REPORT',output,flush=True)
