"""Bounded simulation-only idle / walk-stop / turn-stop trace markers."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

repo=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(repo))
from motion_pipeline.joystick import JoystickCommand, encode_line
root=repo.parent/'motion_exchange'
label=sys.argv[1]
assert label in ('baseline','velocity4','position8','tgs_forces','tgs8','tgs8-normal5')
out=repo/'.build'/f'oscillation-{label}.json'
def status():
    d=json.loads((root/'.runtime/isaac_status.json').read_text())
    assert time.time()-d['updated_epoch_s']<3, 'stale status'
    assert d['state']=='INTERACTIVE', d
    return d
deadline=time.monotonic()+90
while True:
    try:
        initial=status()
        break
    except AssertionError:
        if time.monotonic()>=deadline:
            raise
        time.sleep(.5)
session=initial['session_id']
assert not any('16042' in line and 'ESTAB' in line for line in subprocess.check_output(['ss','-tn'],text=True).splitlines()),'Disconnect PS4 client'
phases=[]
with socket.create_connection(('127.0.0.1',16042),timeout=5) as sock:
    seq=0
    def drive(seconds,dm,forward=0,yaw=0):
        global seq
        end=time.monotonic()+seconds
        next_check=0
        while time.monotonic()<end:
            sock.sendall(encode_line(JoystickCommand(seq,dm,False,0,-forward,yaw,0)))
            seq+=1
            if time.monotonic()>=next_check:
                assert status()['session_id']==session,'session changed'
                next_check=time.monotonic()+.2
            time.sleep(.02)
    try:
        for name,seconds,dm,forward,yaw in [('idle',12,False,0,0),('walk',4,True,.5,0),('walk_stop',20,False,0,0),('turn',4,True,.5,.15),('turn_stop',20,False,0,0)]:
            start=status()
            drive(seconds,dm,forward,yaw)
            finish=status()
            phases.append(dict(name=name,start=start,end=finish))
            out.write_text(json.dumps(dict(label=label,session_id=session,phases=phases),indent=2)+'\n')
            print(name,finish.get('max_joint_velocity_rad_s'),finish.get('root_tilt_rad'),flush=True)
    finally:
        sock.sendall(encode_line(JoystickCommand(seq,False,False,0,0,0,0)))
print('REPORT',out,flush=True)
