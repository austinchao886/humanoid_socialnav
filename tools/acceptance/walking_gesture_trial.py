"""Simulation-only matched walking/gesture trial; no control-gain changes.

Run baseline first, then --with-gesture. Durations are wall-clock seconds;
report stores observed simulation timestamps for subsequent trace analysis.
Status sampling is not a substitute for the 200-Hz trace or native safety.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'services/sonic_tracker')]
from motion_pipeline.joystick import JoystickCommand, encode_line
from sonic_tracker.gesture_safety import require_composition_envelope

REQUEST = 'wave-official-e2e-005'
MOTION = 'wave-official-e2e-005-083aa901'
PLAN = '887e06bae6c25cc87ce8bb2e284a98ec158090deb656da8d6fa9f6452b6c2709'


def ramp(elapsed, duration, initial=.5):
    u = min(1., max(0., elapsed/duration))
    return initial*(1-u*u*(3-2*u))


def check_measurements(d, *, baseline_diagnostics=False, gait_window=False):
    if not baseline_diagnostics:
        return require_composition_envelope(d,now_epoch_s=time.time(),gait_window=gait_window)
    # Plain joystick baseline only: instantaneous root velocity is measured,
    # not confused with the speed command or gesture admission qualification.
    for key,lo,hi in (('root_height_m',.70,.90),('root_tilt_rad',0.,.25),
                      ('max_torque_limit_ratio',0.,1.)):
        value=d.get(key)
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise RuntimeError('baseline pose/torque envelope: '+key)
    stamp=d.get('updated_epoch_s')
    if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=time.time()-stamp<=1.5:
        raise RuntimeError('baseline status freshness')
    velocity=d.get('root_linear_velocity_m_s')
    if (not isinstance(velocity,list) or len(velocity)!=3 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in velocity)):
        raise RuntimeError('invalid baseline velocity measurement')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-gesture',action='store_true')
    parser.add_argument('--gait-envelope',action='store_true',help='Require complete physics-rate gait window.')
    parser.add_argument('--baseline-diagnostics',action='store_true',
                        help='Plain joystick only; record speed bound exceedances without gesture admission.')
    parser.add_argument('--forward',type=float,choices=[.25,.5],default=.25,
                        help='Normalized stick, NOT a linear fraction of physical speed.')
    args=parser.parse_args()
    if args.baseline_diagnostics and args.with_gesture:
        parser.error('--baseline-diagnostics cannot approve or run a gesture')
    if args.baseline_diagnostics and args.gait_envelope:
        parser.error('baseline diagnostics and gait admission are separate modes')
    exchange=ROOT.parent/'motion_exchange'
    runtime=exchange/'.runtime'
    output=ROOT/'.build'/('walking-gesture-'+uuid.uuid4().hex+'.json')
    report=dict(completed=False,with_gesture=args.with_gesture,baseline_diagnostics=args.baseline_diagnostics,gait_envelope=args.gait_envelope,
                gesture_admission_enforced=not args.baseline_diagnostics,normalized_forward=args.forward,events=[],samples=[],
                scope='experimental_same_session_trial_not_quality_qualification')
    session=None; seq=0; sock=None; approval=None; gesture_path=None
    gait_admission_active=False
    existing=set((exchange/'executions').glob('gesture-*.json'))
    def status():
        d=json.loads((runtime/'isaac_status.json').read_text())
        report['last_observed_status']=d
        req=json.loads((runtime/'request.json').read_text())
        if d['state']!='INTERACTIVE' or req['state']!='INTERACTIVE':
            raise RuntimeError('runtime not INTERACTIVE')
        if session is not None and d['session_id']!=session:
            raise RuntimeError('session changed')
        if (d['elastic_support_scale']!=0 or d['elastic_support_attitude_scale']!=0
                or d['control_handoff_progress']!=1):
            raise RuntimeError('unsupported control required')
        check_measurements(d,
            baseline_diagnostics=args.baseline_diagnostics or (args.gait_envelope and not gait_admission_active),
            gait_window=args.gait_envelope and gait_admission_active)
        return d
    def command(deadman,forward):
        nonlocal seq
        sock.sendall(encode_line(JoystickCommand(seq,deadman,False,0,-forward,0,0)))
        seq+=1
    def gesture():
        nonlocal gesture_path
        if not args.with_gesture or approval is None:return None
        if gesture_path is None:
            candidates=[]
            for p in set((exchange/'executions').glob('gesture-*.json'))-existing:
                g=json.loads(p.read_text())
                if g.get('session_id')==session and g.get('prepared_plan_id')==PLAN:
                    candidates.append(p)
            if len(candidates)>1:raise RuntimeError('ambiguous gesture reports')
            if candidates:gesture_path=candidates[0];report['gesture_report']=str(gesture_path)
        if gesture_path is None:return None
        g=json.loads(gesture_path.read_text())
        if g['state'] in ('FAILED','ABORTED'):raise RuntimeError('gesture failed: '+str(g.get('error')))
        return g
    def phase(name,seconds,forward=0.,deadman=True):
        start=time.monotonic();next_check=0.;last_sim=None
        event=dict(name=name,epoch_s=time.time(),before=status(),wall_duration_s=seconds,
                   gait_admission_active=gait_admission_active)
        report['events'].append(event);print('PHASE',name,flush=True)
        while time.monotonic()-start<seconds:
            elapsed=time.monotonic()-start
            command(deadman,forward(elapsed) if callable(forward) else forward)
            if time.monotonic()>=next_check:
                d=status();gesture();next_check=time.monotonic()+.1
                if d['simulation_time_s']!=last_sim:
                    report['samples'].append(dict(phase=name,epoch_s=time.time(),status=d))
                    last_sim=d['simulation_time_s']
            time.sleep(.02)
        event['after']=status()
    def cli(action):
        cmd=['docker','exec','sonic-tracker','motion-cli','--domain','42','--interface','lo',
             'control',action,REQUEST,MOTION]
        if action=='approve_execute':cmd+=['--prepared-plan-id',PLAN]
        return cmd
    try:
        if any('16042' in x and 'ESTAB' in x for x in subprocess.check_output(['ss','-tn'],text=True).splitlines()):
            raise RuntimeError('disconnect joystick operator before scripted trial')
        session=status()['session_id'];report['session_id']=session
        print('REPORT',output,'SESSION',session,flush=True)
        sock=socket.create_connection(('127.0.0.1',16042),timeout=2)
        phase('idle',5,deadman=False)
        phase('approach',8,args.forward)
        # Qualification applies BEFORE approval and throughout composition,
        # not retroactively to initial joystick acceleration. Startup and
        # resumed joystick retain pose/torque/session/native safety checks.
        gait_admission_active=args.gait_envelope
        status()
        if args.with_gesture:
            report['approval_epoch_s']=time.time()
            approval=subprocess.Popen(cli('approve_execute'),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        phase('walk_with_gesture' if args.with_gesture else 'matched_walk',12,args.forward)
        if args.with_gesture and (approval.poll()!=0 or gesture() is None):
            raise RuntimeError('approval did not create an execution report')
        phase('decelerate',4,lambda t:ramp(t,4,args.forward))
        phase('standing_finish',25)
        if args.with_gesture and gesture()['state']!='COMPLETED':
            raise RuntimeError('gesture did not complete in bounded standing window')
        gait_admission_active=False
        phase('resume_joystick',3,args.forward)
        phase('final_decelerate',4,lambda t:ramp(t,4,args.forward))
        phase('post',10,deadman=False)
        report['completed']=True
    except Exception as e:
        report['error']=f'{type(e).__name__}: {e}'
        raise
    finally:
        if sock is not None:
            try:command(False,0.)
            except OSError as e:report['deadman_release_error']=str(e)
            finally:sock.close()
        if not report['completed'] and approval is not None:
            try:subprocess.run(cli('abort'),timeout=5,check=False,capture_output=True)
            except subprocess.TimeoutExpired:report['abort_timeout']=True
        if approval is not None:
            try:report['approval_output']=approval.communicate(timeout=2)
            except subprocess.TimeoutExpired:report['approval_client_still_running']=True
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n')
        print('RESULT',output,'completed=',report['completed'],flush=True)


if __name__=='__main__':main()
