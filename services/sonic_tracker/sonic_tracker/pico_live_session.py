"""Opt-in simulation-only SONIC v1 session; arm requests are never automatic."""
import json,math,os,signal,threading,time
from collections import deque
import numpy as np
from motion_pipeline.pico_live import pack_v1
from motion_contracts.validator import G1_LOWER, G1_UPPER, G1_VELOCITY
from sonic_tracker.pico_replay import Publisher,Watchdog,write


def validate_reference(value, now, session):
    if value.get('session_id')!=session:raise ValueError('source session changed; recalibrate and re-arm')
    age=now-float(value['produced_monotonic_s'])
    if not 0<=age<=.25:raise ValueError('stale reference')
    if not value.get('clock_verified'):raise ValueError('verified host clock calibration required')
    q=np.asarray(value['joint_pos'],dtype=float);quat=np.asarray(value['body_quat'],dtype=float)
    pack_v1(q[None],np.zeros((1,29)),quat[None],[0])
    if np.any(q<G1_LOWER) or np.any(q>G1_UPPER):raise ValueError('joint position limit exceeded')
    latency=float(value['host_sample_to_reference_ms'])/1000+age
    uncertainty=float(value['clock_uncertainty_ms'])/1000
    if not all(math.isfinite(x) for x in (latency,uncertainty)) or not 0<=uncertainty<=.025 or latency+uncertainty>.25 or latency+uncertainty<0:
        raise ValueError('source freshness bound exceeded')
    return q,quat


def realtime_factor(before,after):
    if before.get('session_id')!=after.get('session_id'):raise ValueError('simulation restarted during timing gate')
    wall=float(after['updated_epoch_s'])-float(before['updated_epoch_s'])
    simulation=float(after['simulation_time_s'])-float(before['simulation_time_s'])
    if not math.isfinite(wall) or wall<1:raise ValueError('insufficient fresh simulation timing')
    return simulation/wall


def validate_simulation(environment,status,expected_session,rtf):
    required={'SONIC_DDS_DOMAIN':'42','SONIC_INTERFACE':'lo',
       'SONIC_LOWCMD_TOPIC':'rt/socialnav_sim/g1/lowcmd','SONIC_LOWSTATE_TOPIC':'rt/socialnav_sim/g1/lowstate'}
    if any(environment.get(k)!=v for k,v in required.items()):raise ValueError('isolated simulation only')
    if status.get('session_id')!=expected_session or status.get('state')!='INTERACTIVE':raise ValueError('simulation session is not ready')
    if not math.isfinite(rtf) or not .95<=rtf<=1.05:raise ValueError(f'realtime gate failed: RTF={rtf:.3f}, requires 0.95–1.05')


def execute(s,request):
    directory=s.runtime_dir/'pico_live';result=directory/'session.json'
    report=dict(state='VALIDATING',request=request,scope='simulation_only')
    publisher=None;watchdog=None;engaged=False
    try:
        expiry=float(request['expires_epoch_s'])
        if request.get('action')!='arm' or not 0<expiry-time.time()<=30:raise ValueError('invalid or expired arm request')
        duration=float(request.get('duration_s',120))
        if not 1<=duration<=120:raise ValueError('pilot duration must be 1–120 seconds')
        source=request['source_session_id'];session=request['session_id']
        read=lambda:json.loads((directory/'reference.json').read_text())
        validate_reference(read(),time.monotonic(),source)
        before=s._require_isaac_ready();start=time.monotonic()
        if s.runtime_mode!='JOYSTICK_LOCOMOTION':raise ValueError('another controller mode owns simulation')
        while time.monotonic()-start<2:
            if s.abort_event.is_set():raise RuntimeError('aborted')
            time.sleep(.05)
        after=s._require_isaac_ready()
        rtf=realtime_factor(before,after)
        report['admission_rtf']=rtf;validate_simulation(os.environ,after,session,rtf)
        s._wait_for_stable_standing(session,stable_duration=.5,timeout=15,accepted_states={'INTERACTIVE'},require_stationary=True)
        validate_reference(read(),time.monotonic(),source)
        runtime=json.loads(s.runtime_request_path.read_text())
        from motion_contracts.artifact import SONIC_OFFICIAL_NEUTRAL_ISAACLAB
        previous=np.asarray(SONIC_OFFICIAL_NEUTRAL_ISAACLAB,dtype=float)
        history=deque(maxlen=5);last_sequence=None;count=0
        publisher=Publisher();engaged=True;s.child.send('#')
        s._expect_or_abort([r'\[InterfaceManager\] Switched to: ZMQ'],timeout=5)
        s._expect_or_abort([r'Safety reset: ZMQ streaming disabled'],timeout=5)
        s.child.send('\n');s._expect_or_abort([r'ZMQ STREAMING MODE: ENABLED'],timeout=5)
        s.runtime_mode='PICO_LIVE';report['state']='ARMED';write(result,report)
        watchdog=Watchdog(lambda:s._signal_runtime_mode(signal.SIGUSR2),timeout=.25);watchdog.start()
        start=time.monotonic();last_publish=start;next_status=start
        while time.monotonic()-start<duration:
            now=time.monotonic()
            if s.abort_event.is_set():raise RuntimeError('aborted')
            if watchdog.fired:break
            command=directory/'control.json'
            if command.exists():
                c=json.loads(command.read_text());command.unlink()
                if c.get('action') in ('pause','stop','calibrate'):
                    report['end_reason']=c['action'];break
            s._service_controller_output()
            value=read()
            try:q,quat=validate_reference(value,now,source)
            except ValueError as exc:report['end_reason']=str(exc);break
            if now>=next_status:
                status=s._require_isaac_ready()
                if status['session_id']!=session:raise RuntimeError('simulation restarted')
                if status.get('root_height_m',0)<.5 or status.get('root_tilt_rad',99)>.6:raise RuntimeError('posture bounds exceeded')
                next_status=now+.1
            if last_sequence is not None and value['source_sequence']<last_sequence:
                report['end_reason']='source sequence regressed';break
            if now-watchdog.last>.25:
                report['end_reason']='publisher scheduling gap';break
            if value['source_sequence']==last_sequence or now-last_publish<.019:
                time.sleep(.002);continue
            # A bounded entry blend prevents a jump from the standing reference.
            blend=min(1.,(now-start)/3.);blend=blend*blend*(3-2*blend)
            neutral=np.asarray(SONIC_OFFICIAL_NEUTRAL_ISAACLAB)
            q=neutral+blend*(q-neutral)
            velocity=(q-previous)/max(now-last_publish,.001)
            if np.any(np.abs(velocity)>G1_VELOCITY) or np.max(np.abs(q-previous))>.35:
                report['end_reason']='reference discontinuity';break
            previous=q.copy();last_publish=now;last_sequence=value['source_sequence']
            history.append((q,velocity,quat,count));count+=1
            publisher.send(pack_v1(*[np.stack([r[i] for r in history]) for i in range(4)]));watchdog.feed()
        watchdog.close();report.update(sent_packets=count,watchdog=watchdog.fired)
        report.setdefault('end_reason','STALE_SOURCE' if watchdog.fired else 'DURATION_COMPLETE')
        s._enter_joystick_locomotion(runtime,session,timeout=10)
        s._wait_for_stable_standing(session,stable_duration=.5,timeout=15,accepted_states={'INTERACTIVE'},require_stationary=True)
        report['state']='RETURNED_TO_STAND'
    except Exception as exc:
        report.update(state='FAILED' if engaged else 'REJECTED',error=str(exc))
        if engaged:s.abort_event.set();s._stop()
    finally:
        if watchdog:watchdog.close()
        if publisher:publisher.close()
        directory.mkdir(exist_ok=True);write(result,report)


def poll(s):
    if os.getenv('SONIC_ENABLE_PICO_LIVE','0')!='1':return
    pending=s.runtime_dir/'pico_live/arm.json'
    if not pending.exists() or s.abort_event.is_set():return
    with s.lock:
        if s.execution_thread is not None and s.execution_thread.is_alive():return
        claimed=pending.with_suffix('.consumed.json');pending.replace(claimed)
        try:request=json.loads(claimed.read_text())
        except ValueError as exc:
            write(pending.parent/'session.json',dict(state='REJECTED',error=str(exc)));return
        s.execution_thread=threading.Thread(target=execute,args=(s,request),daemon=True,name='pico-live')
        s.execution_thread.start()
