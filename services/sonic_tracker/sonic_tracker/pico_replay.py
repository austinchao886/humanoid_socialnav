"""Bounded, simulation-only offline PICO reference execution.

Separate from the robot/FK artifact contract. No headset or live source support.
"""
import ctypes as C
import ctypes.util
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import numpy as np

DTYPES = {'f': {4: 'f32', 8: 'f64'}, 'i': {4: 'i32', 8: 'i64'}, 'b': {1: 'bool'}}
REQUIRED = {'smpl_pose': (5,21,3), 'smpl_joints': (5,24,3),
            'body_quat_w': (5,4), 'joint_pos': (5,29), 'joint_vel': (5,29), 'frame_index': (5,)}

def pack(arrays):
    fields=[]; payload=[]
    for name, value in arrays.items():
        dtype=DTYPES[value.dtype.kind][value.dtype.itemsize]
        value=np.ascontiguousarray(value.astype(value.dtype.newbyteorder('<'),copy=False))
        fields.append(dict(name=name,dtype=dtype,shape=list(value.shape)));payload.append(value.tobytes())
    header=json.dumps(dict(v=3,endian='le',count=1,fields=fields),separators=(',',':')).encode()
    if len(header)>1280:raise ValueError('Pose header exceeds protocol limit')
    return b'pose'+header.ljust(1280,b'\0')+b''.join(payload)

def load_recording(directory, expected_sha):
    files=sorted(Path(directory).glob('pose_*.npz'))
    if not 5 <= len(files) <= 3000:raise ValueError('Recording must contain 5–3000 packets')
    digest=hashlib.sha256();packets=[];previous=None
    for f in files:
        raw=f.read_bytes();digest.update(f.name.encode()+b'\0'+raw)
        with np.load(f,allow_pickle=False) as archive:arrays={k:archive[k].copy() for k in archive.files}
        for key,shape in REQUIRED.items():
            if key not in arrays or arrays[key].shape!=shape:raise ValueError('Invalid shape: '+key)
        if not all(np.isfinite(v).all() for v in arrays.values()):raise ValueError('Nonfinite reference')
        ids=arrays['frame_index']
        if ids.dtype.kind!='i' or not np.all(np.diff(ids)==1):raise ValueError('Invalid frame window')
        if previous is not None and int(ids[-1])!=previous+1:raise ValueError('Noncontiguous recording')
        previous=int(ids[-1])
        if np.max(np.abs(np.linalg.norm(arrays['body_quat_w'],axis=-1)-1))>.01:raise ValueError('Invalid body orientation')
        for name in ('left_trigger','right_trigger','left_grip','right_grip','toggle_data_collection','toggle_data_abort','heading_increment'):
            if name in arrays and np.any(arrays[name]!=0):raise ValueError('Offline controller input must be neutral: '+name)
        packets.append(pack(arrays))
    checksum=digest.hexdigest()
    if checksum!=expected_sha:raise ValueError('Recording checksum mismatch')
    return packets,dict(packets=len(packets),duration_s=len(packets)/50,sha256=checksum)

class Publisher:
    """Minimal libzmq binding; all calls belong to the execution worker."""
    def __init__(self):
        self.lib=C.CDLL(ctypes.util.find_library('zmq'))
        for name,args,result in [('zmq_ctx_new',[],C.c_void_p),('zmq_socket',[C.c_void_p,C.c_int],C.c_void_p),
            ('zmq_bind',[C.c_void_p,C.c_char_p],C.c_int),('zmq_send',[C.c_void_p,C.c_void_p,C.c_size_t,C.c_int],C.c_int),
            ('zmq_setsockopt',[C.c_void_p,C.c_int,C.c_void_p,C.c_size_t],C.c_int),('zmq_close',[C.c_void_p],C.c_int),('zmq_ctx_term',[C.c_void_p],C.c_int)]:
            fn=getattr(self.lib,name);fn.argtypes=args;fn.restype=result
        self.ctx=self.lib.zmq_ctx_new();self.socket=self.lib.zmq_socket(self.ctx,1)
        for key,value in [(17,0),(23,3)]:
            v=C.c_int(value);self.lib.zmq_setsockopt(self.socket,key,C.byref(v),C.sizeof(v))
        if self.lib.zmq_bind(self.socket,b'tcp://127.0.0.1:5556')!=0:
            self.close();raise RuntimeError('PICO endpoint 5556 is already in use')
    def send(self,packet):
        if self.lib.zmq_send(self.socket,packet,len(packet),1)!=len(packet):raise RuntimeError('Pose send failed')
    def close(self):
        self.lib.zmq_close(self.socket);self.lib.zmq_ctx_term(self.ctx)

class Watchdog:
    def __init__(self,callback,timeout=.25):
        self.callback=callback;self.timeout=timeout;self.last=time.monotonic();self.done=threading.Event();self.fired=None
        self.thread=threading.Thread(target=self.run,daemon=True)
    def run(self):
        while not self.done.wait(.01):
            now=time.monotonic()
            if now-self.last>self.timeout:
                self.fired=dict(gap_s=now-self.last,triggered_monotonic=now)
                self.callback();return
    def start(self):self.last=time.monotonic();self.thread.start()
    def feed(self):self.last=time.monotonic()
    def close(self):self.done.set();self.thread.join(timeout=1)

def write(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(path)

def execute(s,request):
    report=dict(request=request,state='VALIDATING',scope='offline_pico_simulation_trial_not_qualified',samples=[])
    destination=s.runtime_dir/'pico_replay_result.json';publisher=None;watchdog=None;mutated=False
    try:
        required={'SONIC_DDS_DOMAIN':'42','SONIC_INTERFACE':'lo','SONIC_LOWCMD_TOPIC':'rt/socialnav_sim/g1/lowcmd','SONIC_LOWSTATE_TOPIC':'rt/socialnav_sim/g1/lowstate'}
        if any(os.getenv(k)!=v for k,v in required.items()):raise ValueError('PICO replay requires isolated simulation DDS')
        if time.time()>request['expires_epoch_s']:raise ValueError('Expired replay request')
        duration=float(request['duration_s']);interrupt=request.get('interrupt_after_s')
        if not math.isfinite(duration) or not .5<=duration<=30:raise ValueError('Duration outside .5–30 seconds')
        if interrupt is not None and not 0<float(interrupt)<duration:raise ValueError('Invalid interruption test time')
        directory=(s.exchange/request['recording']).resolve()
        if not directory.is_relative_to((s.exchange/'diagnostics/pico').resolve()):raise ValueError('Recording outside PICO store')
        packets,report['validation']=load_recording(directory,request['sha256'])
        duration=min(duration,len(packets)/50)
        ready=s._require_isaac_ready();session=ready['session_id']
        if session!=request['session_id'] or s.runtime_mode!='JOYSTICK_LOCOMOTION':raise ValueError('Simulation session or mode changed')
        s._wait_for_stable_standing(session,stable_duration=.5,timeout=10,accepted_states={'INTERACTIVE'},require_stationary=True)
        runtime=json.loads(s.runtime_request_path.read_text())
        publisher=Publisher()
        mutated=True;s.child.send('#')
        s._expect_or_abort([r'\[InterfaceManager\] Switched to: ZMQ'],timeout=5)
        # Delegate reset must finish before enabling its stream.
        s._expect_or_abort([r'Safety reset: ZMQ streaming disabled'],timeout=5)
        time.sleep(.2);s.child.send('\n')
        s._expect_or_abort([r'ZMQ STREAMING MODE: ENABLED'],timeout=5)
        s.runtime_mode='PICO_REPLAY';report['state']='PLAYING';write(destination,report)
        watchdog=Watchdog(lambda:s._signal_runtime_mode(signal.SIGUSR2));watchdog.start()
        start=time.monotonic();index=0;last_status=0
        while index < int(duration*50):
            now=time.monotonic()
            if s.abort_event.is_set():raise RuntimeError('Execution aborted')
            if watchdog.fired:break
            s._service_controller_output()
            if now-last_status>.1:
                status=s._require_isaac_ready()
                if status['session_id']!=session:raise RuntimeError('Isaac session changed')
                if status.get('root_height_m',0)<.50 or status.get('root_tilt_rad',99)>.6:raise RuntimeError('PICO trial exceeded posture bounds')
                report['samples'].append(status);last_status=now
            if interrupt is not None and now-start>=float(interrupt):
                time.sleep(.01);continue
            # A scheduling gap cannot silently resume an old reference.
            if now-watchdog.last>.25:time.sleep(.01);continue
            if now<start+index/50:time.sleep(min(.005,start+index/50-now));continue
            publisher.send(packets[index]);watchdog.feed();index+=1
        watchdog.close();report['watchdog']=watchdog.fired;report['sent_packets']=index
        report['end_reason']='STALE_SOURCE' if watchdog.fired else 'END_OF_RECORDING'
        s._enter_joystick_locomotion(runtime,session,timeout=10)
        report['after']=s._wait_for_stable_standing(session,stable_duration=.5,timeout=15,accepted_states={'INTERACTIVE'},require_stationary=True)
        report['state']='RETURNED_TO_STAND'
    except Exception as exc:
        report.update(state='FAILED' if mutated else 'REJECTED',error=str(exc))
        if mutated:
            s.abort_event.set();s._stop()
    finally:
        if watchdog is not None:watchdog.close()
        if publisher is not None:publisher.close()
        write(destination,report)
        print('[pico-replay] '+report['state']+' '+str(report.get('error','')),flush=True)

def poll(s):
    # Native SMPL mode failed neutral-input physics trials; require explicit opt-in.
    if os.getenv('SONIC_ENABLE_PICO_SMPL_TRIAL', '0') != '1':
        return
    pending=s.runtime_dir/'pico_replay_request.json'
    if not pending.exists() or s.abort_event.is_set():return
    with s.lock:
        if s.execution_thread is not None and s.execution_thread.is_alive():return
        claimed=pending.with_suffix('.consumed.json');pending.replace(claimed)
        try:request=json.loads(claimed.read_text())
        except Exception as exc:
            write(s.runtime_dir/'pico_replay_result.json',dict(state='REJECTED',error=str(exc)));return
        s.execution_thread=threading.Thread(target=execute,args=(s,request),daemon=True,name='pico-offline-replay')
        s.execution_thread.start()
