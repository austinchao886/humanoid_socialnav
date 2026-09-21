"""Unarmed raw-PICO retargeting service. Writes a latest reference for diagnostics.

It never publishes controller commands or selects a SONIC mode. Runtime session
integration must explicitly consume and arm this reference after validation.
"""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
import zmq
from motion_pipeline.pico_live import CausalRetargeter,FreshnessGate,canonical_from_pico,host_latency_metadata

def atomic(path,value):
 temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,allow_nan=False));temp.replace(path)

def main():
 p=argparse.ArgumentParser();p.add_argument('--models',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=0);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=True);engine=CausalRetargeter(a.models);ctx=zmq.Context();s=ctx.socket(zmq.SUB);s.setsockopt(zmq.SUBSCRIBE,b'');s.setsockopt(zmq.CONFLATE,1);s.setsockopt(zmq.LINGER,0);s.connect('tcp://127.0.0.1:15558')
 gate=FreshnessGate();start=time.monotonic();last_status=0;count=0;error=None
 try:
  while not a.duration or time.monotonic()-start<a.duration:
   if s.poll(20):
    try:
     sample=s.recv_json();now=time.monotonic()
     if sample.get('schema_version')!=1:raise ValueError('unsupported schema')
     if not 0<=float(sample['source_age_s'])<=.25:raise ValueError('stale at sender')
     session=sample['session_id'];sequence=sample['sequence']
     changed=session!=gate.session or gate.received is None or now-gate.received>.25
     if not gate.accept(session,sequence,now):continue
     pose,root,trans=canonical_from_pico(sample['poses'])
     if changed:engine.reset()
     timestamp=int(sample['capture_timestamp_ns'])/1e9
     begin=time.perf_counter();packet,reference=engine.process(pose,root,trans,timestamp)
     try:calibration=json.loads((a.output/'clock-offset.json').read_text())
     except (FileNotFoundError,json.JSONDecodeError):calibration=None
     reference.update(host_latency_metadata(sample,calibration,time.time()))
     reference.update(session_id=session,source_sequence=sequence,received_monotonic_s=now,
        produced_monotonic_s=time.monotonic(),retarget_ms=(time.perf_counter()-begin)*1000,
        source_send_epoch_s=sample['source_send_epoch_s'],source_age_s=sample['source_age_s'],
        controller_armed=False,scope='diagnostic_reference_only')
     atomic(a.output/'reference.json',reference);count+=1;error=None
    except (ValueError,KeyError,TypeError,json.JSONDecodeError) as exc:
     error=str(exc);gate=FreshnessGate()
   now=time.monotonic()
   if now-last_status>.2:
    age=None if gate.received is None else now-gate.received
    atomic(a.output/'status.json',dict(state='FRESH_UNARMED' if error is None and age is not None and age<=.25 else 'STALE',
      frames=count,input_age_s=age,error=error,controller_armed=False,updated_epoch_s=time.time()))
    last_status=now
 finally:
  atomic(a.output/'status.json',dict(state='STOPPED',frames=count,controller_armed=False,updated_epoch_s=time.time()))
  s.close();ctx.term()
if __name__=='__main__':main()
