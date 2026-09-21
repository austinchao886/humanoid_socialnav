"""Wall-clock paced, latest-sample causal retargeting diagnostic.

Never selects a controller or arms a simulation. Optional publication is on
loopback diagnostic port 15560, separate from active SONIC endpoints.
"""
import argparse,json,sys,time,uuid
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from motion_pipeline.pico_live import CausalRetargeter,canonical_from_pico

def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--models',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--publish-diagnostic',action='store_true');a=p.parse_args()
 rows=[json.loads(x) for x in a.input.read_text().splitlines()]
 times=np.array([r['elapsed_s'] for r in rows]);times-=times[0]
 if len(rows)<2 or not np.all(np.diff(times)>0):raise ValueError('nonmonotonic recording')
 engine=CausalRetargeter(a.models)
 # Warm model/solver using the first pose before starting the source clock.
 pose,root,trans=canonical_from_pico(rows[0]['poses'])
 engine.process(pose,root,trans,-.02)
 pub=None
 if a.publish_diagnostic:
  import zmq
  context=zmq.Context();pub=context.socket(zmq.PUB);pub.setsockopt(zmq.SNDHWM,1);pub.setsockopt(zmq.LINGER,0);pub.bind('tcp://127.0.0.1:15560')
 session=str(uuid.uuid4());records=[];last=-1;start=time.monotonic();next_tick=start
 try:
  while True:
   now=time.monotonic();elapsed=now-start
   if elapsed>times[-1]:break
   index=int(np.searchsorted(times,elapsed,side='right')-1)
   if index>last:
    pose,root,trans=canonical_from_pico(rows[index]['poses'])
    begin=time.perf_counter();packet,state=engine.process(pose,root,trans,float(times[index]));finish=time.monotonic()
    if pub is not None:pub.send(packet)
    state.update(session_id=session,source_sequence=index,skipped_source_samples=index-last-1,
      retarget_ms=(time.perf_counter()-begin)*1000,source_to_reference_ms=(finish-start-times[index])*1000,
      wall_elapsed_s=finish-start)
    records.append(state);last=index
   next_tick+=.02;delay=next_tick-time.monotonic()
   if delay>0:time.sleep(delay)
   else:next_tick=time.monotonic()
 finally:
  if pub is not None:pub.close();context.term()
 a.output.parent.mkdir(parents=True,exist_ok=True)
 a.output.with_suffix('.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
 result=dict(scope='recorded_input_wall_clock_stream_no_controller',session_id=session,frames=len(records),source_duration_s=float(times[-1]),wall_s=time.monotonic()-start,
  retarget_p95_ms=float(np.percentile([r['retarget_ms'] for r in records],95)),
  source_to_reference_p95_ms=float(np.percentile([r['source_to_reference_ms'] for r in records],95)),
  skipped_source_samples=sum(r['skipped_source_samples'] for r in records),publication_endpoint='tcp://127.0.0.1:15560' if pub else None)
 a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
