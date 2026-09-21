"""Socket-level live adapter test using saved raw poses; no SONIC connection."""
import json,os,subprocess,sys,time
from pathlib import Path
import zmq
root=Path('/motion_exchange/diagnostics/pico/live-service-smoke');root.mkdir(parents=True,exist_ok=True)
for name in ['status.json','reference.json']:
 try:(root/name).unlink()
 except FileNotFoundError:pass
ctx=zmq.Context();pub=ctx.socket(zmq.PUB);pub.setsockopt(zmq.LINGER,0);pub.bind('tcp://127.0.0.1:15558')
child=subprocess.Popen([sys.executable,'-m','motion_pipeline.pico_live_service','--models','/models/body_models','--output',str(root),'--duration','20'],stdout=subprocess.DEVNULL)
def wait_state(state,timeout=15):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if child.poll() is not None:raise RuntimeError('service exited '+str(child.returncode))
  try:
   v=json.loads((root/'status.json').read_text())
   if v['state']==state:return v
  except (FileNotFoundError,json.JSONDecodeError):pass
  time.sleep(.02)
 raise AssertionError(('missing state',state))
try:
 wait_state('STALE');time.sleep(.2)
 with open('/workspace/runtime/pico/offline-session-20260920-175428/body.jsonl') as f:raw=json.loads(next(f))
 def send(session,sequence):
  pub.send_json(dict(schema_version=1,session_id=session,sequence=sequence,capture_timestamp_ns=1_000_000_000+sequence*20_000_000,source_send_epoch_s=time.time(),source_age_s=0.,poses=raw['poses']))
 for i in range(30):send('first',i);time.sleep(.02)
 v=wait_state('FRESH_UNARMED',2);assert not v['controller_armed']
 wait_state('STALE',2)
 for i in range(15):send('second',i);time.sleep(.02)
 wait_state('FRESH_UNARMED',2)
 ref=json.loads((root/'reference.json').read_text());assert ref['session_id']=='second';assert not ref['controller_armed'];assert ref['frame_index']<15
 for i in range(15):send('second',14);time.sleep(.02)
 wait_state('STALE',2)
 print(json.dumps(dict(result='PASS',checks=['fresh_unarmed','250ms_stale','new_session_reset','duplicate_not_fresh'],scope='no_controller_socket')))
finally:
 child.terminate();child.wait(timeout=10);pub.close();ctx.term()
