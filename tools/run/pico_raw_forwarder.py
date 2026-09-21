"""Forward the existing single-reader PICO snapshot; never open another SDK client."""
import argparse,json,time,uuid
from pathlib import Path
import zmq
p=argparse.ArgumentParser();p.add_argument('--sample-file',type=Path,required=True);p.add_argument('--port',type=int,default=5558);a=p.parse_args()
if a.port!=5558:raise ValueError('reserved diagnostic raw-PICO port is 5558')
ctx=zmq.Context();s=ctx.socket(zmq.PUB);s.setsockopt(zmq.SNDHWM,1);s.setsockopt(zmq.LINGER,0);s.bind('tcp://127.0.0.1:5558')
session=str(uuid.uuid4());last=None;sequence=0
try:
 while True:
  try:r=json.loads(a.sample_file.read_text())
  except (FileNotFoundError,json.JSONDecodeError):time.sleep(.01);continue
  now=time.monotonic();age=now-float(r['received_at']);stamp=int(r['timestamp_ns'])
  if stamp!=last and 0<=age<=.25:
   if last is not None and stamp<last:session=str(uuid.uuid4());sequence=0
   s.send_json(dict(schema_version=1,session_id=session,sequence=sequence,capture_timestamp_ns=stamp,
      source_received_monotonic_s=r['received_at'],source_send_epoch_s=time.time(),source_age_s=age,poses=r['poses']))
   last=stamp;sequence+=1
  time.sleep(.005)
finally:s.close();ctx.term()
