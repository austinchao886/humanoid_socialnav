"""Benchmark causal conversion without opening any controller socket."""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from motion_pipeline.pico_live import CausalRetargeter
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--models',required=True);p.add_argument('--frames',type=int,default=100);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
with np.load(a.input,allow_pickle=False) as f: data={k:f[k] for k in f.files}
engine=CausalRetargeter(a.models);elapsed=[];records=[]
# Independent first-frame equivalence check against full SMPL-X evaluation.
original_retarget=engine.retarget.retarget
max_fk_error=[]
def checked_retarget(frame):
 import torch
 from smplx.joint_names import JOINT_NAMES
 if len(max_fk_error)<10:
  i=len(max_fk_error)
  with torch.inference_mode():
   out=engine.model(body_pose=torch.tensor(data['pose_body'][i:i+1]).float(),
     global_orient=torch.tensor(data['root_orient'][i:i+1]).float(),
     transl=torch.tensor(data['trans'][i:i+1]-data['trans'][0]).float(),return_verts=False)
  expected=engine.up.apply(out.joints.detach().numpy()[0,:len(engine.parents)])
  actual=np.stack([frame[n][0] for n in JOINT_NAMES[:len(engine.parents)]])
  error=float(np.abs(actual-expected).max());max_fk_error.append(error)
  assert error<1e-5, ('FK mismatch',error)
 return original_retarget(frame)
engine.retarget.retarget=checked_retarget
for i in range(min(a.frames,len(data['pose_body']))):
 start=time.perf_counter();packet,row=engine.process(data['pose_body'][i],data['root_orient'][i],data['trans'][i],i/50);elapsed.append((time.perf_counter()-start)*1000);records.append(row)
result=dict(scope='causal_retarget_only_no_controller',frames=len(records),first_frame_ms=elapsed[0],steady_p50_ms=float(np.percentile(elapsed[10:],50)),steady_p95_ms=float(np.percentile(elapsed[10:],95)),steady_max_ms=max(elapsed[10:]),output_protocol=1,causal=True,max_smplx_fk_error_m=max(max_fk_error))
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));a.output.with_suffix('.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records));print(json.dumps(result,indent=2))
