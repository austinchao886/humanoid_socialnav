"""Build a short G1 artifact from saved PICO SMPL poses using pinned GMR."""
import argparse,hashlib,json,os,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0,'/workspace/services/video_generator/adapters')
from gmr_adapter import load_smplx_compatible,condition_feet
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.smpl import get_gvhmr_data_offline_fast
from motion_contracts.artifact import convert_g1_qpos,render_artifact_preview
from motion_contracts.validator import validate_artifact
from dataclasses import asdict
parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=2);parser.add_argument('--motion-id',default='pico-gmr-2s-v1');parser.add_argument('--converted-dir',type=Path,default=Path('/motion_exchange/diagnostics/pico/session-20260920-175428/sonic-reference-v2'));parser.add_argument('--body-jsonl',type=Path,default=Path('/workspace/runtime/pico/offline-session-20260920-175428/body.jsonl'));args=parser.parse_args()
assert 0<args.duration<=30
assert args.motion_id.startswith('pico-gmr-') and '/' not in args.motion_id and '..' not in args.motion_id
root=Path('/motion_exchange');work=root/'diagnostics/pico'/args.motion_id;work.mkdir(exist_ok=False)
source=args.converted_dir
files=sorted(source.glob('pose_*.npz'))[:round(args.duration*50)]
body=[];orient=[]
base=R.from_quat([.5,.5,.5,.5]);up_inverse=R.from_euler('x',-90,degrees=True)
for file in files:
 with np.load(file) as a:
  body.append(a['smpl_pose'][-1].reshape(63))
  q=a['body_quat_w'][-1]
  orient.append((up_inverse*R.from_quat(q[[1,2,3,0]])*base).as_rotvec())
rawpath=args.body_jsonl
raw=[json.loads(line) for line in rawpath.read_text().splitlines()]
times=np.array([f['elapsed_s'] for f in raw]);times-=times[0]
positions=np.array([f['poses'][0][:3] for f in raw])
targets=np.arange(len(files))*.02+.08
trans=np.stack([np.interp(targets,times,positions[:,axis]) for axis in range(3)],axis=1);trans-=trans[0]
canonical=work/'canonical_smplx.npz'
np.savez(canonical,pose_body=np.stack(body),root_orient=np.stack(orient),trans=trans,betas=np.zeros(10),gender='neutral',mocap_frame_rate=np.array(50.))
data,model,output,height=load_smplx_compatible(canonical,Path('/models/body_models'))
frames,fps=get_gvhmr_data_offline_fast(data,model,output,tgt_fps=50)
retarget=GMR(actual_human_height=height,src_human='smplx',tgt_robot='unitree_g1',use_velocity_limit=True)
qpos=np.stack([np.array(retarget.retarget(frame),copy=True) for frame in frames])
mjcf=Path('/opt/GMR/assets/unitree_g1/g1_mocap_29dof.xml')
qpos,contacts,metrics=condition_feet(qpos,fps,mjcf)
np.savez_compressed(work/'g1_motion.npz',qpos=qpos,fps=fps,foot_contacts=contacts,contact_metrics=metrics)
np.savetxt(work/'qpos.csv',qpos,delimiter=',')
os.environ['G1_MJCF']=str(mjcf)
artifact=root/args.motion_id
manifest=convert_g1_qpos(work/'qpos.csv',artifact,request_id=args.motion_id,motion_id=args.motion_id,source='pico_offline',model='GMR-bb1bbe4',prompt=f'{len(files)/50:.2f} seconds of recorded PICO full-body motion',source_fps=float(fps),neutral_transition_s=3.,neutral_hold_s=1.,source_sha256=hashlib.sha256(rawpath.read_bytes()).hexdigest(),source_metadata=dict(recording=str(rawpath),segment_s=[.08,.08+len(files)/50],retargeter='GMR-bb1bbe40774794fceb2a7c579a3464a28e68c844',body_shape='neutral SMPL-X betas=0',estimated_human_height_m=height))
result=validate_artifact(artifact);print(json.dumps(asdict(result),indent=2),flush=True)
render_artifact_preview(artifact,artifact/'preview.mp4',50.)
raise SystemExit(0 if result.valid else 2)
