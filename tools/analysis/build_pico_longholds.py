"""Unchanged source poses with 15-second plateaus and lookahead-free evaluation."""
import argparse,json,os
from dataclasses import asdict
from pathlib import Path
import numpy as np
from pico_accuracy import qpos_from_artifact,checksum,MJCF,save
from motion_contracts.artifact import _blend_qpos,_neutral_qpos,convert_g1_qpos
from motion_contracts.validator import validate_artifact

def build(exchange):
 source=exchange/'pico-gmr-full-v1';motion='pico-accuracy-longholds-v1';destination=exchange/motion
 work=exchange/'diagnostics/pico/longholds-v1';work.mkdir(parents=True,exist_ok=False)
 if destination.exists():raise FileExistsError(destination)
 q=qpos_from_artifact(source);manifest=json.loads((source/'manifest.json').read_text());previous=_neutral_qpos(q[200]);pieces=[];ranges=[];offset=200
 for label,frame,target in [('neutral',None,previous),('pose_1',214,q[214]),('pose_3',737,q[737])]:
  pieces.append(_blend_qpos(previous,target,np.arange(1,151)/150));offset+=150
  pieces.append(np.repeat(target[None],750,axis=0))
  ranges.append(dict(label=label,source_frame=frame,start=offset,end=offset+750,evaluate_start=offset+500,evaluate_end=offset+600))
  offset+=750;previous=target
 csv=work/'longholds.csv';np.savetxt(csv,np.vstack(pieces),delimiter=',');os.environ['G1_MJCF']=str(MJCF)
 convert_g1_qpos(csv,destination,request_id=motion,motion_id=motion,source='pico_diagnostic',model=manifest['model'],source_fps=50.,smoothing_sigma_frames=0.,neutral_transition_s=3.,neutral_hold_s=1.,source_sha256=checksum(source/'joint_pos.csv'),source_metadata=dict(parent_motion='pico-gmr-full-v1',experiment='longholds',selected_frames=[214,737]))
 validation=validate_artifact(destination)
 if not validation.valid:raise ValueError(asdict(validation))
 generated=qpos_from_artifact(destination)
 for h in ranges:
  target=_neutral_qpos(q[200]) if h['source_frame'] is None else q[h['source_frame']]
  np.testing.assert_allclose(generated[h['start']:h['end']],np.repeat(target[None],750,axis=0),atol=1e-12)
  assert h['end']-h['evaluate_end']>=45 and h['evaluate_end']-h['evaluate_start']==100
 save(destination/'accuracy_experiment.json',dict(parent_motion='pico-gmr-full-v1',experiment='longholds',hold_ranges=ranges,evaluation='seconds 10–12 of each 15-second plateau; next transition is at least 3 s away, beyond 0.9 s lookahead'))
 result=dict(validation=asdict(validation),source_sha256=checksum(source/'joint_pos.csv'),derived_sha256=checksum(destination/'joint_pos.csv'),mjcf_sha256=checksum(MJCF),hold_ranges=ranges)
 save(work/'generation.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--exchange',type=Path,required=True);build(p.parse_args().exchange)
