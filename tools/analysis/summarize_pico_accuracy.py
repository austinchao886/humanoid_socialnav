"""Summarize repeated trials without hiding failed runs or reference conditioning."""
import argparse,json
from pathlib import Path
import numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 trials=json.loads((a.directory/'trials.json').read_text());groups={};failed=[]
 for t in trials:
  name=t['motion_id'];g=groups.setdefault(name,dict(attempts=0,completed=0,runs=[]));g['attempts']+=1
  if t['state']!='COMPLETED' or not t.get('analysis'):
   failed.append(dict(motion_id=name,repetition=t['repetition'],state=t['state'],error=t.get('error')));continue
  # Resolve by basename so the study can also be inspected inside a container.
  d=json.loads((a.directory/Path(t['analysis']).name).read_text());g['completed']+=1
  g['runs'].append(dict(repetition=t['repetition'],session_id=t['session_id'],result=d))
 def span(values):return dict(mean=float(np.mean(values)),min=float(np.min(values)),max=float(np.max(values)))
 summary={}
 for name,g in groups.items():
  out=dict(attempts=g['attempts'],completed=g['completed']);runs=[r['result'] for r in g['runs']]
  if runs:
   out['realtime_factor']=span([d['realtime_factor'] for d in runs])
   out['phases']={phase:{metric:span([d['phases'][phase][metric] for d in runs]) for metric in ['rmse_deg','peak_deg','applied_torque_ratio_peak','saturated_sample_fraction']} for phase in runs[0]['phases']}
   out['all']={metric:span([d['all'][metric] for d in runs]) for metric in ['rmse_deg','peak_deg']}
   matched=[d.get('source_matched_active',d['phases']['active']) for d in runs]
   out['source_matched_active_rmse_deg']=span([d['rmse_deg'] for d in matched])
   joints=runs[0]['phases']['active']['per_joint']
   ranking={joint:span([d['phases']['active']['per_joint'][joint]['rmse_deg'] for d in runs]) for joint in joints}
   out['worst_active_joints']=dict(sorted(ranking.items(),key=lambda kv:kv[1]['mean'],reverse=True)[:8])
   out['active_endpoints']={joint:{metric:span([d['phases']['active']['endpoints'][joint][metric] for d in runs]) for metric in runs[0]['phases']['active']['endpoints'][joint]} for joint in runs[0]['phases']['active']['endpoints']}
   out['holds']={label:dict(final_two_second_rmse_deg=span([d['holds'][label]['rmse_deg'] for d in runs]),max_joint_speed_rad_s=span([d['holds'][label]['settled_max_joint_speed_rad_s'] for d in runs]),before_lookahead_rmse_deg=span([d['holds'][label]['before_next_transition_lookahead']['rmse_deg'] for d in runs])) for label in runs[0]['holds']}
  summary[name]=out
 result=dict(conditions=summary,failed_or_incomplete=failed,scope='Recorded simulation attribution; no live or hardware qualification',notes=['All errors unshifted unless explicitly labeled lag diagnostics.','Range across three trials describes observed variability; not a confidence interval.','Derived pose-hold and half-speed motions are diagnostic conditions, not accepted corrections.'])
 a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:{x:v.get(x) for x in ['attempts','completed','all','source_matched_active_rmse_deg']} for k,v in summary.items()},indent=2))
if __name__=='__main__':main()
