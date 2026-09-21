"""Lag analysis of matched interior samples; not an end-to-end latency measurement."""
import json
from pathlib import Path
import numpy as np
from motion_contracts.validator import G1_ISAACLAB_JOINT_NAMES
root=Path('/exchange')
order=[0,3,6,9,13,17,1,4,7,10,14,18,2,5,8,11,15,19,21,23,25,27,12,16,20,22,24,26,28]
names=[G1_ISAACLAB_JOINT_NAMES[i] for i in order]
reference=np.loadtxt(root/'pico-gmr-full-v1/joint_pos.csv',delimiter=',',skiprows=1)[:,order]
results=[]
for stamp in ['025408Z','044012Z','052119Z']:
 trace=root/'executions'/('isaac_g129_deployment_v1_20260921T'+stamp+'.jsonl');rows=[]
 with trace.open() as stream:
  for line in stream:
   d=json.loads(line);frame=d.get('reference_frame')
   if frame is None:continue
   if rows and frame>1669:break
   if 225<=frame<=1669:rows.append((frame,d['joint_pos_unitree_order']))
 if not rows:raise ValueError('No matching motion segment: '+str(trace))
 frames=np.array([r[0] for r in rows],dtype=int);actual=np.array([r[1] for r in rows])
 shifts=np.arange(-25,26);errors=np.array([actual-reference[frames-shift] for shift in shifts]);rms=np.sqrt(np.mean(errors**2,axis=(1,2)));best=int(np.argmin(rms));base=25;per=np.sqrt(np.mean(errors[base]**2,axis=0));worst=np.argsort(per)[::-1][:6]
 joints=[]
 for j in worst:
  jrms=np.sqrt(np.mean(errors[:,:,j]**2,axis=1));b=int(np.argmin(jrms));joints.append(dict(joint=names[j],rmse_deg=float(np.degrees(per[j])),best_lag_sim_ms=int(shifts[b]*20),lag_adjusted_rmse_deg=float(np.degrees(jrms[b]))))
 results.append(dict(trace=trace.name,samples=len(rows),rmse_deg=float(np.degrees(rms[base])),best_global_lag_sim_ms=int(shifts[best]*20),lag_adjusted_rmse_deg=float(np.degrees(rms[best])),mse_reduction_fraction=float(1-rms[best]**2/rms[base]**2),peak_error_deg=float(np.degrees(np.max(np.abs(errors[base])))),lag_adjusted_peak_deg=float(np.degrees(np.max(np.abs(errors[best])))),worst_joints=joints))
print(json.dumps(dict(scope='offline_recorded_motion_interior',frame_range=[225,1669],scan_sim_ms=[-500,500],positive_lag='actual trails reference in simulation time; not capture-to-response latency',limitations=['Matched interior samples within each run; trace sampling rate differs between runs.','Reference-frame index quantizes alignment to 20 ms.','Retrospective best lag is descriptive, not an implementable controller improvement.'],results=results),indent=2))
