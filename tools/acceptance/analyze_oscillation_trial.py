"""Analyze bounded trial windows without loading an entire persistent trace.

Requires NumPy. Example in the SONIC image: mount executions at /traces,
then pass the trial JSON path and --trace-dir /traces.
"""
import argparse
import json
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('trial',type=Path)
parser.add_argument('--trace-dir',type=Path,default=Path('/traces'))
args=parser.parse_args()
meta=json.loads(args.trial.read_text())
trace=args.trace_dir/Path(meta['phases'][0]['end']['trace_path']).name
last_time=max(p['end']['simulation_time_s'] for p in meta['phases'])
first_time=min(p['start']['simulation_time_s'] for p in meta['phases'])
rows=[]
with trace.open() as stream:
    for line in stream:
        row=json.loads(line)
        if row['simulation_time_s']>last_time:break
        if row['simulation_time_s']>=first_time:rows.append(row)
result={'label':meta['label'],'session_id':meta['session_id'],'trace':trace.name,'phases':[]}
for phase in meta['phases']:
    if phase['name'] not in ('idle','walk_stop','turn_stop'):continue
    end=phase['end']['simulation_time_s']
    start=max(phase['start']['simulation_time_s'],end-5)
    selected=[r for r in rows if start<=r['simulation_time_s']<=end]
    if len(selected)<3:
        raise ValueError(f"Insufficient trace samples for {phase['name']}")
    t=np.array([r['simulation_time_s'] for r in selected])
    q=np.array([r['joint_pos_unitree_order'] for r in selected])
    dq=np.array([r['joint_vel_unitree_order'] for r in selected])
    target=np.array([r['desired_joint_pos_unitree_order'] for r in selected])
    dt=float(np.median(np.diff(t)))
    joints=[]
    for j in np.argsort(np.sqrt(np.mean(dq*dq,axis=0)))[-5:][::-1]:
        power=np.abs(np.fft.rfft(dq[:,j]-dq[:,j].mean()))**2
        freq=np.fft.rfftfreq(len(t),dt)
        joints.append(dict(index=int(j),dq_rms=float(np.sqrt(np.mean(dq[:,j]**2))),dq_mean=float(dq[:,j].mean()),dq_range=float(np.ptp(dq[:,j])),position_derived_dq_rms=float(np.sqrt(np.mean((np.diff(q[:,j])/np.diff(t))**2))),q_range=float(np.ptp(q[:,j])),target_range=float(np.ptp(target[:,j])),peak_hz=float(freq[np.argmax(power)]),power_above_50hz=float(power[freq>50].sum()/max(power.sum(),1e-20))))
    result['phases'].append(dict(name=phase['name'],samples=len(t),dt=dt,max_dq_p95=float(np.quantile(np.max(abs(dq),axis=1),.95)),root_tilt_max=max(r['root_tilt_rad'] for r in selected),joints=joints))
print(json.dumps(result,indent=2))
