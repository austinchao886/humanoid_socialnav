#!/usr/bin/env python3
"""Compare native Isaac samples with the recorded reference frame, without time shifting."""
import argparse
import csv
import json
import math
from pathlib import Path
from motion_contracts.validator import G1_ISAACLAB_JOINT_NAMES

HARDWARE = [f'{side}_{joint}_joint' for side in ('left','right')
            for joint in ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')]
HARDWARE += [f'waist_{axis}_joint' for axis in ('yaw','roll','pitch')]
HARDWARE += [f'{side}_{joint}_joint' for side in ('left','right')
             for joint in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')]

def main():
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('trace',type=Path)
    p.add_argument('reference',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    report=json.loads(a.report.read_text())
    with a.reference.open() as stream:
        reader=csv.reader(stream);header=next(reader)
        if header!=[f'joint_{i}' for i in range(29)]:raise ValueError('unexpected reference CSV schema')
        refs=[[float(v) for v in row] for row in reader]
    indices=[G1_ISAACLAB_JOINT_NAMES.index(n) for n in HARDWARE]
    errors=[];peak_tau=0.;peak_tilt=0.;samples=0
    with a.trace.open() as stream:
        for line in stream:
            sample=json.loads(line);frame=sample.get('reference_frame')
            if frame is None or not 0<=frame<len(refs):continue
            actual=sample['joint_pos_unitree_order']
            if len(actual)!=29:raise ValueError('unexpected actual joint count')
            errors.extend(q-refs[frame][j] for q,j in zip(actual,indices));samples+=1
            peak_tau=max(peak_tau,sample.get('max_torque_limit_ratio',0))
            peak_tilt=max(peak_tilt,sample.get('root_tilt_rad',0))
    if not errors or not all(math.isfinite(e) for e in errors):raise ValueError('invalid or absent tracking samples')
    perf=report['performance']
    result=dict(scope='native_Isaac_trace_reference_alignment_no_lag_optimization',
        report=a.report.name,trace=a.trace.name,trace_samples=samples,
        joint_rmse_deg=math.degrees(math.sqrt(sum(e*e for e in errors)/len(errors))),
        peak_joint_error_deg=math.degrees(max(abs(e) for e in errors)),
        observed_peak_torque_ratio=peak_tau,observed_peak_root_tilt_deg=math.degrees(peak_tilt),
        realtime_factor=perf['unsupported_playback_realtime_factor'],
        reference_duration_s=perf['reference_duration_s'],wall_playback_s=perf['reference_playback_wall_s'],
        limitations=['50 Hz trace can miss between-sample extrema; critical checks remain 200 Hz.',
                    'Not live acceptance. Normalized wrist/ankle errors remain unmeasured.'])
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
