"""Recorded PICO diagnostics. MuJoCo is used for FK only, never replacement physics."""
import argparse, hashlib, json, os
from dataclasses import asdict
from pathlib import Path
import numpy as np
from motion_contracts.artifact import (MUJOCO_TO_ISAACLAB as ORDER, SONIC_BODY_NAMES,
    convert_g1_qpos, _blend_qpos, _neutral_qpos, _mujoco_body_kinematics)
from motion_contracts.validator import validate_artifact
MJCF=Path('/opt/GMR/assets/unitree_g1/g1_mocap_29dof.xml')
MOTOR_FROM_ISAAC=np.argsort(ORDER)

def load(path): return np.loadtxt(path,delimiter=',',skiprows=1)
def save(path,value): path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def checksum(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def qpos_from_artifact(path):
    q=load(path/'joint_pos.csv'); pos=load(path/'body_pos.csv').reshape(-1,14,3)[:,0]
    quat=load(path/'body_quat.csv').reshape(-1,14,4)[:,0]
    return np.column_stack((pos,quat,q[:,MOTOR_FROM_ISAAC]))

def trace_rows(path,n):
    started=False;last=-1
    with path.open() as stream:
        for line in stream:
            d=json.loads(line);f=d.get('reference_frame')
            if f is None:continue
            if not isinstance(f,int) or not 0<=f<n:raise ValueError('invalid reference frame')
            if started and f<last:break
            started=True;last=f
            yield d
            # Stop at first endpoint: avoid overweighting idle after playback.
            if f==n-1:break

def phases(manifest):
    n=manifest['num_frames'];fps=manifest['fps'];c=manifest['temporal_conditioning']
    h=round(c['neutral_hold_s_each_end']*fps);t=round(c['neutral_transition_s_each_end']*fps)
    return [('neutral_start',0,h),('entry',h,h+t),('active',h+t,n-h-t),('exit',n-h-t,n-h),('neutral_end',n-h,n)]

def build(exchange,trace):
    src=exchange/'pico-gmr-full-v1';manifest=json.loads((src/'manifest.json').read_text());q=qpos_from_artifact(src)
    active=q[200:-200];work=exchange/'diagnostics/pico/accuracy-v1';work.mkdir(parents=True,exist_ok=False)
    rows=list(trace_rows(trace,len(q)));best={}
    for d in rows:
        f=d['reference_frame']
        if 200<=f<len(q)-200:
            e=float(np.max(np.abs(np.array(d['joint_pos_unitree_order'])-q[f,7:])))
            best[f]=max(e,best.get(f,0))
    selected=[]
    for f in sorted(best,key=best.get,reverse=True):
        if all(abs(f-g)>=100 and np.linalg.norm(q[f,7:]-q[g,7:])>=.3 for g in selected):selected.append(f)
        if len(selected)==4:break
    if len(selected)<4:raise ValueError('insufficient distinct high-error poses')
    # Exact source poses at every second sample; final duplicated endpoint keeps 2x frame count.
    half=np.empty((len(active)*2,36));half[::2]=active
    for i in range(len(active)-1):half[2*i+1]=_blend_qpos(active[i],active[i+1],np.array([.5]))[0]
    half[-1]=active[-1]
    hold_arrays=[];hold_ranges=[];previous=_neutral_qpos(active[0]);offset=0
    for label,target,source_frame in [('neutral',previous,None)]+[(f'pose_{i+1}',q[f],f) for i,f in enumerate(selected)]:
        transition=_blend_qpos(previous,target,np.arange(1,151)/150)
        hold=np.repeat(target[None],250,axis=0);hold_arrays.extend([transition,hold]);offset+=150
        hold_ranges.append(dict(label=label,start=offset+200,end=offset+450,evaluate_start=offset+350,source_frame=source_frame));offset+=250;previous=target
    cases=[('half',half),('holds',np.vstack(hold_arrays))];outputs=[]
    for name,values in cases:
        motion_id='pico-accuracy-'+name+'-v1';csv=work/(name+'.csv');np.savetxt(csv,values,delimiter=',')
        os.environ['G1_MJCF']=str(MJCF)
        dest=exchange/motion_id
        convert_g1_qpos(csv,dest,request_id=motion_id,motion_id=motion_id,source='pico_diagnostic',model=manifest['model'],source_fps=50.,smoothing_sigma_frames=0.,neutral_transition_s=3.,neutral_hold_s=1.,source_sha256=checksum(src/'joint_pos.csv'),source_metadata=dict(parent_motion='pico-gmr-full-v1',experiment=name,selected_frames=selected))
        validation=validate_artifact(dest)
        if not validation.valid:raise ValueError(json.dumps(asdict(validation)))
        extra=dict(parent_motion='pico-gmr-full-v1',experiment=name,hold_ranges=hold_ranges if name=='holds' else [],source_pose_mapping='frame 200+2*i matches original frame 200+i' if name=='half' else None)
        save(dest/'accuracy_experiment.json',extra);outputs.append(dict(motion_id=motion_id,validation=asdict(validation)))
    # Verify the original artifact's FK and the inverse joint permutation.
    pos,quat=_mujoco_body_kinematics(q,MJCF)
    body=load(src/'body_pos.csv').reshape(-1,14,3)
    if not np.allclose(pos,body,atol=1e-7):raise ValueError('source FK does not reproduce body reference')
    if not np.array_equal(q[:,7:][:,ORDER],load(src/'joint_pos.csv')):raise ValueError('joint permutation mismatch')
    result=dict(selected_frames=selected,artifacts=outputs,mjcf_sha256=checksum(MJCF),source_joint_sha256=checksum(src/'joint_pos.csv'),fk_max_error_m=float(np.max(np.abs(pos-body))))
    save(work/'generation.json',result);print(json.dumps(result,indent=2))

def geometry_height(model,data):
    import mujoco
    lows=[];highs=[]
    for i in range(model.ngeom):
        kind=model.geom_type[i];center=data.geom_xpos[i];rot=data.geom_xmat[i].reshape(3,3);size=model.geom_size[i]
        if kind==mujoco.mjtGeom.mjGEOM_PLANE:continue
        if kind==mujoco.mjtGeom.mjGEOM_MESH:
            m=model.geom_dataid[i];start=model.mesh_vertadr[m];count=model.mesh_vertnum[m]
            verts=model.mesh_vert[start:start+count]@rot.T+center
            lows.append(float(verts[:,2].min()));highs.append(float(verts[:,2].max()));continue
        if kind==mujoco.mjtGeom.mjGEOM_BOX:extent=np.abs(rot)@size
        elif kind==mujoco.mjtGeom.mjGEOM_SPHERE:extent=np.repeat(size[0],3)
        elif kind==mujoco.mjtGeom.mjGEOM_CAPSULE:extent=np.abs(rot[:,2])*size[1]+size[0]
        elif kind==mujoco.mjtGeom.mjGEOM_CYLINDER:extent=np.sqrt(1-rot[:,2]**2)*size[0]+np.abs(rot[:,2])*size[1]
        elif kind==mujoco.mjtGeom.mjGEOM_ELLIPSOID:extent=np.sqrt(np.sum((rot*size)**2,axis=1))
        else:raise ValueError('unsupported model geometry')
        lows.append(float(center[2]-extent[2]));highs.append(float(center[2]+extent[2]))
    return max(highs)-min(lows)

def analyze(artifact,report,trace,output):
    import mujoco
    from scipy.spatial.transform import Rotation
    from motion_contracts.validator import G1_ISAACLAB_JOINT_NAMES
    manifest=json.loads((artifact/'manifest.json').read_text());reference=qpos_from_artifact(artifact)
    rows=list(trace_rows(trace,len(reference)))
    if not rows:raise ValueError('no execution samples')
    frames=np.array([d['reference_frame'] for d in rows]);actual=np.array([d['joint_pos_unitree_order'] for d in rows])
    roots=np.array([d['root_state_w'][:7] for d in rows]);actual_qpos=np.column_stack((roots,actual))
    actual_pos,_=_mujoco_body_kinematics(actual_qpos,MJCF);ref_pos,_=_mujoco_body_kinematics(reference[frames],MJCF)
    ids=[SONIC_BODY_NAMES.index(n) for n in ['left_wrist_yaw_link','right_wrist_yaw_link','left_ankle_roll_link','right_ankle_roll_link']]
    model=mujoco.MjModel.from_xml_path(str(MJCF));data=mujoco.MjData(model);data.qpos[:]=_neutral_qpos(reference[0]);mujoco.mj_forward(model,data);height=geometry_height(model,data)
    ar=Rotation.from_quat(roots[:,[4,5,6,3]]).as_matrix();rr=Rotation.from_quat(reference[frames][:,[4,5,6,3]]).as_matrix()
    local_actual=np.einsum('nji,nkj->nki',ar,actual_pos[:,ids]-roots[:,None,:3]);local_ref=np.einsum('nji,nkj->nki',rr,ref_pos[:,ids]-reference[frames,None,:3])
    world=np.linalg.norm(actual_pos[:,ids]-ref_pos[:,ids],axis=2);local=np.linalg.norm(local_actual-local_ref,axis=2)
    # Supplemental initial-frame alignment, never a best-fit trajectory alignment.
    # SONIC initializes heading relative to the robot; root translation is not a G1 target.
    yaw_actual=np.arctan2(ar[0,1,0],ar[0,0,0]);yaw_ref=np.arctan2(rr[0,1,0],rr[0,0,0])
    align=Rotation.from_euler('z',yaw_actual-yaw_ref).as_matrix()
    offset=roots[0,:3]-align@reference[frames[0],:3];offset[2]=0.
    aligned_ref=ref_pos[:,ids]@align.T+offset
    aligned_world=np.linalg.norm(actual_pos[:,ids]-aligned_ref,axis=2)
    root_delta=roots[:,:3]-reference[frames,:3]
    root_angle=(Rotation.from_matrix(ar).inv()*Rotation.from_matrix(rr)).magnitude()
    error=actual-reference[frames,7:];names=[G1_ISAACLAB_JOINT_NAMES[i] for i in MOTOR_FROM_ISAAC]
    def metrics(mask):
        e=error[mask];result=dict(samples=int(mask.sum()))
        if not len(e):return result
        result.update(rmse_deg=float(np.degrees(np.sqrt(np.mean(e*e)))),peak_deg=float(np.degrees(np.max(np.abs(e)))),per_joint={n:dict(rmse_deg=float(np.degrees(np.sqrt(np.mean(e[:,i]**2)))),bias_deg=float(np.degrees(e[:,i].mean())),peak_deg=float(np.degrees(np.max(np.abs(e[:,i]))))) for i,n in enumerate(names)})
        result['root']=dict(xy_rmse_m=float(np.sqrt(np.mean(np.sum(root_delta[mask,:2]**2,axis=1)))),height_bias_m=float(root_delta[mask,2].mean()),orientation_rmse_deg=float(np.degrees(np.sqrt(np.mean(root_angle[mask]**2)))))
        result['endpoints']={SONIC_BODY_NAMES[j]:dict(world_rmse_m=float(np.sqrt(np.mean(world[mask,i]**2))),pelvis_rmse_m=float(np.sqrt(np.mean(local[mask,i]**2))),initial_aligned_world_rmse_m=float(np.sqrt(np.mean(aligned_world[mask,i]**2))),world_normalized=float(np.sqrt(np.mean(world[mask,i]**2))/height),pelvis_normalized=float(np.sqrt(np.mean(local[mask,i]**2))/height)) for i,j in enumerate(ids)}
        subset=[d for d,m in zip(rows,mask) if m];result['applied_torque_ratio_peak']=max(d['max_torque_limit_ratio'] for d in subset)
        commands=[d for d in subset if d.get('desired_joint_pos_unitree_order') is not None]
        if commands:
            cmd=np.array([d['desired_joint_pos_unitree_order'] for d in commands]);obs=np.array([d['joint_pos_unitree_order'] for d in commands]);ref=reference[[d['reference_frame'] for d in commands],7:]
            result['command_decomposition']=dict(reference_to_command_rmse_deg=float(np.degrees(np.sqrt(np.mean((cmd-ref)**2)))),command_to_actual_rmse_deg=float(np.degrees(np.sqrt(np.mean((obs-cmd)**2)))),per_joint_mean_deg={n:dict(reference=float(np.degrees(ref[:,i].mean())),command=float(np.degrees(cmd[:,i].mean())),actual=float(np.degrees(obs[:,i].mean()))) for i,n in enumerate(names)},caveat='LowCmd PD setpoints can intentionally differ from reference posture; RMS terms are not additive causal contributions.')
        saturated=[np.max(np.abs(np.array(d['requested_torque_unitree_order_nm'])-d['applied_torque_unitree_order_nm']))>1e-6 for d in subset if 'requested_torque_unitree_order_nm' in d]
        result['saturated_sample_fraction']=float(np.mean(saturated)) if saturated else None
        return result
    extra_path=artifact/'accuracy_experiment.json';extra=json.loads(extra_path.read_text()) if extra_path.exists() else {}
    active_phase=next((a,b) for name,a,b in phases(manifest) if name=='active')
    active_mask=(frames>=active_phase[0])&(frames<active_phase[1])
    matched_mask=active_mask & (((frames-active_phase[0])%2==0) if extra.get('experiment')=='half' else np.ones(len(rows),dtype=bool))
    lag_mask=(frames>=active_phase[0]+25)&(frames<active_phase[1]-25)
    lag=[]
    for shift in range(-25,26):
        e=actual[lag_mask]-reference[frames[lag_mask]-shift,7:]
        lag.append(float(np.degrees(np.sqrt(np.mean(e*e)))))
    lag_index=int(np.argmin(lag))
    phase_metrics={name:metrics((frames>=a)&(frames<b)) for name,a,b in phases(manifest)}
    holds={h['label']:metrics((frames>=h['evaluate_start'])&(frames<h['end'])) for h in extra.get('hold_ranges',[])}
    for h in extra.get('hold_ranges',[]):
        holds[h['label']]['before_next_transition_lookahead']=metrics((frames>=h['evaluate_start'])&(frames<h['end']-45))
        mask=(frames>=h['evaluate_start'])&(frames<h['end']);values=np.array([d['joint_vel_unitree_order'] for d,m in zip(rows,mask) if m]);holds[h['label']]['settled_max_joint_speed_rad_s']=float(np.max(np.abs(values))) if len(values) else None
    execution=json.loads(report.read_text());result=dict(motion_id=manifest['motion_id'],report=report.name,result=execution['result'],reference_sha256=checksum(artifact/'joint_pos.csv'),mjcf_sha256=checksum(MJCF),nominal_geometry_height_m=height,normalization='Full robot geometry vertical extent in official neutral pose, excluding ground plane',world_alignment='Raw native world plus separately labeled first-frame yaw/XY alignment; no fitted trajectory alignment. Exact SONIC initial heading buffer is not logged.',realtime_factor=execution.get('performance',{}).get('unsupported_playback_realtime_factor'),all=metrics(np.ones(len(rows),dtype=bool)),source_matched_active=metrics(matched_mask),lag_diagnostic=dict(best_shift_sim_ms=(lag_index-25)*20,unshifted_rmse_deg=lag[25],aligned_rmse_deg=lag[lag_index],scope='fixed active interior; not sensor latency'),phases=phase_metrics,holds=holds,limitations=['Kinematic endpoint comparison uses the same reference model for both states; does not validate deployed geometry equivalence.','Unshifted metrics; trace-rate samples can miss extrema.'])
    save(output,result);print(json.dumps({k:result[k] for k in ['motion_id','result','realtime_factor','nominal_geometry_height_m']}))

def screen(artifact,output):
    """Conservative static screen; being inside both-foot hull does not prove balance."""
    import mujoco
    from scipy.spatial import ConvexHull
    q=qpos_from_artifact(artifact);meta=json.loads((artifact/'accuracy_experiment.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(MJCF));data=mujoco.MjData(model)
    root=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'pelvis')
    feet=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ['left_ankle_roll_link','right_ankle_roll_link']]
    results=[]
    for h in meta['hold_ranges']:
        data.qpos[:]=q[h['start']];mujoco.mj_forward(model,data);points=[];bottoms=[]
        for i in range(model.ngeom):
            if model.geom_bodyid[i] not in feet or model.geom_contype[i]==0:continue
            if model.geom_type[i]!=mujoco.mjtGeom.mjGEOM_SPHERE:raise ValueError('unexpected foot collision geometry')
            radius=model.geom_size[i,0];bottoms.append(float(data.geom_xpos[i,2]-radius))
            for x,y in [(-radius,-radius),(-radius,radius),(radius,-radius),(radius,radius)]:points.append(data.geom_xpos[i,:2]+[x,y])
        hull=ConvexHull(points);com=data.subtree_com[root,:2];signed=hull.equations[:,:2]@com+hull.equations[:,2]
        results.append(dict(pose=h['label'],source_frame=h['source_frame'],foot_bottom_min_m=min(bottoms),foot_bottom_max_m=max(bottoms),com_xy_m=com.tolist(),outside_even_all_feet_hull=bool(max(signed)>0),max_outside_supporting_plane_m=float(max(signed))))
    save(output,dict(scope='reference-model static screen; conservative hull includes both entire feet irrespective of contact height, not a dynamics proof',poses=results))

def main():
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    b=sub.add_parser('build');b.add_argument('--exchange',type=Path,required=True);b.add_argument('--trace',type=Path,required=True)
    a=sub.add_parser('analyze');a.add_argument('--artifact',type=Path,required=True);a.add_argument('--report',type=Path,required=True);a.add_argument('--trace',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('screen');c.add_argument('--artifact',type=Path,required=True);c.add_argument('--output',type=Path,required=True)
    args=vars(p.parse_args());command=args.pop('command');globals()[command](**args)
if __name__=='__main__':main()
