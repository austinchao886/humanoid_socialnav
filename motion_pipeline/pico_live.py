"""Causal PICO-to-G1 conversion; no controller ownership or hardware access.

Input is a canonical SMPL-X frame (63 body axis angles, 3 root axis angles,
3 translation coordinates in SMPL Y-up metres). Output follows SONIC v1.
"""
from collections import deque
import json
import math
import time
import numpy as np


class FreshnessGate:
    """Session changes, reordering and stale arrivals cannot silently re-arm."""
    def __init__(self, timeout=.25):
        self.timeout = timeout
        self.session = None
        self.sequence = -1
        self.received = None
        self.armed = False

    def accept(self, session, sequence, now):
        if not isinstance(session, str) or not session:
            raise ValueError('missing session')
        if not isinstance(sequence, int) or sequence < 0 or not math.isfinite(now):
            raise ValueError('invalid sequence/time')
        if session != self.session:
            self.session, self.sequence, self.armed = session, -1, False
        if sequence <= self.sequence:
            return False
        if self.received is not None and now - self.received > self.timeout:
            self.armed = False
        self.sequence, self.received = sequence, now
        return True

    def arm(self, now):
        if self.received is None or not 0 <= now-self.received <= self.timeout:
            raise ValueError('fresh input required')
        self.armed = True

    def active(self, now):
        if self.received is None or not 0 <= now-self.received <= self.timeout:
            self.armed = False
        return self.armed


def pack_v1(joint_pos, joint_vel, body_quat, indices):
    arrays = dict(joint_pos=np.asarray(joint_pos, dtype='<f4'),
                  joint_vel=np.asarray(joint_vel, dtype='<f4'),
                  body_quat=np.asarray(body_quat, dtype='<f4'),
                  frame_index=np.asarray(indices, dtype='<i8'))
    n = len(arrays['frame_index'])
    shapes = {'joint_pos': (n,29), 'joint_vel': (n,29), 'body_quat': (n,4), 'frame_index': (n,)}
    if not 1 <= n <= 5:
        raise ValueError('history must contain 1–5 frames')
    for name,a in arrays.items():
        if a.shape != shapes[name] or not np.isfinite(a).all():
            raise ValueError('invalid '+name)
    if np.any(np.diff(arrays['frame_index']) <= 0):
        raise ValueError('nonmonotonic frame indices')
    if np.max(np.abs(np.linalg.norm(arrays['body_quat'],axis=1)-1)) > .01:
        raise ValueError('invalid quaternion')
    fields=[dict(name=k,dtype='i64' if k=='frame_index' else 'f32',shape=list(v.shape)) for k,v in arrays.items()]
    header=json.dumps(dict(v=1,endian='le',count=1,fields=fields),separators=(',',':')).encode()
    if len(header)>1280: raise ValueError('oversized header')
    return b'pose'+header.ljust(1280,b'\0')+b''.join(a.tobytes() for a in arrays.values())


class CausalRetargeter:
    def __init__(self, body_models, human_height=1.66, smoothing_tau=.02):
        import torch
        import smplx
        from scipy.spatial.transform import Rotation
        from general_motion_retargeting import GeneralMotionRetargeting
        self.torch, self.R = torch, Rotation
        torch.set_num_threads(1)
        self.model=smplx.create(body_models,'smplx',gender='neutral',use_pca=False)
        self.retarget=GeneralMotionRetargeting(actual_human_height=human_height,
            src_human='smplx',tgt_robot='unitree_g1',use_velocity_limit=True)
        with torch.inference_mode():
            neutral=self.model(betas=torch.zeros(1,self.model.num_betas),
                return_full_pose=True,return_verts=False)
        self.rest=neutral.joints.detach().numpy()[0,:len(self.model.parents)]
        self.parents=self.model.parents.tolist()
        self.up=Rotation.from_euler('x',90,degrees=True)
        self.tau=smoothing_tau
        self.previous=None
        self.timestamp=None
        self.origin=None
        self.history=deque(maxlen=5)
        self.index=0

    def process(self, pose_body, root_orient, trans, timestamp):
        from smplx.joint_names import JOINT_NAMES
        from motion_contracts.artifact import MUJOCO_TO_ISAACLAB
        pose=np.asarray(pose_body,dtype=np.float32).reshape(-1)
        root=np.asarray(root_orient,dtype=np.float32)
        trans=np.asarray(trans,dtype=np.float32)
        if pose.shape!=(63,) or root.shape!=(3,) or trans.shape!=(3,):
            raise ValueError('invalid canonical frame shape')
        if not all(np.isfinite(x).all() for x in [pose,root,trans]) or not math.isfinite(timestamp):
            raise ValueError('nonfinite canonical frame')
        dt=.02 if self.timestamp is None else timestamp-self.timestamp
        if not 0 < dt <= .25: raise ValueError('discontinuous source clock; recalibrate')
        if self.origin is None: self.origin=trans.copy()
        # Shape is constant within a session. Forward kinematics of the cached
        # neutral joints exactly reproduces SMPL-X skeleton joints without LBS.
        local=np.zeros((len(self.parents),3));local[0]=root;local[1:22]=pose.reshape(21,3)
        rotations=[];positions=[];frame={}
        for i,parent in enumerate(self.parents):
            rot=self.R.from_rotvec(local[i])
            if parent<0:
                pos=self.rest[i]+trans-self.origin
            else:
                pos=positions[parent]+rotations[parent].apply(self.rest[i]-self.rest[parent])
                rot=rotations[parent]*rot
            positions.append(pos);rotations.append(rot)
            frame[JOINT_NAMES[i]]=(self.up.apply(pos),(self.up*rot).as_quat()[[3,0,1,2]])
        qpos=np.array(self.retarget.retarget(frame),dtype=float,copy=True)
        if qpos.shape!=(36,) or not np.isfinite(qpos).all(): raise ValueError('invalid GMR output')
        joint=qpos[7:][MUJOCO_TO_ISAACLAB]
        if self.previous is None:
            velocity=np.zeros(29)
        else:
            alpha=1-math.exp(-dt/self.tau) if self.tau>0 else 1.
            joint=self.previous+alpha*(joint-self.previous)
            velocity=(joint-self.previous)/dt
        self.previous=joint.copy();self.timestamp=timestamp
        quat=qpos[3:7]/np.linalg.norm(qpos[3:7])
        self.history.append((joint,velocity,quat,self.index));self.index+=1
        packet=pack_v1(*[np.stack([r[i] for r in self.history]) for i in range(4)])
        return packet,dict(joint_pos=joint.tolist(),joint_vel=velocity.tolist(),
            body_quat=quat.tolist(),source_time_s=timestamp,frame_index=self.index-1)
