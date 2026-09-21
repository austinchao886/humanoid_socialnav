import hashlib,json,os,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from sonic_tracker.pico_replay import load_recording,pack,Watchdog,Publisher,poll
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)
  for i in range(5):
   self.arrays=dict(smpl_pose=np.zeros((5,21,3),np.float32),smpl_joints=np.zeros((5,24,3),np.float32),body_quat_w=np.tile([1.,0,0,0],(5,1)),joint_pos=np.zeros((5,29)),joint_vel=np.zeros((5,29)),frame_index=np.arange(i,i+5,dtype=np.int64))
   np.savez(self.path/f'pose_{i:06d}.npz',**self.arrays)
 def tearDown(self):self.tmp.cleanup()
 def sha(self):
  h=hashlib.sha256()
  for f in sorted(self.path.glob('pose_*.npz')):h.update(f.name.encode()+b'\0'+f.read_bytes())
  return h.hexdigest()
 def test_native_trial_disabled_by_default(self):
  with patch.dict(os.environ, {'SONIC_ENABLE_PICO_SMPL_TRIAL':'0'}):poll(None)
 def test_valid_and_packet(self):
  packets,report=load_recording(self.path,self.sha());self.assertEqual(len(packets),5)
  header=json.loads(packets[0][4:1284].rstrip(b'\0'));self.assertEqual(header['v'],3)
  size=sum(np.prod(f['shape'])*{'f32':4,'f64':8,'i64':8}[f['dtype']] for f in header['fields'])
  self.assertEqual(len(packets[0]),1284+size)
 def test_checksum_rejected(self):
  with self.assertRaisesRegex(ValueError,'checksum'):load_recording(self.path,'bad')
 def test_nan_rejected(self):
  self.arrays['smpl_pose'][0,0,0]=np.nan;np.savez(self.path/'pose_000004.npz',**self.arrays)
  with self.assertRaisesRegex(ValueError,'Nonfinite'):load_recording(self.path,self.sha())
 def test_bad_quaternion_rejected(self):
  self.arrays['body_quat_w'][:]=0;np.savez(self.path/'pose_000004.npz',**self.arrays)
  with self.assertRaisesRegex(ValueError,'orientation'):load_recording(self.path,self.sha())
 def test_repeated_frame_rejected(self):
  self.arrays['frame_index']=np.arange(3,8,dtype=np.int64);np.savez(self.path/'pose_000004.npz',**self.arrays)
  with self.assertRaisesRegex(ValueError,'Noncontiguous'):load_recording(self.path,self.sha())
 def test_stall_triggers_watchdog_once(self):
  calls=[];w=Watchdog(lambda:calls.append(time.monotonic()));w.start();time.sleep(.4);w.close()
  self.assertEqual(len(calls),1);self.assertGreaterEqual(w.fired['gap_s'],.25);self.assertLess(w.fired['gap_s'],.4)
 def test_fresh_frames_do_not_trigger(self):
  calls=[];w=Watchdog(lambda:calls.append(1));w.start()
  for _ in range(8):time.sleep(.05);w.feed()
  w.close();self.assertEqual(calls,[])
if __name__=='__main__':unittest.main()
