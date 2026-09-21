import importlib.util,json,tempfile,unittest
from pathlib import Path
import numpy as np
AVAILABLE=Path('/motion_exchange/pico-accuracy-half-v1/manifest.json').exists()
a=None
if AVAILABLE:
 spec=importlib.util.spec_from_file_location('accuracy',Path(__file__).resolve().parents[2]/'tools/analysis/pico_accuracy.py');a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
@unittest.skipUnless(AVAILABLE,'requires generated accuracy artifacts in pinned GMR container')
class AccuracyTests(unittest.TestCase):
 def test_half_preserves_source_and_derivatives(self):
  root=Path('/motion_exchange');source=a.qpos_from_artifact(root/'pico-gmr-full-v1')[200:-200];half=a.qpos_from_artifact(root/'pico-accuracy-half-v1')[200:-200]
  np.testing.assert_allclose(half[::2],source,atol=1e-12)
  q=a.load(root/'pico-accuracy-half-v1/joint_pos.csv');v=a.load(root/'pico-accuracy-half-v1/joint_vel.csv');np.testing.assert_allclose(v,np.gradient(q,.02,axis=0),atol=1e-10)
  np.testing.assert_allclose(np.linalg.norm(half[:,3:7],axis=1),1,atol=1e-12)
 def test_world_translation_does_not_change_pelvis_error(self):
  artifact=Path('/motion_exchange/pico-gmr-full-v1');q=a.qpos_from_artifact(artifact)
  with tempfile.TemporaryDirectory() as t:
   t=Path(t);trace=t/'trace.jsonl';report=t/'report.json';out=t/'out.json'
   rows=[]
   for f in [0,49,50,199,200,500,1696,1697,1847,1896]:
    root=q[f,:7].copy();root[0]+=.1
    rows.append(dict(reference_frame=f,joint_pos_unitree_order=q[f,7:].tolist(),joint_vel_unitree_order=[0.]*29,root_state_w=root.tolist(),max_torque_limit_ratio=0,requested_torque_unitree_order_nm=[0.]*29,applied_torque_unitree_order_nm=[0.]*29))
   trace.write_text(''.join(json.dumps(d)+'\n' for d in rows));report.write_text(json.dumps(dict(result='COMPLETED',performance={})))
   a.analyze(artifact,report,trace,out);d=json.loads(out.read_text());self.assertAlmostEqual(d['all']['rmse_deg'],0)
   for value in d['all']['endpoints'].values():
    self.assertAlmostEqual(value['world_rmse_m'],.1,places=8);self.assertAlmostEqual(value['pelvis_rmse_m'],0,places=8)
 def test_hold_windows_are_five_seconds(self):
  p=Path('/motion_exchange/pico-accuracy-holds-v1');info=json.loads((p/'accuracy_experiment.json').read_text());q=a.qpos_from_artifact(p)
  for h in info['hold_ranges']:
   self.assertEqual(h['end']-h['start'],250);self.assertEqual(h['end']-h['evaluate_start'],100)
   np.testing.assert_allclose(q[h['start']:h['end']],np.repeat(q[h['start']][None],250,axis=0),atol=1e-12)
 def test_unsettled_hold_and_hidden_requested_effort_remain_visible(self):
  artifact=Path('/motion_exchange/pico-accuracy-holds-v1');q=a.qpos_from_artifact(artifact);holds=json.loads((artifact/'accuracy_experiment.json').read_text())['hold_ranges']
  with tempfile.TemporaryDirectory() as directory:
   t=Path(directory);rows=[]
   for i,h in enumerate(holds):
    for sample,f in enumerate([h['evaluate_start'],h['end']-1]):
     root=q[f,:7].tolist()+[0.,0.,0.,0.,0.,.21 if i==0 else 0.]
     req=[0.]*29;app=[0.]*29
     if sample:req[0]=4.;app[0]=3.
     rows.append(dict(reference_frame=f,joint_pos_unitree_order=q[f,7:].tolist(),joint_vel_unitree_order=[0.]*29,root_state_w=root,root_tilt_rad=.1,max_torque_limit_ratio=1.,requested_torque_unitree_order_nm=req,applied_torque_unitree_order_nm=app))
   (t/'trace').write_text(''.join(json.dumps(d)+'\n' for d in rows));(t/'report').write_text(json.dumps(dict(result='COMPLETED',performance={})))
   a.analyze(artifact,t/'report',t/'trace',t/'out');d=json.loads((t/'out').read_text())
   self.assertFalse(d['holds']['neutral']['settling']['passed'])
   self.assertTrue(d['holds']['pose_1']['settling']['passed'])
   effort=d['all']['per_joint_effort']['left_hip_pitch_joint']
   self.assertEqual(effort['requested_abs_peak_nm'],4.);self.assertEqual(effort['applied_abs_peak_nm'],3.)
   self.assertEqual(effort['clipped_sample_fraction'],.5);self.assertEqual(effort['max_removed_nm'],1.)
 def test_long_hold_evaluation_excludes_later_motion(self):
  artifact=Path('/motion_exchange/pico-accuracy-longholds-v1')
  if not artifact.exists():self.skipTest('requires long-hold diagnostic artifact')
  q=a.qpos_from_artifact(artifact);holds=json.loads((artifact/'accuracy_experiment.json').read_text())['hold_ranges']
  with tempfile.TemporaryDirectory() as directory:
   t=Path(directory);rows=[]
   for h in holds:
    self.assertGreaterEqual(h['end']-h['evaluate_end'],150)
    for f in [h['evaluate_start'],h['evaluate_end']-1,h['evaluate_end']+1]:
     actual=q[f,7:].copy()
     if f>h['evaluate_end']:actual[0]+=1.
     rows.append(dict(reference_frame=f,joint_pos_unitree_order=actual.tolist(),joint_vel_unitree_order=[0.]*29,root_state_w=q[f,:7].tolist()+[0.]*6,root_tilt_rad=.1,max_torque_limit_ratio=0.,requested_torque_unitree_order_nm=[0.]*29,applied_torque_unitree_order_nm=[0.]*29))
   (t/'trace').write_text(''.join(json.dumps(d)+'\n' for d in rows));(t/'report').write_text(json.dumps(dict(result='COMPLETED',performance={})))
   a.analyze(artifact,t/'report',t/'trace',t/'out');d=json.loads((t/'out').read_text())
   self.assertGreater(d['all']['rmse_deg'],0.)
   for h in d['holds'].values():self.assertEqual(h['samples'],2);self.assertEqual(h['rmse_deg'],0.);self.assertTrue(h['settling']['passed'])
if __name__=='__main__':unittest.main()
