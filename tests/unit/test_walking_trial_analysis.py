import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('walking_analysis',Path(__file__).resolve().parents[2]/'tools/analysis/analyze_walking_trial.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class WalkingAnalysisTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.path=Path(tmp.name)/'trace.jsonl'
        endpoint=dict(session_id='s',state='INTERACTIVE',trace_path='/exchange/trace.jsonl')
        self.report=dict(completed=True,session_id='s',with_gesture=False,events=[dict(name='walk',
            before=dict(endpoint,simulation_time_s=0.),after=dict(endpoint,simulation_time_s=.015))])
        self.rows=[]
        for i in range(3):
            self.rows.append(dict(simulation_time_s=i*.005,root_state_w=[0.]*7+[.6 if i==1 else .2,0.,0.],
                lowcmd_age_s=.02,root_tilt_rad=.03,max_torque_limit_ratio=.4,joint_vel_unitree_order=[.1]*29,
                foot_contact_measurements=dict(rows=[[0.]*15+[100.],[0.]*15+[100.]]),
                command_timing_cumulative=dict(interval_counts_over=[0,0,0,0])))
        self.save()
    def save(self):self.path.write_text(''.join(json.dumps(x)+'\n' for x in self.rows))
    def test_completed_run_still_reports_between_status_peak(self):
        result=module.analyze(self.report,self.path)['phases'][0]
        self.assertEqual(result['planar_bound_exceeded_samples'],1)
        self.assertEqual(result['planar_speed_max_m_s'],.6)
        self.assertAlmostEqual(result['max_trace_gap_s'],.005)
    def test_reject_wrong_session_and_incomplete(self):
        self.report['completed']=False
        with self.assertRaises(ValueError):module.analyze(self.report,self.path)
        self.report['completed']=True
        self.report['events'][0]['after']['session_id']='other'
        with self.assertRaises(ValueError):module.analyze(self.report,self.path)
    def test_reject_nonmonotonic_trace(self):
        self.rows[2]['simulation_time_s']=.005;self.save()
        with self.assertRaises(ValueError):module.analyze(self.report,self.path)


if __name__=='__main__':unittest.main()
