import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT/'tools/analysis/analyze_stop_failure.py'
if not MODULE.exists():
    MODULE = Path(__file__).with_name('analyze_stop_failure.py')
spec = importlib.util.spec_from_file_location('stop_analysis', MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StopFailureTests(unittest.TestCase):
    def rows(self):
        return [dict(simulation_time_s=i*.005, desired_joint_pos_unitree_order=[0.]*29,
                     joint_pos_unitree_order=[0.]*29, joint_vel_unitree_order=[0.]*29,
                     root_tilt_rad=.02, lowcmd_age_s=.01,
                     foot_contact_measurements={'rows':[[0.]*15+[30.], [0.]*15+[0.]]})
                for i in range(601)]

    def test_between_status_spike_and_joint_identity(self):
        rows = self.rows()
        rows[250]['desired_joint_pos_unitree_order'][7] = .2
        result = module.summarize(rows)
        self.assertEqual(result['raw_target_step_max_rad_by_joint'][7], .2)
        self.assertEqual(result['raw_target_step_max_rad_by_joint'][6], 0)
        self.assertEqual(result['force_contact_sample_counts']['1'], 601)

    def test_gap_and_nonfinite_rejected(self):
        rows = self.rows(); del rows[10]
        with self.assertRaises(ValueError): module.summarize(rows)
        rows = self.rows(); rows[5]['joint_vel_unitree_order'][0] = float('nan')
        with self.assertRaises(ValueError): module.summarize(rows)

    def test_failure_excludes_deadman_recovery(self):
        rows = self.rows(); rows[-1]['root_tilt_rad'] = 99.
        with tempfile.TemporaryDirectory() as folder:
            trace = Path(folder)/'trace.jsonl'
            trace.write_text(''.join(json.dumps(x)+'\n' for x in rows))
            state = dict(session_id='s', state='INTERACTIVE', trace_path='/exchange/trace.jsonl')
            report = dict(completed=False, error='tilt', session_id='s',
                          events=[dict(name='decelerate', before=dict(state, simulation_time_s=2.))],
                          last_observed_status=dict(state, simulation_time_s=3.))
            result = module.analyze(report, trace)
            self.assertEqual(result['before_rejection']['max_tilt_rad'], .02)
            self.assertEqual(result['before_rejection']['samples'], 200)
            report['last_observed_status']['session_id'] = 'changed'
            with self.assertRaises(ValueError): module.analyze(report, trace)

    def test_completed_trial_is_not_failed_evidence(self):
        with self.assertRaises(ValueError): module.analyze({'completed':True}, Path('none'))


if __name__ == '__main__': unittest.main()
