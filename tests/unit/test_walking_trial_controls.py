import importlib.util
from pathlib import Path
import subprocess
import sys
import time
import unittest

PATH=Path(__file__).resolve().parents[2]/'tools/acceptance/walking_gesture_trial.py'
spec=importlib.util.spec_from_file_location('walking_trial',PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class WalkingTrialControls(unittest.TestCase):
    def status(self):
        return dict(root_height_m=.78,root_tilt_rad=.03,max_torque_limit_ratio=.3,
                    updated_epoch_s=time.time(),root_linear_velocity_m_s=[.6,0.,0.])
    def test_velocity_measurement_not_gesture_admission(self):
        d=self.status()
        module.check_measurements(d,baseline_diagnostics=True)
        with self.assertRaises(RuntimeError):module.check_measurements(d)
    def test_diagnostic_rejects_stale_pose_and_torque_faults(self):
        for key,value in [('root_height_m',.4),('root_tilt_rad',.3),
                          ('max_torque_limit_ratio',1.1),('updated_epoch_s',0.),
                          ('root_linear_velocity_m_s',[float('nan'),0.,0.])]:
            d=self.status();d[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                module.check_measurements(d,baseline_diagnostics=True)
    def test_diagnostic_and_gesture_flags_rejected_before_start(self):
        p=subprocess.run([sys.executable,str(PATH),'--baseline-diagnostics','--with-gesture'],capture_output=True,text=True)
        self.assertEqual(p.returncode,2)
        self.assertIn('cannot approve or run a gesture',p.stderr)
    def test_ramp_endpoints_and_monotonicity(self):
        v=[module.ramp(i*.01,4,.25) for i in range(401)]
        self.assertEqual(v[0],.25);self.assertEqual(v[-1],0.)
        self.assertTrue(all(a>=b>=0 for a,b in zip(v,v[1:])))


if __name__=='__main__':unittest.main()
