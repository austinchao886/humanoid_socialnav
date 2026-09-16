import copy
import time
import unittest
from sonic_tracker.gesture_safety import require_composition_envelope


class GaitEnvelopeTests(unittest.TestCase):
    def status(self):
        return dict(session_id='s',simulation_time_s=10.,updated_epoch_s=time.time(),
            root_height_m=.78,root_tilt_rad=.04,root_linear_velocity_m_s=[.55,0.,.16],max_torque_limit_ratio=.4,
            kinematic_window=dict(schema_version=1,session_id='s',ready=True,error=None,
                sample_count=201,start_simulation_s=9.,end_simulation_s=10.,span_s=1.,
                max_sample_gap_s=.005,mean_window_s=.5,mean_planar_speed_m_s=.44,
                peak_planar_speed_m_s=.56,peak_abs_vertical_speed_m_s=.17,
                min_height_m=.76,max_height_m=.79,peak_tilt_rad=.08,peak_torque_ratio=.5))
    def check(self,s):require_composition_envelope(s,now_epoch_s=time.time(),gait_window=True)
    def test_explicit_opt_in_does_not_change_default(self):
        s=self.status();self.check(s)
        with self.assertRaises(RuntimeError):require_composition_envelope(s,now_epoch_s=time.time())
    def test_mean_does_not_hide_a_peak(self):
        for key,value in [('mean_planar_speed_m_s',.51),('peak_planar_speed_m_s',.66),
                          ('peak_abs_vertical_speed_m_s',.21),('peak_tilt_rad',.21),
                          ('peak_torque_ratio',1.01),('min_height_m',.69)]:
            s=self.status();s['kinematic_window'][key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):self.check(s)
    def test_reject_missing_stale_gapped_cross_session_window(self):
        for key,value in [('ready',False),('session_id','other'),('sample_count',True),
                          ('sample_count',10),('max_sample_gap_s',.02),('end_simulation_s',9.9),
                          ('mean_window_s',1.),('schema_version',True),('error','bad')]:
            s=self.status();s['kinematic_window'][key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):self.check(s)
    def test_reject_every_missing_or_nonfinite_numeric_measurement(self):
        original=self.status()
        for key,value in original['kinematic_window'].items():
            if type(value) not in (int,float):continue
            for bad in (None,float('nan'),float('inf'),True):
                s=copy.deepcopy(original);s['kinematic_window'][key]=bad
                with self.subTest(key=key,bad=bad),self.assertRaises(RuntimeError):self.check(s)
    def test_current_pose_and_status_freshness_still_required(self):
        for key,value in [('root_tilt_rad',.3),('updated_epoch_s',0.),('max_torque_limit_ratio',1.1),
                          ('root_linear_velocity_m_s',[.7,0.,0.])]:
            s=self.status();s[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):self.check(s)

    def test_float_time_roundoff_not_real_future_or_stale_window(self):
        s=self.status();s['simulation_time_s']=112.865
        w=s['kinematic_window'];w.update(start_simulation_s=111.86500000000001,end_simulation_s=112.86500000000001)
        self.check(s)
        for delta in (-.001,.011):
            bad=copy.deepcopy(s);bad['simulation_time_s']=w['end_simulation_s']+delta
            with self.subTest(delta=delta),self.assertRaises(RuntimeError):self.check(bad)


if __name__=='__main__':unittest.main()
