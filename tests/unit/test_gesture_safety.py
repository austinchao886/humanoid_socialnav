import unittest
from sonic_tracker.gesture_safety import require_composition_envelope


class CompositionEnvelopeTests(unittest.TestCase):
    def status(self,**changes):
        result=dict(updated_epoch_s=100.,root_height_m=.78,root_tilt_rad=.02,
                    root_linear_velocity_m_s=[.3,0.,0.],max_torque_limit_ratio=.3)
        result.update(changes)
        return result
    def test_standing_and_slow_walking_allowed(self):
        for speed in (0.,.3,.5):
            require_composition_envelope(self.status(root_linear_velocity_m_s=[speed,0.,0.]),now_epoch_s=100.5)
    def test_outside_bounds_rejected(self):
        for changes in (dict(root_height_m=.69),dict(root_height_m=.91),dict(root_tilt_rad=.26),
                        dict(root_linear_velocity_m_s=[.4,.4,0.]),dict(root_linear_velocity_m_s=[0.,0.,.16]),
                        dict(max_torque_limit_ratio=1.01),dict(updated_epoch_s=98.),dict(updated_epoch_s=101.)):
            with self.subTest(changes=changes),self.assertRaises(RuntimeError):
                require_composition_envelope(self.status(**changes),now_epoch_s=100.)
    def test_missing_nonfinite_and_boolean_rejected(self):
        for changes in (dict(root_height_m=None),dict(root_tilt_rad=float("nan")),
                        dict(max_torque_limit_ratio=True),dict(root_linear_velocity_m_s=[0.,float("inf"),0.])):
            with self.subTest(changes=changes),self.assertRaises(RuntimeError):
                require_composition_envelope(self.status(**changes),now_epoch_s=100.)


if __name__=="__main__":unittest.main()
