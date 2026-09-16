import importlib.util
from pathlib import Path
import sys
import unittest

path = Path(__file__).resolve().parents[2] / "services/sonic_tracker/sonic_tracker/composer.py"
if not path.exists():
    path = Path(__file__).with_name("composer.py")
spec = importlib.util.spec_from_file_location("composer", path)
c = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = c
spec.loader.exec_module(c)


class ComposerTests(unittest.TestCase):
    def sample(self, q=0., dq=0., time=0.):
        return c.JointReference((q,)*29, (dq,)*29, time)

    def test_envelope_endpoints(self):
        self.assertEqual(c.transition_weight(-1, 2), (0, 0))
        self.assertEqual(c.transition_weight(2, 2), (1, 0))
        self.assertAlmostEqual(c.transition_weight(1, 2)[0], .5)

    def test_base_ownership_preserved(self):
        base, gesture = self.sample(.1, .2), self.sample(.5, -.1)
        result = c.compose_right_arm(base, gesture, .5, .2)
        for i in set(range(29)) - set(c.RIGHT_ARM):
            self.assertEqual((result.q[i], result.dq[i]), (.1, .2))
        for i in c.RIGHT_ARM:
            self.assertAlmostEqual(result.q[i], .3)
            self.assertAlmostEqual(result.dq[i], .13)

    def test_velocity_matches_position_derivative_entry_and_exit(self):
        def at(t, exiting):
            base = self.sample(t*.2, .2, t)
            gesture = self.sample(.5-t*.1, -.1, t)
            w, dw = c.transition_weight(t, 2.)
            if exiting:
                w, dw = 1-w, -dw
            return c.compose_right_arm(base, gesture, w, dw)
        for exiting in (False, True):
            for t in (.1, .6, 1.2, 1.9):
                h = 1e-5
                numerical = (at(t+h, exiting).q[12]-at(t-h, exiting).q[12])/(2*h)
                self.assertAlmostEqual(numerical, at(t, exiting).dq[12], places=7)

    def test_endpoint_return_to_base(self):
        base = self.sample(.1, .2)
        self.assertEqual(c.compose_right_arm(base, self.sample(.8, .9), 0, 0), base)

    def test_reject_mismatch_and_invalid_inputs(self):
        with self.assertRaises(ValueError):
            c.compose_right_arm(self.sample(), self.sample(time=1), .5, .1)
        for w, dw in ((-1, 0), (2, 0), (0, .1), (.5, float('nan'))):
            with self.assertRaises(ValueError):
                c.compose_right_arm(self.sample(), self.sample(), w, dw)
        with self.assertRaises(ValueError):
            self.sample(q=float('nan'))
        with self.assertRaises(ValueError):
            c.JointReference((0,)*29, (0,)*29, 0, "mujoco")
        with self.assertRaises(ValueError):
            c.transition_weight(0, 0)


if __name__ == "__main__":
    unittest.main()
