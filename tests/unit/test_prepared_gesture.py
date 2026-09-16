import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.prepared_gesture import prepare_gesture


class PreparedGestureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        (self.path/"manifest.json").write_text(json.dumps(dict(
            motion_id="wave", joint_order="g1_29dof_isaaclab", fps=2, num_frames=3)))
        (self.path/"validation.json").write_text('{"valid":true,"errors":[]}')
        for name, values in (("joint_pos.csv", (0,1,0)), ("joint_vel.csv", (0,0,0))):
            with (self.path/name).open("w") as f:
                prefix="joint_vel" if name == "joint_vel.csv" else "joint"
                writer=csv.writer(f); writer.writerow([f"{prefix}_{i}" for i in range(29)])
                writer.writerows([(v,)*29 for v in values])

    def prepare(self, **kwargs):
        return prepare_gesture(self.path, amplitude=kwargs.get("amplitude", .25), time_scale=2.)

    def test_derivative_and_amplitude(self):
        gesture=self.prepare()
        self.assertEqual(gesture.duration_s, 2.)
        self.assertEqual(gesture.sample(1,100).q[12], .25)
        for t in (.1,.4,.8,1.2,1.8):
            h=1e-5
            derivative=(gesture.sample(t+h,0).q[12]-gesture.sample(t-h,0).q[12])/(2*h)
            self.assertAlmostEqual(derivative, gesture.sample(t,0).dq[12], places=7)

    def test_hash_changes_and_immutable_snapshot(self):
        gesture=self.prepare()
        self.assertEqual(gesture.content_id,self.prepare().content_id)
        self.assertNotEqual(gesture.content_id,self.prepare(amplitude=.5).content_id)
        (self.path/"joint_pos.csv").write_text("broken")
        self.assertEqual(gesture.sample(1,0).q[12],.25)
        with self.assertRaises(ValueError): self.prepare()

    def test_invalid_validation_and_parameters(self):
        for value in (0,-1,2,float("nan")):
            with self.assertRaises(ValueError): self.prepare(amplitude=value)
        (self.path/"validation.json").write_text('{"valid":false}')
        with self.assertRaises(ValueError): self.prepare()

    def test_outside_clip_holds_endpoint(self):
        gesture=self.prepare()
        self.assertEqual(gesture.sample(-1,0).dq,(0.,)*29)
        self.assertEqual(gesture.sample(3,0).dq,(0.,)*29)


if __name__ == "__main__": unittest.main()
