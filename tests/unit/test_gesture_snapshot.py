from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.prepared_gesture import PreparedGesture
from sonic_tracker.gesture_runtime import GesturePlan,GestureRuntime
from sonic_tracker.gesture_snapshot import build_snapshot


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        gesture=PreparedGesture("content","source",2.,.25,2.,
                                ((0.,)*29,(1.,)*29,(0.,)*29),((0.,)*29,)*3)
        plan=GesturePlan(gesture,0.,2.,.5,.5)
        self.runtime=GestureRuntime(plan,approved_plan_id=plan.plan_id,session_id="s",start_simulation_s=10.)

    def packet(self,**changes):
        args=dict(session_id="s",origin_tick=2000,origin_simulation_s=10.,sequence=0)
        args.update(changes)
        return build_snapshot(self.runtime,**args)

    def test_shape_and_clock_preserved(self):
        packet=self.packet()
        self.assertEqual(len(packet["frames"]),256)
        self.assertTrue(all(len(frame)==16 for frame in packet["frames"]))
        self.assertEqual(self.runtime.last_s,10.)
        self.assertEqual(packet["frames"][0][-2:],[0.,0.])
        self.assertNotEqual(packet["frames"][5],packet["frames"][185])
        self.assertEqual(packet["plan_id"],self.runtime.plan.plan_id)

    def test_reject_invalid_identity_and_tick(self):
        for args in (dict(session_id="other"),dict(origin_tick=-1),dict(origin_tick=True),
                     dict(origin_simulation_s=9.),dict(sequence=-1)):
            with self.assertRaises(ValueError):self.packet(**args)

if __name__=="__main__":unittest.main()
