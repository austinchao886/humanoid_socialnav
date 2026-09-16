from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.composer import JointReference, RIGHT_ARM
from sonic_tracker.prepared_gesture import PreparedGesture
from sonic_tracker.gesture_runtime import GesturePlan, GestureRuntime


class GestureRuntimeTests(unittest.TestCase):
    def plan(self):
        gesture = PreparedGesture("content", "source", 2., .25, 2.,
                                  ((0.,)*29,(1.,)*29,(0.,)*29), ((0.,)*29,)*3)
        return GesturePlan(gesture, 0., 2., .5, .5)

    def runtime(self):
        plan = self.plan()
        return GestureRuntime(plan, approved_plan_id=plan.plan_id,
                              session_id="s", start_simulation_s=10.)

    def base(self,t):
        return JointReference((.1*t,)*29, (.1,)*29,t)

    def test_entry_exit_and_ownership(self):
        runtime=self.runtime()
        for t in (10.,10.2,10.8,11.7,12.,12.5):
            base=self.base(t)
            out=runtime.sample(base,session_id="s")
            if t in (10.,12.,12.5): self.assertEqual(out,base)
            for j in set(range(29))-set(RIGHT_ARM):
                self.assertEqual((out.q[j],out.dq[j]),(base.q[j],base.dq[j]))

    def test_cancel_continuity_and_finite_difference(self):
        # Cancel during entry: multiply the original envelope, rather than
        # freezing its derivative or jumping its weight to one.
        runtime=self.runtime()
        before=runtime.sample(self.base(10.2),session_id="s")
        runtime.cancel(10.2)
        after=runtime.sample(self.base(10.2),session_id="s")
        self.assertEqual(before,after)
        runtime.cancel(10.3)
        self.assertEqual(runtime.cancel_s,10.2)
        for t in (10.3,10.4,10.6):
            def value(at):
                r=self.runtime(); r.cancel(10.2)
                return r.sample(self.base(at),session_id="s")
            h=1e-5
            derivative=(value(t+h).q[12]-value(t-h).q[12])/(2*h)
            self.assertAlmostEqual(derivative,value(t).dq[12],places=6)
        self.assertEqual(runtime.sample(self.base(10.8),session_id="s"),self.base(10.8))

    def test_plan_identity_and_session(self):
        plan=self.plan()
        other=GesturePlan(plan.gesture,0.,2.,.6,.5)
        self.assertNotEqual(plan.plan_id,other.plan_id)
        with self.assertRaises(ValueError):
            GestureRuntime(other,approved_plan_id=plan.plan_id,session_id="s",start_simulation_s=10.)
        with self.assertRaises(ValueError): self.runtime().sample(self.base(10.),session_id="new")
        with self.assertRaises(ValueError): self.runtime().sample(self.base(9.),session_id="s")

    def test_cancel_at_entry_hold_and_exit_preserves_reference_derivative(self):
        for cancel_at in (10., 10.2, 10.8, 11.7):
            with self.subTest(cancel_at=cancel_at):
                runtime = self.runtime()
                before = runtime.sample(self.base(cancel_at), session_id="s")
                runtime.cancel(cancel_at)
                self.assertEqual(before, runtime.sample(self.base(cancel_at), session_id="s"))
                for offset in (.05, .15, .3, .45):
                    t = cancel_at + offset
                    def sample(at):
                        r = self.runtime()
                        r.cancel(cancel_at)
                        return r.sample(self.base(at), session_id="s")
                    h = 1e-5
                    left, center, right = sample(t-h), sample(t), sample(t+h)
                    for joint in RIGHT_ARM:
                        self.assertAlmostEqual((right.q[joint]-left.q[joint])/(2*h),
                                               center.dq[joint], places=5)
                    for joint in set(range(29))-set(RIGHT_ARM):
                        self.assertEqual(center.q[joint], self.base(t).q[joint])
                        self.assertEqual(center.dq[joint], self.base(t).dq[joint])
                end = cancel_at + runtime.plan.exit_s + .001
                self.assertEqual(runtime.sample(self.base(end), session_id="s"), self.base(end))

    def test_horizon_preview_does_not_advance_execution_clock(self):
        runtime=self.runtime()
        # SONIC uses ten frames at five 50-Hz steps: 0.1 s between frames.
        bases=tuple(self.base(10.+.1*i) for i in range(10))
        output=runtime.sample_window(bases,session_id="s")
        self.assertEqual(runtime.last_s,10.)
        self.assertEqual(output[0],bases[0])
        self.assertNotEqual(output[1].q[12],output[5].q[12])
        # The next 50-Hz policy tick is earlier than the previous horizon end.
        runtime.sample(self.base(10.02),session_id="s")
        runtime.cancel(10.02)
        next_window=runtime.sample_window(tuple(self.base(10.02+.1*i) for i in range(10)),session_id="s")
        self.assertEqual(next_window[-1],self.base(10.92))

    def test_bad_window_does_not_change_clock(self):
        runtime=self.runtime()
        for bases in ((), (self.base(10.2),self.base(10.1)),
                      (self.base(10.2),self.base(10.2))):
            with self.assertRaises(ValueError): runtime.sample_window(bases,session_id="s")
            self.assertEqual(runtime.last_s,10.)


if __name__ == "__main__": unittest.main()
