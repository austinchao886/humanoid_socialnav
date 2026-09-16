import math
import unittest
from sonic_tracker.composer import transition_weight, compose_right_arm, JointReference
from sonic_tracker.prepared_gesture import PreparedGesture
from sonic_tracker.gesture_runtime import GesturePlan, GestureRuntime


class EnvelopeEndpointTests(unittest.TestCase):
    def test_near_endpoints_remain_valid(self):
        gesture=PreparedGesture("content","wave",2.,.25,2.,((0.,)*29,)*23,((0.,)*29,)*23)
        plan=GesturePlan(gesture,0.,21.92,1.,1.)
        runtime=GestureRuntime(plan,approved_plan_id=plan.plan_id,session_id="s",start_simulation_s=5.015)
        for boundary in (0.,1.,20.92,21.92):
            for delta in (-1e-8,-1e-12,-1e-14,0.,1e-14,1e-12,1e-8):
                t=runtime.start_s+boundary+delta
                with self.subTest(t=t):
                    weight,rate=runtime.envelope(t)
                    base=JointReference((0.,)*29,(0.,)*29,t)
                    compose_right_arm(base,base,weight,rate)

    def test_transition_bounds_and_endpoint_derivative(self):
        for t in (math.nextafter(0.,1.),1e-12,.5,1-1e-8,1-1e-12,math.nextafter(1.,0.)):
            weight,rate=transition_weight(t,1.)
            self.assertTrue(0<=weight<=1)
            if weight in (0.,1.):self.assertEqual(rate,0.)


if __name__=="__main__":unittest.main()
