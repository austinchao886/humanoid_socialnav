import random
import unittest
from sonic_tracker.prepared_gesture import PreparedGesture
from sonic_tracker.composer import RIGHT_ARM


class SamplingParityTests(unittest.TestCase):
    def test_optimized_arm_matches_original_scalar_formula(self):
        rng=random.Random(20260915)
        q=tuple(tuple(rng.uniform(-1,1) for _ in range(29)) for _ in range(10))
        dq=tuple(tuple(rng.uniform(-2,2) for _ in range(29)) for _ in range(10))
        gesture=PreparedGesture("content","motion",50.,.25,2.,q,dq)
        for t in [-1.,0.,gesture.duration_s,1.]+[rng.uniform(0,gesture.duration_s) for _ in range(1000)]:
            actual_q,actual_dq=gesture.sample_right_arm(t)
            c=max(0.,min(len(q)-1.,t/gesture.time_scale*gesture.fps))
            index=min(int(c),len(q)-2);u=c-index;dt=1/gesture.fps
            for k,j in enumerate(RIGHT_ARM):
                a,b=q[index][j],q[index+1][j]
                va,vb=dq[index][j],dq[index+1][j]
                value=(2*u**3-3*u*u+1)*a+(u**3-2*u*u+u)*dt*va+(-2*u**3+3*u*u)*b+(u**3-u*u)*dt*vb
                speed=((6*u*u-6*u)*a/dt+(3*u*u-4*u+1)*va+(-6*u*u+6*u)*b/dt+(3*u*u-2*u)*vb)/gesture.time_scale
                if t<0 or t>gesture.duration_s:speed=0.
                self.assertAlmostEqual(actual_q[k],q[0][j]+.25*(value-q[0][j]),delta=1e-12)
                self.assertAlmostEqual(actual_dq[k],.25*speed,delta=1e-12)


if __name__=="__main__":unittest.main()
