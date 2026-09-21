import json
import numpy as np
import unittest
from motion_pipeline.pico_live import FreshnessGate,pack_v1

def test_gap_requires_rearm_even_after_fresh_packet():
 g=FreshnessGate();assert g.accept('a',0,0);g.arm(0);assert g.active(.1)
 assert g.accept('a',1,.4);assert not g.active(.4)
 g.arm(.4);assert g.active(.4)
 assert g.accept('b',0,.41);assert not g.active(.41)

def test_duplicate_does_not_refresh_watchdog():
 g=FreshnessGate();g.accept('a',0,0);g.arm(0)
 assert not g.accept('a',0,.2);assert not g.active(.26)

def test_protocol_v1_wire_layout():
 p=pack_v1(np.zeros((2,29)),np.ones((2,29)),[[1,0,0,0]]*2,[3,4])
 h=json.loads(p[4:1284].rstrip(b'\0'));assert h['v']==1
 assert [f['name'] for f in h['fields']]==['joint_pos','joint_vel','body_quat','frame_index']
 assert len(p)==1284+2*(29*4+29*4+4*4+8)
 assert np.frombuffer(p[-16:],dtype='<i8').tolist()==[3,4]

def test_invalid_packet_rejected():
 for indices,quat in [([2,1],[[1,0,0,0]]*2),([1,2],[[0,0,0,0]]*2)]:
  with unittest.TestCase().assertRaises(ValueError):pack_v1(np.zeros((2,29)),np.ones((2,29)),quat,indices)

def test_nonfinite_pose_rejected():
 with unittest.TestCase().assertRaises(ValueError):pack_v1(np.full((1,29),np.nan),np.ones((1,29)),[[1,0,0,0]],[0])

if __name__ == "__main__":
 suite=unittest.TestSuite(unittest.FunctionTestCase(v) for k,v in list(globals().items()) if k.startswith("test_"))
 result=unittest.TextTestRunner(verbosity=2).run(suite)
 raise SystemExit(not result.wasSuccessful())
