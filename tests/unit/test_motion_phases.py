import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tools.analysis.describe_motion_phases import describe


class MotionPhaseTests(unittest.TestCase):
    def manifest(self):
        return dict(motion_id="wave",fps=50.,num_frames=549,conditioned_motion_frames=249,
            temporal_conditioning=dict(neutral_transition="cubic_smoothstep",
                neutral_hold_s_each_end=1.,neutral_transition_s_each_end=2.))
    def test_preserves_every_transition_frame(self):
        result=describe(self.manifest())
        self.assertEqual([p["start_frame"] for p in result["phases"]],[0,50,150,399,499])
        self.assertEqual(result["candidate"]["start_offset_s"],2.)
        self.assertEqual(result["candidate"]["end_offset_s"],19.96)
        self.assertAlmostEqual(result["candidate_duration_s"],17.96)
    def test_reject_unrecognized_or_inconsistent_layout(self):
        manifest=self.manifest();manifest["num_frames"]=550
        with self.assertRaises(ValueError):describe(manifest)
        manifest=self.manifest();manifest["temporal_conditioning"]["neutral_transition"]="unknown"
        with self.assertRaises(ValueError):describe(manifest)


if __name__=="__main__":unittest.main()
