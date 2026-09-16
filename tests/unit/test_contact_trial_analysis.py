import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tools.analysis.analyze_contact_trial import analyze


class ContactAnalysisTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.path=Path(temp.name)/"trace.jsonl"
        self.trial=dict(completed=True,session_id="s",trials=[dict(stop="center",
            before_stop={"simulation_time_s":0.},samples=[dict(simulation_time_s=.4,trace_path=str(self.path))])])
        self.rows=[]
        for t in (0.,.2,.4):
            left=[0.]*16;left[3]=1.;left[7]=.2;left[15]=100.
            right=[0.]*16;right[3]=1.;right[7]=2.;right[15]=0.
            self.rows.append(dict(simulation_time_s=t,lowcmd_age_s=.01,root_tilt_rad=.02,
                foot_contact_measurements=dict(schema_version=1,world_frame=True,
                    body_names=["left_ankle_roll_link","right_ankle_roll_link"],rows=[left,right])))
    def run_analysis(self):
        self.path.write_text("".join(json.dumps(r)+"\n" for r in self.rows))
        return analyze(self.trial,self.path)
    def test_contact_filter_excludes_swing_and_reports_sampling(self):
        window=self.run_analysis()["windows"][0]
        self.assertAlmostEqual(window["median_sample_dt_s"],.2)
        self.assertEqual(window["lowcmd_age_max_s"],.01)
        self.assertEqual(window["feet"][0]["contact_link_origin_speed_p95_m_s"],.2)
        self.assertEqual(window["feet"][1]["force_contact_samples"],0)
        self.assertIsNone(window["feet"][1]["contact_link_origin_speed_max_m_s"])
    def test_nonfinite_rejected(self):
        self.rows[1]["foot_contact_measurements"]["rows"][0][7]=float("nan")
        with self.assertRaisesRegex(ValueError,"invalid foot row"):self.run_analysis()
    def test_incomplete_and_wrong_trace_rejected(self):
        self.trial["completed"]=False
        with self.assertRaisesRegex(ValueError,"incomplete"):self.run_analysis()
        self.trial["completed"]=True
        self.trial["trials"][0]["samples"][0]["trace_path"]="other.jsonl"
        with self.assertRaisesRegex(ValueError,"match"):self.run_analysis()


if __name__=="__main__":unittest.main()
