import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / "tools/analysis/analyze_transition_latency.py"
if not SCRIPT.exists():
    SCRIPT = Path(__file__).with_name("analyze_transition_latency.py")
spec = importlib.util.spec_from_file_location("latency_analysis", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TransitionLatencyTests(unittest.TestCase):
    def rows(self):
        return [dict(event=p, epoch_s=t, session_id="s")
                for p, t in zip(module.PHASES, (0, 1, 5, 10, 13))]

    def report(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "round.log"
            path.write_text("DDS diagnostic\n" + '\n'.join(json.dumps(r) for r in rows))
            return module.summarize([path])

    def test_phase_durations_and_non_event_cli_json(self):
        report = self.report(self.rows() + [{"action": "approve_execute"}])
        self.assertFalse(report["excluded"])
        self.assertEqual(report["rounds"][0]["wall_seconds"], dict(
            dispatch_to_hold=1, hold_to_preempt=4, preempt_to_settling=5,
            settling_to_playing=3, dispatch_to_playing=13))

    def test_missing_is_not_silently_passed(self):
        self.assertTrue(self.report(self.rows()[:-1])["excluded"])

    def test_changed_session_rejected(self):
        rows = self.rows()
        rows[-1]["session_id"] = "new"
        self.assertTrue(self.report(rows)["excluded"])

    def test_clock_and_nonfinite_rejected(self):
        for stamp in (-1, float("nan"), float("inf"), True):
            rows = self.rows()
            rows[-1]["epoch_s"] = stamp
            self.assertTrue(self.report(rows)["excluded"])

    def test_repeated_settling_uses_first_marker(self):
        rows = self.rows()
        rows.insert(-1, dict(rows[-2], epoch_s=11))
        self.assertEqual(self.report(rows)["rounds"][0]["wall_seconds"]["settling_to_playing"], 3)

    def test_out_of_order_phase_rejected(self):
        rows = self.rows()
        rows[1]["event"], rows[2]["event"] = rows[2]["event"], rows[1]["event"]
        self.assertTrue(self.report(rows)["excluded"])


if __name__ == "__main__":
    unittest.main()
