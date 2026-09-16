import unittest
from isaac_runtime.lifecycle import require_execution_ready


class ReadinessHeartbeatTests(unittest.TestCase):
    def status(self, **changes):
        value = dict(state="INTERACTIVE", session_id="s", updated_epoch_s=100.)
        value.update(changes)
        return value

    def test_valid_and_boundary(self):
        for state in ("READY", "READY_STANDING", "INTERACTIVE"):
            require_execution_ready(self.status(state=state), now_epoch_s=105.)

    def test_nonfinite_future_missing_and_stale(self):
        for value in (float("nan"), float("inf"), -float("inf"), 101., 94., True, None):
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                require_execution_ready(self.status(updated_epoch_s=value), now_epoch_s=100.)
        status = self.status()
        del status["updated_epoch_s"]
        with self.assertRaises(RuntimeError):
            require_execution_ready(status, now_epoch_s=100.)

    def test_invalid_clock_limit_and_identity(self):
        for kwargs in (dict(now_epoch_s=float("nan")), dict(now_epoch_s=True),
                       dict(now_epoch_s=100., max_age_s=float("inf")),
                       dict(now_epoch_s=100., max_age_s=-1.)):
            with self.subTest(kwargs=kwargs), self.assertRaises(RuntimeError):
                require_execution_ready(self.status(), **kwargs)
        for identity in (None, "", "  ", 123, True):
            with self.subTest(identity=identity), self.assertRaises(RuntimeError):
                require_execution_ready(self.status(session_id=identity), now_epoch_s=100.)


if __name__ == "__main__":
    unittest.main()
