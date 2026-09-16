import json
import tempfile
import unittest
import sys
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch, Mock

from motion_contracts.protocol import ControlCommand, ProtocolError, to_json
from sonic_tracker.supervisor import SonicSupervisor


class PreparedApprovalTests(unittest.TestCase):
    def test_cli_publishes_exact_prepared_plan(self):
        from motion_pipeline.cli import main
        transport = Mock()
        module = SimpleNamespace(JsonDDS=Mock(return_value=transport))
        with patch.dict(sys.modules, {"motion_pipeline.runtime.dds_transport": module}), \
             patch.object(sys, "argv", ["motion-cli", "--domain", "42", "--interface", "lo",
                 "control", "approve_execute", "r", "m", "--prepared-plan-id", "b" * 64]), \
             patch("builtins.print"):
            main()
        transport.publish.assert_called_once()
        topic, payload = transport.publish.call_args.args
        self.assertEqual(topic, "rt/motion/control/cmd")
        self.assertEqual(ControlCommand.parse(payload).prepared_plan_id, "b" * 64)

    def test_legacy_wire_format_unchanged(self):
        command = ControlCommand("request", "motion", "approve_execute")
        payload = to_json(command)
        self.assertEqual(set(json.loads(payload)),
                         {"request_id", "motion_id", "action", "schema_version"})
        self.assertEqual(ControlCommand.parse(payload), command)

    def test_exact_plan_round_trip(self):
        command = ControlCommand("request", "motion", "approve_execute",
                                 prepared_plan_id="a" * 64)
        self.assertEqual(ControlCommand.parse(to_json(command)), command)

    def test_invalid_plan_or_action_rejected(self):
        for value in ("", "A" * 64, "a" * 63, 123, True, []):
            with self.subTest(value=value), self.assertRaises(ProtocolError):
                ControlCommand.parse(json.dumps(dict(schema_version=1,
                    request_id="r", motion_id="m", action="approve_execute",
                    prepared_plan_id=value)))
        for action in ("abort", "reset", "reject"):
            with self.subTest(action=action), self.assertRaises(ProtocolError):
                ControlCommand("r", "m", action, prepared_plan_id="a" * 64)

    def test_unavailable_composition_never_executes_original(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            supervisor = SonicSupervisor(root, root, object())
            command = ControlCommand("r", "m", "approve_execute",
                                     prepared_plan_id="a" * 64)
            with patch.object(supervisor, "_require_isaac_ready") as ready, \
                 patch.object(supervisor, "_write_runtime_request") as request:
                with self.assertRaisesRegex(ProtocolError, "not enabled"):
                    supervisor._execute(command, approval_received_monotonic=0,
                                        approval_received_at="now", queue_wait_s=0)
                ready.assert_not_called()
                request.assert_not_called()
            self.assertFalse((root / "m").exists())


if __name__ == "__main__":
    unittest.main()
