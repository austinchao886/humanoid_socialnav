import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from sonic_tracker.supervisor import SonicSupervisor


class GestureLaunchTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        executable=self.root/"target/release/g1_deploy_onnx_ref"
        executable.parent.mkdir(parents=True);executable.touch()
        self.supervisor=SonicSupervisor(self.root,self.root,object())
        self.supervisor.persistent_process=True
        self.addCleanup(self.supervisor._stop)
        self.env=patch.dict(os.environ,{"SONIC_ENABLE_GESTURE_COMPOSITION":"1",
                                        "SONIC_DDS_DOMAIN":"42","SONIC_INTERFACE":"lo"})
        self.env.start();self.addCleanup(self.env.stop)

    def launch(self,session="session"):
        with patch.object(self.supervisor,"_prepare_persistent_reference_pool",return_value=self.root):
            self.supervisor._spawn_controller("wave",artifact=self.root,log=self.root/"run.log",
                                               planner_enabled=True,session_id=session)

    def test_opt_in_passes_actual_session_to_private_launcher(self):
        with patch("sonic_tracker.supervisor.spawn_with_gesture_channel",side_effect=RuntimeError("spawn sentinel")) as factory:
            with self.assertRaisesRegex(RuntimeError,"spawn sentinel"):self.launch()
            self.assertEqual(factory.call_args.kwargs["env"]["SONIC_GESTURE_SESSION_ID"],"session")

    def test_missing_session_rejected_before_launch(self):
        with patch("sonic_tracker.supervisor.spawn_with_gesture_channel") as factory:
            with self.assertRaisesRegex(RuntimeError,"Isaac session"):self.launch(None)
            factory.assert_not_called()

    def test_disabled_clears_stale_channel_environment(self):
        with patch.dict(os.environ,{"SONIC_ENABLE_GESTURE_COMPOSITION":"0","SONIC_GESTURE_FD":"999",
                                     "SONIC_GESTURE_SESSION_ID":"old"}), \
             patch("sonic_tracker.supervisor.pexpect.spawn",side_effect=RuntimeError("spawn sentinel")) as factory:
            with self.assertRaisesRegex(RuntimeError,"spawn sentinel"):self.launch()
            self.assertNotIn("SONIC_GESTURE_FD",factory.call_args.kwargs["env"])
            self.assertNotIn("SONIC_GESTURE_SESSION_ID",factory.call_args.kwargs["env"])

if __name__=="__main__":unittest.main()
