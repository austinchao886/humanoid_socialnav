from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import time
from unittest.mock import Mock, patch
from motion_contracts.protocol import ControlCommand
from sonic_tracker.supervisor import SonicSupervisor


class GestureOrchestrationTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        root=Path(temp.name)
        self.s=SonicSupervisor(root,root,Mock())
        self.s.persistent_process=True;self.s.control_started=True
        self.s.runtime_mode="JOYSTICK_LOCOMOTION";self.s.interactive_session_id="s"
        self.s.child=Mock();self.s.child.isalive.return_value=True
        self.s.gesture_channel=object()
        self.sender=Mock(channel=self.s.gesture_channel,session_id="s")
        self.s.gesture_sender=self.sender
        self.command=ControlCommand("r","m","approve_execute",prepared_plan_id="a"*64)
        self.plan=SimpleNamespace(plan_id="a"*64)
        self.status=dict(state="INTERACTIVE",session_id="s",elastic_support_scale=0.,
                         elastic_support_attitude_scale=0.,control_handoff_progress=1.,
                         updated_epoch_s=time.time(),root_height_m=.78,root_tilt_rad=.02,
                         root_linear_velocity_m_s=[0.,0.,0.],max_torque_limit_ratio=.3)
        for name,kwargs in (("_resolve_prepared_approval",dict(return_value=(self.plan,object()))),
                            ("_require_isaac_ready",dict(side_effect=lambda:self.status)),
                            ("_service_controller_output",{}),("_stop",{})):
            p=patch.object(self.s,name,**kwargs);p.start();self.addCleanup(p.stop)

    def test_success_reuses_sender_without_switch_or_stop(self):
        self.sender.update.side_effect=[False,True,True]
        for count in (2,1):
            result=self.s._stream_prepared_gesture(self.command,timeout_s=1.)
            self.assertEqual(result["updates"],count)
            self.assertEqual(result["session_id"],"s")
        self.assertEqual(self.sender.begin.call_count,2)
        self.s.child.send.assert_not_called()
        self.s._stop.assert_not_called()
        self.assertEqual(self.s._service_controller_output.call_count,3)

    def test_supported_or_wrong_session_rejected_before_grant(self):
        for change in (dict(elastic_support_scale=.1),dict(control_handoff_progress=.5),
                       dict(elastic_support_attitude_scale=float("nan")),dict(session_id="new"),
                       dict(state="READY_STANDING")):
            original=self.status.copy();self.status.update(change)
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                self.s._stream_prepared_gesture(self.command,timeout_s=1.)
            self.status=original
        self.sender.begin.assert_not_called();self.s._stop.assert_not_called()

    def test_midstream_session_change_stops_no_replay(self):
        def update():
            self.status["session_id"]="new"
            return False
        self.sender.update.side_effect=update
        with self.assertRaisesRegex(RuntimeError,"session/state"):
            self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.sender.begin.assert_called_once();self.s._stop.assert_called_once()

    def test_sender_failure_stops(self):
        self.sender.update.side_effect=ConnectionError("lost")
        with self.assertRaises(ConnectionError):
            self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.s._stop.assert_called_once()

    def test_abort_before_grant_and_during_stream(self):
        self.s.abort_event.set()
        with self.assertRaisesRegex(RuntimeError,"aborted"):
            self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.sender.begin.assert_not_called()
        self.s.abort_event.clear()
        def update():
            self.s.abort_event.set();return False
        self.sender.update.side_effect=update
        with self.assertRaisesRegex(RuntimeError,"aborted"):
            self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.s._stop.assert_called_once()


class GestureOutputTests(unittest.TestCase):
    def test_native_rejection_is_reported_before_later_physics_failure(self):
        s=SimpleNamespace(child=Mock(),_controller_output_tail="")
        s.child.read_nonblocking.side_effect=[
            "[Gest", "ure] Disabled, stale or mismatched simulation snapshot control_tick=100 origin_tick=50\n"]
        with self.assertRaisesRegex(RuntimeError,"SONIC reference input failure.*control_tick=100"):
            SonicSupervisor._service_controller_output(s)

    def test_receiver_failure_is_not_silently_drained(self):
        s=SimpleNamespace(child=Mock(),_controller_output_tail="")
        s.child.read_nonblocking.return_value="[Gesture] Receiver failed: gesture channel disconnected\n"
        with self.assertRaisesRegex(RuntimeError,"reference input failure.*Receiver failed"):
            SonicSupervisor._service_controller_output(s)


if __name__=="__main__":unittest.main()
