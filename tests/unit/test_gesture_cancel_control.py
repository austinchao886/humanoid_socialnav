"""Prepared gesture cancellation contracts; no DDS or simulator commands."""

import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from motion_contracts.protocol import ControlCommand, ProtocolError, to_json
from sonic_tracker.supervisor import SonicSupervisor


class CancelControlTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.s = SonicSupervisor(root, root, Mock())
        self.s.persistent_process = True
        self.s.control_started = True
        self.s.runtime_mode = 'JOYSTICK_LOCOMOTION'
        self.s.interactive_session_id = 's'
        self.s.child = Mock()
        self.s.child.isalive.return_value = True
        self.s.gesture_channel = object()
        self.sender = Mock(channel=self.s.gesture_channel, session_id='s')
        self.s.gesture_sender = self.sender
        self.command = ControlCommand('r', 'm', 'approve_execute', prepared_plan_id='a'*64)
        self.s.active_command = self.command
        self.s.active_gesture_token = 'b'*32
        self.cancel = ControlCommand('r', 'm', 'cancel', execution_token='b'*32)
        self.plan = SimpleNamespace(
            plan_id='a'*64, duration_s=2., gesture=SimpleNamespace(content_id='c')
        )
        patches = (
            ('_resolve_prepared_approval', {'return_value': (self.plan, SimpleNamespace(valid=True))}),
            ('_require_gesture_session', {'return_value': {'session_id': 's'}}),
            ('_service_controller_output', {}),
            ('_stop', {}),
            ('_write_runtime_request', {}),
        )
        for name, kwargs in patches:
            patcher = patch.object(self.s, name, **kwargs)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_protocol_exact_token_and_legacy_wire(self):
        self.assertEqual(ControlCommand.parse(to_json(self.cancel)),self.cancel)
        self.assertNotIn('execution_token',json.loads(to_json(ControlCommand('r','m','abort'))))
        for token in (None,'x'*32,'a'*31,True):
            with self.assertRaises(ProtocolError):ControlCommand('r','m','cancel',execution_token=token)
        with self.assertRaises(ProtocolError):ControlCommand('r','m','abort',execution_token='b'*32)

    def test_cancel_latches_without_stopping_or_aborting_runtime(self):
        self.s._handle(self.cancel)
        self.s._handle(self.cancel)
        self.assertTrue(self.s.gesture_cancel_event.is_set())
        self.assertFalse(self.s.abort_event.is_set())
        self.s._stop.assert_not_called()
        self.s._write_runtime_request.assert_not_called()

    def test_stale_token_wrong_owner_and_ordinary_motion_rejected(self):
        for r,m,token in (('other','m','b'*32),('r','other','b'*32),('r','m','c'*32)):
            with self.assertRaises(ProtocolError):self.s._handle(ControlCommand(r,m,'cancel',execution_token=token))
        self.s.active_command=ControlCommand('r','m','approve_execute')
        with self.assertRaises(ProtocolError):self.s._handle(self.cancel)
        self.assertFalse(self.s.gesture_cancel_event.is_set())
        self.s._stop.assert_not_called()

    def test_stream_cancel_waits_for_sender_consumption_ack(self):
        self.s._handle(self.cancel)
        self.sender.update.side_effect = [False, False, True]
        result=self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.assertTrue(result['cancelled'])
        self.assertEqual(result['updates'], 3)
        for call in self.sender.update.call_args_list:self.assertEqual(call.kwargs,{'cancel':True})
        self.s._stop.assert_not_called()

    def test_abort_remains_immediate_and_takes_priority(self):
        self.s._handle(self.cancel)
        self.s._handle(ControlCommand('r','m','abort'))
        self.s._stop.assert_called_once()
        self.assertTrue(self.s.abort_event.is_set())
        with self.assertRaisesRegex(RuntimeError,'aborted'):
            self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.sender.begin.assert_not_called()

    def test_cancel_transport_failure_still_stops(self):
        self.s._handle(self.cancel)
        self.sender.update.side_effect = ConnectionError('lost')
        with self.assertRaises(ConnectionError):self.s._stream_prepared_gesture(self.command,timeout_s=1.)
        self.s._stop.assert_called_once()

    def test_cancel_missing_consumption_ack_times_out_and_stops(self):
        self.s._handle(self.cancel)
        self.sender.update.return_value = False
        # No wall-clock sleep: first update starts the five-second deadline;
        # the next iteration passes it without a native consumption ACK.
        with patch('sonic_tracker.supervisor.time.monotonic',
                   side_effect=[0., 0., 0., 5.1]), \
             patch.object(self.s.abort_event, 'wait'):
            with self.assertRaisesRegex(RuntimeError, 'cancellation timed out'):
                self.s._stream_prepared_gesture(self.command, timeout_s=100.)
        self.sender.update.assert_called_once_with(cancel=True)
        self.s._stop.assert_called_once()

    def test_cancel_completion_report_is_not_natural_completion(self):
        def stream(command, *,timeout_s,on_started):
            on_started({'origin_sim_tick':100,'execution_id':1})
            return {'cancelled':True,'session_id':'s'}
        with patch.object(self.s,'_stream_prepared_gesture',side_effect=stream):
            self.s._execute_prepared(self.command,approval_received_monotonic=time.monotonic(),
                                     approval_received_at='now',queue_wait_s=0.)
        paths=list((self.s.exchange/'executions').glob('gesture-*.json'))
        report=json.loads(paths[0].read_text())
        self.assertEqual(report['state'],'ABORTED')
        self.assertEqual(report['termination_mode'],'reference_fade')
        self.assertEqual(len(report['execution_token']),32)
        self.assertIsNone(self.s.active_gesture_token)
        self.s._stop.assert_not_called()
        self.s._write_runtime_request.assert_not_called()
        with self.assertRaises(ProtocolError):self.s._handle(self.cancel)

if __name__ == '__main__':
    unittest.main()
