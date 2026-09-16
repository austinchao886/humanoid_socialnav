import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
from motion_contracts.protocol import ControlCommand, ProtocolError
from sonic_tracker.supervisor import SonicSupervisor
from motion_pipeline.runtime.dds_transport import JsonDDS


class PreparedExecutionTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name);self.dds=Mock()
        self.s=SonicSupervisor(self.root,self.root,self.dds)
        self.s.interactive_session_id="session"
        self.command=ControlCommand("r","m","approve_execute",prepared_plan_id="a"*64)
        self.plan=SimpleNamespace(plan_id="a"*64,duration_s=2.,gesture=SimpleNamespace(content_id="content"))
        for name,kwargs in (("_resolve_prepared_approval",dict(return_value=(self.plan,SimpleNamespace(valid=True)))),
                            ("_require_gesture_session",dict(return_value=dict(session_id="session",state="INTERACTIVE"))),
                            ("_write_runtime_request",{})):
            p=patch.object(self.s,name,**kwargs);p.start();self.addCleanup(p.stop)
    def execute(self):
        return self.s._execute(self.command,approval_received_monotonic=time.monotonic(),
            approval_received_at="now",queue_wait_s=0.)
    def stream(self, command, *, timeout_s, on_started):
        on_started(dict(origin_sim_tick=1000,execution_id=1))
        return dict(session_id="session",updates=10)
    def report(self):
        paths=list((self.root/"executions").glob("gesture-*.json"))
        self.assertEqual(len(paths),1)
        return json.loads(paths[0].read_text())
    def test_opt_in_stream_status_and_separate_report(self):
        with patch.dict(os.environ,SONIC_ENABLE_GESTURE_COMPOSITION="1"), \
             patch.object(self.s,"_stream_prepared_gesture",side_effect=self.stream):
            self.execute()
        states=[json.loads(call.args[1])["state"] for call in self.dds.publish.call_args_list]
        self.assertEqual(states,["EXECUTING","COMPLETED"])
        report=self.report()
        self.assertEqual(report["prepared_plan_id"],self.plan.plan_id)
        self.assertEqual(report["first_reference"]["origin_sim_tick"],1000)
        self.assertGreaterEqual(report["approve_to_first_reference_s"],0.)
        self.s._write_runtime_request.assert_not_called()
    def test_failure_never_reports_completed(self):
        def fail(*args,**kwargs):
            self.stream(*args,**kwargs);raise RuntimeError("stream lost")
        with patch.dict(os.environ,SONIC_ENABLE_GESTURE_COMPOSITION="1"), \
             patch.object(self.s,"_stream_prepared_gesture",side_effect=fail):
            with self.assertRaisesRegex(RuntimeError,"stream lost"):self.execute()
        self.assertEqual(self.report()["state"],"FAILED")
        self.assertEqual(self.dds.publish.call_count,1)
    def test_disabled_never_enters_prepared_branch(self):
        with patch.dict(os.environ,SONIC_ENABLE_GESTURE_COMPOSITION="0"), \
             patch.object(self.s,"_execute_prepared") as execute:
            with self.assertRaisesRegex(ProtocolError,"not enabled"):self.execute()
        execute.assert_not_called()

    def test_discovery_wait_occurs_before_stream_not_in_first_status_callback(self):
        dds=JsonDDS.__new__(JsonDDS)
        dds._publishers={}; dds._publisher_type=Mock(); dds._message_type=Mock()
        self.s.dds=dds
        waits=[]
        streaming=False
        def discovery(seconds):
            waits.append((seconds,streaming))
        def stream(*args,**kwargs):
            nonlocal streaming
            streaming=True
            try:return self.stream(*args,**kwargs)
            finally:streaming=False
        with patch.dict(os.environ,SONIC_ENABLE_GESTURE_COMPOSITION="1"), \
             patch("motion_pipeline.runtime.dds_transport.time.sleep",side_effect=discovery), \
             patch.object(self.s,"_stream_prepared_gesture",side_effect=stream):
            self.execute()
        self.assertEqual(waits,[(.25,False)])
        dds._publisher_type.assert_called_once()
        self.assertEqual(dds._publisher_type.return_value.Write.call_count,2)


if __name__=="__main__":unittest.main()
