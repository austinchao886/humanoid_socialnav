import json
from pathlib import Path
import socket
import struct
import sys
import unittest
import queue
import threading
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.gesture_sender import GestureSender
from sonic_tracker.prepared_gesture import PreparedGesture
from sonic_tracker.gesture_runtime import GesturePlan

class SenderTests(unittest.TestCase):
    def setUp(self):
        self.a,self.b=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.a.setblocking(False);self.b.settimeout(1)
        self.messages=queue.Queue();self.native_tick=None;self.native_consumed=0;self.stopping=threading.Event()
        def peer():
            while not self.stopping.is_set():
                try:message=self.b.recv(262144)
                except socket.timeout:continue
                except OSError:return
                if not message:return
                if len(message)==8:
                    if self.native_tick is not None:
                        try:self.b.send(message+struct.pack("<IQ",self.native_tick,self.native_consumed))
                        except OSError:return
                else:self.messages.put(json.loads(message))
        self.worker=threading.Thread(target=peer,daemon=True);self.worker.start()
        def cleanup():
            self.stopping.set();self.a.close();self.b.close();self.worker.join(2)
        self.addCleanup(cleanup)
        self.sender=GestureSender(self.a,"s")
        gesture=PreparedGesture("content","wave",2.,.25,2.,
                                ((0.,)*29,(1.,)*29,(0.,)*29),((0.,)*29,)*3)
        self.plan=GesturePlan(gesture,0.,2.,.5,.5)
    def clock(self,tick):self.native_tick=tick
    def receive(self):return self.messages.get(timeout=1)
    def test_begin_stream_release_and_reuse(self):
        for k in range(2):
            tick=1000+k*500
            self.clock(tick);self.sender.begin(self.plan,approved_plan_id=self.plan.plan_id)
            self.assertEqual(self.receive()["kind"],"grant")
            self.assertFalse(self.sender.update())
            packet=self.receive();self.assertEqual(packet["payload"]["origin_sim_tick"],tick)
            self.clock(tick+400);self.assertFalse(self.sender.update())
            self.assertEqual(self.receive()["kind"],"snapshot")
            self.assertTrue(self.messages.empty())
            self.native_consumed=k+1
            self.clock(tick+404);self.assertTrue(self.sender.update())
            self.assertEqual(self.receive()["kind"],"snapshot")
            self.assertEqual(self.receive()["kind"],"release")
    def test_invalid_approval_and_clock(self):
        with self.assertRaises(RuntimeError):self.sender.begin(self.plan,approved_plan_id=self.plan.plan_id)
        self.clock(1000)
        with self.assertRaises(ValueError):self.sender.begin(self.plan,approved_plan_id="wrong")
        self.clock(999)
        with self.assertRaises(RuntimeError):self.sender.poll_clock()
    def test_cancel_fades_before_release(self):
        self.clock(1000);self.sender.begin(self.plan,approved_plan_id=self.plan.plan_id);self.receive()
        self.clock(1100);self.assertFalse(self.sender.update(cancel=True));self.receive()
        self.clock(1201);self.assertFalse(self.sender.update());self.receive()
        self.assertTrue(self.messages.empty())
        self.native_consumed=1
        self.clock(1205);self.assertTrue(self.sender.update());self.receive()
        self.assertEqual(self.receive()["kind"],"release")
    def test_old_queued_clock_reply_is_not_accepted(self):
        self.b.send(struct.pack("<QIQ",0,999,0))
        self.clock(2000)
        self.assertEqual(self.sender.poll_clock(),2000)
if __name__=="__main__":unittest.main()
