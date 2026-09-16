"""Supervisor-side private channel sender, driven by native simulation ticks."""
import json
import struct
import time
import secrets
from .gesture_runtime import GestureRuntime
from .gesture_snapshot import build_snapshot


class GestureSender:
    def __init__(self, channel, session_id):
        self.channel=channel
        self.session_id=session_id
        self.execution_id=0
        self.sequence=0
        self.runtime=None
        self.tick=None
        self.tick_received=None
        self.last_sent_tick=None
        self.clock_nonce=secrets.randbits(64)
        self.consumed_execution=0

    def poll_clock(self):
        self.clock_nonce=(self.clock_nonce+1)%(2**64)
        nonce=struct.pack("<Q",self.clock_nonce)
        if self.channel.send(nonce)!=8:raise RuntimeError("gesture clock request failed")
        deadline=time.monotonic()+.1
        for _ in range(256):
            if time.monotonic()>deadline:break
            try: payload=self.channel.recv(21)
            except BlockingIOError:
                time.sleep(.001);continue
            if len(payload)!=20: raise RuntimeError("gesture clock closed or malformed")
            if payload[:8]!=nonce:continue # never accept queued old replies
            tick,self.consumed_execution=struct.unpack("<IQ",payload[8:])
            if self.tick is not None and tick<self.tick:
                raise RuntimeError("native simulation tick regressed")
            if self.tick is None or tick>self.tick:
                self.tick=tick; self.tick_received=time.monotonic()
            if time.monotonic()-self.tick_received>.2:
                raise RuntimeError("native gesture clock stopped advancing")
            return self.tick
        raise RuntimeError("native gesture clock response timed out")

    def _send(self, message):
        raw=json.dumps(message,allow_nan=False,separators=(",",":")).encode()
        if len(raw)>262144: raise RuntimeError("gesture message too large")
        if self.channel.send(raw)!=len(raw): raise RuntimeError("incomplete gesture send")

    def begin(self, plan, *, approved_plan_id):
        if self.runtime is not None: raise RuntimeError("gesture already active")
        tick=self.poll_clock()
        runtime=GestureRuntime(plan,approved_plan_id=approved_plan_id,
                               session_id=self.session_id,start_simulation_s=tick*.005)
        execution_id=self.execution_id+1
        self._send(dict(kind="grant",session_id=self.session_id,execution_id=execution_id,plan_id=plan.plan_id))
        self.execution_id=execution_id;self.sequence=0;self.runtime=runtime;self.last_sent_tick=None

    def update(self, *, cancel=False):
        if self.runtime is None: raise RuntimeError("no active gesture")
        tick=self.poll_clock()
        if cancel:self.runtime.cancel(tick*.005)
        if tick==self.last_sent_tick:return False
        payload=build_snapshot(self.runtime,session_id=self.session_id,origin_tick=tick,
                               origin_simulation_s=tick*.005,sequence=self.sequence)
        self._send(dict(kind="snapshot",session_id=self.session_id,
                        execution_id=self.execution_id,payload=payload))
        self.sequence+=1;self.last_sent_tick=tick
        # Receipt alone is insufficient: the control loop must acknowledge use
        # of a complete zero envelope for this execution before release.
        finished=all(frame[14]==0 and frame[15]==0 for frame in payload["frames"])
        if finished and self.consumed_execution==self.execution_id:
            self._send(dict(kind="release",session_id=self.session_id,execution_id=self.execution_id))
            self.runtime=None
            return True
        return False
