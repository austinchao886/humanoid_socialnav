"""Actual artifact -> Python producer -> private PTY IPC -> C++ decoder.

No DDS publications or simulator commands. This is transport acceptance only.
"""
import argparse
import json
from pathlib import Path
import select
import sys
import time
import pexpect

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.prepared_gesture import prepare_gesture
from sonic_tracker.gesture_runtime import GesturePlan,GestureRuntime
from sonic_tracker.gesture_snapshot import build_snapshot
from sonic_tracker.gesture_transport import spawn_with_gesture_channel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe",type=Path)
    parser.add_argument("artifact",type=Path)
    args=parser.parse_args()
    prepared=prepare_gesture(args.artifact,amplitude=.25,time_scale=2.)
    plan=GesturePlan(prepared,0.,prepared.duration_s,1.,1.)
    runtime=GestureRuntime(plan,approved_plan_id=plan.plan_id,session_id="ipc-test",start_simulation_s=0.)
    packet=build_snapshot(runtime,session_id="ipc-test",origin_tick=1000,
                          origin_simulation_s=5.,sequence=0)
    encoded=json.dumps(packet,allow_nan=False,separators=(",",":")).encode()
    child,channel=spawn_with_gesture_channel(str(args.probe.resolve()),["ipc-test",plan.plan_id],
                                            encoding="utf-8",timeout=10,maxread=262144)
    try:
        child.expect_exact("receiver-ready")
        assert select.select([],[channel],[],3)[1],"channel blocked"
        start=time.monotonic()
        assert channel.send(encoded)==len(encoded),"partial seqpacket"
        child.expect(pexpect.EOF)
        lines=[line for line in child.before.splitlines() if line.startswith("result:")]
        assert len(lines)==1,"missing native result"
        native=json.loads(lines[0][len("result:"):])
        assert native["origin_sim_tick"]==packet["origin_sim_tick"]
        assert native["frames"]==packet["frames"],"native data mismatch"
        child.close()
        assert child.exitstatus==0,"native receiver failed"
        print(json.dumps(dict(status="passed",frames=256,scalars=4096,bytes=len(encoded),
                              content_id=prepared.content_id,plan_id=plan.plan_id,
                              exchange_wall_s=time.monotonic()-start,
                              scope="IPC_only_not_controller_or_dynamics")))
    finally:
        channel.close();child.close(force=True)

if __name__=="__main__":main()
