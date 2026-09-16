"""Real artifact streaming over private IPC; synthetic native clock, no DDS."""
import argparse
import json
from pathlib import Path
import statistics
import sys
import time
import pexpect

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.prepared_gesture import prepare_gesture
from sonic_tracker.gesture_runtime import GesturePlan
from sonic_tracker.gesture_sender import GestureSender
from sonic_tracker.gesture_transport import spawn_with_gesture_channel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe",type=Path)
    parser.add_argument("artifact",type=Path)
    parser.add_argument("--executions",type=int,default=2)
    args=parser.parse_args()
    if not 1<=args.executions<=3:parser.error("executions must be 1..3")
    gesture=prepare_gesture(args.artifact,amplitude=.25,time_scale=2.)
    plan=GesturePlan(gesture,0.,gesture.duration_s,1.,1.)
    child,channel=spawn_with_gesture_channel(str(args.probe.resolve()),[str(args.executions)],
                                            encoding="utf-8",timeout=5)
    costs=[]
    try:
        child.expect_exact("receiver-ready")
        sender=GestureSender(channel,"stream-test")
        for _ in range(args.executions):
            sender.begin(plan,approved_plan_id=plan.plan_id)
            deadline=time.monotonic()+plan.duration_s+5
            while True:
                start=time.monotonic()
                completed=sender.update()
                cost=time.monotonic()-start
                costs.append(cost)
                if completed:break
                if time.monotonic()>deadline:raise RuntimeError("sender completion timeout")
                time.sleep(max(0.,.02-cost))
        child.expect(pexpect.EOF)
        results=[line[7:] for line in child.before.splitlines() if line.startswith("result:")]
        child.close()
        if child.exitstatus!=0 or len(results)!=1:raise RuntimeError(f"native failed: {child.before}")
        ordered=sorted(costs)
        print(json.dumps(dict(scope="synthetic_clock_IPC_not_policy_or_dynamics",
            native=json.loads(results[0]),updates=len(costs),
            update_median_ms=statistics.median(costs)*1000,
            update_p95_ms=ordered[int(.95*(len(ordered)-1))]*1000,
            update_max_ms=max(costs)*1000,updates_over_20ms=sum(v>.02 for v in costs))))
    except Exception:
        try:
            child.expect(pexpect.EOF,timeout=2)
            print("native diagnostic: "+child.before,file=sys.stderr)
        except (pexpect.TIMEOUT,pexpect.EOF):
            pass
        print(f"completed updates: {len(costs)}; max update ms: {max(costs,default=0)*1000}",file=sys.stderr)
        raise
    finally:
        channel.close();child.close(force=True)


if __name__=="__main__":main()
