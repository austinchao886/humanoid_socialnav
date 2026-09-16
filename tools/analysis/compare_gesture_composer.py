"""Compare compiled native reference arithmetic to Python, without control I/O."""
import argparse
import json
import math
from pathlib import Path
import random
import subprocess
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"services/sonic_tracker"))
from sonic_tracker.composer import JointReference, RIGHT_ARM, compose_right_arm, transition_weight


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe",type=Path)
    args=parser.parse_args()
    randomizer=random.Random(20260915)
    lines=[]; expected=[]
    for k in range(1000):
        base=JointReference(tuple(randomizer.uniform(-2,2) for _ in range(29)),
                            tuple(randomizer.uniform(-3,3) for _ in range(29)),0.)
        gesture=JointReference(tuple(randomizer.uniform(-2,2) for _ in range(29)),
                               tuple(randomizer.uniform(-3,3) for _ in range(29)),0.)
        w,dw=transition_weight((k%101)/100,1.)
        if k%2: w,dw=1-w,-dw
        result=compose_right_arm(base,gesture,w,dw)
        expected.append((*result.q,*result.dq))
        values=(*base.q,*base.dq,*(gesture.q[i] for i in RIGHT_ARM),
                *(gesture.dq[i] for i in RIGHT_ARM),w,dw)
        lines.append(" ".join(format(v,".17g") for v in values))
    output=subprocess.run([str(args.probe.resolve())],input=str(len(lines))+"\n"+"\n".join(lines)+"\n",
                          text=True,capture_output=True,check=True,timeout=30)
    actual=[tuple(map(float,row.split())) for row in output.stdout.splitlines()]
    if len(actual)!=len(expected) or any(len(row)!=58 for row in actual):
        raise ValueError("native output shape mismatch")
    errors=[abs(a-b) for row, target in zip(actual,expected) for a,b in zip(row,target)]
    if not all(math.isfinite(e) and e<=1e-12 for e in errors):
        raise ValueError("native/Python composition disagreement")
    print(json.dumps(dict(cases=len(lines),seed=20260915,max_abs_error=max(errors),
                          tolerance=1e-12,scope="arithmetic_only_not_tracking_acceptance")))

if __name__=="__main__": main()
