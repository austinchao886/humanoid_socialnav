"""Summarize instrumented stop-trial trace windows, without claiming sole slip.

The window starts at the last reported before-stop simulation time, not the
exact control tick. Foot-origin speed during force contact is a diagnostic
proxy; rotation and contact geometry are not removed. Never use as a safety gate.
"""
import argparse
import json
import math
from pathlib import Path
import statistics


def percentile(values, fraction):
    values=sorted(values)
    if not values:raise ValueError("empty measurement window")
    return values[int((len(values)-1)*fraction)]


def analyze(trial, trace_path, force_threshold_n=20.):
    if not math.isfinite(force_threshold_n) or force_threshold_n<=0:
        raise ValueError("force threshold must be finite and positive")
    if not trial.get("completed"):raise ValueError("trial incomplete")
    windows=[]
    for entry in trial["trials"]:
        if not entry["samples"]:raise ValueError("trial has no samples")
        paths={Path(row["trace_path"]).name for row in entry["samples"]}
        if paths!={trace_path.name}:raise ValueError("trace does not match trial")
        windows.append((entry["stop"],entry["before_stop"]["simulation_time_s"],
                        entry["samples"][-1]["simulation_time_s"]))
    rows=[]
    end=max(window[2] for window in windows)
    with trace_path.open() as stream:
        for line in stream:
            row=json.loads(line)
            if row["simulation_time_s"]>end:break
            if any(start<=row["simulation_time_s"]<=finish for _,start,finish in windows):
                rows.append(row)
    result=[]
    names=["left_ankle_roll_link","right_ankle_roll_link"]
    for name,start,finish in windows:
        selected=[row for row in rows if start<=row["simulation_time_s"]<=finish]
        if len(selected)<3:raise ValueError("insufficient trace coverage")
        times=[row["simulation_time_s"] for row in selected]
        gaps=[b-a for a,b in zip(times,times[1:])]
        if min(gaps)<=0:raise ValueError("nonmonotonic trace")
        feet=[[],[]];ages=[];tilts=[]
        for row in selected:
            data=row.get("foot_contact_measurements",{})
            if data.get("schema_version")!=1 or data.get("body_names")!=names or data.get("world_frame") is not True:
                raise ValueError("missing or incompatible foot measurement")
            if len(data.get("rows",[]))!=2:raise ValueError("invalid foot count")
            age=row["lowcmd_age_s"];tilt=row["root_tilt_rad"]
            if not all(math.isfinite(v) and v>=0 for v in (age,tilt)):
                raise ValueError("invalid command age/tilt")
            ages.append(age);tilts.append(tilt)
            for i,values in enumerate(data["rows"]):
                if len(values)!=16 or not all(math.isfinite(v) for v in values):
                    raise ValueError("invalid foot row")
                if values[15]>=force_threshold_n:
                    feet[i].append(math.hypot(values[7],values[8]))
        result.append(dict(stop=name,sample_count=len(selected),simulation_window=[times[0],times[-1]],
            median_sample_dt_s=statistics.median(gaps),max_sample_gap_s=max(gaps),
            lowcmd_age_max_s=max(ages),lowcmd_age_p95_s=percentile(ages,.95),root_tilt_max_rad=max(tilts),
            feet=[dict(name=foot,force_contact_samples=len(speed),
                contact_link_origin_speed_p95_m_s=percentile(speed,.95) if speed else None,
                contact_link_origin_speed_max_m_s=max(speed) if speed else None)
                for foot,speed in zip(names,feet)]))
    return dict(session_id=trial["session_id"],force_threshold_n=force_threshold_n,
                scope="sampled_contact_link_motion_not_sole_slip_or_continuous_LowCmd_proof",windows=result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial",type=Path);parser.add_argument("trace",type=Path)
    parser.add_argument("--force-threshold-n",type=float,default=20.)
    args=parser.parse_args()
    print(json.dumps(analyze(json.loads(args.trial.read_text()),args.trace,args.force_threshold_n),indent=2))


if __name__=="__main__":main()
