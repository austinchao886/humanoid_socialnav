"""Describe verified neutral-padding layout; does not edit or approve an artifact."""
import argparse
import json
import math
from pathlib import Path


def describe(manifest, *, time_scale=2., amplitude=.25):
    if not math.isfinite(time_scale) or time_scale<1:
        raise ValueError("time_scale must not accelerate source")
    if not math.isfinite(amplitude) or not 0<amplitude<=1:
        raise ValueError("invalid amplitude")
    conditioning=manifest["temporal_conditioning"]
    if conditioning.get("neutral_transition")!="cubic_smoothstep":
        raise ValueError("unrecognized transition layout")
    fps=float(manifest["fps"])
    if not math.isfinite(fps) or fps<=0:raise ValueError("invalid fps")
    hold_s=float(conditioning["neutral_hold_s_each_end"])
    transition_s=float(conditioning["neutral_transition_s_each_end"])
    if not all(math.isfinite(v) and v>=1 for v in (hold_s,transition_s)):
        raise ValueError("invalid padding metadata")
    hold=max(1,round(hold_s*fps));transition=max(1,round(transition_s*fps))
    main=manifest["conditioned_motion_frames"]
    if type(main) is not int or main<=0:raise ValueError("invalid main frame count")
    counts=(hold,transition,main,transition,hold)
    if sum(counts)!=manifest["num_frames"]:raise ValueError("padding/frame count mismatch")
    names=("initial_hold","entry_transition","main_motion","exit_transition","final_hold")
    phases=[];cursor=0
    for name,count in zip(names,counts):
        phases.append(dict(name=name,start_frame=cursor,end_frame_exclusive=cursor+count,
            first_sample_s=cursor/fps*time_scale,last_sample_s=(cursor+count-1)/fps*time_scale))
        cursor+=count
    # Include the first final-hold sample; keep ALL entry/exit transition samples.
    start=hold/fps*time_scale
    end=(hold+transition+main+transition)/fps*time_scale
    return dict(motion_id=manifest["motion_id"],phases=phases,
        source_duration_s=(cursor-1)/fps*time_scale,
        candidate=dict(amplitude=amplitude,time_scale=time_scale,start_offset_s=start,
                       end_offset_s=end,entry_s=1.,exit_s=1.),
        candidate_duration_s=end-start,
        scope="padding_analysis_only_not_dynamic_qualification_or_approval")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact",type=Path)
    args=parser.parse_args()
    print(json.dumps(describe(json.loads((args.artifact/"manifest.json").read_text())),indent=2))


if __name__=="__main__":main()
