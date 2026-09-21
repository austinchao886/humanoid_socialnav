"""Read-only XRT body tracking acceptance probe. Never opens a command socket."""
import argparse
import json
import time
from pathlib import Path

import numpy as np


def validate_body(poses):
    body = np.asarray(poses, dtype=float)
    if body.shape != (24, 7) or not np.isfinite(body).all():
        raise ValueError("Expected finite body poses [24,7] in xyz,xyzw order")
    norms = np.linalg.norm(body[:, 3:], axis=1)
    if np.any(np.abs(norms - 1) > 0.1):
        raise ValueError("Body contains invalid quaternion norms")
    return body


def probe(sdk, duration, clock=time.monotonic, sleep=time.sleep):
    start = clock()
    last_stamp = None
    arrivals, stamps, bodies = [], [], []
    invalid = 0
    sdk.init()
    try:
        while clock() - start < duration:
            if sdk.is_body_data_available():
                stamp = int(sdk.get_time_stamp_ns())
                if stamp > 0 and (last_stamp is None or stamp > last_stamp):
                    try:
                        body = validate_body(sdk.get_body_joints_pose())
                        # Reject a torn sample if the SDK updated during the read.
                        if int(sdk.get_time_stamp_ns()) == stamp:
                            arrivals.append(clock() - start)
                            stamps.append(stamp)
                            bodies.append(body)
                            last_stamp = stamp
                    except ValueError:
                        invalid += 1
            sleep(0.002)
    finally:
        sdk.close()
    elapsed = clock() - start
    gaps = np.diff([0.0, *arrivals, elapsed])
    hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0.0
    max_gap = float(max(gaps))
    report = {
        "status": "PASS" if hz >= 30 and max_gap <= 0.25 and invalid == 0 else "NOT_READY",
        "scope": "raw body tracking only; no robot or simulation commands sent",
        "duration_s": elapsed, "fresh_frames": len(arrivals), "fresh_hz": hz,
        "max_gap_s_including_start_and_end": max_gap, "invalid_reads": invalid,
        "position_span_m": float(np.ptp(np.array(bodies)[:, :, :3], axis=0).max()) if bodies else 0.0,
    }
    return report, np.asarray(bodies), np.asarray(stamps, dtype=np.int64)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or not 1 <= args.duration <= 120:
        parser.error("duration must be between 1 and 120 seconds")
    import xrobotoolkit_sdk as sdk
    report, bodies, stamps = probe(sdk, args.duration)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "tracking.json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(args.output / "raw_body.npz", body_poses=bodies, timestamp_ns=stamps)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["status"] == "PASS" else 2)


if __name__ == "__main__":
    main()
