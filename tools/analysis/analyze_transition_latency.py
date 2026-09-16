"""Observed wall-clock phase latency; does not claim exact DDS receipt timing."""
import argparse
import json
import math
from pathlib import Path
import statistics

PHASES = (
    "walking_before_approval",
    "transition_request_PLANNER_HOLD",
    "transition_request_REFERENCE_PREEMPT",
    "transition_request_SETTLING",
    "transition_request_PLAYING",
)
LABELS = ("dispatch_to_hold", "hold_to_preempt", "preempt_to_settling", "settling_to_playing")


def analyze(path):
    rows = []
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.lstrip().startswith("{"):
            continue  # DDS/CLI human-readable diagnostics
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{number}: malformed JSON") from exc
        if isinstance(row, dict) and "event" in row:
            stamp = row.get("epoch_s")
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError(f"{path}:{number}: invalid epoch_s")
            rows.append(row)
    if any(b["epoch_s"] < a["epoch_s"] for a, b in zip(rows, rows[1:])):
        raise ValueError("nonmonotonic event clock")
    sessions = {r.get("session_id") for r in rows}
    if len(sessions) != 1 or None in sessions or "" in sessions:
        raise ValueError("missing or changed session identity")
    starts = [i for i, r in enumerate(rows) if r["event"] == PHASES[0]]
    if len(starts) != 1:
        raise ValueError("expected one approval dispatch marker")
    first = {}
    for row in rows[starts[0]:]:
        if row["event"] in PHASES:
            first.setdefault(row["event"], row["epoch_s"])
    missing = [p for p in PHASES if p not in first]
    if missing:
        raise ValueError(f"missing phase markers: {missing}")
    times = [first[p] for p in PHASES]
    if times != sorted(times):
        raise ValueError("out-of-order phase markers")
    spans = {label: b-a for label, a, b in zip(LABELS, times, times[1:])}
    spans["dispatch_to_playing"] = times[-1] - times[0]
    return {"log": str(path), "session_id": next(iter(sessions)), "wall_seconds": spans}


def summarize(paths):
    rounds, excluded = [], []
    for path in paths:
        try:
            rounds.append(analyze(path))
        except (ValueError, OSError) as exc:
            excluded.append({"log": str(path), "reason": str(exc)})
    aggregate = {}
    for label in (*LABELS, "dispatch_to_playing"):
        values = [r["wall_seconds"][label] for r in rounds]
        if values:
            aggregate[label] = {"n": len(values), "min": min(values),
                                "median": statistics.median(values), "max": max(values)}
    return {"schema_version": 1, "scope": "observed wall-clock phase timing, not motion-quality acceptance",
            "origin": "walking_before_approval marker preceding CLI dispatch; includes launch and delivery",
            "limitations": "polled phase markers; not exact controller transition timestamps; historical logs are not new tests",
            "rounds": rounds, "excluded": excluded, "aggregate_wall_seconds": aggregate}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = summarize(args.logs)
    payload = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x") as handle:
            handle.write(payload)
    print(payload, end="")
    return 1 if report["excluded"] or not report["rounds"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
