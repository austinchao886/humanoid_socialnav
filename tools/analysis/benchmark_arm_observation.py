"""Offline microbenchmark: no Isaac app, DDS, joystick or robot commands."""
import ast
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import torch

root = Path(sys.argv[1])
sys.path.insert(0, str(root))
from action_provider.arm_observation import observation

tree = ast.parse((root / "run_motion_pipeline_sim.py").read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "AsyncTraceWriter")
exec(compile(ast.Module(body=[node], type_ignores=[]), "actual_trace_writer", "exec"))
device = "cuda" if torch.cuda.is_available() else "cpu"
positions = torch.randn((65, 3), device=device)
quaternions = torch.zeros((65, 4), device=device)
quaternions[:, 0] = 1
velocities = torch.randn((65, 3), device=device)
durations = []
with tempfile.TemporaryDirectory(prefix="arm-observation-benchmark-") as directory:
    target = Path(directory) / "synthetic.jsonl"
    with AsyncTraceWriter(target) as writer:
        for step in range(2020):
            start = time.perf_counter()
            rows = torch.cat((positions, quaternions, velocities, velocities), dim=-1).cpu().tolist()
            packet = observation([f"link_{i}" for i in range(65)], rows,
                session_id="synthetic-not-robot", step=step, dt=.005, monotonic_s=time.monotonic())
            writer.write(json.dumps(packet, separators=(",", ":")) + "\n")
            elapsed = time.perf_counter() - start
            if step >= 20:
                durations.append(elapsed * 1000)
            time.sleep(max(0, .005 - elapsed))
    durations.sort()
    print(json.dumps(dict(scope="synthetic_capture_overhead_not_live_acceptance", device=device,
        samples=len(durations), p50_ms=durations[len(durations)//2],
        p95_ms=durations[int(len(durations)*.95)], max_ms=max(durations),
        above_5ms=sum(v > 5 for v in durations), bytes=target.stat().st_size,
        writer=writer.stats())))
