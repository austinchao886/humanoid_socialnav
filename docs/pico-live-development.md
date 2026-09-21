# Live PICO development log

Scope: standing full-body teleoperation in Isaac; physical hardware and walking are excluded. All changes stay on feature/pico-live-teleoperation; no push or merge.

## Workspace inventory (2026-09-21 UTC)

Workspace root: /home/adcs-public-robot/Documents/socialnav_humanoid_ws

| Repository | Starting branch | Starting revision | Initial changes |
|---|---|---|---|
| motion_pipeline | main | 142f06a65bc7392a1af8ae81e4d4b10a132a9480 | Existing PICO integration, checkpointed below |
| unitree_sim_isaaclab | social-motion/sonic-pipeline-sim | 743f8fffb78e959dfe82ed5beccf67634dd4dde8 | Clean |
| GMR | master | bb1bbe40774794fceb2a7c579a3464a28e68c844 | Clean |
| GEM | main | 16bebf402d8893184249ee206d957b8248cd8310 | Clean |
| unitree_sdk2_python | master | 65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5 | Clean |
| cyclonedds | releases/0.10.x | 5041f3560c088c99e5088b2b8520b69169621196 | Existing untracked install/, preserved |

Nested GR00T-WholeBodyControl vendor checkout was clean. Create branches in siblings only when changes there are necessary.

## Runtime provenance

SONIC image sha256:43f831dd6465d7ad00d391c97191d5adf4f066241ebb3db32223cd1e4e75489e.
Isaac runner/visualizer image sha256:87513b411f492bc9b1cb0b2fd7be749408c18ec0af30cff8b84cf82bdc4c8aa4.
Retarget/test image social-motion/video-gem-gmr:16bebf4-bb1bbe4-workspace-v2.
Isaac binds the sibling workspace and primary source checkout. SONIC currently embeds Python source plus earlier runtime patches; do not overwrite its supervisor with the repository version without reconciling the existing differences. Durable rebuilt image is still required before rollout.

## Commits and validation

- 2eb96b7: checkpoint pre-existing PICO offline/diagnostic integration, preserving the reviewed two-line supervisor hook. This is an evidence checkpoint, not live qualification.
- Causal adapter milestone (commit containing this entry): added motion_pipeline/pico_live.py, protocol/freshness unit tests, and tools/analysis/benchmark_pico_live.py. Five tests passed with standard-library unittest in pinned container; pytest is absent on host and image. No host dependencies installed.
- 100-frame causal benchmark: initial SMPL-X evaluation p50 14.72 ms/p95 17.25 ms; cached skeleton FK p50 7.33 ms/p95 8.82 ms/max 8.95 ms after 10 warmup/verification frames. Maximum skeleton position difference versus full SMPL-X over first 10 frames: 2.05e-7 m. First-frame cold conversion ~35.65 ms. This benchmark does NOT engage the controller and does NOT measure end-to-end latency.

## Outstanding gates

Continuous transport must preserve root translation and exact sample timestamp. Add live service and bounded latest-frame processing; calibrate heading/floor/body scale; test source transitions and standing recovery; reconcile supervisor image; profile physics and diagnostics without weakening safety; investigate torque exceedance; perform same-settings video/PICO comparison and three two-minute headset sessions. No claim of live control or hardware readiness.

Large recordings, videos and raw traces stay under sibling motion_exchange, outside Git. Test outputs: motion_exchange/diagnostics/pico/live-causal-benchmark.json and .jsonl.
