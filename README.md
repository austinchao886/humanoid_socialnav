# Kimodo → Validation → SONIC → Isaac Sim

Pinned sources:

- GR00T-WholeBodyControl fork `60c561b65d1ce2bba59c4cc752a8ffefb51d4884`
- Kimodo `1aece8c124d73d255ceff5086d983b844c9f4e94`
- Kimodo Viser `7c82ad8f8640bad9dff8ded5c5eee908eeb08f11`
- Unitree Isaac Lab fork `28a9944ef7ab0d61242cffe454e2a2329fbd39fb`
- Unitree SDK2 Python `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`
- GENMO/GEM `16bebf402d8893184249ee206d957b8248cd8310`
- GMR `bb1bbe40774794fceb2a7c579a3464a28e68c844`

The canonical machine-readable dependency and local container snapshot is
[`versions.lock.yaml`](versions.lock.yaml).

Repository ownership and the staged refactor rules are documented in
[`docs/architecture/workspace-layout.md`](docs/architecture/workspace-layout.md).
Service-owned integration code lives under `services/`, deployment assets live
under `deploy/`, and human-friendly commands live under `tools/run/`. Temporary
compatibility symlinks keep the former paths working during migration.

Run the complete repository check with a supported Python version (3.10–3.12
recommended for the pinned robotics dependencies):

```bash
PYTHON_BIN=python3.12 ./tools/dev/check.sh
```

The existing `isaac-lab` container remains independent on GPU 0. `kimodo`
generates immutable G1 references on GPU 1. `sonic-tracker` validates manually
approved references and drives the simulator through simulation-only Unitree
DDS topics on GPU 1.

The deployment uses DDS domain `42`. External JSON motion control uses the host
interface (`wlp69s0` in the current Compose file). The simulator's low-level
`rt/socialnav_sim/g1/lowstate`, `rt/socialnav_sim/g1/lowcmd`, and secondary IMU
topics are isolated on loopback (`lo`). The bridge refuses the physical
`rt/lowcmd` topic.

## Setup

```bash
cd /home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline
./tools/bootstrap/bootstrap.sh
./tools/dev/start.sh
```

Start the existing Isaac container with the pinned official SONIC G1 asset and
the physics-driven DDS adapter.  The runner holds the floating base only during
frame-zero settling, releases it for execution, monitors fall/NaN/velocity, and
records the actual state and tracking error:

When `--elastic-target-height` is omitted, the runner holds the asset's spawn
height while waiting for approval, then automatically aligns the elastic target
to reference frame zero. Do not force `1.0 m` for the 0.76 m neutral reference.

```bash
docker exec -it isaac-lab bash
cd /socialnav_humanoid_ws/unitree_sim_isaaclab
DDS_DOMAIN=42 DDS_INTERFACE=lo \
SIM_LOWSTATE_TOPIC=rt/socialnav_sim/g1/lowstate \
SIM_LOWCMD_TOPIC=rt/socialnav_sim/g1/lowcmd \
/isaac-sim/python.sh run_motion_pipeline_sim.py \
  --task Isaac-Flat-G129-SONIC-Official \
  --asset-profile sonic_official_g1 \
  --exit-after-command
```

The initial bootstrap downloads large model files. A Hugging Face token may be required by the upstream model repositories; place it at `~/.cache/huggingface/token` if prompted.

## Use

Run the CLI with a Python environment containing `unitree_sdk2py` and this package:

```bash
DDS_DOMAIN=42 motion-cli listen
DDS_DOMAIN=42 motion-cli generate "A person stands and waves their right hand, then returns to neutral." --request-id wave-001 --seed 42 --duration 4
DDS_DOMAIN=42 motion-cli control approve_execute wave-001 <motion-id-from-status>
DDS_DOMAIN=42 motion-cli control abort wave-001 <motion-id>
```

DDS topics use `std_msgs/String_` containing versioned JSON:

- `rt/motion/generate/cmd`
- `rt/motion/control/cmd`
- `rt/motion/status`

Artifacts are stored in `../motion_exchange/<motion_id>/`. Only references with a valid `validation.json` can cross the SONIC execution gate.

`approve_execute` is intentionally required.  The supervisor re-runs validation,
records `approval.json`, waits for SONIC to report the exact motion-completed
event, and treats timeout or safety output as failure rather than success.

## Acceptance stages

### 1. Official SONIC baseline

The pinned official G1 has passed unsupported Isaac execution for
`squat_001__A359` twice (including reset/re-run) and
`neutral_kick_R_001__A543` once. Reports are under
`motion_exchange/executions/`; fixed-root results are diagnostic-only.

### 2. Deterministic Isaac references

Generate the three immutable 50 Hz references with official MuJoCo FK:

```bash
docker exec kimodo python3 -m motion_pipeline.deterministic --kind all
```

The current artifacts are `isaac-neutral-v1`, `isaac-arm-raise-v1`, and
`isaac-shallow-squat-v1`. Each has a preview and valid validation report but no
automatic approval. Start the qualified official endpoint, inspect the preview,
then approve exactly one artifact with `motion-cli control approve_execute`.
The runner exits after that command, so reset/restart the simulator before the
next approval.

### 3. Kimodo references

Only after the deterministic references pass unsupported execution, start with
the already-valid `wave-official-e2e-005-083aa901`, then generate bow and
side-step. An `INVALID_REFERENCE` directory remains immutable and rejected; it
must never be patched, approved, or sent to SONIC.

### 4. Latency acceptance

The warm headless gate uses the official asset, 200 Hz physics/safety, 50 Hz
reference/policy updates, and a 60 Hz simulator-only LowCmd target stream. Run
three consecutive closed-loop executions with:

```bash
cd /home/adcs-public-robot/Documents/socialnav_humanoid_ws
WARM_RUNS=3 motion_pipeline/tools/acceptance/run_latency_acceptance.sh \
  wave-official-e2e-005-083aa901 \
  bow-e2e-004-retargeted-95pct \
  side-step-e2e-002-0aa18456
```

Every motion must finish `COMPLETED` and every playback must reach realtime
factor `>= 0.8`; duration budgets use the three-run median. The combined report
contains per-motion gates, physics/DDS/SONIC percentiles, image IDs, artifact
checksums, and the explicit fixed-seed generation run group:

```text
motion_exchange/diagnostics/performance/latency_e2e_final_20260816.json
```

Offline trace videos run after `COMPLETED` and therefore never block the next
motion. Pass both the trace and owning motion ID so the runner records
`offline_video_output` in that artifact's `timing.json`:

```bash
/isaac-sim/python.sh run_motion_pipeline_sim.py \
  --task Isaac-Flat-G129-SONIC-Official \
  --asset-profile sonic_official_g1 \
  --headless --enable_cameras --video \
  --replay-motion-id wave-official-e2e-005-083aa901 \
  --replay-trace /motion_exchange/executions/<completed-trace>.jsonl \
  --video-output /motion_exchange/executions/wave-replay.mp4
```

Use `--replay-cold-bootstrap` for a first shader/cache build; it stays in the
waterfall but is excluded from the warm latency gate.

### 4.1 Interactive WebRTC visualization

Do not run the 200 Hz control simulation in the rendering process. The
qualified interactive layout keeps authoritative physics and SONIC on GPU 0
in the headless runner, while an explicitly non-authoritative viewer on GPU 1
mirrors the actual root pose and 29-DOF state through a fixed 300-byte mmap.
The viewer does not subscribe to or publish LowCmd and cannot affect safety or
completion decisions.

Start the persistent WebRTC viewer (TCP 49100) with its opt-in Compose profile:

```bash
cd /home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline
docker compose -f docker-compose.motion.yml --profile gui up -d isaac-visualizer
```

Start the authoritative runner with visualization snapshots enabled:

```bash
docker exec -d isaac-lab-headless-latency sh -lc '
  cd /socialnav_humanoid_ws/unitree_sim_isaaclab
  DDS_DOMAIN=42 DDS_INTERFACE=lo \
  CYCLONEDDS_URI=file:///socialnav_humanoid_ws/motion_pipeline/deploy/config/cyclonedds-localhost.xml \
  SIM_LOWSTATE_TOPIC=rt/socialnav_sim/g1/lowstate \
  SIM_LOWCMD_TOPIC=rt/socialnav_sim/g1/lowcmd \
  /isaac-sim/python.sh run_motion_pipeline_sim.py \
    --task Isaac-Flat-G129-SONIC-Official \
    --asset-profile sonic_official_g1 \
    --headless --exit-after-command --trace-hz 5 \
    --visual-state-output /socialnav_humanoid_ws/motion_exchange/.runtime/g1_visual_state.bin'
```

`motion_exchange/.runtime/visualizer_status.json` reports `STREAMING`, achieved
FPS, render latency, and fresh-snapshot age. Stop the viewer cleanly so it
writes its final performance report:

```bash
docker compose -f docker-compose.motion.yml --profile gui stop isaac-visualizer
```

The accepted wave benchmark is recorded in
`motion_exchange/diagnostics/performance/gui_split_latency_20260817.json`.
The three authoritative playbacks completed at realtime factors 0.827, 0.809,
and 0.806 (median 0.809); the viewer achieved 20.0 FPS with 25.3 ms p95 fresh
snapshot age. Direct joint/root writes are permitted only in this read-only
viewer and must never be used to claim physical tracking success.

### 5. Custom Dex1 USD

Asset profiles live in `deploy/config/asset_profiles/`:

- `sonic_official_g1`: qualified execution profile.
- `g1_dex1_wholebody`: `pending_calibration`; full motion execution is blocked.

Export and compare resolved PhysX properties instead of relying on USD names:

```bash
/isaac-sim/python.sh inspect_g1_asset.py \
  --task Isaac-Flat-G129-Dex1-Wholebody \
  --profile-id g1_dex1_wholebody \
  --output /socialnav_humanoid_ws/motion_exchange/diagnostics/asset_snapshot.json \
  --headless --device cuda:0

python3 /socialnav_humanoid_ws/motion_pipeline/tools/analysis/compare_asset_snapshots.py \
  --baseline /socialnav_humanoid_ws/motion_exchange/diagnostics/asset_snapshot_sonic_official_g1.json \
  --candidate /socialnav_humanoid_ws/motion_exchange/diagnostics/asset_snapshot.json \
  --output /socialnav_humanoid_ws/motion_exchange/diagnostics/asset_compatibility.json
```

For axis/dynamic calibration, launch the custom task only with
`--fixed-root-lowcmd-diagnostic --bootstrap-support fixed`, then use
`tools/analysis/lowcmd_semantics_probe.py` with amplitude no greater than `0.1 rad`.
The diagnostic endpoint never advertises `READY` and the supervisor rejects it.
Qualification requires representative leg, waist, shoulder, elbow, and wrist
response comparisons—not a single joint or fixed-root standing result.

## V1 prompts

```text
A person stands and waves their right hand, then returns to a neutral standing pose.
A person bows politely while standing, then returns upright.
A person takes one short step to the right and returns to a stable standing pose.
```

Navigation, locomotion handoff, concurrent gestures, and real-robot deployment are intentionally outside v1.

## Offline phone-video source (GEM → GMR → G1)

The opt-in `video` profile is separate from Kimodo and consumes only
`rt/motion/generate/video/cmd`. It accepts fixed-camera, single-person MP4/MOV
clips lasting 3–10 seconds. Put source files anywhere under
`/home/adcs-public-robot/Documents/socialnav_humanoid_ws` so the read-only
workspace mount preserves the absolute path inside the worker.

Licensed model setup (one-time): download `SMPLX_NEUTRAL.npz` from the official
SMPL-X site after accepting its license, then place it at:

```text
/home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_models/body_models/smplx/SMPLX_NEUTRAL.npz
```

GEM, HMR2, ViTPose-H, and YOLO checkpoints live under `motion_models/`; the
worker never downloads weights at request time. Start the worker and publish a
request from the same workspace:

```bash
cd /home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline
docker compose -f docker-compose.motion.yml --profile video up -d video-generator

docker run --rm --network host \
  -e PYTHONPATH=/unitree_sdk2_python:/workspace \
  -v /home/adcs-public-robot/Documents/socialnav_humanoid_ws:/home/adcs-public-robot/Documents/socialnav_humanoid_ws:ro \
  -v "$PWD":/workspace:ro \
  -v "$PWD/../unitree_sdk2_python":/unitree_sdk2_python:ro \
  social-motion/video-gem-gmr:16bebf4-bb1bbe4 \
  python -m motion_pipeline.cli --domain 42 --interface lo \
  generate-video /home/adcs-public-robot/Documents/socialnav_humanoid_ws/phone_videos/wave.mp4 \
  --model gem --static-camera
```

The CLI records source SHA-256, original FPS, static-camera mode, and the exact
model commit in the immutable DDS request. The worker rejects mutations or
version mismatches before inference. A valid artifact includes the original and
normalized videos, canonical world/in-camera SMPL, camera intrinsics, tracking
report, overlay/world previews, conditioned G1 qpos, `foot_contacts.csv`,
retarget preview, SONIC CSVs, manifest, and validation report. Video artifacts
still require manual `approve_execute`; there is no real-robot path in v1.
