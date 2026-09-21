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

## Continuous diagnostic transport milestone

- e6faa8b: causal adapter baseline and five passing unit tests.
- Added raw PICO conversion preserving translation from the same sample and upstream quaternion conventions. Added a separate MSI raw snapshot forwarder (5558), latest-only lab subscriber (15558), and controller-free reference/status outputs. Existing SONIC endpoints are not engaged by this service.
- Wall-clock recorded-input test: 1500 output frames from 30.0226 s raw input in 30.1069 s wall time; retarget p95 13.49 ms; source-to-reference p95 25.18 ms. 1198 raw samples skipped intentionally while selecting latest data at 50 Hz from ~90 Hz input. This excludes transport, controller, physics, and display delay.
- Socket smoke test PASS: fresh unarmed, stale after 250 ms, new-session reset, duplicates do not refresh freshness. Ran isolated from all controller sockets using the rebuilt image.
- Built social-motion/pico-live:development, image config sha256:f480905204670f6b0c2e066e431e5e871e54d03e3e3cd766d3299225bff6f413, with pinned pyzmq 26.4.0. Source Dockerfile tracked; this image serves diagnostic retargeting, not an armed controller.

## Performance experiment configuration

The running GPU runner now appends deploy/pico-live-performance.override.yml after the existing composition and arm-observation overlays. It sets trace output to 50 Hz and disables optional foot/arm geometry output; the 200 Hz critical safety monitor remains unchanged. Video trial 20260921T035426Z_0001 completed at 0.6361 RTF (15.98 s reference / 25.1225 s wall); median physics 5.75 ms, monitor 0.70 ms. PICO comparison is pending.

Sibling unitree_sim_isaaclab commit d774f2d on feature/pico-live-teleoperation guards CUDA-only memory telemetry when using CPU physics. Python compilation and diff checks passed. Primary launcher now accepts opt-in ISAAC_RUNNER_DEVICE (default cuda:0); CPU overlay is experimental and must be validated independently. No physics timestep or safety threshold changed.

## Later milestones and experiment outcomes

- 78e4cd4: unarmed live service, raw snapshot transport, controller-free wall-clock test and reproducible image.
- c620b5a: clock-aware telemetry, retained hand drives in vectorized actuator writes, and receiver compose profile. Built image social-motion/pico-live:c620b5a = sha256:2a7cdb017046c524610f729d93a7135468685f5c7d71ea64ea0ff588bf06252d. Receiver remains UNARMED; MSI raw-forwarder 5558 and SSH reverse listener lab 15558 are active. Existing SDK reader and old transport were preserved.
- Clock measurement over one SSH connection: lab minus MSI -0.905 ms, uncertainty ±12.181 ms (best RTT 24.36 ms). Host clock only, not headset sensor-clock calibration. Estimates expire after 300 seconds; uncertainty above 25 ms is unverified. Six protocol/freshness/clock unit tests pass.
- Causal versus offline joint reference: time alignment removes 4 s entry padding and accounts for offline source starting at 0.08 s. RMSE 0.02353 rad (1.35 degrees), peak 0.34893 rad. This is reference consistency, NOT actual robot tracking or human imitation accuracy.
- GPU 50 Hz tracing, no optional geometry/contact diagnostics: video 0.63608 RTF; PICO 0.63044 RTF, both COMPLETED. PICO median loop 7.37 ms, monitor 0.71 ms. This isolates the earlier large source difference to runtime configuration rather than proving a PICO-specific physics cost.
- CPU physics experiment FAILED existing startup safety gate: right-knee speed 26.4358 rad/s exceeds configured 20 rad/s. Restored GPU; limits were never relaxed.
- 9b10260 and sibling a75015a: zero-gain write shortcut was unit-tested but correctly REFUSED the deployment asset because hand drives have nonzero gains. Not active.
- Sibling cf11b3c added phase profiling. An early return skipped counters and prevented LowState publication; corrected in counter-preservation follow-up 6169713 and covered by regression commit 57587ae. No threshold changes. Later sibling 838348d adds optional vectorized writes.
- Vectorized standard implicit actuator writes preserve all hand PD drives and all force/position/velocity targets; four equivalence/refusal tests pass. Video trial 20260921T042026Z_0001 COMPLETED at 0.76911 RTF; median scene write fell from 1.49 to 0.283 ms. This remains an opt-in candidate, not real-time qualification. Per-actuator diagnostic caches are not consumed by the zero-reward task; aggregate torque diagnostics are retained.
- c5acfad and sibling 9afd84d: optional CPU placement of the unchanged safety function using one packed GPU-to-host copy. GPU/CPU comparison passes for 5 random inputs × 3 command/tracking modes × finite/nonfinite cases; checks still run at every 5 ms physics step. PICO physics acceptance trial pending.

Current active experiment overlays: pico-live-performance, pico-live-vectorized-write, pico-live-host-metrics after existing composition and arm-observation overlays. CPU and zero-gain overlays are NOT active. WebRTC remains a separate renderer. No hardware commands have been issued; live SONIC source arming and headset acceptance remain outstanding.

## Latest validation and active configuration (2026-09-21)

The full plan is NOT complete; live control is not qualified or enabled.

- Host critical-metrics experiment: neutral startup/standby failed existing waist error checks in sessions 042821Z and 042931Z. This was before PICO playback. Numeric function equivalence does not establish runtime qualification. Removed overlay.
- Vectorized writer alone: the subsequent PICO request was rejected because stationary standing did not settle (043408Z session). Its faster video result does not qualify it for PICO. Removed overlay.
- Restored baseline: existing composition + arm-observation + pico-live-performance overlays, GPU physics; no CPU, zero-gain, vectorized-write or host-metrics overlays. WebRTC remains separate. Full PICO replay 044012Z_0001 COMPLETED: 37.94 s reference / 59.230 s wall = 0.64055 RTF.
- Native trace analysis (no lag optimization): 2002 samples, joint RMSE 6.3296 degrees, peak 39.3415 degrees, observed torque-limit ratio 1.11751, peak root tilt 35.2275 degrees. These FAIL proposed acceptance. 50 Hz traces may miss extrema; safety monitoring remains 200 Hz. See pico-live-baseline-results.json and tools/analysis/analyze_pico_trace.py. Completion of playback is not qualification.
- Added simulation-only live session adapter and supervisor source hook, disabled by default. Admission rejects wrong DDS isolation, stale/reconnected source, clock uncertainty, joint limits and RTF outside 0.95–1.05; bounded frame history, sequence checks, entry blend and watchdog are implemented. Five admission tests pass, including rejection before publisher creation. Actual armed stream and standing-recovery behavior are NOT physics-validated. Session calibration and public controls remain unfinished.
- Built source-based social-motion/sonic:pico-live-development, image config sha256:4717794684cc159abafef3ff1a0a50736f6dd50fda1c81b9970d9d6182dba528. It patches only the idle PICO hook into the pinned composition image instead of overwriting that image's supervisor. Five tests pass inside the built image with network disabled and no source-package mounts. This image is NOT deployed; the existing SONIC controller remains active.
- Latest touched sibling revision: unitree_sim_isaaclab 9afd84d. All its performance switches remain opt-in; tested active baseline uses none of them.

### Remaining implementation and acceptance work

1. Achieve real-time physics without losing neutral-standing stability; investigate controller/physics timing interaction before adopting write optimizations.
2. Complete session heading/floor/scale/neutral calibration and causal foot conditioning; reduce peak joint and ankle effort errors. Current hardcoded height is not per-user calibration.
3. Qualify v1 streaming and standing recovery using recorded input; add public calibrate/arm/pause/stop controls after recovery tests, then source-switch/dropout tests.
4. Record aligned raw/reference/actual telemetry, normalized wrist/ankle errors, and native WebRTC wall-clock video. Current metrics do not establish human-to-reference accuracy or visible latency.
5. Run three two-minute headset sessions only after recorded tests pass. Receiver is unarmed; fresh headset input has not been observed in this development run. No hardware deployment.

Commit 7891f8a contains the guarded session adapter, admission tests, source image recipe, trace analyzer and current runbook. Rebuilt controller image is also tagged social-motion/sonic:pico-live-7891f8a (not deployed). Documentation/pinning predecessor: 9fde0e7.

## Follow-up: scheduling and motor effort semantics

- Four-thread Kit/PXR experiment (051632Z_0001) COMPLETED at 0.640331 RTF versus baseline 0.640553: no meaningful improvement. Overlay removed. Original launcher experiment initially rejected a split --kit_args argument; fixed to --kit_args=VALUE, then reran from a fresh session. Shell syntax and invalid values 0, -1, 65, abc were checked. No model, timestep or safety thresholds changed.
- Sibling commit 66b6c81 corrects a missing actuation limit: installed ImplicitActuator.compute clips its diagnostic applied_effort but returns the original feed-forward control_action. The bridge previously assumed the task bounded that effort. It now clips blended asset-space effort using the same limits as the monitor and records requested effort separately. Three output-stage tests pass (including CPU/CUDA mapping, unchanged in-range values, NaN visibility); physics validation pending. This changes actuation behavior and must be assessed for stability/tracking, not treated as qualification merely because bounded output cannot exceed 1.0.
- Trace analysis now reports saturation frequency and peak removed effort when requested/applied vectors are available; absent historical telemetry remains unknown, not zero.

### Effort-limit physics validation

Sibling 66b6c81 was loaded by recreating the bind-mounted runner with the original GPU/performance overlays only. PICO trial 052119Z_0001 COMPLETED: 0.634928 RTF, joint RMSE 6.32806 degrees, peak 39.51229 degrees, observed applied torque ratio 1.0. Requested versus applied telemetry shows saturation in 7/2002 samples (0.34965%), peak removal 4.61787 Nm at left_ankle_pitch_joint. Tracking remains essentially at baseline; runtime performance and peak tracking acceptance still fail. The torque ratio is now bounded by construction, so it is not independent evidence of improved motion quality. Full report: docs/pico-live-effort-limited-results.json. Trace sampling is 50 Hz; monitor/actuation remain 200 Hz.

Current active configuration retains the explicit effort correction. The four-thread, vectorized-write, host-metrics, CPU, and zero-gain experiments are inactive. The renderer and unarmed receiver remain separate. Source repositories are bind-mounted into the recreated runner, so the existing Isaac image revision is unchanged. No simulator library files were patched. Live controller remains disabled.

## Restored access and lag attribution

MSI-to-lab SSH restored after VPN connection. Isaac status is fresh/INTERACTIVE, robot upright; receiver remains STALE and unarmed. Added reproducible read-only tools/analysis/analyze_pico_lag.py and compact docs/pico-lag-analysis.json. Scanned -500..500 ms simulation-time shifts at 20 ms increments, with fixed interior reference frames 225..1669 within each trial. Excludes transitions/holds; values therefore differ from full-clip metrics. Old 200 Hz and newer 50 Hz traces have different sample counts.

Latest limited-effort run: best global lag 20 ms simulation time; RMSE 6.2800 -> 6.1821 degrees, squared-error reduction 3.09%; peak 38.7816 -> 38.0748 degrees. Earlier 0.474x trial: 5.9397 -> 5.8967 degrees. Faster unclipped trial: 6.2450 -> 6.1433 degrees. A uniform response delay does not explain most observed reference-to-robot error. This does NOT establish that all residual error would occur on hardware; model/controller/reference contributions remain unresolved. Per-joint best shifts are descriptive, not measured sensor latency; waist pitch hits the negative search boundary and must not be interpreted as anticipation. Wrists, knees, and waist pitch dominate RMSE.

Validation: analyzed all three native traces, finite matched segment arrays with common samples across candidate shifts; source compiles and git diff check passes. No controller commands or runtime changes were made during this analysis. Slower-reference and phase-specific experiments remain outstanding.

## Accuracy diagnosis checkpoint: artifact and analysis tooling

Added pico_accuracy.py with explicit entry/active/exit/neutral segmentation, per-joint signed bias/RMSE/peak, requested/applied saturation, and same-model wrist/ankle FK in native world and pelvis coordinates. Height normalization is the G1 geometry extent in official neutral pose (1.288063 m), not assumed human height. Full reference FK reproduces the original body CSV exactly; inverse joint ordering round-trips exactly. Model mismatch remains untested.

Derived half-speed active motion (3394 total frames) and five pose holds (neutral plus original frames 214, 538, 737, 1174; 2400 frames) both pass existing validation without changed thresholds. Three regression tests pass: exact original poses at half-speed even frames and recomputed velocities; five-second holds with two-second evaluation windows; known 10 cm world translation produces 10 cm world error and zero pelvis-relative error. Original recording/artifact untouched. Generated artifacts and reports reside under motion_exchange, outside Git.

Environment manifest: docs/pico-accuracy-environment.json. Source generator/runtime pinned to social-motion/video-gem-gmr:16bebf4-bb1bbe4-workspace-v2; source scripts mounted read-only, so no image patch is needed. Initial analysis shows large entry elbow/waist errors and substantial world versus local endpoint difference; these are observations, not identified causes. Nine fresh-session comparisons will run in rotating condition order with failures recorded explicitly. Existing recording has up to 1.52 m reference root displacement; it is retained for diagnosis, not treated as a stationary-standing qualification clip.

Study harness initialization fix: SONIC intentionally replaces the runner READY session during fresh startup. Accept that handover before INTERACTIVE and bind the execution to the resulting session. The first setup attempt was interrupted before playback and remains in accuracy-v1/trials; it is not a physical tracking failure. Strict session matching still applies after execution begins.

Reference-interface inspection during trials: deployed policy/release/observation_config.yaml uses g1 encoder mode 0 with joint positions, velocities and anchor orientation at 10 future frames spaced 5 frames apart. The furthest input is 45 frames / 0.9 simulation seconds ahead. Root XYZ position is not among required G1-mode observations; mode filtering computes only required fields. Therefore absolute world-path error is an imitation metric, not evidence that the G1 policy failed an explicit Cartesian position command. No policy input channels were changed.

Pose-hold analysis still reports the requested final two seconds, but additionally reports the portion before the next transition enters the 0.9 s lookahead. A hold that anticipates the next movement must not be classified as a static bias without checking this subwindow. This is analysis-only; references and controller remain unchanged.
