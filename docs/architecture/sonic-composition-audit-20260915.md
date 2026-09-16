# Continuous composition capability audit — 2026-09-15

## Latest status — Git cleanup, 2026-09-16

Code, tests, analysis tools and experimental deployment configuration have been reviewed and committed in bounded batches. This document is a historical investigation log: earlier headings and statements about pending work, running images, or no motion changes describe their individual stages, not the current aggregate state. Source organization is not simulation or hardware qualification. Live cancellation, repeated transitions, stopping oscillation and sole-slip quality still require dynamic validation.

The user authorized synchronization of this updated audit to the same lab host and a documentation commit. The old remote copy was preserved at `/tmp/sonic-audit-backup.9dZHOZ/sonic-composition-audit-20260915.md` (temporary backup; SHA-256 `a6c9e222d2c98b68951c1102d0704b55defdf83391c81db22817edd4ce03362b`). The updated log retained every old line before this clarification was added. Main source/config HEAD before this documentation commit is `40e21d05e7b6513524f30cc0a170adece31f4d39`; vendor is `15abe6d3ee0e86237ad5b52a6e36fe052c803fd6`; simulator is `1ed7a193ed9cb99b67d397edff1f17d2400d53be`. No push, history rewrite, rebuild or live-container change is part of this documentation synchronization.

## Initial audit scope

Status at the initial audit: source-level capability audit, not simulation qualification. No controller parameters, mode switches, or motions changed during that initial read-only stage.

## Authoritative deployment

- Host: adcspublicrobot-codex.
- Repository: /home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline.
- Pipeline HEAD: 065caab; initial worktree clean.
- SONIC vendor HEAD: 9084730c6353f4c9e4c6997fc350744c3ea82e82; vendor worktree clean.
- Running image ID: sha256:817e8c9c356f20e3117721c0f9009c50372f27df57a5e217c5e7d7e459158abf.
- Live process: g1_deploy_onnx_ref, input-type manager, release encoder/decoder, target_vel/V2 planner, persistent reference pool.
- Simulation-only Compose endpoints: DDS domain 42, loopback, rt/socialnav_sim/g1/*.
- Encoder SHA256: 013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3.
- Decoder SHA256: c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed.
- Planner SHA256: 39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea.

## Findings in pinned source

All paths below are relative to vendor/GR00T-WholeBodyControl/gear_sonic_deploy.

1. `src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp` accepts optional 17-DOF upper_body_position and upper_body_velocity along with planner commands. Around lines 582–645 it resets locomotion to IDLE and clears upper-body control after a 1000 ms planner timeout. This is a source behavior, not a demonstrated smooth release.
2. `src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp:3131` snapshots upper-body input. Around lines 799 and 876, full-joint reference observation assembly substitutes upper-body position/velocity before encoding. This is reference-level composition, not motor-command mixing.
3. The upper-body order is explicitly the IsaacLab subset in `include/policy_parameters.hpp:80`: 2,5,8,11,12,15,16,19,20,21,22,23,24,25,26,27,28. It is NOT a contiguous MuJoCo arm array. Waist is included; a wave adapter must retain intentional waist ownership.
4. `include/input_interface/interface_manager.hpp` forwards several VR/hand/token getters but does not override GetUpperBodyJointPositions/GetUpperBodyJointVelocities/HasUpperBodyControl. The controller accesses the manager through InputInterface; inherited getters read the manager's own buffers. Thus the inspected delegate upper-body route is not transparently exposed in manager mode. Confirm with a focused native regression before patching.
5. Full-joint multi-frame assembly repeats the current upper-body sample across the future reference window. It does not consume a future gesture chunk per frame. Position/velocity consistency and phase timing need explicit design and tests.
6. Base upper-body getters check availability and buffer presence, not sample age. Position and velocity are separate buffers; velocity validity is ignored by the control-loop snapshot. Do not enable a new producer until paired sample validation, freshness, cancellation, and smooth release behavior are specified.
7. Mounted release observation_config.yaml enables full-joint position/velocity windows and multiple encoder modes. This plus source support is evidence of an existing candidate route, NOT proof the installed model safely executes the proposed gesture or that the baked binary exactly matches every host header.

## Next implementation steps

- Verify image-source parity for the relevant headers and executable build provenance.
- Add a native manager-delegation regression; fix the missing forwarding without switching the running controller.
- Design an explicitly authorized, timestamped gesture input path while retaining gamepad locomotion/deadman ownership; merely switching to ZMQ would not preserve current joystick behavior.
- Validate 17-DOF ordering, finite paired q/dq, sample/session identity, timeout, entry/exit envelopes, and future-window semantics offline before a rebuild or live test.
- Add reproducible phase-latency summaries from existing round logs, then new baseline captures. Existing historical rounds are not new composition acceptance.
- Preserve conservative fallback and all existing safety gates. No new checkpoint or training is justified yet; model capability remains to be tested.

## Outstanding acceptance

### Explicit gait-window experiment profile

Candidate06 deployed in sessionbf91243d0b4d455b865dd37e0a5be107. First no-gesture harness trial900e87283cf84f3d9bb96c95aa24daba stopped during initial approach: at sim29.925 (approach began28.21), window planar peak.660286,mean.461717,vertical.175662,tilt.079707,torque.572163. No gesture sent. Thresholds were NOT increased. This demonstrates high-rate admission rejecting a peak, not evidence of a fall.

Corrected test-stage scope: initial approach now8 wall seconds under ordinary pose/torque/session/native joystick safety, then explicitly enable complete gait admission BEFORE approval and throughout matched/composition+finish. Disable only AFTER completion for resumed joystick. Every event records gait_admission_active; all trace peaks still analyzed for quality. Same timeline applies to baseline and gesture arms. This avoids conflating admission of a new reference with initial joystick acceleration. Pose/torque checks and existing native safety remain during non-composition phases. Four script control tests passed; revised baseline pending.

Added opt-in SONIC_GESTURE_GAIT_ENVELOPE=1 in Supervisor; default require_composition_envelope remains legacy instantaneous behavior. New simulation experimental profile checks full same-session1s physics window (190–210 samples,~1s coverage,maxgap<=.006,aligned to current status), latest.5s mean PATH speed<=.5m/s, instantaneous/window planar peak<=.65,vertical peak<=.20. Current and window tilt tightened to<=.20rad; window height .70–.90,torque ratio<=1. Freshness,finite data, unsupported/handoff/native/private-channel guards remain. Prepared execution report records admission_profile. This is a changed empirical experiment envelope with explicit margins, NOT a certified safe region or a claim that average velocity alone is enough.

Rationale: baseline mean path max .4435 while instantaneous peak .5557; old .5 instantaneous bound confused command cap with gait excursions. Added5 test groups reject mean/peak/pose faults, stale/gapped/missing/cross-session windows, invalid numeric data, and prove default unchanged. All35 gesture+24 prepared tests passed INSIDE candidate06 image sha256:3570b596b8539e7bf8634efadf84787d8db7c83b1feee71b16207c90741d96b0. Entire old cc963860... baseline replay after1s warmup:7307 checks,zero rejections; empirical compatibility only.

walking_gesture_trial.py now supports --gait-envelope for matched admission and rejects mixing it with --baseline-diagnostics. Candidate06 override explicitly enables gait envelope; sub-floor deceleration experiment remains OFF. Controlled deployment underway; new dynamic trial pending. No checkpoint/gain/contact policy changes. No audit transfer.

### Physics-rate kinematic measurement window (no admission change)

Live runner reload verified the new status field in session3316aae5b4044b89bf721f5ec94992cf:201 samples spanning1.0s, max gap.005s, independent of1-Hz status publishing. SONIC image remains04. Initial reading was supported READY during normal startup, so this alone is not a composition-admission pass; unsupported INTERACTIVE confirmation follows before any control test.

Confirmed subsequent INTERACTIVE at sim19.455 in same new session with window ready201 samples/1s/.005s maxgap; window height .78325–.78759m,tilt peak .03238rad,torque ratio .32318. No walking/gesture issued this measurement deployment. Current image04 unchanged; actual window transport verified, runtime admission integration still pending.

Added simulator action_provider/kinematic_window.py. Runner appends vx/vy to existing packed GPU-to-CPU critical metrics without changing existing19 indices, then accumulates at each200-Hz physics step. Status includes last1s peak speeds/height/tilt/torque, sample coverage/max gap, and latest.5s time-weighted mean planar speed plus norm of mean planar velocity. Opposite motions cannot be hidden by relying only on signed velocity cancellation. Missing/nonfinite/regressing samples invalidate evidence; ready requires near1s coverage and max gap<=.006s. No command filtering or new admission threshold.

Five pure tests pass: constant integration, between-status peak retention/expiration, gap/nonfinite invalidation, clock reset, signed cancellation. Full remote runner diff checked before upload; only this instrumentation changed. Existing native safeguards unchanged. Live controlled reload underway with candidate04 unchanged.

Replay of old cc963860... steady walking produced1170 ready windows: instantaneous peak .555697m/s, maximum .5s mean speed .443465m/s, norm of mean velocity .434437m/s, peak vertical .138134m/s, tilt .060008rad, torque ratio .308764, max gap .005s. This provides measurement evidence, NOT dynamic qualification or authority to replace a peak check with an average. New gait admission must retain transient/pose/contact safeguards and be explicitly validated before walking composition.

### Full candidate05 baseline does not support adopting sub-floor stop; rolled back

Added explicit --baseline-diagnostics to walking_gesture_trial.py, mutually exclusive with --with-gesture BEFORE runtime access. It never approves gesture; retains fresh status/unsupported session/height .70–.90/tilt<=.25/torque<=1 and finite velocity checks, plus native safety. Instantaneous speed exceedances are recorded as diagnostics instead of applying the extra gesture-admission gate to ordinary joystick measurement. Production gesture_safety unchanged. Four tests pass for mutual exclusion, retaining safety checks, strict normal gesture admission and monotonic ramp.

Candidate05 completed full diagnostic baseline report .build/walking-gesture-a255d6d2a377479d92efe550ca9d56bb.json in session25bcf6ec34a64000b0d085ae9945721e. High-frequency phase analysis versus old cc963860... showed deceleration tilt .13704→.19946, final deceleration .16903→.21307rad. Contact-link proxy changed in both directions; no all-metric improvement. All phase cumulative interval-count deltas zero, max LowCmd age<=.02925s. Different initial state/gait phases mean this is not definitive causality, but insufficient evidence for adopting05.

Restored live/override to candidate04 image sha256:bd731001618035a9620745db0b447ca7548842f01123f89e5170215ad7015339 via controlled deployment; verified new session12da8e6bb2f14476ac5803c4a0ddbe46 unsupported INTERACTIVE, tilt .02786rad. Candidate05 image/reports retained. Source sub-floor deceleration now requires explicit SONIC_EXPERIMENTAL_ZERO_SPEED_DECEL=1; default restores pre-experiment behavior. Seven native tests pass under -O3 -ffast-math, including default-off and deadman/planner hold. This source opt-in update is not rebuilt into04 and is not enabled live.

Offline old steady-walk diagnostic: .5-s simulation-window mean velocity norm max .43238, median .35488m/s versus instantaneous max .55570. Shows why instantaneous root speed must not be equated with commanded translation; averaging alone is NOT a new safety qualification. Next work must establish phase/measurement-appropriate gesture admission and stop/reference handling, not keep lowering joystick input to produce a passing test or assume a smooth scalar command guarantees dynamic stability. No walking gesture yet; full objective remains incomplete. Audit remains local.

### Candidate05 deployed; stopping result is mixed, not acceptance

Controlled deployment verified image05 sha256:5f39e39df317fa84d110afa996f28a5b1f3fb7e009c203ba08073e9b16dd4c28; new session25bcf6ec34a64000b0d085ae9945721e. Matched walking harness at .25 stopped during walking BEFORE deceleration due supplemental composition planar-speed gate: sim39.12 velocity[.455133,-.212640,-.068626], tilt .011931,torque ratio .272827. Report walking-gesture-9d8b67cf83be4667912c79f5cc3b2613.json. Cannot use it to evaluate the stop fix. No gesture sent, no thresholds loosened.

Ran existing native-safety-protected short stopping diagnostic `python3 tools/acceptance/stop_transient_trial.py --walk-seconds 3 --forward 0.5` (its original checks, not a composition admission test). Completed .build/stop-transients-20260915T164644.json in same session. Compared to old .build/stop-transients-20260915T153703.json at the same status-sampling level, center 2–5s max planar speed .07549→.01392m/s and max joint speed .46123→.30793rad/s; however center0–2s planar max .14997→.42361 and joint speed2.68592→2.81945. Different gait phase/session history can dominate these short samples; not causal improvement proof. No broad quality pass.

Candidate high-rate stop windows: release76.445–87.985/center91.460–102.930, trace dt .005s, max LowCmd age .028244/.028351s, tilt .046532/.043584rad. Contact-link-origin speedp95 left/right release .003456/.026331,center .004638/.015663m/s. Link-motion proxy results are mixed versus prior5-Hz report and NOT sole-slip acceptance. Runtime remained unsupported INTERACTIVE afterward. Repeat phase-controlled A/B needed before retaining change as an improvement.

Added tools/analysis/analyze_walking_trial.py with phase-specific trace gaps, velocity bound exceedance counts, torque/tilt/command age/contact proxy and cumulative interval deltas; outputs explicitly diagnostic, not qualification. Three tests passed (between-status peak, incomplete/session mismatch, nonmonotonic trace). General baseline collection versus derived-gesture admission remains an important separation; do not infer no high-rate violation from 1-Hz status gate pass. Audit local only.

### Concrete centered-stick stop discontinuity and candidate correction

Candidate05 completed build: image sha256:5f39e39df317fa84d110afa996f28a5b1f3fb7e009c203ba08073e9b16dd4c28. Six native tests (new deceleration fixture plus existing planner-hold tests) passed inside that image under -O3 -ffast-math. Live remains candidate04; no Isaac trial of candidate05 yet. Next controlled deployment should repeat walking_gesture_trial.py --forward .25, compare phase-specific trace metrics to cc963860..., then determine whether to retain the change and proceed to walking gesture. Unit-test success alone does not qualify the sub-.2 planner input range.

Actual Gamepad.handle_input regression with live-style runtime arming and initialized planner reproduced old behavior: from normalized stick .25, centering while F2 remains pressed immediately emitted IDLE with speed -1. The internal speed is slewed but smooth_stop_complete triggered at <=.201m/s; the external output was floored to .2. Thus the previous smooth-stop implementation discarded its sub-.2 deceleration. Deadman test passed in the original implementation.

Changed only centered-stick runtime output: keep SLOW_WALK/direction and use the existing runtime_commanded_speed_ decrement (.015 per handle_input call) to zero; switch to IDLE at <=1e-6. Active-stick speed mapping/floor, yaw, PD gains, checkpoint, explicit deadman and emergency-stop branches are unchanged. Added test_gamepad_deceleration.cpp using REAL Gamepad.handle_input; three tests pass with patched header (no immediate jump, monotonic bounded decrement to zero before IDLE, deadman immediate IDLE). Original first test fails red. Source saved in vendor worktree; candidate05 building, live remains candidate04.

Local planner input implementations assign target_vel directly; MovementState documents speed0 as stationary and mode comments mention .1–.8 slow walk. This establishes interface acceptance, NOT model/dynamics qualification below .2. Candidate requires Isaac baseline comparison before adoption. No claim yet that this improves stop sway or enables walking gesture composition.

### Matched walking/deceleration harness and baseline limits

Added tools/acceptance/walking_gesture_trial.py: simulation-only public DDS approval plus existing localhost joystick bridge, same-session/unsupported/freshness checks, bounded phases (5s idle,4s approach,12s walk,4s cubic command ramp,25s standing finish,3s resumed joystick,4s ramp,10s post), deadman release in finally, failure abort when a gesture was requested, UUID report. All phase durations are WALL time; observed simulation times are recorded. --with-gesture exists but has NOT been run. No model/gain/native motion tuning.

Normalized-forward .5 baseline stopped on velocity envelope; report .build/walking-gesture-d17003a2411a494f8fb6aa4d47cc301d.json. Trace planar peak .57259m/s, vertical .10703m/s; did not send gesture. Runtime recovered to idle within same session. Verified joystick mapping is nonlinear .2–.45m/s command range, not .45 multiplied by stick. Previously implied .5=>.225m/s interpretation is incorrect.

Normalized-forward .25 baseline completed command sequence in same session2664be6365d64acab817081f2ce5e0f9; report .build/walking-gesture-cc963860a09b49ecac6a502ba0f6f358.json. HOWEVER 200-Hz trace exceeded the supplemental .5m/s planar/.15m/s vertical experiment bounds although 1-Hz status sampling missed it: planar max .555697, vertical .154813, tilt .169032, torque ratio .581884, LowCmd age .029496s. Thus this is NOT a qualified baseline and no walking gesture was approved.

Phase localization: approach planar .5488/vz .1548/tilt .0831; sustained matched walk planar .5557/vz .1381/tilt .0600,2.2544m x-displacement over6.84 simulation seconds; first deceleration tilt .1370; resumed joystick/final deceleration tilt .1690; post tilt .1679. Max joint velocity sustained walk5.5094rad/s (not itself an oscillation measure). Short-time root velocity differs materially from commanded walking speed; supplemental bounds were derived from a standing gate and command cap, not calibrated gait dynamics. Next: inspect walking/stop command behavior and establish defensible phase-aware measurements/qualification; do not silently relax limits or call a sampled-status pass a high-rate safety pass. Standalone standing composition remains two successful trials. Audit remains local.

### Same-session repeated standing composition

Second identical prepared approval completed in the SAME candidate04 session2664be6365d64acab817081f2ce5e0f9. Report gesture-15c5638adb4b4bfe808cd74c12ea23bd.json records execution_id2, origin_tick21148, first-send latency .09342255 s,1566 updates over31.44184 wall seconds. Trace motion plus five-second post window105.745–128.700: max LowCmd age .0284352 s, tilt .0288055 rad, torque ratio .3318363; cumulative accepted-interval threshold deltas all zero. Remains INTERACTIVE without another bootstrap. Two stationary completions establish basic repeated grant/stream/release, NOT walking composition, broad motion compatibility, cancellation behavior, or naturalness acceptance.

Reproduce current simulation setup from pipeline root using `docker compose -f docker-compose.motion.yml -f deploy/sonic-composition.override.yml up -d --no-build`; explicit test approval: `docker exec sonic-tracker motion-cli --domain 42 --interface lo control approve_execute wave-official-e2e-005 wave-official-e2e-005-083aa901 --prepared-plan-id 887e06bae6c25cc87ce8bb2e284a98ec158090deb656da8d6fa9f6452b6c2709`. Only after confirming fresh unsupported INTERACTIVE and no connected joystick operator. Candidate remains simulation-only. Next actual scenario increment: slow walking baseline and small wave while walking, then deceleration/return under same session, retaining existing deadman/safety.

### Candidate04 first completed standing composition

Image sha256:bd731001618035a9620745db0b447ca7548842f01123f89e5170215ad7015339 passed 30 gesture + 24 prepared installed-image tests. Deployed via existing override after controlled reset. Public approval for unchanged plan887e06... completed in session2664be6365d64acab817081f2ce5e0f9, report executions/gesture-b58aff83817342a5bedb2134ec92fc27.json. First-reference-send latency .09977285 s; stream1589 updates over31.911 wall seconds (17.96-second simulation reference). Same session remained INTERACTIVE afterward; no motion reset/bootstrap. This is one successful stationary trial, not full acceptance.

Trace isaac_g129_deployment_v1_20260915T202427Z.jsonl sampled every .005 s. Windows baseline33.06–43.055, gesture43.06–61.015, post61.02–66.02: max LowCmd age .028162/.028682/.028313 s, cumulative interval threshold-count deltas all zero (> .1/.25/.5/1 s). Max tilt .024561/.027397/.026772 rad; max torque ratio .263415/.315475/.277281. Max joint speed .09190/1.25101/.26309 rad/s; movement speed alone is NOT an oscillation measure. Actual right-arm joint excursions during gesture include .5154 and .4967 rad, establishing movement beyond a transport-only success.

Force>=20 N contact-link-origin speed p95 (left/right): baseline .001031/.008237, gesture .002348/.008554, post .001777/.007673 m/s. These are link-origin diagnostics, not sole-slip measurements and not proof of no degradation. Post and gesture bounds use first-send tick plus duration; report's after.status timestamp is sampled/stale, not exact release time. Further naturalness/oscillation/slip assessment, repeat tests, walking/deceleration composition and failure cases remain outstanding. No audit document transferred.

### Discovery wait regression isolated

Inspected actual JsonDDS.publish: first publish on a topic initializes writer then sleeps .25 seconds. Prepared execution's first EXECUTING publish occurs synchronously inside on_started after the first snapshot; Supervisor startup previously wrote READY only to a file. This exceeds the native .2-second snapshot freshness bound. New regression using actual JsonDDS with fake publisher/sleep, through _execute_prepared, fails against candidate03 with wait recorded during streaming; patched source passes with wait before streaming. Added idempotent prepare_publisher and warm STATUS_TOPIC at run startup and before prepared execution. Discovery wait and freshness bound remain unchanged; no safety gate relaxed.

Failed-trial trace independently shows small target changes through sim42.990 (largest frame delta .022 rad), then all-zero right-arm targets at43.060, preceding torque saturation43.070. This supports investigating stop-path effects rather than attributing saturation solely to the gesture. Diagnostic candidate03 built but was not deployed. Candidate04 with discovery fix is building; actual simulation retest remains required.

### First actual standing composition attempt: FAILED, not dynamically qualified

Candidate 02 bootstrapped session 70c4034b2cad49f3bcd3a628da2da992. Pretrial trace window 25.200–35.195 simulation seconds was entirely unsupported; maximum LowCmd age .028105 s, cumulative intervals over .1/.25/.5/1 s all zero. Approved trimmed prepared plan via public motion-cli. Report executions/gesture-43ac476c0fb54392bc450eec0c2b7abf.json records first reference send after .0945389 wall seconds, origin tick 8568, execution ID 1. This is send latency, not physical onset.

Attempt FAILED with Supervisor message 'composition torque exceeds actuator envelope'. Trace first ratio >1 at sim 43.070: right_wrist_yaw_joint 1.9677, tilt .02149 rad. Later peak torque ratio 35.3875 and tilt .82787 rad; session ended at 43.190. Native log contains '[Gesture] Disabled, stale or mismatched simulation snapshot' followed by controller stop before recovery. Therefore this is not evidence that gesture amplitude alone caused failure; reference freshness/control timing must be diagnosed first. Existing combined error does not distinguish wall-age expiry, tick mismatch or configuration. Added rejection-time tick/age/execution/config diagnostics and Supervisor capture of native gesture failures, with split-output and receiver-failure regressions. These source changes are not yet rebuilt into live image.

Automatic recovery created session 7a4078a73c874d0aa870cd607c0683de and returned INTERACTIVE. No reapproval or walking composition attempted. Same-session continuity and safety/quality acceptance FAILED for this trial. No gains, thresholds or checkpoint were relaxed. New 200-Hz trace plus cumulative timing are loaded. Audit remains local pending document-transfer authority.

### Candidate 02 packaging verification and controlled deployment

Verified image social-motion/sonic:composition-candidate-20260915-02 exists with ID sha256:735bfd57c4460922a173b64724ffc1123a0f9896ac09d4dadd65d9d783d891a8. In an isolated no-network container with only tests mounted, supervisor imported from installed site-packages; 28 gesture and 23 prepared tests passed. Compose override validates and selects this image, explicit composition opt-in, read-only prepared-wave catalog, and runner contact tracing at 200 Hz. No policy gains/checkpoint changes.

Before controlled deployment, no joystick client was connected, previous session 2a7e69f9fd5b4b23a8f6015616ce957b was unsupported INTERACTIVE (height .78565 m, tilt .03057 rad, max joint speed .06094 rad/s). Stopped old SONIC and recreated runner, then candidate SONIC using --no-build; startup verification is pending. This explicit deployment reset is separate from motion-transition acceptance. No composed motion has been demonstrated yet. This entry remains local and is not authorized for document transfer.

### Opt-in public prepared execution/status branch

_execute now routes commands with prepared_plan_id to _execute_prepared ONLY when SONIC_ENABLE_GESTURE_COMPOSITION=1; legacy full-reference route remains unchanged and default rejects prepared execution. The path validates exact source/ownership and live experiment envelope, keeps the Isaac request INTERACTIVE (no bootstrap/mode-switch request), streams through the persistent sender and writes an independent UUID-named gesture report. DDS EXECUTING occurs after first reference send with execution_stage=reference_streaming; COMPLETED follows sender completion and a final same-session guard. Failure/abort is recorded without falsely reporting completion; outer existing Supervisor failure/abort handling remains responsible for terminal DDS status.

Reports capture command/plan/session identity, validation, before/after status, first origin tick/execution ID and approve_to_first_reference_s. That latency is NOT detected physical action onset, and COMPLETED does not mean dynamic qualification. Three added report/routing tests plus existing prepared group (23 total) and gesture group (28) passed using mounted source in an isolated container. Live controller unchanged and candidate image 01 predates this branch; rebuild/controlled deployment remain next. Catalog is preparation only, not an approval. Audit local pending document-transfer authorization.

### Startup catalog and true read-only prepared validation

Supervisor startup can read SONIC_PREPARED_PLAN_CATALOG: strict schema, <=64 KiB/16 recipes, explicit transformation settings and expected plan hash. Load occurs before DDS/runtime startup; failure restores the previous cache. Status lists prepared IDs and explicitly marks approved=false/dynamic_qualification=false. deploy/sonic-prepared-wave.json selects the 17.96-second hold-trimmed candidate. Live reload while a child/execution exists is rejected.

Real read-only artifact smoke initially failed because validate_artifact writes validation.json. Added keyword write_result=True (legacy default preserved); preparation/approval-preflight use False. Repeating the smoke with the ENTIRE exchange mounted read-only succeeded and returned expected plan 887e06bae6c25cc87ce8bb2e284a98ec158090deb656da8d6fa9f6452b6c2709. All 20 prepared tests passed. This finds/fixes a real hash stability issue not caught by mocked validator tests. No catalog deployed to live supervisor, no derived action approved or executed; candidate image 01 requires rebuild for these sources. Audit remains local pending transfer authorization.

### Low-speed composition experiment envelope

Added gesture_safety.require_composition_envelope and called it from every Supervisor gesture-session check: height .70–.90 m, tilt 0–.25 rad, |vertical speed| <=.15 m/s (runner unsupported-ground bounds), planar speed <=.5 m/s (initial .45 m/s slow-walk command envelope with tracking margin), torque-limit ratio 0–1 and status age 0–1.5 s. Missing/nonfinite/boolean scalars reject. This is an experimental admission/ongoing guard, not a learned feasibility guarantee or a substitute for native high-rate safety; current status publication is only 1 Hz.

All 28 gesture tests passed using source mounted in the candidate test image. Read-only current INTERACTIVE state in session 2a7e69f9fd5b4b23a8f6015616ce957b passed these bounds. No streaming gesture issued and public prepared execution remains disabled. Candidate image 01 predates this source update and must be rebuilt before deployment. Audit remains local pending transfer authorization.

### Wave phase layout and hold-trimmed candidate

Verified manifest metadata against _add_neutral_transitions implementation: original indices [0,50) initial hold, [50,150) entry transition, [150,399) 249-frame main motion, [399,499) exit transition, [499,549) final hold. At time_scale 2, full reference duration is 21.92 s. New describe_motion_phases.py validates known cubic padding and total frame count; two tests passed.

Candidate retains every transition and main sample, starts at frame 50 (2.0 s) and includes first final-hold sample frame 499 (19.96 s), duration 17.96 s, amplitude .25, one-second composition entry/exit envelopes. Plan ID 887e06bae6c25cc87ce8bb2e284a98ec158090deb656da8d6fa9f6452b6c2709. Real artifact preparation confirmed identical first/last right-arm positions; initial source dq max .0005223 rad/s and final dq zero. Composer entry weight/derivative zero handles the nonzero source endpoint derivative. Artifact itself and its bootstrap requirements remain unchanged; this derived plan is NOT approved or dynamically qualified. Removed hold time is not measured approve-latency improvement. Existing source is Kimodo-generated, not video; eventual video-source switching coverage still required. Audit remains local pending transfer authorization.

### Packaged experimental SONIC image

Built social-motion/sonic:composition-candidate-20260915-01 using deploy/docker/sonic-composition.Dockerfile and its narrow ignore file, after verifying the unchanged baseline image ID. Offline build reused existing dependencies, installed current Python sources with --no-deps --no-build-isolation and rebuilt the native target. Image inspect ID sha256:b5893570bb806be28e9b639509fec748276d960fb74c66a07f989718f4b86a40; native binary SHA256 209db062cb7b1b680b7eab57fbc487213e902680e2811384b3b43c5519eb2ddf. Experimental label explicitly says composition-not-qualified; source is uncommitted work and inherited revision labels are not its provenance.

In a no-network/no-GPU test container with ONLY test files mounted, supervisor imported from /usr/local/lib/python3.10/dist-packages. All 25 gesture and 18 prepared tests passed. Live sonic-tracker still uses baseline sha256:817e8c9c356f20e3117721c0f9009c50372f27df57a5e217c5e7d7e459158abf. Candidate has not been deployed, and public prepared-plan execution is still disabled. This verifies packaging/compilation/unit behavior, not policy dynamics or completion of the target scenario. Local audit transfer still pending permission.

### Cumulative command timing instrumentation

Added CommandTiming at the physics provider's valid-command consumption point and per-step command-age observation. Counters persist beyond the existing 4,096 timestamp / 20,000 age sample deques: accepted count, age-observation count, maxima and counts exceeding .1/.25/.5/1 s. Trace and performance reports expose immutable snapshots of these counters. They measure observed command consumption using receive timestamps, not every raw DDS callback; threshold counters are diagnostics, not new control cutoffs. Compare start/end snapshots within a session to detect inter-trace anomalies without confusing bootstrap history with a motion window.

Three pure tests passed, including a .6-second gap surviving 5,000 subsequent fast samples, no pre-session interval and invalid timing rejection. Provider/runner py_compile and diff check passed; tests saved in simulator tests/. Source synchronized, but current live process has not loaded the instrumentation. No restart or control parameter change this turn. Audit remains local pending document-transfer permission.

### Instrumented stop baseline and trace analysis

Repeated stop_transient_trial.py --walk-seconds 3 --forward 0.5 with contact tracing enabled and unchanged 5-Hz trace. Completed report .build/stop-transients-20260915T153703.json; same session 2a7e69f9fd5b4b23a8f6015616ce957b throughout and remained unsupported INTERACTIVE afterwards. Added analyze_contact_trial.py plus three unit tests for force gating, invalid measurements and incomplete/mismatched trials.

Release/center windows contained 64/61 trace samples at .2 s simulation intervals. Sampled LowCmd age maxima .02432/.02407 s, root tilt maxima .06153/.03862 rad. With analysis-only vertical net-force threshold 20 N, left/right contact-link-origin speed p95 was .00779/.00258 m/s for release and .00923/.00438 m/s for center. Windows begin at the last before-stop reported simulation time, not the exact stop command tick. These are raw contact-link diagnostics, not sole-slip estimates, high-frequency oscillation measurements or proof no inter-sample LowCmd gap occurred. Different results between single baseline pairs underscore need for repeated matched comparisons. No composition activated. Audit remains local pending transfer authorization.

### Foot diagnostics successfully loaded in Isaac

Added default-off ISAAC_RUNNER_FOOT_CONTACT_TRACE passthrough in Compose and runner shell (strict 0/1). Shell syntax, Compose config and diff checks passed. Verified no joystick client and fresh INTERACTIVE before controlled deployment: stopped sonic-tracker, recreated only isaac-runner with ISAAC_RUNNER_FOOT_CONTACT_TRACE=1, then restarted the existing SONIC container after new runner READY. This deployment intentionally created session 2a7e69f9fd5b4b23a8f6015616ce957b; it is not evidence of same-session motion transitions.

Runner initialized the two ankle sensors, passed bootstrap and returned to INTERACTIVE with zero elastic support. Trace isaac_g129_deployment_v1_20260915T193508Z.jsonl contains two correctly named 16-scalar foot rows; one observed sample had vertical forces ~181.49/160.45 N and LowCmd age .0141 s. Later status tilt .02218 rad. Reported release realtime factor .5306 is a startup metric, not a controlled estimate of instrumentation overhead. Live contact tracing is now on (5-Hz trace unchanged); composition is still off. Follow-up requires matched measurement settings for baseline/composition, higher-frequency oscillation capture and proper sole/contact geometry interpretation. Audit not transferred pending permission.

### Opt-in foot contact/link diagnostics source

Confirmed pinned IsaacLab APIs in the running image expose ContactSensor.body_names/net_forces_w and articulation body_link_pos_w, body_link_quat_w, body_link_lin_vel_w, body_link_ang_vel_w. The runner had deliberately disabled all contact sensing for performance, not physical collisions. Added --foot-contact-trace (default off) to instantiate only ankle_roll_link sensors with history one and no air-time tracking. Sensor and articulation indices resolve separately by left/right names; unavailable names fail startup. Trace records world link pose/twist and measured net contact force in two 16-scalar rows, plus LowCmd age; report records whether diagnostics are enabled.

Source synchronized to unitree_sim_isaaclab. Python compilation and git diff --check passed. Not yet restarted/loaded into Isaac, so runtime link mapping, sensor output, overhead and contact-point/slip interpretation remain UNVERIFIED. Net foot force is not ground-only force; moving link origin is not automatically sole slip. Baseline and composition trials must use the same instrumentation settings after overhead qualification. Audit remains local pending transfer authorization.

### Fresh simulation stop baseline, unchanged controller parameters

Verified live SONIC domain 42, lo and rt/socialnav_sim/g1 endpoints; joystick listener 127.0.0.1:16042 had no established client. Ran existing stop_transient_trial.py --walk-seconds 3 --forward 0.5. Report: motion_pipeline/.build/stop-transients-20260915T152730.json, completed true. Both release and stick-center trials remained in session 251cc83b314243319f0791814ad6b7fe; post-test status remained INTERACTIVE with zero support. No composition enabled and no controller parameters changed.

Low-frequency unique status samples (not high-frequency maxima): release 0–2 s max joint velocity 3.065 rad/s, 2–5 s .295; center 0–2 s 2.932, 2–5 s .282. In 10–20 s windows, planar speed maxima were .03048 m/s release vs .004386 m/s center; root tilt maxima .04437 vs .04648 rad. Single trial pair is baseline evidence, not statistical equivalence or proof of LowCmd continuity. The current trace includes root state, joint state/targets and torques but no world foot velocity/contact data; do not label root drift as measured foot slip. High-frequency window analysis and foot-contact/slip instrumentation remain necessary. This audit stays local pending transfer permission.

### Supervisor internal streaming orchestration (not publicly activated)

Added _require_gesture_session and _stream_prepared_gesture. Internal execution resolves the approved immutable plan and full artifact gate, requires a live persistent planner/controller and matching INTERACTIVE Isaac session, zero elastic support, and completed handoff. It reuses a session-owned sender so execution IDs persist across gestures, drains PTY safety output, rechecks state per iteration, bounds completion time and retains existing stop-on-abort/failure behavior after a grant may have been sent. _stop clears sender along with its channel. No native mode switch is sent.

Five mock-based orchestration tests passed (two executions without stop/switch, unsupported or wrong state before grant, session change after grant, transport failure, abort before/during streaming); all 18 prepared-related regressions passed. These tests do not run policy or physics. Public prepared-plan _execute still rejects requests: dynamic entry/ongoing quality gates, status/report integration, preparation request exposure and scenario locomotion deceleration remain required before activation. Live services not restarted. Audit remains local pending explicit document-transfer permission.

### Producer sampling optimization and same-configuration comparison

PreparedGesture now shares one Hermite basis per time sample and exposes right-arm-only sampling; snapshot construction no longer calculates/discards 22 untransmitted joints or constructs a full JointReference for every horizon sample. A seeded independent original-formula test covers 1,004 times × seven joints for q/dq within 1e-12, including outside-clip holds. Ten existing prepared/snapshot/sender tests also pass.

Same --network none --cpus 2 image/artifact and two complete 21.92-second executions: 2,192 updates, update median 4.986 ms, p95 5.693 ms, max 7.847 ms, zero updates over 20 ms. Prior run was median 24.104 ms, p95 27.817 ms, max 30.852 ms with 1,635/1,842 over budget. Native maximum snapshot age decreased from 13 to nine ticks (45 ms). This supports meeting the sender timing target in this isolated run, not real-time guarantees under Isaac/GPU load. Same-session physical composition, dynamic safety gates and Supervisor sender orchestration still remain. No live activation; audit kept local pending document-transfer permission.

### Sustained real-artifact IPC exposed endpoint bug and producer cost

Added gesture_stream_probe.cpp and gesture_stream_smoke.py. Native probe supplies a synthetic 200-Hz wall clock and consumes snapshots every 20 ms over the actual private receiver protocol; it has no policy, DDS or simulator. First sustained runs failed after hundreds of updates with native invalid gesture envelope. Floating-point endpoint rounding made weight exactly one while derivative remained tiny/nonzero. Python/native quintic helpers now use the symmetric tail, and representable zero/one envelopes have zero derivative; runtime product envelopes receive the same endpoint normalization. Strict native validity checks remain unchanged. Two new endpoint tests plus ten existing composer/runtime tests passed.

After correction, two complete 21.92-second gestures released and reused one connection successfully: 1,842 sender updates, 2,188 synthetic consumption samples, maximum snapshot age 13 physics ticks (65 ms). Sender update median 24.104 ms, p95 27.817 ms, max 30.852 ms; 1,635 updates exceeded 20 ms. Thus transport continuity passed this isolated run, but the Python producer DOES NOT meet the 50-Hz budget in this two-CPU container. Next optimize reference sampling/serialization with parity tests before live activation, rather than widening timeouts. This is not policy/dynamics acceptance. Audit synchronization still pending explicit document-transfer approval.

### Readiness heartbeat fail-closed correction

Current live status confirmed deployment task/profile Isaac-Flat-G129-Deployment-V1 / g1_deployment_v1, while Compose reference contract remains sonic_official_g1. Runtime lifecycle already admits INTERACTIVE; readiness itself is not restricted to standing. Found shared readiness accepted NaN, infinity/future timestamps and malformed session identities. Added finite clock/age-limit validation, missing/boolean heartbeat rejection, rejection of future timestamps and nonblank string session requirement. Existing five-second age bound and allowed lifecycle states unchanged.

Regression reproduced nine failing subcases against old source; after source fix all three test groups passed on the lab host. No live restart or simulation control command. This validates readiness input handling, not dynamic gesture safety. This local audit update has NOT been synchronized because explicit document-transfer approval remains pending.

### Supervisor preparation and approval-preflight interfaces

Supervisor now owns PreparedPlanStore and exposes prepare_gesture_plan plus _resolve_prepared_approval. Preparation retains full validate_artifact, discards entries when validation fails, and checks source identity after validation. Approval preflight resolves exact ownership/plan/source identity, runs full artifact validation again, and verifies identity afterwards. Neither method sends references, changes the runtime, nor creates execution approval. The existing _execute prepared-plan rejection remains until execution-time simulation checks and sender orchestration are connected.

All 18 test_prepared*.py unittest cases passed in the isolated deployment image, including three new Supervisor orchestration tests (validation on both paths, invalid preparation removed, source changed during execution validation rejected). Source and tests synchronized successfully after one automatic-review timeout and the permitted retry. Live containers were not restarted. Dynamic composition, physical quality and approve-to-motion latency remain unverified.

### Prepared-plan cache and exact-source lookup

Added PreparedPlanStore with bounded capacity (default 16; no silent eviction), immutable entries, direct-child artifact resolution, source request/motion/contract binding and exact plan-ID lookup. Preparation hashes manifest, validation and joint position/velocity bytes before and after parsing to detect concurrent ordinary changes. Lookup rehashes these inputs without re-parsing CSVs and rejects stale/unknown/mismatched approvals. Explicit discard invalidates cached lookup. This is not independent authorization, full artifact revalidation or a live safety check; the existing full execution validator must remain in the eventual Supervisor integration.

Six unit tests pass in the isolated deployment image: reuse without parser calls, wrong ownership/unknown/legacy approval rejection, stale source, source mutation during preparation, capacity/discard and path escape. Read-only smoke using wave-official-e2e-005-083aa901 at amplitude .25/time_scale 2 produced plan d967b5ee4cb1325d9763ee2bc935327d6db7b21122dc21054840a7cc84eb192d, duration 21.92 s, returning the same immutable plan object. One observed preparation was 0.020367 s and lookup 0.000552 s. These are single cache-operation timings, NOT approve-to-motion latency or simulation qualification. No DDS commands, container restart or live activation. Store is not yet connected to Supervisor preparation/execution orchestration.

### Explicit prepared-plan approval contract (execution still disabled)

ControlCommand accepts an optional lowercase 64-hex prepared_plan_id, only for approve_execute. Legacy commands omit the field on serialization and retain the exact previous schema-1 wire keys. CLI adds --prepared-plan-id. Old supervisors reject the unknown field rather than silently accepting a new meaning; the updated supervisor also explicitly rejects prepared-plan execution before ISAAC readiness checks, artifact writes or runtime mutation until the prepared execution branch is implemented. Original whole-body approval remains unchanged.

Tests cover legacy serialization, exact plan round trip, malformed IDs/actions, CLI serialization with a mocked DDS transport, and fail-closed rejection without executing the original motion. This supplies an approval binding field, not authentication or a completed prepared-motion route. Remaining work: immutable prepared plan lookup/cache, source freshness and ownership verification, execution-time simulation safety gates, sender integration and live acceptance.

### Control-consumption acknowledgement before release

The private clock reply now echoes nonce (8 bytes), simulation tick (4 bytes), and consumed execution ID (8 bytes), all integer fields little-endian. Persistent snapshots carry an internally tagged execution ID. After successful policy-command creation, the controller may acknowledge a fresh snapshot only when all 256 envelope weights and derivatives are zero. Sender keeps supplying snapshots until this execution is acknowledged; native session independently rejects release without the matching acknowledgement and a fresh zero envelope. This confirms control-loop processing, NOT physical settling or successful actuator publication.

Verification on the lab host: sender unittest 4/4; receiver/session GoogleTest 8/8 under C++20 -O3 -ffast-math in an isolated --network none container. Cases include delayed acknowledgement, cancellation, previous-execution acknowledgements, ten sequential grants, and rejecting active/stale snapshots as exit acknowledgements. Full g1_deploy_onnx_ref target compiled and linked successfully in a disposable container. Source/tests synchronized to the single motion_pipeline repository; no live container restart or new simulation qualification. Approval-bound prepared-plan execution integration and physical acceptance remain outstanding.

### Fresh clock handshake supersedes unsolicited ticks

Removed unsolicited tick feedback. Sender now sends an 8-byte nonce and accepts only a matching 12-byte reply (nonce + current uint32 tick); stale queued responses cannot refresh clock state. Native SetTick only updates an atomic; worker replies on demand. Sender uses a bounded 100 ms response deadline, rejects regressing tick and a clock not advancing for 200 ms. Four Python sender tests pass, including injected old reply; five native receiver tests pass under production flags. Native handshake processing is off the control thread.

The release/consumer acknowledgement race remains open and must be fixed before the approved execution branch is enabled. No controller deployment or motion test performed in this step.

### Native-clock-driven sender (not execution-enabled)

Added `gesture_sender.py`: grant/buffer/release sender driven by 4-byte little-endian native tick feedback, bounded nonblocking receive drain, no wall-clock motion extrapolation, session/approved-plan binding, monotonic execution/sequence counters, and cancellation via runtime envelope. Three sender tests pass over Unix sockets. Receiver SetTick sends changed ticks nonblocking; receiver tests now 5/5 pass. No Supervisor execution branch sends gestures yet.

Identified integration hazards to fix BEFORE live activation: (1) immediate release after sending zero-weight snapshot can race the policy reader, which may still hold its previous nonzero snapshot; require a native-consumed completion acknowledgement before release. (2) Clock feedback queues can retain old ticks while producer is inactive; continuously drain or use a fresh clock handshake, not local receipt time alone as evidence of native tick freshness. Current native tick guard fails closed, but that alone would cause avoidable execution failures. Sender tests do not yet exercise these asynchronous races. Build after the receiver change remains pending.

### Opt-in actual launch/receiver wiring

Supervisor now passes actual Isaac session into both _spawn_controller call sites. With SONIC_ENABLE_GESTURE_COMPOSITION=1 it requires persistent planner/session/domain42/loopback and uses the private descriptor launcher; _stop closes the channel. Disabled path still uses ordinary pexpect and removes stale SONIC_GESTURE_FD/SESSION_ID. Three mocked branch tests pass; three existing Supervisor output/abort regressions pass via unittest (pytest absent in image).

Native now instantiates GestureReceiver once during initialization only under explicit feature flag, requiring domain42, sim tick sync/delta4, fixed rt/socialnav_sim/g1 lowstate/lowcmd topics, session and valid inherited fd. It closes original fd after receiver duplicates it. Each control tick updates receiver tick and reads its immutable snapshot; errors route to stop. Full integrated native executable compiled/linked successfully in --network none container with actual changed source/includes. No running image replaced, no live restart or gesture sent.

Remaining immediate task: Supervisor's approved gesture execution branch and continuously timed packet sender with scoped grant/release. Private channel exists only on next opt-in launch; preparation and receiver wiring alone do not execute an approved gesture. Full physical and failure acceptance remains open.

### Persistent execution grants

Added `gesture_session.hpp`: private channel grant/snapshot/release envelopes bind fixed session and monotonically increasing execution ID. Grant only while no execution active; snapshot must match active execution and decoder plan; release requires fresh buffer with all remaining weights/derivatives zero. Decoder sequence resets only for a new execution ID, preventing old same-plan snapshot reuse. Three state-machine tests pass, including ten executions on one session and overlap/incomplete/stale/replayed/cross-session rejection.

GestureReceiver now supports persistent mode in addition to the fixed-plan smoke-test constructor. Four receiver tests pass; new integration test performs three grant/snapshot/release cycles through the SAME Unix seqpacket socket without reconnecting. This is local protocol validation, not SONIC motion execution. Supervisor has not yet issued real grants and native controller does not instantiate receiver yet; live control and physical acceptance remain pending.

### Actual-artifact cross-process IPC smoke

Added `tools/analysis/gesture_receiver_probe.cpp` and `tools/acceptance/gesture_ipc_smoke.py`. In --network none with real wave artifact mounted read-only, compiled probe with -O3 -ffast-math and launched it through the new PTY descriptor helper. Python prepared/sampled the real wave, sent a 77,356-byte packet through the private socket, and compared native-decoded output exactly: 256 frames / 4,096 scalars all match; probe exited 0. Content ID 860c474044f353392974478231508aa7212a2b6e7b3fa849559abbc263f97aeb; plan ID d967b5ee4cb1325d9763ee2bc935327d6db7b21122dc21054840a7cc84eb192d.

Observed smoke exchange wall time .12935 s includes PTY EOF/child close and is NOT a streaming latency benchmark. No DDS or simulator commands were published. Reproduce by compiling probe against vendor include, then `python3 tools/acceptance/gesture_ipc_smoke.py /path/to/probe /path/to/wave-artifact`. Next: persistent multi-execution grant/release lifecycle and continuous sender, not controller-per-gesture restart.

### Supervisor PTY descriptor handoff

Added `gesture_transport.py`: DescriptorSpawn preserves pexpect behavior while forwarding an explicit pass_fds tuple to PtyProcess; spawn_with_gesture_channel creates a private Unix seqpacket pair, inherits only the child endpoint, supplies SONIC_GESTURE_FD, closes the parent's child-end copy, and returns a nonblocking Supervisor endpoint. Failure cleans up both endpoints. It neither grants authority nor enables the experimental controller feature.

Three tests pass in the deployed image with --network none: actual PTY child exchanges a packet while expect/logfile remain functional; an intentionally inheritable unrelated descriptor is closed in the child; failed spawn has unchanged /proc/self/fd inventory. Existing Supervisor launch has not yet been replaced. Next work remains persistent scoped grants and integrated producer/native receiver use, then rebuild and simulation acceptance.

### Local IPC receiver

Added `gesture_receiver.hpp` and three socketpair tests. Receiver takes an already-connected same-UID AF_UNIX SOCK_SEQPACKET descriptor, duplicates it CLOEXEC, parses bounded messages on a worker thread, and publishes immutable snapshots under a short mutex. Replay/malformed packet/disconnect errors latch and must route to explicit safe recovery. Destructor joins within bounded poll intervals. No listener/network port is opened. Three tests pass using production compiler flags inside --network none: delivery+replay, disconnect, invalid descriptor.

This receiver is not yet instantiated by the running native controller. Same UID is not independent execution authorization: caller must pass a Supervisor-owned private channel and validated grant. Current decoder is per-plan; persistent multi-plan grant lifecycle still needs implementation rather than restarting the controller per gesture. Verified ptyprocess.PtyProcess.spawn supports pass_fds; current Supervisor uses pexpect.spawn, so the wrapper's descriptor plumbing needs a focused test before modifying process launch. No image or live session changed.

### Native packet decoder

Added `gesture_decoder.hpp` and `test_gesture_decoder.cpp`: max packet size 256 KiB, JSON depth bound, exact schema/layout/joint map, fixed 200-Hz interval, session and 64-character approved-plan binding, integer sequence/tick validation, replay/tick regression rejection, complete finite q/dq/envelope validation, and atomic anti-replay update only after success. Three test groups pass under -O3 -ffast-math, including nine malformed/mismatched cases, replay, excessive size/depth, and recovery after a rejected packet. Decoder is single receiver-thread owned and returns immutable snapshots.

Important: matching IDs do not authenticate a sender. No transport or Supervisor grant path has been enabled. Next must establish a local scoped receiver with Supervisor-owned authorization, then publish immutable snapshots without parsing JSON inside the policy critical section. Full controller rebuild and dynamic acceptance remain pending.

### Asynchronous tick buffer (supersedes exact-origin hook)

Changed snapshot storage from 46 policy-step samples to 256 physics-tick samples at 200 Hz. Native indexes `(current_tick-origin_tick)+4*future_policy_frame`; a buffer up to 40 simulation ticks old still covers the 180-tick observation horizon. Receipt age remains bounded to 200 ms; future/reset ticks are rejected. This corrects the earlier exact-origin design, which would require unrealistic synchronous producer delivery. Native kernel tests now 5/5 pass with production fast-math flags; updated full controller build remains to be repeated.

Added `gesture_snapshot.py` to generate identity-bearing packets containing plan ID, session, sequence, origin tick, joint map, and 256 paired arm q/dq/envelope samples. Preparation does not advance the runtime execution clock. Two producer tests pass. No transport or approval receiver is active yet. Next: bounded decoder with session/plan/sequence checks, then a simulation-only receiver preserving gamepad ownership.

### Native observation hook integrated (not activated)

Added InputInterface::GetGestureSnapshot (default null), manager forwarding, and a 46-frame immutable ArmSample snapshot contract. Native GatherInputInterfaceData copies once per control tick; position and velocity readers apply the same snapshot against the locked planner reference, including subset readers. Requires experimental SONIC_ENABLE_GESTURE_COMPOSITION=1, simulation tick synchronization at four ticks/control, exact origin tick, receipt age within 200 ms, planner_motion identity, playing state, encoder mode 0, and no conflicting upper-body input. Missing snapshot before all remaining weights/derivatives are zero stops control rather than jumping back. Default null path preserves existing behavior.

Full integrated g1_deploy_onnx_ref compiled/linked successfully in an isolated image with the four changed source/header mounts. Manager snapshot delegation tests pass 3/3. Patch saved to vendor worktree; running image not replaced. Actual receiver/session/approval verification, producer, fault-injection acceptance, and dynamic testing remain incomplete. The 200 ms guard is an initial implementation bound, not a validated operating threshold. No source currently emits a snapshot, so this is a compiled consumer hook, not an operational joystick+wave feature.

### Cross-language comparison and native build

Added `tools/analysis/gesture_composer_probe.cpp` and `compare_gesture_composer.py`. Compile the probe with `-std=c++20 -O3 -ffast-math` and the vendor include directory; invoke `python3 tools/analysis/compare_gesture_composer.py /path/to/probe`. Seed 20260915, 1000 cases, entry/exit weights and varied planner/gesture references: maximum q/dq discrepancy 1.7763568394002505e-15 against tolerance 1e-12. This is arithmetic parity, not dynamics.

Full native executable target `g1_deploy_onnx_ref` compiled and linked successfully in an isolated `--network none --cpus 2` container based on the existing image, with the modified InterfaceManager header mounted read-only. No running binary was replaced. The new composition kernel is independently compiled/tested but is not yet hooked into that executable, so this build does not establish integrated composition functionality.

Verified native snapshot boundary: `GatherInputInterfaceData()` precedes the `current_motion_mutex_` critical section; `GatherObservations()` executes inside that section. The gesture snapshot should be selected once per control tick, then position and velocity horizon assembly must use that same snapshot against the locked planner data. Do not allow independent asynchronous q/dq updates inside the two observation readers.

### Native reference composition kernel

Added vendor `include/gesture_composer.hpp` and `unit_tests/test_gesture_composer.cpp`. Kernel applies seven right-arm targets plus envelope/derivative to each actual planner frame, preserving all other joints, without altering the planner input arrays. Four isolated native tests cover ownership, distinct future samples, envelope derivative, invalid data and arithmetic overflow. No controller hook or producer is enabled yet.

Important deployed-build finding: `-O3 -ffast-math` removes ordinary std::isfinite validation; the first production-flag regression failed to reject NaN and overflow while ordinary -O2 passed. Replaced finite checks in this kernel with IEEE-754 exponent-bit checks (C++20 bit_cast and platform static assertion). Production-flag rerun passes 4/4. This fix is local to the new kernel; no claim is made that all existing vendor checks are now audited. Paired Python/native corpus comparison and full native executable build remain pending.

### Rolling reference window

`GestureRuntime.sample_window` now composes each supplied future planner sample at its own simulation timestamp. Only the window origin advances the execution clock; future preview does not consume time. Invalid windows fail without changing the clock. Two additional regressions verify the installed SONIC-style ten-frame, 0.1 s spacing horizon followed by a 0.02 s next tick/cancel, and reject empty/duplicate/regressing windows. Runtime tests now 5/5 pass.

Pinned native source registers `motion_joint_positions_10frame_step5` and matching velocity observation as 290 scalars (10 x 29) at cpp lines 1748–1749. Native planner reference resides in `planner_motion_` and is protected by `current_motion_mutex_`; output captures the current motion/frame, not an existing validated external base-window RPC. Consequently do not round-trip LowState as a substitute for planner future reference. The native integration should apply gesture samples to the actual locked planner window, retaining root/contact data and the existing gamepad loop. The Python kernel is the test oracle/preparation layer, not evidence that this native integration is complete.

### Prepared gesture and reference lifecycle

Added `prepared_gesture.py`: reads exact source manifest/validation/joint CSV bytes into immutable tuples, computes content identity including amplitude/time scale, validates dimensions/order/finite data and endpoint holds, and samples cubic Hermite positions with analytic velocities. It does not inherit approval.json. Four preparation tests pass. Actual wave at amplitude .25 and time_scale 2 has duration 21.92 s and sampled peak right-arm speed .982124 rad/s at 200 Hz; finite-difference derivative residual approximately 3.04e-6 rad/s. This is a conservative offline candidate, not a natural-speed final gesture or dynamic qualification.

Added `gesture_runtime.py`: connects prepared sampling and right-arm composition. An immutable plan explicitly chooses segment offsets and entry/exit envelopes; its identity includes all these choices. Runtime requires the matching approved plan identifier and fixed session, rejects regressing simulation clocks, and returns the live planner reference after completion/cancellation. Cancellation multiplies the normal envelope by a quintic fade and includes the full product derivative, preserving q/dq at cancellation even during entry. Repeated cancellation cannot restart the fade. Three lifecycle tests plus four preparation and five composition tests pass remotely (12 total).

The approved_plan_id parameter is only an identity check, NOT an authorization mechanism. Supervisor must validate DDS authorization and execute-time state before constructing the runtime. No inbound endpoint or native consumer is enabled yet. Input freshness, bounded interruption handling, feasibility and tracking gates are still required; exceptions must route to the existing safe path, not be silently converted into a new execution.

Next integration task: native per-frame reference window consumption while preserving joystick/deadman ownership; do not replace the current manager with zmq_manager to bypass this requirement. Segment choices need inspection of the actual wave and derived-motion qualification rather than automatic cropping of the original support/settling contract.

### Offline composition primitive and wave inspection

Added `services/sonic_tracker/sonic_tracker/composer.py` and `tests/unit/test_composer.py`. Five tests pass locally and remotely. The reference-only kernel uses a quintic envelope and the full position derivative, including `weight_rate * (gesture_q - base_q)`. Numerical derivative checks cover both entry and exit with moving base and gesture. It preserves every non-right-arm joint exactly, rejects mismatched sample times/order and nonfinite values, and has no transport or actuator writes. It is NOT wired into the running policy; feasibility, authorization, freshness, contact checks, and frame-window integration remain pending.

Wave source manifest: 549 frames at 50 Hz, 249 conditioned source frames, front/back conditioning specified as 1 s neutral hold and 2 s neutral transition per end. Existing execution contract explicitly requires bootstrap support and frame-zero settling. Do not silently reinterpret its existing approval as authorization/qualification for a derived walking composition. Preserve original artifact and create a separately traceable prepared derivative.

Measured right-arm maximum absolute joint velocities (IsaacLab indices): 12=3.384, 16=1.066, 20=4.851, 22=3.089, 24=7.314, 26=1.856, 28=4.301 rad/s. Joint 12 ranges -1.901 to 0.200 rad. This is not yet a qualified small-amplitude walking gesture. Next: prepare a bounded amplitude/time-scaled derivative, retain source hash and explicit segment boundaries, check velocities/accelerations and enter/exit continuity before streaming. Numerical pass is not a balance or foot-slip pass.

Reproduce kernel tests: `python3 tests/unit/test_composer.py` from pipeline root.

### Reproducible historical latency baseline

Added `tools/analysis/analyze_transition_latency.py` and `tests/unit/test_transition_latency.py` (6 tests pass locally and on host). Analyzer rejects missing/out-of-order phases, changed/missing session, and invalid/regressing timestamps; reports exclusions rather than silently passing partial rounds. Output creation is exclusive to preserve existing reports.

Run from pipeline root:

```sh
python3 tests/unit/test_transition_latency.py
python3 tools/analysis/analyze_transition_latency.py .build/runtime-acceptance-20260913T134714/round-*.log
```

Recorded report: `.build/transition-latency-baseline-20260915.json`. All 10 historical rounds accepted, no exclusions, session 041aa283c3a8455d8be4b1cc032edef7. Dispatch-marker-to-PLAYING median 13.472 s, min 12.643 s, max 14.799 s. Component medians: dispatch-to-hold 1.014 s; hold-to-preempt 4.813 s; preempt-to-settling 5.055 s; settling-to-playing 2.629 s. Sum of component medians is not necessarily median total. These are polled wall-clock observations, not exact DDS-receipt timings, and not new acceptance trials.

Wave candidate artifact is `../motion_exchange/wave-official-e2e-005-083aa901/` (joint_pos.csv/joint_vel.csv plus manifest/validation). Persistent selection entries point to container-absolute `/motion_exchange` paths and must not be mistaken for missing host artifacts. Candidate has not yet been qualified for upper-body-only use.

### Follow-up: native regression and minimal fix

- Original host and baked-image manager headers both had SHA256 54525f5bae95174a40b5de591227954b1a4da4583f28a1aa899b0c8a92a44880 before editing.
- Important distinction: InterfaceManager owns ZMQEndpointInterface (streamed motion), NOT ZMQManager (planner command plus optional upper-body targets). ZMQManager is a separate `--input-type zmq_manager`. Forwarding alone therefore does not create a joystick-plus-gesture producer.
- Added three upper-body forwarding overrides and a test-only friend peer in InterfaceManager. Native `test_upper_body_forwarding.cpp` checks all 17 position/velocity values, live updates, withdrawal, delegate replacement, and null fallback through the real manager class.
- Red/green isolated execution: original-behavior header with only test-peer access fails both tests; fixed header passes 2/2. Both runs use `docker run --rm --network none` with the current image, no hardware devices, and no DDS publisher.
- Changes saved in the vendor worktree; no image rebuild or live restart. Full controller build, input freshness, paired data validity, and simulation composition acceptance remain pending.
- Reproduction scratch on host: `/tmp/sonic-composition-test.5HmPsH`, containing baseline/fixed headers, test, and `run_upper_body_test.sh`. Run current image with this directory mounted read-only at `/audit`, the selected header mounted over `/sonic/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/interface_manager.hpp`, `--network none`, and `--entrypoint bash ... /audit/run_upper_body_test.sh`.

Walking-plus-wave has NOT been implemented or demonstrated. Same-session transitions, latency improvements, quality comparisons (oscillation, stop sway, foot slip), repeated trials, cancellation, incompatible references, and interrupted input all remain to be qualified against the full thread goal.

### 2026-09-15 candidate07 live revalidation (supersedes earlier deployment status)

Observed live image `social-motion/sonic:composition-candidate-20260915-07`; Isaac and SONIC already running, no deployment/test process left active. Did not recreate either container. Session `04820627777b4c9a910831e63b577fe0`.

Re-ran installed-image tests in a separate network-disabled container with host tests mounted read-only: 36 gesture tests and 24 prepared-plan tests passed. An initial attempt inside the live container could not discover `/audit/tests/unit` because that test mount was absent; this was test invocation failure, not a failing assertion.

Executed `python3 tools/acceptance/walking_gesture_trial.py --gait-envelope --forward 0.25`. Report `.build/walking-gesture-973154bad58443feb7b2c2a3a83babb1.json` is **incomplete / failed admission**, not a quality pass. No gesture was approved. Idle, approach and matched walking finished; deceleration rejected current root tilt `0.2310990184 rad` against the experimental `0.20 rad` bound. Window was valid (201 samples, 1 s, max gap approximately 0.005 s); the candidate06 floating-point freshness failure did not recur in this trial.

Trace: `motion_exchange/executions/isaac_g129_deployment_v1_20260915T211422Z.jsonl`. Observed deceleration starts at simulation 1448.65 s; rejection at 1450.385 s. The 200 Hz trace shows tilt increasing from 0.05011 rad in the preceding walking interval to 0.23237 rad during stop/recovery. Over simulation 1447–1454 s, largest LowCmd age was 0.029372 s. Cumulative accepted-interval threshold counts (0.1/0.25/0.5/1 s) remained all zero. Session remained INTERACTIVE and subsequently settled to roughly 0.024 rad tilt. These observations implicate stopping dynamics rather than a LowCmd outage, but do not establish the reference-level cause. Rejection triggers harness deadman release; post-rejection metrics must not be attributed solely to the planned centering maneuver.

Source inspection: default `gamepad.hpp` maps non-dead-zone stick magnitudes to a minimum 0.2 m/s planner request, then selects IDLE at centered stick with runtime speed <=0.201. A smooth stick ramp therefore does not imply a continuous planner-speed ramp to zero. The existing zero-speed experiment remains OFF; prior candidate05 evidence did not qualify it. Need correlate actual planner mode/reference and gait contact phase with stopping transient before selecting a fix. Do not widen admission bounds or repeat trials merely to obtain a favorable stopping phase. Walking composition and full dynamic-quality acceptance remain unproven.

This audit update is local only; no document upload performed.

### Failed-stop analysis and planner seam inspection

Added `tools/analysis/analyze_stop_failure.py` and four regression tests in `tests/unit/test_stop_failure_analysis.py`, locally and in the remote repo. Tests pass in both environments. Analyzer accepts the incomplete deceleration case, rejects changed session/wrong trace/nonfinite data/missing physics samples, and excludes samples at or after the rejecting status timestamp so deadman recovery is not mixed into intended centering. It reports raw LowCmd target steps, actual speeds/errors and foot vertical-force occupancy; none is a planner-reference derivative, oscillation qualification or sole-slip metric.

Reproduction from pipeline root:
`python3 tools/analysis/analyze_stop_failure.py .build/walking-gesture-973154bad58443feb7b2c2a3a83babb1.json ../motion_exchange/executions/isaac_g129_deployment_v1_20260915T211422Z.jsonl`

400 pre-stop samples (1446.65–1448.645 s) versus 347 pre-rejection deceleration samples (1448.65–1450.38 s): tilt max 0.050109 → 0.230791 rad; largest raw target step 0.231799 → 0.214501 rad (not evidence of an increased target jump); right ankle pitch raw target error max 0.529473 → 1.113424 rad (Unitree index 10; may be corrective control, not causal reference evidence). No sample had both feet below 20 N world-vertical net force; single/double counts 212/188 before, 95/252 during centering. Durations differ; these counts do not establish contact stability or slip.

Native source hash reverified against local copy: `205b93e3f0f773ef58b566a4fa60b1416b65e7507be5093a7feea9ac5d1dce56`. `CurrentFrameAdvancement` blends new planner animation over 8 frames at 50 Hz with a linear weight. Joint position and velocity are each interpolated independently; the velocity formula omits the derivative term `w_dot * (q_new-q_old)`. Consequently the supplied velocity is not generally the derivative of the blended position. This is a concrete source-level inconsistency, NOT proof it caused the observed stop sway. `UpdatePlanning` conditions on the current planner motion plus lookahead and produces a new sequence; mode changes request replanning. Need measure actual seam mismatch/consumption timing before qualifying any replacement. No native control change, gain change, admission widening, deployment restart or additional robot motion in this analysis turn.

### Candidate08 diagnostic implementation

Added `planner_seam_diagnostics.hpp` and `test_planner_seam_diagnostics.cpp`; integrated optional measurement into native `CurrentFrameAdvancement` before the unchanged blend assignments. Enabled only with simulation-tick synchronization and `SONIC_PLANNER_SEAM_DIAGNOSTICS=1`, capped at 512 adopted-plan records per process. Records control tick, old/generation frame, blend start, sample validity, maximum old/new position gap and omitted linear-weight derivative magnitude across interior blend samples. No q/dq, root, gain, checkpoint, gate or timing parameter is changed. Output goes through existing child log; diagnostic overhead must still be checked against LowCmd timing during simulation.

Tests cover a constant-trajectory blend with analytically known missing 1 rad/s term, endpoint exclusion, equal trajectories, invalid dt and nonfinite inputs. Pass under local clang and image g++ with C++20/O3/fast-math. Clang constant-folding initially broke a known-NaN diagnostic test; materializing IEEE bits with a volatile integer fixed it without affecting controller behavior. Test integrates with the existing globbed GoogleTest suite; define `SONIC_STANDALONE_SEAM_TEST` only for isolated execution.

Diagnostic image `social-motion/sonic:composition-candidate-20260915-08` built successfully, final manifest-list digest `sha256:43f831dd6465d7ad00d391c97191d5adf4f066241ebb3db32223cd1e4e75489e`; final installed standalone test passes. Final layer uses first candidate08 build (`sha256:6908d089fc6635382f3b0647405cfaa88dfc9ff32df44c1b92cecdfb0463e749`) as base to incorporate test integration correction; first build derives from candidate07. At this entry, live containers still candidate07, deployment precheck in progress. No live seam measurements yet.

### Candidate08 first walking-plus-gesture execution (2026-09-16 UTC)

Prior SSH precheck exited timeout; subsequent read-only check succeeded and showed no connected joystick operator. Deployed candidate08 with `SONIC_PLANNER_SEAM_DIAGNOSTICS=1`, recreating runner/tracker for this new experimental session. Bootstrap completed unsupported INTERACTIVE; session `05bfdf51bcdf448e90a960af15804ed7`. Intermittent SSH read timeouts did not trigger extra restarts. Status was freshly checked before motion.

Baseline `.build/walking-gesture-521f62644a9d49288179c255fed27c33.json` completed all phases. 200 Hz trace: deceleration tilt max 0.183692 rad; final deceleration 0.176693; largest phase LowCmd age 0.030415 s; all interval-threshold counter deltas zero. 91 planner seam records, none invalid. Maximum omitted velocity term: approach 1.08464, steady walking 0.725754, deceleration 2.19918, final deceleration 2.07018 rad/s. These are maximum interior seam discrepancies over future samples, not necessarily the current consumed reference. Supports investigation, not proof of cause.

Then ran `python3 tools/acceptance/walking_gesture_trial.py --gait-envelope --forward 0.25 --with-gesture` in the SAME session. Trial `.build/walking-gesture-2ec25618df1f4d30a772cef35546ec2f.json` completed all phases. Prepared plan `887e06bae6c25cc87ce8bb2e284a98ec158090deb656da8d6fa9f6452b6c2709` report COMPLETED, execution_id 1, origin tick 90600, duration 17.96 sim seconds, 1626 reference updates over 32.65344 wall seconds. Approve-to-first-reference 0.0934621 wall seconds (NOT physical onset latency). This source is the existing Kimodo wave, not a video-derived motion.

Actual right-arm joint excursions while walking (Unitree 22–28): [0.44137, 0.13969, 0.44125, 0.34690, 0.38227, 0.22007, 0.26778] rad. Walking phase root displacement approximately [2.42719, 0.05920] m. Native completed acknowledgement plus actual moving robot/arm supports first simultaneous-composition execution, not full naturalness qualification.

Same-session matched phase comparison baseline → gesture: walking tilt 0.08303 → 0.06410 rad; deceleration tilt 0.18369 → 0.15338; final deceleration 0.17669 → 0.17403. Walking maximum joint speed 5.2883 → 5.7020 rad/s (not an oscillation metric), deceleration torque-limit ratio 0.51655 → 0.62346. Gesture maximum phase LowCmd age 0.030037 s; all interval-threshold deltas zero. No session change/timeout/SAFE_STOP observed; post-trial INTERACTIVE same session.

IMPORTANT quality exception: left contact-link origin planar speed p95 during deceleration increased 0.16071 → 1.07595 m/s while right decreased 0.13239 → 0.04734. This is NOT yet a sole slip measurement; investigate contact geometry, angular motion and force before repeating or claiming no degradation. The full goal is unachieved: naturalness/oscillation/sole-slip comparison, repeatability, graceful cancellation, incompatible-reference/input-loss safety and video-derived source still need evidence. No threshold widening or control-law fix was made for this successful execution.

### Contact-motion geometry check (read-only; no additional trials)

Confirmed trace rows use `body_link_pos_w`, `body_link_quat_w`, `body_link_lin_vel_w`, `body_link_ang_vel_w` and independently name-resolved contact sensor `net_forces_w`, not mismatched articulation/sensor indices. G1 profile source asset is `assets/robots/g1-29dof_wholebody_deployment-v1/g1_29dof_with_inspire_rev_1_0.usd`.

Read the USD without starting another SimulationApp. Required existing USD Python path `/isaac-sim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310` and its `bin` library path; BBoxCache must ignore visualization visibility to include hidden colliders (this USD binding accepts positional bool arguments). Stage metres-per-unit is 1.0. Both ankle-roll `collisions` Xforms have collisionEnabled=True and an enclosing radius relative to the ankle link of 0.1344432936 m. Reproducible reader saved locally at `/private/tmp/gear-sonic-deadman-fix/inspect_foot_geometry.py`. Several read-only SSH queries timed out; only terminal queries were retried, and no services were restarted.

Added `tools/analysis/contact_velocity_bound.py` and `tests/unit/test_contact_velocity_bound.py` to the repo; five tests pass locally and remotely. For any rigid point within radius R, planar speed is at least `max(0, norm(v_origin_xy)-norm(omega)*R)`. Used R=0.20 m, deliberately larger than the asset bound. This removes more rotation than necessary, giving a conservative lower bound; it is not an exact contact point or ground-slip classifier. Tests cover translation, rolling, rotation insufficient to explain translation, invalid inputs, gaps and contact segments.

Same-session deceleration baseline → gesture, left foot:

| Vertical force filter | Max point-speed lower bound (m/s) | P95 lower bound (m/s) | Integrated material-point travel lower bound (m) |
|---|---|---|---|
| >=20 N | 0.470872 → 1.044927 | 0 → 0.860059 | 0.007343 → 0.138183 |
| >=50 N | 0.000335 → 0.368187 | 0 → 0 | 0.00000168 → 0.002967 |
| >=100 N | 0.000335 → 0.000834 | 0 → 0 | 0.00000168 → 0.00000417 |

The integration covers adjacent force-qualified samples only, is not net displacement, and must NOT be reported as measured slip distance. At peak time 460.52 s, link planar speed 1.267066 m/s, force z=25.43444 N, angular velocity [0.08094,0.99001,-0.51754] rad/s. Rotation cannot explain the large observed contact-associated motion. The excess is concentrated in light-load contact rather than strongly loaded stance. Could involve dragging/landing/speculative contact; contact-point height and runtime collision/contact settings still need inspection. Increasing the force filter is sensitivity analysis, not permission to ignore the 20 N event or qualify motion. No additional approval, motion, gain, friction or safety-threshold change in this turn.

### Candidate09 and start of authorized Git maintenance

Verified actual running command uses release observation_config.yaml. G1 encoder mode 0 explicitly requires `motion_joint_positions_10frame_step5` and `motion_joint_velocities_10frame_step5`: the q/dq inconsistency is on an active observation path, not unused metadata.

Added opt-in `SONIC_EXPERIMENTAL_C2_PLANNER_BLEND=1`, additionally requiring simulation-tick synchronization. `planner_blend.hpp` computes a quintic weight with zero first/second endpoint weight derivatives and includes `w_dot*(q_new-q_old)` in joint velocity. Existing 8-frame/0.16 s duration retained. Root position/orientation uses the same weight; no motor-command mixing. Legacy branch remains default. Tests cover finite-difference agreement for moving input trajectories, endpoints, identical trajectories and invalid duration; pass with C++20/O3/fast-math locally and in image. This tests analytic blending, not quality of the upstream sampled trajectories or closed-loop balance. Existing seam log's `max_omitted_dq` is still the counterfactual LINEAR diagnostic; `[PlannerBlendProfile] c2_kinematic=1` identifies the applied experiment.

Image `social-motion/sonic:composition-candidate-20260916-09`, image ID `sha256:ef45e71100f1aaba802b4120b49057f9e7dcc076e8b9b6d8f2e85e2ad080d104`, built from candidate08. Native compilation, derivative checks and 36 gesture + 24 prepared tests pass. Deployed with experiment flag enabled after confirming no joystick client. New unsupported INTERACTIVE session `5ca0443a2ed44feeb98b238b0e0ddc2e`, observed tilt 0.0242554 rad. No candidate09 walking trial yet. Candidate09 is NOT qualified; remaining sources were dirty when built.

User explicitly authorized ongoing Git commit management while pursuing the goal. Inspected main/vendor/Isaac statuses and confirmed staging areas initially empty. Created main-repo commit `19f2c5ff26c1aa693ea375d3411a6aa439bdf1af` (`Add read-only stop and contact motion diagnostics`) containing only analyze_stop_failure.py, contact_velocity_bound.py and their two test files; 4+5 tests rerun and pass before commit. Other modifications preserved, no push/history rewrite. This commit is NOT a checkpoint of the full deployed image: native composition, C2 experiment, supervisor and simulator changes still need reviewed, dependency-aware commits and vendor pointer recording. Current audit remains local only.

### Candidate09 repeated baseline: not accepted; rollback initiated

First C2 baseline `.build/walking-gesture-9cb1a1d1fd9e4b7abee3a2c7e6769dce.json` completed in session `5ca0443a2ed44feeb98b238b0e0ddc2e`; trace `isaac_g129_deployment_v1_20260916T041634Z.jsonl`. Log confirms c2_kinematic=1. Deceleration tilt 0.082846 versus candidate08 baseline 0.183692 rad, but deceleration max joint speed 5.9210 versus 4.1035 rad/s, standing-finish tilt 0.06642 versus 0.02870 and post-stop tilt 0.13711 versus 0.07121. Not a uniformly improved result; different gait phases may confound this single comparison. Largest phase LowCmd age 0.029568 s; all interval-counter deltas zero. Left-foot >=20N point-speed lower-bound integral 0.001734 m, right 0.000817 m; these are not exact slip distances.

Predeclared two additional baseline rounds, stopping on first failure. Initial connection attempts failed before execution. After read-only SSH connectivity recovered, round two `.build/walking-gesture-5c70ce8da4744c529230456fce3efb4e.json` failed at deceleration (simulation 907.035 s): 1-second physics window peak planar root speed 0.673224 m/s >0.65 bound, mean path speed 0.313012, peak tilt 0.148625 rad, peak torque ratio 0.702434; complete 201-sample window with 0.005 s max gap. Third round did NOT run. Harness released deadman; observed same-session INTERACTIVE, tilt 0.018484 rad afterward. No gesture trial under candidate09. This does not prove C2 alone caused the event, but prevents accepting it as the stop fix. No threshold raised.

Before round two, vendor repo commit `80b9197276c2353083d442ef61ea819628e7e81c` (`Add tested planner reference blend and seam diagnostics kernels`) was created on existing `social-motion/isaac-integration` branch after rerunning both native standalone checks under Linux/O3/fast-math. Includes only planner_blend.hpp, planner_seam_diagnostics.hpp and their two tests. Earlier commit attempt timed out; HEAD/staging/process checks confirmed no execution before retry. No push, branch change, history rewrite or unrelated staging. Native controller integration remains dirty; main vendor pointer not yet committed as a full runtime snapshot.

Override reverted locally to candidate08 without SONIC_EXPERIMENTAL_C2_PLANNER_BLEND. Source experiment and images retained for reproducibility. Remote rollback deployment command launched (shell handle 28754); completion must be checked before claiming rollback complete or issuing another deployment. Temporary network routing inspected read-only; VPN route/utun7 existed; no network settings changed.

Rollback follow-up: handle 28754 exited 0; runner and tracker recreated/started. Docker inspect confirms live tracker image candidate08. Supervisor logs READY, but INTERACTIVE has not yet been observed after this rollback; verify bootstrap completion before any next motion. No additional test is running.

### Graceful cancellation control draft and regression checks

Follow-up observation confirmed candidate08 session `8799fc15a9594c43aeddf2421b2d4ff0` reached INTERACTIVE. No additional motion or deployment during cancellation unit-test work.

Added explicit `cancel` action with required exact execution token, request ID and motion ID. Tokens are execution bindings, not authentication. The supervisor latches cancellation for a prepared gesture; the existing sender fades the reference and waits for the native zero-envelope consumption ACK before release. Emergency `abort` remains distinct and takes priority. Missing cancellation ACK has a five-second wall deadline and stops the controller; channel failure retains the stop path. Successful cancellation reports ABORTED with termination_mode=reference_fade, preserving the current joystick session rather than requesting a runtime abort. This does not mean joystick locomotion is stopped.

Verified remote pre-edit hashes before uploading protocol.py, cli.py, supervisor.py and test_gesture_cancel_control.py. Isolated candidate08 container, network disabled, repository mounted read-only with source-overlay PYTHONPATH: gesture suite 43 tests passed and prepared suite 24 passed. Added cancellation-timeout test afterward: cancellation suite now 8 tests, all passed. `git diff --check` passed. Unit tests mock sender/session and do not prove live native cancellation or dynamics; candidate08 remains unchanged, cancellation code is not baked/deployed. Integration/race coverage and live cancellation acceptance remain outstanding. No new commit for these changes yet because their prepared-composition dependencies also remain uncommitted.

Next integration step: read current remote gesture_stream_smoke.py and gesture_stream_probe.cpp. Added local `--cancel-after` option to smoke harness, requiring positive finite simulation offset that leaves room before natural completion. Each execution checks that fade time elapsed, exact native execution ACK arrived, and release occurred before natural completion; records repeated execution IDs and elapsed simulation time. Local py_compile passed. This remains a synthetic-clock transport check, not policy/dynamics validation. Upload/test currently pending because subsequent SSH/SCP attempts failed with `Can't assign requested address`; no deployment or simulator commands were sent, and no network settings changed. Native receiver is Linux-specific (SO_PEERCRED/ucred), so macOS compilation would not verify the actual transport. Local patch remains `/private/tmp/gear-sonic-deadman-fix/gesture_stream_smoke.py`; compare remote again before uploading.

Connectivity remained unavailable on the next goal continuation. Added local runtime regression covering cancellation at entry start, mid-entry, hold, and natural exit. Checks exact q/dq equality at cancellation, finite-difference q derivative against dq for all seven right-arm joints throughout fade, unchanged other 22 joints, and return to base reference. All six runtime tests passed using Python standard library and a temporary in-memory package path pointing at the local source copies (no production package installed or modified). New test file change remains local, not committed/uploaded; compare remote before transfer. This is reference mathematics only, not closed-loop stability evidence. Remote cancellation/native tests and simulation quality acceptance remain pending.

### User-requested cleanup after connectivity restored

SSH successfully read the live repository after retrying one permission-review timeout (the timeout was not a network failure). Reviewed remote core source/tests and confirmed staging empty. Synced the additional runtime cancellation regression and readability cleanup of cancellation test setup (split compound statements, named patch configuration, module scope docstring). Ran 28 composition/preparation/runtime/snapshot/endpoint/parity/cancel-control tests in network-disabled candidate08 with read-only source overlay; all passed, and git diff --check passed.

Created main commit `a34d923d6ddd1805e12a1ddcbaf5fc3705e42c6b` (`Add tested reference-only gesture composition core`): four modules composer.py, prepared_gesture.py, gesture_runtime.py, gesture_snapshot.py plus six corresponding kernel test files. Twenty of the 28 passing tests belong to this commit; eight cancellation-control tests depend on still-uncommitted supervisor/protocol work and were deliberately excluded. No push, deployment, robot command, vendor pointer update, or unrelated-file staging. Cancellation test cleanup is on remote but remains uncommitted with its integration dependencies. Native smoke --cancel-after change remains local pending remote comparison/upload. Full integration and quality acceptance remain incomplete.

### Repository-first cleanup: transport and simulator batches

Re-inventoried main, vendor, simulator modified/untracked/staged state before proceeding. Staging areas were empty. Reviewed main transport/safety sources and all four corresponding test files. Main commit `e9be18a` adds only gesture_sender.py, gesture_transport.py, gesture_safety.py and four tests; 16 tests pass in isolated network-disabled candidate08 with read-only source overlay. Commit message explicitly labels experimental admission checks, not dynamic safety qualification or a complete deployment snapshot.

Reviewed both simulator integration diffs plus command_timing.py, kinematic_window.py and their tests. Simulator commit `1ed7a193ed9cb99b67d397edff1f17d2400d53be` adds cumulative command timing, physics-rate window and opt-in foot-contact tracing (six files). Eight unit tests and py_compile of both integration files passed. Existing simulator branch preserved, git status afterward empty. No cache files staged; no simulator restarted. Simulation traces from earlier trials still used then-dirty source, not these newly created commit hashes as sole provenance.

Remaining main groups: protocol/CLI/store/validator/Supervisor and their tests depend on prepared composition; DDS readiness and Isaac lifecycle integration need diff review; deployment override/Dockerfile/catalog/shell/compose should follow vendor and simulator identification; analysis/acceptance tools need reviewed tests; remote audit is an older unsynchronized file and must not be blindly committed as current results. Main vendor pointer remains pending vendor consolidation. All other main changes preserved.

Vendor inventory: four modified integration headers/source, four new gesture headers, six nested native tests, and two additional tests in gear_sonic_deploy/unit_tests. Comparison proves the nested receiver/session tests are older protocol coverage (12-byte clock reply, release without consumption ACK), while the outer tests cover current 20-byte reply and consumed execution. Do not treat them as identical or stage both blindly. Need inspect CMake/test build entrypoint, consolidate with recoverable preservation of old files, rerun Linux native tests, then split default-off control experiments from native transport integration where practical. No uncertain-source files deleted. Git cleanup remains in progress; no function expansion, push, history rewrite, or deployment in this turn.

### Readiness fix and integration regression collection

Reviewed full current CLI/protocol/DDS/Supervisor/lifecycle diff, and prepared approval/cache/execution, launch, orchestration, readiness tests. Independent main commit `3ac159e` contains only lifecycle.py finite/future heartbeat/session checks and test_readiness_heartbeat.py; three tests passed in an isolated source-overlay container before commit.

Important test-runner distinction: some legacy tests are pytest functions, not unittest classes. Initial unittest integration run had three import errors because pytest was missing and did not collect function-only artifact/import-boundary cases; it is NOT a legacy regression pass. Host Python and existing .venv-test and /tmp/motion-persistent-test-venv also lacked pytest. Installed pytest==8.1.1 only into /tmp/test-deps in a disposable --rm candidate08 container, source read-only; production image and host dependencies unchanged. Correct pytest run collected 86 cases: 85 passed, one failed, test_artifact.py::test_convert_and_validate, because the generated artifact had root_only_test_fallback instead of required mujoco_fk. Source inspection confirms FK fallback occurs on missing model file or import; neither dependency has yet been checked due to subsequent SSH failures. Default G1_MJCF points at /workspace/kimodo/kimodo/assets/skeletons/g1skel34/xml/g1.xml, not the repo mount. Do not weaken validator or claim full suite passed. Need locate model/set G1_MJCF and verify MuJoCo in test environment, then rerun.

No integration commit made yet: CLI, protocol, validator write_result option, DDS warmup, prepared_plan_store, supervisor and six integration tests remain pending complete validation/reporting. Artifact/import-boundary legacy files were not modified. Last remote connectivity attempts failed with Can't assign requested address; no mutation command was pending, and no new integration stage was created. Readiness commit completed before connectivity loss. Prior remaining source/deployment/vendor/tool groups remain preserved.

### Prepared integration committed after FK environment correction

Connection recovered. Located existing model at vendor/kimodo/kimodo/assets/skeletons/g1skel34/xml/g1.xml; disposable candidate08 test confirms MuJoCo import was absent. Re-ran same 86 pytest cases with pytest==8.1.1, mujoco==3.3.7, numpy==1.26.4 installed only to /tmp/test-deps in a --rm container and G1_MJCF pointing at the read-only repository model. All 86 passed in 5.54 seconds. No production dependency, source validation or test expectation changed to obtain this pass.

Created main `045876274e62a87d1b11d042a39c4efb09108c16` (`Integrate experimental prepared gesture approval and cancellation`), exactly 12 reviewed files: CLI, DDS transport, protocol, validator, prepared_plan_store, supervisor and six approval/execution/cache/cancel/launch/orchestration test files. Commit message records tested dependency versions and outstanding native cancellation/dynamic qualification. Staging was empty beforehand; diff --cached --check passed. Postcommit status retains compose/shell/vendor, deployment files, old remote audit, analysis/acceptance tools and five associated tests. No push/deploy or controller command. Earlier acceptance traces still correspond to then-dirty builds, not this commit as an exact image source snapshot.

### Vendor receiver/core consolidation

Confirmed native CMakeLists.txt lines 245–251 discovers only its own nested unit_tests/*.cpp; outer gear_sonic_deploy/unit_tests was not included. Reviewed four native headers and all four corresponding tests. Moved two obsolete nested receiver/session tests into recoverable host backup `/tmp/sonic-test-consolidation.KwfVzU`, then moved newer outer tests to the actual nested discovery directory. No contents discarded; empty outer directory left alone. Current tests validate 20-byte clock reply and consumed-execution ACK, unlike old copies.

Compiled four test sources with g++ -std=c++20 -O3 -ffast-math -pthread -Iinclude and -lgtest -lgtest_main inside network-disabled candidate08, vendor source mounted read-only. All 16 tests across composer/decoder/session/receiver passed. Created vendor commit `b8de203555ae3c6e90e0a4df49933868be1fe4a5` (`Add experimental reference composition and private gesture receiver`), eight files only. This standalone test build is not full controller build or dynamic qualification. Vendor remains dirty in gamepad.hpp, input_interface.hpp, interface_manager.hpp, g1_deploy_onnx_ref.cpp and two new integration/experiment tests. Main vendor pointer deliberately remains uncommitted until integration batch is settled. No deployment, push or history rewrite.

### Vendor input and disabled deceleration batches

Reviewed remaining native diff and both input tests. Standalone compilation against current repo headers (not image header copies) exposed utils.hpp relying on transitive <mutex> for std::unique_lock. Added only the direct include. Recompiled with C++20/O3/fast-math/HAS_ROS2=0, current source mounted read-only and SDK headers from candidate08. All seven tests passed: three active-input forwarding/withdrawal/snapshot tests and four deceleration/default-off/deadman tests. Network-disabled container isolates the tests' loopback ZMQ constructor from live services; no robot DDS commands.

Created three bounded vendor commits after staged diff checks:
- `9a53348be0f8e223add56ff404b032ff747725b3`: direct mutex include only.
- `28f5e911a8cc75e0c70271f846f5256a74959b4b`: active upper-body and gesture input forwarding plus three tests.
- `7beb370e2322e55cfeda7a09293ed34c0b410986`: default-off sub-floor deceleration experiment plus four tests. Commit explicitly records failure to improve prior full-trial tilt and is not a qualified stop fix.

Final vendor status contains only modified g1_deploy_onnx_ref.cpp. This file has already been reviewed but combines reference receiver/controller wiring, planner seam diagnostics and opt-in C2 blending; still needs separated staging/build verification. No deployment or push; main vendor pointer remains pending. Main deployment/tools/docs groups from previous status remain untouched.

### Exact native source builds and vendor pin recorded

Partially staged eight reviewed gesture-controller hunks, leaving 51 diagnostic/C2 lines unstaged. Initial filter quoting error produced no patch/index change; corrected filter asserted exact hunk count and boundaries. Built staged tree `a6a73e98da216dc03e0492e05f4fa3b0509e9bfb` by git-archiving only gear_sonic_deploy/src into disposable network-disabled candidate08 and running cmake --build /sonic/gear_sonic_deploy/build --target g1_deploy_onnx_ref -j2. Full compile and link succeeded. Committed exact matching tree as vendor `211d24df9a7e98a8619eb504dd7dc7530449c8b0` (reference receiver/controller integration only).

Then staged remaining diagnostic/C2 additions, independently built exact tree `2839d188ad866cc8f1ee0c20c1d9996610a7fe41` using same process; compile/link succeeded. Committed vendor `15abe6d3ee0e86237ad5b52a6e36fe052c803fd6`. Commit explicitly says C2 is default-off and prior candidate09 repeat exceeded envelope; not dynamically qualified. Vendor status verified clean. A subsequent relative-directory error prevented main pointer update (no main mutation occurred); corrected using absolute repo path, reverified empty staging and clean vendor, committed main `d3364db` pinning vendor from 9084730 to 15abe6d.

Separately main `e53ec42` exposes default-off foot contact diagnostics in compose and runner script. Reviewed two-file diff; bash -n, docker compose config --quiet, and invalid-flag exit-2-before-launch check passed. Requires simulator support committed in 1ed7a19. No runner/controller launched by these checks.

End-of-stage main status has only untracked experimental Dockerfile/dockerignore/override/catalog, older remote audit, five analysis/acceptance test files, and analysis/acceptance tools. No tracked modified files remain; staging empty. Vendor clean. No production image/tag changed, no deployment/push/history rewrite. Both compile environments used candidate08 SDK dependencies; these builds are not a reconstructed image provenance claim for existing simulation trials.

### Analysis and acceptance tools versioned

Reviewed four read-only analysis modules and their four unit test files, then ran all 14 tests with remote Python standard library. Main `193caba` records transition timing, motion phase layout, sampled contact and walking metrics. Limitations remain explicit: not sole slip, not oscillation, not exact controller transition timestamps, and no quality pass inferred from sequence completion.

Reviewed walking_gesture_trial.py and four offline controls/gate tests. Tested only the isolated tests, not the actual motion sequence. Main `d871424` versions the command-sending harness and its tests, clearly identified as experimental simulation-only tooling.

Reviewed parity/receiver/stream probes and both Python IPC smoke scripts. Built three probes with C++20/O3/fast-math/pthread against current committed vendor headers inside network-disabled candidate08, repo and wave artifact mounted read-only. 1000 native/Python comparisons: max error 1.7763568394002505e-15, tolerance 1e-12. Snapshot IPC: 256 frames/4096 scalars/77335 bytes matched exactly, wall exchange 0.12931 s, source content 860c474044f353392974478231508aa7212a2b6e7b3fa849559abbc263f97aeb. Two synthetic-clock executions completed on same private channel: 2192 updates, max snapshot age 9 ticks, median/p95/max update 8.00565/8.69906/10.41078 ms, zero updates above 20 ms. This is not live policy or dynamics evidence. Main `ca762fe` records six probe/smoke files. Local draft --cancel-after extension was deliberately NOT mixed into this existing-code cleanup; it remains pending later feature work.

Postcommit main status: only four experimental deployment files and older remote audit remain untracked. No staged leftovers, tracked code changes, source deletion, push, or live service mutation in this stage. Deployment/image provenance and documentation cleanup still required before marking repo cleanup complete.

### Deployment files and final code inventory

Main `40e21d0` records four reviewed experimental deployment files. Merged compose config passed; PreparedPlanStore recomputed catalog plan hash and validate_artifact(write_result=False) passed against the actual wave artifact in a network-disabled container with both source/artifact mounted read-only. No approval generated. Dockerfile was reviewed, but no new full image built/deployed in this cleanup; native staged builds were validated separately.

Final remote inventory: main HEAD 40e21d0 with ONLY untracked docs/architecture/sonic-composition-audit-20260915.md; vendor clean at 15abe6d3ee0e86237ad5b52a6e36fe052c803fd6; simulator clean at 1ed7a193ed9cb99b67d397edff1f17d2400d53be. Main source/setting/tool commits since 065caab: 19f2c5f, a34d923, e9be18a, 3ac159e, 0458762, e53ec42, d3364db, 193caba, d871424, ca762fe, 40e21d0. Vendor commits since 9084730: 80b9197, b8de203, 9a53348, 28f5e91, 7beb370, 211d24d, 15abe6d. No push or history rewrite.

Re-inspected live tracker: social-motion/sonic:composition-candidate-20260915-08, image ID sha256:43f831dd6465d7ad00d391c97191d5adf4f066241ebb3db32223cd1e4e75489e. Build base local tag social-motion/sonic:196f27c-workspace-v3 currently resolves to sha256:817e8c9c356f20e3117721c0f9009c50372f27df57a5e217c5e7d7e459158abf. These are local image IDs/tags, not claimed registry digests. Current committed source is NOT identical to historical candidate08 build; earlier dynamic trials ran then-dirty worktrees. No retrospective commit-only provenance claims.

Remote audit is older (185 lines); do not blindly commit it as current. Asked user explicitly for permission to sync this updated audit to same lab repo, preserving old backup, and commit. Awaiting reply; no audit upload attempted. Local unshipped --cancel-after smoke extension remains a separately identified draft for future work, not included in current cleanup commits.
