# Interactive simulation acceptance

Scope: Isaac Sim, domain 42, loopback interface, SONIC simulation topics.
These tests are **not physical-robot qualification**. Disconnect the PS4 TCP
client before testing; only one command source should control port 16042.

## Normal operation

Start services individually with the scripts under `tools/run`. Wait for a
fresh Isaac `INTERACTIVE` status before using joystick input. Container `Up`
alone does not mean the controller is ready.

1. Hold L1 to enable joystick locomotion. Release L1 to remove the movement
   command immediately; the standing controller remains active.
2. Approve a validated reference using
   `./tools/run/approve_motion.sh REQUEST_ID MOTION_ID`.
3. The supervisor preempts locomotion, checks settling, plays the reference,
   returns to neutral, checks standing stability, then restores joystick mode.
4. Release L1 during the reference and reapply it only after `INTERACTIVE`.
   A fresh release/repress interlock on reference return is not yet implemented;
   holding L1 can resume movement when joystick mode returns.
5. Use `./tools/run/listen_motion_status.sh` to see DDS failures. Command
   publication is not an execution-success acknowledgement.

`./tools/run/abort_motion.sh REQUEST_ID MOTION_ID` is a **hard cancellation**,
not a smooth abort-to-standing. Isaac restarts into supported `READY`; abort
remains latched. Do not expect automatic joystick recovery. After inspecting
the cause and disconnecting joystick input, an operator may restart the SONIC
service to begin a fresh interactive bootstrap. Do not automatically approve
another motion after a safety failure.

## Automated checks

Run from the motion_pipeline repository on the simulator host:

```bash
python3 tools/acceptance/run_runtime_acceptance.py
python3 tools/acceptance/runtime_edge_acceptance.py
```

The first command attempts ten alternating video/Kimodo round trips in one
Isaac session. Rounds 4 and 8 include a gentle normalized yaw input of 0.15
(this is joystick input, **not** 0.15 rad/s). Each round includes locomotion
before approval and after return. Failure stops the sequence. Reports are in
`.build/runtime-acceptance-*`; the original session must remain unchanged.

The second command checks L1 release, loss of the joystick TCP connection, and
mid-reference abort. Its report distinguishes new supported-session recovery
from same-session interactive recovery. A bounded status gap is allowed only
during explicit hard-abort recovery, never during normal transitions.

The protocol-driven input exercises the server bridge/controller path; it
does not qualify the physical PS4, browser mapping, or Internet transport.
The short release/disconnect checks establish functional state continuity,
not subjective gait quality or a comprehensive transient-tilt bound.

## September 13 qualification findings

- First ten-round attempt: rounds 1 (video) and 2 (Kimodo) passed. Round 3
  completed its video but failed before returning to joystick; the run stopped.
  Session: `f00a4ddf1f394c899ece0a4dd40e52c4`.
- Report: `.build/runtime-acceptance-20260913T001236/summary.json`.
- Last 40 simulation seconds of the failed neutral hold: maximum joint speed
  oscillated between 0.866 and 0.950 rad/s; the gate requires <=0.9 continuously
  for 3 wall-clock seconds. Sampled quiet stretches were at most 0.8 simulated
  seconds. Maximum sampled root tilt was 0.035 rad. This supports a standing
  gate timeout as the upstream failure; subsequent LowCmd timeout alone does
  not identify the original exception.
- Added original execution-error logging and last-state details on standing
  timeout. Safety thresholds were not relaxed.
- L1 release and TCP disconnect maintained the same interactive session in
  initial short checks. Abort stopped reference playback and Isaac restarted
  to supported READY. The abort latch previously caused futile automatic
  bootstrap retries; maintenance now leaves an explicit abort latched.
- Supervisor regression suite: 22 tests passed (including a real verbose PTY
  continuity test and abort rearm inhibition).
- Instrumented second attempt:
  `.build/runtime-acceptance-20260913T002422/summary.json`. Round 1 completed
  video and passed the supervisor neutral gate, but after the INTERACTIVE
  request Isaac remained in GROUNDING. Sampled joint speeds were around
  1.5--1.7 rad/s, root tilt around 0.056 rad. The 120-second round-trip bound
  expired before INTERACTIVE. This is a separate planner-grounding stall,
  not evidence of another LowCmd outage. The stalled run was explicitly
  aborted; thresholds were left unchanged.
- Gentle-turn rounds 4 and 8 were **not reached**. Neither ten-round sequence
  passed. The automated edge script initially rejected the expected hard-abort
  status gap; that harness bug was fixed, but a complete end-to-end rerun of
  the revised edge script is still pending.

Continuous switching qualification remains open until a complete ten-round
run passes; automatic recovery is not a substitute for LowCmd/session
continuity. Smooth abort-to-standing and Pico live-reference integration
remain subsequent work.

## Warm-return follow-up

The simulator previously called `begin_control_handoff()` even for an
unsupported same-session reference-to-planner return. This rearmed cold-start
torque blending, target shaping, and bootstrap damping on an already-live
controller. The warm path now retains authoritative control via
`end_control_handoff()`; supported cold startup still uses the original
handoff. Ground stability gates and watchdog thresholds are unchanged.
Two lightweight cold/warm dispatch regression tests pass.

Report `.build/runtime-acceptance-20260913T003253/summary.json`:

- Video, Kimodo, video: three complete round trips passed in Isaac session
  `1e618850849e4367b86ebaa60fdd9f2e` with no session restart.
- Round 4, with normalized yaw input 0.15, failed during entry preemption,
  before the Kimodo reference played: raw right-shoulder-pitch command error
  reached 4.2323 rad and the existing safety gate stopped execution.
- Controller logs confirm the indexed standby reference was correctly
  `isaac-neutral-v1`. The fault occurred directly after planner disable and
  reference/heading reset, not after selecting the requested Kimodo artifact.
- This does not establish ten-round acceptance or safe turning preemption.
  Next implementation should explicitly stop locomotion/turning under the
  live planner, validate stable hold, and only then change reference ownership.
  That intermediate phase needs to suppress fresh joystick movement intent
  while retaining emergency-stop/deadman behavior. Do not mask the fault by
  raising raw-command limits or clamping the reported error.

The revised edge harness completed with exit code 0:
`.build/runtime-edge-20260913T003752.json`. L1 release and TCP disconnect
retained session `7801820577204666a65fc3ae21c2fb2f`; abort during video
EXECUTING produced ABORTED then supported READY in new session
`1e6d69e71a0d4628ba6e007fbfa9d715`. No automatic interactive rearm occurred.
This supersedes the earlier pending edge-harness rerun note. It does **not**
qualify stopping quality: the sampled horizontal speed five seconds after
L1 release was approximately 0.25 m/s, so residual motion/oscillation still
needs a time-window-based velocity and tilt criterion rather than checking
only that state remains INTERACTIVE.

## Planner-held preemption follow-up

Native source `9084730` adds a supervisor-owned hold before planner disable.
The sequence is now:

1. Publish runtime request `PLANNER_HOLD`; signal the gamepad planner to hold.
2. Keep nominal unsupported planner control and the current facing anchor.
   Ignore movement, heading reset, play and mode-toggle inputs during hold;
   retain emergency controls. The planner is not disabled on deadman release.
3. Require three continuous seconds of the existing neutral height/tilt/joint
   speed gate, additionally with horizontal root speed <=0.15 m/s and absolute
   yaw rate <=0.20 rad/s. Missing velocity data does not pass. Timeout is 60s.
4. Only then reacquire reference support, disable planner to indexed neutral,
   qualify neutral, and switch to keyboard/reference control without another
   reset. Normal completion uses the already-corrected warm planner return.

Isaac remains INTERACTIVE during the unsupported hold; the runtime request
state identifies that joystick input is temporarily suppressed. The test
harness requires both request and simulator state to return to INTERACTIVE,
so a held planner is not mistaken for a completed round trip.

The native yaw integrator incorrectly used 0.02s despite `G1Deploy::Input`
running at 100Hz (0.01s). Corrected that timestep, preventing approximately
double heading accumulation. This corrects the input-loop computation, not
wall-time versus simulated-time realtime-factor differences.

Validation: three compiled native tests and 23 supervisor tests passed.

- The initial support-before-hold experiment did not settle and was aborted.
- Unsupported hold before correcting yaw also timed out; joint speeds stayed
  above the unchanged 0.9 rad/s threshold.
- Final corrected-yaw test passed a turning -> video -> joystick round trip
  in session `4345cc20e6914815b2f703992d213d0e`:
  `.build/planner-hold-corrected-yaw-video.log`.
- Subsequent ten-round attempt `.build/runtime-acceptance-20260913T005752/`
  passed video round 1, but Kimodo round 2 failed **before playback** when
  planner hold did not converge. Final sampled joint speed was 1.706 rad/s,
  root horizontal speed about 0.0065 m/s and tilt 0.040 rad. The safety/recovery
  path replaced the session; this is not continuous-switching acceptance.

Remaining blocker: reproducible planner idle joint settling, not just root
translation/rotation. Single turning success is insufficient to claim the
shoulder discontinuity is eliminated or that ten-round qualification passed.
No neutral/admission/watchdog/position-error thresholds were relaxed.

## TGS joint-settling investigation (2026-09-13)

At 200 Hz, some ankle joints report a persistent velocity despite almost
constant position. Treat this separately from actual oscillation: FFT power
above 50 Hz alone is not proof of appreciable physical motion, and removing
the velocity mean from a plot would hide the bias entering the policy.

PhysX documents a TGS steady-state position/velocity discrepancy when external
forces are integrated once per frame but constraints are integrated per
substep. The per-iteration external-force flag mitigates, but does not promise
to eliminate, this discrepancy for articulations:
https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html#tgs-steady-state-velocity-and-position-discrepancy

The installed Isaac 4.5 PhysxSchema exposes
`CreateEnableExternalForcesEveryIterationAttr`; the installed Isaac Lab config
does not expose a corresponding field. The runner now authors the scene flag
before `gym.make`, reads it back afterward, and fails if it was not retained.
`SONIC_TGS_FORCES_EVERY_ITERATION` selects it. Reports now record the actual
requested task solver iterations rather than copying the profile constants.
Three mocked scene-authoring tests and two warm-handoff tests pass.

Bounded idle/walk-stop/turn-stop trials compared 4/1, 4/4, 8/4 iterations,
and the per-iteration external-force flag. The combined 8/4 + flag candidate
gave the most consistently low stopped velocity in these trials. No PD gains,
torque limits, deadman behavior, support forces during interactive standing,
or safety gates were changed. No reported joint velocity is filtered.

For each stopped phase, the table uses the final five **simulation** seconds
(1001 trace samples), not a single endpoint. Values are the 95th percentile
of the instantaneous maximum absolute joint velocity, in rad/s:

| Phase | Baseline 4/1, flag off | Candidate 8/4, flag on |
| --- | ---: | ---: |
| Idle | 0.3242 | 0.1245 |
| After forward motion / L1 release | 0.6491 | 0.1264 |
| After gentle turn / L1 release | 0.5804 | 0.1266 |

Reports: `.build/oscillation-baseline-metrics.json` and
`.build/oscillation-tgs8-metrics.json`. These are separate session trials, not
a statistical multi-seed qualification. Some residual velocity bias remains:
candidate idle right ankle pitch mean velocity is 0.1190 rad/s while the
position-derived velocity RMS is 0.00269 rad/s. After turning, waist-roll
position range is 0.0112 rad over the final five seconds. Therefore this is
reduced settling disturbance, not proof of zero joint oscillation.

`tools/acceptance/oscillation_trial.py` records bounded phase markers and
releases deadman in cleanup. `standing_window_acceptance.py` is a read-only
2 Hz window check using existing hold bounds and requiring unsupported,
same-session, joystick-owned INTERACTIVE throughout. It does not replace
the 200 Hz runtime safety monitor.

### Same-session switching result

`.build/runtime-acceptance-20260913T124710/summary.json` passed **10/10**
alternating video/Kimodo round trips with the 8/4 + per-iteration-force
candidate and 200 Hz trace. All ten used Isaac session
`c10585e2a6674d3d88c8aa582280b927`; rounds 4 and 8 included normalized yaw
0.15 before approval. Every round completed playback, returned to joystick,
and could walk again, without replacing the session. This supersedes the
earlier failed ten-round attempts for this candidate configuration only.

This is **not** ten-round smooth-stop acceptance: after four wall seconds of
deadman release, rounds 3 and 8 sampled maximum joint speed 0.970 and 1.784
rad/s respectively. Round 8 root tilt was 0.0599 rad. Other stopped endpoints
ranged 0.115--0.333 rad/s. The functional harness checks state/session, not
that all stop transients have vanished. It now records simulation time and
trace path to correlate future transients with the high-rate joint signals.

With high-frequency trace, startup release realtime factor was approximately
0.642 for baseline and 0.534 for the candidate, in separate runs. This suggests
a compute cost; neither value is a full-session timing benchmark. Normal
5 Hz trace must be checked separately. Existing traces are retained, not
deleted when the diagnostic frequency is reduced.

The 5 Hz trace rerun `.build/oscillation-tgs8-normal5-metrics.json` retained
low settled speeds: final-five-simulation-second max-joint-speed p95 was
0.0915 rad/s idle, 0.2132 after forward/L1 release, and 0.1876 after gentle
turn/L1 release. These windows contain only 25 samples; do not use their FFT
output to infer high-frequency vibration. Startup release realtime factor was
still about 0.531, so reducing trace logging did not restore realtime speed.

`.build/standing-window-20260913T130051.json` passed all 600 observations
over 300 wall seconds (185.3 simulated seconds) in session
`dda508bdf46e4ddbab08124967afd477`, with 5 Hz trace. Entire-window maxima:
joint speed 0.1897 rad/s, root tilt 0.03202 rad, planar speed 0.000792 m/s,
and yaw rate 0.003344 rad/s. Both support scales stayed zero and request/state
remained INTERACTIVE. This validates settled standing, not every L1 transient.

The deployment task and asset-profile defaults now agree on 8/4. The runner
enables per-iteration TGS external forces by default only for
`g1_deployment_v1`; other profiles keep their previous default. Explicit
`SONIC_TGS_FORCES_EVERY_ITERATION=0` permits A/B, and values other than 0/1
fail validation. Five TGS helper tests, three solver default/bounds tests,
and two cold/warm handoff tests pass. No SONIC native/policy changes are part
of this physics correction; the runner/task/profile are read from the mounted
workspace, so a SONIC image rebuild is not needed for these changes.

Safety-edge report `.build/runtime-edge-20260913T130642.json` passed L1
release, TCP disconnect, and hard abort during video EXECUTING. Abort
recovered to supported READY in session `3212b39d9e144e10a94c369ed4eb6978`,
without automatic interactive rearm. These are functional safety-edge tests,
not certification of smooth stopping or smooth abort-to-standing.

### Base Compose verification and version checkpoint

After restoring SSH, launched **only** `docker-compose.motion.yml` (no
diagnostic override) and explicitly restarted SONIC after the deliberate abort.
The new runner retained the TGS flag from its deployment-profile default;
trace environment is 5 Hz with no solver/TGS override variables. Asset-profile
tests passed 12/12; simulator helper/default/handoff tests passed 10/10.

`.build/tgs-defaults-video-smoke-20260913.log` passed gentle-turn -> video ->
joystick, including walking again, in session
`c3d4c15719174cf497d33c6af3eaad2f`. The subsequent
`.build/standing-window-20260913T132622.json` passed all 120 observations
over 60 wall seconds (37.0 simulated seconds). Max joint speed was 0.2010
rad/s, tilt 0.02218 rad, planar speed 0.00720 m/s, yaw rate 0.11697 rad/s.
No support or session replacement occurred during the test. This final
short window does not replace the earlier five-minute qualification.

Simulator checkpoint: `37d9bb0` in unitree_sim_isaaclab. The functional
switching/settled-standing improvement is ready for simulation use; sporadic
L1 stop transients, realtime performance, smooth abort-to-standing and Pico
live-reference integration remain open. No hardware qualification is implied.

## Stop-transient follow-up: distinguish motion from velocity bias

The prior 8/4 same-session and standing passes did not guarantee stability for
every stopped stance. Six new stop trials at 3/4/5 wall seconds of forward
input, comparing deadman release with stick centering while deadman stayed
held, reproduced elevated reported joint speed under **both** stop modes:
`.build/stop-transients-20260913T133511.json`. A repeated four-second pair is
`.build/stop-transients-20260913T133846.json`. No input, policy, gain, or safety
behavior was modified for these tests.

The status file updates only about once per second. Repeated reads must not
be counted as high-rate independent observations. A new **subscriber-only**
tool, `tools/analysis/record_sim_stop.py`, captures LowState and LowCmd on
hard-coded `rt/socialnav_sim/g1/*` topics, domain 42, loopback. It cannot
publish a command and is bounded to at most 600 wall seconds. The normal
Isaac trace remains at 5 Hz. Output is created exclusively, not overwritten.

`tools/analysis/analyze_sim_stop.py` detects the actual incoming stop packet,
compares dq with position differences using the 200 Hz physics tick counter,
and reports position-derived velocity and position range across all joints.
Three synthetic/protocol tests pass. Recording callback queue drops were zero;
analyzed LowState tick gaps were four (50 Hz in simulation time).

Baseline DDS record `.build/stop-dds-baseline-20260913.jsonl` covers the last
two stops of the six-case test plus the repeated four-second pair. Its
10--20-wall-second max-joint-speed p95 values were 0.1281, 1.7463, 0.1842,
and 1.7924 rad/s. In the final release case, right ankle roll dq mean was
-1.4763 rad/s and RMS 1.5212, but position-derived RMS was only 0.03410 and
position range 0.01694 rad. This is substantial residual numerical velocity
bias, not a 1.5 rad/s physical ankle oscillation. There was also real movement:
left shoulder roll position-derived RMS reached 0.2046 rad/s in that window.

### Bounded 8/1 experiment

Retained eight position iterations and per-iteration external forces; reduced
velocity iterations to one via `.build/stop-velocity1.yml`, within the existing
diagnostic bounds. No permanent default change was made for the experiment.
NVIDIA notes that TGS generally needs very few (around one) velocity iterations:
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.1/dev_guide/simulation_control/simulation_control.html
This supports an experiment, not a guarantee for this installed simulator.

`.build/stop-transients-20260913T134227.json` completed all six stop cases in
session `041aa283c3a8455d8be4b1cc032edef7`. The complete DDS record and metrics
are `.build/stop-dds-velocity1-20260913.jsonl` and the corresponding
`-metrics.json`. For the same 10--20 wall-second windows, max-joint-speed p95
was 0.2775, 0.1756, 0.0841, 0.1173, 0.1327, and 0.1303 rad/s. The subsequent
60-wall-second standing window passed all 120 observations:
`.build/standing-window-20260913T134519.json`.

This is a single-session comparison with differing initial stances, not a
paired multi-seed proof that 8/1 is always better. Real residual motion remains:
the first 8/1 stop had waist-roll position-derived RMS 0.1660 rad/s and range
0.03341 rad, despite the much lower reported max dq. Therefore reduced
velocity bias and passing settle gates must not be described as zero visible
oscillation. Functional switching qualification for this candidate is tracked
separately below; deadman still immediately clears locomotion intent.

The joint-speed-only DDS convergence measure requires three continuous wall
seconds with max absolute dq <=0.9 rad/s; gaps over 0.2 seconds reset the
window. It is not the complete root/translation safety gate. All six 8/1
cases completed that window 3.81--4.95 wall seconds after the received stop
packet. Five synthetic/protocol tests now cover mode decoding, independent
position-derived velocity, and both passing/failing convergence cases.

The 8/1 candidate passed **10/10** video/Kimodo round trips, including turns
in rounds 4 and 8, without changing session
`041aa283c3a8455d8be4b1cc032edef7`:
`.build/runtime-acceptance-20260913T134714/summary.json`. This is a separate
qualification from the earlier 8/4 ten-round pass. It covers half-stick
forward input and gentle turns, not all joystick inputs or physical hardware.

Full-stick straight-walk follow-up stayed within the existing 0.45 m/s
requested-speed cap; no cap or deadman semantics were changed:
`.build/stop-transients-20260913T135816.json` and
`.build/stop-dds-velocity1-full-20260913-metrics.json`. The four-second
stick-center and deadman-release stops had 10--20-wall-second max-dq p95
0.0739 and 0.1436 rad/s; position-derived RMS maxima were 0.01944 and
0.08481 rad/s. Joint-speed-only three-second convergence completed at
5.24 and 3.61 wall seconds respectively. The subsequent 60-wall-second
standing test passed all 120 observations:
`.build/standing-window-20260913T135920.json`.

Full-stick stop transients are still nonzero: low-rate status samples in the
first two seconds reached dq 4.63 rad/s and root tilt 0.1014 rad for centering,
and dq 4.31 rad/s / tilt 0.0567 rad for deadman release. These include the
initial braking motion and must not be called an oscillation amplitude or an
instantaneous physical stop. The runtime stayed INTERACTIVE with no session
replacement. Full-turn, reverse, lateral and physical-robot qualification are
not established by these straight-walk tests.

Safety-edge report `.build/runtime-edge-20260913T140050.json` passed L1,
disconnect, and hard abort during video EXECUTING with 8/1. Abort recovered
to supported READY in new session `e8951824f6154d21a311c80e161c634f` without
automatic interactive rearm. The deployment task/profile default is now
8/1 with per-iteration TGS external forces retained. Explicit velocity-iteration
override 4 remains supported for reproducing the earlier configuration.
No joystick, planner, policy, PD gains, velocity feedback filters, torque
limits or safety thresholds were changed in this follow-up.

Final base-Compose restart (without `.build/stop-velocity1.yml`) entered
INTERACTIVE in session `251cc83b314243319f0791814ad6b7fe`; TGS readback was
true, trace was 5 Hz, and no solver/TGS environment override was present.
`.build/standing-window-20260913T140345.json` passed all 120 observations
over 60 wall seconds. The task/profile defaults supply 8/1 directly.
Twenty-eight related tests passed: 5 TGS helper, 4 solver defaults/bounds,
2 handoff, 12 asset-profile, and 5 stop-analysis/protocol tests.
Simulator checkpoint is `33c2cd5`. No SONIC image rebuild was necessary.

Remaining work is reduced **physical** stop/idle motion and realtime speed,
not merely lowering reported dq. The numerical improvement and finite test
coverage above do not certify full-turn/reverse/lateral behavior or hardware.
