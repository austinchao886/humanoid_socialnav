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
