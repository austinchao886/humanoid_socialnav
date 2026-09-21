# PICO 4 Ultra source: simulation bring-up

Status: receiver and isolated publisher tooling prepared. Live body tracking,
SMPL conversion, and closed-loop physical simulation are separate gates;
none is implied by a successful image build or a synthetic replay.

## Workspace audit, 2026-09-18

Active integration repository on `adcspublicrobot-codex`:
`/home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline`.
The Mac `motion_pipeline_deploy` directory is an older deployment snapshot.

- Integration HEAD at inspection: `142f06a65bc7392a1af8ae81e4d4b10a132a9480`.
- Vendor HEAD: `15abe6d3ee0e86237ad5b52a6e36fe052c803fd6` (clean).
- Existing tracker image: `social-motion/sonic:composition-candidate-20260915-08`.
- Existing controller uses `--input-type manager`, with custom persistent
  supervisor startup. It is not the upstream PICO `zmq_manager` session.
- Existing Isaac runner uses `g1_deployment_v1`; DDS domain 42, loopback,
  simulation-specific topics. Preserve these boundaries during integration.
- No host PICO environment or PC service was present at inspection.

## Current commands

Run these from the integration repository on the workstation:

```bash
docker build -f deploy/docker/pico.Dockerfile -t social-motion/pico:15abe6d .
./tools/run/run_pico_pc_service.sh
# In a second terminal, with the headset streaming full-body data:
./tools/run/run_pico_probe.sh --duration 10
# After the raw tracking gate passes:
./tools/run/run_pico_capture.sh
```

The PC service is extracted from the pinned Ubuntu 22.04 package into
`runtime/pico/pc-service`; no system package installation is needed. Its SDK
uses shared memory, so diagnostic containers use host IPC. The publisher has
CPU PyTorch and no GPU allocation or DDS dependencies. Stop the foreground
service/capture with Ctrl-C. The setup session started the service in the
background; identify its PID with `pgrep -af RoboticsServiceProcess` and send
SIGTERM to that exact process when finished.

The headset's PC Service IP must be `172.26.13.88` on a reachable network.
Use Head and Controller tracking, Send, and Full body. Pair/calibrate both
ankle trackers in the headset. A connection to another computer will not
populate this workstation's SDK shared memory.

The probe records raw 24-joint `xyz, qx,qy,qz,qw` samples and device timestamps.
It requires fresh timestamps at >=30 Hz, valid finite poses/quaternions, and
no observed gap >250 ms, including the beginning/end of the observation.
These are initial input-quality criteria, not measured end-to-end latency.
Its files are not canonical SMPL replay files.

The capture command runs the pinned upstream coordinate conversion and
publishes only poses on diagnostic port **5566**, recording upstream NPZ
batches. It does not publish manager start/stop commands. The running tracker
uses another endpoint. Do not infer live motion quality from a build or import
check; inspect the recorded stream while moving each arm and foot separately.

## Remaining physical simulation gates

1. Pass the raw body probe while wearing the hardware; inspect sample movement.
2. Capture and verify converted SMPL poses, handedness, floor orientation,
   timestamps and packet gaps. Confirm loss/reconnection behavior.
3. Integrate a mutually exclusive PICO mode into the persistent supervisor, or
   run a separate isolated SONIC simulation session. Upstream uses
   `pico_manager_thread_server.py --manager` paired with `--input-type
   zmq_manager`. Do not simply send PICO manager commands to the existing
   keyboard/gamepad manager or launch a second LowCmd writer on domain 42.
4. Confirm the selected encoder/decoder and observation config support SMPL
   streaming. Record exact image ID, checkpoint hashes, repository revisions
   and dirty patch before the trial. The current release checkpoint must not
   be relabeled as the upstream low-latency checkpoint.
5. Validate unsupported standing, arm motion, shallow squat, short steps,
   stop and tracking loss. Record actual simulated state and target motion;
   a kinematic viewer alone is not acceptance. Measure falls, tracking error,
   stream age, latency and realtime factor. Test disconnect recovery explicitly.

The pinned reader currently detects stale tracking after **5 seconds**. That
is not the probe's 250 ms threshold and must be addressed in the live control
path before claiming prompt tracking-loss handling.

Official references:
- [PICO setup](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/vr_teleop_setup.html)
- [Whole-body simulation workflow](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vr_wholebody_teleop.html)

Hardware availability was confirmed by the user. The PC Service address is
being checked because no connection to the workstation was visible during
the initial audit.

## Verification results

- Built `social-motion/pico:15abe6d`, image identifier
  `sha256:cfcbfa95f9e8fc4857da787fd9eaf6378968e9f335a465cf8f5bd81a2a661ced`.
- Publisher CLI/import and G1 FK calibration initialization passed.
- Five probe tests passed: fresh samples, frozen timestamps, invalid poses,
  missing stream tail, and invalid body shape/quaternions.
- Shell syntax, Compose config, and tracked diff whitespace checks passed.
- Live probe `runtime/pico/probe-20260918T230601Z/tracking.json`: **NOT_READY**,
  0 fresh frames over 10.02 seconds. SDK connected to the PC service at
  `127.0.0.1:60061`. Receiver listens for headset connections on TCP 63901.
- Bounded publisher smoke test waited for body tracking and was interrupted
  after 8 seconds; no converted frames or physics acceptance obtained.
  The upstream autostart call prints a missing `/opt/apps/roboticsservice/runService.sh`
  warning inside the container because the PC service is managed on the host;
  the SDK itself successfully connects to that host service.
- Existing SONIC/Isaac processes were not restarted or switched to PICO.
- Added source files remain uncommitted. No vendor source was modified.
