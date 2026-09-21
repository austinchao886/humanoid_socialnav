# PICO full-body source: completed offline simulation test

The complete saved recording has passed a single unsupported-physics Isaac run.
The robot returned to standing and normal joystick standby.

## Working path

PICO recording on MSI → saved SMPL conversion → GMR full-body retargeting →
validated G1 joint/FK artifact → existing SONIC supervisor → Isaac simulation.

The successful artifact is `pico-gmr-full-v1` in the lab's
`/home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_exchange/`.
It contains 29.94 seconds of captured motion plus 3-second neutral transitions
and 1-second neutral holds at both ends: 37.94 seconds / 1,897 frames at 50 Hz.

The original MSI recording remains at
`/home/austin/Documents/pico_recordings/session-20260920-175428/`.

## Measured result

- Result: COMPLETED; full reference frames 0–1896 observed.
- Artificial root support released during playback.
- 8,002 unsupported trace samples used for metrics (includes final hold).
- Joint tracking RMSE: 0.106 rad (6.08°), across all joints and sampled times.
- Largest absolute joint tracking error: 0.695 rad (39.8°).
- Peak command-to-actual joint error: 1.885 rad.
- Root height: 0.728–0.799 m; peak tilt: 0.615 rad (35.2°).
- Both arms and both legs moved in the actual physics trace.
- Real-time factor: 0.474; playback took 80.08 wall-clock seconds.

This is one completed offline trial, not a claim of perfect imitation, real-time
performance, live streaming qualification or physical-robot readiness.

## Repeat the already-loaded successful recording

On the lab server:

```sh
docker exec sonic-tracker motion-cli --domain 42 --interface lo \
  control approve_execute pico-gmr-full-v1 pico-gmr-full-v1
```

Use `--interface lo` explicitly: the CLI does not inherit DDS_INTERFACE for this
argument. The execution gate revalidates the artifact before playback.

For a new derived artifact, from the lab motion_pipeline repository:

```sh
./tools/run/run_pico_offline_retarget.sh pico-gmr-NEW 30
```

New IDs preserve existing artifacts. The underlying converter accepts
`--body-jsonl` and `--converted-dir` for other recordings. It uses the existing
pinned video/GMR container and local body models; no headset is required.
A new artifact must be preloaded before a warm execution. The first full-trial
attempt exposed the existing new-reference reload gap: LowCmd stopped for 0.5 s,
Isaac safely restarted, and the supervisor recovered. The retry succeeded after
confirming the full reference was loaded. Do not weaken the stale-command guard.

## Direct SMPL route remains experimental

Native protocol-v3 SMPL playback failed for the original recording, a
pelvis-centered copy, and an independent neutral SMPL fixture. These failures
are separate from the successful GMR/G1 route. The precise native policy/input
compatibility issue remains unresolved; pelvis centering alone did not fix it.
The trial handler is now opt-in with SONIC_ENABLE_PICO_SMPL_TRIAL=1 on future
supervisor starts. Its unit-tested watchdog is not a qualified live fallback.

The normal G1 playback path provides the neutral entry/exit and completion
handling used by the successful trial. Live PICO streaming is a later task.

## Evidence

Lab summary: `/motion_exchange/diagnostics/pico/offline_acceptance.json`.

Execution report:
`/motion_exchange/executions/isaac_g129_deployment_v1_20260921T025408Z_0001_pico-gmr-full-v1.json`.

Actual-state trace:
`/motion_exchange/executions/isaac_g129_deployment_v1_20260921T025408Z.jsonl`.

Artifact `preview.mp4` is a reference preview; it is not a video of actual physics.
