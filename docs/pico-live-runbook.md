# PICO development runbook

Status: recorded playback works; live teleoperation is not qualified. Hardware deployment is excluded.

## Reproduce the passing playback baseline on the lab server

From /home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline:

```sh
docker stop sonic-tracker
docker compose -p social-motion-pipeline -f docker-compose.motion.yml -f deploy/sonic-composition.override.yml -f deploy/arm-observation.override.yml -f deploy/pico-live-performance.override.yml up -d --no-deps isaac-runner
docker start sonic-tracker
```

Wait for INTERACTIVE in ../motion_exchange/.runtime/isaac_status.json and supervisor readiness. The supplied recording is already approved for this simulation test:

```sh
docker exec sonic-tracker motion-cli --domain 42 --interface lo control approve_execute pico-gmr-full-v1 pico-gmr-full-v1
```

Results are in ../motion_exchange/executions. Check the per-motion report's performance.unsupported_playback_realtime_factor; do not infer wall-clock speed from a rendered replay.

## Diagnostic source

MSI's existing SDK reader owns body_sample.json. tools/run/pico_raw_forwarder.py reads that snapshot and publishes loopback 5558. An SSH reverse forward maps lab loopback 15558 to it. deploy/pico-live.override.yml starts a persistent retargeter; outputs are ../motion_exchange/.runtime/pico_live/{status,reference}.json. It never selects a controller mode. Clock calibration from tools/analysis/measure_pico_clock.py expires after 300 seconds and must be refreshed for latency measurements. These timestamps describe host sampling, not verified headset sensor exposure.

## Experimental controller image

```sh
docker build -f deploy/docker/sonic-pico-live.Dockerfile -t social-motion/sonic:pico-live-development .
```

This image defaults SONIC_ENABLE_PICO_LIVE=0 and is not deployed. It contains an experimental arm-request adapter, not a qualified public control interface. Admission tests do not establish safe recovery. Do not promote this image until recorded-stream, calibration, recovery, source switching and real-time tests pass.

## Recovery during baseline experiments

Existing emergency-stop and runner critical checks remain active. A failed trial is not retried by weakening thresholds. Restore the baseline overlays above after stopping SONIC; wait for a new ready simulation session before another recorded test. The unarmed receiver can be stopped independently with docker stop pico-live. Reconnection must never implicitly arm live control.

See pico-live-development.md for cross-repository revisions, failed experiments and outstanding acceptance work. Recordings, traces, weights and images stay outside Git; only compact summaries are committed.
