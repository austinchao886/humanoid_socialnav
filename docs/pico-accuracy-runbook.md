# Recorded PICO accuracy study

Run on the lab server in `/home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline`. This is isolated Isaac simulation, not hardware or live qualification. Keep the existing GPU torque-limited configuration and separate WebRTC renderer. The harness restarts Isaac and SONIC for each trial; do not run alongside another simulation experiment.

## Inputs and provenance

`docs/pico-accuracy-environment.json` records repository/image revisions, active settings, mounted source and checksums. Original `../motion_exchange/pico-gmr-full-v1` is immutable. Derived `pico-accuracy-half-v1` and `pico-accuracy-holds-v1` passed the existing validator. Their construction and selected source poses are recorded in `../motion_exchange/diagnostics/pico/accuracy-v1/generation.json` and each artifact's `accuracy_experiment.json`. The generator intentionally refuses to overwrite an existing study directory. Do not delete the original to regenerate a study.

## Repeat the nine fresh-session trials

```bash
python3 tools/run/run_pico_accuracy_trials.py --output ../motion_exchange/diagnostics/pico/accuracy-v1/trials-new
```

Choose a new output directory. The order rotates original/half/holds, half/holds/original, holds/original/half. Preserve `trials.json` including failures. This retains existing neutral admission and execution safety checks; the harness refuses nonisolated DDS or unqualified optimization settings. Fresh initialization can take about 90 seconds before each playback. Playback uses simulation-tick synchronization; wall-clock speed is reported separately.

## Reanalyze consistently and summarize

```bash
export PICO_EXCHANGE=/home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_exchange
export PICO_STUDY=/motion_exchange/diagnostics/pico/accuracy-v1/trials-v2
export PICO_IMAGE=social-motion/video-gem-gmr:16bebf4-bb1bbe4-workspace-v2

docker run --rm --network none -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 -v "$PWD:/workspace:ro" -v "$PICO_EXCHANGE:/motion_exchange" --entrypoint python "$PICO_IMAGE" /workspace/tools/analysis/pico_accuracy.py reanalyze --directory "$PICO_STUDY" --exchange /motion_exchange

docker run --rm --network none -e OPENBLAS_NUM_THREADS=1 -v "$PWD:/workspace:ro" -v "$PICO_EXCHANGE:/motion_exchange" --entrypoint python "$PICO_IMAGE" /workspace/tools/analysis/summarize_pico_accuracy.py "$PICO_STUDY" --output "$PICO_STUDY/summary.json"

docker run --rm --network none -e OPENBLAS_NUM_THREADS=1 -v "$PWD:/workspace:ro" -v "$PICO_EXCHANGE:/motion_exchange" --entrypoint python "$PICO_IMAGE" /workspace/tests/unit/test_pico_accuracy.py
```

MuJoCo supplies forward kinematics only; all measured robot execution is native Isaac. Each analysis contains phase-specific per-joint RMSE/peak/signed bias, requested/applied effort, saturation, root and endpoint errors, and diagnostic per-joint lag scans. Lag-adjusted metrics never replace unshifted acceptance metrics. Global and per-joint shifts are not sensor-latency measurements; boundary minima are especially inconclusive.

Endpoint errors use the same reference-model kinematics for actual and target states. Height normalization is the geometry extent in official neutral pose (1.288063 m). Raw world, pelvis-relative, and supplemental initial-frame yaw/XY-aligned world errors are distinct. SONIC G1 mode does not explicitly command root XYZ; exact controller heading initialization is not logged. Reference/deployed model equivalence is unproven.

Hold settling is an analysis gate, not a changed safety threshold: over the final two seconds require joint speed <=0.9 rad/s, planar speed <=0.15 m/s, yaw rate <=0.2 rad/s, tilt <=0.6 rad, and root height >=0.5 m. All holds remain in output, including failures to settle. Also report the portion before the next transition enters SONIC's 0.9-second future window. Unsettled holds cannot establish static controller bias.

The existing recording contains about 1.52 m root displacement. These comparisons diagnose that recording; they do not qualify stationary standing, walking, live transport, or hardware. The study preserves the baseline controller, gains, physics timestep and safety limits. No automatic candidate promotion occurs.
