# PICO accuracy attribution checkpoint — 2026-09-21

This study retains the torque-limited baseline. No gain, reference-conditioning, model, timestep or safety-threshold change is promoted. Three original-speed, three half-speed and three pose-hold trials use fresh Isaac/SONIC sessions in rotating order. Results below are unshifted native Isaac measurements; MuJoCo is used only for same-model forward kinematics.

## Interpretation and decision

**Demonstrated:** tracking differences are repeatable and persist beyond a simple uniform delay. Slower reference motion does not remove the large peak errors. Entry transitions and active motion must be assessed separately. Several held poses do not satisfy the declared settling gate, so large held-pose error is not by itself proof of static controller bias.

**Confirmed interface behavior:** deployed SONIC G1 mode consumes joint positions, joint velocities and anchor orientation, with a future window reaching 0.9 simulation seconds. Root XYZ is not an explicit G1-mode reference objective. Initial heading is aligned to the robot. Therefore raw world-path error measures imitation mismatch but cannot by itself establish failure to track an explicitly commanded Cartesian root path. PD targets can intentionally differ from desired posture; command/reference/actual differences are descriptive, not additive causal percentages.

**Supported hypotheses:** reference demand and the controller's posture/balance response deserve further isolation, particularly the high-error elbow/waist pose and wrist pitch. The first selected pose has approximately 29 degrees of reference pelvis pitch and 56 degrees of torso pitch in the reference model. This is substantial coupled torso demand. It does not prove that the pose is infeasible. Foot-height/contact conditioning also remains a candidate: selected reference poses have their lowest collision points about 4–6.5 cm above the reference floor. Root height alone is not an explicit command in this policy mode.

**Unresolved:** reference/deployed model equivalence, static feasibility under actual contacts, controller bias versus deliberate balance response, and hardware behavior. A conservative reference-model COM projection lies inside the hull of both entire feet for all selected poses, but that includes lifted feet and does not prove feasible support. No experiment independently identifies simulation-model mismatch.

**Decision:** retain the baseline. Half-speed playback is a diagnostic condition, not a real-time teleoperation correction. No conditioning is adopted merely to reduce error against an easier target. A correction requires an isolated demonstrated fault or a candidate whose original-speed benefit exceeds repeat variability without compromising stability, delay or torque visibility.

## Measurement scope

- Original reference and recording are unchanged. Half speed doubles only the active segment at 50 Hz, preserves each original pose at every second sample, and recomputes derivatives and body kinematics. Holds use neutral plus source frames 214, 538, 737 and 1174, smooth transitions and five-second plateaus. Existing validation and neutral entry/exit requirements pass unchanged.
- Analysis stops at the first final reference frame, excluding indefinite post-playback idle. Earlier approximately 6.33-degree figures used a different trace window; any difference from those figures is not claimed as a controller improvement. Original/half comparisons below use this same window definition and matched active source poses.
- Full-trace per-joint RMSE/peak/signed offsets and requested/applied effort are available in each trial analysis. The reference's joint permutation round-trips exactly and reference-model FK reproduces its body CSV exactly. Quaternions use WXYZ in artifacts and are converted explicitly for SciPy; normalized quaternions and derivative consistency are tested. The deployed contract names the matching joints and axis signs. These checks find no demonstrated mapping defect; they do not independently prove physical asset equivalence.
- Wrist/ankle errors use one kinematic model for both reference and actual states. Normalize by 1.288063 m, the model's neutral geometry height. Wrist-center position does not measure hand orientation; low wrist-position error can coexist with large wrist-pitch error.
- Raw world and pelvis-relative errors are reported separately. Supplemental initial-frame yaw/XY alignment is not a fitted trajectory alignment and is not the exact unlogged SONIC heading buffer.
- Metrics sample the 50 Hz trace and can miss extrema; safety and motor actuation remain at 200 Hz. Applied effort is explicitly clamped, so a bounded ratio is not independent proof of better tracking. Requested effort and clipping remain visible.
- The reference travels about 1.52 m. This is a retained diagnostic recording, not a stationary-standing qualification set. No claims about walking, live latency or hardware are made.

## Hold qualification

Final-two-second settling requires maximum joint speed <=0.9 rad/s, planar speed <=0.15 m/s, yaw rate <=0.2 rad/s, tilt <=0.6 rad and root height >=0.5 m. These are explicit analysis gates, not changes to execution safety. Also report the final-window portion before the next transition enters the 0.9-second lookahead. Failed settling observations remain visible and are excluded from settled-only summaries.

## Reproducibility and remaining work

Use `docs/pico-accuracy-runbook.md`, `docs/pico-accuracy-environment.json`, the compact result JSON and trial ledger. Detailed analyses and large native traces remain outside Git in `../motion_exchange/diagnostics/pico/accuracy-v1/trials-v2` and `../motion_exchange/executions`. The initial harness startup attempt was interrupted before playback because it misunderstood the supervisor's intentional fresh-session handover; this is preserved separately and is not a tracking trial failure.

The next justified diagnostic is to isolate the coupled high-error poses with a longer final plateau that has no upcoming transition, record the exact applied heading/contact state, and audit reference/deployed joint geometry and limits. Test one demonstrated mapping/conditioning/controller correction at a time. If conditioning becomes necessary, report deviation from the unchanged original reference as well as robot-to-conditioned-target error. The real-time speed target, recorded recovery qualification and live two-minute sessions remain outstanding; no live controller or hardware deployment is qualified by these results.

## Nine-trial numerical results

Values are mean [minimum, maximum] across three runs; these ranges are not confidence intervals. All nine playbacks completed without a recorded execution safety stop. This does not mean every pose settled or that acceptance passed.

| Condition | Full-clip RMSE (deg) | Peak (deg) | Matched active RMSE (deg) | Simulation / wall time |
|---|---:|---:|---:|---:|
| pico-gmr-full-v1 | 6.291 [6.260, 6.311] | 39.917 [39.670, 40.061] | 6.327 [6.302, 6.347] | 0.639 [0.635, 0.644] |
| pico-accuracy-half-v1 | 6.127 [6.108, 6.137] | 40.289 [39.867, 40.616] | 6.173 [6.149, 6.190] | 0.637 [0.634, 0.644] |
| pico-accuracy-holds-v1 | 6.989 [6.977, 7.006] | 40.579 [40.026, 41.628] | 7.469 [7.454, 7.485] | 0.637 [0.633, 0.640] |

Half speed reduces matched active RMSE by 0.154 degrees (2.44%). This exceeds the observed original-speed RMSE range, but remains modest, does not resolve peak error, and requires doubling active-motion duration. It is not accepted as a real-time correction. Half-speed full-clip metrics also weight active motion more heavily; use matched-pose metrics for the fair comparison.

### Original-speed phase contribution

| Phase | RMSE (deg) | Peak (deg) | Share of total squared joint error |
|---|---:|---:|---:|
| neutral_start | 2.826 [2.798, 2.869] | 6.241 [6.199, 6.314] | 0.53% |
| entry | 8.138 [8.132, 8.143] | 39.375 [39.197, 39.498] | 13.23% |
| active | 6.327 [6.302, 6.347] | 39.917 [39.670, 40.061] | 79.82% |
| exit | 4.491 [4.322, 4.744] | 17.191 [15.893, 18.370] | 4.04% |
| neutral_end | 5.974 [5.882, 6.060] | 16.444 [15.828, 16.888] | 2.38% |

Active motion dominates integrated squared error (~80%); entry has the highest average RMSE. Fixing entry alone would not solve active tracking.

### Original-speed active endpoints

| Endpoint | Raw world RMSE (m) | Initial-aligned world (m) | Pelvis RMSE (m) | Raw world / height | Pelvis / height |
|---|---:|---:|---:|---:|---:|
| left_wrist_yaw_link | 0.5015 | 0.5510 | 0.0233 | 38.94% | 1.81% |
| right_wrist_yaw_link | 0.6001 | 0.5542 | 0.0254 | 46.59% | 1.97% |
| left_ankle_roll_link | 0.4787 | 0.5482 | 0.0531 | 37.17% | 4.12% |
| right_ankle_roll_link | 0.5350 | 0.5484 | 0.0587 | 41.54% | 4.56% |

### Pose-hold observations

All-runs columns deliberately include unsettled observations. Only the final column is a settled-only estimate.

| Hold | Final two-second RMSE (deg), all runs | Before lookahead (deg), all runs | Settled runs | Before lookahead (deg), settled only |
|---|---:|---:|---:|---:|
| neutral | 2.785 [2.768, 2.816] | 2.764 [2.734, 2.800] | 2/3 | 2.767 [2.734, 2.800] |
| pose_1 | 13.681 [13.668, 13.694] | 13.675 [13.666, 13.689] | 0/3 | unavailable |
| pose_2 | 4.702 [4.631, 4.780] | 4.695 [4.627, 4.776] | 1/3 | 4.776 [4.776, 4.776] |
| pose_3 | 6.886 [6.498, 7.198] | 6.896 [6.507, 7.215] | 0/3 | unavailable |
| pose_4 | 5.938 [5.724, 6.221] | 6.117 [5.990, 6.247] | 1/3 | 6.247 [6.247, 6.247] |

The largest-error pose never meets the settling gate; its approximately 13.68-degree error is repeatable during a constant reference, but cannot be labeled a demonstrated static bias. Yaw-rate failures are common. Pose 3 also has no settled repeat. Single settled observations for poses 2 and 4 are insufficient to establish repeatable static behavior.

### Lag and saturation

- pico-gmr-full-v1: global best simulation-time shifts [0, -20, 20] ms; unshifted active-interior RMSE [6.1245, 6.1743, 6.162]; retrospectively aligned [6.1245, 6.1657, 6.1563] degrees. These are not cross-machine latency measurements.
- pico-accuracy-half-v1: global best simulation-time shifts [-40, -40, 20] ms; unshifted active-interior RMSE [6.0455, 6.077, 6.0781]; retrospectively aligned [5.9949, 6.0174, 6.0606] degrees. These are not cross-machine latency measurements.
- pico-accuracy-holds-v1: global best simulation-time shifts [0, 60, 60] ms; unshifted active-interior RMSE [7.5329, 7.5099, 7.5466]; retrospectively aligned [7.5329, 7.5069, 7.5433] degrees. These are not cross-machine latency measurements.

Original-speed active samples show clipping in 0.935% on average (range 0.735–1.269%). Applied torque ratio peaks at 1.0. Half-speed and pose-hold traces contain no sampled requested/applied clipping, yet retain large errors. Saturation therefore does not explain all observed tracking error; sampled absence is not proof that no 200 Hz step saturated.

### Every joint, original-speed active phase

RMSE and signed bias are means across runs; peak is the maximum observed across all three. Requested/applied effort are maxima across all three active phases, not necessarily simultaneous.

| Joint | RMSE (deg) | Signed bias (deg) | Peak (deg) | Requested peak (Nm) | Applied peak (Nm) | Max clipped (Nm) |
|---|---:|---:|---:|---:|---:|---:|
| left_wrist_pitch_joint | 11.277 | 8.900 | 31.967 | 3.318 | 3.318 | 0.000 |
| waist_pitch_joint | 10.208 | -8.767 | 28.103 | 28.768 | 28.768 | 0.000 |
| right_elbow_joint | 9.321 | 5.140 | 40.061 | 8.610 | 8.610 | 0.000 |
| left_knee_joint | 8.697 | 2.101 | 36.028 | 102.215 | 102.215 | 0.000 |
| left_elbow_joint | 8.665 | 2.656 | 38.688 | 8.275 | 8.275 | 0.000 |
| right_wrist_pitch_joint | 8.522 | -1.204 | 29.929 | 3.919 | 3.919 | 0.000 |
| right_knee_joint | 8.431 | 2.967 | 27.923 | 91.428 | 91.428 | 0.000 |
| right_wrist_roll_joint | 7.995 | 5.094 | 29.163 | 1.221 | 1.221 | 0.000 |
| left_wrist_yaw_joint | 6.585 | 1.660 | 21.747 | 2.009 | 2.009 | 0.000 |
| left_wrist_roll_joint | 6.489 | -4.838 | 22.576 | 1.075 | 1.075 | 0.000 |
| right_ankle_pitch_joint | 6.460 | -4.136 | 26.187 | 58.499 | 50.000 | 8.499 |
| right_shoulder_yaw_joint | 6.399 | 4.742 | 16.759 | 4.746 | 4.746 | 0.000 |
| right_hip_yaw_joint | 6.187 | -1.566 | 20.544 | 32.788 | 32.788 | 0.000 |
| right_wrist_yaw_joint | 6.017 | -0.829 | 19.367 | 1.552 | 1.552 | 0.000 |
| left_hip_yaw_joint | 5.614 | -2.672 | 25.421 | 41.889 | 41.889 | 0.000 |
| right_ankle_roll_joint | 5.553 | -4.150 | 15.727 | 29.382 | 29.382 | 0.000 |
| left_ankle_pitch_joint | 5.250 | 0.314 | 30.058 | 59.274 | 50.000 | 9.274 |
| right_hip_pitch_joint | 5.128 | -3.178 | 17.229 | 50.497 | 50.497 | 0.000 |
| left_hip_pitch_joint | 4.557 | -2.151 | 14.798 | 49.529 | 49.529 | 0.000 |
| left_shoulder_yaw_joint | 4.522 | -3.087 | 13.865 | 5.552 | 5.552 | 0.000 |
| right_hip_roll_joint | 4.086 | 1.291 | 17.876 | 69.233 | 69.233 | 0.000 |
| left_shoulder_roll_joint | 3.536 | 1.131 | 13.534 | 16.509 | 16.509 | 0.000 |
| right_shoulder_pitch_joint | 3.447 | 0.845 | 12.085 | 11.172 | 11.172 | 0.000 |
| left_ankle_roll_joint | 3.300 | -0.215 | 13.002 | 15.836 | 15.836 | 0.000 |
| left_hip_roll_joint | 3.261 | 0.668 | 15.707 | 62.719 | 62.719 | 0.000 |
| left_shoulder_pitch_joint | 2.876 | 0.714 | 15.236 | 11.354 | 11.354 | 0.000 |
| right_shoulder_roll_joint | 2.503 | 0.038 | 8.835 | 13.221 | 13.221 | 0.000 |
| waist_yaw_joint | 2.136 | -0.150 | 7.796 | 16.464 | 16.464 | 0.000 |
| waist_roll_joint | 1.511 | 0.000 | 10.271 | 33.209 | 33.209 | 0.000 |

### Qualification outcome

- Joint RMSE <=6 degrees: **not achieved** at original speed. Peak <=30 degrees: **not achieved**.
- Simulation speed within 0.95–1.05 times wall time: **not achieved** (~0.639 original-speed mean).
- Four focused analysis tests pass, including exact resampling/derivatives, known FK translation, hold duration, and unsettled/clipped-effort visibility. Both derived artifacts passed existing validation.
- No controller correction promoted. No new distortion from conditioning introduced. Baseline retained; live headset and recovery qualification remain outstanding.

