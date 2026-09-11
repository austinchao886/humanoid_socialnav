# Persistent interactive runtime roadmap

Status: phase 1 implemented and simulator-accepted on 2026-09-11.

Gear SONIC will remain the only component that publishes G1 LowCmd. The runtime
will switch command/reference sources without repeatedly replacing the whole
body controller.

```text
STARTING
  -> SUPPORTED_BOOTSTRAP
  -> READY_STANDING
  -> LOCOMOTION | OFFLINE_REFERENCE | XR_LIVE_REFERENCE
  -> RETURN_TO_STAND
  -> READY_STANDING
```

Safety override has the highest arbitration priority, followed by a fresh Pico
live reference, approved offline reference, fresh joystick command with
deadman, and standing fallback. Stale input must never remain active.

Phase 1 separates a successful command from the simulator process lifecycle.
The first approved reference performs the supported SONIC handoff. After the
reference and monitored post-hold finish, Isaac acknowledges `COMPLETED`, then
returns to `READY_STANDING` in the same `session_id`; the SONIC process and its
policy/TensorRT state remain loaded. The LowCmd watchdog is phase-aware: active
motion uses a 0.5 s deadline and idle standing uses a bounded 1.0 s deadline to
tolerate host scheduling jitter. Any stale LowCmd beyond those limits, unsafe state, execution
failure, or abort enters the fail-safe path and starts a clean supported
session instead of automatically accepting another motion.

The next controller-lifecycle increment is a dedicated startup standing
reference so SONIC can take ownership before the first user motion. After that,
add a validated abort-to-standing transition, joystick/planner arbitration,
and finally the Pico 4 Ultra live-reference source with freshness/deadman
rules. An unvalidated mid-motion `R` reset is not considered a safe abort.

## Phase 1 acceptance evidence

- Ten consecutive `isaac-neutral-v1` executions completed in Isaac session
  `5940484287ff4d7a8976dde886ff58e4` without changing the Isaac or SONIC PID.
- `isaac-arm-raise-v1` then completed in that same session, demonstrating a
  preloaded offline-reference switch without another bootstrap handoff.
- All warm executions reported `settling_wall_s=0`; return to
  `READY_STANDING` took about 0.3--0.4 s.
- Five continuous minutes of unsupported `READY_STANDING` stayed within the
  monitored height/tilt envelope (about 0.765 m and 0.02 rad).
- Active abort stopped the controller session and recovered to a new supported
  `READY` session without automatically running another motion. Smooth
  abort-to-standing remains a later, separately qualified transition.
- Functional persistence passed. Playback realtime factor was approximately
  0.78--0.80, so the existing 0.8 performance target remains a separate
  optimization item rather than a lifecycle acceptance result.

The retained isolated one-motion-per-session flow remains the regression and
qualification harness. See the Gear SONIC Notion page for the agreed staged
validation sequence.
