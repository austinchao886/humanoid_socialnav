# Persistent interactive runtime roadmap

Status: phase 1 implemented; simulator acceptance pending.

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
policy/TensorRT state remain loaded. Any stale LowCmd, unsafe state, execution
failure, or abort enters the fail-safe path and starts a clean supported
session instead of automatically accepting another motion.

The next controller-lifecycle increment is a dedicated startup standing
reference so SONIC can take ownership before the first user motion. After that,
add a validated abort-to-standing transition, joystick/planner arbitration,
and finally the Pico 4 Ultra live-reference source with freshness/deadman
rules. An unvalidated mid-motion `R` reset is not considered a safe abort.

The retained isolated one-motion-per-session flow remains the regression and
qualification harness. See the Gear SONIC Notion page for the agreed staged
validation sequence.
