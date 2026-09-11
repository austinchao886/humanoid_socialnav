# Persistent interactive runtime roadmap

Status: planned; not implemented by this workspace refactor.

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

The retained isolated one-motion-per-session flow remains the regression and
qualification harness. See the Gear SONIC Notion page for the agreed staged
validation sequence.
