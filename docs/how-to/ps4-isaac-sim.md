# PS4 joystick in Isaac Sim

This simulation-only adapter lets a DualShock 4 connected to a developer Mac
drive Gear SONIC locomotion on the remote Isaac Sim host. Isaac remains the
only `LowState` publisher.

## Data path

```text
PS4 on Mac
  -> ps4_client.py (normalized controls, 50 Hz)
  -> SSH local-forwarded TCP 16042
  -> ps4-joystick-bridge in sonic-tracker
  -> rt/motion/joystick/cmd (DDS domain 42, loopback)
  -> Isaac runner freshness/sequence validation
  -> LowState.wireless_remote
  -> Gear SONIC native Gamepad input
```

L1 is the locomotion deadman, the left stick selects movement direction, the
right stick changes facing, and Options is the emergency stop. Releasing L1,
disconnecting the controller, stopping the client, or losing input for 200 ms
forces neutral input.

## Start

Start `isaac-runner`, `isaac-visualizer`, and `sonic-tracker` as usual. On the
server, from the motion pipeline repository, run:

```bash
tools/run/run_ps4_bridge.sh
```

On the Mac, open the SSH tunnel:

```bash
ssh -N -L 16042:127.0.0.1:16042 adcspublicrobot-codex
```

In another Mac terminal, from a checkout of this repository:

```bash
python3 -m pip install pygame
python3 tools/joystick/ps4_client.py
```

Keep L1 released until Isaac and SONIC both report `INTERACTIVE`. Hold L1 and
move the left stick gently for the first walking test. Release L1 before
approving a video motion. The supervisor preempts joystick mode, plays the
approved reference, flushes to neutral standing, checks stability, and then
returns the same session to `INTERACTIVE`.

## Safety boundary

The bridge binds to remote loopback only and DDS is also restricted to `lo`.
It cannot publish the simulator's physical state or command topics. A malformed,
out-of-order, or stale command is rejected or converted to a zeroed Unitree
remote packet. This is a development adapter, not the eventual hardware input
transport.
