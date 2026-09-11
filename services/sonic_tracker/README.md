# SONIC tracker

The `sonic-tracker` service owns approval handling and Gear SONIC process
lifecycle. Its installed entry point is `sonic_tracker.supervisor:main`. Gear SONIC remains the only
component allowed to publish G1 LowCmd in the planned persistent runtime.

With `SONIC_INTERACTIVE_RUNTIME=1` and `SONIC_ENABLE_PLANNER=1`, the supervisor
waits for Isaac, starts SONIC from `SONIC_STANDING_MOTION_ID`, and enters native
Unitree joystick locomotion. Hold F2 while commanding motion with the left
stick. An approved DDS `approve_execute` command automatically preempts the
planner with its offline reference and restores joystick mode after the motion
returns safely to standing.
