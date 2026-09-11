# SONIC tracker

The `sonic-tracker` service owns approval handling and Gear SONIC process
lifecycle. Its installed entry point is `sonic_tracker.supervisor:main`. Gear SONIC remains the only
component allowed to publish G1 LowCmd in the planned persistent runtime.
