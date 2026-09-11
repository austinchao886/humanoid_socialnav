# Services

This directory owns code that is specific to one runtime service or container.
Shared DDS contracts, artifact rules, validation, and robot compatibility
profiles live in `packages/motion_contracts`. Each service owns its installable
Python package below this directory.

- `motion_generator`: Kimodo generation service ownership and runtime notes.
- `sonic_tracker`: Gear SONIC supervisor and controller integration boundary.
- `isaac_runtime`: integration code used by the authoritative Isaac process.
- `isaac_visualizer`: non-authoritative visualization service boundary.
- `video_generator`: GEM/GMR adapter entry points used by the video worker.
