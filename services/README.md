# Services

This directory owns code that is specific to one runtime service or container.
Shared DDS contracts, artifact rules, validation, and orchestration utilities
remain in the `motion_pipeline` Python package until the shared-package split.

- `isaac_runtime`: integration code used by the authoritative Isaac process.
- `video_generator`: GEM/GMR adapter entry points used by the video worker.

The `sonic-tracker` and generator modules still live in `motion_pipeline` for
entry-point compatibility. They will move behind compatibility imports in the
next refactor phase.
