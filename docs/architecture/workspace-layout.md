# Workspace layout

## Decision

Social Motion remains one integration repository. Runtime services have clear
source ownership inside `services/`, while cross-service contracts remain in a
shared Python package. Third-party projects remain pinned dependencies instead
of being copied into our source tree.

## Ownership

| Path | Owner and purpose |
| --- | --- |
| `motion_pipeline/` | Application CLI, shared core utilities, DDS adapter, and compatibility imports |
| `packages/motion_contracts/` | Stable DDS messages, artifacts, validation, and robot-asset contracts |
| `services/` | Independently owned installable packages for each container/runtime |
| `packages/` | Future independently testable shared packages |
| `deploy/` | Dockerfiles, runtime configuration, and Compose overrides |
| `tools/run/` | Human-friendly one-command launch and control scripts |
| `tools/` | Bootstrap, development, runtime, analysis, acceptance, and human-facing commands |
| `tests/` | Unit, integration, and simulator/hardware acceptance tests |
| `vendor/` | Pinned third-party Git submodules |
| `runtime/` | Documentation boundary for untracked runtime state |

Compatibility symlinks preserve the former `docker/`, `config/`, `scripts/`,
root acceptance commands, `video_adapters/`, and `run_files/`
paths during migration.
New code and documentation should use the canonical paths.

## Development container policy

Production and acceptance runs use the source baked into immutable images.
Development runs add `deploy/compose.dev.yml`, which bind-mounts a service's
complete Python source boundary:

```bash
docker compose \
  -f docker-compose.motion.yml \
  -f deploy/compose.dev.yml \
  up -d sonic-tracker
```

A Python source change still requires restarting the long-running service so
the process imports the new code. It does not require rebuilding the image.
Dependency, Dockerfile, C++ controller, or baked model changes require rebuild.

## Refactor constraints

1. Do not change Compose service names, CLI names, DDS topics, or JSON contracts.
2. Keep third-party source pinned by commit and record image digests separately.
3. Move one ownership boundary at a time and retain compatibility entry points.
4. Verify unit tests and `docker compose config` after every phase.
5. Do not combine workspace moves with persistent-runtime behavior changes.

## External workspaces

`unitree_sim_isaaclab` and `unitree_sdk2_python` remain pinned sibling
workspaces because Isaac Sim owns their runtime environment. `vendor/` contains
the GR00T and Kimodo submodules used as Docker build inputs. Compose host paths
can be overridden through `.env`; `.env.example` documents every supported
workspace path.

The pinned Unitree fork is the only source of truth for Isaac simulator and
action-provider code. This repository does not keep copied Isaac overlays.
