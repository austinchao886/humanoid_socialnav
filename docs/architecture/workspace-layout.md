# Workspace layout

## Decision

Social Motion remains one integration repository. Runtime services have clear
source ownership inside `services/`, while cross-service contracts remain in a
shared Python package. Third-party projects remain pinned dependencies instead
of being copied into our source tree.

## Ownership

| Path | Owner and purpose |
| --- | --- |
| `motion_pipeline/` | Shared Python package, CLI, DDS/artifact contracts, and current service entry points |
| `services/` | Code owned by a specific container or runtime |
| `packages/` | Future independently testable shared packages |
| `deploy/` | Dockerfiles, runtime configuration, and Compose overrides |
| `tools/run/` | Human-friendly one-command launch and control scripts |
| `scripts/` | Bootstrap, test, diagnostics, and acceptance tooling pending the next split |
| `tests/` | Contract and unit tests |
| `vendor/` | Pinned third-party Git submodules |
| `runtime/` | Documentation boundary for untracked runtime state |

Compatibility symlinks preserve the former `docker/`, `config/`,
`isaac_adapter/`, `video_adapters/`, and `run_files/` paths during migration.
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
