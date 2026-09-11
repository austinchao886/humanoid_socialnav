# Run commands

These small scripts are the supported human-facing shortcuts for starting and
controlling one service at a time. They do not assume a fixed checkout path.

```bash
./tools/run/run_kimodo.sh
./tools/run/run_sonic.sh
./tools/run/run_isaac.sh
./tools/run/run_isaac_vis.sh
./tools/run/listen_motion_status.sh
./tools/run/approve_motion.sh [request_id] [motion_id]
```

The former `run_files/` path is a compatibility symlink, so existing commands
continue to work during the migration.

For development source mounts, add the development override explicitly:

```bash
docker compose \
  -f docker-compose.motion.yml \
  -f deploy/compose.dev.yml \
  up -d sonic-tracker
```
