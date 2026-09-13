#!/usr/bin/env bash
set -euo pipefail
# Current abort is a hard cancellation; Isaac recovers in a new session.
request_id="${1:?Usage: abort_motion.sh request_id motion_id}"
motion_id="${2:?Usage: abort_motion.sh request_id motion_id}"
exec docker exec -it sonic-tracker \
  motion-cli --domain 42 --interface lo control abort "$request_id" "$motion_id"
