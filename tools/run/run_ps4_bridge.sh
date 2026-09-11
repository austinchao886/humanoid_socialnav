#!/usr/bin/env bash
set -euo pipefail

docker exec -it sonic-tracker \
  ps4-joystick-bridge \
  --host 127.0.0.1 \
  --port "${PS4_BRIDGE_PORT:-16042}"
