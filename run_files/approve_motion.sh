#!/usr/bin/env bash
set -euo pipefail

request_id="isaac-arm-raise-v1"
motion_id="isaac-arm-raise-v1"

docker exec -it sonic-tracker \
  motion-cli \
  --domain 42 \
  --interface lo \
  control approve_execute \
  "$request_id" \
  "$motion_id"
