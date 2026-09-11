#!/usr/bin/env bash
set -euo pipefail

docker exec -it sonic-tracker \
  motion-cli \
  --domain 42 \
  --interface lo \
  listen