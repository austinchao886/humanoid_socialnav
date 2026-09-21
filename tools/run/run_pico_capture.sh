#!/usr/bin/env bash
# Isolated reference capture; port 5566 has no production SONIC subscriber.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$ROOT/runtime/pico"
exec docker run --rm --init --network host --ipc host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -v "$ROOT/vendor/GR00T-WholeBodyControl:/sonic:ro" \
  -v "$ROOT/runtime/pico:/results" \
  social-motion/pico:15abe6d python3 gear_sonic/scripts/pico_manager_thread_server.py \
  --input-source xrt --port 5566 --target_fps 50 \
  --record_dir "/results/capture-$(date -u +%Y%m%dT%H%M%SZ)" --record_format npz
