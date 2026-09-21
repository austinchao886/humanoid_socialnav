#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$ROOT/runtime/pico"
exec docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" \
  -v "$ROOT/tools/analysis/pico_tracking_probe.py:/probe.py:ro" \
  -v "$ROOT/runtime/pico:/results" \
  social-motion/pico:15abe6d python3 /probe.py \
  --output "/results/probe-$(date -u +%Y%m%dT%H%M%SZ)" "$@"
