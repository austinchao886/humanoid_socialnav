#!/usr/bin/env bash
# Convert a saved PICO recording into a validated G1 artifact; does not execute it.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOTION_ID="${1:?Usage: run_pico_offline_retarget.sh pico-gmr-NEW [duration_seconds]}"
DURATION="${2:-30}"
cd "$ROOT"
docker run --rm --network none \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 -e MUJOCO_GL=egl \
  -v "$ROOT:/workspace:ro" -v "$ROOT/../motion_exchange:/motion_exchange" \
  -v "$ROOT/../motion_models:/models:ro" --entrypoint python \
  social-motion/video-gem-gmr:16bebf4-bb1bbe4-workspace-v2 \
  /workspace/tools/analysis/retarget_pico_recording.py \
  --duration "$DURATION" --motion-id "$MOTION_ID"
