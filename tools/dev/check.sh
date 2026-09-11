#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

"$ROOT/tools/dev/test.sh"

while IFS= read -r script; do
  bash -n "$script"
done < <(find "$ROOT/tools" -type f -name '*.sh' -print | sort)

"${PYTHON_BIN:-python3}" -m compileall -q \
  "$ROOT/motion_pipeline" \
  "$ROOT/packages" \
  "$ROOT/services" \
  "$ROOT/tests"

docker compose --profile '*' -f docker-compose.motion.yml config --quiet
docker compose --profile '*' \
  -f docker-compose.motion.yml \
  -f deploy/compose.dev.yml \
  config --quiet

echo "workspace checks passed"
