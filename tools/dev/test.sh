#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
KIMODO_G1_MJCF="${KIMODO_G1_MJCF:-$ROOT/vendor/kimodo/kimodo/assets/skeletons/g1skel34/xml/g1.xml}"

if [[ ! -f "$KIMODO_G1_MJCF" ]]; then
  echo "Kimodo G1 MJCF not found: $KIMODO_G1_MJCF" >&2
  echo "Run: git submodule update --init vendor/kimodo" >&2
  exit 2
fi

"$PYTHON_BIN" -m venv "$ROOT/.venv-test"
"$ROOT/.venv-test/bin/pip" install -q -r "$ROOT/requirements-dev.txt"
KIMODO_G1_MJCF="$KIMODO_G1_MJCF" \
  "$ROOT/.venv-test/bin/pytest" -q "$ROOT/tests/unit"
