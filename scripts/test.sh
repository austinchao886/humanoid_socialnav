#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv "$ROOT/.venv-test"
"$ROOT/.venv-test/bin/pip" install -q -r "$ROOT/requirements-dev.txt"
"$ROOT/.venv-test/bin/pytest" -q "$ROOT/tests"
