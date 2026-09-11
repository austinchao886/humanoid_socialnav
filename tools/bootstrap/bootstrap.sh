#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="$(cd "$ROOT/.." && pwd)"
cd "$ROOT"

git submodule update --init

test "$(git -C vendor/kimodo rev-parse HEAD)" = "1aece8c124d73d255ceff5086d983b844c9f4e94"
test "$(git -C vendor/GR00T-WholeBodyControl rev-parse HEAD)" = "60c561b65d1ce2bba59c4cc752a8ffefb51d4884"

ensure_checkout() {
  local path="$1"
  local repository="$2"
  local commit="$3"

  if [[ ! -d "$path/.git" ]]; then
    git clone "$repository" "$path"
  fi
  git -C "$path" fetch origin
  git -C "$path" checkout --detach "$commit"
  test "$(git -C "$path" rev-parse HEAD)" = "$commit"
}

# Kimodo's Docker build requires its editable Viser fork inside the Kimodo
# checkout, but upstream Kimodo does not track that directory as a submodule.
ensure_checkout \
  "$ROOT/vendor/kimodo/kimodo-viser" \
  "https://github.com/nv-tlabs/kimodo-viser.git" \
  "7c82ad8f8640bad9dff8ded5c5eee908eeb08f11"
ensure_checkout \
  "$WORKSPACE/unitree_sim_isaaclab" \
  "https://github.com/austinchao886/unitree_sim_isaaclab.git" \
  "28a9944ef7ab0d61242cffe454e2a2329fbd39fb"
ensure_checkout \
  "$WORKSPACE/unitree_sdk2_python" \
  "https://github.com/unitreerobotics/unitree_sdk2_python.git" \
  "65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5"

if ! command -v git-lfs >/dev/null; then
  mkdir -p .tools/git-lfs
  cd .tools/git-lfs
  apt download git-lfs
  dpkg-deb -x git-lfs_*.deb .
  export PATH="$PWD/usr/bin:$PATH"
  cd "$ROOT"
fi

git lfs install --local
git -C vendor/kimodo lfs pull
git -C vendor/GR00T-WholeBodyControl lfs pull
python3 vendor/GR00T-WholeBodyControl/download_from_hf.py
PIPELINE_COMMIT="$(git rev-parse HEAD)" \
  docker compose -f docker-compose.motion.yml build
