#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if pgrep -f '[/]RoboticsServiceProcess' >/dev/null; then
  echo "XRoboToolKit PC service is already running" >&2
  exit 1
fi
DEST="$ROOT/runtime/pico/pc-service"
mkdir -p "$DEST"
if [[ ! -x "$DEST/opt/apps/roboticsservice/RoboticsServiceProcess" ]]; then
  dpkg-deb -x "$ROOT/vendor/GR00T-WholeBodyControl/decoupled_wbc/control/teleop/device/pico/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb" "$DEST"
fi
cd "$DEST/opt/apps/roboticsservice"
export LD_LIBRARY_PATH="$PWD:$PWD/lib:$PWD/SDK/x64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QT_PLUGIN_PATH="$PWD/plugins"
exec ./RoboticsServiceProcess
