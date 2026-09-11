#!/usr/bin/env bash
set -euo pipefail

workspace="${MOTION_WORKSPACE:-/home/adcs-public-robot/Documents/socialnav_humanoid_ws}"
dds_interface="${DDS_INTERFACE:-lo}"
status_path="$workspace/motion_exchange/.runtime/isaac_status.json"
active_timeout_s="${ACTIVE_TIMEOUT_S:-90}"
terminal_timeout_s="${TERMINAL_TIMEOUT_S:-240}"
ready_timeout_s="${READY_TIMEOUT_S:-120}"
motions=("$@")
if [ "${#motions[@]}" -eq 0 ]; then
  motions=(
    wave-official-e2e-005-083aa901
    bow-e2e-004-retargeted-95pct
    side-step-e2e-002-0aa18456
  )
fi

read_status_field() {
  /usr/bin/python3 -c \
    "import json; print(json.load(open('$status_path')).get('$1', ''))" \
    2>/dev/null || true
}

wait_for_ready() {
  local previous_session="${1:-}"
  local attempts=$((ready_timeout_s * 5))
  for _ in $(seq 1 "$attempts"); do
    local state session profile
    state="$(read_status_field state)"
    session="$(read_status_field session_id)"
    profile="$(read_status_field asset_profile)"
    if [ "$state" = "READY" ] \
      && [ "$profile" = "g1_deployment_v1" ] \
      && [ -n "$session" ] \
      && [ "$session" != "$previous_session" ]; then
      printf '%s\n' "$session"
      return 0
    fi
    sleep 0.2
  done
  return 1
}

ready_session="$(wait_for_ready)" || {
  echo "FAILED reason=initial_ready_timeout" >&2
  exit 1
}

for motion_id in "${motions[@]}"; do
  request_id="$(/usr/bin/python3 -c \
    "import json; print(json.load(open('$workspace/motion_exchange/$motion_id/manifest.json'))['request_id'])")"
  approval_epoch_s="$(/usr/bin/python3 -c 'import time; print(time.time())')"
  docker exec kimodo motion-cli --domain 42 --interface "$dds_interface" control \
    approve_execute "$request_id" "$motion_id" >/dev/null || true

  active=false
  for _ in $(seq 1 $((active_timeout_s * 5))); do
    state="$(read_status_field state)"
    session="$(read_status_field session_id)"
    active_motion="$(read_status_field motion_id)"
    if [ "$session" = "$ready_session" ] \
      && [ "$active_motion" = "$motion_id" ]; then
      case "$state" in
        SETTLING|GROUNDING|EXECUTING|POST_HOLD_COMPLETE|COMPLETED|UNSAFE|FAILED|ABORTED)
          active=true
          break
          ;;
      esac
    fi
    sleep 0.2
  done
  if [ "$active" != true ]; then
    echo "FAILED motion=$motion_id reason=approval_not_received session=$ready_session" >&2
    exit 1
  fi

  terminal=false
  for _ in $(seq 1 $((terminal_timeout_s * 5))); do
    state="$(read_status_field state)"
    session="$(read_status_field session_id)"
    active_motion="$(read_status_field motion_id)"
    if [ "$session" = "$ready_session" ] \
      && [ "$active_motion" = "$motion_id" ]; then
      case "$state" in
        COMPLETED|UNSAFE|FAILED|ABORTED)
          terminal=true
          break
          ;;
      esac
    fi
    sleep 0.2
  done
  terminal_epoch_s="$(/usr/bin/python3 -c 'import time; print(time.time())')"
  if [ "$terminal" != true ] || [ "$state" != "COMPLETED" ]; then
    reason="$(read_status_field reason)"
    echo "FAILED motion=$motion_id state=$state reason=$reason session=$ready_session" >&2
    exit 1
  fi

  report_path="$(read_status_field report_path)"
  report_host_path="$workspace/motion_exchange${report_path#/motion_exchange}"
  metrics="$(/usr/bin/python3 - "$report_host_path" "$approval_epoch_s" "$terminal_epoch_s" "$ready_session" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1]))
perf = report["performance"]
physics = perf["phase_latency"]["physics_step"]
print(json.dumps({
    "motion_id": report["motion_id"],
    "session_id": sys.argv[4],
    "result": report["result"],
    "approval_to_terminal_wall_s": float(sys.argv[3]) - float(sys.argv[2]),
    "runner_startup_s_excluded_from_approval": perf["isaac_runner_startup_s"],
    "settling_wall_s": perf["settling_wall_s"],
    "playback_realtime_factor": perf["unsupported_playback_realtime_factor"],
    "physics_step_p50_ms": physics["p50_ms"],
    "physics_step_p95_ms": physics["p95_ms"],
    "lowcmd_age_p95_ms": perf["lowcmd"]["lowcmd_age_p95_ms"],
    "report_path": report["trace_path"].replace(".jsonl", ".json"),
}, sort_keys=True))
PY
)"

  next_ready_started="$(/usr/bin/python3 -c 'import time; print(time.time())')"
  next_session="$(wait_for_ready "$ready_session")" || {
    echo "FAILED motion=$motion_id reason=next_ready_timeout" >&2
    exit 1
  }
  next_ready_epoch_s="$(/usr/bin/python3 -c 'import time; print(time.time())')"
  /usr/bin/python3 - "$metrics" "$next_ready_started" "$next_ready_epoch_s" "$next_session" <<'PY'
import json
import sys

metrics = json.loads(sys.argv[1])
metrics["terminal_to_next_ready_wall_s"] = float(sys.argv[3]) - float(sys.argv[2])
metrics["next_session_id"] = sys.argv[4]
print(json.dumps(metrics, sort_keys=True))
PY
  ready_session="$next_session"
done
