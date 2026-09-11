#!/usr/bin/env bash
set -uo pipefail

workspace="${MOTION_WORKSPACE:-/socialnav_humanoid_ws}"
task="${ISAAC_RUNNER_TASK:-Isaac-Flat-G129-Deployment-V1}"
asset_profile="${ISAAC_RUNNER_ASSET_PROFILE:-g1_deployment_v1}"
dds_domain="${DDS_DOMAIN:-42}"
dds_interface="${DDS_INTERFACE:-lo}"
trace_hz="${ISAAC_RUNNER_TRACE_HZ:-5}"
command_timeout_s="${ISAAC_RUNNER_COMMAND_TIMEOUT_S:-0.5}"
standing_command_timeout_s="${ISAAC_RUNNER_STANDING_COMMAND_TIMEOUT_S:-1.0}"
restart_delay_s="${ISAAC_RUNNER_RESTART_DELAY_S:-1}"
runtime_mode="${ISAAC_RUNNER_MODE:-persistent}"
visual_state_output="${ISAAC_RUNNER_VISUAL_STATE_OUTPUT:-$workspace/motion_exchange/.runtime/g1_visual_state.bin}"
execution_dir="$workspace/motion_exchange/executions"

case "$runtime_mode" in
  persistent)
    lifecycle_args=()
    ;;
  isolated)
    lifecycle_args=(--exit-after-command)
    ;;
  *)
    echo "[isaac-runner-service] invalid ISAAC_RUNNER_MODE=$runtime_mode (expected persistent or isolated)" >&2
    exit 2
    ;;
esac

mkdir -p "$execution_dir"
generation=0

while true; do
  generation=$((generation + 1))
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  log_path="$execution_dir/persistent-runner-${stamp}-${generation}.log"
  echo "[isaac-runner-service] starting generation=$generation mode=$runtime_mode task=$task asset_profile=$asset_profile log=$log_path"

  DDS_DOMAIN="$dds_domain" \
  DDS_INTERFACE="$dds_interface" \
  SIM_LOWSTATE_TOPIC="${SIM_LOWSTATE_TOPIC:-rt/socialnav_sim/g1/lowstate}" \
  SIM_LOWCMD_TOPIC="${SIM_LOWCMD_TOPIC:-rt/socialnav_sim/g1/lowcmd}" \
  /isaac-sim/python.sh "$workspace/unitree_sim_isaaclab/run_motion_pipeline_sim.py" \
    --task "$task" \
    --asset-profile "$asset_profile" \
    --headless \
    "${lifecycle_args[@]}" \
    --sonic-command-timeout "$command_timeout_s" \
    --standing-command-timeout "$standing_command_timeout_s" \
    --trace-hz "$trace_hz" \
    --visual-state-output "$visual_state_output" \
    >"$log_path" 2>&1
  runner_rc=$?

  echo "[isaac-runner-service] generation=$generation exited rc=$runner_rc; starting a clean session in ${restart_delay_s}s"
  sleep "$restart_delay_s"
done
