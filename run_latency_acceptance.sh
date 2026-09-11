#!/usr/bin/env bash
set -euo pipefail

trap 'rc=$?; echo "HARNESS_ERROR rc=$rc line=$LINENO command=$BASH_COMMAND" >&2' ERR

workspace="${MOTION_WORKSPACE:-/home/adcs-public-robot/Documents/socialnav_humanoid_ws}"
runs="${WARM_RUNS:-3}"
dds_interface="${DDS_INTERFACE:-lo}"
motions=("$@")
if [ "${#motions[@]}" -eq 0 ]; then
  motions=(
    wave-official-e2e-005-083aa901
    bow-e2e-004-retargeted-95pct
    side-step-e2e-002-0aa18456
  )
fi

status_path="$workspace/motion_exchange/.runtime/isaac_status.json"
read_status_field() {
  /usr/bin/python3 -c "import json; print(json.load(open('$status_path')).get('$1', ''))" 2>/dev/null || true
}

for motion_id in "${motions[@]}"; do
  request_id="$(docker exec sonic-tracker python3 -c "import json; print(json.load(open('/motion_exchange/$motion_id/manifest.json'))['request_id'])")"
  motion_metrics=()
  for run_index in $(seq 1 "$runs"); do
    old_session="$(read_status_field session_id)"
    log="$workspace/motion_exchange/executions/acceptance-${motion_id}-${run_index}.log"
    docker exec -d isaac-lab-headless-latency sh -lc \
      "cd /socialnav_humanoid_ws/unitree_sim_isaaclab && DDS_DOMAIN=42 DDS_INTERFACE=$dds_interface SIM_LOWSTATE_TOPIC=rt/socialnav_sim/g1/lowstate SIM_LOWCMD_TOPIC=rt/socialnav_sim/g1/lowcmd /isaac-sim/python.sh run_motion_pipeline_sim.py --task Isaac-Flat-G129-SONIC-Official --asset-profile sonic_official_g1 --headless --exit-after-command --trace-hz 5 > /socialnav_humanoid_ws/motion_exchange/executions/acceptance-${motion_id}-${run_index}.log 2>&1"

    ready=false
    for _ in $(seq 1 120); do
      state="$(read_status_field state)"
      session="$(read_status_field session_id)"
      if [ "$state" = "READY" ] && [ -n "$session" ] && [ "$session" != "$old_session" ]; then
        ready=true
        break
      fi
      sleep 1
    done
    if [ "$ready" != true ]; then
      echo "FAILED motion=$motion_id run=$run_index reason=isaac_ready_timeout log=$log"
      exit 1
    fi
    ready_session="$session"

    # Some DDS CLI builds return a non-zero status after successfully publishing.
    # Treat the command as a dispatch attempt and verify delivery from the Isaac
    # session/motion state below instead of accepting the CLI exit code as proof.
    approval_cli_rc=0
    docker exec kimodo motion-cli --domain 42 --interface "$dds_interface" control \
      approve_execute "$request_id" "$motion_id" || approval_cli_rc=$?

    active=false
    for _ in $(seq 1 90); do
      state="$(read_status_field state)"
      session="$(read_status_field session_id)"
      active_motion="$(read_status_field motion_id)"
      if [ "$session" != "$ready_session" ]; then
        sleep 1
        continue
      fi
      if [ "$active_motion" = "$motion_id" ]; then
        case "$state" in
          SETTLING|GROUNDING|EXECUTING|COMPLETED|UNSAFE|FAILED|ABORTED)
            active=true
            break
            ;;
        esac
      fi
      sleep 1
    done
    if [ "$active" != true ]; then
      echo "FAILED motion=$motion_id run=$run_index reason=approval_not_received cli_rc=$approval_cli_rc log=$log"
      exit 1
    fi

    terminal=false
    for _ in $(seq 1 240); do
      state="$(read_status_field state)"
      status_session="$(read_status_field session_id)"
      status_motion="$(read_status_field motion_id)"
      if [ "$status_session" = "$ready_session" ] && [ "$status_motion" = "$motion_id" ]; then
        case "$state" in
          COMPLETED|UNSAFE|FAILED|ABORTED)
            terminal=true
            break
            ;;
        esac
      fi
      sleep 1
    done
    if [ "$terminal" != true ]; then
      echo "FAILED motion=$motion_id run=$run_index reason=execution_timeout log=$log"
      exit 1
    fi
    if [ "$state" != "COMPLETED" ]; then
      reason="$(read_status_field reason)"
      echo "FAILED motion=$motion_id run=$run_index state=$state reason=$reason log=$log"
      exit 1
    fi

    metrics="$(docker exec sonic-tracker python3 -c "import json; s=json.load(open('/motion_exchange/.runtime/isaac_status.json')); t=json.load(open('/motion_exchange/$motion_id/timing.json'))['stages']; p=s['performance']; print(json.dumps({'motion_id':'$motion_id','run':$run_index,'state':s['state'],'playback_rtf':p['unsupported_playback_realtime_factor'],'startup_s':p['isaac_runner_startup_s'],'settling_s':p['settling_wall_s'],'post_hold_s':t['post_hold']['duration_s'],'sonic_initialization_s':t['sonic_initialization']['duration_s'],'approval_to_playback_s':t['approval_to_playback']['duration_s'],'report_finalization_s':p['report_trace_finalization_s'],'report_path':s['report_path']}))")"
    echo "$metrics"
    motion_metrics+=("$metrics")
    # Functional acceptance requires every playback to meet realtime. Other
    # stage budgets are intentionally evaluated from the three-run median.
    docker exec sonic-tracker python3 -c "import json,sys; m=json.loads(sys.argv[1]); assert m['playback_rtf'] >= 0.8, m" "$metrics"
  done

  metrics_json="$(IFS=,; echo "[${motion_metrics[*]}]")"
  docker exec sonic-tracker python3 -c "import json,statistics,sys; rows=json.loads(sys.argv[1]); budgets={'startup_s':30.0,'settling_s':15.0,'post_hold_s':3.0,'sonic_initialization_s':20.0,'approval_to_playback_s':30.0,'report_finalization_s':5.0}; med={k:statistics.median(float(r[k]) for r in rows) for k in budgets}; slow={k:{'median_s':med[k],'budget_s':v} for k,v in budgets.items() if med[k]>v}; out={'motion_id':rows[0]['motion_id'],'warm_runs':len(rows),'median':med,'result':'SLOW' if slow else 'PASS','slow':slow}; print(json.dumps(out)); assert not slow, out" "$metrics_json"
done
