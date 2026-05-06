#!/usr/bin/env bash
set -euo pipefail

QCGAF_CKPT="${1:-${QCGAF_CHECKPOINT:-}}"
GRU_MODEL="${2:-${GRU_MODEL_PATH:-}}"
DURATION_SEC="${3:-120}"
USE_SIM_TIME="${4:-${USE_SIM_TIME:-false}}"
STRICT_TRAFFIC="${5:-false}"
WITH_SCENE="${6:-false}"
READINESS_TIMEOUT_SEC="${7:-90}"
MIN_HZ_DYNAMIC="${8:-8.0}"
MIN_HZ_FUSED="${9:-8.0}"
MIN_HZ_PRED="${10:-8.0}"

if [[ -z "$QCGAF_CKPT" || -z "$GRU_MODEL" ]]; then
  echo "Usage: $0 <qcgaf_checkpoint.pt> <gru_model.pth> [duration_sec] [use_sim_time] [strict_traffic] [with_scene] [readiness_timeout_sec] [min_hz_dynamic] [min_hz_fused] [min_hz_pred]"
  exit 1
fi

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
REPORT_ROOT="$WS_DIR/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPORT_ROOT/dod_validation_$STAMP"
mkdir -p "$RUN_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

LAUNCH_LOG="$RUN_DIR/launch.log"
RESULT_MD="$RUN_DIR/result.md"
TOPIC_HZ_LOG="$RUN_DIR/topic_hz.txt"
NODE_LIST_LOG="$RUN_DIR/node_list.txt"

pkill -9 -f 'run_detector_with_scene.launch.py|run_full_stack_qcgaf_gru.launch.py|qcgaf_fusion_node|gru_prediction_node|lvdot_detector_main|uav_pedestrian_prototype.launch.py|gz sim|parameter_bridge|lvdot_yolo_node' >/dev/null 2>&1 || true
sleep 1

if [[ "$WITH_SCENE" == "true" ]]; then
  ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false rviz:=false detector_rviz:=false enable_uav_controller:=false \
    launch_pose_stub:=true enable_yolo:=true launch_yolo_node:=true fusion_mode:=dual \
    enable_stage_timers:=true enable_vis_stage:=true executor_threads:=4 >"$LAUNCH_LOG" 2>&1 &
  STACK_PID=$!

  sleep 12
  ros2 run qcgaf_fusion fusion_node --ros-args \
    -p config:=/home/skbt2/lvdot_ros2_migration_ws/install/qcgaf_fusion/share/qcgaf_fusion/config/config.yaml \
    -p checkpoint:="$QCGAF_CKPT" -p verbose:=false -p debug_metrics:=false >>"$LAUNCH_LOG" 2>&1 &
  QCGAF_PID=$!

  ros2 run gru_predictor predict_node --ros-args \
    -p config:=/home/skbt2/lvdot_ros2_migration_ws/install/gru_predictor/share/gru_predictor/config/config_tuned.yaml \
    -p model:="$GRU_MODEL" -p horizon:=5 -p device:=cpu >>"$LAUNCH_LOG" 2>&1 &
  GRU_PID=$!
else
  ros2 launch lvdot_bringup run_full_stack_qcgaf_gru.launch.py \
    use_sim_time:="$USE_SIM_TIME" qcgaf_checkpoint:="$QCGAF_CKPT" gru_model:="$GRU_MODEL" >"$LAUNCH_LOG" 2>&1 &
  STACK_PID=$!
  QCGAF_PID=""
  GRU_PID=""
fi

cleanup() {
  [[ -n "${GRU_PID}" ]] && kill "$GRU_PID" 2>/dev/null || true
  [[ -n "${QCGAF_PID}" ]] && kill "$QCGAF_PID" 2>/dev/null || true
  kill "$STACK_PID" 2>/dev/null || true
  sleep 1
  pkill -9 -f 'run_detector_with_scene.launch.py|run_full_stack_qcgaf_gru.launch.py|qcgaf_fusion_node|gru_prediction_node|lvdot_detector_main|uav_pedestrian_prototype.launch.py|gz sim|parameter_bridge|lvdot_yolo_node' >/dev/null 2>&1 || true
}
trap cleanup EXIT

PASS=true
TRAFFIC_OK=true

start_ts=$(date +%s)
ready=false
while true; do
  now_ts=$(date +%s)
  elapsed=$((now_ts - start_ts))

  topics_ok=true
  if ! /home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/check_qcgaf_gru_topics.sh >/dev/null 2>&1; then
    topics_ok=false
  fi

  nodes_ok=true
  nodes="$(ros2 node list || true)"
  for n in /lvdot_detector_node /qcgaf_fusion_node /gru_prediction_node; do
    if ! grep -qx "$n" <<< "$nodes"; then
      nodes_ok=false
      break
    fi
  done

  if [[ "$topics_ok" == true && "$nodes_ok" == true ]]; then
    ready=true
    break
  fi

  if (( elapsed >= READINESS_TIMEOUT_SEC )); then
    break
  fi
  sleep 2
done

/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/check_qcgaf_gru_topics.sh | tee "$RUN_DIR/topic_presence.txt" || PASS=false
ros2 node list | tee "$NODE_LIST_LOG"
for n in /lvdot_detector_node /qcgaf_fusion_node /gru_prediction_node; do
  if ! grep -qx "$n" "$NODE_LIST_LOG"; then
    echo "Missing node: $n" | tee -a "$RUN_DIR/node_missing.txt"
    PASS=false
  fi
done

if [[ "$ready" != true ]]; then
  PASS=false
fi

{
  echo "=== /onboard_detector/dynamic_bboxes ==="
  timeout 20s ros2 topic hz /onboard_detector/dynamic_bboxes || true
  echo "=== /qcgaf/fused_bboxes ==="
  timeout 20s ros2 topic hz /qcgaf/fused_bboxes || true
  echo "=== /gru_predictor/predicted_positions ==="
  timeout 20s ros2 topic hz /gru_predictor/predicted_positions || true
} | tee "$TOPIC_HZ_LOG"

extract_rate() {
  local section="$1"
  awk -v sec="$section" '
    $0 ~ "^=== "sec" ===$" {in_sec=1; next}
    /^=== / && in_sec {in_sec=0}
    in_sec && /average rate:/ {rate=$3}
    END {if (rate == "") print "0"; else print rate}
  ' "$TOPIC_HZ_LOG"
}

RATE_DYNAMIC="$(extract_rate '/onboard_detector/dynamic_bboxes')"
RATE_FUSED="$(extract_rate '/qcgaf/fused_bboxes')"
RATE_PRED="$(extract_rate '/gru_predictor/predicted_positions')"

if [[ "$RATE_DYNAMIC" == "0" || "$RATE_FUSED" == "0" || "$RATE_PRED" == "0" ]]; then
  TRAFFIC_OK=false
fi

if [[ "$STRICT_TRAFFIC" == "true" ]]; then
  awk -v a="$RATE_DYNAMIC" -v b="$MIN_HZ_DYNAMIC" 'BEGIN{exit !(a>=b)}' || PASS=false
  awk -v a="$RATE_FUSED" -v b="$MIN_HZ_FUSED" 'BEGIN{exit !(a>=b)}' || PASS=false
  awk -v a="$RATE_PRED" -v b="$MIN_HZ_PRED" 'BEGIN{exit !(a>=b)}' || PASS=false
fi

sleep "$DURATION_SEC"
if ! kill -0 "$STACK_PID" 2>/dev/null; then
  PASS=false
fi

VERDICT="PASS"
if [[ "$PASS" != true ]]; then
  VERDICT="FAIL"
elif [[ "$TRAFFIC_OK" != true ]]; then
  VERDICT="PASS_WITH_NO_TRAFFIC"
fi

cat > "$RESULT_MD" <<EOM
# DoD Validation Result

- Timestamp: $STAMP
- Duration(s): $DURATION_SEC
- use_sim_time: $USE_SIM_TIME
- strict_traffic: $STRICT_TRAFFIC
- with_scene: $WITH_SCENE
- readiness_timeout_sec: $READINESS_TIMEOUT_SEC
- min_hz_dynamic: $MIN_HZ_DYNAMIC
- min_hz_fused: $MIN_HZ_FUSED
- min_hz_pred: $MIN_HZ_PRED
- measured_hz_dynamic: $RATE_DYNAMIC
- measured_hz_fused: $RATE_FUSED
- measured_hz_pred: $RATE_PRED
- QCGAF checkpoint: $QCGAF_CKPT
- GRU model: $GRU_MODEL

## Verdict

- RESULT: $VERDICT

## Artifacts

- launch log: $LAUNCH_LOG
- topic presence: $RUN_DIR/topic_presence.txt
- node list: $NODE_LIST_LOG
- topic rates: $TOPIC_HZ_LOG

EOM

if [[ "$VERDICT" == "FAIL" ]]; then
  echo "DoD validation FAIL: $RUN_DIR"
  exit 1
fi

echo "DoD validation $VERDICT: $RUN_DIR"
exit 0
