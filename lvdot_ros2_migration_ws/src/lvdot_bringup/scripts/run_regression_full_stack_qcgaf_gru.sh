#!/usr/bin/env bash
set -euo pipefail

QCGAF_CKPT="${1:-${QCGAF_CHECKPOINT:-}}"
GRU_MODEL="${2:-${GRU_MODEL_PATH:-}}"
DURATION_SEC="${3:-300}"
USE_SIM_TIME="${4:-${USE_SIM_TIME:-false}}"

if [[ -z "$QCGAF_CKPT" || -z "$GRU_MODEL" ]]; then
  echo "Usage: $0 <qcgaf_checkpoint.pt> <gru_model.pth> [duration_sec] [use_sim_time:true|false]"
  echo "Or set env vars: QCGAF_CHECKPOINT and GRU_MODEL_PATH"
  exit 1
fi

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
REPORT_DIR="$WS_DIR/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPORT_DIR/full_stack_run_$STAMP"
mkdir -p "$RUN_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

LAUNCH_LOG="$RUN_DIR/launch.log"
TOPIC_HZ_LOG="$RUN_DIR/topic_hz.log"
SUMMARY_MD="$RUN_DIR/summary.md"

ros2 launch lvdot_bringup run_full_stack_qcgaf_gru.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  qcgaf_checkpoint:="$QCGAF_CKPT" \
  gru_model:="$GRU_MODEL" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" || true
    sleep 1
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep 12
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/check_qcgaf_gru_topics.sh | tee "$RUN_DIR/topic_presence.txt"

{
  echo "=== /onboard_detector/dynamic_bboxes ==="
  timeout 20s ros2 topic hz /onboard_detector/dynamic_bboxes || true
  echo "=== /qcgaf/fused_bboxes ==="
  timeout 20s ros2 topic hz /qcgaf/fused_bboxes || true
  echo "=== /gru_predictor/predicted_positions ==="
  timeout 20s ros2 topic hz /gru_predictor/predicted_positions || true
} | tee "$TOPIC_HZ_LOG"

sleep "$DURATION_SEC"

cat > "$SUMMARY_MD" <<EOM
# Full Stack Regression Summary

- Timestamp: $STAMP
- Duration (sec): $DURATION_SEC
- use_sim_time: $USE_SIM_TIME
- QCGAF checkpoint: $QCGAF_CKPT
- GRU model: $GRU_MODEL

## Artifacts

- launch log: $LAUNCH_LOG
- topic presence: $RUN_DIR/topic_presence.txt
- topic rates: $TOPIC_HZ_LOG

EOM

echo "Regression artifacts: $RUN_DIR"
