#!/usr/bin/env bash
set -euo pipefail

QCGAF_CKPT="${1:-${QCGAF_CHECKPOINT:-}}"
GRU_MODEL="${2:-${GRU_MODEL_PATH:-}}"
USE_SIM_TIME="${3:-${USE_SIM_TIME:-false}}"
TIMEOUT_SEC="${4:-45}"

if [[ -z "$QCGAF_CKPT" || -z "$GRU_MODEL" ]]; then
  echo "Usage: $0 <qcgaf_checkpoint.pt> <gru_model.pth> [use_sim_time:true|false] [timeout_sec]"
  echo "Or set env vars: QCGAF_CHECKPOINT and GRU_MODEL_PATH"
  exit 1
fi

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

LOG_FILE="/tmp/lvdot_full_stack_smoke_$(date +%Y%m%d_%H%M%S).log"

ros2 launch lvdot_bringup run_full_stack_qcgaf_gru.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  qcgaf_checkpoint:="$QCGAF_CKPT" \
  gru_model:="$GRU_MODEL" >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" || true
    sleep 1
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep 10
set +e
timeout "${TIMEOUT_SEC}s" bash -lc "until /home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/check_qcgaf_gru_topics.sh; do sleep 2; done"
RC=$?
set -e

if [[ $RC -ne 0 ]]; then
  echo "[FAIL] Smoke test failed. Log: $LOG_FILE"
  exit 1
fi

echo "[PASS] Smoke test passed. Log: $LOG_FILE"
