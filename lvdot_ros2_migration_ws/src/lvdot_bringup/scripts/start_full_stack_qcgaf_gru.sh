#!/usr/bin/env bash
set -euo pipefail

QCGAF_CKPT="${1:-${QCGAF_CHECKPOINT:-}}"
GRU_MODEL="${2:-${GRU_MODEL_PATH:-}}"
USE_SIM_TIME="${3:-${USE_SIM_TIME:-false}}"

if [[ -z "$QCGAF_CKPT" || -z "$GRU_MODEL" ]]; then
  echo "Usage: $0 <qcgaf_checkpoint.pt> <gru_model.pth> [use_sim_time:true|false]"
  echo "Or set env vars: QCGAF_CHECKPOINT and GRU_MODEL_PATH"
  exit 1
fi

if [[ ! -f "$QCGAF_CKPT" ]]; then
  echo "Missing QCGAF checkpoint: $QCGAF_CKPT"
  exit 1
fi
if [[ ! -f "$GRU_MODEL" ]]; then
  echo "Missing GRU model: $GRU_MODEL"
  exit 1
fi

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

exec ros2 launch lvdot_bringup run_full_stack_qcgaf_gru.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  qcgaf_checkpoint:="$QCGAF_CKPT" \
  gru_model:="$GRU_MODEL"
