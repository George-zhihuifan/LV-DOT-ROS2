#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${HOME}/ros2_depth_eval_ws"
CONFIG_FILE="${WS_DIR}/src/depth_eval_tools/config/experiment_targets.yaml"
OUT_FILE="${1:-${WS_DIR}/artifacts/experiment_gt.csv}"

mkdir -p "$(dirname "${OUT_FILE}")"

exec ros2 run depth_eval_tools experiment_gt_export "${CONFIG_FILE}" "${OUT_FILE}"
