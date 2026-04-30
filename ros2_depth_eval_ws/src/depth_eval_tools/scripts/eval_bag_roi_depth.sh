#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${HOME}/ros2_depth_eval_ws"
BAG_DIR="${1:?bag dir required}"
GT_CSV="${2:-${WS_DIR}/artifacts/experiment_gt.csv}"
OUT_CSV="${3:-${WS_DIR}/artifacts/bag_roi_depth_eval.csv}"

exec ros2 run depth_eval_tools bag_roi_depth_eval \
  "${BAG_DIR}" \
  "${GT_CSV}" \
  --output-csv "${OUT_CSV}"
