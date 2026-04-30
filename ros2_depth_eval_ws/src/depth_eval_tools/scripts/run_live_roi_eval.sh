#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${HOME}/ros2_depth_eval_ws"
GT_CSV="${1:-${WS_DIR}/artifacts/experiment_gt.csv}"
OUT_CSV="${2:-${WS_DIR}/artifacts/live_roi_depth_eval.csv}"

exec ros2 run depth_eval_tools live_roi_depth_eval \
  --ros-args \
  -p gt_csv:="${GT_CSV}" \
  -p output_csv:="${OUT_CSV}"
