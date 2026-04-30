#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${HOME}/ros2_depth_eval_ws"
INPUT_CSV="${1:-${WS_DIR}/artifacts/live_roi_depth_eval.csv}"
OUTPUT_CSV="${2:-${WS_DIR}/artifacts/live_roi_depth_eval_summary.csv}"
OUTPUT_MD="${3:-${WS_DIR}/artifacts/live_roi_depth_eval_summary.md}"

exec ros2 run depth_eval_tools summarize_live_roi_eval \
  "${INPUT_CSV}" \
  "${OUTPUT_CSV}" \
  --output-md "${OUTPUT_MD}"
