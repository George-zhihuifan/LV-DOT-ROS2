#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${HOME}/ros2_depth_eval_ws"
OUT_CSV="${1:-${WS_DIR}/artifacts/static_depth_validity_eval.csv}"
SUMMARY_CSV="${2:-${WS_DIR}/artifacts/static_depth_validity_eval_summary.csv}"
DURATION_SEC="${3:-10.0}"
SAMPLE_STRIDE="${4:-10}"

exec ros2 run depth_eval_tools static_depth_validity_eval \
  --ros-args \
  -p output_csv:="${OUT_CSV}" \
  -p summary_csv:="${SUMMARY_CSV}" \
  -p duration_sec:="${DURATION_SEC}" \
  -p sample_stride:="${SAMPLE_STRIDE}"
