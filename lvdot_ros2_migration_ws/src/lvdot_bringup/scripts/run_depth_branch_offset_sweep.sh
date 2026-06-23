#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${WS_ROOT:-/home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_ABLATION="${WS_ROOT}/src/lvdot_bringup/scripts/run_ablation.sh"
OUT_ROOT="${OUT_ROOT:-${WS_ROOT}/logs/depth_branch_offset_sweep_$(date +%Y%m%d_%H%M%S)}"
OFFSETS_CSV="${OFFSETS_CSV:-0.05,0.10,0.15,0.20,0.25,0.30}"
SCENARIOS_FILTER="${SCENARIOS_FILTER:-dense_open}"
CONFIGS_FILTER="${CONFIGS_FILTER:-A3_lvdot_baseline}"
NUM_RUNS="${NUM_RUNS:-1}"
EVAL_DURATION_SEC="${EVAL_DURATION_SEC:-20.0}"
WARMUP_SEC="${WARMUP_SEC:-8.0}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
RVIZ="${RVIZ:-false}"

mkdir -p "${OUT_ROOT}"

IFS=',' read -r -a OFFSETS <<< "${OFFSETS_CSV}"

for offset in "${OFFSETS[@]}"; do
  offset_tag="$(printf '%s' "${offset}" | tr '.' 'p')"
  run_dir="${OUT_ROOT}/offset_${offset_tag}"
  mkdir -p "${run_dir}"
  echo "[offset-sweep] start offset=${offset} -> ${run_dir}"

  QCGAF_EXTRA_ARGS="depth_branch_offset_sec:=${offset}" \
  OUT_ROOT="${run_dir}" \
  CONFIGS_FILTER="${CONFIGS_FILTER}" \
  SCENARIOS_FILTER="${SCENARIOS_FILTER}" \
  NUM_RUNS="${NUM_RUNS}" \
  EVAL_DURATION_SEC="${EVAL_DURATION_SEC}" \
  WARMUP_SEC="${WARMUP_SEC}" \
  GAZEBO_GUI="${GAZEBO_GUI}" \
  RVIZ="${RVIZ}" \
  "${RUN_ABLATION}"

  echo "[offset-sweep] done offset=${offset}"
done

echo "[offset-sweep] all runs finished under ${OUT_ROOT}"
