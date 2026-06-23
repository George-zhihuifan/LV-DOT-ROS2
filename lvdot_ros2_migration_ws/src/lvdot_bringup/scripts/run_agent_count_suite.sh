#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${WS_ROOT:-/home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws}"
RUN_ABLATION="${WS_ROOT}/src/lvdot_bringup/scripts/run_ablation.sh"
SCENARIO_DIR="${SCENARIO_DIR:-${WS_ROOT}/src/lvdot_bringup/config/agent_count_scenarios}"
OUT_ROOT="${OUT_ROOT:-${WS_ROOT}/logs/agent_count_suite_$(date +%Y%m%d_%H%M%S)}"
MIN_AGENTS="${MIN_AGENTS:-1}"
MAX_AGENTS="${MAX_AGENTS:-6}"
CONFIGS_FILTER="${CONFIGS_FILTER:-A4_qcgaf}"
NUM_RUNS="${NUM_RUNS:-1}"
EVAL_DURATION_SEC="${EVAL_DURATION_SEC:-20.0}"
WARMUP_SEC="${WARMUP_SEC:-5.0}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
RVIZ="${RVIZ:-false}"

if [[ ! -d "${SCENARIO_DIR}" ]]; then
  echo "Scenario directory not found: ${SCENARIO_DIR}" >&2
  exit 1
fi

scenario_entries=()
for count in $(seq "${MIN_AGENTS}" "${MAX_AGENTS}"); do
  scenario_file="${SCENARIO_DIR}/pedestrian_dense_$(printf '%02d' "${count}")agents.yaml"
  if [[ ! -f "${scenario_file}" ]]; then
    echo "Missing scenario file: ${scenario_file}" >&2
    exit 1
  fi
  scenario_entries+=("agents_${count}:${scenario_file}")
done

scenario_override=""
for entry in "${scenario_entries[@]}"; do
  if [[ -n "${scenario_override}" ]]; then
    scenario_override+="|"
  fi
  scenario_override+="${entry}"
done

mkdir -p "${OUT_ROOT}"

SCENARIOS_OVERRIDE="${scenario_override}" \
OUT_ROOT="${OUT_ROOT}" \
CONFIGS_FILTER="${CONFIGS_FILTER}" \
NUM_RUNS="${NUM_RUNS}" \
EVAL_DURATION_SEC="${EVAL_DURATION_SEC}" \
WARMUP_SEC="${WARMUP_SEC}" \
GAZEBO_GUI="${GAZEBO_GUI}" \
RVIZ="${RVIZ}" \
"${RUN_ABLATION}"
