#!/usr/bin/env bash
set -euo pipefail

source_setup() {
  local setup_path="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_path}"
  set -u
}

source_setup /opt/ros/jazzy/setup.bash
source_setup /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source_setup /home/skbt2/lvdot_ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUITE_SCRIPT="${SCRIPT_DIR}/run_real_scene_regression_suite.sh"

ARTIFACT_ROOT="/home/skbt2/lvdot_ros2_ws/artifacts"
RUNS_CSV="${ARTIFACT_ROOT}/a5_6_runs.csv"
SUMMARY_TXT="${ARTIFACT_ROOT}/a5_6_summary.txt"
SUMMARY_MD="${ARTIFACT_ROOT}/a5_6_summary.md"

printf 'run_id,artifact_dir,status,hit_count,validation,crash,log_path\n' > "${RUNS_CSV}"

run_status_from_log() {
  local log_file="$1"
  if grep -q '\[suite\] preflight failed' "${log_file}"; then
    printf 'preflight_fail'
  elif grep -q 'launch exited before detector became ready' "${log_file}"; then
    printf 'launch_fail'
  elif grep -q 'detector did not become ready within' "${log_file}"; then
    printf 'launch_fail'
  else
    printf 'runtime_crash'
  fi
}

for run_id in 1 2 3 4 5 6 7; do
  stamp="$(date +%Y%m%d_%H%M%S)"
  run_log="${ARTIFACT_ROOT}/a5_6_run_${run_id}_${stamp}.log"

  echo "[a5_6] run=${run_id} start log=${run_log}"
  set +e
  READY_TIMEOUT_SECONDS=900 \
  READY_POLL_SECONDS=5 \
  READY_MIN_UPTIME_SECONDS=20 \
  POST_READY_SAMPLE_SECONDS=60 \
  ENABLE_YOLO=true \
  LAUNCH_YOLO_NODE=true \
  USE_ALL_CLASSES=false \
  ENABLE_COLOR_FALLBACK=false \
  ENABLE_VIS_STAGE=false \
  EXECUTOR_THREADS=1 \
  YOLO_CONF_THRESHOLD=0.25 \
  YOLO_INFERENCE_HZ=10.0 \
  WINDOW_SECONDS=30 \
  SAMPLE_HZ=1 \
  HIT_MIN_COUNT=8 \
  COUNT_TOLERANCE=0 \
  SERVICE_QUERY_RANGE=120.0 \
  "${SUITE_SCRIPT}" > "${run_log}" 2>&1
  ec=$?
  set -e

  artifact_dir="$(sed -n 's/^\[suite\] artifact_dir=//p' "${run_log}" | tail -n1)"
  if [[ -z "${artifact_dir}" ]]; then
    artifact_dir="(missing_artifact_dir:${run_log})"
  fi
  summary_file="${artifact_dir}/summary.txt"

  status=""
  validation="FAIL"
  hit_count="0"
  crash="false"

  if [[ -f "${summary_file}" ]]; then
    status="$(sed -n 's/^execution_status=//p' "${summary_file}" | head -n1)"
    validation="$(sed -n 's/^validation=//p' "${summary_file}" | head -n1)"
    hit_count="$(sed -n 's/^window_hit_count=//p' "${summary_file}" | head -n1)"
  fi

  if [[ -z "${status}" ]]; then
    status="$(run_status_from_log "${run_log}")"
  fi
  if [[ -z "${validation}" ]]; then
    validation="FAIL"
  fi
  if [[ -z "${hit_count}" || ! "${hit_count}" =~ ^[0-9]+$ ]]; then
    hit_count="0"
  fi

  case "${status}" in
    launch_fail|runtime_crash) crash="true" ;;
    *) crash="false" ;;
  esac

  printf '%s,%s,%s,%s,%s,%s,%s\n' \
    "${run_id}" "${artifact_dir}" "${status}" "${hit_count}" "${validation}" "${crash}" "${run_log}" \
    >> "${RUNS_CSV}"

  echo "[a5_6] run=${run_id} done ec=${ec} status=${status} validation=${validation} hit=${hit_count} crash=${crash}"
done

python3 "${SCRIPT_DIR}/summarize_a5_6.py" \
  --runs-csv "${RUNS_CSV}" \
  --out-txt "${SUMMARY_TXT}" \
  --out-md "${SUMMARY_MD}"

echo "[a5_6] runs_csv=${RUNS_CSV}"
echo "[a5_6] summary_txt=${SUMMARY_TXT}"
echo "[a5_6] summary_md=${SUMMARY_MD}"
