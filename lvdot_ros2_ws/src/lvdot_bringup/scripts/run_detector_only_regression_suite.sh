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
source_setup /home/skbt2/lvdot_ros2_ws/install/setup.bash

WARMUP_SECONDS="${WARMUP_SECONDS:-6}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-/home/skbt2/lvdot_ros2_ws/artifacts}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${ARTIFACT_ROOT}/detector_only_regression_${STAMP}"
mkdir -p "${ARTIFACT_DIR}"

VALIDATION_RESULT="FAIL"
FAILURE_REASON="unknown"

is_environment_restricted() {
  grep -Eq 'getifaddrs: Operation not permitted|error in getifaddrs: Unknown error -1|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${ARTIFACT_DIR}/launch.log"
}

write_summary() {
  cat > "${ARTIFACT_DIR}/summary.txt" <<EOF
validation=${VALIDATION_RESULT}
failure_reason=${FAILURE_REASON}
artifact_dir=${ARTIFACT_DIR}
warmup_seconds=${WARMUP_SECONDS}
launch_log=${ARTIFACT_DIR}/launch.log
topics=${ARTIFACT_DIR}/topics.txt
services=${ARTIFACT_DIR}/services.txt
input_health_status=${ARTIFACT_DIR}/input_health_status.yaml
stage_timers_status=${ARTIFACT_DIR}/stage_timers_status.yaml
pipeline_stats_status=${ARTIFACT_DIR}/pipeline_stats_status.yaml
EOF
}

update_latest_link() {
  ln -sfn "${ARTIFACT_DIR}" "${ARTIFACT_ROOT}/latest_detector_only_regression"
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

update_latest_link

echo "[detector-suite] artifact_dir=${ARTIFACT_DIR}"
echo "[detector-suite] launching detector only"
ros2 launch lvdot_bringup run_detector.launch.py rviz:=false \
  > "${ARTIFACT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[detector-suite] waiting ${WARMUP_SECONDS}s for warmup"
sleep "${WARMUP_SECONDS}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  if is_environment_restricted; then
    FAILURE_REASON="dds_environment_restricted"
  else
    FAILURE_REASON="launch_exited_early"
  fi
  write_summary
  exit 1
fi

wait_for_detector_interfaces() {
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    timeout 5 ros2 topic list --no-daemon > "${ARTIFACT_DIR}/topics.txt" 2>&1 || true
    timeout 5 ros2 service list -t --no-daemon > "${ARTIFACT_DIR}/services.txt" 2>&1 || true
    if grep -Fqx '/onboard_detector/pipeline_stats_status' "${ARTIFACT_DIR}/topics.txt" \
      && grep -Fqx '/onboard_detector/get_dynamic_obstacles [lvdot_interfaces/srv/GetDynamicObstacles]' "${ARTIFACT_DIR}/services.txt"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if ! wait_for_detector_interfaces; then
  if is_environment_restricted; then
    FAILURE_REASON="dds_environment_restricted"
  else
    FAILURE_REASON="expected_topics_or_service_missing"
  fi
  write_summary
  exit 1
fi

timeout 8 ros2 topic echo /onboard_detector/input_health_status --once \
  > "${ARTIFACT_DIR}/input_health_status.yaml" 2>&1 || true
timeout 8 ros2 topic echo /onboard_detector/stage_timers_status --once \
  > "${ARTIFACT_DIR}/stage_timers_status.yaml" 2>&1 || true
timeout 8 ros2 topic echo /onboard_detector/pipeline_stats_status --once \
  > "${ARTIFACT_DIR}/pipeline_stats_status.yaml" 2>&1 || true

VALIDATION_RESULT="PASS"
FAILURE_REASON="none"

write_summary

echo "[detector-suite] validation=${VALIDATION_RESULT}"
echo "[detector-suite] summary=${ARTIFACT_DIR}/summary.txt"

if [[ "${VALIDATION_RESULT}" != "PASS" ]]; then
  exit 1
fi
