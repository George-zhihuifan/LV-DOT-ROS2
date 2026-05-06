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

fail=0
WORLD_PATH="/home/skbt2/ros2_depth_eval_ws/install/depth_eval_bringup/share/depth_eval_bringup/worlds/pedestrian_prototype.sdf"
SMOKE_TEST_TIMEOUT="${SMOKE_TEST_TIMEOUT:-3}"
ROS2_CLI_RETRIES="${ROS2_CLI_RETRIES:-3}"
ROS2_CLI_TIMEOUT="${ROS2_CLI_TIMEOUT:-5}"

check_command() {
  local name="$1"
  if command -v "${name}" >/dev/null 2>&1; then
    echo "[PASS] command: ${name}"
  else
    echo "[FAIL] command: ${name}"
    fail=1
  fi
}

check_file() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    echo "[PASS] file: ${path}"
  else
    echo "[FAIL] file: ${path}"
    fail=1
  fi
}

check_command ros2
check_command gz

check_file /home/skbt2/ros2_depth_eval_ws/install/setup.bash
check_file /home/skbt2/lvdot_ros2_ws/install/setup.bash
check_file "${WORLD_PATH}"

tmp_out="$(mktemp)"
tmp_err="$(mktemp)"
trap 'rm -f "${tmp_out}" "${tmp_err}"' EXIT

ros2_cli_ok=0
for attempt in $(seq 1 "${ROS2_CLI_RETRIES}"); do
  : > "${tmp_out}"
  : > "${tmp_err}"
  if timeout "${ROS2_CLI_TIMEOUT}" ros2 topic list --no-daemon >"${tmp_out}" 2>"${tmp_err}"; then
    echo "[PASS] ros2_cli_runtime (attempt ${attempt}/${ROS2_CLI_RETRIES})"
    ros2_cli_ok=1
    break
  fi
  sleep 1
done

if (( ros2_cli_ok == 0 )); then
  if grep -Eq 'PermissionError|Operation not permitted|getifaddrs|TRANSPORT_UDP' "${tmp_err}"; then
    echo "[FAIL] ros2_cli_runtime: runtime environment blocks DDS/network setup"
  else
    err_text="$(tr '\n' ' ' < "${tmp_err}")"
    if [[ -z "${err_text}" ]]; then
      err_text="timeout_or_empty_error_after_${ROS2_CLI_RETRIES}_attempts"
    fi
    echo "[FAIL] ros2_cli_runtime: ${err_text}"
  fi
  fail=1
fi

if /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/smoke_test_detector_only.sh; then
  :
else
  fail=1
fi

gz_out="$(mktemp)"
gz_err="$(mktemp)"
trap 'rm -f "${tmp_out}" "${tmp_err}" "${gz_out}" "${gz_err}"' EXIT

timeout "${SMOKE_TEST_TIMEOUT}" gz sim -s -r "${WORLD_PATH}" >"${gz_out}" 2>"${gz_err}" || true
if grep -Eq 'getifaddrs: Operation not permitted|error in getifaddrs: Unknown error -1|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${gz_err}"; then
  echo "[FAIL] gazebo_smoke_test: runtime environment blocks Gazebo/DDS startup"
  fail=1
else
  echo "[PASS] gazebo_smoke_test"
fi

if (( fail == 0 )); then
  echo "=== RESULT: PASS ==="
else
  echo "=== RESULT: FAIL ==="
  exit 1
fi
