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

SMOKE_TIMEOUT="${SMOKE_TIMEOUT:-10}"
tmp_out="$(mktemp)"
tmp_err="$(mktemp)"
trap 'rm -f "${tmp_out}" "${tmp_err}"' EXIT

timeout "${SMOKE_TIMEOUT}" ros2 launch lvdot_bringup run_detector.launch.py rviz:=false >"${tmp_out}" 2>"${tmp_err}" || true

if grep -Eq 'Config loaded:|LV-DOT ROS2 skeleton started|process started with pid' "${tmp_out}" "${tmp_err}"; then
  echo "[PASS] detector_launch_smoke_test"
else
  echo "[FAIL] detector_launch_smoke_test"
  echo "--- stdout ---"
  sed -n '1,120p' "${tmp_out}"
  echo "--- stderr ---"
  sed -n '1,120p' "${tmp_err}"
  exit 1
fi
