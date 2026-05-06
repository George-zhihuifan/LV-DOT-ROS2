#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_ROOT="${ARTIFACT_ROOT:-/home/skbt2/lvdot_ros2_ws/artifacts}"
LATEST_LINK="${ARTIFACT_ROOT}/latest_scene_environment_diag"
REPORT_PATH="${LATEST_LINK}/report.txt"

if [[ ! -L "${LATEST_LINK}" && ! -d "${LATEST_LINK}" ]]; then
  echo "[FAIL] latest scene diagnostics artifact not found: ${LATEST_LINK}"
  exit 1
fi

if [[ ! -f "${REPORT_PATH}" ]]; then
  echo "[FAIL] latest scene diagnostics report not found: ${REPORT_PATH}"
  exit 1
fi

cat "${REPORT_PATH}"
