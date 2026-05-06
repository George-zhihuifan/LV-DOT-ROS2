#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_ROOT="${ARTIFACT_ROOT:-/home/skbt2/lvdot_ros2_ws/artifacts}"
LATEST_LINK="${ARTIFACT_ROOT}/latest_scene_staged_diag"
REPORT_PATH="${LATEST_LINK}/report.txt"

if [[ ! -L "${LATEST_LINK}" && ! -d "${LATEST_LINK}" ]]; then
  echo "[show-latest-scene-staged] latest_scene_staged_diag not found under ${ARTIFACT_ROOT}" >&2
  exit 1
fi

if [[ ! -f "${REPORT_PATH}" ]]; then
  echo "[show-latest-scene-staged] report not found: ${REPORT_PATH}" >&2
  exit 1
fi

cat "${REPORT_PATH}"
