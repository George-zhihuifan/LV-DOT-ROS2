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

ARTIFACT_ROOT="${ARTIFACT_ROOT:-/home/skbt2/lvdot_ros2_ws/artifacts}"
GLOBAL_ARTIFACT_ROOT="/home/skbt2/lvdot_ros2_ws/artifacts"
STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${ARTIFACT_ROOT}/regression_matrix_${STAMP}"
mkdir -p "${ARTIFACT_DIR}"

detector_summary=""
scene_summary=""
detector_validation="UNKNOWN"
detector_failure_reason="unknown"
scene_validation="UNKNOWN"
scene_failure_reason="unknown"
scene_ready_wait_seconds="unknown"
scene_enable_yolo="unknown"
scene_launch_yolo_node="unknown"
scene_validate_failure_keys="unknown"
scene_sample_delta="unknown"
scene_depth_delta="unknown"
scene_color_delta="unknown"
scene_lidar_delta="unknown"
scene_yolo_delta="unknown"
scene_detection_tick_delta="unknown"
scene_tracking_tick_delta="unknown"
scene_dynamic_delta="unknown"
scene_diag_summary=""
scene_diag_result="SKIPPED"
scene_diag_failure_reason="skipped"
scene_staged_summary=""
scene_staged_result="SKIPPED"
scene_staged_failure_reason="skipped"
next_action="unknown"

contains_key() {
  local haystack="$1"
  local needle="$2"
  [[ ",${haystack}," == *",${needle},"* ]]
}

extract_field() {
  local file="$1"
  local key="$2"
  sed -n "s/^${key}=//p" "${file}" | head -n1
}

update_latest_link() {
  ln -sfn "${ARTIFACT_DIR}" "${ARTIFACT_ROOT}/latest_regression_matrix"
}

update_global_latest_link() {
  local target_path="$1"
  local link_name="$2"
  if [[ -n "${target_path}" ]]; then
    ln -sfn "${target_path}" "${GLOBAL_ARTIFACT_ROOT}/${link_name}"
  fi
}

latest_summary() {
  local pattern="$1"
  find "${ARTIFACT_DIR}" -maxdepth 2 -type f -path "${ARTIFACT_DIR}/${pattern}/summary.txt" | sort | tail -n1
}

extract_delta_field() {
  local file="$1"
  local key="$2"
  if [[ -z "${file}" || ! -f "${file}" ]]; then
    return
  fi
  sed -n "s/^${key}_delta=//p" "${file}" | head -n1
}

echo "[matrix] artifact_dir=${ARTIFACT_DIR}"
update_latest_link

echo "[matrix] running detector-only suite"
if ARTIFACT_ROOT="${ARTIFACT_DIR}" \
  /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/run_detector_only_regression_suite.sh; then
  :
else
  :
fi
detector_summary="$(latest_summary 'detector_only_regression_*')"
if [[ -n "${detector_summary}" ]]; then
  detector_validation="$(extract_field "${detector_summary}" validation)"
  detector_failure_reason="$(extract_field "${detector_summary}" failure_reason)"
  update_global_latest_link "$(dirname "${detector_summary}")" latest_detector_only_regression
fi

echo "[matrix] running scene suite"
if ARTIFACT_ROOT="${ARTIFACT_DIR}" \
  /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/run_real_scene_regression_suite.sh; then
  :
else
  :
fi
scene_summary="$(latest_summary 'real_scene_regression_*')"
if [[ -n "${scene_summary}" ]]; then
  scene_validation="$(extract_field "${scene_summary}" validation)"
  scene_failure_reason="$(extract_field "${scene_summary}" failure_reason)"
  scene_ready_wait_seconds="$(extract_field "${scene_summary}" ready_wait_seconds)"
  scene_enable_yolo="$(extract_field "${scene_summary}" enable_yolo)"
  scene_launch_yolo_node="$(extract_field "${scene_summary}" launch_yolo_node)"
  scene_validate_failure_keys="$(extract_field "${scene_summary}" validate_failure_keys)"
  scene_sample_delta="$(extract_field "${scene_summary}" sample_delta)"
  scene_depth_delta="$(extract_delta_field "${scene_sample_delta}" depth_count)"
  scene_color_delta="$(extract_delta_field "${scene_sample_delta}" color_count)"
  scene_lidar_delta="$(extract_delta_field "${scene_sample_delta}" lidar_count)"
  scene_yolo_delta="$(extract_delta_field "${scene_sample_delta}" yolo_count)"
  scene_detection_tick_delta="$(extract_delta_field "${scene_sample_delta}" detection_tick_count)"
  scene_tracking_tick_delta="$(extract_delta_field "${scene_sample_delta}" tracking_tick_count)"
  scene_dynamic_delta="$(extract_delta_field "${scene_sample_delta}" dynamic_count)"
  update_global_latest_link "$(dirname "${scene_summary}")" latest_real_scene_regression
fi

if [[ "${scene_failure_reason}" == "dds_environment_restricted" || "${scene_failure_reason}" == "scene_or_bridge_startup_failed" || "${scene_failure_reason}" == "launch_exited_early" || "${scene_failure_reason}" == "validation_failed" ]]; then
  echo "[matrix] running scene diagnostics"
  if ARTIFACT_ROOT="${ARTIFACT_DIR}" \
    /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/run_scene_environment_diagnostics.sh; then
    :
  else
    :
  fi
  scene_diag_summary="$(latest_summary 'scene_environment_diag_*')"
  if [[ -n "${scene_diag_summary}" ]]; then
    scene_diag_result="$(extract_field "${scene_diag_summary}" result)"
    scene_diag_failure_reason="$(extract_field "${scene_diag_summary}" failure_reason)"
    update_global_latest_link "$(dirname "${scene_diag_summary}")" latest_scene_environment_diag
  fi

  echo "[matrix] running scene staged diagnostics"
  if ARTIFACT_ROOT="${ARTIFACT_DIR}" \
    /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/run_scene_staged_diagnostics.sh; then
    :
  else
    :
  fi
  scene_staged_summary="$(latest_summary 'scene_staged_diag_*')"
  if [[ -n "${scene_staged_summary}" ]]; then
    scene_staged_result="$(extract_field "${scene_staged_summary}" result)"
    scene_staged_failure_reason="$(extract_field "${scene_staged_summary}" failure_reason)"
    update_global_latest_link "$(dirname "${scene_staged_summary}")" latest_scene_staged_diag
  fi
fi

overall_result="UNKNOWN"
if [[ "${detector_failure_reason}" == "dds_environment_restricted" || "${scene_failure_reason}" == "dds_environment_restricted" ]]; then
  overall_result="ENVIRONMENT_RESTRICTED"
elif [[ "${detector_validation}" == "PASS" && "${scene_validation}" == "PASS" ]]; then
  overall_result="PASS"
elif [[ "${detector_validation}" == "PASS" && "${scene_failure_reason}" == "dds_environment_restricted" ]]; then
  overall_result="DETECTOR_PASS_SCENE_ENV_BLOCKED"
else
  overall_result="FAIL"
fi

if [[ "${overall_result}" == "PASS" ]]; then
  next_action="start_real_behavior_review"
elif [[ "${overall_result}" == "DETECTOR_PASS_SCENE_ENV_BLOCKED" || "${overall_result}" == "ENVIRONMENT_RESTRICTED" ]]; then
  if [[ "${scene_staged_failure_reason}" != "skipped" && "${scene_staged_failure_reason}" != "none" ]]; then
    next_action="${scene_staged_failure_reason}"
  elif [[ "${scene_diag_failure_reason}" != "skipped" && "${scene_diag_failure_reason}" != "none" ]]; then
    next_action="${scene_diag_failure_reason}"
  elif [[ "${detector_failure_reason}" == "dds_environment_restricted" ]]; then
    next_action="fix_desktop_dds_or_network_permissions"
  else
    next_action="fix_desktop_scene_environment"
  fi
else
  if [[ "${detector_validation}" == "PASS" && "${scene_failure_reason}" == "ready_timeout" && "${scene_diag_result}" == "PASS" && "${scene_staged_result}" == "PASS" ]]; then
    next_action="increase_ready_timeout_or_fix_uav_startup_latency"
  elif [[ "${detector_validation}" == "PASS" && "${scene_failure_reason}" == "validation_failed" && "${scene_diag_result}" == "PASS" && "${scene_staged_result}" == "PASS" ]]; then
    if contains_key "${scene_validate_failure_keys}" "yolo_input_missing" || contains_key "${scene_validate_failure_keys}" "yolo_input_zero" || contains_key "${scene_validate_failure_keys}" "yolo_input_delta_non_positive"; then
      next_action="inspect_yolo_launch_and_input_flow"
    elif contains_key "${scene_validate_failure_keys}" "depth_input_delta_non_positive" || contains_key "${scene_validate_failure_keys}" "color_input_delta_non_positive" || contains_key "${scene_validate_failure_keys}" "lidar_input_delta_non_positive"; then
      next_action="inspect_scene_sensor_flow_during_sampling_window"
    elif contains_key "${scene_validate_failure_keys}" "detection_timer_delta_non_positive" || contains_key "${scene_validate_failure_keys}" "lidar_detection_timer_delta_non_positive" || contains_key "${scene_validate_failure_keys}" "tracking_timer_delta_non_positive" || contains_key "${scene_validate_failure_keys}" "classification_timer_delta_non_positive" || contains_key "${scene_validate_failure_keys}" "vis_timer_delta_non_positive"; then
      next_action="inspect_detector_pipeline_progress_after_ready"
    elif contains_key "${scene_validate_failure_keys}" "service_type_invalid"; then
      next_action="inspect_detector_service_registration"
    else
      next_action="inspect_post_ready_sampling_window_and_pipeline_history"
    fi
  else
    next_action="inspect_detector_and_scene_summaries"
  fi
fi

cat > "${ARTIFACT_DIR}/summary.txt" <<EOF
overall_result=${overall_result}
artifact_dir=${ARTIFACT_DIR}
detector_summary=${detector_summary}
detector_validation=${detector_validation}
detector_failure_reason=${detector_failure_reason}
scene_summary=${scene_summary}
scene_validation=${scene_validation}
scene_failure_reason=${scene_failure_reason}
scene_ready_wait_seconds=${scene_ready_wait_seconds}
scene_enable_yolo=${scene_enable_yolo}
scene_launch_yolo_node=${scene_launch_yolo_node}
scene_validate_failure_keys=${scene_validate_failure_keys}
scene_sample_delta=${scene_sample_delta}
scene_depth_delta=${scene_depth_delta}
scene_color_delta=${scene_color_delta}
scene_lidar_delta=${scene_lidar_delta}
scene_yolo_delta=${scene_yolo_delta}
scene_detection_tick_delta=${scene_detection_tick_delta}
scene_tracking_tick_delta=${scene_tracking_tick_delta}
scene_dynamic_delta=${scene_dynamic_delta}
scene_diag_summary=${scene_diag_summary}
scene_diag_result=${scene_diag_result}
scene_diag_failure_reason=${scene_diag_failure_reason}
scene_staged_summary=${scene_staged_summary}
scene_staged_result=${scene_staged_result}
scene_staged_failure_reason=${scene_staged_failure_reason}
next_action=${next_action}
EOF

cat > "${ARTIFACT_DIR}/report.txt" <<EOF
LV-DOT ROS2 Regression Matrix Report

overall_result: ${overall_result}
next_action: ${next_action}

detector_only:
  validation: ${detector_validation}
  failure_reason: ${detector_failure_reason}
  summary: ${detector_summary}

scene_regression:
  validation: ${scene_validation}
  failure_reason: ${scene_failure_reason}
  ready_wait_seconds: ${scene_ready_wait_seconds}
  enable_yolo: ${scene_enable_yolo}
  launch_yolo_node: ${scene_launch_yolo_node}
  validate_failure_keys: ${scene_validate_failure_keys}
  sample_delta: ${scene_sample_delta}
  depth_delta: ${scene_depth_delta}
  color_delta: ${scene_color_delta}
  lidar_delta: ${scene_lidar_delta}
  yolo_delta: ${scene_yolo_delta}
  detection_tick_delta: ${scene_detection_tick_delta}
  tracking_tick_delta: ${scene_tracking_tick_delta}
  dynamic_delta: ${scene_dynamic_delta}
  summary: ${scene_summary}

scene_diagnostics:
  result: ${scene_diag_result}
  failure_reason: ${scene_diag_failure_reason}
  summary: ${scene_diag_summary}

scene_staged:
  result: ${scene_staged_result}
  failure_reason: ${scene_staged_failure_reason}
  summary: ${scene_staged_summary}

artifacts:
  root: ${ARTIFACT_DIR}
  matrix_summary: ${ARTIFACT_DIR}/summary.txt
  matrix_report: ${ARTIFACT_DIR}/report.txt
EOF

cat > "${ARTIFACT_DIR}/report.json" <<EOF
{
  "overall_result": "${overall_result}",
  "next_action": "${next_action}",
  "artifact_dir": "${ARTIFACT_DIR}",
  "detector": {
    "summary": "${detector_summary}",
    "validation": "${detector_validation}",
    "failure_reason": "${detector_failure_reason}"
  },
  "scene": {
    "summary": "${scene_summary}",
    "validation": "${scene_validation}",
    "failure_reason": "${scene_failure_reason}",
    "ready_wait_seconds": "${scene_ready_wait_seconds}",
    "enable_yolo": "${scene_enable_yolo}",
    "launch_yolo_node": "${scene_launch_yolo_node}",
    "validate_failure_keys": "${scene_validate_failure_keys}",
    "sample_delta": "${scene_sample_delta}",
    "depth_delta": "${scene_depth_delta}",
    "color_delta": "${scene_color_delta}",
    "lidar_delta": "${scene_lidar_delta}",
    "yolo_delta": "${scene_yolo_delta}",
    "detection_tick_delta": "${scene_detection_tick_delta}",
    "tracking_tick_delta": "${scene_tracking_tick_delta}",
    "dynamic_delta": "${scene_dynamic_delta}"
  },
  "scene_diagnostics": {
    "summary": "${scene_diag_summary}",
    "result": "${scene_diag_result}",
    "failure_reason": "${scene_diag_failure_reason}"
  },
  "scene_staged": {
    "summary": "${scene_staged_summary}",
    "result": "${scene_staged_result}",
    "failure_reason": "${scene_staged_failure_reason}"
  }
}
EOF

echo "[matrix] overall_result=${overall_result}"
echo "[matrix] summary=${ARTIFACT_DIR}/summary.txt"
echo "[matrix] report=${ARTIFACT_DIR}/report.txt"

if [[ "${overall_result}" != "PASS" && "${overall_result}" != "DETECTOR_PASS_SCENE_ENV_BLOCKED" && "${overall_result}" != "ENVIRONMENT_RESTRICTED" ]]; then
  exit 1
fi
