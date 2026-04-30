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

WARMUP_SECONDS="${WARMUP_SECONDS:-12}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-/home/skbt2/lvdot_ros2_ws/artifacts}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${ARTIFACT_ROOT}/scene_staged_diag_${STAMP}"
mkdir -p "${ARTIFACT_DIR}"

RESULT="FAIL"
FAILURE_REASON="unknown"
NEXT_ACTION="inspect_stage_reports"

stage_gazebo="FAIL"
stage_bridge="FAIL"
stage_sensors="FAIL"
stage_uav_state="FAIL"

write_summary() {
  cat > "${ARTIFACT_DIR}/summary.txt" <<EOF
result=${RESULT}
failure_reason=${FAILURE_REASON}
next_action=${NEXT_ACTION}
artifact_dir=${ARTIFACT_DIR}
warmup_seconds=${WARMUP_SECONDS}
stage_gazebo=${stage_gazebo}
stage_bridge=${stage_bridge}
stage_sensors=${stage_sensors}
stage_uav_state=${stage_uav_state}
launch_log=${ARTIFACT_DIR}/launch.log
node_list=${ARTIFACT_DIR}/node_list.txt
topic_list=${ARTIFACT_DIR}/topic_list.txt
service_list=${ARTIFACT_DIR}/service_list.txt
EOF
}

write_report() {
  cat > "${ARTIFACT_DIR}/report.txt" <<EOF
Scene Staged Diagnostics Report

result: ${RESULT}
failure_reason: ${FAILURE_REASON}
next_action: ${NEXT_ACTION}

stages:
  gazebo: ${stage_gazebo}
  bridge: ${stage_bridge}
  sensors: ${stage_sensors}
  uav_state: ${stage_uav_state}

artifacts:
  root: ${ARTIFACT_DIR}
  summary: ${ARTIFACT_DIR}/summary.txt
  launch_log: ${ARTIFACT_DIR}/launch.log
  node_list: ${ARTIFACT_DIR}/node_list.txt
  topic_list: ${ARTIFACT_DIR}/topic_list.txt
  service_list: ${ARTIFACT_DIR}/service_list.txt
EOF
}

update_latest_link() {
  ln -sfn "${ARTIFACT_DIR}" "${ARTIFACT_ROOT}/latest_scene_staged_diag"
}

classify_failure() {
  if grep -Eq 'getifaddrs: Operation not permitted|error in getifaddrs: Unknown error -1|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${ARTIFACT_DIR}/launch.log"; then
    FAILURE_REASON="dds_environment_restricted"
    NEXT_ACTION="fix_desktop_dds_or_network_permissions"
  elif [[ "${stage_gazebo}" != "PASS" ]]; then
    FAILURE_REASON="gazebo_failed"
    NEXT_ACTION="inspect_gazebo_startup_and_world_assets"
  elif [[ "${stage_bridge}" != "PASS" ]]; then
    FAILURE_REASON="bridge_failed"
    NEXT_ACTION="inspect_parameter_bridge_arguments_and_ros_gz_install"
  elif [[ "${stage_sensors}" != "PASS" ]]; then
    FAILURE_REASON="sensor_chain_failed"
    NEXT_ACTION="inspect_sensor_plugin_and_bridge_topic_mapping"
  elif [[ "${stage_uav_state}" != "PASS" ]]; then
    FAILURE_REASON="uav_state_chain_failed"
    NEXT_ACTION="inspect_uav_controller_and_set_pose_service_chain"
  else
    FAILURE_REASON="scene_validation_failed"
    NEXT_ACTION="inspect_stage_reports"
  fi
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

update_latest_link

echo "[scene-staged] artifact_dir=${ARTIFACT_DIR}"
echo "[scene-staged] launching scene"
ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py \
  rviz:=false \
  gazebo_gui:=false \
  > "${ARTIFACT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[scene-staged] waiting ${WARMUP_SECONDS}s"
sleep "${WARMUP_SECONDS}"

timeout 5 ros2 node list --no-daemon > "${ARTIFACT_DIR}/node_list.txt" 2>&1 || true
timeout 5 ros2 topic list --no-daemon > "${ARTIFACT_DIR}/topic_list.txt" 2>&1 || true
timeout 5 ros2 service list -t --no-daemon > "${ARTIFACT_DIR}/service_list.txt" 2>&1 || true

if grep -Eq 'getifaddrs: Operation not permitted|error in getifaddrs: Unknown error -1|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${ARTIFACT_DIR}/launch.log"; then
  classify_failure
  write_summary
  write_report
  echo "[scene-staged] result=${RESULT}"
  echo "[scene-staged] summary=${ARTIFACT_DIR}/summary.txt"
  exit 1
fi

if grep -Eq '^/gazebo$|^/parameter_bridge$|/uav_trajectory_controller|/pedestrian_state_publisher' "${ARTIFACT_DIR}/node_list.txt" \
  || grep -Eq 'process started with pid' "${ARTIFACT_DIR}/launch.log"; then
  stage_gazebo="PASS"
fi

if grep -Eq '^/world/pedestrian_prototype/set_pose\s+\[ros_gz_interfaces/srv/SetEntityPose\]$|^/world/pedestrian_prototype/set_pose/blocking\s+\[ros_gz_interfaces/srv/SetEntityPose\]$' "${ARTIFACT_DIR}/service_list.txt"; then
  stage_bridge="PASS"
fi

if grep -Eq '^/rgbd_camera/camera_info$' "${ARTIFACT_DIR}/topic_list.txt" \
  && grep -Eq '^/rgbd_camera/depth_image$' "${ARTIFACT_DIR}/topic_list.txt" \
  && grep -Eq '^/rgbd_camera/image$' "${ARTIFACT_DIR}/topic_list.txt" \
  && grep -Eq '^/rgbd_camera/points$' "${ARTIFACT_DIR}/topic_list.txt"; then
  stage_sensors="PASS"
fi

if grep -Eq '^/mavros/local_position/pose$' "${ARTIFACT_DIR}/topic_list.txt" \
  && grep -Eq '^/mavros/local_position/odom$' "${ARTIFACT_DIR}/topic_list.txt"; then
  stage_uav_state="PASS"
fi

if [[ "${stage_gazebo}" == "PASS" && "${stage_bridge}" == "PASS" && "${stage_sensors}" == "PASS" && "${stage_uav_state}" == "PASS" ]]; then
  RESULT="PASS"
  FAILURE_REASON="none"
  NEXT_ACTION="start_scene_behavior_review"
else
  classify_failure
fi

write_summary
write_report

echo "[scene-staged] result=${RESULT}"
echo "[scene-staged] summary=${ARTIFACT_DIR}/summary.txt"
echo "[scene-staged] report=${ARTIFACT_DIR}/report.txt"

if [[ "${RESULT}" != "PASS" ]]; then
  exit 1
fi
