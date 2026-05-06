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
ARTIFACT_DIR="${ARTIFACT_ROOT}/scene_environment_diag_${STAMP}"
mkdir -p "${ARTIFACT_DIR}"

RESULT="FAIL"
FAILURE_REASON="unknown"
NEXT_ACTION="inspect_summary"

write_summary() {
  cat > "${ARTIFACT_DIR}/summary.txt" <<EOF
result=${RESULT}
failure_reason=${FAILURE_REASON}
next_action=${NEXT_ACTION}
artifact_dir=${ARTIFACT_DIR}
warmup_seconds=${WARMUP_SECONDS}
launch_log=${ARTIFACT_DIR}/launch.log
node_list=${ARTIFACT_DIR}/node_list.txt
topic_list=${ARTIFACT_DIR}/topic_list.txt
service_list=${ARTIFACT_DIR}/service_list.txt
camera_info=${ARTIFACT_DIR}/camera_info.txt
depth_image=${ARTIFACT_DIR}/depth_image.txt
color_image=${ARTIFACT_DIR}/color_image.txt
pointcloud=${ARTIFACT_DIR}/pointcloud.txt
pose=${ARTIFACT_DIR}/pose.txt
odom=${ARTIFACT_DIR}/odom.txt
EOF
}

write_report() {
  cat > "${ARTIFACT_DIR}/report.txt" <<EOF
Scene Environment Diagnostics Report

result: ${RESULT}
failure_reason: ${FAILURE_REASON}
next_action: ${NEXT_ACTION}

artifacts:
  root: ${ARTIFACT_DIR}
  summary: ${ARTIFACT_DIR}/summary.txt
  launch_log: ${ARTIFACT_DIR}/launch.log
  node_list: ${ARTIFACT_DIR}/node_list.txt
  topic_list: ${ARTIFACT_DIR}/topic_list.txt
  service_list: ${ARTIFACT_DIR}/service_list.txt
  camera_info: ${ARTIFACT_DIR}/camera_info.txt
  depth_image: ${ARTIFACT_DIR}/depth_image.txt
  color_image: ${ARTIFACT_DIR}/color_image.txt
  pointcloud: ${ARTIFACT_DIR}/pointcloud.txt
  pose: ${ARTIFACT_DIR}/pose.txt
  odom: ${ARTIFACT_DIR}/odom.txt
EOF

  cat > "${ARTIFACT_DIR}/report.json" <<EOF
{
  "result": "${RESULT}",
  "failure_reason": "${FAILURE_REASON}",
  "next_action": "${NEXT_ACTION}",
  "artifact_dir": "${ARTIFACT_DIR}"
}
EOF
}

update_latest_link() {
  ln -sfn "${ARTIFACT_DIR}" "${ARTIFACT_ROOT}/latest_scene_environment_diag"
}

classify_failure_reason() {
  if grep -Eq 'getifaddrs: Operation not permitted|error in getifaddrs: Unknown error -1|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${ARTIFACT_DIR}/launch.log"; then
    FAILURE_REASON="dds_environment_restricted"
    NEXT_ACTION="fix_desktop_dds_or_network_permissions"
  elif grep -Eq 'process has died.*gazebo-|process has died.*parameter_bridge' "${ARTIFACT_DIR}/launch.log"; then
    FAILURE_REASON="gazebo_or_bridge_failed"
    NEXT_ACTION="inspect_launch_log_for_gazebo_and_bridge_failures"
  elif [[ ! -s "${ARTIFACT_DIR}/camera_info.txt" ]]; then
    FAILURE_REASON="sensor_topics_missing"
    NEXT_ACTION="inspect_bridge_and_sensor_plugin_configuration"
  elif [[ ! -s "${ARTIFACT_DIR}/pose.txt" ]] || [[ ! -s "${ARTIFACT_DIR}/odom.txt" ]]; then
    FAILURE_REASON="uav_state_topics_missing"
    NEXT_ACTION="inspect_uav_controller_and_set_pose_service_chain"
  else
    FAILURE_REASON="scene_validation_failed"
    NEXT_ACTION="inspect_scene_summary_and_topic_outputs"
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

echo "[scene-diag] artifact_dir=${ARTIFACT_DIR}"
echo "[scene-diag] launching scene"
ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py \
  rviz:=false \
  gazebo_gui:=false \
  > "${ARTIFACT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[scene-diag] waiting ${WARMUP_SECONDS}s"
sleep "${WARMUP_SECONDS}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[scene-diag] launch exited during warmup"
  classify_failure_reason
  write_summary
  write_report
  exit 1
fi

timeout 5 ros2 node list --no-daemon > "${ARTIFACT_DIR}/node_list.txt" 2>&1 || true
timeout 5 ros2 topic list --no-daemon > "${ARTIFACT_DIR}/topic_list.txt" 2>&1 || true
timeout 5 ros2 service list -t --no-daemon > "${ARTIFACT_DIR}/service_list.txt" 2>&1 || true

timeout 8 ros2 topic echo /rgbd_camera/camera_info --once > "${ARTIFACT_DIR}/camera_info.txt" 2>&1 || true
timeout 8 ros2 topic echo /rgbd_camera/depth_image --once > "${ARTIFACT_DIR}/depth_image.txt" 2>&1 || true
timeout 8 ros2 topic echo /rgbd_camera/image --once > "${ARTIFACT_DIR}/color_image.txt" 2>&1 || true
timeout 8 ros2 topic echo /rgbd_camera/points --once > "${ARTIFACT_DIR}/pointcloud.txt" 2>&1 || true
timeout 8 ros2 topic echo /mavros/local_position/pose --once > "${ARTIFACT_DIR}/pose.txt" 2>&1 || true
timeout 8 ros2 topic echo /mavros/local_position/odom --once > "${ARTIFACT_DIR}/odom.txt" 2>&1 || true

if [[ -s "${ARTIFACT_DIR}/camera_info.txt" ]] \
  && [[ -s "${ARTIFACT_DIR}/depth_image.txt" ]] \
  && [[ -s "${ARTIFACT_DIR}/color_image.txt" ]] \
  && [[ -s "${ARTIFACT_DIR}/pointcloud.txt" ]] \
  && [[ -s "${ARTIFACT_DIR}/pose.txt" ]] \
  && [[ -s "${ARTIFACT_DIR}/odom.txt" ]]; then
  RESULT="PASS"
  FAILURE_REASON="none"
  NEXT_ACTION="start_scene_behavior_review"
else
  classify_failure_reason
fi

write_summary
write_report

echo "[scene-diag] result=${RESULT}"
echo "[scene-diag] summary=${ARTIFACT_DIR}/summary.txt"
echo "[scene-diag] report=${ARTIFACT_DIR}/report.txt"

if [[ "${RESULT}" != "PASS" ]]; then
  exit 1
fi
