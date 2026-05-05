#!/usr/bin/env bash
set -euo pipefail

LAUNCH_LOG="/tmp/start_metric5_interface_stack.launch.log"

cleanup_stale() {
  set +e
  for p in $(pgrep -f 'run_metric5_interface.launch.py|run_detector_with_scene.launch.py|run_detector_with_adapter.launch.py|uav_pedestrian_prototype.launch.py|pedestrian_state_publisher|uav_trajectory_controller|lvdot_detector_main|lvdot_yolo_node|pose_stub|gt_detection_publisher|image_pointcloud_relay|metric5_interface_adapter.py|parameter_bridge|rviz2|gz sim|ros2 launch' || true); do
    if [[ "$p" != "$$" ]]; then
      kill -9 "$p" 2>/dev/null || true
    fi
  done
  sleep 1
  set -e
}

stop_all() {
  cleanup_stale
  rm -f /tmp/start_metric5_interface_stack.launch.pid
  echo "[stop] all related processes stopped"
}

source_env() {
  set +u
  source /opt/ros/jazzy/setup.bash
  source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
  source /home/skbt2/lvdot_ros2_ws/install/setup.bash
  set -u
}

start_stack() {
  nohup ros2 launch lvdot_bringup run_metric5_interface.launch.py \
    launch_detector_stack:=true \
    gazebo_gui:=false \
    rviz:=false \
    detector_rviz:=false \
    enable_uav_controller:=false \
    enable_yolo:=true \
    launch_yolo_node:=true \
    fusion_mode:=dual \
    output_mode:=both \
    source_ns_prefix:=tracked \
    input_image_topic:=/rgbd_camera/image \
    input_pointcloud_topic:=/uav_lidar/scan/points \
    input_detected_image_topic:=/yolo_detector/detected_image \
    input_boxes_topic:=/onboard_detector/tracked_bboxes \
    output_marker_topic:=/metric5/detection_3d_marker \
    output_image_topic:=/metric5/detection_2d_image \
    > "${LAUNCH_LOG}" 2>&1 &
  echo $! > /tmp/start_metric5_interface_stack.launch.pid
}

print_status() {
  echo "[start] launch_log=${LAUNCH_LOG}"
  echo "[start] probing output topics..."
  timeout 8 ros2 topic info /metric5/detection_3d_marker || true
  timeout 8 ros2 topic info /metric5/detection_2d_image || true
}

wait_for_stop_key() {
  echo "[control] press 'q' then Enter to stop all processes"
  while true; do
    if ! IFS= read -r key; then
      sleep 0.2
      continue
    fi
    if [[ "${key}" == "q" || "${key}" == "Q" ]]; then
      stop_all
      break
    fi
    echo "[control] unknown key '${key}', press 'q' to stop"
  done
}

main() {
  source_env
  cleanup_stale
  start_stack
  sleep 12
  print_status
  wait_for_stop_key
}

main "$@"

