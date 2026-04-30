#!/usr/bin/env bash
set -euo pipefail

# One-click launcher for LiDAR + YOLO test chain (map-aligned RViz view)
# - Strict cleanup of stale processes
# - Start scene + detector in lidar_driven mode
# - Enable YOLO node
# - Use pose_stub as a stable single pose source
# - Open RViz with map-aligned cloud/boxes/image config

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="${SCRIPT_DIR%/scripts}/rviz/lidar_boxes_map_aligned.rviz"
LAUNCH_LOG="/tmp/start_lidar_yolo_map_aligned.launch.log"
RVIZ_LOG="/tmp/start_lidar_yolo_map_aligned.rviz.log"

cleanup_stale() {
  set +e
  for p in $(pgrep -f 'run_detector_with_scene.launch.py|run_detector_with_adapter.launch.py|uav_pedestrian_prototype.launch.py|pedestrian_state_publisher|uav_trajectory_controller|lvdot_detector_main|lvdot_yolo_node|pose_stub|gt_detection_publisher|image_pointcloud_relay|parameter_bridge|rviz2|gz sim|ros2 launch' || true); do
    if [[ "$p" != "$$" ]]; then
      kill -9 "$p" 2>/dev/null || true
    fi
  done
  sleep 1
  set -e
}

stop_all() {
  cleanup_stale
  rm -f /tmp/start_lidar_yolo_map_aligned.launch.pid /tmp/start_lidar_yolo_map_aligned.rviz.pid
  echo "[stop] all related processes stopped"
}

source_env() {
  set +u
  source /opt/ros/jazzy/setup.bash
  source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
  source /home/skbt2/lvdot_ros2_ws/install/setup.bash
  set -u
}

start_chain() {
  nohup ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false \
    rviz:=false \
    detector_rviz:=false \
    enable_uav_controller:=false \
    launch_pose_stub:=true \
    enable_yolo:=true \
    launch_yolo_node:=true \
    fusion_mode:=lidar_driven \
    enable_stage_timers:=true \
    enable_vis_stage:=true \
    executor_threads:=2 \
    > "${LAUNCH_LOG}" 2>&1 &
  echo $! > /tmp/start_lidar_yolo_map_aligned.launch.pid
}

start_rviz() {
  nohup rviz2 -d "${RVIZ_CONFIG}" > "${RVIZ_LOG}" 2>&1 &
  echo $! > /tmp/start_lidar_yolo_map_aligned.rviz.pid
}

print_status() {
  echo "[start] launch_log=${LAUNCH_LOG}"
  echo "[start] rviz_log=${RVIZ_LOG}"
  if timeout 8 ros2 topic echo /onboard_detector/pipeline_stats_status --once >/tmp/start_lidar_yolo_map_aligned.pipeline.txt 2>/dev/null; then
    echo "[start] pipeline snapshot:"
    sed -n '1,80p' /tmp/start_lidar_yolo_map_aligned.pipeline.txt
  else
    echo "[start] warning: pipeline_stats not ready yet"
  fi
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
  start_chain
  sleep 10
  start_rviz
  sleep 2
  print_status
  wait_for_stop_key
}

main "$@"
