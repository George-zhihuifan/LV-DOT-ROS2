#!/usr/bin/env bash
set -euo pipefail

SCENE_WS="/home/skbt2/ros2_depth_eval_ws"
LVDOT_WS="/home/skbt2/lvdot_ros2_ws"
RVIZ_CFG="${LVDOT_WS}/src/lvdot_bringup/rviz/lidar_only.rviz"

mkdir -p /tmp/lvdot_logs

cleanup() {
  pkill -f "ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py" || true
  pkill -f "ros2 launch lvdot_bringup run_detector_with_adapter.launch.py" || true
  pkill -f "lvdot_detector_main" || true
  pkill -f "lvdot_yolo_node" || true
  pkill -f "gt_detection_publisher" || true
  pkill -f "pose_stub" || true
  pkill -f "image_pointcloud_relay" || true
  pkill -f "parameter_bridge" || true
  pkill -f "gz sim" || true
  pkill -f "rviz2 -d ${RVIZ_CFG}" || true
}

if [[ "${1:-}" == "stop" ]]; then
  cleanup
  echo "Stopped lidar+yolo stack."
  exit 0
fi

cleanup
sleep 1

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py gazebo_gui:=false rviz:=false enable_uav_controller:=false" > /tmp/lvdot_logs/lidar_yolo_scene.log 2>&1 < /dev/null & disown
sleep 3

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; source ${LVDOT_WS}/install/setup.bash; ros2 launch lvdot_bringup run_detector_with_adapter.launch.py launch_relay:=false launch_pose_stub:=true launch_yolo_node:=true launch_gt_publisher:=false enable_yolo:=true rviz:=false fusion_mode:=lidar_driven depth_image_topic:=/__disabled_depth color_image_topic:=/rgbd_camera/image lidar_pointcloud_topic:=/livox/lidar/pointcloud yolo_detection_topic:=/yolo_detector/detected_bounding_boxes" > /tmp/lvdot_logs/lidar_yolo_detector.log 2>&1 < /dev/null & disown
sleep 5

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; source ${LVDOT_WS}/install/setup.bash; rviz2 -d ${RVIZ_CFG}" > /tmp/lvdot_logs/lidar_yolo_rviz.log 2>&1 < /dev/null & disown
sleep 2

echo "Lidar+YOLO stack started."
echo "RViz config: ${RVIZ_CFG}"
echo "Logs: /tmp/lvdot_logs/lidar_yolo_scene.log /tmp/lvdot_logs/lidar_yolo_detector.log /tmp/lvdot_logs/lidar_yolo_rviz.log"
echo "Stop: bash ${LVDOT_WS}/scripts/start_lidar_yolo_stack.sh stop"
