#!/usr/bin/env bash
set -euo pipefail

SCENE_WS="/home/skbt2/ros2_depth_eval_ws"
LVDOT_WS="/home/skbt2/lvdot_ros2_ws"
RVIZ_CFG="${LVDOT_WS}/src/lvdot_bringup/rviz/uv_explain.rviz"

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
  echo "Stopped uv-explain stack."
  exit 0
fi

cleanup
sleep 1

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py gazebo_gui:=false rviz:=false enable_uav_controller:=false" > /tmp/lvdot_logs/uv_explain_scene.log 2>&1 < /dev/null & disown
sleep 3

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; source ${LVDOT_WS}/install/setup.bash; ros2 launch lvdot_bringup run_detector_with_adapter.launch.py launch_relay:=false launch_pose_stub:=true launch_yolo_node:=false launch_gt_publisher:=false enable_yolo:=false rviz:=false fusion_mode:=dual" > /tmp/lvdot_logs/uv_explain_detector.log 2>&1 < /dev/null & disown
sleep 4

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; source ${LVDOT_WS}/install/setup.bash; rviz2 -d ${RVIZ_CFG}" > /tmp/lvdot_logs/uv_explain_rviz.log 2>&1 < /dev/null & disown
sleep 2

echo "UV-explain stack started."
echo "RViz config: ${RVIZ_CFG}"
echo "Logs: /tmp/lvdot_logs/uv_explain_scene.log /tmp/lvdot_logs/uv_explain_detector.log /tmp/lvdot_logs/uv_explain_rviz.log"
echo "Stop: bash ${LVDOT_WS}/scripts/start_uv_explain_stack.sh stop"
