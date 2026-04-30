#!/usr/bin/env bash
set -euo pipefail

SCENE_WS="/home/skbt2/ros2_depth_eval_ws"
LVDOT_WS="/home/skbt2/lvdot_ros2_ws"
RVIZ_CFG="${LVDOT_WS}/src/lvdot_bringup/rviz/uv_only.rviz"

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

ensure_rviz_cfg() {
  if [[ -f "${RVIZ_CFG}" ]]; then
    return
  fi
  cat > "${RVIZ_CFG}" <<'RVIZEOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 80
      Cell Size: 1
      Color: 160; 160; 160
      Alpha: 0.35
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: U-Depth Map
      Topic:
        Value: /onboard_detector/detected_u_depth_map
        Reliability Policy: Best Effort
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 1
      Queue Size: 1
      Median window: 5
      Min Value: 0
      Max Value: 1
      Normalize Range: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: UV BBoxes
      Topic:
        Value: /onboard_detector/uv_bboxes
      Queue Size: 100
  Enabled: true
  Global Options:
    Fixed Frame: map
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Value: true
RVIZEOF
}

if [[ "${1:-}" == "stop" ]]; then
  cleanup
  echo "Stopped uv-only stack."
  exit 0
fi

ensure_rviz_cfg
cleanup
sleep 1

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py gazebo_gui:=false rviz:=false enable_uav_controller:=false" > /tmp/lvdot_logs/uv_scene.log 2>&1 < /dev/null & disown
sleep 3

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; source ${LVDOT_WS}/install/setup.bash; ros2 launch lvdot_bringup run_detector_with_adapter.launch.py launch_relay:=false launch_pose_stub:=true launch_yolo_node:=false launch_gt_publisher:=false enable_yolo:=false rviz:=false lidar_pointcloud_topic:=/__disabled_lidar" > /tmp/lvdot_logs/uv_detector.log 2>&1 < /dev/null & disown
sleep 4

setsid bash -lc "source /opt/ros/jazzy/setup.bash; source ${SCENE_WS}/install/setup.bash; source ${LVDOT_WS}/install/setup.bash; rviz2 -d ${RVIZ_CFG}" > /tmp/lvdot_logs/uv_rviz.log 2>&1 < /dev/null & disown
sleep 2

echo "UV-only stack started."
echo "RViz config: ${RVIZ_CFG}"
echo "Logs: /tmp/lvdot_logs/uv_scene.log /tmp/lvdot_logs/uv_detector.log /tmp/lvdot_logs/uv_rviz.log"
echo "Stop: bash ${LVDOT_WS}/scripts/start_uv_only_stack.sh stop"
