#!/usr/bin/env bash
set -eo pipefail
export COLCON_TRACE="${COLCON_TRACE-}"

SCENE_WS="/home/skbt2/ros2_depth_eval_ws"
LVDOT_WS="/home/skbt2/lvdot_ros2_ws"
RVIZ_CFG="/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz"

cleanup() {
  pkill -f "ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py" || true
  pkill -f "ros2 launch depth_eval_bringup" || true
  pkill -f "ros2 launch lvdot_bringup run_detector_with_adapter.launch.py" || true
  pkill -f "ros2 launch lvdot_bringup run_detector.launch.py" || true
  pkill -f "pedestrian_state_publisher" || true
  pkill -f "pedestrian_pose_sync_system" || true
  pkill -f "ros_gz_bridge/parameter_bridge" || true
  pkill -f "lvdot_ros2_adapter/image_pointcloud_relay" || true
  pkill -f "lvdot_ros2_adapter/lvdot_yolo_node" || true
  pkill -f "lvdot_ros2_adapter/gt_detection_publisher" || true
  pkill -f "lvdot_ros2_adapter/pose_stub" || true
  pkill -f "rviz2 -d ${RVIZ_CFG}" || true
  pkill -f "gz sim -g" || true
  pkill -f "gz sim -s -r" || true
  pkill -f "gz sim" || true
  pkill -f "gzclient" || true
  pkill -f "gzserver" || true
  pkill -f "gazebo --verbose" || true
}

desktop_session_ok() {
  [[ -n "${DISPLAY:-}" ]] && [[ -n "${XDG_RUNTIME_DIR:-}" ]]
}

print_quick_status() {
  echo "Process status:"
  ps -ef | grep -E "gzserver|gzclient|rviz2|run_detector_with_adapter|uav_pedestrian_prototype" | grep -v grep || true
}

if [[ "${1:-}" == "stop" ]]; then
  cleanup
  echo "Stopped scene + detector + rviz."
  exit 0
fi

cleanup
sleep 1

GAZEBO_GUI="${GAZEBO_GUI:-false}"
RVIZ_ENABLE="${RVIZ_ENABLE:-true}"

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

if ! desktop_session_ok; then
  GAZEBO_GUI="false"
  RVIZ_ENABLE="false"
  echo "[WARN] No desktop session detected (DISPLAY/XDG_RUNTIME_DIR missing)."
  echo "[WARN] Force GUI off to avoid gzclient/rviz2 flash-exit."
fi

(
  source "${SCENE_WS}/install/setup.bash"
  exec ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py gazebo_gui:="${GAZEBO_GUI}" rviz:=false enable_uav_controller:=false
) > /tmp/full_stack_scene.log 2>&1 &
SCENE_PID=$!

sleep 3

(
  source "${SCENE_WS}/install/setup.bash"
  source "${LVDOT_WS}/install/setup.bash"
  exec ros2 launch lvdot_bringup run_detector_with_adapter.launch.py launch_pose_stub:=true launch_yolo_node:=true enable_yolo:=true
) > /tmp/full_stack_lvdot.log 2>&1 &
LVDOT_PID=$!

sleep 4
RVIZ_PID=""
if [[ "${RVIZ_ENABLE}" == "true" ]]; then
  (
    source "${SCENE_WS}/install/setup.bash"
    source "${LVDOT_WS}/install/setup.bash"
    exec rviz2 -d "${RVIZ_CFG}"
  ) > /tmp/full_stack_rviz.log 2>&1 &
  RVIZ_PID=$!
fi

sleep 3
if [[ "${GAZEBO_GUI}" == "true" ]] && ! pgrep -f gzclient >/dev/null 2>&1; then
  echo "[WARN] gzclient is not running. Check /tmp/full_stack_scene.log"
fi
if [[ "${RVIZ_ENABLE}" == "true" ]] && ! pgrep -f "rviz2 -d ${RVIZ_CFG}" >/dev/null 2>&1; then
  echo "[WARN] rviz2 is not running. Check /tmp/full_stack_rviz.log"
fi

echo "Started."
echo " GUI flags: gazebo_gui=${GAZEBO_GUI}, rviz=${RVIZ_ENABLE}"
echo " scene pid: ${SCENE_PID} (log: /tmp/full_stack_scene.log)"
echo " lvdot pid: ${LVDOT_PID} (log: /tmp/full_stack_lvdot.log)"
if [[ -n "${RVIZ_PID}" ]]; then
  echo " rviz  pid: ${RVIZ_PID} (log: /tmp/full_stack_rviz.log)"
fi
print_quick_status
echo "Use: /home/skbt2/start_full_stack.sh stop"
