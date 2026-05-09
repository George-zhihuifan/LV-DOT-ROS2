#!/usr/bin/env bash
set -euo pipefail

SCENE_WS="/home/skbt2/ros2_depth_eval_ws"
LVDOT_WS="/home/skbt2/lvdot_ros2_ws"
MIGRATION_WS="/home/skbt2/lvdot_ros2_migration_ws"
QCGAF_CFG="${QCGAF_CFG:-/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/config/qcgaf_takeover.yaml}"
QCGAF_CKPT="${QCGAF_CKPT:-/home/skbt2/QCGAF-GRU-UAV-Project/qcgaf_fusion/outputs/best_model.pt}"
RVIZ_CFG="${RVIZ_CFG:-/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz}"

cleanup() {
  pkill -f "ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py" || true
  pkill -f "ros2 launch lvdot_bringup run_detector_with_adapter.launch.py" || true
  pkill -f "qcgaf_fusion_node|qcgaf_fusion/fusion_node" || true
  pkill -f "lvdot_detector_main|lvdot_yolo_node|image_pointcloud_relay|parameter_bridge|pedestrian_state_publisher|pose_stub|gt_detection_publisher" || true
  pkill -f "rviz2 -d ${RVIZ_CFG}" || true
  pkill -f "gz sim|gzserver|gzclient|gazebo" || true
}

if [[ "${1:-}" == "stop" ]]; then
  cleanup
  sleep 1
  echo "Stopped QC takeover stack."
  exit 0
fi

if [[ ! -f "$QCGAF_CFG" ]]; then
  echo "Missing QCGAF config: $QCGAF_CFG"
  exit 1
fi
if [[ ! -f "$QCGAF_CKPT" ]]; then
  echo "Missing QCGAF checkpoint: $QCGAF_CKPT"
  exit 1
fi

cleanup
sleep 1

(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${SCENE_WS}/install/setup.bash"
  set -u
  exec ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py gazebo_gui:=false rviz:=false enable_uav_controller:=false
) >/tmp/qc_takeover_scene.log 2>&1 &
SCENE_PID=$!

sleep 3

(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${SCENE_WS}/install/setup.bash"
  source "${LVDOT_WS}/install/setup.bash"
  set -u
  exec ros2 launch lvdot_bringup run_detector_with_adapter.launch.py launch_pose_stub:=true launch_yolo_node:=true enable_yolo:=true
) >/tmp/qc_takeover_detector.log 2>&1 &
DET_PID=$!

sleep 3

(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${MIGRATION_WS}/install/setup.bash"
  set -u
  exec ros2 run qcgaf_fusion fusion_node --ros-args \
    -p config:="$QCGAF_CFG" \
    -p checkpoint:="$QCGAF_CKPT" \
    -p verbose:=false \
    -p debug_metrics:=true \
    -p use_sim_time:=true
) >/tmp/qc_takeover_qcgaf.log 2>&1 &
QCGAF_PID=$!

sleep 2

(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${SCENE_WS}/install/setup.bash"
  source "${LVDOT_WS}/install/setup.bash"
  set -u
  exec rviz2 -d "${RVIZ_CFG}"
) >/tmp/qc_takeover_rviz.log 2>&1 &
RVIZ_PID=$!

sleep 2

echo "Started QC takeover stack"
echo " scene : $SCENE_PID (/tmp/qc_takeover_scene.log)"
echo " det   : $DET_PID (/tmp/qc_takeover_detector.log)"
echo " qcgaf : $QCGAF_PID (/tmp/qc_takeover_qcgaf.log)"
echo " rviz  : $RVIZ_PID (/tmp/qc_takeover_rviz.log)"
echo " QC fused topic: /onboard_detector/filtered_bboxes_qc"

echo "Quick check:"
set +u
source /opt/ros/jazzy/setup.bash
source "${SCENE_WS}/install/setup.bash"
source "${LVDOT_WS}/install/setup.bash"
set -u
ros2 topic list | rg '/onboard_detector/(visual_bboxes|lidar_bboxes|filtered_bboxes_qc)' || true
