#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-yolo}"
WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
RVIZ_DIR="$WS_DIR/src/lvdot_bringup/rviz"

case "$MODE" in
  yolo)  RVIZ_CFG="$RVIZ_DIR/yolo_only.rviz" ;;
  lidar) RVIZ_CFG="$RVIZ_DIR/lidar_only.rviz" ;;
  depth) RVIZ_CFG="$RVIZ_DIR/uv_only.rviz" ;;
  *) echo "Usage: $0 [yolo|lidar|depth]"; exit 1 ;;
esac

pkill -9 -f 'run_detector_with_scene.launch.py|uav_pedestrian_prototype.launch.py|lvdot_detector_main|lvdot_yolo_node|parameter_bridge|rviz2|gz sim|ros2 launch|gt_detection_publisher|image_pointcloud_relay' >/dev/null 2>&1 || true
sleep 2

set +u
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

nohup ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  gazebo_gui:=false rviz:=false detector_rviz:=false \
  enable_uav_controller:=false launch_pose_stub:=true \
  enable_yolo:=true launch_yolo_node:=true fusion_mode:=dual \
  enable_stage_timers:=true enable_vis_stage:=true executor_threads:=4 \
  > /tmp/visual_probe_${MODE}.launch.log 2>&1 &

echo $! > /tmp/visual_probe_${MODE}.launch.pid
sleep 10
nohup rviz2 -d "$RVIZ_CFG" > /tmp/visual_probe_${MODE}.rviz.log 2>&1 &
echo $! > /tmp/visual_probe_${MODE}.rviz.pid

echo "[visual_probe] mode=$MODE"
echo "[visual_probe] launch_log=/tmp/visual_probe_${MODE}.launch.log"
echo "[visual_probe] rviz_log=/tmp/visual_probe_${MODE}.rviz.log"
