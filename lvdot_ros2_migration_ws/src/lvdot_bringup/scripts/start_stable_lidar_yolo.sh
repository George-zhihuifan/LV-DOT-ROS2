#!/usr/bin/env bash
set -euo pipefail

# Stable mode: reduce pose-write pressure and processing load.
# - strict cleanup
# - lidar_driven (depth branch disabled from fusion)
# - disable UAV trajectory controller (avoids extra set_pose traffic)
# - keep YOLO + LiDAR
# - open minimal RViz (LiDAR cloud + YOLO 2D image)

set +u
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source /home/skbt2/lvdot_ros2_ws/install/setup.bash
set -u

pkill -9 -f 'run_detector_with_scene.launch.py|uav_pedestrian_prototype.launch.py|lvdot_detector_main|lvdot_yolo_node|parameter_bridge|^gz sim|rviz2' || true
sleep 1

nohup ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  gazebo_gui:=false \
  rviz:=false \
  detector_rviz:=false \
  enable_uav_controller:=false \
  enable_yolo:=true \
  launch_yolo_node:=true \
  fusion_mode:=lidar_driven \
  enable_stage_timers:=false \
  enable_vis_stage:=false \
  executor_threads:=2 \
  > /tmp/stable_lidar_yolo_chain.log 2>&1 &

sleep 8

nohup rviz2 -d /home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/rviz/lidar_yolo_2view_only.rviz \
  > /tmp/stable_lidar_yolo_rviz.log 2>&1 &

sleep 2

echo "[stable] started"
echo "  chain_log: /tmp/stable_lidar_yolo_chain.log"
echo "  rviz_log : /tmp/stable_lidar_yolo_rviz.log"

# Print a quick health snapshot
if timeout 6 ros2 topic echo /onboard_detector/pipeline_stats_status --once >/tmp/stable_pipeline_snapshot.txt 2>/dev/null; then
  echo "[stable] pipeline snapshot:"
  sed -n '1,80p' /tmp/stable_pipeline_snapshot.txt
else
  echo "[stable] warning: pipeline stats not ready yet"
fi
