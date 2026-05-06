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

exec ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  gazebo_gui:=false \
  rviz:=false \
  enable_stage_timers:=true \
  enable_yolo:=false
