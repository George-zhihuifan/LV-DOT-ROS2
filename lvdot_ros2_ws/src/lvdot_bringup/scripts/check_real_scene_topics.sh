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

echo "=== input_health ==="
timeout 5 ros2 topic echo /onboard_detector/input_health --once || true
echo

echo "=== input_health_status ==="
timeout 5 ros2 topic echo /onboard_detector/input_health_status --once || true
echo

echo "=== stage_timers ==="
timeout 5 ros2 topic echo /onboard_detector/stage_timers --once || true
echo

echo "=== stage_timers_status ==="
timeout 5 ros2 topic echo /onboard_detector/stage_timers_status --once || true
echo

echo "=== pipeline_stats ==="
timeout 5 ros2 topic echo /onboard_detector/pipeline_stats --once || true
echo

echo "=== pipeline_stats_status ==="
timeout 5 ros2 topic echo /onboard_detector/pipeline_stats_status --once || true
echo

echo "=== dynamic_obstacles service type ==="
ros2 service list -t --no-daemon | rg '^/onboard_detector/get_dynamic_obstacles\\s+\\[lvdot_interfaces/srv/GetDynamicObstacles\\]$' || true
