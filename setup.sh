#!/usr/bin/env bash
# LV-DOT-ROS2 一键编译脚本
# 用法: ./setup.sh
# 按正确顺序 source ROS2 → 编译仿真工作区 → 编译主线工作区
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ">>> ROS2 Humble"
source /opt/ros/humble/setup.bash

echo ">>> 编译 ros2_depth_eval_ws (仿真/传感器/桥)"
cd "$REPO_ROOT/ros2_depth_eval_ws"
colcon build --symlink-install
source install/setup.bash

echo ">>> 编译 lvdot_ros2_migration_ws (主线检测+融合)"
cd "$REPO_ROOT/lvdot_ros2_migration_ws"
colcon build --symlink-install
source install/setup.bash

echo ">>> 编译完成"
echo "每次新开终端运行以下命令即可:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $REPO_ROOT/ros2_depth_eval_ws/install/setup.bash"
echo "  source $REPO_ROOT/lvdot_ros2_migration_ws/install/setup.bash"
