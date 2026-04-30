#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim

if ! grep -q "source /opt/ros/jazzy/setup.bash" "${HOME}/.bashrc"; then
  echo "source /opt/ros/jazzy/setup.bash" >> "${HOME}/.bashrc"
fi

echo "Install complete. Reopen the shell or run:"
echo "source /opt/ros/jazzy/setup.bash"
echo "source ${HOME}/ros2_depth_eval_ws/install/setup.bash"
