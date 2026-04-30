#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-$HOME/ros2_depth_eval_ws/bags/minimal_depth_eval}"

mkdir -p "${OUT_DIR}"

exec ros2 bag record \
  -o "${OUT_DIR}" \
  /rgbd_camera/image \
  /rgbd_camera/camera_info \
  /rgbd_camera/depth_image \
  /rgbd_camera/points
