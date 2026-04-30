#!/usr/bin/env bash
set -euo pipefail

AUTO_LAUNCH=false
SAMPLE_SEC=8
for arg in "$@"; do
  case "$arg" in
    --auto-launch) AUTO_LAUNCH=true ;;
    --sample=*) SAMPLE_SEC="${arg#*=}" ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source /home/skbt2/lvdot_ros2_ws/install/setup.bash
set -u

LAUNCH_PID=""
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    pkill -9 -P "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "${AUTO_LAUNCH}" == "true" ]]; then
  # strict clean
  pkill -9 -f 'run_detector_with_scene.launch.py|uav_pedestrian_prototype.launch.py|lvdot_detector_main|parameter_bridge|^gz sim' || true
  sleep 1

  ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false rviz:=false detector_rviz:=false \
    enable_yolo:=false launch_yolo_node:=false \
    > /tmp/quick_check_depth_lidar_chain.log 2>&1 &
  LAUNCH_PID=$!
  sleep 12
fi

echo "[check] camera_info"
CAMERA_INFO=$(timeout 8 ros2 topic echo /rgbd_camera/camera_info --once 2>/dev/null || true)
if [[ -z "${CAMERA_INFO}" ]]; then
  echo "FAIL camera_info_missing"
  exit 1
fi
FX=$(echo "${CAMERA_INFO}" | awk '/k:/{f=1;next} f&&/^- /{print $2}' | sed -n '1p')
FY=$(echo "${CAMERA_INFO}" | awk '/k:/{f=1;next} f&&/^- /{print $2}' | sed -n '5p')
CX=$(echo "${CAMERA_INFO}" | awk '/k:/{f=1;next} f&&/^- /{print $2}' | sed -n '3p')
CY=$(echo "${CAMERA_INFO}" | awk '/k:/{f=1;next} f&&/^- /{print $2}' | sed -n '6p')
echo "fx=${FX} fy=${FY} cx=${CX} cy=${CY}"

python3 - <<'PY'
import rclpy, numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

rclpy.init()
node=rclpy.create_node('quick_depth_stats')
q=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
msg_box=[]

def cb(msg):
    msg_box.append(msg)

node.create_subscription(Image,'/rgbd_camera/depth_image',cb,q)
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.2)
    if msg_box:
        break

if not msg_box:
    print('FAIL depth_image_missing')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

m=msg_box[-1]
a=np.frombuffer(bytes(m.data),dtype=np.float32)
finite=np.isfinite(a)
valid=finite & (a>0.2) & (a<12.0)
print(f'depth finite_ratio={finite.mean():.4f} valid_ratio={valid.mean():.4f} inf_ratio={np.isinf(a).mean():.4f}')
if valid.mean() < 0.01:
    print('FAIL depth_valid_ratio_too_low')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(2)

node.destroy_node(); rclpy.shutdown()
PY

echo "[check] detector stats"
PST=$(timeout 8 ros2 topic echo /onboard_detector/pipeline_stats_status --once 2>/dev/null || true)
if [[ -z "${PST}" ]]; then
  echo "FAIL pipeline_stats_missing"
  exit 1
fi
LRAW=$(echo "${PST}" | awk '/raw_lidar_sample_count:/{print $2}')
LFIL=$(echo "${PST}" | awk '/filtered_lidar_sample_count:/{print $2}')
DBOX=$(echo "${PST}" | awk '/db_bbox_count:/{print $2}')
LBOX=$(echo "${PST}" | awk '/lidar_bbox_count:/{print $2}')
FBOX=$(echo "${PST}" | awk '/filtered_bbox_count:/{print $2}')
echo "raw_lidar=${LRAW} filtered_lidar=${LFIL} db_bbox=${DBOX} lidar_bbox=${LBOX} fused_bbox=${FBOX}"

if [[ "${LRAW:-0}" -le 0 ]]; then
  echo "FAIL lidar_stream_zero"
  exit 3
fi

echo "PASS quick_check_depth_lidar_chain"
