# LV-DOT ROS2 在 Ubuntu 22.04 的迁移与部署说明

> 目标：在**全新 Ubuntu 22.04** 机器上部署本项目，跑通“场景 + 检测 + 融合（含 QC 可选）+ RViz”。

## 1. 总体结论（先看）

- 建议系统：`Ubuntu 22.04.5 LTS`（原生双系统/物理机优先）
- 建议 ROS2：`Humble`
- 建议仿真：`Gazebo (gz sim) + ros_gz_bridge`（Humble 对应版本）
- 不建议：WSL 作为最终性能评估环境（可开发，稳定性/时序不如原生）

本项目在当前开发机用的是 `ROS2 Jazzy`，而 Ubuntu22 官方长期稳定组合是 `ROS2 Humble`。迁移时应按 Humble 重新安装与编译。

---

## 2. 迁移内容与目录

需要拷贝这 3 个工作空间：

- `/home/skbt2/lvdot_ros2_ws`
- `/home/skbt2/ros2_depth_eval_ws`
- `/home/skbt2/lvdot_ros2_migration_ws`（若要使用 qcgaf/gru）

推荐目标路径（新机器）：

- `~/lvdot_ros2_ws`
- `~/ros2_depth_eval_ws`
- `~/lvdot_ros2_migration_ws`

---

## 3. 系统与基础依赖安装

## 3.1 系统更新

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release software-properties-common git
```

## 3.2 安装 ROS2 Humble

按 ROS2 官方 Humble 安装流程执行（apt 源 + desktop）。

安装后验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

## 3.3 开发工具与常用库

```bash
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-pip python3-venv \
  libeigen3-dev libopencv-dev \
  libpcl-dev pcl-tools \
  ros-humble-cv-bridge ros-humble-vision-msgs \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-rviz2
```

初始化 rosdep：

```bash
sudo rosdep init || true
rosdep update
```

---

## 4. Gazebo / ros_gz 依赖

本项目使用 `gz sim` 与 `ros_gz_bridge`。

在 Humble 上安装对应 ros_gz 包（版本以 apt 可用项为准）：

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
```

验证：

```bash
gz sim --help
ros2 pkg list | grep ros_gz
```

---

## 5. Python 依赖（YOLO / QC / GRU）

## 5.1 基础 Python 包

```bash
python3 -m pip install --upgrade pip
python3 -m pip install numpy scipy pyyaml
```

## 5.2 YOLO 相关

```bash
python3 -m pip install ultralytics
```

## 5.3 QC/GRU（如启用）

```bash
python3 -m pip install torch torchvision
```

> 若有 NVIDIA GPU，请按 PyTorch 官方命令安装对应 CUDA 版本。

---

## 6. 编译顺序（非常重要）

必须按以下顺序：

1. `ros2_depth_eval_ws`
2. `lvdot_ros2_ws`
3. `lvdot_ros2_migration_ws`（可选，若启用 qcgaf/gru）

## 6.1 编译 ros2_depth_eval_ws

```bash
cd ~/ros2_depth_eval_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 6.2 编译 lvdot_ros2_ws

```bash
cd ~/lvdot_ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_depth_eval_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 6.3 编译 lvdot_ros2_migration_ws（可选）

```bash
cd ~/lvdot_ros2_migration_ws
source /opt/ros/humble/setup.bash
source ~/ros2_depth_eval_ws/install/setup.bash
source ~/lvdot_ros2_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

---

## 7. 环境 source 规范

建议写入 `~/.bashrc`（按需）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_depth_eval_ws/install/setup.bash
source ~/lvdot_ros2_ws/install/setup.bash
# 如需 qcgaf/gru 再加：
# source ~/lvdot_ros2_migration_ws/install/setup.bash
```

若不写 `~/.bashrc`，每次新终端按上面顺序手动 source。

---

## 8. 首次运行与验收

## 8.1 运行全链路（规则融合）

```bash
cd ~/lvdot_ros2_ws
bash scripts/start_full_stack.sh
```

检查：

```bash
ros2 topic list | grep onboard_detector
ros2 topic hz /rgbd_camera/image
ros2 topic hz /livox/lidar/pointcloud
```

## 8.2 运行 QC 接管融合（已提供脚本）

```bash
cd ~/lvdot_ros2_ws
bash scripts/start_qc_takeover_stack.sh
```

关键输出话题：

- `/onboard_detector/filtered_bboxes_qc`

---

## 9. 常见问题与处理

## 9.1 RViz 闪退 / 无窗口

- 检查 `DISPLAY` 与 GPU 驱动
- 先单独运行 `rviz2` 看是否能启动
- 看日志：`/tmp/*rviz*.log`

## 9.2 “topic does not appear to be published yet”

- 通常是场景或检测节点未成功启动
- 先查进程：

```bash
pgrep -af 'uav_pedestrian_prototype|lvdot_detector_main|parameter_bridge|gz sim'
```

## 9.3 融合退化（频繁 fallback）

- 看 `/tmp/*detector*.log` 中 `Depth/LiDAR skew` / `Depth/YOLO skew`
- 如果频繁超阈值，优先降负载和放宽时序门限

## 9.4 WSL 性能/时序不稳定

- 结论：可开发，不建议作为最终评测环境
- 正式评测建议迁移到原生 Ubuntu22 + NVIDIA 驱动

---

## 10. 建议的“交付打包”方式

推荐发给对方：

1. 仓库地址（含分支/tag）
2. 本文档 `UBUNTU22_DEPLOYMENT_GUIDE.md`
3. 模型权重路径说明：
   - `QCGAF`: `.../qcgaf_fusion/outputs/best_model.pt`
   - `GRU`: `.../gru_predictor/outputs/.../best_model.pth`
4. 一键脚本清单：
   - `scripts/start_full_stack.sh`
   - `scripts/start_qc_takeover_stack.sh`

---

## 11. 最小验收标准（建议）

- 能稳定启动场景 + 检测 + RViz（连续 3 次）
- `/onboard_detector/filtered_bboxes` 持续发布
-（QC 模式）`/onboard_detector/filtered_bboxes_qc` 持续发布
- 30 秒内无大规模进程退出（gazebo / detector / qcgaf）

