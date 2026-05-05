# LV-DOT ROS2 项目交付指南

本文用于把 `/home/skbt2/lvdot_ros2_ws` 项目打包交给他人，并确保对方可复现“指标5接口适配”能力。

## 1. 交付内容建议

建议交付以下内容：

- 主项目源码目录：`lvdot_ros2_ws/`
- 依赖工作区目录（若对方没有）：`ros2_depth_eval_ws/`
- 版本说明文档（推荐）：`COMMIT_HASHES.txt`
- 本文档：`DELIVERY_GUIDE.md`
- 实现与部署文档：`METRIC5_IMPLEMENTATION_AND_DEPLOYMENT.md`

## 2. 打包方式（推荐）

在 `/home/skbt2` 下执行：

```bash
# 1) 生成源码压缩包（排除 build/install/log）
tar --exclude='*/build' --exclude='*/install' --exclude='*/log' -czf lvdot_ros2_ws_src.tar.gz lvdot_ros2_ws

# 2) 若需要同时交付场景依赖工作区
tar --exclude='*/build' --exclude='*/install' --exclude='*/log' -czf ros2_depth_eval_ws_src.tar.gz ros2_depth_eval_ws
```

如果你走 Git 交付，直接给仓库地址即可（你现在是 `LV-DOT-ROS2`，其中包含两个子目录工作区）。

## 3. 交付给对方时要附带的信息

请同时给对方：

- ROS2 发行版：`jazzy`
- Ubuntu 版本（你的开发环境）
- GPU/CPU 推理条件（YOLO是否走CPU）
- 启动脚本入口：
  - 指标5适配一键启动：`src/lvdot_bringup/scripts/start_metric5_interface_stack.sh`
- 对外接口话题：
  - 3D：`/metric5/detection_3d_marker` (`visualization_msgs/Marker`, `type=CUBE`)
  - 2D：`/metric5/detection_2d_image` (`sensor_msgs/Image`)

## 4. 交付后的验收最小步骤（给对方）

```bash
cd ~/lvdot_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lvdot_bringup --symlink-install
source install/setup.bash

# 一键启动
./src/lvdot_bringup/scripts/start_metric5_interface_stack.sh
```

新开终端验收话题类型：

```bash
source /opt/ros/jazzy/setup.bash
source ~/lvdot_ros2_ws/install/setup.bash
ros2 topic info /metric5/detection_3d_marker
ros2 topic info /metric5/detection_2d_image
```

期望输出：

- `/metric5/detection_3d_marker` -> `visualization_msgs/msg/Marker`
- `/metric5/detection_2d_image` -> `sensor_msgs/msg/Image`

