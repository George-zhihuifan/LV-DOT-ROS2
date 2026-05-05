## 1. 需求对齐

上层要求：

- 输入：`sensor_msgs/Image` 原始图像 + `sensor_msgs/PointCloud2` 原始点云
- 输出：
  - 3D：`visualization_msgs/Marker`（`type = Marker::CUBE`）
  - 或 2D：绘制后的检测图 `sensor_msgs/Image`



## 2. 实现方式

### 2.1 适配节点

文件：`src/lvdot_bringup/scripts/metric5_interface_adapter.py`

核心逻辑：

1. 订阅输入源：
- 原图：`/rgbd_camera/image`（可参数化）
- 原始点云：`/uav_lidar/scan/points`（可参数化）

2. 订阅检测结果：
- 2D检测图：`/yolo_detector/detected_image`
- 3D框输入：`/onboard_detector/tracked_bboxes`（默认）

3. 输出统一接口：
- 3D 输出：`/metric5/detection_3d_marker`（`visualization_msgs/Marker`）
  - 每个目标生成一个 `CUBE`
  - 来源：从原 `LINE_LIST` 框点集合反算 `min/max` 后得到中心与尺寸
- 2D 输出：`/metric5/detection_2d_image`（`sensor_msgs/Image`）
  - 直接发布 YOLO 已绘制图

4. 支持模式：
- `output_mode=3d`：仅3D
- `output_mode=2d`：仅2D
- `output_mode=both`：同时输出（默认）

### 2.2 启动入口

- Launch：`src/lvdot_bringup/launch/run_metric5_interface.launch.py`
- 一键脚本：`src/lvdot_bringup/scripts/start_metric5_interface_stack.sh`

设计点：

- `launch_detector_stack:=true/false` 可切换是否连同检测链一起启动
- 默认启动全链并加载适配节点

## 3. 关键话题映射

### 输入（可参数覆盖）

- `input_image_topic` 默认 `/rgbd_camera/image`
- `input_pointcloud_topic` 默认 `/uav_lidar/scan/points`
- `input_detected_image_topic` 默认 `/yolo_detector/detected_image`
- `input_boxes_topic` 默认 `/onboard_detector/tracked_bboxes`

### 输出（对外统一接口）

- `output_marker_topic` 默认 `/metric5/detection_3d_marker`
- `output_image_topic` 默认 `/metric5/detection_2d_image`

## 4. 部署步骤

## 4.1 环境准备

要求：

- Ubuntu + ROS2 Jazzy
- 已具备 `lvdot_ros2_ws`（及需要时 `ros2_depth_eval_ws`）

## 4.2 编译

```bash
cd ~/lvdot_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lvdot_bringup --symlink-install
source install/setup.bash
```

## 4.3 启动

### 方式A：一键全链启动（推荐）

```bash
~/lvdot_ros2_ws/src/lvdot_bringup/scripts/start_metric5_interface_stack.sh
```

### 方式B：仅启动适配层（接已有上游）

```bash
source /opt/ros/jazzy/setup.bash
source ~/lvdot_ros2_ws/install/setup.bash
ros2 launch lvdot_bringup run_metric5_interface.launch.py launch_detector_stack:=false output_mode:=both
```

## 4.4 验收

```bash
source /opt/ros/jazzy/setup.bash
source ~/lvdot_ros2_ws/install/setup.bash

ros2 topic info /metric5/detection_3d_marker
ros2 topic info /metric5/detection_2d_image
```

期望类型：

- `/metric5/detection_3d_marker` -> `visualization_msgs/msg/Marker`
- `/metric5/detection_2d_image` -> `sensor_msgs/msg/Image`

可进一步检查3D marker字段：

```bash
ros2 topic echo /metric5/detection_3d_marker --once
```

应看到：

- `type: 1`（CUBE）
- `action: 0`（ADD）

## 5. 参数调优建议

- `source_ns_prefix`：默认 `tracked`，表示取跟踪框作为3D输出源
- 如果要直接输出检测框可改为对应命名空间
- `cube_alpha`：3D框透明度
- `cube_lifetime_sec`：显示时长

## 6. 风险与边界说明

- 当前3D CUBE是从上游线框几何反算得到，姿态按轴对齐（`orientation.w=1`）
- 若上游未来改为旋转框，需要在适配层补充姿态重建逻辑
- 适配层不改变核心检测/跟踪结果，仅做接口标准化

