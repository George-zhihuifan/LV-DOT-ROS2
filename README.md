# LV-DOT 项目使用指南

> ROS2 Humble + Gazebo Fortress 上的动态行人检测仿真平台。
> 包含场景、UAV、行人、RealSense D435i 仿真相机、Livox Mid-360 仿真激光雷达、QCGAF + GRU 融合检测。

---

## 系统要求

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble |
| Gazebo | Fortress (Sim 6.16) |
| Python | 3.10+ |
| GPU | NVIDIA（建议，用于 OGRE 渲染） |

确认环境：
```bash
lsb_release -a            # 应该是 Ubuntu 22.04
ls /opt/ros/humble        # 必须存在
ign gazebo --version      # 应输出 6.16.x
```

---

## 一次性构建

```bash
# 1. 构建 ros2_depth_eval_ws（仿真场景 + 传感器仿真节点）
cd $LVDOT_ROOT/ros2_depth_eval_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 2. 构建 lvdot_ros2_migration_ws（detector + QCGAF + GRU）
cd $LVDOT_ROOT/lvdot_ros2_migration_ws
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
colcon build --symlink-install
```

如果只改动了某个包，可以单独构建：
```bash
colcon build --packages-select depth_eval_bringup
colcon build --packages-select lvdot_realistic_sensors
```

---

## 启动完整 pipeline

每次新开 terminal **先 source 三个环境**：

```bash
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source $LVDOT_ROOT/lvdot_ros2_migration_ws/install/setup.bash
```

### 启动方式 1：完整端到端（推荐）

```bash
ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  enable_yolo:=false \
  launch_yolo_node:=false \
  rviz:=true \
  detector_rviz:=false
```

参数含义：
| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_yolo` | false | 是否启用 YOLO 视觉检测分支（需要权重，false 时只走 UV-map + DBSCAN） |
| `launch_yolo_node` | false | 是否启动 YOLO 节点（与上面同步设置） |
| `rviz` | false | 是否开启 RViz2 可视化 |
| `detector_rviz` | false | 是否同时开 detector 自己的 RViz |
| `enable_uav_controller` | false | 是否运行 UAV 轨迹控制器（让无人机动起来） |
| `publish_states` | true | 是否启动行人状态发布（让行人动起来） |
| `gazebo_gui` | false | 是否开 Gazebo GUI（默认无头） |

### 启动方式 2：分步调试

```bash
# Terminal 1: 仿真场景 + bridge + relay
ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py \
  publish_states:=true \
  enable_uav_controller:=true \
  relay_lvdot_topics:=true \
  rviz:=true

# Terminal 2: pose stub（让 detector 有位姿输入）
ros2 run lvdot_ros2_adapter pose_stub --ros-args -p use_sim_time:=true

# Terminal 3: detector
ros2 launch lvdot_bringup run_detector.launch.py \
  enable_yolo:=false launch_yolo_node:=false launch_relay:=false launch_pose_stub:=false

# 可选 Terminal 4: 真实硬件级 D435i / Mid-360 仿真节点（叠加噪声/扫描模式）
ros2 run lvdot_realistic_sensors d435i_sim --ros-args -p use_sim_time:=true
ros2 run lvdot_realistic_sensors mid360_sim --ros-args -p use_sim_time:=true
```

---

## 关键 topic 一览

### 仿真传感器输入（来自 Gazebo）
| Topic | 类型 | 频率 | 内容 |
|-------|------|------|------|
| `/rgbd_camera_color` | `sensor_msgs/Image` | 30 Hz | D435i 风格 RGB |
| `/rgbd_camera_depth` | `sensor_msgs/Image (32FC1)` | 30 Hz | D435i 风格深度（米） |
| `/uav_lidar/scan/points` | `sensor_msgs/PointCloud2` | 10 Hz | 360° lidar 原始密集栅格 |
| `/camera/imu` | `sensor_msgs/Imu` | 200 Hz | D435i 风格 IMU |
| `/clock` | `rosgraph_msgs/Clock` | ~600 Hz | sim 时钟 |

### LV-DOT 兼容拓扑（relay 翻译后）
| Topic | 类型 | 内容 |
|-------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 给 detector 的彩色图 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image (32FC1)` | 给 detector 的深度图 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 内参 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 内参 |
| `/livox/lidar/pointcloud` | `sensor_msgs/PointCloud2` | 给 detector 的点云 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | UAV 位姿（pose_stub） |

### detector 输出
| Topic | 内容 |
|-------|------|
| `/onboard_detector/dynamic_bboxes` | 动态目标 marker array（人/车） |
| `/onboard_detector/lidar_bboxes` | lidar 分支检测框 |
| `/onboard_detector/visual_bboxes` | 视觉分支检测框 |
| `/onboard_detector/tracked_bboxes` | 跟踪框 |
| `/onboard_detector/raw_dynamic_point_cloud` | 动态点云 |
| `/onboard_detector/pipeline_stats` | pipeline 性能/计数 |

### QCGAF + GRU 输出
| Topic | 内容 |
|-------|------|
| `/qcgaf/fused_bboxes` | 融合后输出框 |

---

## 真实硬件级 D435i / Mid-360 仿真节点

包 `lvdot_realistic_sensors` 提供两个节点，在 Gazebo 原始 sensor 上叠加真实硬件特性：

### `d435i_sim`
- 二次方深度噪声 σ(z) = 0.0014 × z² 
- 5% 随机像素 dropout（模拟 D435i 暗/反光表面丢点）
- 量化到 1mm（真实 D435i 深度分辨率）
- 发布带 D435i 真实内参的 CameraInfo

订阅 `/rgbd_camera_color`、`/rgbd_camera_depth`  
发布 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、`/camera/*/camera_info`

### `mid360_sim`
- **非重复 Risley 双棱镜扫描轨迹**（rose-curve），每帧 20K 点
- 360° 水平 × -7°~+52° 垂直 FOV
- 0.1-70m 测距
- 距离噪声 σ ≈ 2cm @ 25m

订阅 `/uav_lidar/scan/points`  
发布 `/livox/lidar/pointcloud`

启动方式：
```bash
ros2 run lvdot_realistic_sensors d435i_sim --ros-args -p use_sim_time:=true
ros2 run lvdot_realistic_sensors mid360_sim --ros-args -p use_sim_time:=true
```

注意：用了这两个节点后，关掉 `image_pointcloud_relay`（设 `relay_lvdot_topics:=false`），避免拓扑冲突。

---

## 验证 pipeline 是否正常

启动后**所有 8 个指标必须正常**：

| # | 指标 | 检查命令 | 通过标准 |
|---|------|---------|---------|
| 1 | Gazebo 加载 | `ign topic -l` | 列出 `/rgbd_camera_color` / `/uav_lidar/scan/points` / `/clock` |
| 2 | RGB 渲染 | `python3 check_sensors.py`（见日志） | `std > 5`, 颜色 unique > 50 |
| 3 | 深度有效率 | 同上 | `finite > 20%`, depth ∈ [0.1, 10m] |
| 4 | Lidar 命中场景 | `ros2 topic echo /uav_lidar/scan/points` | > 5000 个非自反射点 |
| 5 | Relay 拓扑 | `ros2 topic hz /camera/color/image_raw` | ≥ 25 Hz |
| 6 | Pose 同步 | detector log `depth_pose_sync` | > 0 且增长 |
| 7 | Detector 输出 | `ros2 topic hz /onboard_detector/dynamic_bboxes` | ≥ 20 Hz |
| 8 | QCGAF 融合 | `ros2 topic hz /qcgaf/fused_bboxes` | ≥ 10 Hz |

---

## 常见问题

### Q1: 启动后 RGB 全是灰色 (178,178,178)
原因：UAV 模型 `<static>false</static>` 在物理引擎下漂走了。  
解决：检查 `models/uav_d435i_platform/model.sdf` 第 4 行必须是 `<static>true</static>`。

### Q2: detector 一直 idle，pipeline_stats 全 0
原因：`enable_stage_timers: false` 或 `enable_yolo_sync: true`（但 YOLO 不发布）。  
解决：`config/detector_param.yaml` 设 `enable_stage_timers: true` 和 `enable_yolo_sync: false`。

### Q3: pose=0 odom=0
原因：pose_stub 没启动，或时间戳不匹配。  
解决：
1. `launch_pose_stub:=true`
2. pose_stub 在 `/clock` 流通后启动，且 `use_sim_time:=true`

### Q4: `ros-jazzy-*` 装不上
本机是 Ubuntu 22.04，**只能用 ROS2 Humble**。要用 Jazzy 需要升级到 Ubuntu 24.04 或用 Docker 容器。

### Q5: 重启 Gazebo 时报 `port busy`
```bash
killall -9 ign gazebo gz parameter_bridge ruby
sleep 3
```
然后重新启动。

---

## 调试日志位置

- Gazebo 服务端日志：`~/.ignition/gazebo/log/<timestamp>/server_console.log`
- Ogre 渲染日志：`~/.ignition/rendering/ogre.log`
- ROS2 节点日志：`~/.ros/log/<timestamp>/`
- 本项目过程记录：`<外部资料目录>/毕设/log/`

---

## 工程改造记录

本项目原本写给 **ROS2 Jazzy + Gazebo Harmonic + SDF 1.10**，但运行机器只有 Humble + Fortress + SDF 1.9，因此做了以下降级改造（详见 `log/2026-05-11.md`）：

1. **世界 SDF 插件**：`gz-sim-*` / `gz::sim::*` → `ignition-gazebo-*` / `ignition::gazebo::*`
2. **C++ 插件**：源码 namespace 全 port 到 `ignition::gazebo`
3. **Bridge 类型**：`gz.msgs.*` → `ignition.msgs.*`
4. **rgbd_camera 拆分**：Fortress 不支持组合类型，拆成独立 `camera` + `depth_camera`
5. **UAV `<static>true</static>`**：避免物理漂移导致相机掉视野
6. **添加 `<scene>` ambient/background**：保证渲染管线初始化正常
7. **detector_param.yaml**：`enable_stage_timers: true`、`enable_yolo_sync: false`

---

## 联系

如遇问题，参考 `<外部资料目录>/毕设/log/2026-05-11.md` 详细调试记录。
