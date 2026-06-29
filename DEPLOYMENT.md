# LV-DOT-ROS2 部署指南 (DEPLOYMENT)

本文档面向**从 GitHub clone 本仓库后从零部署**的场景,说明环境依赖、编译、路径调整与最小可运行验证。
若你只是想了解已部署环境下如何启动/调试完整链路,请看
`lvdot_ros2_migration_ws/docs/QCGAF_PIPELINE_STARTUP.md`。

> 仓库地址: https://github.com/George-zhihuifan/LV-DOT-ROS2

---

## 1. 仓库内容与边界

本仓库只包含**代码 / 配置 / 启动脚本 / 仿真模型 / 文档 / 小体积模型权重**,
**不包含**实验录包(ROS bag / `.db3`)、评估产物、日志等大文件(这些在 `.gitignore` 中排除)。

三个工作区:

| 工作区 | 作用 | 是否主线 |
|---|---|---|
| `lvdot_ros2_migration_ws` | **主线**:LV-DOT 检测器 + QC-GAF 融合 + GRU 预测 + bringup/launch | ✅ 主线 |
| `ros2_depth_eval_ws` | Gazebo 仿真场景、传感器仿真(d435i/mid360)、评估工具、ros_gz 桥 | ✅ 仿真依赖 |
| `lvdot_ros2_ws` | 早期工作区,已被 migration_ws 取代 | ⚠️ 旧,可忽略 |

已随仓库提供的小体积模型权重(无需另外下载):

- `lvdot_ros2_migration_ws/models/qcgaf/best_model.pt` — QC-GAF 融合头
- `lvdot_ros2_migration_ws/models/gru/best_model.pth` — GRU 预测器
- `lvdot_ros2_migration_ws/models/yolo/yolo11n.pt` / `.onnx` / `.engine` — YOLO 检测器

---

## 2. 环境要求

| 组件 | 版本/说明 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | **Humble** |
| Gazebo | **Ignition / `ign gazebo`**(physics: `ignition-physics5-dartsim-plugin`,对应 Fortress) |
| Python | 3.10(随 Humble) |
| 编译工具 | `colcon` |

### 2.1 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz-bridge ros-humble-ros-gz-sim \
  ros-humble-vision-msgs \
  python3-colcon-common-extensions python3-rosdep
```

### 2.2 Python 依赖

主线节点依赖 PyTorch 与 Ultralytics(QC-GAF / GRU / YOLO):

```bash
pip install torch numpy ultralytics
```

> GPU 推理需自行安装匹配 CUDA 的 torch;CPU 也可运行,推理较慢。

### 2.3 rosdep(可选,自动补齐声明式依赖)

```bash
sudo rosdep init   # 首次
rosdep update
cd ~/LV-DOT-ROS2
rosdep install --from-paths ros2_depth_eval_ws/src lvdot_ros2_migration_ws/src \
  --ignore-src -r -y
```

---

## 3. 获取代码

```bash
cd ~
git clone https://github.com/George-zhihuifan/LV-DOT-ROS2.git
cd LV-DOT-ROS2
```

---

## 4. 编译

两个工作区按顺序编译:**先 `ros2_depth_eval_ws`(提供仿真与桥),再 `lvdot_ros2_migration_ws`(主线)**。

```bash
source /opt/ros/humble/setup.bash

# 4.1 仿真/评估工作区
cd ~/LV-DOT-ROS2/ros2_depth_eval_ws
colcon build --symlink-install
source install/setup.bash

# 4.2 主线工作区
cd ~/LV-DOT-ROS2/lvdot_ros2_migration_ws
colcon build --symlink-install
source install/setup.bash
```

包构成(参考):`lvdot_core/lvdot_ros2/lvdot_interfaces/lvdot_bringup` 为 `ament_cmake`,
`qcgaf_fusion/gru_predictor` 为 `ament_python`;仿真侧 `depth_eval_bringup/lvdot_realistic_sensors/lvdot_ros2_adapter` 为 `ament_python`。

---

## 5. 路径调整(重要)

仓库内的文档与部分启动示例使用了开发机绝对路径 `/home/mcb/LV-DOT-ROS2/...`。
**clone 到其它机器/用户后,需要把这些路径换成你自己的工作区根目录。**

建议统一用一个环境变量管理:

```bash
export LVDOT_ROOT=~/LV-DOT-ROS2
```

每次新开终端,先 source 三件套:

```bash
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source $LVDOT_ROOT/lvdot_ros2_migration_ws/install/setup.bash
```

传给 launch 的 `scenario_config:=` / `detector_config:=` 等绝对路径参数,
请相应改成 `$LVDOT_ROOT/...` 下的实际文件。

---

## 6. 最小可运行验证

按从小到大三步验证部署是否成功。

### 6.1 只起 Gazebo 场景(验证仿真与桥)

```bash
ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py \
  gazebo_gui:=true rviz:=false \
  scenario_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/config/agent_count_scenarios/pedestrian_dense_01agents.yaml
```

预期:Gazebo 起来,`ros_gz_bridge` 与 `pedestrian_state_publisher` 运行。
另开终端检查话题:

```bash
ros2 topic list | grep -E "rgbd_camera|uav_lidar|pedestrian_sim"
```

### 6.2 起完整链路(检测 + QC-GAF 融合)

```bash
ros2 launch lvdot_bringup run_full_pipeline.launch.py \
  gazebo_gui:=true rviz:=true \
  use_realistic_sensors:=true \
  scenario_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/config/agent_count_scenarios/pedestrian_dense_01agents.yaml \
  detector_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml \
  enable_qcgaf:=true enable_gru:=false \
  launch_evaluator:=false launch_advanced_evaluator:=false
```

预期节点:Gazebo / 桥 / `d435i_sim` / `mid360_sim` / `pose_stub` /
`lvdot_yolo_node` / `lvdot_detector_main` / `qcgaf_fusion_node` / RViz。

### 6.3 验证关键话题有数据

```bash
ros2 topic hz /mid360/pointcloud
ros2 topic hz /onboard_detector/dynamic_bboxes
ros2 topic echo /qcgaf/fused_bboxes --once
```

健康标准:融合输出 `/qcgaf/fused_bboxes` 持续有框,而不是长期停在纯 LiDAR fallback。

---

## 7. 场景与配置切换

- **人数/密度**:把 `scenario_config:=` 换成 `config/agent_count_scenarios/` 或 `config/clean_scenarios/` 下对应 `..._0Nagents.yaml`。
- **检测器参数**:换 `detector_config:=`,可选 `detector_param_baseline.yaml` / `detector_param_tuned_v3.yaml` / `detector_param_lidar_original.yaml` 等。
- **纯 LiDAR 基线**:使用 `fusion_mode:=lidar_driven`(配合 `enable_qcgaf:=false`),评估读 `/onboard_detector/tracked_bboxes`(marker namespace = `tracked`)。
- **360° LiDAR 变体 / 静态障碍变体**:见 `launch/` 下 `*_lidar360` 与 `*_staticobs` 后缀的启动文件。

完整启动参数、launch 层级、RViz 配置、可观测话题清单见
`lvdot_ros2_migration_ws/docs/QCGAF_PIPELINE_STARTUP.md`。

---

## 8. 常见问题

| 现象 | 排查方向 |
|---|---|
| `package not found` | 没 source 对应 `install/setup.bash`,或编译顺序反了(先 depth_eval_ws 再 migration_ws) |
| Gazebo 起不来 / 物理插件报错 | 确认装的是 Ignition(`ign gazebo`)且有 `ignition-physics5-dartsim-plugin` |
| 找不到 scenario/detector 文件 | launch 参数里的绝对路径未改成本机 `$LVDOT_ROOT/...` |
| `/qcgaf/fused_bboxes` 长期空或纯 fallback | 检查 `d435i_sim` 深度有效率;相机无检测时 QC-GAF 会跳过该帧(相机锚定) |
| torch / ultralytics ImportError | 在运行节点的 Python 环境里 `pip install torch ultralytics` |

---

## 9. 与论文实验的关系(诚信说明)

本仓库是**可运行的工程主线**。论文中的实验数据来自本机大体积录包与评估产物,
这些**不随仓库分发**。若需复现论文表格/图,需要自行采集对应场景的 bag 并运行评估脚本。

> 注:论文部分实验表的口径与数据可信度仍在内部核查中,复现时请以实际跑出的评估产物为准。
