# lvdot_ros2_ws

这个工作空间用于 `LV-DOT -> ROS2` 迁移与回归。

边界约束：
- 不在这里放 `depth_eval` 主线代码
- 不回写 `/home/skbt2/ros2_depth_eval_ws`
- ROS1 参考保持在 `/home/skbt2/LV-DOT`

核心包：
- `lvdot_core`：ROS1 真算法核心（`DBSCAN/Kalman/UVdetector/lidarDetector`）
- `lvdot_interfaces`：ROS2 srv/msg
- `lvdot_ros2`：ROS2 detector 节点与运行时桥接
- `lvdot_bringup`：launch、rviz、回归脚本

## 真实场景回归

当前建议的最小回归流程已经固化成脚本：

- 启动场景和 detector：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/run_real_scene_regression.sh`
- 一键执行完整回归闭环并落结果：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/run_real_scene_regression_suite.sh`
- 运行前做环境预检：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/preflight_real_scene_environment.sh`
- 单独验证 detector 主链能否启动：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/smoke_test_detector_only.sh`
- 对 detector 主链做独立回归并落产物：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/run_detector_only_regression_suite.sh`
- 跑完整回归矩阵并给出总判断：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/run_regression_matrix.sh`
- 单独诊断 scene 环境阻塞项：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/run_scene_environment_diagnostics.sh`
- 分层诊断 scene 端四条主链：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/run_scene_staged_diagnostics.sh`
- 直接查看最近一次 scene 诊断报告：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/show_latest_scene_diagnostics_report.sh`
- 直接查看最近一次分层 scene 诊断报告：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/show_latest_scene_staged_report.sh`
- 直接查看最近一次 matrix 报告：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/show_latest_regression_report.sh`
- 观察流水线统计：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/watch_pipeline_stats.sh`
- 做最小检查：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/check_real_scene_topics.sh`
- 做最小判定：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/scripts/validate_real_scene_regression.sh`

当前诊断 topic 分两层：

- 兼容层：
  - `/onboard_detector/input_health`
  - `/onboard_detector/stage_timers`
  - `/onboard_detector/pipeline_stats`
- 结构化层：
  - `/onboard_detector/input_health_status`
  - `/onboard_detector/stage_timers_status`
- `/onboard_detector/pipeline_stats_status`

## RViz

现在 `lvdot_bringup` 里已经有 detector 专用 RViz 配置：

- [lvdot_detector.rviz](/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz)

可视化重点包括：

- `dynamic_bboxes`
- `dynamic_point_cloud`
- `raw_dynamic_point_cloud`
- `raw_lidar_point_cloud`
- `detected_color_image`
- `detected_depth_map`
- `detected_u_depth_map`
- `u_depth_bird_view`

启动方式：

1. 单独起 detector + RViz

```bash
ros2 launch lvdot_bringup run_detector.launch.py rviz:=true
```

2. adapter + detector + RViz

```bash
ros2 launch lvdot_bringup run_detector_with_adapter.launch.py rviz:=true
```

3. scene + detector，并且只起 detector 自己的 RViz

```bash
ros2 launch lvdot_bringup run_detector_with_scene.launch.py detector_rviz:=true rviz:=false
```

这里：

- `rviz`
  - 仍然留给 scene 那边自己的 RViz
- `detector_rviz`
  - 专门控制 `lvdot_bringup` 里的 detector RViz

脚本约定：

- 以 `/home/skbt2/ros2_depth_eval_ws` 作为 scene/adapter underlay
- 以 `/home/skbt2/lvdot_ros2_ws` 作为 LV-DOT ROS2 overlay

这样做的目的：

- 避免每次手工重复 `source`
- 避免每次重新拼 launch 参数
- 让真场景回归流程可重复

建议顺序：

1. 先运行 `preflight_real_scene_environment.sh`
2. 如果要先隔离 detector 主链问题，再运行 `smoke_test_detector_only.sh`
3. 如果要留完整 detector-only 产物，再运行 `run_detector_only_regression_suite.sh`
4. 如果要一次拿到“detector 主链”和“scene 集成”的总判断，运行 `run_regression_matrix.sh`
5. 如果还想单独看 scene 端细节，再运行 `run_scene_environment_diagnostics.sh`
6. 或者单独运行 `run_real_scene_regression_suite.sh`
7. 如需人工观察，再单独运行 `run_real_scene_regression.sh`
8. 配合 `watch_pipeline_stats.sh`
9. 必要时再手工运行 `check_real_scene_topics.sh` 和 `validate_real_scene_regression.sh`

`run_regression_matrix.sh` 的总结果当前约定为：

- `PASS`
- `DETECTOR_PASS_SCENE_ENV_BLOCKED`
- `ENVIRONMENT_RESTRICTED`
- `FAIL`

如果 matrix 返回：

- `DETECTOR_PASS_SCENE_ENV_BLOCKED`
- `ENVIRONMENT_RESTRICTED`

建议下一步直接看：

- `show_latest_scene_diagnostics_report.sh`
- `show_latest_scene_staged_report.sh`

它会单独告诉你 scene 侧更接近：

- `dds_environment_restricted`
- `gazebo_or_bridge_failed`
- `sensor_topics_missing`
- `uav_state_topics_missing`
- `scene_validation_failed`

如果你想更快知道 scene 端到底是卡在哪一层，而不是先去抓完整 topic 内容，直接运行：

- `run_scene_staged_diagnostics.sh`

它会把 scene 端拆成 4 层：

- `gazebo`
- `bridge`
- `sensors`
- `uav_state`

并在 `summary.txt` / `report.txt` 里直接给出每层 `PASS/FAIL`。 

`run_scene_environment_diagnostics.sh` 当前也会维护：

- `latest_scene_environment_diag`

并生成：

- `summary.txt`
- `report.txt`
- `report.json`

退出码约定：

- `PASS` -> `0`
- `DETECTOR_PASS_SCENE_ENV_BLOCKED` -> `0`
- `FAIL` -> `1`

运行结束后优先看：

- `artifacts/regression_matrix_*/report.txt`
- 或者直接运行：
  - `show_latest_regression_report.sh`

这个文件会直接汇总：

- `overall_result`
- `next_action`
- detector-only summary 路径
- scene regression summary 路径
- scene regression 的：
  - `ready_wait_seconds`
  - `enable_yolo`
  - `launch_yolo_node`

如果 scene suite 失败，matrix 现在会自动继续补跑：

- `run_scene_environment_diagnostics.sh`
- `run_scene_staged_diagnostics.sh`

所以：

- `show_latest_regression_report.sh`
  - 给你总判断
  - 并且现在会直接带出 scene regression 这一轮是不是已经等到 ready、是否打开了 YOLO
- `show_latest_scene_diagnostics_report.sh`
  - 给你 scene 端细粒度 topic/service/sensor 失败原因
- `show_latest_scene_staged_report.sh`
  - 给你 scene 端按 `gazebo / bridge / sensors / uav_state` 分层后的失败原因

当前 artifacts 根目录还会维护 3 个固定链接：

- `latest_detector_only_regression`
- `latest_real_scene_regression`
- `latest_regression_matrix`
- `latest_scene_environment_diag`
- `latest_scene_staged_diag`

即使这些结果是 matrix 内部嵌套跑出来的，当前也会同步刷新到全局 `latest_*` 链接，不会再出现 `show_latest_*` 读到旧结果的问题。

其中 `latest_regression_matrix` 目录下会同时有：

- `summary.txt`
- `report.txt`
- `report.json`

套件脚本会自动完成：

- 启动 scene + detector
- 等待 detector ready（不再固定只睡一个短 warmup）
- ready 后继续跑一段采样窗口
- 执行 topic 检查
- 执行最小判定
- 抓取结构化状态快照
- 抓取 service 响应
- 把结果写到 `/home/skbt2/lvdot_ros2_ws/artifacts/real_scene_regression_*`

`run_real_scene_regression_suite.sh` 当前支持这些关键环境变量：

- `READY_TIMEOUT_SECONDS`
  - 默认 `900`
- `READY_POLL_SECONDS`
  - 默认 `5`
- `POST_READY_SAMPLE_SECONDS`
  - 默认 `60`
- `ENABLE_YOLO`
  - 默认 `true`
- `LAUNCH_YOLO_NODE`
  - 默认跟随 `ENABLE_YOLO`

也就是说，scene regression 现在已经不是：

- 固定睡 `20s`
- 立刻 snapshot

而是：

1. 等待 detector 真正 ready
2. ready 后再留出采样窗口
3. 最后再做 validation

`summary.txt` 现在除了 `validation=PASS/FAIL`，还会给出：

- `failure_reason=...`

当前约定包括：

- `none`
- `dds_environment_restricted`
- `scene_or_bridge_startup_failed`
- `launch_exited_early`
- `validation_failed`

最小判定标准：

- `input_health_status`
  - `depth/color/lidar/pose/odom > 0`
- `stage_timers_status`
  - `detection/lidar_detection/tracking/classification/vis > 0`
- `pipeline_stats_status`
  - 至少能收到结构化消息
- `service`
  - `/onboard_detector/get_dynamic_obstacles`
  - 类型必须是 `lvdot_interfaces/srv/GetDynamicObstacles`

注意：

- 在受限沙箱环境里，如果看到：
  - `getifaddrs: Operation not permitted`
  - `TRANSPORT_UDP Error`
  - `parameter_bridge` / `gz sim` 直接退出
- 这不是当前 `lvdot_ros2` 主链本身的编译问题，而是运行环境对 DDS/网络接口的限制。
- 这类情况下，真实回归结果无效，应在正常桌面环境里运行 suite。
- `preflight_real_scene_environment.sh` 现在会做一轮轻量 `gz sim` smoke test。
  - 如果这里已经失败，就不应该继续跑完整 suite。

## 迁移完成度矩阵

下面这张表不是计划，是当前代码实际状态。

状态定义：

- `已对齐`
  - 语义和 ROS1 主链基本一致
- `近似实现`
  - 结构和主功能已在，但仍是最小版或简化版
- `未迁`
  - 还没有真正落代码

| 能力项 | ROS1 参考 | ROS2 当前状态 | 说明 |
| --- | --- | --- | --- |
| 算法核心包 `lvdot_core` | `dbscan / kalmanFilter / uvDetector / lidarDetector` | 已对齐 | 四个算法模块已整体搬入 `src/lvdot_core/`，纯 C++（无 ROS 依赖），`onboardDetector` 命名空间保留 |
| service 接口 `GetDynamicObstacles` | `onboard_detector/GetDynamicObstacles.srv` | 已对齐 | 请求/响应结构已一致，平面距离排序和 `velocity.z=0` 已对齐 |
| 输入订阅层 | depth/color/lidar/pose/odom/yolo | 已对齐 | 6 路输入都已接到 ROS2 节点 |
| message_filters 同步层 | depth-pose / lidar-pose / depth-odom / lidar-odom | 已对齐 | 已有 ROS2 同步骨架并接进主节点 |
| detection 阶段框架 | `detectionCB` | 已对齐 | stage timer 和状态流已接通 |
| depth 投影与 `dbscanDetect()` 主链 | `projectDepthImage -> filterPoints -> clusterPointsAndBBoxes` | 已对齐 | `cluster_points_to_bboxes` 已改成调用 `onboardDetector::DBSCAN`，bbox/center/std 提取语义与 ROS1 一致 |
| `uvDetect()` 图像侧检测 | `UVdetector` | 已对齐 | `run_uv_detector` 直接调 `onboardDetector::UVdetector::detect/extract_3Dbox`，相机→世界坐标 transform 与 `transformUVBBoxes` 一致 |
| LiDAR 检测主链 | `lidarDetect()` | 已对齐 | LiDAR 路径复用同一套 `onboardDetector::DBSCAN`，参数走 `lidar_dbscan_*`，与 `lidarDetector::lidarDBSCAN` 等价 |
| `filterLVBBoxes()` visual/lidar 融合 | `filterLVBBoxes()` | 近似实现 | 已有连通组合并、cluster 保留和 one-to-many split 最小版 |
| YOLO one-to-many split | ROS1 split 逻辑 | 近似实现 | 已按 detection 中心分样本并重建子框，但仍是简化版 |
| tracking 主链 | `trackingCB` | 近似实现 | 已有 feature association、history、filter state |
| Kalman/filter 历史层 | `kalmanFilterAndUpdateHist` / `kalmanFilterMatrixAcc` / `getKalmanObservationAcc` | 已对齐 | Track 内嵌 `onboardDetector::kalman_filter`，6×6 const-accel 矩阵 + 历史观测构造与 ROS1 等价 |
| classification 主链 | `classificationCB` | 近似实现 | 已有 velocity、voting、force-dynamic、consistency、size constrain |
| bbox 输出层 | uv/db/visual/lidar/filtered/tracked/dynamic | 已对齐 | 主要 bbox 话题都已接上 |
| 点云输出层 | filtered/depth/dynamic/raw_dynamic/downsampled/raw_lidar/lidar_clusters | 已对齐 | 主语义已按 ROS1 收口 |
| 图像输出层 | detected depth/U-map/bird/color | 已对齐 | `detected_depth_map` / `detected_u_depth_map` 直接读取 `UVdetector::depth_show / U_map_show`，与 ROS1 `publishUVImages` 同源 |
| 轨迹/速度可视化 | history / velocity markers | 已对齐 | marker 输出已接上 |
| 结构化诊断 | 无对应 | 已增强 | ROS2 新增结构化 `*_status` msg，便于回归和自动检查 |
| 真场景回归脚本 | 无统一脚本 | 已增强 | ROS2 新增启动/检查/验证脚本链 |
| RViz 配置完整迁移 | ROS1 rviz | 近似实现 | 已有 detector 专用 RViz 配置，但布局和观感还不是 ROS1 等价 |
| 原 ROS1 `dynamicDetector` 代码逐行等价迁移 | `dynamicDetector.cpp/h` | 未迁 | 当前路线是”算法模块原样复用 + ROS2 包装层重写”，外壳不是逐行平移 |

### 当前判断

如果目标是：

- **先做可运行、可回归、可继续迭代的 ROS2 版本**
  - 这条线已经基本成立

如果目标是：

- **和 ROS1 `dynamicDetector` 完全等价**
  - 算法核心（DBSCAN / Kalman / UVdetector / lidarDetector）已经是原样迁移
  - `dynamicDetector` 外壳的 `filterLVBBoxes` / tracking association / classification 细节仍是近似实现

最大的剩余差距主要在：

- `filterLVBBoxes()` 融合细节与 one-to-many split
- tracking association（特征权重 / best-match）与历史管理
- classification 细节
- RViz 观感层

### `lvdot_core` 说明

`src/lvdot_core/` 是从 ROS1 原仓库 `/home/skbt2/LV-DOT/onboard_detector/include/` 搬进来的纯算法包，约定：

- 命名空间保持 `onboardDetector`
- 无 ROS1 / ROS2 依赖，只依赖 Eigen / OpenCV / PCL
- 头文件改名为小写 + snake_case（`dbscan.hpp` / `kalman_filter.hpp` / `uv_detector.hpp` / `lidar_detector.hpp` / `box3d.hpp`）
- 实现代码和原仓库算法等价，只修改了 include 路径和一处 `using namespace std` 作用域

`lvdot_ros2` 通过 `lvdot_core` 使用这些真算法，`Box3D` 和 `onboardDetector::box3D` 之间有 `to_core_box3d / from_core_box3d` 桥接 helper。

## 稳定启动流程（推荐）

以下流程是当前最稳、最省事的日常用法，目标是避免 GUI 闪退和“行人闪现”复发。

### 0) 一次性前提

- 在桌面会话终端运行（需要 `DISPLAY` 和 `XDG_RUNTIME_DIR`）
- 使用统一入口脚本：`/home/skbt2/start_full_stack.sh`

### 1) 每次启动前先清场

```bash
/home/skbt2/start_full_stack.sh stop
```

### 2) 一键启动 scene + detector + RViz

```bash
/home/skbt2/start_full_stack.sh
```

说明：
- 脚本会清理旧的 `gz sim` / bridge / relay / yolo / rviz 相关进程，避免多实例冲突。
- 脚本会输出日志路径：
  - `/tmp/full_stack_scene.log`
  - `/tmp/full_stack_lvdot.log`
  - `/tmp/full_stack_rviz.log`

### 3) 快速健康检查（建议）

```bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
ros2 node list | rg -n "pedestrian|lvdot|pose_sync"
ros2 topic list | rg -n "onboard_detector|pedestrian_sim|rgbd_camera"
```

期望：
- 只存在一套 scene 主实例（不要重复 `gz sim -s -r`）
- `pedestrian_state_publisher` / `pedestrian_pose_sync_system` 不应出现跨实例叠加

### 4) 退出

```bash
/home/skbt2/start_full_stack.sh stop
```

### 常见故障与结论

- 现象：Gazebo 里行人“闪现/瞬移”，一段时间后又复发  
  根因：历史残留进程 + 新启动进程并行，导致同名行人模型被多路状态交替写入（不是单纯算法帧率问题）。

- 现象：脚本启动时 Gazebo/RViz 秒退  
  根因：曾经的 `set -u` 触发 ROS2 setup 链 `unbound variable`；现已在脚本中修复为兼容模式。
