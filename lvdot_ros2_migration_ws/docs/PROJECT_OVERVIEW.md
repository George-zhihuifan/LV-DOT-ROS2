# LV-DOT + QCGAF + GRU 项目说明（2026-05-12，全栈调通 + 自动评估 + u_map 调参）

本文档面向已完成 ROS2 迁移、当前在 Gazebo Fortress 仿真里跑通全栈感知 + 跟踪 + 预测链的状态。
目的是讲清楚整条链每一段在做什么、彼此怎么衔接，以及目前哪里还应该继续投入。

---

## 1. 一句话总览

> 单目深度（U-V disparity）+ 360° lidar 同时检测障碍物 → YOLO 提供视觉语义 → QC-GAF
> 把"视觉/lidar"按质量加权融合 → 卡尔曼+轨迹跟踪 → GRU 预测每个动态目标未来 2.5 秒位置。

最终对外输出三层：

| 层 | Topic | 含义 |
|---|---|---|
| 检测层 | `/onboard_detector/dynamic_bboxes` | 已分类为"动态"的目标框 |
| 融合层 | `/qcgaf/fused_bboxes` | QC-GAF 网络融合后的最终目标框 |
| 预测层 | `/gru_predictor/predicted_positions` | 每个 track 未来 5×0.5s 的轨迹 |

---

## 2. 数据流（自下而上）

```
  Gazebo Fortress 仿真世界
  ├── rgbd_camera (SDF 内置, depth_camera + camera)
  │     ├── HFOV: depth=87°, color=69° (两个相机视角不一样)
  │     └── 30 Hz, 640×480, depth=32FC1 米
  ├── gpu_lidar (SDF 内置 Livox Mid360 风格)
  │     └── PointCloud2 → /uav_lidar/scan/points
  ├── pedestrian_pose_sync_system (Gazebo 插件)
  │     └── 把 PoseArray 翻译成 actor 实体位姿，让行人 walking 动画在世界里移动
  └── 11 个 pedsim 行人 + 静态家具/车
        │
        ▼  (ros_gz_bridge 桥接)
  ROS 2 拓扑
  ├── /rgbd_camera/{image, depth_image, camera_info}
  ├── /uav_lidar/scan/points
  ├── /clock (sim_time)
  └── /mavros/local_position/{pose, odom}  ←─ lvdot_pose_stub 发布
                                              (UAV 在 Gazebo 是 <static>，
                                               pose_stub 发对应静态位姿
                                               x=1.8, y=-1.6, z=1.0)
        │
        ▼
  YOLO 节点 (lvdot_yolo_node, Ultralytics yolo11n.pt, FP16, GPU1)
  ├── 订阅 /rgbd_camera/image
  ├── 推理 imgsz=352, 每帧 ~80-300 ms (1080)
  └── 发 /yolo_detector/detected_bounding_boxes (Detection2DArray, ~8Hz)
        │
        ▼
  LV-DOT Detector (lvdot_detector_main, C++, GPU0 渲染共享)
  ├── 输入: depth_image, color_image, lidar, pose, odom, yolo_dets
  ├── 内部 4 个 stage:
  │     1. detection   → U-V disparity → u_map → depth boxes
  │     2. lidar_det.  → PointCloud2 → DBSCAN cluster → lidar boxes
  │     3. tracking    → Kalman + 匈牙利匹配 → 持续 track
  │     4. classification → 速度阈值判定 dynamic/static
  ├── 同步: ApproximateTime (slop=0.5s) 配 depth↔pose, lidar↔pose
  └── 输出 13+ marker topics:
        ├── /onboard_detector/uv_bboxes           (raw 深度框, 红, 抖)
        ├── /onboard_detector/dbscan_bboxes
        ├── /onboard_detector/lidar_bboxes        (raw lidar 框, 绿, 稳)
        ├── /onboard_detector/visual_bboxes_qcgaf ← 给 QC 用 (~14 Hz)
        ├── /onboard_detector/lidar_bboxes_qcgaf  ← 给 QC 用 (~12 Hz)
        ├── /onboard_detector/filtered_bboxes
        ├── /onboard_detector/tracked_bboxes      ← Kalman 平滑后
        └── /onboard_detector/dynamic_bboxes      ← 给 GRU 用 (~16 Hz)
        │
        ├──────────────────────┐
        ▼                      ▼
  QC-GAF Fusion          GRU Predictor
  (qcgaf_fusion_node,    (gru_prediction_node,
   PyTorch, GPU1)         PyTorch, GPU1)
  ├── 订: visual+lidar    ├── 订: dynamic_bboxes
  │     bboxes (sync     ├── 每个 track ID 维护
  │     slop=0.5)          │     一个 HybridPredictor
  ├── 7 个 quality        │     (Kalman + GRU 双引擎)
  │     score (亮度/边   ├── horizon=5 × dt=0.5s
  │     缘/深度方差/     │     = 预测 2.5s
  │     YOLO conf/...) ├── γ 自适应权重
  ├── Gated Attention   │     (近用 Kalman, 远用 GRU)
  │     融合 cam+lidar   └── 发 LINE_STRIP +
  │     两路特征              SPHERE 端点 marker
  └── 发 /qcgaf/fused_bboxes
```

---

## 3. 每个组件的角色

| 组件 | 位置 | 角色 | 状态 |
|---|---|---|---|
| `depth_eval_bringup` | ros2_depth_eval_ws | 世界 SDF、UAV 模型、Gazebo 启动、`pedestrian_pose_sync_system` 插件、桥接 | ✅ 运行良好 |
| `lvdot_realistic_sensors` | ros2_depth_eval_ws | d435i_sim / mid360_sim 噪声叠加节点 | ✅ 已接入数据链（run_detector_with_scene 默认 `use_realistic_sensors:=true`）。d435i 节点已稳定串到 detector 和 YOLO；mid360 sampler 还有质量问题（见 §6.3 新 S2） |
| `lvdot_ros2_adapter` | ros2_depth_eval_ws | YOLO 节点、pose_stub、image-pointcloud relay | ✅ |
| `lvdot_ros2` | lvdot_ros2_migration_ws | LV-DOT 原 C++ 检测器迁移版 (depth+lidar+tracking+dynamic) | ✅ |
| `lvdot_bringup` | lvdot_ros2_migration_ws | 启动文件、detector_param.yaml、RViz 配置 | ✅ |
| `qcgaf_fusion` | lvdot_ros2_migration_ws | Quality-aware Gated Attention Fusion (PyTorch) | ✅ 已修 sync_slop |
| `gru_predictor` | lvdot_ros2_migration_ws | Hybrid Kalman+GRU 预测 (PyTorch) | ✅ |
| `lvdot_interfaces` | lvdot_ros2_migration_ws | 自定义消息接口 | ✅ |
| `lvdot_core` | lvdot_ros2_migration_ws | 共享头/实用工具 | ✅ |

---

## 4. 关键性能（最新一轮：realistic_sensors 接入 + 渲染 / 性能修复后）

> **重要更正**：原 PROJECT_OVERVIEW 报的"dynamic_bboxes 16 Hz"经核查实际是 detector publish empty marker（`action=DELETEALL`）的频率，不是真实检测频率——之前 Gazebo depth 100% Inf、lidar 只见 UAV 机身（见 §5 bug 14、15），detector 一直在空载。本次修复后才第一次拿到真检测。

| 阶段 | 速率 | 备注 |
|---|---|---|
| Gazebo 相机 raw | ~14-25 Hz | depth 32FC1，米；修 UAV `<static>true</static>` 后**首次产生有效深度**（finite ~49%，range 1.4-9.7 m）；速率随 CPU 负载浮动 |
| Gazebo lidar raw | ~5-7 Hz | finite ~13.5% / 20000 点，range 0.15-60 m |
| d435i_sim /d435i/depth | **~16-19 Hz**（之前 3.6 Hz）| `array.array('B', tobytes())` 跨过 rclpy Image.data 的 per-byte type-check（~200 ms/MB → ~1 ms/MB）+ MultiThreadedExecutor + 缓存参数 |
| d435i_sim /d435i/color | ~22 Hz | 同上修复 |
| mid360_sim /mid360/pointcloud | ~6.6 Hz，**3000+ 点/帧**（之前 12-640 点） | 抛弃失效的 Risley bin-lookup，直接对 Gazebo finite 点做向量化 range 噪声 |
| YOLO (FP16, GPU1) | **~19 Hz**（之前 4 Hz） | 跟上 d435i 提速 |
| Detector dynamic_bboxes 发布速率 | ~30 Hz | publish 频率（含 DELETEALL+实际 marker） |
| Detector dynamic 计数（**真**检测） | 9 个动态目标/帧，`dyn_points=168`，`yolo_human=3` | pipeline stats 显示 `tracks=26, fused=13` |
| Detector tracked_bboxes | ~26 Hz | |
| QC fused_bboxes | 未重测 | 待后续 step |
| GRU predictions | 未重测 | 待后续 step |
| GPU0 (1080 Ti) | ~30% util | Gazebo + Ogre2 渲染 |
| GPU1 (1080) | ~4% util | YOLO + QC + GRU，**还很闲** |

---

## 5. 已修过的 Bug（按时间排序）

| # | 位置 | 修法 | 效果 |
|---|---|---|---|
| 1 | pose_stub 位姿 | orbit OFF, static 对齐 SDF UAV (1.8,-1.6,1.0) | depth_pose_sync 0%→100% |
| 2 | YOLO 权重缺失 | symlink LV-DOT/.../yolo11n.pt → QCGAF | YOLO 加载 OK |
| 3 | QC cam_topic 错 | filtered_bboxes → visual_bboxes_qcgaf | QC 看到视觉输入 |
| 4 | detector sync_slop | 0.30→0.50, depth_yolo_skew 0.8→1.5 | YOLO 软同步通 |
| 5 | u_map_min_length_line | 2→1 | 深度框更敏感 |
| 6 | GRU 空 markers return | 改成空也 publish | GRU <1Hz → 14Hz |
| 7 | Gazebo `__NV_PRIME_RENDER_OFFLOAD=1` | 移除 | 解决渲染条纹 |
| 8 | RViz 双开 | rviz:=false detector_rviz:=true | 只开一个 |
| 9 | YOLO 单卡争抢 | additional_env: CUDA_VISIBLE_DEVICES=1 | 1Hz → 8Hz |
| 10 | body_to_camera_depth 平移 | 0.18→0.30, 0.06→0.05 (对齐 SDF) | 近距离对齐 |
| 11 | color_intrinsics fx | 337→464 (匹配 RGB HFOV 69°) | YOLO 反投影准确 |
| 12 | YOLO FP16 | 加 `half=True` + warmup | min latency 124→79ms |
| 13 | QC sync_slop | 0.12→0.50 (visual↔lidar 戳差 >120ms 被丢) | QC 重新出框 |
| 14 | **UAV 模型 `<static>false</static>` → depth 全 Inf** | `uav_d435i_platform/model.sdf` 改回 `<static>true</static>` | depth finite 0% → 49%，lidar finite 1.8% → 13.5%。验证方法：把同样的 UAV 用 static=true 放最小世界（UAV+地面+1 box），depth 立即出框。原 PROJECT_OVERVIEW 误描述为"UAV 是 static"，实际 SDF 早被改成 false，导致整个 scene 一直在空载。Ogre2 server-side rendering 对非 static + `<gravity>false</gravity>` + depth_camera 同链接这个组合有 bug |
| 15 | S1: realistic_sensors 接入 | 新建 `lvdot_realistic_sensors/launch/realistic_sensors.launch.py`；`run_detector_with_scene.launch.py` 加 `use_realistic_sensors` 开关（默认 true），通过 PythonExpression 把 detector + YOLO 的 color/depth/lidar topic 切到 `/d435i/*` + `/mid360/pointcloud` | detector subs 由 `/rgbd_camera/{image,depth_image}` + `/uav_lidar/scan/points` → `/d435i/color/image_raw` + `/d435i/depth/image_rect_raw` + `/mid360/pointcloud`；A/B toggle 可回退 |
| 16 | **mid360_sim 每帧只出 12-640 点（应 20000）** | `mid360_sim.py` 抛弃失效的 1024×128 bin-lookup（Gazebo lidar 只有 ~13.5% finite，Risley 轨迹 ~99% 落空），改为对 finite 输入向量化叠加 Mid-360 σ≈2cm@25m 距离噪声，全 numpy 无 Python `for` 循环 | 12-640 → **~3000 点/帧**，detector `lidar_samples` 由 0/0 → 911/911 |
| 17 | **d435i_sim 跑不到 4 Hz（深度卡 200-260 ms/帧）** | (a) 用 `array.array('B', depth.tobytes())` 替代直接赋值 `Image.data = bytes`——rclpy 对 bytes 做 per-element type-check，1.2 MB 要 ~200 ms；array.array 走 C 层 ~1 ms。(b) MultiThreadedExecutor + 两个 callback group 让 color/depth 并行。(c) 参数缓存，避免每帧 get_parameter。(d) 跳过 `astype(float32)` 冗余转换 | `/d435i/depth` 3.6 Hz → **19 Hz**；`/yolo` 4 Hz → **19 Hz**；`bytes` 阶段 200 ms → 1 ms |
| 18 | mid360_sim 也有相同的 PointCloud2.data bytes 慢赋值问题 | `encode_pointcloud2` 内 `msg.data = array.array('B', data.tobytes())` | mid360 输出也不再被 rclpy bytes setter 拖慢 |
| 19 | **pose_stub 只发 `/mavros/local_position/pose`，Gazebo UAV 实体不动** | `pose_stub.py` 加 `publish_gazebo_cmd` 参数（默认 true），orbit_enabled=true 时同时把 PoseStamped 发到 `/uav_motion/pose_cmd`；launch 暴露 `pose_stub_orbit_enabled/radius/speed` 参数 | `UavPoseSyncSystem` 插件吃到 cmd 后用 `SetWorldPoseCmd` 移动 `<static>true</static>` UAV；orbit 25 Hz 在 (1.8,-1.6,1.0) 半径 0.4 m 圆 |
| 20 | **`detection_evaluator` 已存在但没接入 launch** | 在 `run_detector_with_adapter.launch.py` 加 `launch_evaluator/evaluator_csv_path/evaluator_match_threshold_m/evaluator_det_topic` 几个参数；`run_detector_with_scene.launch.py` 透传 | `ros2 launch ... launch_evaluator:=true` 即可滚出 `[window Nfr] prec=X rec=Y F1=Z MOTA=W mean_err=Em`，逐帧写 CSV |
| 21 | u_map 调参从未量化过 | `scripts/u_map_sweep.py` 跑参数 grid，每格起 detector+scene+evaluator 1 分钟，按 CSV 累计 TP/FP/FN 计 F1 选最优 | 16 格 grid 跑出 `(tp_pt=2, tp_ln=5, min_len=2) F1=0.20 rec=0.137` 优于默认 `(1,1,1) F1=0` —— 写进 `detector_param_tuned_v3.yaml` |

---

## 6. 个人理解 / 主观判断

### 6.1 这条 pipeline 的本质设计

LV-DOT (原作) 是个 **"双源冗余 + 时序跟踪"** 设计：单目深度提供"近距密集"，lidar 提供"远距稀疏 + 360°"，两者数据特性互补。YOLO 是后补的语义层，主要解决"这个点云团是人还是垃圾桶"。

QC-GAF 又在 LV-DOT 上加了一层"按质量学习权重"的融合——相当于给 LV-DOT 的硬规则融合换了一个学出来的版本。

GRU 是再下游一层，独立于 QC，做的是"已经被检测+跟踪的目标的未来位置"。

### 6.2 当前真正的"长板"

- ✅ realistic_sensors（d435i_sim + mid360_sim）**已接入数据链** + 性能修复（S1 + #2 + #3 done）
- ✅ Gazebo 场景 **首次产生有效传感器数据**（UAV `<static>true</static>` 修复，bug #14）
- ✅ Detector 主链端到端跑通到 `tracks=26, dynamic=9, dyn_points=168, yolo_human=3`（首次真检测）
- ✅ d435i_sim 19 Hz / YOLO 19 Hz / mid360_sim 3000+ 点/帧 — 数据流没有人为瓶颈了
- ✅ 双卡分流后 YOLO/QC/GRU 不再互相抢资源
- ✅ 整条管线从传感到预测端到端可视化 (RViz)

### 6.3 当前真正的"短板"（按严重度排序，2026-05-12 更新）

#### S 级（影响最终质量，必须修）

1. ~~**`lvdot_realistic_sensors` 形同虚设**~~ ✅ **已完成（bug #15）**
2. ~~**mid360_sim 输出几乎是空的**~~ ✅ **已完成（bug #16）**
3. ~~**d435i_sim 跑不到 30 Hz**~~ ✅ **已完成（bug #17, #18）**

4. ~~**UAV 静态（运动学验证缺失）**~~ ✅ **已完成（bug #19）** —— pose_stub 加 `publish_gazebo_cmd`，orbit 时同步发 `/uav_motion/pose_cmd`，Gazebo UAV 实体跟着圆轨动。验证：`ros2 topic echo /uav_motion/pose_cmd` 取 3 个样本 x∈[1.70, 2.12] 在 [1.4, 2.2] 期望范围内。

5. ~~**depth_yolo_sync = 0**~~ ✅ **核查后实质已修复**：详见原说明。

#### A 级（影响精度/可用性）

6. ~~**没有真实标注 → 无法量化检测精度**~~ ✅ **已完成（bug #20）**
   - 已有的 `detection_evaluator` 接进 `run_detector_with_scene.launch.py`，按行人 GT 滚出 prec/rec/F1/MOTA
   - 实测基线（**无 YOLO + 默认参数 1,1,1 + 2.5m 匹配阈值 + 30 秒 orbit**）：1202 帧, prec=0.34, rec=0.04, F1=0.08, MOTA=-0.04, mean_err=1.45m

7. ~~**u_map 调参没有自动化**~~ ✅ **已完成（bug #21）**
   - `scripts/u_map_sweep.py` 跑 grid，每格 50 秒，按 CSV 累计 TP/FP/FN 选最优
   - 16 格 grid 跑出 `(tp_pt=2, tp_ln=5, min_len=2)` F1=0.202 rec=0.137 — 写到 `detector_param_tuned_v3.yaml`

8. ~~**QC 只在某些窗口出框，整体不稳定**~~ ✅ **核查后已实现**：`fusion_node.py` 里 `enable_lidar_fallback=True`（默认），n_cam==0 时把 lidar boxes 复制到 cam slot。30 秒 sim 里 lidar_fallback 计数 = frames 计数 = 126，说明无 YOLO 时 100% 走 fallback、整段没空窗。

#### B 级（性能优化）

9. **YOLO 提速**（原 B7）
   - 已 pip 装好 `tensorrt-10.15.1` 在 GPU1 上；engine 导出本次跳过（用户判断当前优先级不够）
   - 待恢复时：`model.export(format='engine', imgsz=352, half=True, device=0)` 一行就行，再在 `lvdot_yolo_node.py` 优先加载 `.engine`

10. **GPU1 占用低**（原 B8）

### 6.4 当前评估指标快照（2026-05-12）

| 维度 | 数值 | 说明 |
|---|---|---|
| 数据流端到端 | ✅ 全通 | depth + lidar + pose + (orbit→) Gazebo entity 移动 → detector → (QC→) GRU |
| 实体运动 | ✅ orbit 25 Hz | UAV 沿 r=0.4 m 圆运动；Gazebo 物理实体确实在动 |
| Detector recall（默认参数）| 0.04 (2.5m gate) | 主要被深度范围 12 m 卡——8 个行人里只有 4 个在 UAV 视野内（距离 6-11 m） |
| Detector recall（tuned v3）| 0.137 (2.5m gate) | u_map_sweep 选最优 |
| Detector precision | 0.34 / 0.38 | 假阳很多——`fusion_components=11 lidar_only=10` 没语义的话只能按形状卡 |
| Mean position error | 1.45-1.81 m | 检测中心在胸腔（z≈1.0），GT 在脚（z≈0.02），但匹配是 2D 所以与 z 无关；xy 偏 ~1.5m |
| QC fused output | ~10 boxes/frame | lidar_fallback 100% 触发 |
| Sync warns | ~10% | \|cam-lidar\|>80ms 的帧；500ms slop 内仍可匹配 |

---

## 7. 改进路线建议（优先级排序，2026-05-12 更新）

```
现在 → ✅ S1 (realistic_sensors 接入)            ← 已完成
     → ✅ bug #14 (UAV 静态 → 修复渲染)            ← 已完成
     → ✅ 短板 #2 (mid360_sim 向量化)              ← 已完成
     → ✅ 短板 #3 (d435i_sim array.array)         ← 已完成
     → ✅ 短板 #5 (YOLO 已是 event-driven)         ← 已确认
     → ✅ 短板 #4 (UAV 真运动 via /uav_motion/pose_cmd) ← 已完成（bug #19）
     → ✅ 短板 #6 (基于 GT 的自动评估)              ← 已完成（bug #20）
     → ✅ 短板 #7 (u_map auto-tune sweep)         ← 已完成（bug #21）
     → ✅ 短板 #8 (QC 单路 fallback)               ← 已实现 + 验证
     → ⏸  性能优化 #9 (YOLO TensorRT)             ← TRT 已装好 但 engine 导出待做
     → ➡  下一步：参考 `EVALUATION_AND_ROADMAP.md`
```

---

## 8. 当前可用启动方式

```bash
# Workspace 设置
cd /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws
source /opt/ros/humble/setup.bash
source /home/mcb/LV-DOT-ROS2/ros2_depth_eval_ws/install/setup.bash
source install/setup.bash

# 1) 单纯起 detector + scene + YOLO (带 GUI)
ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  enable_yolo:=true launch_yolo_node:=true launch_pose_stub:=true \
  pose_stub_orbit_enabled:=true \
  executor_threads:=8 gazebo_gui:=true detector_rviz:=true rviz:=false

# 1b) 同上 + GT 自动评估 (rolling prec/rec/F1/MOTA 写 CSV)
ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  enable_yolo:=true launch_yolo_node:=true launch_pose_stub:=true \
  pose_stub_orbit_enabled:=true \
  launch_evaluator:=true evaluator_match_threshold_m:=2.5 \
  evaluator_csv_path:=/tmp/lvdot_eval.csv

# 1c) 用 tuned v3 (u_map_sweep 选出的最优 u_map) 替换默认 param
ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  enable_yolo:=true launch_yolo_node:=true launch_pose_stub:=true \
  pose_stub_orbit_enabled:=true \
  u_map_threshold_point:=2 u_map_threshold_line:=5 u_map_min_length_line:=2

# 1d) u_map auto-tune sweep (16 个组合，每组 ~50s)
python3 src/lvdot_bringup/scripts/u_map_sweep.py \
  --grid 1,2 1,2,3,5 1,2 --warmup 15 --collect 35 \
  --match-threshold 2.5 --output /tmp/u_map_sweep.csv

# 2) 单独起 QC 融合 (GPU1)
CUDA_VISIBLE_DEVICES=1 ros2 run qcgaf_fusion fusion_node --ros-args \
  -p config:=$PWD/install/qcgaf_fusion/share/qcgaf_fusion/config/config.yaml \
  -p checkpoint:=/home/mcb/QCGAF-GRU-UAV-Project/qcgaf_fusion/outputs/best_model.pt \
  -p verbose:=false

# 3) 单独起 GRU 预测 (GPU1)
CUDA_VISIBLE_DEVICES=1 ros2 run gru_predictor predict_node --ros-args \
  -p config:=$PWD/install/gru_predictor/share/gru_predictor/config/config_tuned.yaml \
  -p model:=/home/mcb/QCGAF-GRU-UAV-Project/gru_predictor/outputs/nuscenes_3d_tuned/best_model.pth \
  -p input_topic:=/onboard_detector/dynamic_bboxes \
  -p output_topic:=/gru_predictor/predicted_positions \
  -p horizon:=5 -p device:=cuda
```

YOLO 节点已通过 `additional_env={"CUDA_VISIBLE_DEVICES":"1"}` 在 launch 里自动钉到 GPU1。
