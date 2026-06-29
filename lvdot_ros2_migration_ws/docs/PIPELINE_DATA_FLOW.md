# 全栈数据流与算法说明（主线管道）

> 2026-05-12 第一版。对应 `run_full_pipeline.launch.py` 启动的 11 个进程组成的链路。
> 主线：**深度相机 + lidar（带 d435i_sim/mid360_sim 真实噪声）→ depth U-V + YOLO 2D + lidar DBSCAN → QC-GAF 融合 + Kalman/Hungarian 跟踪 → 动态分类 → Kalman+GRU 预测**。

---

## 1. 端到端数据流图

```
┌─────────── Gazebo Sim ───────────────────────┐
│ pedestrian_prototype.sdf (8 个行人 + UAV)    │
│ • UavPoseSyncSystem 插件                     │
│ • PedestrianPoseSyncSystem 插件              │
│ • rgbd_camera (raw depth/color)              │
│ • gpu_lidar (raw Livox Mid-360 风格)         │
└────────────────────┬─────────────────────────┘
                     │ (Gazebo native topics)
                     ▼
┌─── ros_gz_bridge (parameter_bridge) ─────────┐  /rgbd_camera/image (raw color)
│  Gazebo msg → ROS msg                        │  /rgbd_camera/depth_image (raw depth)
└────────────────────┬─────────────────────────┘  /uav_lidar/scan/points (raw lidar)
                     │
                     ▼
┌── d435i_sim ───┐   ┌── mid360_sim ──┐   ┌── pedestrian_state_publisher ──┐
│ σ=0.0014·z²    │   │ Risley σ≈2cm   │   │ /pedestrian_sim/agent_states   │
│ + 5% dropout   │   │ @25m + 距离白噪声│   │ (GT，给 evaluator)            │
│ + 1mm quantize │   │                │   └────────────────────────────────┘
└───┬────────────┘   └───┬────────────┘
    │ /d435i/color/image_raw  │ /mid360/pointcloud
    │ /d435i/depth/image_rect_raw
    ▼                         ▼
┌─── pose_stub (orbit 25Hz) ──────────────┐
│ /mavros/local_position/{pose,odom}      │ ← 给 detector
│ /uav_motion/pose_cmd ───┐               │ ← 回到 Gazebo 把 UAV 实体搬到该位姿
└─────────────────────────┼───────────────┘
                          ▼
              (UAV 物理实体跟着位姿动)
    │                     │
    ▼                     ▼
┌────── lvdot_yolo_node (Ultralytics yolo11n, GPU1, FP16) ──────┐
│ 订: /d435i/color/image_raw                                    │
│ 算: YOLO11n CNN @ imgsz=352, conf=0.25, 类别 'person'         │
│ 出: /yolo_detector/detected_bounding_boxes (Detection2DArray) │
└──────────────────────────┬────────────────────────────────────┘
                           │
                           ▼
┌═════════════════ lvdot_detector_main (C++, GPU0) ═══════════════════╗
║ 订: 上面三路 + pose + odom + yolo                                    ║
║                                                                      ║
║  Stage 1: 深度检测 (run_core)                                        ║
║   • U-V disparity: 按列累加 depth → U-map → 阈值化得线段             ║
║   • 用 u_map_threshold_point / threshold_line / min_length_line 滤波 ║
║   • 线段映回 3D → d_boxes (深度框)                                    ║
║   • YOLO 2D box 反投影到 3D 做 semantic gating (标 "这是人")          ║
║   → uv_boxes  (/onboard_detector/uv_bboxes，原始紫)                  ║
║                                                                      ║
║  Stage 2: lidar 检测                                                 ║
║   • DBSCAN @ epsilon=0.07, min_points=10                            ║
║   • 每个 cluster 拟合 AABB → lidar_boxes (/onboard_detector/lidar_bboxes)║
║                                                                      ║
║  Stage 3: 跨模态过滤 + 视觉/lidar 中间产物输出                       ║
║   • depth_boxes × lidar_boxes IoU 匹配 (filtering_BBox_IOU_threshold=0.4)║
║   • mutual-best 才保留 → filtered_bboxes                            ║
║   • 同时单独输出给 QC 用的两路:                                       ║
║       /onboard_detector/visual_bboxes_qcgaf  (depth U-V 那路)        ║
║       /onboard_detector/lidar_bboxes_qcgaf   (DBSCAN 那路)           ║
║                                                                      ║
║  Stage 4: 跟踪 (Kalman + Hungarian)                                  ║
║   • 7 维状态: [x, y, z, vx, vy, vz, size]                            ║
║   • kalman_filter_param = [0.25, 0.01, 0.05, 0.05, 0.04, 0.3, 0.6]   ║
║   • 帧间 Hungarian (feature_weight 控制相似度: pos / size / vel 等)   ║
║   • history_size=100, max_unmatched_frames=14 (14 帧丢就删 track)    ║
║   → tracked_bboxes  (/onboard_detector/tracked_bboxes，深蓝平滑)     ║
║                                                                      ║
║  Stage 5: 动态分类                                                   ║
║   • 用 Kalman vel 与 dynamic_velocity_threshold=0.2 m/s 比           ║
║   • 30 帧窗口内 ≥60% 帧速度超阈值 (dynamic_voting_threshold=0.6)     ║
║     才标 dynamic; frames_force_dynamic=10 帧后强制定型                 ║
║   → dynamic_bboxes (/onboard_detector/dynamic_bboxes, 红色 LINE_LIST)║
╚══════════════════════════════╪══════════════════════════════════╪═══╝
                               │                                  │
                ┌──────────────┘                       这里给 GRU →┘
                ▼
┌────── qcgaf_fusion_node (PyTorch, GPU1) ────────────┐
│ 订:                                                  │
│   /onboard_detector/visual_bboxes_qcgaf  (Stage 3)  │
│   /onboard_detector/lidar_bboxes_qcgaf  (Stage 3)   │
│   /d435i/color, /d435i/depth, yolo dets, mid360, IMU │
│                                                      │
│ 在线计算 7 维 quality 向量:                            │
│   q = [亮度, 边缘强度, 深度有效率, yolo conf 均值,    │
│        lidar 点密度, IMU 振动, 深度时序一致性]          │
│                                                      │
│ 模型: QC-GAF (Quality-aware Gated Attention Fusion)  │
│   • cam_dim=9, lidar_dim=10                          │
│   • 两路独立 MLP → embedding                           │
│   • Quality-gated cross-attention 加权 cam/lidar      │
│   • 输出: 3D box + conf [x,y,z, w,h,l, score]         │
│ 退化路径:                                             │
│   • n_cam==0 时 enable_lidar_fallback=True 把 lidar   │
│     直接复制到 cam slot (避免漏帧)                     │
│ 出: /qcgaf/fused_bboxes (CUBE, ns='qcgaf_fused')      │
└──────────────────────────────────────────────────────┘
                                                          │
                                                          ▼
                       ┌─── gru_prediction_node (PyTorch, GPU1) ────┐
                       │ 订: /onboard_detector/dynamic_bboxes        │
                       │                                            │
                       │ 每个 track_id 维护一个 HybridPredictor:      │
                       │   • Kalman 分支: 等速短期外推                │
                       │   • GRU 分支: 历史 8 帧 (x,y,z,vx,vy)        │
                       │     双层 GRU hidden=64 输出未来轨迹           │
                       │   • γ 自适应权重融合两者:                     │
                       │       t 近 (<1s)  γ→0  用 Kalman            │
                       │       t 远 (>2s)  γ→1  用 GRU               │
                       │                                            │
                       │ horizon=5 × dt=0.5s = 预测未来 2.5s          │
                       │ 出: /gru_predictor/predicted_positions      │
                       │     (LINE_STRIP + SPHERE 端点 marker)       │
                       └────────────────────────────────────────────┘
```

---

## 2. 关键阶段对应表

| 阶段 | 输入 topic | 关键算法 | 输出 topic |
|---|---|---|---|
| **传感器仿真** | Gazebo raw | d435i_sim: 高斯噪声 σ=0.0014z² + 5% dropout + 1mm quant<br>mid360_sim: Risley σ≈2cm@25m + 距离白噪声 | `/d435i/{color,depth}`<br>`/mid360/pointcloud` |
| **位姿** | (空) | orbit(r=0.4m, ω=0.3 rad/s) | `/mavros/local_position/{pose,odom}`<br>`/uav_motion/pose_cmd` |
| **YOLO 2D** | d435i color | YOLO11n CNN @ imgsz=352 FP16 | `/yolo_detector/detected_bounding_boxes` (px 坐标) |
| **Depth U-V** | d435i depth + YOLO | U-V 视差直方图 → 线段聚类 → YOLO 反投影 gating | 内部 visual_bboxes (3D) |
| **Lidar 检测** | mid360 pointcloud | DBSCAN (ε=0.07, min=10) → AABB 拟合 | 内部 lidar_bboxes (3D) |
| **跨模态过滤** | visual + lidar | mutual-best IoU 匹配 (threshold=0.4) | `/onboard_detector/{visual,lidar}_bboxes_qcgaf` |
| **跟踪** | filtered_bboxes | 7 维 Kalman + Hungarian (max_unmatched=14) | `/onboard_detector/tracked_bboxes` |
| **动态分类** | tracked vel | 阈值 0.2 m/s + 30 帧 60% 投票 | `/onboard_detector/dynamic_bboxes` |
| **QC-GAF 融合** | visual_qcgaf + lidar_qcgaf + 7 维 quality | PyTorch: 两路 MLP + Quality-gated Cross-Attention | `/qcgaf/fused_bboxes` |
| **GRU 预测** | dynamic_bboxes 历史 | HybridPredictor: 双层 GRU + Kalman + γ 自适应 | `/gru_predictor/predicted_positions` (5×0.5s) |

---

## 3. 几个需要注意的架构事实

### 3.1 QC 不在跟踪链路里

QC-GAF 的输入是 **detector 内部 Stage 3 输出的 `visual_bboxes_qcgaf` / `lidar_bboxes_qcgaf`**，也就是单帧的瞬时检测（**已经过初步 IoU 互配过滤**），而不是 `tracked_bboxes` 也不是 `dynamic_bboxes`。

QC 的输出 `/qcgaf/fused_bboxes` 是**一条单独的支路**，没有回到 detector 的跟踪/分类阶段。这意味着：

- QC 改进的是"单帧的位置/大小精度"
- detector 自己的 Kalman 跟踪和动态分类**不受 QC 影响**
- 下游 GRU 也不消费 QC 的输出

### 3.2 GRU 接的是 detector 的 dynamic_bboxes

`gru_prediction_node` 的 `input_topic` 默认是 `/onboard_detector/dynamic_bboxes`。也就是说：

- GRU 看到的是 detector **自己**的跟踪 + 分类结果
- 如果 detector 的 track_id 频繁切换（IDS 多），GRU 的历史窗口就是错的，预测精度必然差
- 优化 GRU 模型本身（重训）的上限被**前段跟踪的稳定性**卡死

### 3.3 QC 和 GRU 是两条并列分支，不是串行

```
detector ── Stage 3 ──→ QC fusion ──→ /qcgaf/fused_bboxes (单帧融合输出)
        └── Stage 5 ──→ GRU pred  ──→ /gru_predictor/predicted_positions (轨迹预测)
```

它们**只有共同的上游（detector）**，没有数据互通。这是设计选择，不是 bug。但这也意味着：

- 单独提升 QC 模型质量 → 只影响单帧融合精度
- 单独提升 GRU 模型质量 → 只影响轨迹预测精度
- **detector 本身的 tracking/classification 质量是这两个支路共同的"天花板"**

### 3.4 退化路径（fallback）

| 场景 | 现状 |
|---|---|
| YOLO 没检到 person | depth U-V 还是会输出框，但缺 semantic 标签 |
| n_cam==0 (visual_bboxes_qcgaf 空) | QC 走 `enable_lidar_fallback`，把 lidar 复制到 cam slot |
| n_lidar==0 (lidar_bboxes_qcgaf 空) | QC 用 default lidar features，模型自适应权重 |
| track 连续 14 帧没匹配上 | detector 删 track；GRU 也会因 max_idle=3s 清掉对应 predictor |
| GRU 历史不够 8 帧 | HybridPredictor 用 Kalman 短期外推 |

---

## 4. 当前 launch 文件入口

```bash
ros2 launch lvdot_bringup run_full_pipeline.launch.py \
  gazebo_gui:=true rviz:=true \
  use_realistic_sensors:=true \
  pose_stub_orbit_enabled:=true \
  qcgaf_checkpoint:=<外部项目目录>/qcgaf_fusion/outputs/best_model.pt \
  gru_model:=<外部项目目录>/gru_predictor/outputs/nuscenes_3d_tuned/best_model.pth
```

加 `launch_evaluator:=true` 即可在最后挂一个 evaluator 节点评 `/qcgaf/fused_bboxes`。
