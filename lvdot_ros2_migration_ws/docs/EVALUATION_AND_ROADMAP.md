# 项目评估与改进路线（LV-DOT + QC-GAF + GRU UAV 感知预测栈）

> 2026-05-12 第一版。基于 5 月 12 日的"全栈调通 + 自动 GT 评估 + u_map sweep"那一轮的实测数据。

本文档分三段，按顺序回答：
1. **同类项目通常用什么指标**——纵向跟 nuScenes / KITTI / Waymo 等公开 benchmark 对齐
2. **这套 pipeline 当前处于什么水平**——把我们刚跑出来的数字放到 §1 的坐标里
3. **后续要把指标拉上去，依次该做什么**——分短中长三段，给可执行清单

---

## 1. 同类项目的标准评估指标

这套 pipeline 的功能可以拆三层：**3D 目标检测**、**多目标跟踪**、**轨迹预测**。三层各有自己的工业标准。最后还有**端到端**的系统级指标。

### 1.1 3D 目标检测（对应 LV-DOT detector + QC-GAF fusion）

| 指标 | 定义 | 公开 benchmark 上常见的"过线"分数 |
|---|---|---|
| **3D mAP @ IoU=0.5** | 每个类别在不同置信度阈值上的 PR 曲线下面积，再对类别取均值 | nuScenes / KITTI 行人 mAP@0.5：SOTA 60-75%；基线方法（PointPillars）≈ 50% |
| **3D mAP @ IoU=0.25** | 同上，但 IoU gate 放到 0.25（粗匹配） | 行人通常比 0.5 高 10-15 个百分点 |
| **NDS** (nuScenes Detection Score) | mAP × 0.5 + (1 - mATE) × 0.1 + 类似项 × 4，把位置 / 大小 / 朝向 / 速度误差都揉进来 | SOTA 60-70%，基线 30-40% |
| **mATE** (Average Translation Error) | 平均中心位置误差，m | SOTA ≤ 0.30m；可用 ≤ 0.50m；研究阶段 1.0m+ |
| **mASE / mAOE** | 大小 / 朝向误差 | 可用 ≤ 0.20 / 0.20rad |
| **Recall @ N detections** | TopN 候选下的召回 | 用于排查 "miss" 是因为打分低还是根本没出框 |
| **per-range Recall** | 按距离段（0-10m / 10-20m / 20-30m）单独报 | 室内/近距应用一般只看 0-10m |

我们用的是**最简单的 2D 中心距离匹配**（center-based eval），等价于 mAP 但把 IoU 换成 "xy 距离 ≤ gate"。这是公开 benchmark 早期的做法，现在普遍换成 IoU-3D。

### 1.2 多目标跟踪（对应 detector 的 tracking stage）

| 指标 | 定义 | 备注 |
|---|---|---|
| **MOTA** | 1 − (FN + FP + IDS) / GT | 可用 ≥ 0.50；SOTA ≥ 0.75 |
| **MOTP** | 匹配上的目标位置误差（≈ mATE） | |
| **IDF1** | 按身份对齐的 F1 | MOTA 不惩罚 ID 频繁切换，IDF1 会 |
| **IDS** (Identity Switches) | 同一 GT 目标被换 track ID 的次数 | 越少越好 |
| **HOTA** | 把检测和关联拆开评的新标准 | 2020 年后的论文优先报这个 |
| **Track Fragmentation** | 同一 GT 被切成几个 track 段 | 反映 occlusion 鲁棒性 |
| **MT / ML** | 大多数轨迹被跟上 / 大多数被丢的占比 | ≥ 80% MT 才算可用 |

### 1.3 轨迹预测（对应 GRU predictor）

| 指标 | 定义 | nuScenes / Argoverse 上的过线 |
|---|---|---|
| **ADE** (Average Displacement Error) | 预测轨迹和 GT 在每个时间步的 L2 误差均值，m | 行人 @ 3s：SOTA ≤ 0.50m；基线 1.0m+ |
| **FDE** (Final Displacement Error) | 仅看最末时间步的 L2 误差 | ADE 的 1.5-2.5× |
| **minADE_K / minFDE_K** | 多模态预测的 K 条轨迹里挑 ADE/FDE 最小的 | K=5/6 是主流 |
| **Miss Rate @ K, R** | K 条预测里全没 FDE ≤ R 的占比 | R=2m，越低越好 |
| **NLL** | 概率预测的负对数似然 | 报概率分布的模型才用 |

### 1.4 端到端 / 系统级

| 指标 | 定义 | 量级 |
|---|---|---|
| **End-to-end Latency** | sensor 时间戳 → prediction 出框的 wall-clock 间隔 | UAV 近避障 < 200ms 算可用，< 100ms 是目标 |
| **Pipeline Throughput** | 持续稳定的输出 Hz | 应至少 ≥ 10Hz |
| **GPU Util / Power** | 推理时 GPU 平均利用率 | 平台调度的间接参考 |
| **Robustness 退化曲线** | 在低光 / 振动 / 雨雾 / 部分遮挡下 mAP 下降幅度 | QC-GAF 这种 quality-aware 方法专门测这个 |

---

## 2. 当前这套 pipeline 的水平

数据来源：2026-05-12 跑的几组 sim（30-60 秒），未跑 YOLO，2.5 m center-distance 匹配。

### 2.1 实测指标

| 指标 | 当前值 | 同类基线参考 | 评估 |
|---|---|---|---|
| Detector Recall (默认参数) | **0.04** @ 2.5m gate | KITTI 行人 SOTA ≈ 0.75 @ 0.5 IoU | **远低于基线** |
| Detector Recall (tuned v3) | **0.137** @ 2.5m gate | 同上 | 优化后仍**很低** |
| Detector Precision | 0.34-0.55 @ 2.5m gate | SOTA ≈ 0.80 | 中低 |
| F1 | 0.08 → 0.20 (调参后) | SOTA ≥ 0.70 | 远低于可用线 |
| Mean Position Error | 1.45-1.81 m | mATE 可用线 ≤ 0.50m | **3× 于可用阈值** |
| QC fused output rate | ~10 boxes/frame, lidar fallback 100% | 视觉与 lidar 各贡献 ≈ 50% | 视觉链路当前没数据 |
| End-to-end pipeline | 全通 + 25 Hz | 目标 ≥ 10 Hz | **数据流上达标** |
| MOTA | 没测 | 可用 ≥ 0.50 | 没量化 |
| ADE / FDE @ 2.5s | 没测 | ADE 可用 ≤ 0.50m | 没量化 |

### 2.2 这些数字说明的事

1. **数据流是健康的**——detector 30 Hz、QC 100% 不漏帧、GRU 接得上、UAV 真的在动。这是 2026-05-11 之前都没达到的状态。
2. **精度不在可用区**——recall 14%，1.5m+ 中心误差。完全不是"上飞机"的水平，但也是**第一次有数字**——之前根本无法报。
3. **指标低的两个根本原因**：
   - **没跑 YOLO**：visual 链路 100% 空，detector 只能靠 depth + lidar 几何，没有语义。QC fallback 100% 走 lidar，等于只用了一路传感器。
   - **行人多数在视野外**：8 个 GT 行人，4-5 个距离 UAV 6-11m（在 12m depth 极限内），其他在 14-20m（超距）。recall 的分母被超距目标拉大。
4. **u_map sweep 的边际收益小但确定**：F1 从 0 提到 0.20，主要靠 `threshold_line` 从 1 提到 5（强制要求更长的连续 line group 才算 box）——本质是把 noise 推到 false positive 之外，但同时丢了一些真目标。

### 2.3 现在能给的"诚实结论"

> 这套 pipeline 目前是**研究阶段的早期可运行版本**，离工业可用还有数量级差距，主要差在感知精度。
>
> 但它已经具备了**做精度改进的基础设施**：自动 GT 评估 + 参数 sweep + realistic 传感器 + 双卡分流——之前每次调参都靠肉眼看 RViz，现在每改一行能在 1 分钟内拿到客观分数。

---

## 3. 后续改进路线（按 ROI 排序）

下面这些都是"再投一周左右的工，可以看到具体指标变化"的事，按性价比从高到低排。

### 3.1 短期（1-3 天，每条都能直接拉分）

#### A. 启 YOLO 重跑 sweep —— **预期 recall 翻倍以上**
- 当前 sweep 的所有结果都是 **`enable_yolo:=false`** 下跑的。YOLO 一开，visual 链路立刻有 boxes，QC 走"真融合"而不是 100% fallback。
- 命令：`python3 scripts/u_map_sweep.py --enable-yolo --grid 1,2 2,3,5 1,2`
- 验证目标：F1 ≥ 0.40，recall ≥ 0.30。
- 同时把 sweep 的"匹配 gate" 从 2.5m 拉回 1.5m（行业标准 ≤ 1m），看 precision 怎么变。

#### B. evaluator 加 3D IoU 评分模式 —— **可对齐 nuScenes**
- 当前 `detection_evaluator.py` 用 2D 中心距离。改成可选用 3D IoU 后，可直接报 mAP@0.25 / mAP@0.5。
- 改动：`detection_evaluator.py::_match` 加一个 `metric: center|iou3d` 开关。GT 没有真实尺寸，可以用 0.5×0.5×1.7m 的人形默认 box（同 nuScenes "pedestrian" 类别）。
- 验证目标：能滚出 mAP@0.25 与 mAP@0.5 两列。

#### C. 把行人放近一点，让 GT 落进检测范围
- 修 `worlds/pedestrian_prototype.sdf` 或 `pedestrian_planner_*.py`，让多数行人轨迹的 envelope 在 UAV 5m 范围内活动。
- 或者反过来：让 UAV orbit 半径增大、移动到 (10, 0, 1) 之类更接近行人活动区的位置。
- 验证目标：GT 内"在 12m depth 范围内的行人数" / "GT 总数" ≥ 0.7（当前 ~0.5）。

#### D. MOTA / IDF1 评分
- evaluator 加一段：维护 `gt_name → matched_det_id` 的字典，每帧记 ID switch；同时累计 FN + FP，按 MOTA 定义算。
- 工作量：100 行 Python 以内，直接加在 `detection_evaluator.py`。
- 验证目标：拿到 MOTA + IDF1 + IDS 三个数。

### 3.2 中期（1-2 周，方向性投入）

#### E. 启用 realistic_sensors 重新做整套评估
- 当前指标全是 `use_realistic_sensors:=false`（原始 Gazebo 相机）跑出来的。realistic_sensors（d435i + mid360 噪声）一定会让指标降一些，但更接近真实部署。
- 验证目标：raw vs realistic 各跑一遍 sweep，输出对比表。

#### F. YOLO TensorRT engine（短板 #9）
- 已装好 `tensorrt-10.15.1`。`model.export(format='engine', imgsz=352, half=True, device=0)` 一行导出。
- 改 `lvdot_yolo_node.py`：优先加载 `.engine`，落空再 `.pt`。
- 验证目标：YOLO 平均 latency 79ms → 30ms 以下；YOLO 实际速率从 19Hz → 28Hz+。

#### G. ADE / FDE 评测 GRU
- 类似 `detection_evaluator.py`，但订 `/gru_predictor/predicted_positions` + GT。每帧把预测的 5 步 × 0.5s 与 GT 在对应未来时刻的位置比 L2。
- 工作量：≤200 行 Python。
- 验证目标：在 horizon=2.5s 拿到 ADE / FDE 第一组数。

#### H. QC-GAF 的 quality 输入 真用上 IMU
- 当前 `quality_dim=7`，但 IMU 振动那一维基本没被算（imu_buffer 在 sim 里没有真振动）。在 sim 里给 UAV 加合理的 angular velocity 噪声，让 QC 学到"振动严重时给 lidar 加权"——这正是 QC-GAF 论文卖点。
- 验证目标：在振动较大的 50 frame 窗口内，QC 输出的 cam vs lidar 加权比从 0.5/0.5 飘到 0.3/0.7。

### 3.3 长期（方向性，做一年的）

#### I. 真实数据采集 + YOLO fine-tune
- 用 d435i 实机录 10-30 min 行人场景，把 YOLO 在自己数据上 fine-tune（或换 yolo11s / m）。
- 仿真行人 actor 长得很塑料，YOLO 默认 COCO 权重对它们的 conf 偏低；真实数据会一并解决。

#### J. 不同 weather / lighting 下的鲁棒性曲线
- sim 里加 fog / night / glare 等场景（这正是 QC-GAF 训练里的 noise 配置——`config.yaml` 已经有 normal/glare/vibration/rain_fog 四档）。每档跑一遍 sweep，画 robustness 衰减曲线。

#### K. 真机迁移 + 实测对比
- 把 ROS2 stack 部署到真飞机上跑同一个评估脚本，对照仿真分数。差距通常 ≥ 30-50%，是真正 ship-ready 的标尺。

#### L. 替换检测主干
- 长期看，单目深度 + DBSCAN 这个 detector 是 2019-2020 年代的方案。SOTA 是 multi-view BEV transformer（BEVFusion、UVTR）或单目 3D detector（FCOS3D、PETR）。如果项目要冲 SOTA，最终得换主干。

---

## 4. 一图流总结

```
┌─────────────────────────────────────────────────────────────┐
│ 当前状态 (2026-05-12)                                        │
│  ├─ 数据流 ✅                                                │
│  ├─ 自动 GT 评估 ✅                                          │
│  ├─ Recall = 14%   (SOTA = 75%)                             │
│  ├─ mATE  = 1.5m   (可用 ≤ 0.5m)                            │
│  └─ MOTA / ADE / FDE 未测                                    │
│                                                              │
│ 离"看到第一个像样的分数" 还差：                              │
│  1. 启 YOLO 重跑 (低成本，预期翻倍)        ←── 这周做         │
│  2. 加 3D IoU & MOTA 评估                                    │
│  3. 把行人位置 / UAV 轨迹挪到合理范围                        │
│                                                              │
│ 离"接近 benchmark 基线" 还差：                               │
│  4. realistic_sensors 完整对照                               │
│  5. YOLO TensorRT                                            │
│  6. GRU ADE/FDE 量化                                         │
│  7. QC-GAF 的 quality 信号真用上                             │
│                                                              │
│ 离"ship 真机" 还差：                                         │
│  8. 真机数据 + fine-tune                                     │
│  9. 多 weather / lighting 鲁棒性                             │
│ 10. 换 SOTA 主干（可选，看目标）                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. 跑这套评估的具体命令清单（cheat sheet）

```bash
# Workspace
cd $LVDOT_ROOT/lvdot_ros2_migration_ws
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source install/setup.bash

# 1. 跑一次有 GT 评估的 30 秒 demo（YOLO 启）
ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  enable_yolo:=true launch_yolo_node:=true launch_pose_stub:=true \
  pose_stub_orbit_enabled:=true \
  launch_evaluator:=true evaluator_match_threshold_m:=2.5 \
  evaluator_csv_path:=/tmp/lvdot_eval.csv \
  u_map_threshold_point:=2 u_map_threshold_line:=5 u_map_min_length_line:=2

# 看终端的 [window Nfr] prec=.. rec=.. F1=.. MOTA=.. mean_err=.. m
# 详细 CSV 在 /tmp/lvdot_eval.csv

# 2. u_map 自动 sweep
python3 src/lvdot_bringup/scripts/u_map_sweep.py \
  --grid 1,2 1,2,3,5 1,2 --warmup 15 --collect 35 \
  --match-threshold 2.5 --enable-yolo \
  --output /tmp/u_map_sweep_yolo.csv

# 3. 跑全栈 (detector + QC + GRU)
ros2 launch lvdot_bringup run_full_stack_qcgaf_gru.launch.py \
  qcgaf_checkpoint:=<外部项目目录>/qcgaf_fusion/outputs/best_model.pt \
  gru_model:=<外部项目目录>/gru_predictor/outputs/nuscenes_3d_tuned/best_model.pth \
  use_sim_time:=true
```
