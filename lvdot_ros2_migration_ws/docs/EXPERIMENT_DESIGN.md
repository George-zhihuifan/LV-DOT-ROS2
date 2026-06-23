# LV-DOT ROS2 实验评估体系设计方案

## 1. 背景与目标

原始 LV-DOT 论文使用 3D IoU 匹配、多阈值 P/R/F1 + 位置误差作为核心指标，在自建数据集上对比了 Dynablox 和 M-detector。我们的项目在此基础上：
- 从 ROS1 迁移到 ROS2
- 从地面机器人改为无人机平台
- 新增 QC-GAF 质量感知融合网络
- 新增 GRU 轨迹预测模块

本方案目标：建立一套与论文可对标、同时覆盖新增模块的完整评估体系。

---

## 2. 指标体系设计

### 2.1 检测指标（Detection Metrics）— 用于评估 LV-DOT 基线和 QC-GAF

对标论文 Table I 格式：

**匹配规则**：3D Axis-Aligned Bounding Box IoU

**GT bbox 构造**：Gazebo GT 只有位置点，需要附加固定尺寸：
- 行人 bbox: (0.5m, 0.5m, 1.7m)（宽、深、高）
- GT z 中心 = position.z + 1.7/2 = 0.02 + 0.85 = 0.87m

**指标集**：

| 指标 | 公式 | 说明 |
|---|---|---|
| Precision@IoU=τ | TP / (TP + FP) | τ ∈ {0.3, 0.5, 0.7} |
| Recall@IoU=τ | TP / (TP + FN) | τ ∈ {0.3, 0.5, 0.7} |
| F1@IoU=τ | 2·P·R / (P+R) | τ ∈ {0.3, 0.5, 0.7} |
| Mean Position Error | mean(‖det_center - gt_center‖₂) | 仅计算 IoU≥0.3 的匹配对 |
| IoU 曲线 | P/R/F1 vs IoU ∈ [0.05, 0.95] | 步长 0.05，共 19 个点 |

**3D IoU 计算**：
```
IoU(A, B) = Volume(A ∩ B) / Volume(A ∪ B)

A ∩ B: 
  overlap_x = max(0, min(A.x+A.w/2, B.x+B.w/2) - max(A.x-A.w/2, B.x-B.w/2))
  overlap_y = max(0, min(A.y+A.d/2, B.y+B.d/2) - max(A.y-A.d/2, B.y-B.d/2))
  overlap_z = max(0, min(A.z+A.h/2, B.z+B.h/2) - max(A.z-A.h/2, B.z-B.h/2))
  intersection = overlap_x * overlap_y * overlap_z
  union = vol_A + vol_B - intersection
```

**匹配策略**：贪心匹配（和论文一致）
1. 计算所有 GT-Det 对的 3D IoU
2. 按 IoU 降序排列
3. 贪心分配（每个 GT 和 Det 最多匹配一次）
4. IoU < τ 的不算匹配

### 2.2 跟踪指标（Tracking Metrics）— 用于评估跟踪器一致性

| 指标 | 说明 |
|---|---|
| MOTA | Multi-Object Tracking Accuracy = 1 - (FP + FN + IDS) / total_GT |
| IDF1 | ID F1 Score，衡量轨迹 ID 一致性 |
| IDSW | ID Switch 次数 |
| Frag | Fragmentation（轨迹中断次数） |
| MT / ML | Mostly Tracked / Mostly Lost 轨迹比例 |

实现方式：用 `motmetrics` Python 库（pip install motmetrics），输入逐帧匹配结果即可。

### 2.3 预测指标（Prediction Metrics）— 用于评估 GRU

| 指标 | 公式 | 说明 |
|---|---|---|
| ADE (Average Displacement Error) | mean(‖pred_t - gt_t‖₂) for t ∈ [1, H] | 所有预测步的平均位移误差 |
| FDE (Final Displacement Error) | ‖pred_H - gt_H‖₂ | 最后一步的位移误差 |
| ADE@1s | ADE 在 horizon=1s 内 | 短期预测精度 |
| ADE@2.5s | ADE 在 horizon=2.5s 内 | 长期预测精度 |

评估方法：
- 在 t 时刻记录 GRU 的 H 步预测 [p̂_{t+1}, ..., p̂_{t+H}]
- 等到 t+1, ..., t+H 时刻到来后，用实际 GT 位置计算误差
- 需要一个 buffer 存储历史预测，等 GT 到来后回溯计算

---

## 3. 评估器改造方案

### 3.1 新建 `advanced_evaluator.py`（不改原有 detection_evaluator.py）

**位置**：`ros2_depth_eval_ws/src/lvdot_ros2_adapter/lvdot_ros2_adapter/advanced_evaluator.py`

**功能**：
- 订阅 GT: `/pedestrian_sim/agent_states`
- 订阅检测: 可配置 topic（detector / qcgaf / gru）
- 3D IoU 匹配
- 逐帧计算多阈值 P/R/F1
- 轨迹 ID 关联（MOTA/IDF1）
- GRU 预测回溯评估（ADE/FDE）
- 输出详细 CSV + 终端汇总

**参数**：
```yaml
gt_topic: /pedestrian_sim/agent_states
det_topic: /onboard_detector/dynamic_bboxes   # 或 /qcgaf/fused_bboxes
pred_topic: /gru_predictor/predicted_positions  # 可选
det_marker_type: 5          # 5=LINE_LIST(detector), 1=CUBE(qcgaf)
det_namespace: dynamic       # 或 qcgaf_fused
gt_bbox_size: [0.5, 0.5, 1.7]  # 行人 GT bbox 尺寸 [w, d, h]
iou_thresholds: [0.3, 0.5, 0.7]
csv_path: /tmp/lvdot_advanced_eval.csv
summary_path: /tmp/lvdot_eval_summary.json
eval_duration_sec: 60.0     # 评估持续时间
warmup_sec: 15.0            # 前 N 秒不计入（等 Gazebo 稳定）
```

**CSV 格式**（每帧一行）：
```
stamp_sec, gt_n, det_n,
tp_03, fp_03, fn_03, tp_05, fp_05, fn_05, tp_07, fp_07, fn_07,
mean_err_m, mean_iou,
gt_ids, det_ids, matched_pairs
```

**汇总 JSON**：
```json
{
  "duration_sec": 45.0,
  "total_frames": 1350,
  "iou_0.3": {"precision": 0.75, "recall": 0.88, "f1": 0.81, "pos_error": 0.09},
  "iou_0.5": {"precision": 0.72, "recall": 0.85, "f1": 0.78, "pos_error": 0.09},
  "iou_0.7": {"precision": 0.60, "recall": 0.72, "f1": 0.65, "pos_error": 0.08},
  "tracking": {"mota": 0.65, "idf1": 0.72, "idsw": 12, "frag": 8},
  "prediction": {"ade_1s": 0.15, "ade_2s": 0.35, "fde": 0.52}
}
```

### 3.2 实现步骤

**Step 1**: 3D IoU 匹配核心
- 从 LINE_LIST markers 提取 bbox (min/max of 24 vertices → center + size)
- 从 CUBE markers 提取 bbox (pose.position + scale)
- 从 AgentPoseArray 构造 GT bbox (position + fixed size)
- 实现 `compute_3d_iou(box_a, box_b) → float`
- 实现 `greedy_match(gt_boxes, det_boxes, iou_threshold) → [(gt_idx, det_idx, iou)]`

**Step 2**: 多阈值逐帧评估
- 对每帧，在 τ=0.3/0.5/0.7 各做一次匹配
- 记录 TP/FP/FN + 匹配对的位置误差和 IoU

**Step 3**: 跟踪评估（MOTA/IDF1）
- 维护 GT ID → Det ID 的帧间映射
- 检测 ID switch：同一个 GT 在连续帧匹配到不同 Det ID
- 用 motmetrics 库计算 MOTA/IDF1/IDSW/Frag

**Step 4**: GRU 预测评估
- 订阅 `/gru_predictor/predicted_positions`
- 解析 LINE_STRIP markers 的 points 为 [current_pos, pred_1, pred_2, ..., pred_H]
- 存入 buffer: `{track_id: {timestamp: [pred_positions]}}`
- 当 GT 到来时，回溯 buffer 中 H 步前的预测，计算 ADE/FDE

**Step 5**: CSV 输出 + JSON 汇总
- 每帧写 CSV
- 结束时（Ctrl+C 或 eval_duration_sec 到时）输出汇总 JSON

---

## 4. 仿真场景改造方案

### 4.1 行人密度调整

当前问题：8 个行人分散在 25m×10m 区域，UAV 视野内经常只有 1-2 个。

方案：创建一个**密集版 waypoint 配置** `pedestrian_dense.yaml`

改动：
- 保持 8 个行人不变
- 将所有 waypoint 集中到 12m×6m 区域（X: 2-14, Y: -3 to 3）
- 减小 waypoint 间距，增加 attraction waypoint 密度
- 行人初始位置也集中到该区域

效果：UAV 穿梭飞行时，视野内平均 4-6 个行人。

### 4.2 多场景变体

创建 3 个场景配置：

| 场景 | 行人数 | 区域 | 静态障碍 | 目的 |
|---|---|---|---|---|
| `dense_open` | 8 | 12×6m 无遮挡 | 少 | 理想条件基准 |
| `dense_cluttered` | 8 | 12×6m 有遮挡 | 多（柱子、隔板） | 遮挡场景 |
| `sparse_wide` | 8 | 25×10m | 适中 | 远距离检测 |

实现：每个场景对应一个 `pedestrian_xxx.yaml`，共享同一个 SDF 世界文件，通过 launch 参数切换。

### 4.3 UAV 运动模式

保留已实现的 `uav_waypoint_mission.py` 穿梭模式，针对每个场景配置调整 waypoints：
- `dense_open`：在密集区域内小范围穿梭
- `dense_cluttered`：绕障碍物穿梭
- `sparse_wide`：大范围巡航（当前路径）

---

## 5. 消融实验设计

### 5.1 实验组

| 编号 | 名称 | UV深度 | LiDAR | YOLO | QC-GAF | GRU | 说明 |
|---|---|---|---|---|---|---|---|
| A1 | LiDAR-only | - | ✓ | - | - | - | 对标论文 "w/o visual" |
| A2 | Visual-only | ✓ | - | ✓ | - | - | 对标论文 "w/o LiDAR" |
| A3 | LV-DOT baseline | ✓ | ✓ | ✓ | - | - | 对标论文完整 LV-DOT |
| A4 | + QC-GAF | ✓ | ✓ | ✓ | ✓ | - | 我们的融合改进 |
| A5 | + GRU | ✓ | ✓ | ✓ | ✓ | ✓ | 完整系统 |
| A6 | + §3.3 noise adapt | ✓ | ✓ | ✓ | ✓(+noise) | ✓ | 质量感知噪声自适应 |

### 5.2 每个实验组的控制变量

**A1 (LiDAR-only)**：
```bash
ros2 launch ... fusion_mode:=lidar_only enable_yolo:=false
```
- `detector_param.yaml` 中 `fusion_mode: lidar_only`

**A2 (Visual-only)**：
```bash
ros2 launch ... fusion_mode:=visual_only
```
- `detector_param.yaml` 中 `fusion_mode: visual_only`

**A3 (LV-DOT baseline)**：
```bash
ros2 launch ... fusion_mode:=dual
```
- 评估 topic: `/onboard_detector/dynamic_bboxes`

**A4 (+ QC-GAF)**：
```bash
ros2 launch ... fusion_mode:=dual qcgaf_integration_mode:=refinement
```
- 评估 topic: `/qcgaf/fused_bboxes`

**A5 (+ GRU)**：
- 检测评估仍用 A4 的 topic
- 额外评估 GRU 的 ADE/FDE

**A6 (+ noise adaptation)**：
```bash
ros2 launch ... qcgaf_noise_adaptation_enabled:=true
```

### 5.3 每组实验执行流程

```
对每个场景 S ∈ {dense_open, dense_cluttered, sparse_wide}:
  对每个实验组 A ∈ {A1, A2, A3, A4, A5, A6}:
    1. 启动仿真 + 全链路（对应参数）
    2. 等待 15 秒 warmup
    3. 收集 60 秒评估数据
    4. 停止，保存 CSV + JSON
    5. 输出到 logs/eval_{S}_{A}_{timestamp}/
```

总计：3 场景 × 6 实验组 = **18 次实验**

---

## 6. 实施路线图

### Phase 1：评估器改造（优先级最高）

| 步骤 | 内容 | 预计工作量 |
|---|---|---|
| 1.1 | 实现 `compute_3d_iou()` 函数 | 30 min |
| 1.2 | 实现 `greedy_iou_match()` 函数 | 30 min |
| 1.3 | 实现 GT bbox 构造（position + fixed size） | 15 min |
| 1.4 | 实现 Det bbox 提取（LINE_LIST 和 CUBE 两种格式） | 30 min |
| 1.5 | 实现多阈值逐帧评估循环 | 30 min |
| 1.6 | 实现 CSV + JSON 输出 | 20 min |
| 1.7 | 注册 entry_point，添加 launch 参数 | 15 min |
| 1.8 | 跟踪指标（MOTA/IDF1）集成 motmetrics | 45 min |
| 1.9 | GRU 预测评估（ADE/FDE buffer） | 45 min |

### Phase 2：场景改造

| 步骤 | 内容 | 预计工作量 |
|---|---|---|
| 2.1 | 创建 `pedestrian_dense.yaml` 密集分布配置 | 30 min |
| 2.2 | 创建对应的 waypoint mission 路径 | 20 min |
| 2.3 | 添加 launch 参数切换场景配置 | 15 min |
| 2.4 | 验证密集场景下 4-6 个行人持续在视野内 | 20 min |

### Phase 3：消融实验

| 步骤 | 内容 | 预计工作量 |
|---|---|---|
| 3.1 | 确认 fusion_mode 参数正确控制 LiDAR/visual 开关 | 20 min |
| 3.2 | 写自动化实验运行脚本 `run_ablation.sh` | 30 min |
| 3.3 | 执行 18 组实验 | ~30 min（自动） |
| 3.4 | 汇总脚本：读取所有 JSON，生成论文格式表格 | 30 min |

### Phase 4：结果分析

| 步骤 | 内容 |
|---|---|
| 4.1 | 生成 Table I 格式对比表（A1-A6 × IoU 0.3/0.5/0.7） |
| 4.2 | 生成 IoU 曲线图（P/R/F1 vs IoU threshold） |
| 4.3 | 生成跟踪指标对比表（MOTA/IDF1/IDSW） |
| 4.4 | 生成 GRU 预测指标表（ADE/FDE @ 1s/2.5s） |

---

## 7. 最终输出格式

### 7.1 检测对比表（对标论文 Table I）

| Method | IoU=0.3 ||| IoU=0.5 ||| IoU=0.7 |||
|---|---|---|---|---|---|---|---|---|---|
| | P | R | F1 | P | R | F1 | P | R | F1 |
| LiDAR-only (A1) | | | | | | | | | |
| Visual-only (A2) | | | | | | | | | |
| LV-DOT baseline (A3) | | | | | | | | | |
| + QC-GAF (A4) | | | | | | | | | |
| + QC-GAF + GRU (A5) | | | | | | | | | |
| + Noise Adapt (A6) | | | | | | | | | |

### 7.2 跟踪指标表

| Method | MOTA | IDF1 | IDSW | Frag | MT | ML |
|---|---|---|---|---|---|---|
| LV-DOT baseline | | | | | | |
| + QC-GAF | | | | | | |
| + Full system | | | | | | |

### 7.3 预测指标表

| Method | ADE@1s | ADE@2.5s | FDE@2.5s |
|---|---|---|---|
| Kalman-only | | | |
| GRU-only | | | |
| Hybrid (ours) | | | |

---

## 8. 涉及的文件

| 文件 | 操作 |
|---|---|
| `ros2_depth_eval_ws/src/lvdot_ros2_adapter/lvdot_ros2_adapter/advanced_evaluator.py` | **新建** |
| `ros2_depth_eval_ws/src/lvdot_ros2_adapter/setup.py` | 添加 entry_point |
| `ros2_depth_eval_ws/src/depth_eval_bringup/config/pedestrian_dense.yaml` | **新建** |
| `lvdot_ros2_migration_ws/src/lvdot_bringup/launch/run_full_pipeline.launch.py` | 添加场景参数 |
| `lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/run_ablation.sh` | **新建** |
| `lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/summarize_results.py` | **新建** |
