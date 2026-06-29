# 当下要做的事 (NEXT_STEPS) — LV-DOT-ROS2

> 2026-05-12 快照。基于 docs/ 全套文档 + 最新 reports + git status + 当下运行实测。
> 这份文档**只列"要动手的事"**,背景与设计推理见 `EVALUATION_AND_ROADMAP.md` / `THESIS_VS_CODE_GAP_ANALYSIS.md` / `PROJECT_OVERVIEW.md`。

---

## 0. 此刻事实(2026-05-12 17:35 实测)

全链路在跑(`ros2 launch lvdot_bringup run_full_pipeline.launch.py`, headless),三个顶层输出都健康:

| Topic | Hz | 状态 |
|---|---|---|
| `/onboard_detector/dynamic_bboxes` | ~22 | ✅ |
| `/qcgaf/fused_bboxes` | ~20 | ✅ |
| `/gru_predictor/predicted_positions` | ~19 | ✅ |
| `/yolo_detector/detected_bounding_boxes` | **~4** | ⚠️ 远低于预期(应 19) |

11 节点全部在线:d435i_sim, mid360_sim, pose_stub, yolo, detector, qcgaf_fusion, gru_prediction, ros_gz_bridge, pedestrian_state_publisher, uav_pose_sync_system, pedestrian_pose_sync_system。

Detector 内部 pipeline stats(取若干帧典型值):

```
tracks=20-26  fused=8-14  dynamic=1-2  fusion_components=10
yolo_in=0  yolo_human=0  yolo_match3d=0  yolo_fused_used=0
QC: avg_cam_in=6.2 avg_lidar_in=9.6 lidar_fallback ≈ 50%
```

---

## 1. 必须先做(本周内,blocker 类)

### 1.1 ⚠️ 大量代码未提交,先验证再 commit

`git status` 显示主 workspace 有 30+ 文件 modified、4 个 docs untracked,姊妹 `ros2_depth_eval_ws` 也有 20+ 处改动。其中:

- **§3.3 跟踪层 Q/R 自适应**已经全套写完(KF Q_base/R_base + setNoiseScales / tracking_filter noiseAdaptationEnabled + Hc/Hl / detector 订阅 `/qcgaf/quality_vector` / fusion_node 发布 quality vector / detector_param.yaml 启用),但**没人验证 Q/R 是否真的随质量在变**。
- `gru_predictor/src/`、`qcgaf_fusion/src/`、`lvdot_bringup/scripts/` 大量训练 / 评估脚本被 `D`(删除),原因不在 commit message 里。需要明确这是有意精简还是遗失。
- 4 份新文档(`PROJECT_OVERVIEW.md`、`PIPELINE_DATA_FLOW.md`、`EVALUATION_AND_ROADMAP.md`、`THESIS_VS_CODE_GAP_ANALYSIS.md`)只在 untracked,丢了就没了。

**动作:**
1. 在 `tracking_filter.cpp` 加一行限频 INFO 日志(每秒 1 次),打印 `H_c, H_l, q_scale, r_scale` 实际数值
2. 启动确认 q_scale/r_scale 在 [1.0, 2.0] 范围内随质量浮动(质量低 → scale 高)
3. 一旦确认生效,把 `lvdot_core / lvdot_ros2 / qcgaf_fusion / lvdot_bringup` 的 §3.3 改动作为一个 commit 提交
4. 4 份新 docs 作为一个 commit
5. 删除的脚本作为一个 commit(并在 commit message 解释精简原因)

### 1.2 ⚠️ YOLO 实际只跑 ~4 Hz(应 19 Hz)

PROJECT_OVERVIEW §4 写"YOLO ~19 Hz",但 2026-05-12 17:35 直接 `ros2 topic hz` 测出 **~4 Hz**。这是 `yolo_in=0 / yolo_human=0 / depth_yolo_sync=0` 全程为零的根因——YOLO 太慢 → depth_yolo skew 超过 `max_depth_yolo_skew_sec=0.8s` → detector 跳过 YOLO。

后果叠加:
- detector 没有语义标签 → `yolo_human=0`,dynamic 分类只能靠速度
- QC fusion 完全没有 visual semantic 输入,`lidar_fallback ≈ 50%` 在用
- 论文 §1.3.6 "语义 + 运动双条件 dynamic 分类"实际只有运动条件

**动作:** 
1. 看 `lvdot_yolo_node.py` 当前实测 latency。怀疑点:GPU1 是不是被 QC + GRU 抢占?inference_hz 参数(默认 10)是不是被限频?FP16 / warmup 是否生效?
2. 修到稳定 ≥ 15Hz。若 GPU 抢占,临时把 GRU/QC 之一钉到 GPU0(双卡分流再分一次)
3. 验证修复后 `yolo_in > 0 / yolo_human > 0`,QC `lidar_fallback` 比例下降

### 1.3 跑一次完整 gt_eval,把当下基线"钉死"

自从 §3.3 写完后**没人跑过正式 eval**。最新有 summary.md 的 gt_eval 是 `gt_eval_suite_20260506_122212`(tracked recall=0.136, err=1.278m),那是 §3.3 之前。

但是 `scripts/run_gt_eval_suite.sh` 在 git status 里显示已删除(`D`),需要先决定:
- 选项 A:从 git 恢复 `git restore src/lvdot_bringup/scripts/run_gt_eval_suite.sh`
- 选项 B:重写一个精简版(launch + evaluator + 30-60s 采样 + 汇总)
- 选项 C:用 EVALUATION_AND_ROADMAP.md §5 的 cheat sheet 手动跑

**动作:** 至少跑 5×60s suite,对比 §3.3 关 vs 开两种配置,把"开 §3.3 是否真有 recall/error 增益"量化下来。

---

## 2. 论文核心改进剩余项

来自 `THESIS_VS_CODE_GAP_ANALYSIS.md` 的 "必须补"清单,按建议顺序(评估先于算法):

### 2.1 评估指标体系对齐 §4.2(**这是 prerequisite,先做**)
当前 `detection_evaluator.py` 只有 2D 中心距离 + 简化 MOTA。论文 §4.2 要求:
- P / R / F1 **at 3D IoU 0.3 / 0.5 / 0.7** (GT 用 0.5×0.5×1.7m 人形默认 box)
- **IDF1 / IDSW** (每帧维护 `gt_id → matched_track_id`,记 switch)
- **ADE / FDE @ horizon 5×0.5s** (新写 `prediction_evaluator.py`)
- 逐模块 latency

`detection_evaluator.py` 现在在 `ros2_depth_eval_ws/src/lvdot_ros2_adapter/`(未 commit 的 untracked 文件)。

### 2.2 §3.1 检测层 EMA 平滑 (small, 已经设计好)
论文 §3.1。加在 `lvdot_detector_main` 发布 `dynamic_bboxes` / `tracked_bboxes` 前,自适应 α (位移超阈值切高 α_high)。

### 2.3 §3.2 融合层匹配关系迟滞 (medium)
论文 §3.2。`qcgaf_fusion_node` 内对软匹配矩阵 S 做 EMA + 双阈值 hysteresis(`θ_enter / θ_exit`)。

### 2.4 §1.3.5 关联代价改 Mahalanobis + IoU3D (medium, C++)
论文公式 (42) `C_ij = λ_d · d_Maha + λ_iou · (1 - IoU_3D)`。当前是基线 feature_weight 相似度。改动在 `tracking_filter.cpp`,需要从 KF 拿 `P_t|t-1` 算 Mahalanobis,IoU 用 AABB 简化。新增 3 个超参 `λ_d / λ_iou / C_gate`。

### 2.5 §1.3.4 GRU γ 自适应三因素 (medium, Python)
论文公式 (40)(41):`γ = sigmoid(β·(e_GRU - e_KF)) · max(0, 1 - c_occ/C_max)`。当前 `hybrid_predictor.py` 按时间步分段,没有按错误对比、也没遮挡修正。

### 2.6 §4.3 消融实验 launch 开关 (small,但要后跑数据)
launch 加 3 开关:`enable_qcgaf` / `enable_jitter_suppression` / `enable_gru`(false 时走规则融合/纯 KF)。然后跑 A-G 7 配置,每个 60s,汇总成论文表。

### 2.7 (optional) §1.3.6 动态分类语义条件
小改动:detector 内部把 `yolo_human` 标志带到 track,分类时 OR 进速度阈值。**前提是 §1.2 的 yolo_in=0 先修好**。

---

## 3. 工程整理(技术债)

### 3.1 README_migration_ops.md 完全过时
- 引用路径 `/home/skbt2/lvdot_ros2_migration_ws`(实际是 `/home/mcb/`)
- 引用脚本 `smoke_test_full_stack_qcgaf_gru.sh / start_full_stack_qcgaf_gru.sh / validate_dod_qcgaf_gru.sh` 在 git status 里**全部显示 `D`**
- `migration_decisions.md` 和 `topic_contract_ros2_qcgaf_gru.md` 也有 skbt2 路径残留

**动作:** 要么改写 README 引用 `run_full_pipeline.launch.py`,要么从 git 恢复必要脚本,二选一。

### 3.2 scripts/ 现在只剩 u_map_sweep.py
原有 30+ 个 `run_*.sh / start_*.sh / validate_*.sh / show_latest_*.sh / smoke_test_*.sh / check_*.sh` 已删。需要确认精简意图,如果是有意的,docs/ 全部引用要更新。

### 3.3 双 workspace 的 git status 都很乱
- `lvdot_ros2_ws`(老 workspace) 有一堆 M 但应该是 read-only
- `ros2_depth_eval_ws` 有 d435i/mid360/pose_stub/yolo_node/evaluator 一堆有意义改动 + 大量 model.sdf / world.sdf 改动 + `depth_eval_tools/` 整包被删 + `lvdot_realistic_sensors/` 整包是 untracked
- 都需要单独梳理 commit

### 3.4 跨 workspace 引用残留
有些 docs 还写 `/home/skbt2/...`(`migration_decisions.md` / `topic_contract_ros2_qcgaf_gru.md` / `README_migration_ops.md` / `CHANGELOG_TIMING_TUNED_V2.md`)。改成 `/home/mcb/`。

---

## 4. 优先级建议(按 ROI / 依赖关系)

```
P0(本周必做,blocker):
  1. §3.3 noise adaptation 加 debug 日志 → 验证 → commit          (1-2 h)
  2. 修 YOLO 4Hz → 15Hz+,让 yolo_in > 0                          (半天)
  3. 跑一次完整 gt_eval,§3.3 关 vs 开 两种基线                   (1 h)

P1(下周,论文章节依赖):
  4. detection_evaluator 加 IoU 0.3/0.5/0.7 + IDF1/IDSW          (1-2 天)
  5. 新写 prediction_evaluator (ADE/FDE)                          (1 天)
  6. §3.1 EMA 平滑 (代码 200 行内)                                (半天)
  7. §3.2 融合层迟滞                                              (1-2 天)

P2(2 周内,论文核心改进):
  8. §1.3.5 Mahalanobis + IoU3D 关联代价                          (2-3 天)
  9. §1.3.4 GRU γ 三因素                                          (1-2 天)
  10. §1.3.6 动态分类语义条件 (依赖 P0#2)                          (半天)

P3(论文章节最后一步):
  11. §4.3 消融实验 launch 开关 + A-G 7 配置跑数据                (3-5 天)
  12. 工程整理:README / scripts / 跨 ws commit                   (1 天)

明确不做(已在 THESIS_VS_CODE §4 决定):
  - KF 加加速度维度
  - YOLO 框拆分多人
  - 重训 QC-GAF 用 KITTI / Gazebo 退化集
  - 升 ROS 版本回 Noetic
```

总工作量 P0-P3 ≈ **3-5 周**(含调参 + sim 跑数据)。只做 P0+P1 → 2 周可拿到论文 §4.2 所需的全套数字。

---

## 5. 验证 checklist(每次评估都记)

- [ ] launch 启动后 11 节点都在
- [ ] dynamic / fused / pred 三 topic 都 ≥ 10 Hz
- [ ] YOLO topic ≥ 15 Hz(当下 ❌ 只有 4)
- [ ] `yolo_in / yolo_human > 0`(当下 ❌ 全 0)
- [ ] `skew_warn_avg < 5`
- [ ] QC `lidar_fallback / frames < 30%`(当下 ≈ 50%)
- [ ] KF q_scale / r_scale 随 H 浮动(当下未验证)
- [ ] tracked recall ≥ 0.20 @ 2.5m gate(基线 0.136)
- [ ] mean_position_error ≤ 1.0m(基线 1.28m)
- [ ] (新) IDSW < 0.5/track/min
- [ ] (新) ADE @ 2.5s ≤ 0.8m

---

## 6. 启动命令(实测可用,2026-05-12)

```bash
cd $LVDOT_ROOT/lvdot_ros2_migration_ws
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source install/setup.bash

# 全链路(headless)
ros2 launch lvdot_bringup run_full_pipeline.launch.py \
  gazebo_gui:=false rviz:=false \
  qcgaf_checkpoint:=<外部项目目录>/qcgaf_fusion/outputs/best_model.pt \
  gru_model:=<外部项目目录>/gru_predictor/outputs/nuscenes_3d_tuned/best_model.pth

# 带 GUI
ros2 launch lvdot_bringup run_full_pipeline.launch.py \
  gazebo_gui:=true rviz:=true

# 加 GT 评估
ros2 launch lvdot_bringup run_full_pipeline.launch.py \
  launch_evaluator:=true \
  evaluator_csv_path:=/tmp/lvdot_eval_full.csv \
  evaluator_match_threshold_m:=2.5
```

清理残留进程(`pkill -f` 在当前 sandbox 下不工作,只能用 `kill -9 PID`):
```bash
ps -ef | grep -E "ros2|gz sim|ign gazebo|lvdot_|qcgaf_|gru_pred|d435i_sim|mid360_sim|fusion_node|predict_node|pose_stub|pedestrian|ros_gz|rviz2" | grep -v grep | awk '{print $2}' | xargs -r kill -9
```
