# Changelog: Timing Sync Hardening + Tuned V2 Default

Date: 2026-05-06
Workspace: `/home/skbt2/lvdot_ros2_migration_ws`

## 1) 目标

- 先解决工业级时序同步稳定性问题（skew告警高、融合分支频繁失效）。
- 在时序稳定后，进行最小侵入参数优化（不改核心算法结构）。

## 2) 代码级改动（时序硬化）

### 2.1 新增可配置时序参数
文件：`src/lvdot_ros2/include/lvdot_ros2/lvdot_detector_config.hpp`

新增参数：
- `enable_sync_context`
- `max_depth_lidar_skew_sec`
- `max_depth_yolo_skew_sec`
- `max_lidar_yolo_skew_sec`
- `stale_message_age_sec`
- `skew_startup_grace_sec`
- `future_stamp_tolerance_sec`

### 2.2 检测节点时序逻辑改造
文件：`src/lvdot_ros2/src/lvdot_detector_node.cpp`

改动要点：
- 由硬编码 skew 门限改为参数读取。
- 增加启动宽限（startup grace）避免刚启动时误判 stale。
- 增加消息年龄检查（stale age）和未来时间戳容忍（future tolerance）。
- `enable_sync_context` 从参数配置驱动。
- 启动日志打印完整时序配置，便于审计与回归。

### 2.3 场景启动链透传 sim time
文件：`src/lvdot_bringup/launch/run_detector_with_scene.launch.py`

改动：
- 新增 `use_sim_time` 启动参数并传给 detector launch。

## 3) 配置改动

### 3.1 默认配置加入时序参数
文件：`src/lvdot_bringup/config/detector_param.yaml`

加入：
- `enable_sync_context: true`
- `max_depth_lidar_skew_sec: 0.8`
- `max_depth_yolo_skew_sec: 0.8`
- `max_lidar_yolo_skew_sec: 1.0`
- `stale_message_age_sec: 2.0`
- `skew_startup_grace_sec: 8.0`
- `future_stamp_tolerance_sec: 0.2`

### 3.2 Tuned V2（并已设为默认）
文件：
- `src/lvdot_bringup/config/detector_param_tuned_v2.yaml`
- `src/lvdot_bringup/config/detector_param.yaml`（当前已覆盖为 tuned_v2）

关键参数：
- `lidar_DBSCAN_epsilon: 0.07`
- `filtering_BBox_IOU_threshold: 0.4`
- `max_match_range: 0.6`
- `max_size_diff_range: 0.6`
- `max_unmatched_frames: 8`
- `frame_skip: 3`

## 4) 评估脚本与结果

评估脚本：
- `src/lvdot_bringup/scripts/run_gt_eval_suite.sh`

### 4.1 时序硬化后基线（5x60, agents）
报告：`reports/gt_eval_suite_20260505_235651/summary.md`

- `skew_warn_avg: 1.00`
- `tracked recall: 0.084 ± 0.035`
- `tracked err: 1.299m ± 0.012`

### 4.2 Tuned V2（5x60, agents）
报告：`reports/gt_eval_suite_20260506_122212/summary.md`

- `skew_warn_avg: 1.00`（保持低位）
- `tracked recall: 0.136`（相对基线提升）
- `tracked err: 1.278m`（相对基线下降）

## 5) 结论

- 时序同步问题已从“高频告警+融合失效”降到稳定低告警状态。
- 在不改核心算法结构前提下，tuned_v2 带来可观增益。
- 当前推荐使用 `detector_param.yaml`（已是 tuned_v2）。

## 6) 后续建议

- 若业务目标要求 `tracked recall >= 0.20`，建议在当前基础上进入训练阶段：
  1. 先 GRU（轨迹连续性）
  2. 再 QCGAF（融合质量）

