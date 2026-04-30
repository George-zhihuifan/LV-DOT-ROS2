# Incident Report: Full Stack GUI/RViz/Blue BBox Issues
Date: 2026-04-29
Workspace: /home/skbt2/lvdot_ros2_ws + /home/skbt2/ros2_depth_eval_ws

## 1. User-Observed Symptoms
1. GUI 窗口（Gazebo/RViz）出现后闪退或消失。
2. RViz 中“蓝框”不可见（点云相关框看不到或时有时无）。
3. 全流程状态看起来混乱，行为不一致。

## 2. What Was Done
### 2.1 Data-path / stats checks (UV / DB / Fusion / LiDAR)
- 检查了 `PipelineStats.msg` 字段定义与发布赋值链路。
- 抽查了 2026-04-29 最新有效回归产物（如 `real_scene_regression_20260429_165240`, `163409`）。
- 结论：有效回归样本为 PASS，UV/DB/Fusion/LiDAR 分支在有效样本中有正常输出。

### 2.2 Depth source / camera parameter checks
- 检查了深度图编码与单位转换逻辑（`32FC1` / `16UC1` + `depth_scale_factor`）。
- 检查了当前参数配置来源（`detector_param.yaml`）和 relay 行为。
- 结论：深度转换逻辑基本正确；准确性与覆盖率受固定参数与场景影响，已有评估结果显示 valid ratio 约 50% 左右。

### 2.3 Blue BBox issue handling
- 定位 RViz 配置问题：
  - `DB BBoxes` 默认关闭。
  - `Filtered BBoxes` 默认关闭。
  - `DB BBoxes` topic 配置不匹配（RViz用 `/onboard_detector/db_bboxes`，节点实际发 `/onboard_detector/dbscan_bboxes`）。
- 已修改文件：
  - `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz`
- 修改内容：
  - `DB BBoxes` -> `Enabled: true`
  - `DB BBoxes` topic -> `/onboard_detector/dbscan_bboxes`
  - `Filtered BBoxes` -> `Enabled: true`

### 2.4 GUI flash/exit handling
- 识别到使用普通后台方式启动时，进程可能被会话回收，导致窗口闪退。
- 尝试了 `nohup` 与 `setsid + disown` 启动方式。
- `setsid + disown` 模式下可观察到 `gz sim -g`、`rviz2`、`lvdot_detector_main` 持续存活。

## 3. Current Findings (Important)
1. 出现过“多实例并行运行”情况（多套 `ros2 launch` / `lvdot_detector_main` / `rviz2` / `gz sim` 同时存在）。
2. 多实例导致 `/clock` 多源或时间回跳，RViz日志出现大量：
   - `Detected jump back in time. Resetting RViz.`
3. 这会直接造成 RViz 显示重置、观感上像“闪退/不稳定/框消失”。

## 4. What Still Remains Problematic
1. 进程清场不彻底时，容易再次进入多实例冲突状态。
2. RViz 虽已改配置，但如果多实例同时推流，显示仍可能不稳定。
3. `u_map_enhanced_*` 统计长期为0的问题仍存在（根因是增强标志置位链路未完全打通），不影响 GUI 显示，但影响诊断解释力。

## 5. Recommended Stable Procedure
1. 每次启动前先强制全停（单实例纪律）。
2. 仅保留一套 scene + detector + rviz。
3. 启动后先检查：
   - 只有一条 `ros2 launch depth_eval_bringup ...`
   - 只有一条 `ros2 launch lvdot_bringup ...`
   - 只有一条 `gz sim -g`
   - 只有一条 `rviz2 -d ...lvdot_detector.rviz`
4. 若 RViz 再次提示 time jump，优先排查是否又启动了第二套流程。

## 6. Related Logs / Artifacts
- Issue log (blue bbox):
  - `/home/skbt2/lvdot_ros2_ws/artifacts/issue_log_2026-04-29_blue_bbox.md`
- This report:
  - `/home/skbt2/lvdot_ros2_ws/artifacts/incident_report_2026-04-29_full_stack_gui_rviz.md`
- Runtime logs:
  - `/tmp/full_stack_scene.log`
  - `/tmp/full_stack_lvdot.log`
  - `/tmp/full_stack_rviz.log`

