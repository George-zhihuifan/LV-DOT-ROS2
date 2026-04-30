# Issue Log - Blue BBox Missing (2026-04-29)

## Symptom
RViz中点云蓝框（用户口述）出现后闪退/消失，后续不可见。

## Root Cause
1. 进程生命周期问题：之前启动方式绑定当前会话，导致GUI窗口闪退。
2. RViz配置问题：
   - `DB BBoxes` 默认关闭（Enabled: false）。
   - `Filtered BBoxes` 默认关闭（Enabled: false）。
   - `DB BBoxes` 订阅topic错误：`/onboard_detector/db_bboxes`，而节点实际发布为 `/onboard_detector/dbscan_bboxes`。

## Fix Applied
1. 改用稳定启动方式（setsid + disown）保持Gazebo/RViz不随会话退出。
2. 更新RViz配置文件：
   - 打开 `DB BBoxes`。
   - 打开 `Filtered BBoxes`。
   - 将 `DB BBoxes` topic 改为 `/onboard_detector/dbscan_bboxes`。

## Modified File
- `/home/skbt2/lvdot_ros2_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz`

## Verification
- `rviz2` 进程存在。
- `gz sim -g` 进程存在。
- 日志显示RViz正常订阅点云topic。

