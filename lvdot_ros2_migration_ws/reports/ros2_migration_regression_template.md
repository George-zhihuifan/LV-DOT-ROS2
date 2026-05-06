# ROS2 Migration Regression Report

Date:
Operator:
Workspace: `/home/skbt2/lvdot_ros2_migration_ws`

## 1. Run Setup

- Detector config:
- QCGAF config:
- QCGAF checkpoint:
- GRU config:
- GRU model:
- use_sim_time:
- Data source (bag/live):

## 2. Startup Commands

```bash
source /opt/ros/jazzy/setup.bash
source /home/skbt2/lvdot_ros2_migration_ws/install/setup.bash
ros2 launch lvdot_bringup run_full_stack_qcgaf_gru.launch.py \
  qcgaf_checkpoint:=... \
  gru_model:=...
```

## 3. Topic Presence

- `/onboard_detector/visual_bboxes_qcgaf`:
- `/onboard_detector/lidar_bboxes_qcgaf`:
- `/onboard_detector/dynamic_bboxes`:
- `/qcgaf/fused_bboxes`:
- `/gru_predictor/predicted_positions`:

## 4. Runtime Metrics

- Duration (min):
- Detector topic rate:
- QCGAF output rate:
- GRU output rate:
- Node crash/restart count:
- Max observed latency:

## 5. Functional Checks

- QCGAF fused boxes visually aligned (Y/N):
- GRU trajectory direction consistent (Y/N):
- Frame IDs consistent across outputs (Y/N):
- ROS time behavior correct under replay (Y/N):

## 6. Issues and Fixes

1.
2.
3.

## 7. Verdict

- [ ] PASS
- [ ] FAIL

Notes:

