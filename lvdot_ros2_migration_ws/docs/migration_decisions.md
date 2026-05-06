# Migration Decisions - ROS2 LV-DOT/QCGAF/GRU

Date: 2026-04-30
Workspace: `/home/skbt2/lvdot_ros2_migration_ws`

## D-001: Isolated workspace

Decision:
- Use only `/home/skbt2/lvdot_ros2_migration_ws` for migration coding.

Reason:
- Protect existing validated workspace `/home/skbt2/lvdot_ros2_ws` from accidental breakage.

Status:
- DONE

## D-002: QCGAF input strategy

Decision:
- Adopt Strategy A (strict compatibility):
  - add detector publishers for
    - `/onboard_detector/visual_bboxes_qcgaf`
    - `/onboard_detector/lidar_bboxes_qcgaf`

Reason:
- Matches ROS1 enhanced project runtime contract and avoids hidden semantic drift.

Consequence:
- Small C++ delta in `lvdot_ros2` now, lower integration uncertainty later.

## D-003: GRU migration order

Decision:
- Migrate `gru_predictor` first, then `qcgaf_fusion`.

Reason:
- GRU path is low-coupling (single input and output topic), ideal as ROS2 Python packaging template.

## D-004: Message schema policy

Decision:
- Keep `visualization_msgs/MarkerArray` for phase-1 migration.

Reason:
- Fastest path to preserve behavior and RViz observability.

Deferred:
- Potential custom typed message upgrade after stable ROS2 release.

## D-005: Time and QoS baseline

Decision:
- Keep ROS-time compatible launch (`use_sim_time` configurable).
- Start with conservative QoS:
  - sensor inputs: SensorDataQoS
  - marker inputs/outputs: reliable depth 10

Reason:
- Balance reproducibility and delivery speed.

Risk note:
- If ATS sync jitter appears, fallback to explicit time-window matching for QCGAF.

