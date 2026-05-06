# ROS2 Migration Stage Acceptance Report v2

Date: 2026-05-01
Workspace: `/home/skbt2/lvdot_ros2_migration_ws`

## 1. Final Status (Current Stage)

Status: **Completed for planned migration scope, with strict scene-based DoD PASS**.

What is complete:
- ROS2 migration of `gru_predictor` and `qcgaf_fusion`.
- ROS2 detector compatibility topics for QC-GAF input:
  - `/onboard_detector/visual_bboxes_qcgaf`
  - `/onboard_detector/lidar_bboxes_qcgaf`
- Full-stack launch and validation tooling delivered.
- Real model weights integrated from GitHub commit `405a9bd`.
- Gazebo pedestrian + UAV scene full pipeline executed.
- 3 independent regression runs completed.
- Strict traffic DoD in scene mode passed.

## 2. Runtime Validation Executed

### 2.1 DoD validations
- No-input baseline scenario: `PASS_WITH_NO_TRAFFIC`
  - `/home/skbt2/lvdot_ros2_migration_ws/reports/dod_validation_20260501_125440`
- Strict scene-based validation: `PASS`
  - `/home/skbt2/lvdot_ros2_migration_ws/reports/dod_validation_20260501_150454`
  - `/home/skbt2/lvdot_ros2_migration_ws/reports/dod_validation_20260501_153324`

### 2.2 Regression runs (3 groups)
- `/home/skbt2/lvdot_ros2_migration_ws/reports/full_stack_run_20260501_130448`
- `/home/skbt2/lvdot_ros2_migration_ws/reports/full_stack_run_20260501_135615`
- `/home/skbt2/lvdot_ros2_migration_ws/reports/full_stack_run_20260501_140048`

## 3. Quality hardening implemented after code review

- Input validation for image decoding in `qcgaf_fusion`.
- Callback-level exception guards for QCGAF/GRU runtime paths.
- Sync jitter warning for QCGAF camera/lidar inputs.
- GRU extrapolation logic fix (avoid repeated future points).
- Parameter boundary checks (Python + C++ detector config fallback).
- DoD scripts upgraded:
  - topic presence + publisher count checks
  - strict traffic threshold checks (`min_hz_*`)
  - scene-mode strict validation support

## 4. Key Deliverables

### Docs
- `/home/skbt2/lvdot_ros2_migration_ws/docs/topic_contract_ros2_qcgaf_gru.md`
- `/home/skbt2/lvdot_ros2_migration_ws/docs/migration_decisions.md`
- `/home/skbt2/lvdot_ros2_migration_ws/docs/README_migration_ops.md`
- `/home/skbt2/lvdot_ros2_migration_ws/docs/env_example.sh`

### Packages
- `gru_predictor` (ROS2 ament_python)
- `qcgaf_fusion` (ROS2 ament_python)
- `lvdot_ros2` (compatibility topics + config hardening)
- `lvdot_core` (input validity hardening)
- `lvdot_bringup` (full-stack launch + ops scripts)

### Ops scripts
- `start_full_stack_qcgaf_gru.sh`
- `smoke_test_full_stack_qcgaf_gru.sh`
- `run_regression_full_stack_qcgaf_gru.sh`
- `validate_dod_qcgaf_gru.sh`
- `check_qcgaf_gru_topics.sh`

## 5. Acceptance Conclusion

For the migration scope defined in the execution plan (baseline ROS2 + QC-GAF + GRU integration and runnable full pipeline),
**the work is accepted with strict runtime validation passed**.

