# ROS2 Migration Stage Acceptance Report v1

Date: 2026-04-30
Workspace: `/home/skbt2/lvdot_ros2_migration_ws`
Scope: LV-DOT baseline + QCGAF + GRU migration execution progress

## 1. Executive Status

Overall status: **In Progress, major migration milestones completed**.

Completed high-value milestones:
- Isolated migration workspace established.
- D1 interface contract frozen.
- GRU ROS2 package migration completed and build verified.
- QCGAF ROS2 package migration completed and build verified.
- ROS2 detector compatibility publishers for QCGAF inputs added.
- Full-stack launch, smoke/regression/DoD validation scripts delivered.

Blocking item for final runtime acceptance:
- Real model checkpoint paths for QCGAF and GRU must be provided to execute full-stack runtime validation.

## 2. Delivered Artifacts

### 2.1 Planning and Contracts
- `/home/skbt2/lvdot_ros2_migration_ws/docs/topic_contract_ros2_qcgaf_gru.md`
- `/home/skbt2/lvdot_ros2_migration_ws/docs/migration_decisions.md`
- `/home/skbt2/lvdot_ros2_migration_ws/docs/README_migration_ops.md`
- `/home/skbt2/lvdot_ros2_migration_ws/docs/env_example.sh`

### 2.2 GRU ROS2 Migration
- `/home/skbt2/lvdot_ros2_migration_ws/src/gru_predictor/package.xml`
- `/home/skbt2/lvdot_ros2_migration_ws/src/gru_predictor/setup.py`
- `/home/skbt2/lvdot_ros2_migration_ws/src/gru_predictor/launch/gru_predictor.launch.py`
- `/home/skbt2/lvdot_ros2_migration_ws/src/gru_predictor/gru_predictor/predict_node.py`

### 2.3 QCGAF ROS2 Migration
- `/home/skbt2/lvdot_ros2_migration_ws/src/qcgaf_fusion/package.xml`
- `/home/skbt2/lvdot_ros2_migration_ws/src/qcgaf_fusion/setup.py`
- `/home/skbt2/lvdot_ros2_migration_ws/src/qcgaf_fusion/launch/qcgaf_fusion.launch.py`
- `/home/skbt2/lvdot_ros2_migration_ws/src/qcgaf_fusion/qcgaf_fusion/fusion_node.py`

### 2.4 Detector Compatibility Delta
- Added publishers in ROS2 detector:
  - `/onboard_detector/visual_bboxes_qcgaf`
  - `/onboard_detector/lidar_bboxes_qcgaf`
- Files:
  - `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_ros2/include/lvdot_ros2/lvdot_detector_node.hpp`
  - `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_ros2/src/lvdot_detector_node.cpp`

### 2.5 Full-Stack Ops Tooling
- `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/launch/run_full_stack_qcgaf_gru.launch.py`
- `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/start_full_stack_qcgaf_gru.sh`
- `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/check_qcgaf_gru_topics.sh`
- `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/smoke_test_full_stack_qcgaf_gru.sh`
- `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/run_regression_full_stack_qcgaf_gru.sh`
- `/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/validate_dod_qcgaf_gru.sh`

## 3. Build Verification Snapshot

Builds verified successful in this workspace:
- `gru_predictor`
- `qcgaf_fusion`
- `lvdot_core`
- `lvdot_interfaces`
- `lvdot_ros2`
- `lvdot_bringup`

Observed warnings:
- Existing PCL/CMake dev warnings in baseline packages.
- No migration-blocking compile errors after applied changes.

## 4. DoD Readiness Matrix

| DoD item | Current status | Evidence |
|---|---|---|
| Single command launch detector+qcgaf+gru | Ready (pending model paths) | full-stack launch file + start script |
| 30-min stable run | Pending runtime validation | DoD script ready |
| Required key topics output | Pending runtime validation | topic check script ready |
| Reproducible fixed command | Ready | documented in ops README |
| Regression artifacts/logs | Ready | regression + DoD scripts write reports |

## 5. Risks and Current Mitigation

1. QoS/time sync instability under certain bag/bridge scenarios
- Mitigation: frozen QoS baseline + dedicated smoke/DoD scripts.

2. Model/environment mismatch (torch/checkpoint format)
- Mitigation: explicit checkpoint/model parameters and path validation in nodes/scripts.

3. Semantic mismatch between detector outputs and QCGAF expectations
- Mitigation: strict compatibility topics restored (`*_qcgaf`) and contract documented.

## 6. Remaining Work to Reach Stage-2 Acceptance

1. Provide actual QCGAF checkpoint and GRU model paths.
2. Execute `validate_dod_qcgaf_gru.sh` with real models.
3. Run regression script and produce at least one PASS artifact bundle.
4. Record measured topic rates and stability duration.
5. Close any runtime issues (if surfaced) and re-run DoD.

## 7. Recommended Next Command

```bash
source /home/skbt2/lvdot_ros2_migration_ws/docs/env_example.sh
# edit env_example.sh with real model paths first
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/validate_dod_qcgaf_gru.sh
```

