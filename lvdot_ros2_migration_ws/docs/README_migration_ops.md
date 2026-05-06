# Migration Ops Quickstart

Workspace: `/home/skbt2/lvdot_ros2_migration_ws`

## Build

```bash
cd /home/skbt2/lvdot_ros2_migration_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Option A: pass model paths directly

```bash
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/start_full_stack_qcgaf_gru.sh \
  /path/to/qcgaf.pt /path/to/gru.pth
```

## Option B: set environment defaults once

```bash
source /home/skbt2/lvdot_ros2_migration_ws/docs/env_example.sh
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/start_full_stack_qcgaf_gru.sh
```

## Smoke Test

```bash
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/smoke_test_full_stack_qcgaf_gru.sh
```

## Regression Run

```bash
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/run_regression_full_stack_qcgaf_gru.sh \
  "${QCGAF_CHECKPOINT}" "${GRU_MODEL_PATH}" 300 false
```

## DoD Validation

```bash
/home/skbt2/lvdot_ros2_migration_ws/src/lvdot_bringup/scripts/validate_dod_qcgaf_gru.sh
```
