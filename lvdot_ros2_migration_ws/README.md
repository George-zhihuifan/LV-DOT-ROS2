# LV-DOT ROS2 Migration Mainline

This workspace is the current mainline for the LV-DOT ROS2 perception stack used by
the QC-GAF paper experiments. It contains the migrated detector, QC-GAF fusion,
GRU predictor, launch files, scenario configs, and evaluation utilities.

## Role

The full project currently uses two workspaces:

- `$LVDOT_ROOT/ros2_depth_eval_ws`: Gazebo scene, UAV/pedestrian
  simulation, sensor bridge, pose stub, YOLO adapter, and evaluators.
- `$LVDOT_ROOT/lvdot_ros2_migration_ws`: main detector, QC-GAF,
  GRU, bringup launch files, RViz config, experiment configs, and reports.

The older `$LVDOT_ROOT/lvdot_ros2_ws` workspace is not the current
mainline.

## Packages

- `lvdot_core`: shared C++ detector and tracking components.
- `lvdot_ros2`: ROS2 detector node and runtime bridge.
- `lvdot_bringup`: launch files, detector configs, scenario configs, RViz, and
  evaluation scripts.
- `qcgaf_fusion`: quality-aware gated attention fusion node.
- `gru_predictor`: hybrid Kalman/GRU trajectory prediction node.
- `lvdot_interfaces`: custom ROS2 interfaces.

## Build

Build the simulation/support workspace first, then this workspace:

```bash
cd $LVDOT_ROOT/ros2_depth_eval_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

cd $LVDOT_ROOT/lvdot_ros2_migration_ws
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
colcon build --symlink-install
```

For each new terminal:

```bash
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source $LVDOT_ROOT/lvdot_ros2_migration_ws/install/setup.bash
```

## Current Mainline Startup

Standard QC-GAF inspection chain:

```bash
ros2 launch lvdot_bringup run_full_pipeline.launch.py \
  gazebo_gui:=true \
  rviz:=true \
  use_realistic_sensors:=true \
  scenario_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/config/agent_count_scenarios/pedestrian_dense_01agents.yaml \
  detector_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml \
  enable_qcgaf:=true \
  enable_gru:=false \
  launch_evaluator:=false \
  launch_advanced_evaluator:=false
```

Clean 2-agent paper-style run:

```bash
ros2 launch lvdot_bringup run_full_pipeline_clean_2agents.launch.py \
  gazebo_gui:=true \
  rviz:=true \
  use_realistic_sensors:=true \
  enable_yolo:=true \
  launch_yolo_node:=true \
  enable_qcgaf:=true \
  enable_gru:=true
```

Matching clean scene launch files exist for 1/2/3/4/5/6 agents, plus static
obstacle and lidar360 variants.

## Local Runtime Assets

The launch files expect local model assets under:

- `models/qcgaf/best_model.pt`
- `models/gru/best_model.pth`
- `models/yolo/yolo11n.engine` or compatible YOLO weight

Large generated outputs such as `build/`, `install/`, `log/`, `logs/`,
`datasets/`, and `models/` are treated as local runtime artifacts rather than
source files.

## Useful Docs

- `docs/QCGAF_PIPELINE_STARTUP.md`: detailed launch hierarchy and topic routing.
- `docs/PROJECT_OVERVIEW.md`: system overview and data flow.
- `docs/PIPELINE_DATA_FLOW.md`: topic-level pipeline contract.
- `docs/EXPERIMENT_DESIGN.md`: experiment setup notes.
- `docs/THESIS_VS_CODE_GAP_ANALYSIS.md`: comparison between broader thesis plan
  and implemented code.

## Notes

The current paper mainline is QC-GAF-focused. GRU and broader LV-DOT tracking
components remain in the workspace, but the CAC2026 manuscript primarily uses
QC-GAF fusion and its controlled evaluation evidence.
