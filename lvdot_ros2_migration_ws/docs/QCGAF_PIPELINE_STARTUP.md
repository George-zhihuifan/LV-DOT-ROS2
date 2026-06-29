# QC-GAF Pipeline Startup Guide

## 1. Purpose

This document explains how the current `QC-GAF` mainline pipeline is started, which files are responsible for Gazebo and RViz, and what the standard launch command is for the `1-agent + A4(QC-GAF)` inspection setup.

The goal is to answer four concrete questions:

1. Which command should be used to start the full chain?
2. Which launch file actually loads Gazebo?
3. Which world and scenario files are being used?
4. Which RViz config is being loaded, and how can it be started separately?

## 2. Workspace and Environment

The full chain spans two workspaces:

- `ros2_depth_eval_ws`
- `lvdot_ros2_migration_ws`

Source them in this order:

```bash
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source $LVDOT_ROOT/lvdot_ros2_migration_ws/install/setup.bash
```

## 3. Recommended Full Startup Command

This is the current standard command for the `1-agent + A4(QC-GAF)` visual inspection chain:

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

This command starts:

- Gazebo server
- Gazebo GUI
- `ros_gz_bridge`
- `pedestrian_state_publisher`
- `d435i_sim`
- `mid360_sim`
- `pose_stub`
- `lvdot_yolo_node`
- `lvdot_detector_main`
- `qcgaf_fusion_node`
- RViz

## 4. Launch File Hierarchy

The actual startup is layered. The top-level command above does not directly create the world and detector by itself; it includes lower-level launch files.

### 4.1 Top-level full pipeline

File:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/launch/run_full_pipeline.launch.py`

Responsibilities:

- defines the full production chain
- includes scene + realistic sensor overlay + detector stack
- starts `qcgaf_fusion_node`
- optionally starts `gru_prediction_node`
- starts top-level RViz

Important detail:

- this file includes `run_detector_with_scene.launch.py`
- when including it, it explicitly passes:
  - `rviz := false`
  - `detector_rviz := false`

That means the RViz window you see in the full chain is the RViz node defined in `run_full_pipeline.launch.py`, not the RViz node inside lower launch files.

### 4.2 Scene + sensor + detector wrapper

File:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/launch/run_detector_with_scene.launch.py`

Responsibilities:

- includes the Gazebo scene launch from `depth_eval_bringup`
- includes the realistic sensor overlay launch
- includes the detector/adapter launch
- switches topic sources depending on `use_realistic_sensors`

When `use_realistic_sensors:=true`, the detector consumes:

- color: `/d435i/color/image_raw`
- depth: `/d435i/depth/image_rect_raw`
- lidar: `/mid360/pointcloud`

When `use_realistic_sensors:=false`, it falls back to raw Gazebo topics.

### 4.3 Detector + YOLO + pose stub

File:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/launch/run_detector_with_adapter.launch.py`

Responsibilities:

- starts `pose_stub` or `uav_waypoint_mission`
- starts `lvdot_yolo_node`
- starts `lvdot_detector_main`
- optionally starts detector-side RViz
- optionally starts evaluator nodes

This file is where detector-side parameters such as these are wired:

- `depth_branch_offset_sec`
- `depth_branch_history_size`
- `max_depth_lidar_skew_sec`
- `inference_hz`
- `imgsz`
- `max_det`

### 4.4 Gazebo scene launch

File:

- `$LVDOT_ROOT/ros2_depth_eval_ws/src/depth_eval_bringup/launch/uav_pedestrian_prototype.launch.py`

Responsibilities:

- starts Gazebo server
- optionally starts Gazebo GUI
- starts `ros_gz_bridge`
- starts `pedestrian_state_publisher`
- optionally starts scene-side RViz

This is the file that actually launches Gazebo with:

```bash
ign gazebo -s -r --physics-engine ignition-physics5-dartsim-plugin <world>
```

Important detail:

- Gazebo server is created with `on_exit=Shutdown()`
- if the Gazebo server process exits, the whole scene launch is torn down

## 5. Which World File Gazebo Loads

The Gazebo world file is:

- `$LVDOT_ROOT/ros2_depth_eval_ws/src/depth_eval_bringup/worlds/pedestrian_prototype.sdf`

At runtime, the installed copy is used:

- `$LVDOT_ROOT/ros2_depth_eval_ws/install/depth_eval_bringup/share/depth_eval_bringup/worlds/pedestrian_prototype.sdf`

This world contains the UAV scene and the plugins needed by the prototype pipeline.

Two scene plugins that appear in startup logs are:

- `uav_pose_sync_system`
- `pedestrian_pose_sync_system`

## 6. Which Scenario File Controls the Agent Layout

For the `1-agent` inspection case, the scenario file is:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/config/agent_count_scenarios/pedestrian_dense_01agents.yaml`

This file controls the runtime pedestrian layout used by `pedestrian_state_publisher` and the pose/agent-state machinery.

If you want `2/3/4/5/6` agents, switch this file to the matching YAML in the same directory.

## 7. Which Detector Config Is Used for A4

For the current `A4(QC-GAF)` mainline inspection command, the detector config is:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml`

In the current workflow:

- the detector stays on baseline detector parameters
- `A4` is introduced by turning on `enable_qcgaf:=true`
- `GRU` is intentionally disabled during inspection with `enable_gru:=false`

If later you want to inspect a different detector parameter set, replace `detector_config:=...`.

## 8. Which RViz Config Is Loaded

The full pipeline RViz config is:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz`

This file is referenced by:

- `$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/launch/run_full_pipeline.launch.py`

The RViz node started by the top-level launch is:

- package: `rviz2`
- executable: `rviz2`
- name: `lvdot_full_pipeline_rviz`

## 9. How To Start RViz Separately

If the full chain is already running and you only want to open RViz manually:

```bash
source /opt/ros/humble/setup.bash
source $LVDOT_ROOT/ros2_depth_eval_ws/install/setup.bash
source $LVDOT_ROOT/lvdot_ros2_migration_ws/install/setup.bash

rviz2 -d $LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz
```

## 10. How To Start Gazebo Separately

If you only want the scene without the detector/fusion stack:

```bash
ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py \
  gazebo_gui:=true \
  rviz:=false \
  scenario_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/config/agent_count_scenarios/pedestrian_dense_01agents.yaml
```

This starts:

- Gazebo server
- Gazebo GUI
- scene bridge
- pedestrian state publisher

It does not start:

- `d435i_sim`
- `mid360_sim`
- `lvdot_yolo_node`
- `lvdot_detector_main`
- `qcgaf_fusion_node`

## 11. Main Runtime Topics Worth Watching

For current manual inspection, these topics matter most.

### 11.1 Sensor streams

- `/rgbd_camera/depth_image`
- `/d435i/depth/image_rect_raw`
- `/d435i/color/image_raw`
- `/mid360/pointcloud`

### 11.2 Detector outputs

- `/onboard_detector/visual_bboxes_qcgaf`
- `/onboard_detector/lidar_bboxes_qcgaf`
- `/onboard_detector/dynamic_bboxes`

### 11.3 Fusion output

- `/qcgaf/fused_bboxes`

### 11.4 YOLO side

- `/yolo_detector/detected_bounding_boxes`
- `/yolo_detector/detected_image`

## 12. Recommended RViz Checks

When inspecting the `1-agent + A4` scene manually, focus on these comparisons:

1. raw Gazebo depth vs realistic depth
   - `/rgbd_camera/depth_image`
   - `/d435i/depth/image_rect_raw`
2. LiDAR detections vs visual detections
   - `/onboard_detector/lidar_bboxes_qcgaf`
   - `/onboard_detector/visual_bboxes_qcgaf`
3. detector output vs fused output
   - `/onboard_detector/dynamic_bboxes`
   - `/qcgaf/fused_bboxes`

If fusion is healthy, the fused output should not stay in pure LiDAR fallback all the time.

## 13. Current Known Runtime Observation

In the current `1-agent + A4` manual run, the following symptoms were observed:

- `QCGAF metrics` kept reporting `lidar_fallback = frames`
- `d435i_sim` reported low valid depth ratio and high `Inf` ratio
- detector logs showed depth branch activity, but only a very small fraction of projected depth samples were valid

This means the chain is starting and processing data, but the fusion path is not yet behaving as intended.

## 14. Quick Command Summary

Full `1-agent + A4(QC-GAF)`:

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

Gazebo-only scene:

```bash
ros2 launch depth_eval_bringup uav_pedestrian_prototype.launch.py \
  gazebo_gui:=true \
  rviz:=false \
  scenario_config:=$LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/config/agent_count_scenarios/pedestrian_dense_01agents.yaml
```

RViz-only:

```bash
rviz2 -d $LVDOT_ROOT/lvdot_ros2_migration_ws/src/lvdot_bringup/rviz/lvdot_detector.rviz
```
