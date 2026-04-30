# Pipeline Performance Tuning — 2026-04-28

## TL;DR
RViz + Gazebo GUI + missing mesa env were starving the gz render thread.
After fixes, with **no RViz / no gz GUI**:

| Topic | Before | After |
|---|---|---|
| `/rgbd_camera/image` | 5.0 Hz | **28.5 Hz** |
| `/rgbd_camera/depth_image` | 13.0 Hz | **26.6 Hz** |
| `/rgbd_camera/points` | 8.1 Hz | **23.6 Hz** |
| `/yolo_detector/detected_image` | 3.4 Hz → 6.7 Hz | **9.0 Hz** |

`update_rate` in SDF is now 30; YOLO inference timer is the new ceiling
(`inference_hz: 10.0` in `/tmp/yolo_params.yaml`).

## Root causes found

### 1. RViz2 was the dominant CPU hog (~265% CPU)
mesa→d3d12 software path makes RViz extremely expensive. Killing RViz
alone moved `/rgbd_camera/image` from 1.5 Hz → 5 Hz, then to 28 Hz once
the render env was fixed too.

### 2. `GALLIUM_DRIVER` / `MESA_LOADER_DRIVER_OVERRIDE` were unset
gz was falling back to llvmpipe (pure CPU rasterizer). Setting both to
`d3d12` makes gz use the WSL2 D3D12 GPU passthrough, ~6× faster on the
RGB camera. Neither launch file currently exports these — must be
exported in the shell or wrapped by a launch script.

### 3. Both gz GUI and gz server compete for cores
`gz sim -g` (the GUI) was a separate ruby process at ~150% CPU. Disabling
GUI (`gazebo_gui:=false`) recovers ~150% CPU.

### 4. `image_pointcloud_relay` was still running after relay-bypass
ROS-Python forwarder was a no-op orphan eating 49% CPU. Disabled in
`run_detector_with_scene.launch.py` by changing both `launch_relay` and
`relay_lvdot_topics` defaults to `false`. Detector + YOLO already
subscribe `/rgbd_camera/*` directly.

### 5. SDF camera `update_rate: 15` was the original ceiling
Bumped to `30` in `models/uav_d435i_platform/model.sdf`. Also disabled
`<visualize>` and `<enable_metrics>` on the rgbd sensor (small wins,
worth taking).

## Files changed
- `ros2_depth_eval_ws/src/depth_eval_bringup/models/uav_d435i_platform/model.sdf`
  - `update_rate: 15 → 30`
  - `visualize: true → false`
  - `enable_metrics: true → false`
- `lvdot_ros2_ws/src/lvdot_bringup/launch/run_detector_with_scene.launch.py`
  - `relay_lvdot_topics: "true" → "false"`
  - `launch_relay: "true" → "false"`

## How to launch (fast path)
```bash
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source /home/skbt2/lvdot_ros2_ws/install/setup.bash

export GALLIUM_DRIVER=d3d12
export MESA_LOADER_DRIVER_OVERRIDE=d3d12

ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false rviz:=false detector_rviz:=false \
    enable_yolo:=true launch_yolo_node:=true enable_vis_stage:=true
```
RViz can be opened separately when needed; expect rates to drop ~30%
while it's open. Open one image panel at a time if framerate matters.

## Open / next levers (not done today)
- **inference_hz: 10 → 25**: YOLO is GPU-offloaded; current 8.9 Hz output
  is timer-limited, not GPU-limited. Bump and re-measure.
- **Bake GALLIUM_DRIVER=d3d12 into launch**: a tiny wrapper or
  `SetEnvironmentVariable` action in the launch file would prevent
  llvmpipe regressions.
- **3D fusion still produces 0 boxes** (separate issue from
  `detector_pipeline_debug_20260428.md` — U-map 6 lines → 0 merged
  boxes). Needs inspection of `detection_filter.cpp` U-map DB merge
  function.
- **detector_main 354% → 22% CPU** drop: detector now barely working
  because depth pipeline output is 0. Real load will return when
  fusion is fixed.

## Current process map (running)
- `gz sim -s` (server only)
- `parameter_bridge` (gz↔ros)
- `lvdot_detector_main`
- `lvdot_yolo_node` (subs `/rgbd_camera/image`)
- `pedestrian_state_publisher` + `uav_trajectory_publisher`
