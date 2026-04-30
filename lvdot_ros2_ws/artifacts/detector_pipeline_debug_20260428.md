# Detector Pipeline Debug Log — 2026-04-28

## Goal
Get end-to-end LV-DOT (ROS2) producing 3D detection boxes for pedestrians visible in
the depth_eval Gazebo scene. YOLO 2D works; **3D fusion path is producing 0 boxes**.

## Confirmed working
- **YOLO 2D**: directly running `yolo11n.pt` on `/camera/color/image_raw` returns
  `person 0.83`. `/yolo_detector/detected_image` shows the bounding box drawn.
  See `/tmp/yolo_input.png` and `/tmp/yolo_vis.png`.
- **/clock + use_sim_time** propagated to all nodes; depth/pose stamps now share
  sim-time (sec≈600).
- **Depth pipeline reaches** `voxelFilter`: `filtered_depth_cloud` has ~1.5k–2.4k
  points spread x∈[11.8, 17.2], y∈[-4.9, 1.7], z∈[0.18, 1.85] (geometry sane).

## Symptom
`/onboard_detector/pipeline_stats` reports
```
depth_samples=2377/285200 u_map=3 depth_boxes=0 u_map_merge=0 ... fusion=0
```
- 2377 of 285200 depth pixels survive the voxel filter (~0.8 %).
- U-map produces 3 line groups but 0 boxes after merge.
- All downstream (db / filtered / dynamic) marker arrays are empty (`DELETEALL`).
- `/onboard_detector/detected_color_image` therefore has no overlay (only the
  raw color frame with no boxes).

## Root cause investigation
1. **(Ruled out)** ground/roof clipping. `filtered_depth_cloud` z-range fits inside
   `[ground=0, roof=3]`.
2. **(Ruled out)** sensor pose / time mismatch. `depth_pose_sync_count=0` is a red
   herring (sync context not enabled by default), and stamps share sim_time.
3. **Voxel dedup** in `lvdot_core/src/detection_filter.cpp::voxelFilter`:
   each 0.1 m³ voxel emits one point only when its hit count reaches
   `voxelOccThresh`. Default 5.0 was killing 95 %+ of the cloud in sim
   (clean depth → few hits per voxel). Lowered to **1.0** → output now ≈
   unique-voxel count (2377), which matches geometry.
4. **U-map thresholds** still too strict for sim density. Lowered:
   - `u_map_threshold_point` 3 → 1
   - `u_map_threshold_line`  2 → 1
   - `u_map_min_length_line` 6 → 3
   - `dbscan_min_points_cluster` 20 → 5
   - `depth_skip_pixel` 2 → 1
5. After live restart with new params: still `u_map=3, depth_boxes=0`. So the
   U-map step *forms columns*, but the **column→box conversion / merge step**
   yields zero. Need to inspect `last_u_map_db_merge_count_` increment site
   and the U-map DB merge function in `lvdot_core` to see what gates output.

## Files changed
- `lvdot_ros2_ws/src/lvdot_bringup/config/detector_param.yaml`:
  - `voxel_occupied_thresh: 5.0 → 1.0`
  - `u_map_threshold_point: 3 → 1`
  - `u_map_threshold_line: 2 → 1`
  - `u_map_min_length_line: 6 → 3`
  - `dbscan_min_points_cluster: 20 → 5`
  - `depth_skip_pixel: 2 → 1`

Detector params are init-only — must restart `lvdot_detector_main` after edits.
Used `/tmp/restart_detector.sh` (loads new yaml, sim_time, executor_threads=4).

## Open question (next step)
Why U-map produces 3 line groups but **0 merged boxes** even with relaxed
thresholds. Candidates:
- `u_map_col_scale=0.5` may be collapsing too many columns.
- DB-merge IoU/range gates inside `detection_filter.cpp` (not yet inspected).
- 2377 points spread across multiple distant pillars / pedestrians may not
  form columns dense enough for the U-map line-segment detector at all.

## Repro
```bash
# scene + detector stack already running:
#   ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
#     gazebo_gui:=true rviz:=true detector_rviz:=true \
#     enable_yolo:=true launch_yolo_node:=true enable_vis_stage:=true
ros2 topic echo /onboard_detector/pipeline_stats --once
ros2 topic echo /onboard_detector/dbscan_bboxes --once  # action=3 (DELETEALL)
```
