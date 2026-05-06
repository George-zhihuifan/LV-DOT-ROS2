# Topic Contract (ROS2) - LV-DOT / QCGAF / GRU

Date: 2026-04-30
Workspace: `/home/skbt2/lvdot_ros2_migration_ws`
Purpose: Freeze ROS2 interface contract before migration coding.

## 1) Scope

This contract covers runtime interfaces for:
- `lvdot_ros2` detector node
- to-be-migrated `qcgaf_fusion` node
- to-be-migrated `gru_predictor` node

## 2) Time Source Contract

- Default: ROS time compatible (`use_sim_time` configurable at launch).
- Requirement: all marker outputs stamp with node clock time or synchronized input stamp.
- For bag replay: `use_sim_time=true` must be set consistently across detector/qcgaf/gru.

## 3) Frame Contract

- Box marker frame must stay consistent with detector output frame.
- Current detector behavior publishes marker arrays in runtime frame (commonly `map`).
- QCGAF and GRU outputs must inherit source frame unless explicit transform stage is introduced.

## 4) Detector Inputs (already in ROS2 baseline)

Node: `lvdot_detector_node`

- `depth_image_topic` (`sensor_msgs/msg/Image`) default `/camera/depth/image_rect_raw`
- `color_image_topic` (`sensor_msgs/msg/Image`) default `/camera/color/image_raw`
- `lidar_pointcloud_topic` (`sensor_msgs/msg/PointCloud2`) default `/pointcloud`
- `pose_topic` (`geometry_msgs/msg/PoseStamped`) default `/mavros/local_position/pose`
- `odom_topic` (`nav_msgs/msg/Odometry`) default `/mavros/local_position/odom`
- `yolo_detection_topic` (`vision_msgs/msg/Detection2DArray`) default `/yolo_detector/detected_bounding_boxes`

QoS (current baseline):
- sensor streams: `SensorDataQoS`
- state streams: keep last 10

## 5) Detector Outputs (already in ROS2 baseline)

MarkerArray:
- `/onboard_detector/uv_bboxes`
- `/onboard_detector/dbscan_bboxes`
- `/onboard_detector/visual_bboxes`
- `/onboard_detector/lidar_bboxes`
- `/onboard_detector/filtered_before_yolo_bboxes`
- `/onboard_detector/filtered_bboxes`
- `/onboard_detector/tracked_bboxes`
- `/onboard_detector/dynamic_bboxes`
- `/onboard_detector/history_trajectories`
- `/onboard_detector/velocity_visualizaton`

PointCloud2/Image/Status topics remain unchanged from baseline and are out of this migration core scope.

## 6) QCGAF Contract (target ROS2)

Input topics (frozen):
- `/onboard_detector/visual_bboxes_qcgaf` (`visualization_msgs/msg/MarkerArray`)
- `/onboard_detector/lidar_bboxes_qcgaf` (`visualization_msgs/msg/MarkerArray`)
- `/camera/color/image_raw` or configured color topic (`sensor_msgs/msg/Image`)
- `/camera/depth/image_raw` or `/camera/depth/image_rect_raw` (`sensor_msgs/msg/Image`)
- `/yolo_detector/detected_bounding_boxes` (`vision_msgs/msg/Detection2DArray`)
- `/pointcloud` or `/livox/lidar` (`sensor_msgs/msg/PointCloud2`)
- `/imu/data` (`sensor_msgs/msg/Imu`)

Output topic:
- `/qcgaf/fused_bboxes` (`visualization_msgs/msg/MarkerArray`)

QoS policy (initial):
- synced marker inputs: reliable, depth 10
- sensor inputs (image/cloud/imu): SensorDataQoS
- output fused boxes: reliable, depth 10

## 7) GRU Contract (target ROS2)

Input topic:
- `/onboard_detector/dynamic_bboxes` (`visualization_msgs/msg/MarkerArray`)

Output topic:
- `/gru_predictor/predicted_positions` (`visualization_msgs/msg/MarkerArray`)

QoS policy (initial):
- input: reliable, depth 10
- output: reliable, depth 10

## 8) Compatibility Mapping

ROS1 enhanced project behavior to preserve:
- QCGAF consumes `visual_bboxes_qcgaf` and `lidar_bboxes_qcgaf`.
- QCGAF publishes `/qcgaf/fused_bboxes` for detector-side fusion override path.
- GRU consumes `/onboard_detector/dynamic_bboxes` and publishes prediction markers.

## 9) Gaps Identified (must be implemented)

In current ROS2 baseline copy, detector does NOT publish:
- `/onboard_detector/visual_bboxes_qcgaf`
- `/onboard_detector/lidar_bboxes_qcgaf`

These two publishers are required for strict ROS1 behavior equivalence.

## 10) Validation Commands

```bash
# after migration coding/build
ros2 topic list | grep -E 'qcgaf|gru_predictor|onboard_detector/(dynamic_bboxes|visual_bboxes_qcgaf|lidar_bboxes_qcgaf)'
ros2 topic echo /onboard_detector/dynamic_bboxes --once
ros2 topic echo /qcgaf/fused_bboxes --once
ros2 topic echo /gru_predictor/predicted_positions --once
```

