#pragma once

#include <string>
#include <vector>

namespace lvdot_ros2
{

struct LVdotDetectorConfig
{
  int localization_mode{0};

  std::string depth_image_topic{"/camera/depth/image_rect_raw"};
  std::string color_image_topic{"/camera/color/image_raw"};
  std::string lidar_pointcloud_topic{"/pointcloud"};
  std::string pose_topic{"/mavros/local_position/pose"};
  std::string odom_topic{"/mavros/local_position/odom"};
  std::string yolo_detection_topic{"/yolo_detector/detected_bounding_boxes"};
  // dual: run both depth and lidar fusion paths
  // depth_driven: only depth path writes fusion outputs
  // lidar_driven: only lidar path writes fusion outputs
  std::string fusion_mode{"dual"};

  std::vector<double> depth_intrinsics{554.3827128226441, 554.3827128226441, 320.0, 240.0};
  std::vector<double> color_intrinsics{554.3827128226441, 554.3827128226441, 320.0, 240.0};
  double depth_scale_factor{1.0};
  double depth_min_value{0.2};
  double depth_max_value{12.0};
  int u_map_row_downsample{4};
  double u_map_col_scale{0.5};
  int u_map_threshold_point{3};
  int u_map_threshold_line{2};
  int u_map_min_length_line{6};
  int depth_filter_margin{10};
  int depth_skip_pixel{2};
  int image_cols{640};
  int image_rows{480};

  std::vector<double> body_to_camera_depth{
    1.0, 0.0, 0.0, 0.18,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.06,
    0.0, 0.0, 0.0, 1.0
  };
  std::vector<double> body_to_camera_color{
    1.0, 0.0, 0.0, 0.18,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.06,
    0.0, 0.0, 0.0, 1.0
  };
  std::vector<double> body_to_lidar{
    1.0, 0.0, 0.0, 0.18,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.06,
    0.0, 0.0, 0.0, 1.0
  };

  double time_step{0.033};
  double ground_height{0.0};
  double roof_height{3.0};
  double voxel_occupied_thresh{5.0};
  std::vector<double> local_sensor_range{5.0, 5.0, 5.0};
  int dbscan_min_points_cluster{20};
  double dbscan_search_range_epsilon{0.05};
  int lidar_dbscan_min_points{10};
  double lidar_dbscan_epsilon{0.05};
  int post_min_cluster_points{0};
  int downsample_threshold{3500};
  int gaussian_downsample_rate{6};
  double filtering_bbox_iou_threshold{0.2};
  double max_match_range{0.5};
  double max_size_diff_range{0.5};
  std::vector<double> feature_weight{3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0.0, 0.0, 0.0};
  int history_size{100};
  int max_unmatched_frames{0};
  int fix_size_history_threshold{10};
  double fix_size_dimension_threshold{0.4};
  std::vector<double> kalman_filter_param{0.25, 0.01, 0.05, 0.05, 0.04, 0.3, 0.6};
  int kalman_filter_averaging_frames{10};
  int frame_skip{5};
  double dynamic_velocity_threshold{0.2};
  double dynamic_voting_threshold{0.8};
  int frames_force_dynamic{10};
  int frames_force_dynamic_check_range{30};
  int dynamic_consistency_threshold{15};
  bool target_constrain_size{true};
  std::vector<double> target_object_size{0.5, 0.5, 1.5};
  std::vector<double> min_object_size{0.20, 0.20, 0.25};
  std::vector<double> max_object_size{3.0, 3.0, 2.0};
  bool enable_stage_timers{false};
  bool enable_vis_stage{true};
  int executor_threads{4};
  bool enable_sync_context{true};
  double max_depth_lidar_skew_sec{0.8};
  double max_depth_yolo_skew_sec{0.8};
  double max_lidar_yolo_skew_sec{1.0};
  double stale_message_age_sec{2.0};
  double skew_startup_grace_sec{8.0};
  double future_stamp_tolerance_sec{0.2};
};

}  // namespace lvdot_ros2
