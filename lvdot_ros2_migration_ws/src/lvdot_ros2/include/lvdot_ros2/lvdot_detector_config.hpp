#pragma once

#include <string>
#include <vector>
#include <cstddef>

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

  // QC-GAF integration mode (how QC-GAF output feeds tracking):
  //   disabled    – baseline behavior, tracker consumes rule-fusion filtered_bboxes
  //   refinement  – Path Z: QC-GAF refines filtered_bboxes geometry via nearest-neighbor match
  //                 (clusters preserved 1:1 from rule fusion)
  //   replacement – Path X: tracker consumes QC-GAF boxes directly, clusters empty
  // See LV-DOT-Materials/毕设/qcgaf_integration_analysis_20260512.md for design rationale.
  std::string qcgaf_integration_mode{"disabled"};
  double qcgaf_match_distance_threshold{1.0};  // meters; nearest-neighbor radius for refinement mode

  // §3.3 quality-aware adaptive noise.  When enabled, the detector reads the
  // QC-GAF quality vector and passes Hc/Hl to the core tracker, which scales
  // KF Q (process noise) and R (measurement noise) per frame to approximate
  // thesis formulas (27)(28).
  bool qcgaf_noise_adaptation_enabled{false};
  double qcgaf_alpha_q{1.0};
  double qcgaf_alpha_r{1.0};
  std::string qcgaf_quality_topic{"/qcgaf/quality_vector"};

  // Optional GRU-assisted tracking association.  The GRU predictor remains a
  // soft motion prior only; KF/geometry still dominate track association.
  std::string gru_prediction_topic{"/gru_predictor/predicted_positions"};
  bool enable_gru_association_cost{false};
  double gru_association_weight{0.10};
  double gru_prediction_max_age_sec{0.30};
  double gru_prediction_gate_m{1.50};

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
  int u_map_min_bbox_area{25};
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
  double sim_prev_weight{1.0};
  double sim_proped_weight{1.0};
  bool adaptive_similarity_weight{false};
  double similarity_distance_norm{0.5};
  double min_match_similarity{-2.0};
  double tracker_high_score_threshold{0.55};
  double tracker_low_score_threshold{0.10};
  double tracker_new_track_score_threshold{0.25};
  int history_size{100};
  int tracker_tentative_min_hits{3};
  int tracker_tentative_max_unmatched_frames{1};
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
  bool enable_yolo_sync{true};
  std::size_t sync_queue_size{60};
  double sync_slop_sec{0.10};
  double max_depth_lidar_skew_sec{0.8};
  double max_depth_yolo_skew_sec{0.8};
  double max_lidar_yolo_skew_sec{1.0};
  std::size_t depth_branch_history_size{32};
  double depth_branch_offset_sec{0.20};
  bool depth_branch_match_latest_causal{true};
  double max_depth_branch_age_sec{0.65};
  double max_depth_branch_ready_lag_sec{0.80};
  double depth_branch_future_tolerance_sec{0.03};
  bool enable_depth_branch_in_lidar_fusion{true};
  double stale_message_age_sec{2.0};
  double skew_startup_grace_sec{8.0};
  double future_stamp_tolerance_sec{0.2};
};

}  // namespace lvdot_ros2
