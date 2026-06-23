#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <deque>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lvdot_interfaces/msg/input_health.hpp>
#include <lvdot_interfaces/msg/pipeline_stats.hpp>
#include <lvdot_interfaces/msg/stage_timers.hpp>
#include <lvdot_interfaces/srv/get_dynamic_obstacles.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "lvdot_ros2/lvdot_detector_config.hpp"
#include "lvdot_ros2/lvdot_box3d.hpp"
#include "lvdot_ros2/lvdot_runtime_state.hpp"
#include "lvdot_ros2/ros2_sync_context.hpp"

namespace onboardDetector
{
struct FilterLVBBoxesOutput;
}

namespace lvdot_ros2
{
class LVdotDetectorNode : public rclcpp::Node
{
public:
  explicit LVdotDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  int executor_threads() const;

private:
  void create_subscribers();
  void create_publishers();
  void create_services();
  void create_timers();
  void load_config_from_parameters();
  void log_input_health();

  void on_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_color_image(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_lidar_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_yolo_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
  void on_qcgaf_fused_bboxes(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
  void on_qcgaf_quality_vector(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
  void on_gru_predictions(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);

  void on_get_dynamic_obstacles(
    const std::shared_ptr<lvdot_interfaces::srv::GetDynamicObstacles::Request> request,
    std::shared_ptr<lvdot_interfaces::srv::GetDynamicObstacles::Response> response);
  void on_depth_pose_sync(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg);
  void on_lidar_pose_sync(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg);
  void on_depth_odom_sync(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);
  void on_lidar_odom_sync(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);
  void on_depth_yolo_sync(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & yolo_msg);
  void on_lidar_yolo_sync(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & yolo_msg);
  void on_depth_branch_worker_timer();
  void on_detection_timer();
  void on_lidar_detection_timer();
  void on_tracking_timer();
  void on_classification_timer();
  void on_vis_timer();
  void refresh_filtered_cluster_centers(
    const onboardDetector::FilterLVBBoxesOutput & filter_output);
  void update_common_filter_stats(
    const onboardDetector::FilterLVBBoxesOutput & filter_output);
  void record_depth_locked(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void record_lidar_locked(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void record_pose_locked(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);
  void record_odom_locked(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void record_yolo_locked(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);

  LVdotDetectorConfig config_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_detection_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr qcgaf_fused_bboxes_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr qcgaf_quality_vector_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr gru_predictions_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uv_depth_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr u_depth_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uv_bird_view_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_color_img_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uv_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr db_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_bboxes_qcgaf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_bboxes_qcgaf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_bboxes_before_yolo_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_bboxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr history_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vel_vis_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_depth_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_clusters_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_dynamic_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsample_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_lidar_points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr input_health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_timers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pipeline_stats_pub_;
  rclcpp::Publisher<lvdot_interfaces::msg::InputHealth>::SharedPtr input_health_struct_pub_;
  rclcpp::Publisher<lvdot_interfaces::msg::StageTimers>::SharedPtr stage_timers_struct_pub_;
  rclcpp::Publisher<lvdot_interfaces::msg::PipelineStats>::SharedPtr pipeline_stats_struct_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cluster_debug_pub_;

  rclcpp::Service<lvdot_interfaces::srv::GetDynamicObstacles>::SharedPtr get_dynamic_obstacles_srv_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::CallbackGroup::SharedPtr depth_branch_callback_group_;
  rclcpp::CallbackGroup::SharedPtr detection_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_detection_callback_group_;
  rclcpp::CallbackGroup::SharedPtr tracking_callback_group_;
  rclcpp::CallbackGroup::SharedPtr classification_callback_group_;
  rclcpp::CallbackGroup::SharedPtr vis_callback_group_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr depth_branch_timer_;
  rclcpp::TimerBase::SharedPtr detection_timer_;
  rclcpp::TimerBase::SharedPtr lidar_detection_timer_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  rclcpp::TimerBase::SharedPtr classification_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  std::unique_ptr<ROS2SyncContext> sync_context_;

  rclcpp::Time last_depth_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_color_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_yolo_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_branch_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_branch_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_time_{0, 0, RCL_ROS_TIME};

  std::size_t depth_count_{0};
  std::size_t color_count_{0};
  std::size_t lidar_count_{0};
  std::size_t pose_count_{0};
  std::size_t odom_count_{0};
  std::size_t yolo_count_{0};
  std::size_t qcgaf_msg_count_{0};
  std::size_t last_qcgaf_bbox_count_{0};
  rclcpp::Time last_qcgaf_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t qcgaf_quality_msg_count_{0};
  std::size_t gru_prediction_msg_count_{0};
  std::size_t last_gru_prediction_count_{0};
  std::size_t depth_pose_sync_count_{0};
  std::size_t lidar_pose_sync_count_{0};
  std::size_t depth_odom_sync_count_{0};
  std::size_t lidar_odom_sync_count_{0};
  std::size_t depth_yolo_sync_count_{0};
  std::size_t lidar_yolo_sync_count_{0};
  std::size_t detection_tick_count_{0};
  std::size_t lidar_detection_tick_count_{0};
  std::size_t tracking_tick_count_{0};
  std::size_t classification_tick_count_{0};
  std::size_t vis_tick_count_{0};
  std::size_t last_projected_depth_sample_count_{0};
  std::size_t last_filtered_depth_sample_count_{0};
  std::size_t last_u_map_box_count_{0};
  std::size_t last_projected_depth_box_count_{0};
  std::size_t last_u_map_db_merge_count_{0};
  std::size_t last_u_map_enhanced_db_count_{0};
  std::size_t last_u_map_enhanced_visual_count_{0};
  std::size_t last_u_map_enhanced_filtered_before_yolo_count_{0};
  std::size_t last_u_map_enhanced_filtered_count_{0};
  std::size_t last_raw_lidar_sample_count_{0};
  std::size_t last_filtered_lidar_sample_count_{0};
  std::size_t last_visual_bbox_count_{0};
  std::size_t last_db_bbox_count_{0};
  std::size_t last_lidar_bbox_count_{0};
  std::size_t last_filtered_before_yolo_count_{0};
  std::size_t last_filtered_bbox_count_{0};
  std::size_t last_track_count_{0};
  std::size_t last_dynamic_count_{0};
  std::size_t last_split_source_boxes_{0};
  std::size_t last_split_success_boxes_{0};
  std::size_t last_split_output_boxes_{0};
  std::size_t last_fusion_component_count_{0};
  std::size_t last_visual_only_component_count_{0};
  std::size_t last_lidar_only_component_count_{0};
  std::size_t last_yolo_input_count_{0};
  std::size_t last_yolo_candidate_3d_count_{0};
  std::size_t last_yolo_matched_3d_count_{0};
  std::size_t last_yolo_matched_detection_count_{0};
  std::size_t last_yolo_human_marked_count_{0};
  std::size_t last_yolo_fused_used_count_{0};
  std::size_t last_uv_input_count_{0};
  std::size_t last_db_input_count_{0};
  std::size_t last_uv_best_match_count_{0};
  std::size_t last_db_best_match_count_{0};
  std::size_t last_uv_db_mutual_match_count_{0};
  std::size_t last_uv_no_db_candidate_count_{0};
  std::size_t last_uv_not_mutual_count_{0};
  std::size_t last_uv_mutual_iou_reject_count_{0};
  std::size_t last_fixed_size_count_{0};
  std::size_t last_dynamic_rejected_by_size_{0};
  std::size_t last_dynamic_filtered_point_count_{0};
  std::size_t last_raw_dynamic_point_count_{0};
  std::size_t service_call_count_{0};
  std::size_t last_service_response_count_{0};
  std::size_t last_depth_processed_count_{0};
  std::size_t last_lidar_processed_count_{0};
  std::size_t det_depth_lidar_stale_drop_count_{0};
  std::size_t det_depth_lidar_skew_drop_count_{0};
  std::size_t det_depth_yolo_stale_drop_count_{0};
  std::size_t det_depth_yolo_skew_drop_count_{0};
  std::size_t lidar_depth_stale_drop_count_{0};
  std::size_t lidar_depth_skew_drop_count_{0};
  std::size_t lidar_yolo_stale_drop_count_{0};
  std::size_t lidar_yolo_skew_drop_count_{0};
  std::size_t det_waiting_depth_count_{0};
  std::size_t det_waiting_pose_count_{0};
  std::size_t lidar_waiting_new_lidar_count_{0};
  std::size_t fused_zero_with_lidar_count_{0};
  std::size_t depth_branch_match_count_{0};
  std::size_t depth_branch_miss_count_{0};
  std::size_t depth_branch_reject_future_count_{0};
  std::size_t depth_branch_reject_age_count_{0};
  std::size_t depth_branch_reject_ready_lag_count_{0};
  double depth_branch_match_abs_sum_sec_{0.0};
  double depth_branch_match_abs_max_sec_{0.0};
  double depth_branch_match_signed_sum_sec_{0.0};
  double depth_branch_target_abs_sum_sec_{0.0};
  double depth_branch_target_abs_max_sec_{0.0};
  double depth_branch_target_signed_sum_sec_{0.0};
  double depth_branch_ready_lag_sum_sec_{0.0};
  double depth_branch_ready_lag_max_sec_{0.0};
  std::size_t filter_update_seq_{0};
  std::size_t last_tracking_filter_update_seq_{0};
  std::size_t tracking_update_seq_{0};
  std::size_t last_classification_tracking_update_seq_{0};
  std::size_t latest_depth_seq_{0};
  std::size_t last_depth_branch_processed_seq_{0};
  std::string last_detection_phase_{"idle"};
  std::string last_depth_branch_phase_{"idle"};
  std::string last_lidar_detection_phase_{"idle"};
  std::string last_tracking_phase_{"idle"};
  std::string last_classification_phase_{"idle"};

  LVdotRuntimeState runtime_state_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> depth_history_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidar_history_;
  std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> pose_history_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odom_history_;
  std::deque<vision_msgs::msg::Detection2DArray::ConstSharedPtr> yolo_history_;
  std::deque<LVdotRuntimeState::BranchCache> depth_branch_history_;
  mutable std::mutex state_mutex_;

  static constexpr const char * kServiceName = "onboard_detector/get_dynamic_obstacles";
  static constexpr const char * kOutputPrefix = "onboard_detector";
};

}  // namespace lvdot_ros2
