#pragma once

#include <deque>
#include <array>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "lvdot_core/kalman_filter.hpp"
#include "lvdot_ros2/lvdot_box3d.hpp"

namespace lvdot_ros2
{

struct LVdotRuntimeState
{
  struct DepthSample
  {
    geometry_msgs::msg::Point point;
    double depth{0.0};
    int u{0};
    int v{0};
  };

  struct TrackState
  {
    // Real Eigen-based Kalman filter (ported verbatim from lvdot_core).
    onboardDetector::kalman_filter kf;
    bool kf_initialized{false};
    // Exported state vector kept for compatibility with existing consumers.
    // Entries are refreshed from kf.output() after every prediction or update.
    std::array<double, 6> filter_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool filter_initialized{false};
    Box3D current_box;
    geometry_msgs::msg::Point current_center;
    geometry_msgs::msg::Vector3 current_std;
    std::deque<Box3D> box_history;
    std::deque<std::vector<DepthSample>> cluster_history;
    std::deque<geometry_msgs::msg::Point> center_history;
    std::deque<geometry_msgs::msg::Vector3> std_history;
    bool matched_in_frame{false};
    std::size_t age{0};
    std::size_t missed_frames{0};
  };

  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_image;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_image;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_lidar_pointcloud;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom;
  vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_yolo_detections;

  geometry_msgs::msg::Point current_position;
  geometry_msgs::msg::Vector3 current_velocity;
  double current_yaw{0.0};
  bool has_sensor_pose{false};

  std::vector<Box3D> uv_bboxes;
  std::vector<Box3D> db_bboxes;
  std::vector<Box3D> filtered_bboxes_before_yolo;
  std::vector<Box3D> filtered_bboxes;
  std::vector<Box3D> visual_bboxes;
  std::vector<Box3D> lidar_bboxes;
  std::vector<Box3D> tracked_bboxes;
  std::vector<Box3D> dynamic_bboxes;

  std::vector<std::deque<Box3D>> box_history;
  std::vector<DepthSample> projected_depth_samples;
  std::vector<DepthSample> filtered_depth_samples;
  std::vector<DepthSample> raw_lidar_samples;
  std::vector<DepthSample> filtered_lidar_samples;
  std::vector<std::vector<DepthSample>> db_clusters;
  std::vector<geometry_msgs::msg::Point> db_cluster_centers;
  std::vector<std::vector<DepthSample>> visual_clusters;
  std::vector<std::vector<DepthSample>> filtered_clusters_before_yolo;
  std::vector<std::vector<DepthSample>> filtered_clusters;
  std::vector<std::vector<DepthSample>> lidar_clusters;
  std::vector<geometry_msgs::msg::Vector3> db_cluster_stds;
  std::vector<geometry_msgs::msg::Vector3> visual_cluster_stds;
  std::vector<geometry_msgs::msg::Vector3> filtered_cluster_stds_before_yolo;
  std::vector<geometry_msgs::msg::Vector3> filtered_cluster_stds;
  std::vector<geometry_msgs::msg::Point> lidar_cluster_centers;
  std::vector<geometry_msgs::msg::Vector3> lidar_cluster_stds;
  std::vector<geometry_msgs::msg::Point> filtered_cluster_centers;
  std::vector<TrackState> track_states;

  // Hysteresis counter for clearing dynamic_bboxes.  Incremented each
  // classification tick where tracks are empty; reset when tracks are non-empty
  // or dynamic boxes are produced.  dynamic_bboxes is only cleared after this
  // reaches the threshold (3 consecutive empty-track ticks ≈ ~100ms).
  int empty_tracks_consecutive{0};

  // Visualization images produced by the real UVdetector (mirror of
  // dynamicDetector::uvDetector_->depth_show / U_map_show / bird_view).
  cv::Mat uv_depth_show;
  cv::Mat uv_u_map_show;
  cv::Mat uv_bird_view;
};

}  // namespace lvdot_ros2
