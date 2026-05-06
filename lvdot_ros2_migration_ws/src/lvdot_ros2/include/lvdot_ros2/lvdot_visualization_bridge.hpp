#pragma once

#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "lvdot_ros2/lvdot_box3d.hpp"
#include "lvdot_ros2/lvdot_detector_config.hpp"
#include "lvdot_ros2/lvdot_runtime_state.hpp"

namespace lvdot_ros2
{

std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a);

visualization_msgs::msg::MarkerArray make_box_markers(
  const std::vector<Box3D> & boxes,
  const std::string & frame_id,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color,
  const rclcpp::Time & stamp);

visualization_msgs::msg::MarkerArray make_history_markers(
  const std::vector<LVdotRuntimeState::TrackState> & tracks,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

visualization_msgs::msg::MarkerArray make_velocity_markers(
  const std::vector<Box3D> & boxes,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

sensor_msgs::msg::PointCloud2 make_pointcloud2(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

sensor_msgs::msg::PointCloud2 make_uniform_colored_pointcloud2(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  uint8_t r,
  uint8_t g,
  uint8_t b);

sensor_msgs::msg::PointCloud2 make_colored_pointcloud2(
  const std::vector<std::vector<LVdotRuntimeState::DepthSample>> & clusters,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  bool use_unique_colors);

std::vector<geometry_msgs::msg::Point> samples_to_points(
  const std::vector<LVdotRuntimeState::DepthSample> & samples);

std::vector<geometry_msgs::msg::Point> clusters_to_points(
  const std::vector<std::vector<LVdotRuntimeState::DepthSample>> & clusters);

struct DynamicStaticPointSplit
{
  std::vector<geometry_msgs::msg::Point> dynamic_points;
  std::vector<geometry_msgs::msg::Point> static_points;
};

DynamicStaticPointSplit split_points_by_box_flags(
  const std::vector<std::vector<LVdotRuntimeState::DepthSample>> & clusters,
  const std::vector<Box3D> & boxes);

std::vector<geometry_msgs::msg::Point> points_in_boxes(
  const std::vector<LVdotRuntimeState::DepthSample> & samples,
  const std::vector<Box3D> & boxes);

std::vector<geometry_msgs::msg::Point> points_in_boxes(
  const std::vector<geometry_msgs::msg::Point> & source_points,
  const std::vector<Box3D> & boxes);

std::vector<geometry_msgs::msg::Point> points_outside_boxes(
  const std::vector<geometry_msgs::msg::Point> & source_points,
  const std::vector<Box3D> & boxes);

std::optional<sensor_msgs::msg::Image> make_detected_color_image(
  const LVdotRuntimeState & state,
  const std::string & fallback_frame_id);

std::optional<sensor_msgs::msg::Image> make_detected_depth_image(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::string & fallback_frame_id);

std::optional<sensor_msgs::msg::Image> make_detected_u_depth_image(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::string & fallback_frame_id);

std::optional<sensor_msgs::msg::Image> make_bird_view_image(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace lvdot_ros2
