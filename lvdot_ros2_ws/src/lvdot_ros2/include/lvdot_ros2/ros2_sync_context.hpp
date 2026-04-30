#pragma once

#include <functional>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lvdot_ros2/lvdot_detector_config.hpp"

namespace lvdot_ros2
{

class ROS2SyncContext
{
public:
  using DepthPoseCallback = std::function<void(
      const sensor_msgs::msg::Image::ConstSharedPtr &,
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &)>;
  using LidarPoseCallback = std::function<void(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &,
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &)>;
  using DepthOdomCallback = std::function<void(
      const sensor_msgs::msg::Image::ConstSharedPtr &,
      const nav_msgs::msg::Odometry::ConstSharedPtr &)>;
  using LidarOdomCallback = std::function<void(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &,
      const nav_msgs::msg::Odometry::ConstSharedPtr &)>;

  ROS2SyncContext(rclcpp::Node & node, const LVdotDetectorConfig & config);

  void register_pose_callbacks(DepthPoseCallback depth_pose_cb, LidarPoseCallback lidar_pose_cb);
  void register_odom_callbacks(DepthOdomCallback depth_odom_cb, LidarOdomCallback lidar_odom_cb);

private:
  using DepthPosePolicy =
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>;
  using LidarPosePolicy =
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped>;
  using DepthOdomPolicy =
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, nav_msgs::msg::Odometry>;
  using LidarOdomPolicy =
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

  rclcpp::Node & node_;
  const LVdotDetectorConfig & config_;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

  std::shared_ptr<message_filters::Synchronizer<DepthPosePolicy>> depth_pose_sync_;
  std::shared_ptr<message_filters::Synchronizer<LidarPosePolicy>> lidar_pose_sync_;
  std::shared_ptr<message_filters::Synchronizer<DepthOdomPolicy>> depth_odom_sync_;
  std::shared_ptr<message_filters::Synchronizer<LidarOdomPolicy>> lidar_odom_sync_;

  DepthPoseCallback depth_pose_cb_;
  LidarPoseCallback lidar_pose_cb_;
  DepthOdomCallback depth_odom_cb_;
  LidarOdomCallback lidar_odom_cb_;

  void handle_depth_pose(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg);
  void handle_lidar_pose(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg);
  void handle_depth_odom(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);
  void handle_lidar_odom(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);
};

}  // namespace lvdot_ros2
