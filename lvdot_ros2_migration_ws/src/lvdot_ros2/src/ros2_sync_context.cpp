#include "lvdot_ros2/ros2_sync_context.hpp"

#include <rmw/qos_profiles.h>

namespace lvdot_ros2
{

ROS2SyncContext::ROS2SyncContext(rclcpp::Node & node, const LVdotDetectorConfig & config)
: node_(node), config_(config)
{
  depth_sub_.subscribe(&node_, config_.depth_image_topic, rmw_qos_profile_sensor_data);
  lidar_sub_.subscribe(&node_, config_.lidar_pointcloud_topic, rmw_qos_profile_sensor_data);

  if (config_.localization_mode == 0) {
    pose_sub_.subscribe(&node_, config_.pose_topic, rmw_qos_profile_default);
  } else {
    odom_sub_.subscribe(&node_, config_.odom_topic, rmw_qos_profile_default);
  }
}

void ROS2SyncContext::register_pose_callbacks(
  DepthPoseCallback depth_pose_cb,
  LidarPoseCallback lidar_pose_cb)
{
  depth_pose_cb_ = std::move(depth_pose_cb);
  lidar_pose_cb_ = std::move(lidar_pose_cb);

  depth_pose_sync_ = std::make_shared<message_filters::Synchronizer<DepthPosePolicy>>(
    DepthPosePolicy(100), depth_sub_, pose_sub_);
  lidar_pose_sync_ = std::make_shared<message_filters::Synchronizer<LidarPosePolicy>>(
    LidarPosePolicy(100), lidar_sub_, pose_sub_);

  depth_pose_sync_->registerCallback(&ROS2SyncContext::handle_depth_pose, this);
  lidar_pose_sync_->registerCallback(&ROS2SyncContext::handle_lidar_pose, this);
}

void ROS2SyncContext::register_odom_callbacks(
  DepthOdomCallback depth_odom_cb,
  LidarOdomCallback lidar_odom_cb)
{
  depth_odom_cb_ = std::move(depth_odom_cb);
  lidar_odom_cb_ = std::move(lidar_odom_cb);

  depth_odom_sync_ = std::make_shared<message_filters::Synchronizer<DepthOdomPolicy>>(
    DepthOdomPolicy(100), depth_sub_, odom_sub_);
  lidar_odom_sync_ = std::make_shared<message_filters::Synchronizer<LidarOdomPolicy>>(
    LidarOdomPolicy(100), lidar_sub_, odom_sub_);

  depth_odom_sync_->registerCallback(&ROS2SyncContext::handle_depth_odom, this);
  lidar_odom_sync_->registerCallback(&ROS2SyncContext::handle_lidar_odom, this);
}

void ROS2SyncContext::handle_depth_pose(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  if (depth_pose_cb_) {
    depth_pose_cb_(depth_msg, pose_msg);
  }
}

void ROS2SyncContext::handle_lidar_pose(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  if (lidar_pose_cb_) {
    lidar_pose_cb_(lidar_msg, pose_msg);
  }
}

void ROS2SyncContext::handle_depth_odom(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  if (depth_odom_cb_) {
    depth_odom_cb_(depth_msg, odom_msg);
  }
}

void ROS2SyncContext::handle_lidar_odom(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  if (lidar_odom_cb_) {
    lidar_odom_cb_(lidar_msg, odom_msg);
  }
}

}  // namespace lvdot_ros2
