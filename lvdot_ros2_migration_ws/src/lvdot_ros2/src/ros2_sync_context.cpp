#include "lvdot_ros2/ros2_sync_context.hpp"

#include <rmw/qos_profiles.h>

namespace lvdot_ros2
{

ROS2SyncContext::ROS2SyncContext(rclcpp::Node & node, const LVdotDetectorConfig & config)
: node_(node), config_(config)
{
  depth_sub_.subscribe(&node_, config_.depth_image_topic, rmw_qos_profile_sensor_data);
  lidar_sub_.subscribe(&node_, config_.lidar_pointcloud_topic, rmw_qos_profile_sensor_data);
  yolo_sub_.subscribe(&node_, config_.yolo_detection_topic, rmw_qos_profile_sensor_data);

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
    DepthPosePolicy(config_.sync_queue_size), depth_sub_, pose_sub_);
  lidar_pose_sync_ = std::make_shared<message_filters::Synchronizer<LidarPosePolicy>>(
    LidarPosePolicy(config_.sync_queue_size), lidar_sub_, pose_sub_);
  depth_pose_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_sec));
  lidar_pose_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_sec));

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
    DepthOdomPolicy(config_.sync_queue_size), depth_sub_, odom_sub_);
  lidar_odom_sync_ = std::make_shared<message_filters::Synchronizer<LidarOdomPolicy>>(
    LidarOdomPolicy(config_.sync_queue_size), lidar_sub_, odom_sub_);
  depth_odom_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_sec));
  lidar_odom_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_sec));

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

void ROS2SyncContext::register_yolo_callbacks(
  DepthYoloCallback depth_yolo_cb,
  LidarYoloCallback lidar_yolo_cb)
{
  depth_yolo_cb_ = std::move(depth_yolo_cb);
  lidar_yolo_cb_ = std::move(lidar_yolo_cb);

  depth_yolo_sync_ = std::make_shared<message_filters::Synchronizer<DepthYoloPolicy>>(
    DepthYoloPolicy(config_.sync_queue_size), depth_sub_, yolo_sub_);
  lidar_yolo_sync_ = std::make_shared<message_filters::Synchronizer<LidarYoloPolicy>>(
    LidarYoloPolicy(config_.sync_queue_size), lidar_sub_, yolo_sub_);
  depth_yolo_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_sec));
  lidar_yolo_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_sec));

  depth_yolo_sync_->registerCallback(&ROS2SyncContext::handle_depth_yolo, this);
  lidar_yolo_sync_->registerCallback(&ROS2SyncContext::handle_lidar_yolo, this);
}

void ROS2SyncContext::handle_depth_yolo(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & yolo_msg)
{
  if (depth_yolo_cb_) {
    depth_yolo_cb_(depth_msg, yolo_msg);
  }
}

void ROS2SyncContext::handle_lidar_yolo(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & yolo_msg)
{
  if (lidar_yolo_cb_) {
    lidar_yolo_cb_(lidar_msg, yolo_msg);
  }
}

}  // namespace lvdot_ros2
