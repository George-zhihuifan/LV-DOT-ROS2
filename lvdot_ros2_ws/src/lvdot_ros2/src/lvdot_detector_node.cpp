#include "lvdot_ros2/lvdot_detector_node.hpp"

#include <chrono>
#include <algorithm>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>

#include "lvdot_core/dbscan.hpp"
#include "lvdot_core/classification_filter.hpp"
#include "lvdot_core/detection_filter.hpp"
#include "lvdot_core/fusion_filter.hpp"
#include "lvdot_core/kalman_filter.hpp"
#include "lvdot_core/lidar_detector.hpp"
#include "lvdot_core/lidar_preprocess.hpp"
#include "lvdot_core/tracking_filter.hpp"
#include "lvdot_core/uv_detector.hpp"
#include "lvdot_ros2/lvdot_runtime_bridge.hpp"
#include "lvdot_ros2/lvdot_visualization_bridge.hpp"

namespace lvdot_ros2
{

namespace
{

template<typename T>
T resolve_parameter(rclcpp::Node & node, const char * name, const T & default_value)
{
  return node.get_parameter_or<T>(name, default_value);
}

geometry_msgs::msg::Point camera_translation(const std::vector<double> & transform);

double planar_distance(double x0, double y0, const Box3D & box)
{
  return std::hypot(x0 - box.x, y0 - box.y);
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Matrix3d rotation_from_runtime_state(const LVdotRuntimeState & state)
{
  auto from_quat = [](const geometry_msgs::msg::Quaternion & q) {
    Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
    return eq.normalized().toRotationMatrix();
  };

  if (state.latest_odom) {
    return from_quat(state.latest_odom->pose.pose.orientation);
  }
  if (state.latest_pose) {
    return from_quat(state.latest_pose->pose.orientation);
  }

  const double cyaw = std::cos(state.current_yaw);
  const double syaw = std::sin(state.current_yaw);
  Eigen::Matrix3d yaw_only;
  yaw_only <<
    cyaw, -syaw, 0.0,
    syaw,  cyaw, 0.0,
    0.0,   0.0,  1.0;
  return yaw_only;
}

std::string detection_label(const vision_msgs::msg::Detection2D & detection)
{
  if (detection.results.empty()) {
    return "";
  }
  return detection.results.front().hypothesis.class_id;
}

double detection_score(const vision_msgs::msg::Detection2D & detection)
{
  if (detection.results.empty()) {
    return 0.0;
  }
  return detection.results.front().hypothesis.score;
}

bool detection_is_human(const vision_msgs::msg::Detection2D & detection)
{
  const auto label = detection_label(detection);
  return label == "person" || label == "human" || label == "pedestrian";
}

Box3D box_from_detection(
  const vision_msgs::msg::Detection2D & detection,
  std::size_t index,
  const LVdotDetectorConfig & config)
{
  Box3D box;
  const auto width = detection.bbox.size_x;
  const auto height = detection.bbox.size_y;
  const auto center_x = detection.bbox.center.position.x;
  const auto center_y = detection.bbox.center.position.y;
  const auto image_cols = std::max(1.0, static_cast<double>(config.image_cols));
  const auto image_rows = std::max(1.0, static_cast<double>(config.image_rows));

  box.id = static_cast<double>(index);
  box.x = (center_x - config.color_intrinsics[2]) / std::max(1.0, config.color_intrinsics[0]);
  box.y = (center_y - config.color_intrinsics[3]) / std::max(1.0, config.color_intrinsics[1]);
  box.z = 0.0;
  box.x_width = std::max(0.0, width / image_cols);
  box.y_width = std::max(0.0, height / image_rows);
  box.z_width = config.target_object_size.size() >= 3 ? config.target_object_size[2] : 1.5;
  box.is_human = detection_is_human(detection);
  box.is_dynamic_candidate = box.is_human;
  box.is_dynamic = box.is_human;
  box.vx = 0.0;
  box.vy = 0.0;
  box.vz = 0.0;
  box.ax = 0.0;
  box.ay = 0.0;
  box.az = 0.0;
  box.fix_size = false;
  box.is_estimated = false;
  return box;
}

geometry_msgs::msg::Point box_center_point(const Box3D & box)
{
  geometry_msgs::msg::Point point;
  point.x = box.x;
  point.y = box.y;
  point.z = box.z;
  return point;
}

geometry_msgs::msg::Point camera_translation(const std::vector<double> & transform)
{
  geometry_msgs::msg::Point translation;
  if (transform.size() >= 12) {
    translation.x = transform[3];
    translation.y = transform[7];
    translation.z = transform[11];
  }
  return translation;
}

Eigen::Matrix3d transform_rotation(const std::vector<double> & transform)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  if (transform.size() >= 11) {
    rotation <<
      transform[0], transform[1], transform[2],
      transform[4], transform[5], transform[6],
      transform[8], transform[9], transform[10];
  }
  return rotation;
}

std::vector<Eigen::Vector3d> decode_lidar_body_points(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<Eigen::Vector3d> points;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  points.reserve(static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(std::max<uint32_t>(1, cloud.height)));

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }
    // Contract: lidar_pointcloud_topic must provide points in LiDAR-local frame.
    // The LiDAR pipeline will apply body/sensor/world transforms exactly once.
    points.emplace_back(
      static_cast<double>(*iter_x),
      static_cast<double>(*iter_y),
      static_cast<double>(*iter_z));
  }

  return points;
}

std::vector<onboardDetector::ClusterPoint> to_core_cluster(
  const std::vector<LVdotRuntimeState::DepthSample> & cluster)
{
  std::vector<onboardDetector::ClusterPoint> output;
  output.reserve(cluster.size());
  for (const auto & sample : cluster) {
    onboardDetector::ClusterPoint point;
    point.point = Eigen::Vector3d(sample.point.x, sample.point.y, sample.point.z);
    point.depth = sample.depth;
    point.u = sample.u;
    point.v = sample.v;
    point.has_image_point = true;
    output.push_back(point);
  }
  return output;
}

std::vector<LVdotRuntimeState::DepthSample> from_core_cluster(
  const std::vector<onboardDetector::ClusterPoint> & cluster)
{
  std::vector<LVdotRuntimeState::DepthSample> output;
  output.reserve(cluster.size());
  for (const auto & point : cluster) {
    LVdotRuntimeState::DepthSample sample;
    sample.point.x = point.point.x();
    sample.point.y = point.point.y();
    sample.point.z = point.point.z();
    sample.depth = point.depth;
    sample.u = point.u;
    sample.v = point.v;
    output.push_back(sample);
  }
  return output;
}

geometry_msgs::msg::Point eigen_to_point(const Eigen::Vector3d & value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

geometry_msgs::msg::Vector3 eigen_to_vector3(const Eigen::Vector3d & value)
{
  geometry_msgs::msg::Vector3 vector;
  vector.x = value.x();
  vector.y = value.y();
  vector.z = value.z();
  return vector;
}

std::size_t count_u_map_enhanced(const std::vector<Box3D> & boxes)
{
  return static_cast<std::size_t>(std::count_if(
    boxes.begin(), boxes.end(), [](const Box3D & box) {return box.is_u_map_enhanced;}));
}

std::vector<LVdotRuntimeState::DepthSample> from_core_samples(
  const std::vector<onboardDetector::ClusterPoint> & points)
{
  return from_core_cluster(points);
}

struct LidarDetectionResult
{
  std::vector<LVdotRuntimeState::DepthSample> raw_samples;
  std::vector<LVdotRuntimeState::DepthSample> filtered_samples;
  std::vector<Box3D> bboxes;
  std::vector<std::vector<LVdotRuntimeState::DepthSample>> clusters;
  std::vector<geometry_msgs::msg::Point> centers;
  std::vector<geometry_msgs::msg::Vector3> stds;
};

LidarDetectionResult run_lidar_detector(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::function<void(const char *)> & phase_update)
{
  LidarDetectionResult result;
  if (!state.latest_lidar_pointcloud || !state.has_sensor_pose) {
    return result;
  }

  const auto & cloud_msg = *state.latest_lidar_pointcloud;
  const auto lidar_t = camera_translation(config.body_to_lidar);
  const auto lidar_r = transform_rotation(config.body_to_lidar);

  onboardDetector::LidarPreprocessConfig preprocess_config;
  preprocess_config.position = Eigen::Vector3d(
    state.current_position.x,
    state.current_position.y,
    state.current_position.z);
  preprocess_config.orientationBody = rotation_from_runtime_state(state);
  preprocess_config.sensorRotation = lidar_r;
  preprocess_config.sensorTranslation = Eigen::Vector3d(lidar_t.x, lidar_t.y, lidar_t.z);
  preprocess_config.localSensorRange = Eigen::Vector3d(
    config.local_sensor_range[0],
    config.local_sensor_range[1],
    config.local_sensor_range[2]);
  preprocess_config.groundHeight = config.ground_height;
  preprocess_config.roofHeight = config.roof_height;
  preprocess_config.downsampleThreshold = config.downsample_threshold;
  preprocess_config.gaussianDownsampleRate = config.gaussian_downsample_rate;

  phase_update("run_lidar_detector/decode");
  const auto body_points = decode_lidar_body_points(cloud_msg);
  phase_update("run_lidar_detector/project");
  const auto raw_samples = onboardDetector::projectLidarPoints(body_points, preprocess_config);
  phase_update("run_lidar_detector/downsample");
  const auto filtered_samples = onboardDetector::downsampleLidarPoints(raw_samples, preprocess_config);
  result.raw_samples = from_core_samples(raw_samples);
  result.filtered_samples = from_core_samples(filtered_samples);
  if (filtered_samples.empty()) {
    return result;
  }

  phase_update("run_lidar_detector/pcl");
  auto pcl_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->reserve(filtered_samples.size());
  for (const auto & sample : filtered_samples) {
    pcl::PointXYZ point;
    point.x = static_cast<float>(sample.point.x());
    point.y = static_cast<float>(sample.point.y());
    point.z = static_cast<float>(sample.point.z());
    pcl_cloud->push_back(point);
  }

  onboardDetector::lidarDetector detector;
  detector.setParams(config.lidar_dbscan_epsilon, config.lidar_dbscan_min_points);
  detector.getPointcloud(pcl_cloud);
  phase_update("run_lidar_detector/dbscan");
  detector.lidarDBSCAN();
  phase_update("run_lidar_detector/cluster_convert");

  auto & core_bboxes = detector.getBBoxes();
  auto & core_clusters = detector.getClusters();
  result.bboxes.reserve(core_bboxes.size());
  result.clusters.reserve(core_clusters.size());
  result.centers.reserve(core_clusters.size());
  result.stds.reserve(core_clusters.size());

  auto append_cluster_detection =
    [&](const onboardDetector::Cluster & cluster, const geometry_msgs::msg::Point & center_hint) {
      if (config.post_min_cluster_points > 0 &&
          static_cast<int>(cluster.points->size()) < config.post_min_cluster_points)
      {
        return;
      }

      std::vector<double> xs;
      std::vector<double> ys;
      std::vector<double> zs;
      xs.reserve(cluster.points->size());
      ys.reserve(cluster.points->size());
      zs.reserve(cluster.points->size());
      for (const auto & point : cluster.points->points) {
        xs.push_back(static_cast<double>(point.x));
        ys.push_back(static_cast<double>(point.y));
        zs.push_back(static_cast<double>(point.z));
      }

      auto quantile = [](std::vector<double> & values, double q) -> double {
        if (values.empty()) {
          return 0.0;
        }
        q = std::clamp(q, 0.0, 1.0);
        const std::size_t idx = static_cast<std::size_t>(q * static_cast<double>(values.size() - 1));
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx), values.end());
        return values[idx];
      };

      // Keep LiDAR boxes recall-oriented for sparse pedestrian clusters.
      // Using wide quantiles avoids dropping valid fringe points.
      const double x_min = quantile(xs, 0.02);
      const double x_max = quantile(xs, 0.98);
      const double y_min = quantile(ys, 0.02);
      const double y_max = quantile(ys, 0.98);
      const double z_min = std::max(quantile(zs, 0.02), config.ground_height);
      const double z_max = quantile(zs, 0.98);

      Box3D lidar_box;
      lidar_box.x = 0.5 * (x_min + x_max);
      lidar_box.y = 0.5 * (y_min + y_max);
      lidar_box.z = 0.5 * (z_min + z_max);
      lidar_box.x_width = std::max(0.05, x_max - x_min);
      lidar_box.y_width = std::max(0.05, y_max - y_min);
      lidar_box.z_width = std::max(0.05, z_max - z_min);

      // Do not reject small LiDAR clusters by min-size at this stage.
      // Small / sparse pedestrian returns are common in simulation.
      if (lidar_box.x_width > config.max_object_size[0] ||
          lidar_box.y_width > config.max_object_size[1] ||
          lidar_box.z_width > config.max_object_size[2])
      {
        return;
      }

      result.bboxes.push_back(lidar_box);

      std::vector<LVdotRuntimeState::DepthSample> cluster_samples;
      cluster_samples.reserve(cluster.points->size());
      for (const auto & point : cluster.points->points) {
        LVdotRuntimeState::DepthSample sample;
        sample.point.x = point.x;
        sample.point.y = point.y;
        sample.point.z = point.z;
        const double dx = point.x - state.current_position.x;
        const double dy = point.y - state.current_position.y;
        const double dz = point.z - state.current_position.z;
        sample.depth = std::sqrt(dx * dx + dy * dy + dz * dz);
        sample.u = 0;
        sample.v = 0;
        cluster_samples.push_back(sample);
      }
      result.clusters.push_back(std::move(cluster_samples));

      geometry_msgs::msg::Point center = center_hint;
      if (center.x == 0.0 && center.y == 0.0 && center.z == 0.0) {
        center.x = lidar_box.x;
        center.y = lidar_box.y;
        center.z = lidar_box.z;
      }
      result.centers.push_back(center);

      geometry_msgs::msg::Vector3 std;
      std.x = std::sqrt(std::max(0.0f, cluster.eigen_values.x()));
      std.y = std::sqrt(std::max(0.0f, cluster.eigen_values.y()));
      std.z = std::sqrt(std::max(0.0f, cluster.eigen_values.z()));
      result.stds.push_back(std);
    };

  for (std::size_t i = 0; i < core_bboxes.size() && i < core_clusters.size(); ++i) {
    const auto & core_box = core_bboxes[i];
    const auto & cluster = core_clusters[i];

    // Split oversized clusters with a tighter secondary DBSCAN to avoid
    // merging adjacent nearby obstacles into one large box.
    const bool oversized_cluster = (core_box.x_width > 1.2) || (core_box.y_width > 1.2);
    if (oversized_cluster) {
      auto sub_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      sub_cloud->reserve(cluster.points->size());
      for (const auto & p : cluster.points->points) {
        sub_cloud->push_back(p);
      }

      onboardDetector::lidarDetector split_detector;
      const double split_eps = std::max(0.03, config.lidar_dbscan_epsilon * 0.65);
      const int split_min_pts = std::max(6, config.lidar_dbscan_min_points - 2);
      split_detector.setParams(split_eps, split_min_pts);
      split_detector.getPointcloud(sub_cloud);
      split_detector.lidarDBSCAN();

      auto & split_clusters = split_detector.getClusters();
      if (split_clusters.size() >= 2) {
        for (const auto & sub_cluster : split_clusters) {
          geometry_msgs::msg::Point center_hint;
          center_hint.x = static_cast<double>(sub_cluster.centroid.x());
          center_hint.y = static_cast<double>(sub_cluster.centroid.y());
          center_hint.z = static_cast<double>(sub_cluster.centroid.z());
          append_cluster_detection(sub_cluster, center_hint);
        }
        continue;
      }
    }

    geometry_msgs::msg::Point center_hint;
    center_hint.x = static_cast<double>(cluster.centroid.x());
    center_hint.y = static_cast<double>(cluster.centroid.y());
    center_hint.z = static_cast<double>(cluster.centroid.z());
    append_cluster_detection(cluster, center_hint);
  }

  return result;
}


geometry_msgs::msg::Vector3 cluster_std_from_samples(
  const std::vector<LVdotRuntimeState::DepthSample> & samples,
  const geometry_msgs::msg::Point & center)
{
  geometry_msgs::msg::Vector3 stddev;
  if (samples.empty()) {
    return stddev;
  }
  for (const auto & sample : samples) {
    stddev.x += std::pow(sample.point.x - center.x, 2.0);
    stddev.y += std::pow(sample.point.y - center.y, 2.0);
    stddev.z += std::pow(sample.point.z - center.z, 2.0);
  }
  const double denom = static_cast<double>(samples.size());
  stddev.x = std::sqrt(stddev.x / denom);
  stddev.y = std::sqrt(stddev.y / denom);
  stddev.z = std::sqrt(stddev.z / denom);
  return stddev;
}

geometry_msgs::msg::Point cluster_center_from_samples(
  const std::vector<LVdotRuntimeState::DepthSample> & samples,
  const geometry_msgs::msg::Point & fallback)
{
  if (samples.empty()) {
    return fallback;
  }
  geometry_msgs::msg::Point center;
  for (const auto & sample : samples) {
    center.x += sample.point.x;
    center.y += sample.point.y;
    center.z += sample.point.z;
  }
  const double denom = static_cast<double>(samples.size());
  center.x /= denom;
  center.y /= denom;
  center.z /= denom;
  return center;
}

}  // namespace

LVdotDetectorNode::LVdotDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lvdot_detector_node", options)
{
  load_config_from_parameters();
  status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  detection_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lidar_detection_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tracking_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  classification_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  vis_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  create_subscribers();
  create_publishers();
  create_services();
  create_timers();

  const bool enable_sync_context = resolve_parameter<bool>(*this, "enable_sync_context", false);
  if (enable_sync_context) {
    sync_context_ = std::make_unique<ROS2SyncContext>(*this, config_);
    if (config_.localization_mode == 0) {
      sync_context_->register_pose_callbacks(
        std::bind(&LVdotDetectorNode::on_depth_pose_sync, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LVdotDetectorNode::on_lidar_pose_sync, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_context_->register_odom_callbacks(
        std::bind(&LVdotDetectorNode::on_depth_odom_sync, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LVdotDetectorNode::on_lidar_odom_sync, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "LV-DOT ROS2 skeleton started. depth=%s color=%s lidar=%s pose=%s odom=%s yolo=%s",
    config_.depth_image_topic.c_str(),
    config_.color_image_topic.c_str(),
    config_.lidar_pointcloud_topic.c_str(),
    config_.pose_topic.c_str(),
    config_.odom_topic.c_str(),
    config_.yolo_detection_topic.c_str());
  RCLCPP_INFO(
    get_logger(),
    "LiDAR frame contract: topic '%s' must publish LiDAR-local points. "
    "If upstream switches this topic to world-frame points, disable local->world transform in detector preprocessing.",
    config_.lidar_pointcloud_topic.c_str());
}

int LVdotDetectorNode::executor_threads() const
{
  return std::max(1, config_.executor_threads);
}

void LVdotDetectorNode::load_config_from_parameters()
{
  config_.localization_mode = resolve_parameter<int>(*this, "localization_mode", config_.localization_mode);
  config_.depth_image_topic = resolve_parameter<std::string>(*this, "depth_image_topic", config_.depth_image_topic);
  config_.color_image_topic = resolve_parameter<std::string>(*this, "color_image_topic", config_.color_image_topic);
  config_.lidar_pointcloud_topic = resolve_parameter<std::string>(*this, "lidar_pointcloud_topic", config_.lidar_pointcloud_topic);
  config_.pose_topic = resolve_parameter<std::string>(*this, "pose_topic", config_.pose_topic);
  config_.odom_topic = resolve_parameter<std::string>(*this, "odom_topic", config_.odom_topic);
  config_.yolo_detection_topic = resolve_parameter<std::string>(*this, "yolo_detection_topic", config_.yolo_detection_topic);
  config_.fusion_mode = resolve_parameter<std::string>(*this, "fusion_mode", config_.fusion_mode);
  if (config_.fusion_mode != "dual" &&
      config_.fusion_mode != "depth_driven" &&
      config_.fusion_mode != "lidar_driven")
  {
    RCLCPP_WARN(
      get_logger(),
      "Invalid fusion_mode='%s', fallback to 'dual'.",
      config_.fusion_mode.c_str());
    config_.fusion_mode = "dual";
  }

  config_.depth_intrinsics = resolve_parameter<std::vector<double>>(*this, "depth_intrinsics", config_.depth_intrinsics);
  config_.color_intrinsics = resolve_parameter<std::vector<double>>(*this, "color_intrinsics", config_.color_intrinsics);
  config_.depth_scale_factor = resolve_parameter<double>(*this, "depth_scale_factor", config_.depth_scale_factor);
  config_.depth_min_value = resolve_parameter<double>(*this, "depth_min_value", config_.depth_min_value);
  config_.depth_max_value = resolve_parameter<double>(*this, "depth_max_value", config_.depth_max_value);
  config_.u_map_row_downsample = resolve_parameter<int>(*this, "u_map_row_downsample", config_.u_map_row_downsample);
  config_.u_map_col_scale = resolve_parameter<double>(*this, "u_map_col_scale", config_.u_map_col_scale);
  config_.u_map_threshold_point = resolve_parameter<int>(*this, "u_map_threshold_point", config_.u_map_threshold_point);
  config_.u_map_threshold_line = resolve_parameter<int>(*this, "u_map_threshold_line", config_.u_map_threshold_line);
  config_.u_map_min_length_line = resolve_parameter<int>(*this, "u_map_min_length_line", config_.u_map_min_length_line);
  config_.depth_filter_margin = resolve_parameter<int>(*this, "depth_filter_margin", config_.depth_filter_margin);
  config_.depth_skip_pixel = resolve_parameter<int>(*this, "depth_skip_pixel", config_.depth_skip_pixel);
  config_.image_cols = resolve_parameter<int>(*this, "image_cols", config_.image_cols);
  config_.image_rows = resolve_parameter<int>(*this, "image_rows", config_.image_rows);
  config_.body_to_camera_depth = resolve_parameter<std::vector<double>>(*this, "body_to_camera_depth", config_.body_to_camera_depth);
  config_.body_to_camera_color = resolve_parameter<std::vector<double>>(*this, "body_to_camera_color", config_.body_to_camera_color);
  config_.body_to_lidar = resolve_parameter<std::vector<double>>(*this, "body_to_lidar", config_.body_to_lidar);
  config_.time_step = resolve_parameter<double>(*this, "time_step", config_.time_step);
  config_.ground_height = resolve_parameter<double>(*this, "ground_height", config_.ground_height);
  config_.roof_height = resolve_parameter<double>(*this, "roof_height", config_.roof_height);
  config_.voxel_occupied_thresh = resolve_parameter<double>(*this, "voxel_occupied_thresh", config_.voxel_occupied_thresh);
  config_.local_sensor_range = resolve_parameter<std::vector<double>>(*this, "local_sensor_range", config_.local_sensor_range);
  config_.dbscan_min_points_cluster = resolve_parameter<int>(*this, "dbscan_min_points_cluster", config_.dbscan_min_points_cluster);
  config_.dbscan_search_range_epsilon = resolve_parameter<double>(*this, "dbscan_search_range_epsilon", config_.dbscan_search_range_epsilon);
  config_.lidar_dbscan_min_points = resolve_parameter<int>(*this, "lidar_DBSCAN_min_points", config_.lidar_dbscan_min_points);
  config_.lidar_dbscan_epsilon = resolve_parameter<double>(*this, "lidar_DBSCAN_epsilon", config_.lidar_dbscan_epsilon);
  config_.post_min_cluster_points = resolve_parameter<int>(*this, "post_min_cluster_points", config_.post_min_cluster_points);
  config_.downsample_threshold = resolve_parameter<int>(*this, "downsample_threshold", config_.downsample_threshold);
  config_.gaussian_downsample_rate = resolve_parameter<int>(*this, "gaussian_downsample_rate", config_.gaussian_downsample_rate);
  config_.filtering_bbox_iou_threshold = resolve_parameter<double>(*this, "filtering_BBox_IOU_threshold", config_.filtering_bbox_iou_threshold);
  config_.max_match_range = resolve_parameter<double>(*this, "max_match_range", config_.max_match_range);
  config_.max_size_diff_range = resolve_parameter<double>(*this, "max_size_diff_range", config_.max_size_diff_range);
  config_.feature_weight = resolve_parameter<std::vector<double>>(*this, "feature_weight", config_.feature_weight);
  config_.history_size = resolve_parameter<int>(*this, "history_size", config_.history_size);
  config_.max_unmatched_frames = resolve_parameter<int>(*this, "max_unmatched_frames", config_.max_unmatched_frames);
  config_.fix_size_history_threshold = resolve_parameter<int>(*this, "fix_size_history_threshold", config_.fix_size_history_threshold);
  config_.fix_size_dimension_threshold = resolve_parameter<double>(*this, "fix_size_dimension_threshold", config_.fix_size_dimension_threshold);
  config_.kalman_filter_param = resolve_parameter<std::vector<double>>(*this, "kalman_filter_param", config_.kalman_filter_param);
  config_.kalman_filter_averaging_frames = resolve_parameter<int>(*this, "kalman_filter_averaging_frames", config_.kalman_filter_averaging_frames);
  config_.frame_skip = resolve_parameter<int>(*this, "frame_skip", config_.frame_skip);
  config_.dynamic_velocity_threshold = resolve_parameter<double>(*this, "dynamic_velocity_threshold", config_.dynamic_velocity_threshold);
  config_.dynamic_voting_threshold = resolve_parameter<double>(*this, "dynamic_voting_threshold", config_.dynamic_voting_threshold);
  config_.frames_force_dynamic = resolve_parameter<int>(*this, "frames_force_dynamic", config_.frames_force_dynamic);
  config_.frames_force_dynamic_check_range = resolve_parameter<int>(*this, "frames_force_dynamic_check_range", config_.frames_force_dynamic_check_range);
  config_.dynamic_consistency_threshold = resolve_parameter<int>(*this, "dynamic_consistency_threshold", config_.dynamic_consistency_threshold);
  config_.target_constrain_size = resolve_parameter<bool>(*this, "target_constrain_size", config_.target_constrain_size);
  config_.target_object_size = resolve_parameter<std::vector<double>>(*this, "target_object_size", config_.target_object_size);
  config_.min_object_size = resolve_parameter<std::vector<double>>(*this, "min_object_size", config_.min_object_size);
  config_.max_object_size = resolve_parameter<std::vector<double>>(*this, "max_object_size", config_.max_object_size);
  if (config_.min_object_size.size() < 3) {
    config_.min_object_size = {0.20, 0.20, 0.25};
  }
  if (config_.max_object_size.size() < 3) {
    config_.max_object_size = {3.0, 3.0, 2.0};
  }
  config_.enable_stage_timers = resolve_parameter<bool>(*this, "enable_stage_timers", config_.enable_stage_timers);
  config_.enable_vis_stage = resolve_parameter<bool>(*this, "enable_vis_stage", config_.enable_vis_stage);
  config_.executor_threads = std::max(
    1,
    resolve_parameter<int>(*this, "executor_threads", config_.executor_threads));

  RCLCPP_INFO(
    get_logger(),
    "Config loaded: localization_mode=%d dt=%.3f image=%dx%d depth_range=[%.2f, %.2f] fusion_mode=%s stage_timers=%s vis_stage=%s executor_threads=%d",
    config_.localization_mode,
    config_.time_step,
    config_.image_cols,
    config_.image_rows,
    config_.depth_min_value,
    config_.depth_max_value,
    config_.fusion_mode.c_str(),
    config_.enable_stage_timers ? "true" : "false",
    config_.enable_vis_stage ? "true" : "false",
    config_.executor_threads);
}

void LVdotDetectorNode::create_subscribers()
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10));

  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    config_.depth_image_topic, sensor_qos,
    std::bind(&LVdotDetectorNode::on_depth_image, this, std::placeholders::_1));
  color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    config_.color_image_topic, sensor_qos,
    std::bind(&LVdotDetectorNode::on_color_image, this, std::placeholders::_1));
  lidar_pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.lidar_pointcloud_topic, sensor_qos,
    std::bind(&LVdotDetectorNode::on_lidar_pointcloud, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.pose_topic, state_qos,
    std::bind(&LVdotDetectorNode::on_pose, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic, state_qos,
    std::bind(&LVdotDetectorNode::on_odom, this, std::placeholders::_1));
  yolo_detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    config_.yolo_detection_topic, sensor_qos,
    std::bind(&LVdotDetectorNode::on_yolo_detections, this, std::placeholders::_1));
}

void LVdotDetectorNode::create_publishers()
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(10));

  uv_depth_map_pub_ = create_publisher<sensor_msgs::msg::Image>("onboard_detector/detected_depth_map", sensor_qos);
  u_depth_map_pub_ = create_publisher<sensor_msgs::msg::Image>("onboard_detector/detected_u_depth_map", sensor_qos);
  uv_bird_view_pub_ = create_publisher<sensor_msgs::msg::Image>("onboard_detector/u_depth_bird_view", sensor_qos);
  detected_color_img_pub_ = create_publisher<sensor_msgs::msg::Image>("onboard_detector/detected_color_image", sensor_qos);

  uv_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/uv_bboxes", marker_qos);
  db_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/dbscan_bboxes", marker_qos);
  visual_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/visual_bboxes", marker_qos);
  lidar_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/lidar_bboxes", marker_qos);
  filtered_bboxes_before_yolo_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "onboard_detector/filtered_before_yolo_bboxes", marker_qos);
  filtered_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/filtered_bboxes", marker_qos);
  tracked_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/tracked_bboxes", marker_qos);
  dynamic_bboxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/dynamic_bboxes", marker_qos);
  history_traj_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/history_trajectories", marker_qos);
  vel_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("onboard_detector/velocity_visualizaton", marker_qos);

  filtered_depth_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/filtered_depth_cloud", sensor_qos);
  lidar_clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/lidar_clusters", sensor_qos);
  filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/filtered_point_cloud", sensor_qos);
  dynamic_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/dynamic_point_cloud", sensor_qos);
  raw_dynamic_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/raw_dynamic_point_cloud", sensor_qos);
  downsample_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/downsampled_point_cloud", sensor_qos);
  raw_lidar_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("onboard_detector/raw_lidar_point_cloud", sensor_qos);
  input_health_pub_ = create_publisher<std_msgs::msg::String>("onboard_detector/input_health", rclcpp::QoS(rclcpp::KeepLast(10)));
  stage_timers_pub_ = create_publisher<std_msgs::msg::String>("onboard_detector/stage_timers", rclcpp::QoS(rclcpp::KeepLast(10)));
  pipeline_stats_pub_ = create_publisher<std_msgs::msg::String>("onboard_detector/pipeline_stats", rclcpp::QoS(rclcpp::KeepLast(10)));
  input_health_struct_pub_ = create_publisher<lvdot_interfaces::msg::InputHealth>(
    "onboard_detector/input_health_status", rclcpp::QoS(rclcpp::KeepLast(10)));
  stage_timers_struct_pub_ = create_publisher<lvdot_interfaces::msg::StageTimers>(
    "onboard_detector/stage_timers_status", rclcpp::QoS(rclcpp::KeepLast(10)));
  pipeline_stats_struct_pub_ = create_publisher<lvdot_interfaces::msg::PipelineStats>(
    "onboard_detector/pipeline_stats_status", rclcpp::QoS(rclcpp::KeepLast(10)));
  cluster_debug_pub_ = create_publisher<std_msgs::msg::String>(
    "onboard_detector/cluster_debug_status", rclcpp::QoS(rclcpp::KeepLast(10)));
}

void LVdotDetectorNode::create_services()
{
  get_dynamic_obstacles_srv_ = create_service<lvdot_interfaces::srv::GetDynamicObstacles>(
    kServiceName,
    std::bind(
      &LVdotDetectorNode::on_get_dynamic_obstacles,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void LVdotDetectorNode::create_timers()
{
  using namespace std::chrono_literals;
  health_timer_ = create_wall_timer(
    2s,
    std::bind(&LVdotDetectorNode::log_input_health, this),
    status_callback_group_);

  if (!config_.enable_stage_timers) {
    return;
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(config_.time_step));

  detection_timer_ = create_wall_timer(
    period,
    std::bind(&LVdotDetectorNode::on_detection_timer, this),
    detection_callback_group_);
  lidar_detection_timer_ = create_wall_timer(
    period,
    std::bind(&LVdotDetectorNode::on_lidar_detection_timer, this),
    lidar_detection_callback_group_);
  tracking_timer_ = create_wall_timer(
    period,
    std::bind(&LVdotDetectorNode::on_tracking_timer, this),
    tracking_callback_group_);
  classification_timer_ = create_wall_timer(
    period,
    std::bind(&LVdotDetectorNode::on_classification_timer, this),
    classification_callback_group_);
  if (config_.enable_vis_stage) {
    vis_timer_ = create_wall_timer(
      period,
      std::bind(&LVdotDetectorNode::on_vis_timer, this),
      vis_callback_group_);
  }
}

void LVdotDetectorNode::log_input_health()
{
  if (!rclcpp::ok(this->get_node_base_interface()->get_context())) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  std_msgs::msg::String input_health_msg;
  std::ostringstream input_health;
  input_health
    << "depth=" << depth_count_
    << " color=" << color_count_
    << " lidar=" << lidar_count_
    << " pose=" << pose_count_
    << " odom=" << odom_count_
    << " yolo=" << yolo_count_
    << " depth_pose_sync=" << depth_pose_sync_count_
    << " lidar_pose_sync=" << lidar_pose_sync_count_
    << " depth_odom_sync=" << depth_odom_sync_count_
    << " lidar_odom_sync=" << lidar_odom_sync_count_;
  input_health_msg.data = input_health.str();
  input_health_pub_->publish(input_health_msg);
  lvdot_interfaces::msg::InputHealth input_health_struct;
  input_health_struct.depth_count = depth_count_;
  input_health_struct.color_count = color_count_;
  input_health_struct.lidar_count = lidar_count_;
  input_health_struct.pose_count = pose_count_;
  input_health_struct.odom_count = odom_count_;
  input_health_struct.yolo_count = yolo_count_;
  input_health_struct.depth_pose_sync_count = depth_pose_sync_count_;
  input_health_struct.lidar_pose_sync_count = lidar_pose_sync_count_;
  input_health_struct.depth_odom_sync_count = depth_odom_sync_count_;
  input_health_struct.lidar_odom_sync_count = lidar_odom_sync_count_;
  input_health_struct_pub_->publish(input_health_struct);
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Input health: depth=%zu color=%zu lidar=%zu pose=%zu odom=%zu yolo=%zu depth_pose_sync=%zu lidar_pose_sync=%zu depth_odom_sync=%zu lidar_odom_sync=%zu",
    depth_count_,
    color_count_,
    lidar_count_,
    pose_count_,
    odom_count_,
    yolo_count_,
    depth_pose_sync_count_,
    lidar_pose_sync_count_,
    depth_odom_sync_count_,
    lidar_odom_sync_count_);

  std_msgs::msg::String stage_timers_msg;
  std::ostringstream stage_timers;
  stage_timers
    << "detection=" << detection_tick_count_
    << " lidar_detection=" << lidar_detection_tick_count_
    << " tracking=" << tracking_tick_count_
    << " classification=" << classification_tick_count_
    << " vis=" << vis_tick_count_
    << " det_phase=" << last_detection_phase_
    << " lidar_phase=" << last_lidar_detection_phase_
    << " track_phase=" << last_tracking_phase_
    << " cls_phase=" << last_classification_phase_;
  stage_timers_msg.data = stage_timers.str();
  stage_timers_pub_->publish(stage_timers_msg);
  lvdot_interfaces::msg::StageTimers stage_timers_struct;
  stage_timers_struct.detection_tick_count = detection_tick_count_;
  stage_timers_struct.lidar_detection_tick_count = lidar_detection_tick_count_;
  stage_timers_struct.tracking_tick_count = tracking_tick_count_;
  stage_timers_struct.classification_tick_count = classification_tick_count_;
  stage_timers_struct.vis_tick_count = vis_tick_count_;
  stage_timers_struct_pub_->publish(stage_timers_struct);
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Stage timers: detection=%zu lidar_detection=%zu tracking=%zu classification=%zu vis=%zu",
    detection_tick_count_,
    lidar_detection_tick_count_,
    tracking_tick_count_,
    classification_tick_count_,
    vis_tick_count_);
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Processing phases: detection=%s lidar_detection=%s tracking=%s classification=%s",
    last_detection_phase_.c_str(),
    last_lidar_detection_phase_.c_str(),
    last_tracking_phase_.c_str(),
    last_classification_phase_.c_str());
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Pipeline stats: depth_samples=%zu/%zu u_map=%zu depth_boxes=%zu u_map_merge=%zu u_map_db=%zu u_map_visual=%zu u_map_fused=%zu u_map_filtered=%zu lidar_samples=%zu/%zu uv=%zu db=%zu lidar=%zu fused=%zu filtered=%zu tracks=%zu dynamic=%zu dyn_points=%zu raw_dyn_points=%zu service=%zu/%zu split=%zu->%zu (%zu outputs) fusion_components=%zu visual_only=%zu lidar_only=%zu yolo_in=%zu yolo_match3d=%zu yolo_match_det=%zu yolo_human=%zu uv_in=%zu db_in=%zu uv_best=%zu db_best=%zu mutual=%zu uv_no_db=%zu uv_not_mutual=%zu uv_iou_reject=%zu fixed_size=%zu size_reject=%zu",
    last_filtered_depth_sample_count_,
    last_projected_depth_sample_count_,
    last_u_map_box_count_,
    last_projected_depth_box_count_,
    last_u_map_db_merge_count_,
    last_u_map_enhanced_db_count_,
    last_u_map_enhanced_visual_count_,
    last_u_map_enhanced_filtered_before_yolo_count_,
    last_u_map_enhanced_filtered_count_,
    last_filtered_lidar_sample_count_,
    last_raw_lidar_sample_count_,
    last_visual_bbox_count_,
    last_db_bbox_count_,
    last_lidar_bbox_count_,
    last_filtered_before_yolo_count_,
    last_filtered_bbox_count_,
    last_track_count_,
    last_dynamic_count_,
    last_dynamic_filtered_point_count_,
    last_raw_dynamic_point_count_,
    service_call_count_,
    last_service_response_count_,
    last_split_source_boxes_,
    last_split_success_boxes_,
    last_split_output_boxes_,
    last_fusion_component_count_,
    last_visual_only_component_count_,
    last_lidar_only_component_count_,
    last_yolo_input_count_,
    last_yolo_matched_3d_count_,
    last_yolo_matched_detection_count_,
    last_yolo_human_marked_count_,
    last_uv_input_count_,
    last_db_input_count_,
    last_uv_best_match_count_,
    last_db_best_match_count_,
    last_uv_db_mutual_match_count_,
    last_uv_no_db_candidate_count_,
    last_uv_not_mutual_count_,
    last_uv_mutual_iou_reject_count_,
    last_fixed_size_count_,
    last_dynamic_rejected_by_size_);

  std_msgs::msg::String stats_msg;
  std::ostringstream stats;
  stats
    << "depth_samples=" << last_filtered_depth_sample_count_ << "/" << last_projected_depth_sample_count_
    << " u_map=" << last_u_map_box_count_
    << " depth_boxes=" << last_projected_depth_box_count_
    << " u_map_merge=" << last_u_map_db_merge_count_
    << " u_map_db=" << last_u_map_enhanced_db_count_
    << " u_map_visual=" << last_u_map_enhanced_visual_count_
    << " u_map_fused=" << last_u_map_enhanced_filtered_before_yolo_count_
    << " u_map_filtered=" << last_u_map_enhanced_filtered_count_
    << " lidar_samples=" << last_filtered_lidar_sample_count_ << "/" << last_raw_lidar_sample_count_
    << " uv=" << last_visual_bbox_count_
    << " db=" << last_db_bbox_count_
    << " lidar=" << last_lidar_bbox_count_
    << " fused=" << last_filtered_before_yolo_count_
    << " filtered=" << last_filtered_bbox_count_
    << " tracks=" << last_track_count_
    << " dynamic=" << last_dynamic_count_
    << " dyn_points=" << last_dynamic_filtered_point_count_
    << " raw_dyn_points=" << last_raw_dynamic_point_count_
    << " service=" << service_call_count_ << "/" << last_service_response_count_
    << " split=" << last_split_source_boxes_ << "->" << last_split_success_boxes_
    << " outputs=" << last_split_output_boxes_
    << " fusion_components=" << last_fusion_component_count_
    << " visual_only=" << last_visual_only_component_count_
    << " lidar_only=" << last_lidar_only_component_count_
    << " yolo_in=" << last_yolo_input_count_
    << " yolo_match3d=" << last_yolo_matched_3d_count_
    << " yolo_match_det=" << last_yolo_matched_detection_count_
    << " yolo_human=" << last_yolo_human_marked_count_
    << " uv_in=" << last_uv_input_count_
    << " db_in=" << last_db_input_count_
    << " uv_best=" << last_uv_best_match_count_
    << " db_best=" << last_db_best_match_count_
    << " mutual=" << last_uv_db_mutual_match_count_
    << " uv_no_db=" << last_uv_no_db_candidate_count_
    << " uv_not_mutual=" << last_uv_not_mutual_count_
    << " uv_iou_reject=" << last_uv_mutual_iou_reject_count_
    << " fixed_size=" << last_fixed_size_count_
    << " size_reject=" << last_dynamic_rejected_by_size_;
  stats_msg.data = stats.str();
  pipeline_stats_pub_->publish(stats_msg);
  lvdot_interfaces::msg::PipelineStats stats_struct;
  stats_struct.projected_depth_sample_count = last_projected_depth_sample_count_;
  stats_struct.filtered_depth_sample_count = last_filtered_depth_sample_count_;
  stats_struct.u_map_box_count = last_u_map_box_count_;
  stats_struct.projected_depth_box_count = last_projected_depth_box_count_;
  stats_struct.u_map_db_merge_count = last_u_map_db_merge_count_;
  stats_struct.u_map_enhanced_db_count = last_u_map_enhanced_db_count_;
  stats_struct.u_map_enhanced_visual_count = last_u_map_enhanced_visual_count_;
  stats_struct.u_map_enhanced_filtered_before_yolo_count = last_u_map_enhanced_filtered_before_yolo_count_;
  stats_struct.u_map_enhanced_filtered_count = last_u_map_enhanced_filtered_count_;
  stats_struct.raw_lidar_sample_count = last_raw_lidar_sample_count_;
  stats_struct.filtered_lidar_sample_count = last_filtered_lidar_sample_count_;
  stats_struct.visual_bbox_count = last_visual_bbox_count_;
  stats_struct.db_bbox_count = last_db_bbox_count_;
  stats_struct.lidar_bbox_count = last_lidar_bbox_count_;
  stats_struct.filtered_before_yolo_count = last_filtered_before_yolo_count_;
  stats_struct.filtered_bbox_count = last_filtered_bbox_count_;
  stats_struct.track_count = last_track_count_;
  stats_struct.dynamic_count = last_dynamic_count_;
  stats_struct.split_source_boxes = last_split_source_boxes_;
  stats_struct.split_success_boxes = last_split_success_boxes_;
  stats_struct.split_output_boxes = last_split_output_boxes_;
  stats_struct.fusion_component_count = last_fusion_component_count_;
  stats_struct.visual_only_component_count = last_visual_only_component_count_;
  stats_struct.lidar_only_component_count = last_lidar_only_component_count_;
  stats_struct.fixed_size_count = last_fixed_size_count_;
  stats_struct.dynamic_rejected_by_size = last_dynamic_rejected_by_size_;
  stats_struct.dynamic_filtered_point_count = last_dynamic_filtered_point_count_;
  stats_struct.raw_dynamic_point_count = last_raw_dynamic_point_count_;
  stats_struct.service_call_count = service_call_count_;
  stats_struct.service_response_count = last_service_response_count_;
  pipeline_stats_struct_pub_->publish(stats_struct);
}

void LVdotDetectorNode::on_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++depth_count_;
  last_depth_stamp_ = msg->header.stamp;
  runtime_state_.latest_depth_image = msg;
}

void LVdotDetectorNode::on_color_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++color_count_;
  last_color_stamp_ = msg->header.stamp;
  runtime_state_.latest_color_image = msg;
}

void LVdotDetectorNode::on_lidar_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++lidar_count_;
  last_lidar_stamp_ = msg->header.stamp;
  runtime_state_.latest_lidar_pointcloud = msg;
}

void LVdotDetectorNode::on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++pose_count_;
  last_pose_stamp_ = msg->header.stamp;
  runtime_state_.latest_pose = msg;
  runtime_state_.current_position = msg->pose.position;
  runtime_state_.current_velocity.x = 0.0;
  runtime_state_.current_velocity.y = 0.0;
  runtime_state_.current_velocity.z = 0.0;
  runtime_state_.current_yaw = yaw_from_quaternion(msg->pose.orientation);
  runtime_state_.has_sensor_pose = true;
}

void LVdotDetectorNode::on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++odom_count_;
  last_odom_stamp_ = msg->header.stamp;
  runtime_state_.latest_odom = msg;
  runtime_state_.current_position = msg->pose.pose.position;
  runtime_state_.current_velocity = msg->twist.twist.linear;
  runtime_state_.current_yaw = yaw_from_quaternion(msg->pose.pose.orientation);
  runtime_state_.has_sensor_pose = true;
}

void LVdotDetectorNode::on_yolo_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++yolo_count_;
  last_yolo_stamp_ = msg->header.stamp;
  runtime_state_.latest_yolo_detections = msg;
}

void LVdotDetectorNode::on_get_dynamic_obstacles(
  const std::shared_ptr<lvdot_interfaces::srv::GetDynamicObstacles::Request> request,
  std::shared_ptr<lvdot_interfaces::srv::GetDynamicObstacles::Response> response)
{
  std::vector<Box3D> dynamic_bboxes;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++service_call_count_;
    dynamic_bboxes = runtime_state_.dynamic_bboxes;
  }
  response->position.clear();
  response->velocity.clear();
  response->size.clear();

  struct Candidate
  {
    double distance;
    const Box3D * box;
  };

  std::vector<Candidate> candidates;
  candidates.reserve(dynamic_bboxes.size());

  for (const auto & box : dynamic_bboxes) {
    const double planar_distance = ::lvdot_ros2::planar_distance(
      request->current_position.x, request->current_position.y, box);
    if (planar_distance <= request->range) {
      candidates.push_back(Candidate{planar_distance, &box});
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.distance < rhs.distance;
    });

  for (const auto & candidate : candidates) {
    geometry_msgs::msg::Vector3 position;
    geometry_msgs::msg::Vector3 velocity;
    geometry_msgs::msg::Vector3 size;

    const auto & box = *candidate.box;
    position.x = box.x;
    position.y = box.y;
    position.z = box.z;

    velocity.x = box.vx;
    velocity.y = box.vy;
    velocity.z = 0.0;

    size.x = box.x_width;
    size.y = box.y_width;
    size.z = box.z_width;

    response->position.push_back(position);
    response->velocity.push_back(velocity);
    response->size.push_back(size);
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_service_response_count_ = response->position.size();
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    3000,
    "get_dynamic_obstacles responded with %zu obstacles from %zu tracked dynamic boxes.",
    response->position.size(),
    dynamic_bboxes.size());
}

void LVdotDetectorNode::on_depth_pose_sync(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++depth_pose_sync_count_;
  last_depth_stamp_ = depth_msg->header.stamp;
  last_pose_stamp_ = pose_msg->header.stamp;
  runtime_state_.latest_depth_image = depth_msg;
  runtime_state_.latest_pose = pose_msg;
  runtime_state_.current_position = pose_msg->pose.position;
  runtime_state_.current_velocity.x = 0.0;
  runtime_state_.current_velocity.y = 0.0;
  runtime_state_.current_velocity.z = 0.0;
  runtime_state_.current_yaw = yaw_from_quaternion(pose_msg->pose.orientation);
  runtime_state_.has_sensor_pose = true;
}

void LVdotDetectorNode::on_lidar_pose_sync(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++lidar_pose_sync_count_;
  last_lidar_stamp_ = lidar_msg->header.stamp;
  last_pose_stamp_ = pose_msg->header.stamp;
  runtime_state_.latest_lidar_pointcloud = lidar_msg;
  runtime_state_.latest_pose = pose_msg;
  runtime_state_.current_position = pose_msg->pose.position;
  runtime_state_.current_velocity.x = 0.0;
  runtime_state_.current_velocity.y = 0.0;
  runtime_state_.current_velocity.z = 0.0;
  runtime_state_.current_yaw = yaw_from_quaternion(pose_msg->pose.orientation);
  runtime_state_.has_sensor_pose = true;
}

void LVdotDetectorNode::on_depth_odom_sync(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++depth_odom_sync_count_;
  last_depth_stamp_ = depth_msg->header.stamp;
  last_odom_stamp_ = odom_msg->header.stamp;
  runtime_state_.latest_depth_image = depth_msg;
  runtime_state_.latest_odom = odom_msg;
  runtime_state_.current_position = odom_msg->pose.pose.position;
  runtime_state_.current_velocity = odom_msg->twist.twist.linear;
  runtime_state_.current_yaw = yaw_from_quaternion(odom_msg->pose.pose.orientation);
  runtime_state_.has_sensor_pose = true;
}

void LVdotDetectorNode::on_lidar_odom_sync(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++lidar_odom_sync_count_;
  last_lidar_stamp_ = lidar_msg->header.stamp;
  last_odom_stamp_ = odom_msg->header.stamp;
  runtime_state_.latest_lidar_pointcloud = lidar_msg;
  runtime_state_.latest_odom = odom_msg;
  runtime_state_.current_position = odom_msg->pose.pose.position;
  runtime_state_.current_velocity = odom_msg->twist.twist.linear;
  runtime_state_.current_yaw = yaw_from_quaternion(odom_msg->pose.pose.orientation);
  runtime_state_.has_sensor_pose = true;
}

void LVdotDetectorNode::refresh_filtered_cluster_centers(
  const onboardDetector::FilterLVBBoxesOutput & filter_output)
{
  runtime_state_.filtered_cluster_centers.clear();
  runtime_state_.filtered_cluster_centers.reserve(runtime_state_.filtered_bboxes.size());
  for (std::size_t i = 0; i < runtime_state_.filtered_bboxes.size(); ++i) {
    auto & box = runtime_state_.filtered_bboxes[i];
    geometry_msgs::msg::Point center;
    if (i < filter_output.filteredPcClusterCenters.size()) {
      center = eigen_to_point(filter_output.filteredPcClusterCenters[i]);
    } else if (i < runtime_state_.filtered_clusters.size()) {
      center = cluster_center_from_samples(runtime_state_.filtered_clusters[i], box_center_point(box));
    } else {
      center = box_center_point(box);
    }
    box.x = center.x;
    box.y = center.y;
    box.z = center.z;
    runtime_state_.filtered_cluster_centers.push_back(center);
  }
}

void LVdotDetectorNode::update_common_filter_stats(
  const onboardDetector::FilterLVBBoxesOutput & filter_output)
{
  last_visual_bbox_count_ = runtime_state_.visual_bboxes.size();
  last_fusion_component_count_ = filter_output.stats.fusion_component_count;
  last_visual_only_component_count_ = filter_output.stats.visual_only_component_count;
  last_lidar_only_component_count_ = filter_output.stats.lidar_only_component_count;
  last_yolo_input_count_ = filter_output.stats.yolo_input_count;
  last_yolo_matched_3d_count_ = filter_output.stats.yolo_matched_3d_count;
  last_yolo_matched_detection_count_ = filter_output.stats.yolo_matched_detection_count;
  last_yolo_human_marked_count_ = filter_output.stats.yolo_human_marked_count;
  last_uv_input_count_ = filter_output.stats.uv_input_count;
  last_db_input_count_ = filter_output.stats.db_input_count;
  last_uv_best_match_count_ = filter_output.stats.uv_best_match_count;
  last_db_best_match_count_ = filter_output.stats.db_best_match_count;
  last_uv_db_mutual_match_count_ = filter_output.stats.uv_db_mutual_match_count;
  last_uv_no_db_candidate_count_ = filter_output.stats.uv_no_db_candidate_count;
  last_uv_not_mutual_count_ = filter_output.stats.uv_not_mutual_count;
  last_uv_mutual_iou_reject_count_ = filter_output.stats.uv_mutual_iou_reject_count;
  last_filtered_before_yolo_count_ = runtime_state_.filtered_bboxes_before_yolo.size();
  last_split_source_boxes_ = filter_output.stats.split_source_boxes;
  last_split_success_boxes_ = filter_output.stats.split_success_boxes;
  last_split_output_boxes_ = filter_output.stats.split_output_boxes;
  last_u_map_enhanced_visual_count_ = count_u_map_enhanced(runtime_state_.visual_bboxes);
  last_u_map_enhanced_filtered_before_yolo_count_ =
    count_u_map_enhanced(runtime_state_.filtered_bboxes_before_yolo);
  last_u_map_enhanced_filtered_count_ = count_u_map_enhanced(runtime_state_.filtered_bboxes);
  last_filtered_bbox_count_ = runtime_state_.filtered_bboxes.size();
  ++filter_update_seq_;
}

void LVdotDetectorNode::on_detection_timer()
{
  if (config_.fusion_mode == "lidar_driven") {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++detection_tick_count_;
    last_detection_phase_ = "disabled_by_fusion_mode";
    return;
  }

  LVdotRuntimeState snapshot;
  constexpr double kMaxDepthYoloSkewSec = 0.30;
  constexpr double kMaxDepthLidarSkewSec = 0.20;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++detection_tick_count_;
    last_detection_phase_ = "snapshot";
    snapshot = runtime_state_;
  }

  if (!snapshot.has_sensor_pose) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "waiting_sensor_pose";
    return;
  }

  if (!snapshot.latest_depth_image) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "waiting_depth";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "build_input";
  }
  const auto detection_input = build_detection_input(snapshot, config_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "run_core";
  }
  const auto detection_output = onboardDetector::runDetection(detection_input);

  LVdotRuntimeState detection_snapshot = snapshot;
  apply_detection_output(detection_output, detection_snapshot);

  // If LiDAR is too old/new relative to this depth tick, avoid mixing stale
  // lidar branch into current depth-driven fusion.
  if (detection_snapshot.latest_depth_image && detection_snapshot.latest_lidar_pointcloud) {
    const rclcpp::Time depth_stamp(detection_snapshot.latest_depth_image->header.stamp);
    const rclcpp::Time lidar_stamp(detection_snapshot.latest_lidar_pointcloud->header.stamp);
    const double skew_sec = std::abs((depth_stamp - lidar_stamp).seconds());
    if (skew_sec > kMaxDepthLidarSkewSec) {
      detection_snapshot.lidar_bboxes.clear();
      detection_snapshot.lidar_clusters.clear();
      detection_snapshot.lidar_cluster_centers.clear();
      detection_snapshot.lidar_cluster_stds.clear();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Depth/LiDAR skew %.3fs > %.3fs. Ignore stale LiDAR branch for this depth fusion tick.",
        skew_sec,
        kMaxDepthLidarSkewSec);
    }
  }

  // Drop stale YOLO detections for this depth tick.
  if (detection_snapshot.latest_depth_image && detection_snapshot.latest_yolo_detections) {
    const rclcpp::Time depth_stamp(detection_snapshot.latest_depth_image->header.stamp);
    const rclcpp::Time yolo_stamp(detection_snapshot.latest_yolo_detections->header.stamp);
    const double skew_sec = std::abs((depth_stamp - yolo_stamp).seconds());
    if (skew_sec > kMaxDepthYoloSkewSec) {
      detection_snapshot.latest_yolo_detections.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Depth/YOLO skew %.3fs > %.3fs. Ignore stale YOLO for this depth fusion tick.",
        skew_sec,
        kMaxDepthYoloSkewSec);
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "build_filter_input";
  }
  const auto filter_input = build_filter_input(detection_snapshot, config_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "run_filter";
  }
  const auto filter_output = onboardDetector::filterLVBBoxes(filter_input);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_detection_phase_ = "apply_output";
    apply_detection_output(detection_output, runtime_state_);
    apply_filter_output(filter_output, runtime_state_);
    refresh_filtered_cluster_centers(filter_output);

    last_projected_depth_sample_count_ = runtime_state_.projected_depth_samples.size();
    last_filtered_depth_sample_count_ = runtime_state_.filtered_depth_samples.size();
    last_u_map_box_count_ = runtime_state_.uv_bboxes.size();
    // "depth_boxes" in pipeline stats should reflect UV 3D boxes extracted from depth.
    // It was previously hardcoded to 0, causing false zero-output diagnostics.
    last_projected_depth_box_count_ = runtime_state_.uv_bboxes.size();
    last_u_map_db_merge_count_ = 0;
    last_db_bbox_count_ = runtime_state_.db_bboxes.size();
    last_u_map_enhanced_db_count_ = count_u_map_enhanced(runtime_state_.db_bboxes);
    update_common_filter_stats(filter_output);
    last_detection_phase_ = "idle";
  }
}

void LVdotDetectorNode::on_lidar_detection_timer()
{
  if (config_.fusion_mode == "depth_driven") {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++lidar_detection_tick_count_;
    last_lidar_detection_phase_ = "disabled_by_fusion_mode";
    return;
  }

  LVdotRuntimeState snapshot;
  std::size_t current_lidar_count = 0;
  constexpr double kMaxDepthLidarSkewSec = 0.20;
  constexpr double kMaxLidarYoloSkewSec = 0.50;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++lidar_detection_tick_count_;
    last_lidar_detection_phase_ = "snapshot";
    snapshot = runtime_state_;
    current_lidar_count = lidar_count_;
  }

  if (!snapshot.latest_lidar_pointcloud || current_lidar_count <= last_lidar_processed_count_) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_lidar_detection_phase_ = "waiting_new_lidar";
    return;
  }

  const auto set_lidar_phase = [this](const char * phase) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_lidar_detection_phase_ = phase;
    };
  set_lidar_phase("run_lidar_detector");
  const auto lidar_output = run_lidar_detector(snapshot, config_, set_lidar_phase);
  snapshot.raw_lidar_samples = lidar_output.raw_samples;
  snapshot.filtered_lidar_samples = lidar_output.filtered_samples;
  snapshot.lidar_bboxes = lidar_output.bboxes;
  snapshot.lidar_clusters = lidar_output.clusters;
  snapshot.lidar_cluster_centers = lidar_output.centers;
  snapshot.lidar_cluster_stds = lidar_output.stds;

  // If depth is too old relative to LiDAR, do not fuse stale UV/DB branches
  // into this LiDAR tick. This prevents stale visual boxes from polluting
  // current LiDAR-only detections.
  if (snapshot.latest_lidar_pointcloud && snapshot.latest_depth_image) {
    const rclcpp::Time lidar_stamp(snapshot.latest_lidar_pointcloud->header.stamp);
    const rclcpp::Time depth_stamp(snapshot.latest_depth_image->header.stamp);
    const double skew_sec = std::abs((lidar_stamp - depth_stamp).seconds());
    if (skew_sec > kMaxDepthLidarSkewSec) {
      snapshot.uv_bboxes.clear();
      snapshot.db_bboxes.clear();
      snapshot.db_clusters.clear();
      snapshot.db_cluster_centers.clear();
      snapshot.db_cluster_stds.clear();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Depth/LiDAR skew %.3fs > %.3fs. Fallback to lidar-only fusion for this LiDAR tick.",
        skew_sec,
        kMaxDepthLidarSkewSec);
    }
  } else if (!snapshot.latest_depth_image) {
    snapshot.uv_bboxes.clear();
    snapshot.db_bboxes.clear();
    snapshot.db_clusters.clear();
    snapshot.db_cluster_centers.clear();
    snapshot.db_cluster_stds.clear();
  }

  // Drop stale YOLO detections for this lidar tick.
  if (snapshot.latest_lidar_pointcloud && snapshot.latest_yolo_detections) {
    const rclcpp::Time lidar_stamp(snapshot.latest_lidar_pointcloud->header.stamp);
    const rclcpp::Time yolo_stamp(snapshot.latest_yolo_detections->header.stamp);
    const double skew_sec = std::abs((lidar_stamp - yolo_stamp).seconds());
    if (skew_sec > kMaxLidarYoloSkewSec) {
      snapshot.latest_yolo_detections.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "LiDAR/YOLO skew %.3fs > %.3fs. Ignore stale YOLO for this lidar fusion tick.",
        skew_sec,
        kMaxLidarYoloSkewSec);
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_lidar_detection_phase_ = "build_filter_input";
  }
  const auto filter_input = build_filter_input(snapshot, config_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_lidar_detection_phase_ = "run_filter";
  }
  const auto filter_output = onboardDetector::filterLVBBoxes(filter_input);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_lidar_detection_phase_ = "apply_output";
    runtime_state_.raw_lidar_samples = lidar_output.raw_samples;
    runtime_state_.filtered_lidar_samples = lidar_output.filtered_samples;
    runtime_state_.lidar_bboxes = lidar_output.bboxes;
    runtime_state_.lidar_clusters = lidar_output.clusters;
    runtime_state_.lidar_cluster_centers = lidar_output.centers;
    runtime_state_.lidar_cluster_stds = lidar_output.stds;
    last_raw_lidar_sample_count_ = runtime_state_.raw_lidar_samples.size();
    last_filtered_lidar_sample_count_ = runtime_state_.filtered_lidar_samples.size();
    last_lidar_bbox_count_ = runtime_state_.lidar_bboxes.size();

    apply_filter_output(filter_output, runtime_state_);
    refresh_filtered_cluster_centers(filter_output);
    update_common_filter_stats(filter_output);
    last_lidar_processed_count_ = current_lidar_count;
    last_lidar_detection_phase_ = "idle";
  }
}

void LVdotDetectorNode::on_tracking_timer()
{
  LVdotRuntimeState snapshot;
  std::size_t filter_seq = 0;
  std::size_t last_tracking_filter_seq = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++tracking_tick_count_;
    last_fixed_size_count_ = 0;
    last_tracking_phase_ = "snapshot";
    snapshot = runtime_state_;
    filter_seq = filter_update_seq_;
    last_tracking_filter_seq = last_tracking_filter_update_seq_;
  }

  if (filter_seq == last_tracking_filter_seq) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_tracking_phase_ = "waiting_new_filtered_update";
    return;
  }
  if (snapshot.filtered_bboxes.empty()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_tracking_phase_ = "waiting_filtered_bboxes";
    last_tracking_filter_update_seq_ = filter_seq;
    runtime_state_.track_states.clear();
    runtime_state_.tracked_bboxes.clear();
    runtime_state_.box_history.clear();
    last_track_count_ = 0;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_tracking_phase_ = "build_input";
  }
  const auto tracking_input = build_tracking_input(snapshot, config_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_tracking_phase_ = "run_core";
  }
  const auto tracking_output = onboardDetector::runTracking(tracking_input);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_tracking_phase_ = "apply_output";
    apply_tracking_output(tracking_output, runtime_state_);
    last_tracking_filter_update_seq_ = filter_seq;
    ++tracking_update_seq_;
    last_fixed_size_count_ = tracking_output.fixedSizeCount;
    last_track_count_ = runtime_state_.tracked_bboxes.size();
    last_tracking_phase_ = "idle";
  }
}

void LVdotDetectorNode::on_classification_timer()
{
  LVdotRuntimeState snapshot;
  std::size_t tracking_seq = 0;
  std::size_t last_classification_tracking_seq = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++classification_tick_count_;
    last_classification_phase_ = "snapshot";
    snapshot = runtime_state_;
    tracking_seq = tracking_update_seq_;
    last_classification_tracking_seq = last_classification_tracking_update_seq_;
  }
  if (tracking_seq == last_classification_tracking_seq) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_classification_phase_ = "waiting_new_tracking_update";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_classification_phase_ = "build_input";
  }
  const auto classification_input = build_classification_input(snapshot, config_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_classification_phase_ = "run_core";
  }
  const auto classification_output = onboardDetector::runClassification(classification_input);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_classification_phase_ = "apply_output";
    apply_classification_output(classification_output, runtime_state_, last_dynamic_rejected_by_size_);
    last_classification_tracking_update_seq_ = tracking_seq;
    last_dynamic_count_ = runtime_state_.dynamic_bboxes.size();
    last_classification_phase_ = "idle";
  }
}

void LVdotDetectorNode::on_vis_timer()
{
  LVdotRuntimeState snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++vis_tick_count_;
    snapshot = runtime_state_;
  }

  const auto frame_id =
    snapshot.latest_pose ? snapshot.latest_pose->header.frame_id :
    (snapshot.latest_odom ? snapshot.latest_odom->header.frame_id : "map");
  const auto stamp = now();
  const auto filtered_depth_points = samples_to_points(snapshot.filtered_depth_samples);
  const auto cluster_split = split_points_by_box_flags(snapshot.filtered_clusters, snapshot.filtered_bboxes);
  std::vector<geometry_msgs::msg::Point> dynamic_points = cluster_split.dynamic_points;
  std::vector<geometry_msgs::msg::Point> static_points = cluster_split.static_points;
  if (dynamic_points.empty() && !snapshot.dynamic_bboxes.empty()) {
    const auto filtered_cluster_points = clusters_to_points(snapshot.filtered_clusters);
    dynamic_points = points_in_boxes(filtered_cluster_points, snapshot.dynamic_bboxes);
    static_points = points_outside_boxes(filtered_cluster_points, snapshot.dynamic_bboxes);
  }
  const auto raw_dynamic_points = points_in_boxes(snapshot.raw_lidar_samples, snapshot.dynamic_bboxes);

  if (auto color_image = make_detected_color_image(snapshot, frame_id)) {
    detected_color_img_pub_->publish(*color_image);
  }

  if (auto depth_image = make_detected_depth_image(snapshot, config_, frame_id)) {
    uv_depth_map_pub_->publish(*depth_image);
  }

  if (auto u_depth_image = make_detected_u_depth_image(snapshot, config_, frame_id)) {
    u_depth_map_pub_->publish(*u_depth_image);
  }

  if (auto bird_view = make_bird_view_image(snapshot, config_, frame_id, stamp)) {
    uv_bird_view_pub_->publish(*bird_view);
  }

  visual_bboxes_pub_->publish(
    make_box_markers(
      snapshot.visual_bboxes, frame_id, "visual",
      make_color(0.2f, 0.7f, 1.0f, 0.35f), stamp));
  uv_bboxes_pub_->publish(
    make_box_markers(
      snapshot.uv_bboxes, frame_id, "uv",
      make_color(0.5f, 0.5f, 1.0f, 0.25f), stamp));
  db_bboxes_pub_->publish(
    make_box_markers(
      snapshot.db_bboxes, frame_id, "db",
      make_color(0.1f, 0.6f, 1.0f, 0.25f), stamp));
  lidar_bboxes_pub_->publish(
    make_box_markers(
      snapshot.lidar_bboxes, frame_id, "lidar",
      make_color(0.1f, 1.0f, 0.3f, 0.35f), stamp));
  filtered_bboxes_before_yolo_pub_->publish(
    make_box_markers(
      snapshot.filtered_bboxes_before_yolo, frame_id, "filtered_before_yolo",
      make_color(1.0f, 0.8f, 0.2f, 0.30f), stamp));
  filtered_bboxes_pub_->publish(
    make_box_markers(
      snapshot.filtered_bboxes, frame_id, "filtered",
      make_color(0.20f, 0.45f, 1.00f, 0.80f), stamp));
  tracked_bboxes_pub_->publish(
    make_box_markers(
      snapshot.tracked_bboxes, frame_id, "tracked",
      make_color(0.05f, 0.30f, 1.00f, 0.90f), stamp));
  dynamic_bboxes_pub_->publish(
    make_box_markers(
      snapshot.dynamic_bboxes, frame_id, "dynamic",
      make_color(1.0f, 0.2f, 0.2f, 0.45f), stamp));
  history_traj_pub_->publish(make_history_markers(snapshot.track_states, frame_id, stamp));
  vel_vis_pub_->publish(make_velocity_markers(snapshot.tracked_bboxes, frame_id, stamp));

  filtered_depth_points_pub_->publish(make_pointcloud2(filtered_depth_points, frame_id, stamp));
  filtered_points_pub_->publish(make_uniform_colored_pointcloud2(static_points, frame_id, stamp, 255, 255, 255));
  dynamic_points_pub_->publish(make_uniform_colored_pointcloud2(dynamic_points, frame_id, stamp, 255, 80, 60));
  raw_dynamic_points_pub_->publish(make_uniform_colored_pointcloud2(raw_dynamic_points, frame_id, stamp, 255, 80, 60));
  raw_lidar_points_pub_->publish(
    make_pointcloud2(samples_to_points(snapshot.raw_lidar_samples), frame_id, stamp));

  {
    std::ostringstream dbg;
    dbg << "clusters=" << snapshot.filtered_clusters.size()
        << " filtered_boxes=" << snapshot.filtered_bboxes.size()
        << " dynamic_boxes=" << snapshot.dynamic_bboxes.size() << "\n";

    auto in_dynamic_bboxes = [&snapshot](double id) {
      constexpr double kEps = 1e-6;
      for (const auto & box : snapshot.dynamic_bboxes) {
        if (std::abs(box.id - id) < kEps) {
          return true;
        }
      }
      return false;
    };

    for (std::size_t i = 0; i < snapshot.filtered_clusters.size(); ++i) {
      const bool has_box = i < snapshot.filtered_bboxes.size();
      const auto point_count = snapshot.filtered_clusters[i].size();
      if (!has_box) {
        dbg << "cluster[" << i << "]: points=" << point_count
            << " box=none\n";
        continue;
      }

      const auto & box = snapshot.filtered_bboxes[i];
      dbg << "cluster[" << i << "]: points=" << point_count
          << " box_id=" << box.id
          << " is_human=" << (box.is_human ? 1 : 0)
          << " is_dynamic=" << (box.is_dynamic ? 1 : 0)
          << " is_dynamic_candidate=" << (box.is_dynamic_candidate ? 1 : 0)
          << " in_dynamic_bboxes=" << (in_dynamic_bboxes(box.id) ? 1 : 0)
          << " xyz=(" << box.x << "," << box.y << "," << box.z << ")\n";
    }

    std_msgs::msg::String dbg_msg;
    dbg_msg.data = dbg.str();
    cluster_debug_pub_->publish(dbg_msg);
  }

  if (snapshot.latest_lidar_pointcloud) {
    lidar_clusters_pub_->publish(
      make_colored_pointcloud2(snapshot.lidar_clusters, frame_id, stamp, true));
    downsample_points_pub_->publish(make_pointcloud2(samples_to_points(snapshot.filtered_lidar_samples), frame_id, stamp));
  } else {
    lidar_clusters_pub_->publish(
      make_colored_pointcloud2(snapshot.lidar_clusters, frame_id, stamp, true));
    downsample_points_pub_->publish(make_pointcloud2(samples_to_points(snapshot.filtered_lidar_samples), frame_id, stamp));
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_dynamic_filtered_point_count_ = dynamic_points.size();
    last_raw_dynamic_point_count_ = raw_dynamic_points.size();
  }
}

}  // namespace lvdot_ros2
