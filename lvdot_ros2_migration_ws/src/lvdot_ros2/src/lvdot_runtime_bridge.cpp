#include "lvdot_ros2/lvdot_runtime_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "lvdot_ros2/lvdot_box3d.hpp"

namespace lvdot_ros2
{

namespace
{

cv::Mat depth_image_to_uint16_mm(
  const sensor_msgs::msg::Image & image,
  const LVdotDetectorConfig & config)
{
  const int rows = static_cast<int>(image.height);
  const int cols = static_cast<int>(image.width);
  if (rows <= 0 || cols <= 0 || image.data.empty()) {
    return {};
  }

  cv::Mat out(rows, cols, CV_16UC1, cv::Scalar(0));

  if (image.encoding == "32FC1") {
    const float * src = reinterpret_cast<const float *>(image.data.data());
    const std::size_t row_stride = image.step / sizeof(float);
    for (int v = 0; v < rows; ++v) {
      uint16_t * dst = out.ptr<uint16_t>(v);
      const float * src_row = src + static_cast<std::size_t>(v) * row_stride;
      for (int u = 0; u < cols; ++u) {
        const float value = src_row[u];
        if (!std::isfinite(value) || value <= 0.0f) {
          dst[u] = 0u;
          continue;
        }
        const double mm = static_cast<double>(value) * 1000.0;
        dst[u] = (mm > static_cast<double>(std::numeric_limits<uint16_t>::max()))
          ? std::numeric_limits<uint16_t>::max()
          : static_cast<uint16_t>(mm);
      }
    }
    return out;
  }

  if (image.encoding == "16UC1") {
    const double scale = std::max(config.depth_scale_factor, 1e-6);
    const uint16_t * src = reinterpret_cast<const uint16_t *>(image.data.data());
    const std::size_t row_stride = image.step / sizeof(uint16_t);
    for (int v = 0; v < rows; ++v) {
      uint16_t * dst = out.ptr<uint16_t>(v);
      const uint16_t * src_row = src + static_cast<std::size_t>(v) * row_stride;
      for (int u = 0; u < cols; ++u) {
        if (src_row[u] == 0u) {
          dst[u] = 0u;
          continue;
        }
        const double meters = static_cast<double>(src_row[u]) / scale;
        const double mm = meters * 1000.0;
        dst[u] = (mm > static_cast<double>(std::numeric_limits<uint16_t>::max()))
          ? std::numeric_limits<uint16_t>::max()
          : static_cast<uint16_t>(mm);
      }
    }
    return out;
  }

  return {};
}

void matrix4_to_rotation_translation(
  const std::vector<double> & flat,
  Eigen::Matrix3d & rotation,
  Eigen::Vector3d & translation)
{
  rotation.setIdentity();
  translation.setZero();
  if (flat.size() < 12) {
    return;
  }
  rotation <<
    flat[0], flat[1], flat[2],
    flat[4], flat[5], flat[6],
    flat[8], flat[9], flat[10];
  translation << flat[3], flat[7], flat[11];
}

Eigen::Matrix3d world_rotation_from_state(const LVdotRuntimeState & state)
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

std::vector<onboardDetector::ImageBBox2D> to_core_yolo_detections(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections)
{
  std::vector<onboardDetector::ImageBBox2D> output;
  if (!detections) {
    return output;
  }

  output.reserve(detections->detections.size());
  for (const auto & detection : detections->detections) {
    if (detection.bbox.size_x <= 0.0 || detection.bbox.size_y <= 0.0) {
      continue;
    }
    onboardDetector::ImageBBox2D box;
    box.x = static_cast<int>(std::round(detection.bbox.center.position.x - detection.bbox.size_x * 0.5));
    box.y = static_cast<int>(std::round(detection.bbox.center.position.y - detection.bbox.size_y * 0.5));
    box.width = static_cast<int>(std::round(detection.bbox.size_x));
    box.height = static_cast<int>(std::round(detection.bbox.size_y));
    box.is_human = true;
    box.score = detection_score(detection);
    output.push_back(box);
  }
  return output;
}

onboardDetector::TrackState to_core_track_state(const LVdotRuntimeState::TrackState & track)
{
  onboardDetector::TrackState core;
  core.kf = track.kf;
  core.kf_initialized = track.kf_initialized;
  core.filter_state = track.filter_state;
  core.filter_initialized = track.filter_initialized;
  core.currentBox = to_core_box3d(track.current_box);
  core.currentCenter = Eigen::Vector3d(track.current_center.x, track.current_center.y, track.current_center.z);
  core.currentStd = Eigen::Vector3d(track.current_std.x, track.current_std.y, track.current_std.z);
  for (const auto & box : track.box_history) {
    core.boxHistory.push_back(to_core_box3d(box));
  }
  for (const auto & cluster : track.cluster_history) {
    core.clusterHistory.push_back(to_core_cluster(cluster));
  }
  for (const auto & center : track.center_history) {
    core.centerHistory.emplace_back(center.x, center.y, center.z);
  }
  for (const auto & std : track.std_history) {
    core.stdHistory.emplace_back(std.x, std.y, std.z);
  }
  core.matchedInFrame = track.matched_in_frame;
  core.age = track.age;
  core.consecutiveHits = track.consecutive_hits;
  core.missedFrames = track.missed_frames;
  core.confirmed = track.confirmed;
  core.hasLastObservation = track.has_last_observation;
  core.lastObservedBox = to_core_box3d(track.last_observed_box);
  core.lastObservedCenter = Eigen::Vector3d(
    track.last_observed_center.x, track.last_observed_center.y, track.last_observed_center.z);
  core.lastObservedStd = Eigen::Vector3d(
    track.last_observed_std.x, track.last_observed_std.y, track.last_observed_std.z);
  core.hasExternalPrediction = track.has_external_prediction;
  core.externalPrediction = Eigen::Vector3d(
    track.external_prediction.x,
    track.external_prediction.y,
    track.external_prediction.z);
  core.externalPredictionAgeSec = track.external_prediction_age_sec;
  return core;
}

LVdotRuntimeState::TrackState from_core_track_state(const onboardDetector::TrackState & track)
{
  LVdotRuntimeState::TrackState ros;
  ros.kf = track.kf;
  ros.kf_initialized = track.kf_initialized;
  ros.filter_state = track.filter_state;
  ros.filter_initialized = track.filter_initialized;
  ros.current_box = from_core_box3d(track.currentBox);
  ros.current_center = eigen_to_point(track.currentCenter);
  ros.current_std = eigen_to_vector3(track.currentStd);
  for (const auto & box : track.boxHistory) {
    ros.box_history.push_back(from_core_box3d(box));
  }
  for (const auto & cluster : track.clusterHistory) {
    ros.cluster_history.push_back(from_core_cluster(cluster));
  }
  for (const auto & center : track.centerHistory) {
    ros.center_history.push_back(eigen_to_point(center));
  }
  for (const auto & std : track.stdHistory) {
    ros.std_history.push_back(eigen_to_vector3(std));
  }
  ros.matched_in_frame = track.matchedInFrame;
  ros.age = track.age;
  ros.consecutive_hits = track.consecutiveHits;
  ros.missed_frames = track.missedFrames;
  ros.confirmed = track.confirmed;
  ros.has_last_observation = track.hasLastObservation;
  ros.last_observed_box = from_core_box3d(track.lastObservedBox);
  ros.last_observed_center = eigen_to_point(track.lastObservedCenter);
  ros.last_observed_std = eigen_to_vector3(track.lastObservedStd);
  ros.has_external_prediction = track.hasExternalPrediction;
  ros.external_prediction = eigen_to_point(track.externalPrediction);
  ros.external_prediction_age_sec = track.externalPredictionAgeSec;
  return ros;
}

onboardDetector::TrackingConfig build_tracking_config(const LVdotDetectorConfig & config)
{
  onboardDetector::TrackingConfig tracking;
  tracking.dt = config.time_step;
  tracking.maxMatchRange = config.max_match_range;
  tracking.maxMatchSizeRange = config.max_size_diff_range;
  tracking.featureWeights = Eigen::VectorXd::Zero(9);
  for (std::size_t i = 0; i < std::min<std::size_t>(9, config.feature_weight.size()); ++i) {
    tracking.featureWeights(static_cast<Eigen::Index>(i)) = config.feature_weight[i];
  }
  tracking.simPrevWeight = config.sim_prev_weight;
  tracking.simPropedWeight = config.sim_proped_weight;
  tracking.adaptiveSimilarityWeight = config.adaptive_similarity_weight;
  tracking.similarityDistanceNorm = config.similarity_distance_norm;
  tracking.minMatchSimilarity = config.min_match_similarity;
  tracking.trackHighScoreThreshold = config.tracker_high_score_threshold;
  tracking.trackLowScoreThreshold = config.tracker_low_score_threshold;
  tracking.newTrackScoreThreshold = config.tracker_new_track_score_threshold;
  tracking.tentativeMinHits = config.tracker_tentative_min_hits;
  tracking.tentativeMaxUnmatchedFrames = config.tracker_tentative_max_unmatched_frames;
  tracking.enableGruAssociationCost = config.enable_gru_association_cost;
  tracking.gruAssociationWeight = config.gru_association_weight;
  tracking.gruPredictionGate = config.gru_prediction_gate_m;
  tracking.histSize = config.history_size;
  tracking.fixSizeHistThresh = config.fix_size_history_threshold;
  tracking.fixSizeDimThresh = config.fix_size_dimension_threshold;
  const auto & k = config.kalman_filter_param;
  tracking.eP = k.size() > 0 ? k[0] : tracking.eP;
  tracking.eQPos = k.size() > 1 ? k[1] : tracking.eQPos;
  tracking.eQVel = k.size() > 2 ? k[2] : tracking.eQVel;
  tracking.eQAcc = k.size() > 3 ? k[3] : tracking.eQAcc;
  tracking.eRPos = k.size() > 4 ? k[4] : tracking.eRPos;
  tracking.eRVel = k.size() > 5 ? k[5] : tracking.eRVel;
  tracking.eRAcc = k.size() > 6 ? k[6] : tracking.eRAcc;
  tracking.kfAvgFrames = config.kalman_filter_averaging_frames;
  tracking.maxUnmatchedFrames = config.max_unmatched_frames;
  // §3.3 adaptive noise knobs.  Actual Hc/Hl values are filled per-frame in
  // build_tracking_input from the latest quality-vector subscriber state.
  tracking.noiseAdaptationEnabled = config.qcgaf_noise_adaptation_enabled;
  tracking.alphaQ = config.qcgaf_alpha_q;
  tracking.alphaR = config.qcgaf_alpha_r;
  return tracking;
}

onboardDetector::ClassificationConfig build_classification_config(const LVdotDetectorConfig & config)
{
  onboardDetector::ClassificationConfig classification;
  classification.dt = config.time_step;
  classification.skipFrame = config.frame_skip;
  classification.dynamicVelocityThreshold = config.dynamic_velocity_threshold;
  classification.dynamicVotingThreshold = config.dynamic_voting_threshold;
  classification.forceDynamicFrames = config.frames_force_dynamic;
  classification.forceDynamicCheckRange = config.frames_force_dynamic_check_range;
  classification.dynamicConsistencyThreshold = config.dynamic_consistency_threshold;
  classification.constrainSize = config.target_constrain_size;
  for (std::size_t i = 0; i + 2 < config.target_object_size.size(); i += 3) {
    classification.targetObjectSize.emplace_back(
      config.target_object_size[i],
      config.target_object_size[i + 1],
      config.target_object_size[i + 2]);
  }
  return classification;
}

}  // namespace

onboardDetector::DetectionInput build_detection_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  onboardDetector::DetectionInput input;
  input.depthImageMm = depth_image_to_uint16_mm(*state.latest_depth_image, config);
  auto & detection = input.config;
  bool intrinsics_ok = false;
  if (config.depth_intrinsics.size() >= 4) {
    detection.fx = config.depth_intrinsics[0];
    detection.fy = config.depth_intrinsics[1];
    detection.cx = config.depth_intrinsics[2];
    detection.cy = config.depth_intrinsics[3];
    intrinsics_ok = std::isfinite(detection.fx) && std::isfinite(detection.fy) &&
      std::isfinite(detection.cx) && std::isfinite(detection.cy) &&
      std::abs(detection.fx) > 1e-6 && std::abs(detection.fy) > 1e-6;
  }
  if (!intrinsics_ok) {
    detection.fx = 337.35705085528514;
    detection.fy = 337.35705085528514;
    detection.cx = 320.0;
    detection.cy = 240.0;
  }
  detection.depthScale = std::max(config.depth_scale_factor, 1e-6);
  detection.depthMinValue = config.depth_min_value;
  detection.depthMaxValue = config.depth_max_value;
  detection.raycastMaxLength = config.depth_max_value;
  detection.depthFilterMargin = config.depth_filter_margin;
  detection.skipPixel = config.depth_skip_pixel;
  detection.groundHeight = config.ground_height;
  detection.roofHeight = config.roof_height;
  detection.voxelOccThresh = config.voxel_occupied_thresh;
  detection.dbMinPointsCluster = config.dbscan_min_points_cluster;
  detection.dbEpsilon = config.dbscan_search_range_epsilon;
  detection.localSensorRange = Eigen::Vector3d(
    config.local_sensor_range.size() > 0 ? config.local_sensor_range[0] : 5.0,
    config.local_sensor_range.size() > 1 ? config.local_sensor_range[1] : 5.0,
    config.local_sensor_range.size() > 2 ? config.local_sensor_range[2] : 5.0);
  detection.position = Eigen::Vector3d(
    state.current_position.x,
    state.current_position.y,
    state.current_position.z);
  Eigen::Matrix3d body_R_cam;
  Eigen::Vector3d body_t_cam;
  matrix4_to_rotation_translation(config.body_to_camera_depth, body_R_cam, body_t_cam);
  const Eigen::Matrix3d world_R_body = world_rotation_from_state(state);
  detection.positionDepth = world_R_body * body_t_cam + detection.position;
  detection.orientationDepth = world_R_body * body_R_cam;
  // Keep depth/DB branch permissive (ROS1 strategy: "wide first, tighten later").
  // Do not apply min-size hard gate at early depth clustering stage.
  detection.minObjectSize = Eigen::Vector3d(0.0, 0.0, 0.0);
  detection.maxObjectSize = Eigen::Vector3d(
    config.max_object_size[0],
    config.max_object_size[1],
    config.max_object_size[2]);
  detection.uMapRowDownsample = config.u_map_row_downsample;
  detection.uMapColScale = static_cast<float>(std::clamp(config.u_map_col_scale, 0.1, 1.0));
  detection.uMapThresholdPoint = static_cast<float>(std::max(1, config.u_map_threshold_point));
  detection.uMapThresholdLine = static_cast<float>(std::max(1, config.u_map_threshold_line));
  detection.uMapMinLengthLine = std::max(1, config.u_map_min_length_line);
  detection.uMapMinBBoxArea = std::max(1, config.u_map_min_bbox_area);
  return input;
}

void apply_detection_output(
  const onboardDetector::DetectionOutput & output,
  LVdotRuntimeState & state)
{
  state.projected_depth_samples = from_core_cluster(output.projectedDepthPoints);
  state.filtered_depth_samples = from_core_cluster(output.filteredDepthPoints);

  state.db_bboxes.clear();
  state.db_bboxes.reserve(output.dbBBoxes.size());
  for (const auto & box : output.dbBBoxes) {
    state.db_bboxes.push_back(from_core_box3d(box));
  }

  state.db_clusters.clear();
  state.db_clusters.reserve(output.dbClusters.size());
  for (const auto & cluster : output.dbClusters) {
    state.db_clusters.push_back(from_core_cluster(cluster));
  }

  state.db_cluster_centers.clear();
  state.db_cluster_centers.reserve(output.dbClusterCenters.size());
  for (const auto & center : output.dbClusterCenters) {
    state.db_cluster_centers.push_back(eigen_to_point(center));
  }

  state.db_cluster_stds.clear();
  state.db_cluster_stds.reserve(output.dbClusterStds.size());
  for (const auto & std : output.dbClusterStds) {
    state.db_cluster_stds.push_back(eigen_to_vector3(std));
  }

  state.uv_bboxes.clear();
  state.uv_bboxes.reserve(output.uvBBoxes.size());
  for (const auto & box : output.uvBBoxes) {
    state.uv_bboxes.push_back(from_core_box3d(box));
  }

  state.uv_depth_show = output.depthShow.clone();
  state.uv_u_map_show = output.uMapShow.clone();
  state.uv_bird_view = output.birdView.clone();
}

onboardDetector::FilterLVBBoxesInput build_filter_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  onboardDetector::FilterLVBBoxesInput input;
  const auto copy_boxes_to_core = [](const auto & src, auto & dst) {
      dst.reserve(src.size());
      for (const auto & box : src) {
        dst.push_back(to_core_box3d(box));
      }
    };
  copy_boxes_to_core(state.uv_bboxes, input.uvBBoxes);
  copy_boxes_to_core(state.db_bboxes, input.dbBBoxes);
  copy_boxes_to_core(state.lidar_bboxes, input.lidarBBoxes);

  input.pcClustersVisual.reserve(state.db_clusters.size());
  for (const auto & cluster : state.db_clusters) {
    input.pcClustersVisual.push_back(to_core_cluster(cluster));
  }
  input.pcClusterCentersVisual.reserve(state.db_cluster_centers.size());
  for (const auto & center : state.db_cluster_centers) {
    input.pcClusterCentersVisual.emplace_back(center.x, center.y, center.z);
  }
  input.pcClusterStdsVisual.reserve(state.db_cluster_stds.size());
  for (const auto & std : state.db_cluster_stds) {
    input.pcClusterStdsVisual.emplace_back(std.x, std.y, std.z);
  }

  input.lidarPcClusters.reserve(state.lidar_clusters.size());
  for (const auto & cluster : state.lidar_clusters) {
    input.lidarPcClusters.push_back(to_core_cluster(cluster));
  }
  input.lidarPcClusterCenters.reserve(state.lidar_cluster_centers.size());
  for (const auto & center : state.lidar_cluster_centers) {
    input.lidarPcClusterCenters.emplace_back(center.x, center.y, center.z);
  }
  input.lidarPcClusterStds.reserve(state.lidar_cluster_stds.size());
  for (const auto & std : state.lidar_cluster_stds) {
    input.lidarPcClusterStds.emplace_back(std.x, std.y, std.z);
  }

  input.yoloDetectionResults = to_core_yolo_detections(state.latest_yolo_detections);

  Eigen::Matrix3d body_R_cam;
  Eigen::Vector3d body_t_cam;
  matrix4_to_rotation_translation(config.body_to_camera_color, body_R_cam, body_t_cam);
  const Eigen::Matrix3d world_R_body = world_rotation_from_state(state);

  const Eigen::Vector3d world_position(
    state.current_position.x,
    state.current_position.y,
    state.current_position.z);
  input.positionColor = world_R_body * body_t_cam + world_position;
  input.orientationColor = world_R_body * body_R_cam;
  bool intrinsics_ok = false;
  if (config.color_intrinsics.size() >= 4) {
    input.fxC = config.color_intrinsics[0];
    input.fyC = config.color_intrinsics[1];
    input.cxC = config.color_intrinsics[2];
    input.cyC = config.color_intrinsics[3];
    intrinsics_ok = std::isfinite(input.fxC) && std::isfinite(input.fyC) &&
      std::isfinite(input.cxC) && std::isfinite(input.cyC) &&
      std::abs(input.fxC) > 1e-6 && std::abs(input.fyC) > 1e-6;
  }
  if (!intrinsics_ok && config.depth_intrinsics.size() >= 4) {
    input.fxC = config.depth_intrinsics[0];
    input.fyC = config.depth_intrinsics[1];
    input.cxC = config.depth_intrinsics[2];
    input.cyC = config.depth_intrinsics[3];
    intrinsics_ok = std::isfinite(input.fxC) && std::isfinite(input.fyC) &&
      std::isfinite(input.cxC) && std::isfinite(input.cyC) &&
      std::abs(input.fxC) > 1e-6 && std::abs(input.fyC) > 1e-6;
  }
  if (!intrinsics_ok) {
    // Guard against invalid 2D projection parameters. Disable YOLO-assisted
    // split/match for this tick rather than projecting with bad intrinsics.
    input.yoloDetectionResults.clear();
    input.fxC = 337.35705085528514;
    input.fyC = 337.35705085528514;
    input.cxC = 320.0;
    input.cyC = 240.0;
  }
  input.boxIOUThresh = config.filtering_bbox_iou_threshold;
  return input;
}

void apply_filter_output(
  const onboardDetector::FilterLVBBoxesOutput & output,
  LVdotRuntimeState & state)
{
  state.visual_bboxes.clear();
  state.visual_clusters.clear();
  state.filtered_bboxes_before_yolo.clear();
  state.filtered_clusters_before_yolo.clear();
  state.filtered_cluster_centers.clear();
  state.filtered_cluster_stds_before_yolo.clear();
  state.filtered_bboxes.clear();
  state.filtered_clusters.clear();
  state.filtered_cluster_stds.clear();

  for (const auto & box : output.visualBBoxes) {
    state.visual_bboxes.push_back(from_core_box3d(box));
  }
  for (const auto & cluster : output.visualPcClusters) {
    state.visual_clusters.push_back(from_core_cluster(cluster));
  }
  state.visual_cluster_stds.clear();
  for (const auto & std : output.visualPcClusterStds) {
    state.visual_cluster_stds.push_back(eigen_to_vector3(std));
  }

  for (const auto & box : output.filteredBBoxesBeforeYolo) {
    state.filtered_bboxes_before_yolo.push_back(from_core_box3d(box));
  }
  for (const auto & cluster : output.filteredPcClustersBeforeYolo) {
    state.filtered_clusters_before_yolo.push_back(from_core_cluster(cluster));
  }
  for (const auto & center : output.filteredPcClusterCentersBeforeYolo) {
    state.filtered_cluster_centers.push_back(eigen_to_point(center));
  }
  for (const auto & std : output.filteredPcClusterStdsBeforeYolo) {
    state.filtered_cluster_stds_before_yolo.push_back(eigen_to_vector3(std));
  }

  for (const auto & box : output.filteredBBoxes) {
    state.filtered_bboxes.push_back(from_core_box3d(box));
  }
  for (const auto & cluster : output.filteredPcClusters) {
    state.filtered_clusters.push_back(from_core_cluster(cluster));
  }
  for (const auto & std : output.filteredPcClusterStds) {
    state.filtered_cluster_stds.push_back(eigen_to_vector3(std));
  }
}

onboardDetector::TrackingInput build_tracking_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  onboardDetector::TrackingInput input;
  input.filteredBBoxes.reserve(state.filtered_bboxes.size());
  for (const auto & box : state.filtered_bboxes) {
    input.filteredBBoxes.push_back(to_core_box3d(box));
  }
  input.filteredPcClusters.reserve(state.filtered_clusters.size());
  for (const auto & cluster : state.filtered_clusters) {
    input.filteredPcClusters.push_back(to_core_cluster(cluster));
  }
  input.filteredPcClusterCenters.reserve(state.filtered_cluster_centers.size());
  for (const auto & center : state.filtered_cluster_centers) {
    input.filteredPcClusterCenters.emplace_back(center.x, center.y, center.z);
  }
  input.filteredPcClusterStds.reserve(state.filtered_cluster_stds.size());
  for (const auto & std : state.filtered_cluster_stds) {
    input.filteredPcClusterStds.emplace_back(std.x, std.y, std.z);
  }
  input.tracks.reserve(state.track_states.size());
  for (const auto & track : state.track_states) {
    input.tracks.push_back(to_core_track_state(track));
  }
  input.position = Eigen::Vector3d(
    state.current_position.x,
    state.current_position.y,
    state.current_position.z);
  input.config = build_tracking_config(config);
  input.config.Hc = state.qcgaf_Hc;
  input.config.Hl = state.qcgaf_Hl;
  return input;
}

void apply_qcgaf_geometry_refinement(
  LVdotRuntimeState & state,
  double max_match_distance)
{
  // Path Z: replace rule-fusion box geometry with the nearest QC-GAF box
  // (within max_match_distance).  Clusters and Box3D flags/velocity are
  // preserved so downstream tracker still has the cluster history it expects.
  if (state.filtered_bboxes.empty() || state.qcgaf_filtered_bboxes.empty()) {
    return;
  }
  const double max_sq = max_match_distance * max_match_distance;
  for (auto & rule_box : state.filtered_bboxes) {
    double best_sq = std::numeric_limits<double>::infinity();
    const Box3D * best = nullptr;
    for (const auto & qc_box : state.qcgaf_filtered_bboxes) {
      const double dx = qc_box.x - rule_box.x;
      const double dy = qc_box.y - rule_box.y;
      const double dz = qc_box.z - rule_box.z;
      const double dsq = dx * dx + dy * dy + dz * dz;
      if (dsq < best_sq) {
        best_sq = dsq;
        best = &qc_box;
      }
    }
    if (best != nullptr && best_sq <= max_sq) {
      rule_box.x = best->x;
      rule_box.y = best->y;
      rule_box.z = best->z;
      rule_box.x_width = best->x_width;
      rule_box.y_width = best->y_width;
      rule_box.z_width = best->z_width;
    }
  }
}

void apply_depth_auxiliary_correction(
  LVdotRuntimeState & state,
  double max_match_distance)
{
  if (state.filtered_bboxes.empty()) {
    return;
  }

  std::vector<const Box3D *> depth_boxes;
  depth_boxes.reserve(state.db_bboxes.size() + state.uv_bboxes.size());
  for (const auto & box : state.db_bboxes) {
    depth_boxes.push_back(&box);
  }
  for (const auto & box : state.uv_bboxes) {
    depth_boxes.push_back(&box);
  }
  if (depth_boxes.empty()) {
    return;
  }

  const double max_sq = max_match_distance * max_match_distance;
  constexpr double kCenterBlend = 0.35;
  constexpr double kSizeBlend = 0.20;
  for (auto & fused_box : state.filtered_bboxes) {
    double best_sq = std::numeric_limits<double>::infinity();
    const Box3D * best = nullptr;
    for (const auto * depth_box : depth_boxes) {
      const double dx = depth_box->x - fused_box.x;
      const double dy = depth_box->y - fused_box.y;
      const double dz = depth_box->z - fused_box.z;
      const double dsq = dx * dx + dy * dy + dz * dz;
      if (dsq < best_sq) {
        best_sq = dsq;
        best = depth_box;
      }
    }
    if (best == nullptr || best_sq > max_sq) {
      continue;
    }

    fused_box.x = (1.0 - kCenterBlend) * fused_box.x + kCenterBlend * best->x;
    fused_box.y = (1.0 - kCenterBlend) * fused_box.y + kCenterBlend * best->y;
    fused_box.z = (1.0 - kCenterBlend) * fused_box.z + kCenterBlend * best->z;
    fused_box.x_width = (1.0 - kSizeBlend) * fused_box.x_width + kSizeBlend * best->x_width;
    fused_box.y_width = (1.0 - kSizeBlend) * fused_box.y_width + kSizeBlend * best->y_width;
    fused_box.z_width = (1.0 - kSizeBlend) * fused_box.z_width + kSizeBlend * best->z_width;
    fused_box.is_u_map_enhanced = fused_box.is_u_map_enhanced || best->is_u_map_enhanced;
    fused_box.score = std::max(fused_box.score, best->score);
  }
}

onboardDetector::TrackingInput build_tracking_input_qcgaf_replacement(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  // Path X: use QC-GAF boxes as the tracker's observation list.  Clusters are
  // emitted as empty arrays; the tracker's cluster-aware features degrade
  // gracefully (tracking_filter.cpp falls back to box center when
  // filteredPcClusterCenters[i] is out of range).
  onboardDetector::TrackingInput input;
  input.filteredBBoxes.reserve(state.qcgaf_filtered_bboxes.size());
  for (const auto & box : state.qcgaf_filtered_bboxes) {
    input.filteredBBoxes.push_back(to_core_box3d(box));
  }
  // filteredPcClusters / filteredPcClusterCenters / filteredPcClusterStds are
  // left empty intentionally.  See doc §五.2 mode=replacement.
  input.tracks.reserve(state.track_states.size());
  for (const auto & track : state.track_states) {
    input.tracks.push_back(to_core_track_state(track));
  }
  input.position = Eigen::Vector3d(
    state.current_position.x,
    state.current_position.y,
    state.current_position.z);
  input.config = build_tracking_config(config);
  input.config.Hc = state.qcgaf_Hc;
  input.config.Hl = state.qcgaf_Hl;
  return input;
}

void apply_tracking_output(
  const onboardDetector::TrackingOutput & output,
  LVdotRuntimeState & state)
{
  state.track_states.clear();
  state.track_states.reserve(output.tracks.size());
  for (const auto & track : output.tracks) {
    state.track_states.push_back(from_core_track_state(track));
  }

  state.tracked_bboxes.clear();
  state.box_history.clear();
  state.tracked_bboxes.reserve(output.trackedBBoxes.size());
  state.box_history.reserve(state.track_states.size());
  for (const auto & box : output.trackedBBoxes) {
    state.tracked_bboxes.push_back(from_core_box3d(box));
  }
  for (const auto & track : state.track_states) {
    state.box_history.push_back(track.box_history);
  }
}

onboardDetector::ClassificationInput build_classification_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  onboardDetector::ClassificationInput input;
  input.tracks.reserve(state.track_states.size());
  for (const auto & track : state.track_states) {
    input.tracks.push_back(to_core_track_state(track));
  }
  input.config = build_classification_config(config);
  return input;
}

void apply_classification_output(
  const onboardDetector::ClassificationOutput & output,
  LVdotRuntimeState & state,
  std::size_t & dynamic_rejected_by_size_out)
{
  state.track_states.clear();
  state.track_states.reserve(output.tracks.size());
  for (const auto & track : output.tracks) {
    state.track_states.push_back(from_core_track_state(track));
  }

  // If dynamic boxes were produced this tick, update immediately and reset
  // the hysteresis counter.  If no dynamic boxes but tracks exist (consistency
  // window not yet satisfied), keep the previous list.  Only clear after
  // empty_tracks_consecutive reaches 3 (~100 ms at 30 Hz classification) to
  // avoid service calls racing transient empty windows.
  if (!output.dynamicBBoxes.empty()) {
    state.empty_tracks_consecutive = 0;
    state.dynamic_bboxes.clear();
    state.dynamic_bboxes.reserve(output.dynamicBBoxes.size());
    for (const auto & box : output.dynamicBBoxes) {
      state.dynamic_bboxes.push_back(from_core_box3d(box));
    }
  } else if (output.tracks.empty()) {
    ++state.empty_tracks_consecutive;
    if (state.empty_tracks_consecutive >= 3) {
      state.dynamic_bboxes.clear();
    }
  } else {
    // tracks non-empty but no dynamic output this tick — retain previous list
    state.empty_tracks_consecutive = 0;
  }
  dynamic_rejected_by_size_out = output.dynamicRejectedBySize;
}

}  // namespace lvdot_ros2
