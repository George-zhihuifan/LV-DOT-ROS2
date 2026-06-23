#include "lvdot_ros2/lvdot_visualization_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lvdot_ros2
{

namespace
{

int marker_id_from_box_id(double box_id)
{
  constexpr double kMinId = static_cast<double>(std::numeric_limits<int32_t>::min());
  constexpr double kMaxId = static_cast<double>(std::numeric_limits<int32_t>::max());
  const double clamped = std::clamp(std::round(box_id), kMinId, kMaxId);
  return static_cast<int>(clamped);
}

bool point_in_box(const geometry_msgs::msg::Point & point, const Box3D & box)
{
  return std::abs(point.x - box.x) <= box.x_width * 0.5 &&
    std::abs(point.y - box.y) <= box.y_width * 0.5 &&
    std::abs(point.z - box.z) <= box.z_width * 0.5;
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

cv::Mat color_image_to_bgr(const sensor_msgs::msg::Image & image)
{
  if (image.width == 0 || image.height == 0 || image.data.empty()) {
    return {};
  }

  if (image.encoding == "bgr8") {
    return cv::Mat(
      static_cast<int>(image.height),
      static_cast<int>(image.width),
      CV_8UC3,
      const_cast<unsigned char *>(image.data.data()),
      image.step).clone();
  }

  if (image.encoding == "rgb8") {
    cv::Mat rgb(
      static_cast<int>(image.height),
      static_cast<int>(image.width),
      CV_8UC3,
      const_cast<unsigned char *>(image.data.data()),
      image.step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }

  if (image.encoding == "mono8") {
    cv::Mat mono(
      static_cast<int>(image.height),
      static_cast<int>(image.width),
      CV_8UC1,
      const_cast<unsigned char *>(image.data.data()),
      image.step);
    cv::Mat bgr;
    cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  return {};
}

sensor_msgs::msg::Image make_image_msg(
  const cv::Mat & image,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & encoding)
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;
  return *cv_bridge::CvImage(header, encoding, image).toImageMsg();
}

void draw_detections(
  cv::Mat & image,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections)
{
  if (image.empty() || !detections) {
    return;
  }

  for (const auto & detection : detections->detections) {
    const auto half_w = static_cast<int>(std::round(detection.bbox.size_x * 0.5));
    const auto half_h = static_cast<int>(std::round(detection.bbox.size_y * 0.5));
    const auto cx = static_cast<int>(std::round(detection.bbox.center.position.x));
    const auto cy = static_cast<int>(std::round(detection.bbox.center.position.y));
    const cv::Rect rect(
      std::max(0, cx - half_w),
      std::max(0, cy - half_h),
      std::max(1, std::min(image.cols - std::max(0, cx - half_w), half_w * 2)),
      std::max(1, std::min(image.rows - std::max(0, cy - half_h), half_h * 2)));

    const auto label = detection_label(detection);
    const auto score = detection_score(detection);
    const bool human = detection_is_human(detection);
    const cv::Scalar color = human ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 128, 0);
    cv::rectangle(image, rect, color, 2, cv::LINE_AA);

    std::ostringstream text;
    text << (label.empty() ? "det" : label) << " " << std::fixed << std::setprecision(2) << score;
    cv::putText(
      image,
      text.str(),
      cv::Point(rect.x, std::max(15, rect.y - 4)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      color,
      1,
      cv::LINE_AA);
  }
}

bool depth_value_at(
  const sensor_msgs::msg::Image & image,
  int u,
  int v,
  double scale,
  double & depth_out)
{
  if (u < 0 || v < 0 || u >= static_cast<int>(image.width) || v >= static_cast<int>(image.height)) {
    return false;
  }

  const auto offset = static_cast<std::size_t>(v) * image.step;
  if (image.encoding == "32FC1") {
    const auto byte_offset = offset + static_cast<std::size_t>(u) * sizeof(float);
    if (byte_offset + sizeof(float) > image.data.size()) {
      return false;
    }
    float value = 0.0f;
    std::memcpy(&value, image.data.data() + byte_offset, sizeof(float));
    if (!std::isfinite(value) || value <= 0.0f) {
      return false;
    }
    depth_out = static_cast<double>(value);
    return true;
  }

  if (image.encoding == "16UC1") {
    const auto byte_offset = offset + static_cast<std::size_t>(u) * sizeof(uint16_t);
    if (byte_offset + sizeof(uint16_t) > image.data.size()) {
      return false;
    }
    uint16_t value = 0u;
    std::memcpy(&value, image.data.data() + byte_offset, sizeof(uint16_t));
    if (value == 0u) {
      return false;
    }
    depth_out = static_cast<double>(value) / std::max(scale, 1e-6);
    return true;
  }

  return false;
}

cv::Mat build_u_map_counts(
  const sensor_msgs::msg::Image & image,
  const LVdotDetectorConfig & config)
{
  const int cols = std::max(
    1,
    static_cast<int>(std::round(image.width * std::clamp(config.u_map_col_scale, 0.1, 1.0))));
  const int bins = std::max(1, static_cast<int>(image.height) / std::max(1, config.u_map_row_downsample));
  cv::Mat counts(bins, cols, CV_32SC1, cv::Scalar(0));
  const int stride = std::max(1, config.depth_skip_pixel * 2);
  const double depth_min = std::max(0.0, config.depth_min_value);
  const double depth_max = std::max(depth_min + 1e-6, config.depth_max_value);
  const double depth_bin_width = (depth_max - depth_min) / static_cast<double>(bins);

  for (int v = 0; v < static_cast<int>(image.height); v += stride) {
    for (int u = 0; u < static_cast<int>(image.width); u += stride) {
      double depth = 0.0;
      if (!depth_value_at(image, u, v, config.depth_scale_factor, depth)) {
        continue;
      }
      const double clipped = std::clamp(depth, depth_min, depth_max);
      const int depth_bin = std::clamp(
        static_cast<int>((clipped - depth_min) / std::max(1e-6, depth_bin_width)),
        0,
        bins - 1);
      const int u_scaled = std::clamp(
        static_cast<int>(std::floor(u * std::clamp(config.u_map_col_scale, 0.1, 1.0))),
        0,
        cols - 1);
      counts.at<int>(depth_bin, u_scaled) += 1;
    }
  }

  return counts;
}

cv::Mat make_depth_visualization(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  if (!state.uv_depth_show.empty()) {
    return state.uv_depth_show.clone();
  }

  const auto & image_ptr = state.latest_depth_image;
  if (!image_ptr) {
    return {};
  }
  const auto & image = *image_ptr;
  cv::Mat normalized(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC1, cv::Scalar(0));
  const double depth_min = std::max(0.0, config.depth_min_value);
  const double depth_max = std::max(depth_min + 1e-6, config.depth_max_value);
  for (int v = 0; v < static_cast<int>(image.height); ++v) {
    auto * row = normalized.ptr<unsigned char>(v);
    for (int u = 0; u < static_cast<int>(image.width); ++u) {
      double depth = 0.0;
      if (!depth_value_at(image, u, v, config.depth_scale_factor, depth)) {
        row[u] = 0;
        continue;
      }
      const double clipped = std::clamp(depth, depth_min, depth_max);
      const double ratio = 1.0 - ((clipped - depth_min) / (depth_max - depth_min));
      row[u] = static_cast<unsigned char>(std::round(ratio * 255.0));
    }
  }
  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_BONE);
  return colored;
}

cv::Mat make_u_depth_map(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  if (!state.uv_u_map_show.empty()) {
    return state.uv_u_map_show.clone();
  }

  const auto & image_ptr = state.latest_depth_image;
  if (!image_ptr) {
    return {};
  }
  const auto & image = *image_ptr;
  const cv::Mat counts = build_u_map_counts(image, config);

  double max_count = 0.0;
  cv::minMaxLoc(counts, nullptr, &max_count);
  cv::Mat normalized(counts.rows, counts.cols, CV_8UC1, cv::Scalar(0));
  if (max_count > 0.0) {
    counts.convertTo(normalized, CV_8UC1, (255.0 * 10.0) / max_count);
  }
  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
  return colored;
}

bool local_point_to_bird_pixel(
  double x_local,
  double y_local,
  int image_cols,
  int image_rows,
  double fx,
  double px,
  cv::Point & pixel_out)
{
  if (image_cols <= 0 || image_rows <= 0) {
    return false;
  }
  const cv::Point2f center(static_cast<float>(image_cols * 0.5), static_cast<float>(image_rows));
  if (x_local <= 0.0) {
    return false;
  }

  const double px_bird = center.x + (y_local * static_cast<double>(image_rows) / std::max(1e-6, fx));
  const double py_bird = center.y - x_local;
  if (!std::isfinite(px_bird) || !std::isfinite(py_bird)) {
    return false;
  }
  const int x_rounded = static_cast<int>(std::llround(px_bird));
  const int y_rounded = static_cast<int>(std::llround(py_bird));
  const int x_clamped = std::clamp(x_rounded, 0, image_cols - 1);
  const int y_clamped = std::clamp(y_rounded, 0, image_rows - 1);
  if (x_clamped < 0 || x_clamped >= image_cols || y_clamped < 0 || y_clamped >= image_rows) {
    return false;
  }
  pixel_out = cv::Point(x_clamped, y_clamped);
  return true;
}

bool cluster_local_bounds(
  const std::vector<LVdotRuntimeState::DepthSample> & cluster,
  const LVdotRuntimeState & state,
  double & x_min,
  double & x_max,
  double & y_min,
  double & y_max)
{
  if (cluster.empty()) {
    return false;
  }

  const double cyaw = std::cos(state.current_yaw);
  const double syaw = std::sin(state.current_yaw);
  x_min = std::numeric_limits<double>::max();
  x_max = std::numeric_limits<double>::lowest();
  y_min = std::numeric_limits<double>::max();
  y_max = std::numeric_limits<double>::lowest();
  for (const auto & sample : cluster) {
    const double dx = sample.point.x - state.current_position.x;
    const double dy = sample.point.y - state.current_position.y;
    const double x_local = cyaw * dx + syaw * dy;
    const double y_local = -syaw * dx + cyaw * dy;
    x_min = std::min(x_min, x_local);
    x_max = std::max(x_max, x_local);
    y_min = std::min(y_min, y_local);
    y_max = std::max(y_max, y_local);
  }
  return std::isfinite(x_min) && std::isfinite(x_max) && std::isfinite(y_min) && std::isfinite(y_max);
}

cv::Mat make_bird_view(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config)
{
  cv::Mat bird(500, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
  const double fx = std::max(
    1.0,
    config.depth_intrinsics.empty() ? 1.0 : config.depth_intrinsics[0]);
  const double px =
    config.depth_intrinsics.size() >= 3 ?
    config.depth_intrinsics[2] : static_cast<double>(config.image_cols) * 0.5;
  const double cyaw = std::cos(state.current_yaw);
  const double syaw = std::sin(state.current_yaw);
  const cv::Point camera_center(bird.cols / 2, bird.rows);
  const cv::Point left_fov(
    static_cast<int>(std::round(camera_center.x + (bird.rows * (0.0 - px) / fx))),
    0);
  const cv::Point right_fov(
    static_cast<int>(std::round(camera_center.x + (bird.rows * (static_cast<double>(config.image_cols) - px) / fx))),
    0);
  cv::line(bird, camera_center, left_fov, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  cv::line(bird, camera_center, right_fov, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

  for (const auto & sample : state.filtered_depth_samples) {
    const double dx = sample.point.x - state.current_position.x;
    const double dy = sample.point.y - state.current_position.y;
    const double x_local = cyaw * dx + syaw * dy;
    const double y_local = -syaw * dx + cyaw * dy;
    cv::Point pixel;
    if (!local_point_to_bird_pixel(x_local, y_local, bird.cols, bird.rows, fx, px, pixel)) {
      continue;
    }
    if (pixel.x < 0 || pixel.x >= bird.cols || pixel.y < 0 || pixel.y >= bird.rows) {
      continue;
    }
    bird.at<cv::Vec3b>(pixel.y, pixel.x) = cv::Vec3b(0, 220, 220);
  }

  auto draw_box = [&](
    const Box3D & box,
    const std::vector<LVdotRuntimeState::DepthSample> * cluster,
    const cv::Scalar & color)
  {
    double x_local = 0.0;
    double y_local = 0.0;
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;
    bool have_cluster_bounds = false;
    if (cluster) {
      have_cluster_bounds = cluster_local_bounds(*cluster, state, x_min, x_max, y_min, y_max);
    }

    if (have_cluster_bounds) {
      x_local = 0.5 * (x_min + x_max);
      y_local = 0.5 * (y_min + y_max);
    } else {
      const double dx = box.x - state.current_position.x;
      const double dy = box.y - state.current_position.y;
      x_local = cyaw * dx + syaw * dy;
      y_local = -syaw * dx + cyaw * dy;
      x_min = x_local - std::max(0.1, box.x_width) * 0.5;
      x_max = x_local + std::max(0.1, box.x_width) * 0.5;
      y_min = y_local - std::max(0.1, box.y_width) * 0.5;
      y_max = y_local + std::max(0.1, box.y_width) * 0.5;
    }

    cv::Point center_pixel;
    if (!local_point_to_bird_pixel(x_local, y_local, bird.cols, bird.rows, fx, px, center_pixel)) {
      return;
    }

    cv::Point min_pixel;
    cv::Point max_pixel;
    if (!local_point_to_bird_pixel(x_max, y_min, bird.cols, bird.rows, fx, px, min_pixel) ||
        !local_point_to_bird_pixel(std::max(0.01, x_min), y_max, bird.cols, bird.rows, fx, px, max_pixel))
    {
      min_pixel = cv::Point(
        center_pixel.x - std::max(2, static_cast<int>(std::round(std::max(0.1, box.y_width) * bird.rows / fx)) / 2),
        center_pixel.y - std::max(2, static_cast<int>(std::round(std::max(0.1, box.x_width))) / 2));
      max_pixel = cv::Point(
        center_pixel.x + std::max(2, static_cast<int>(std::round(std::max(0.1, box.y_width) * bird.rows / fx)) / 2),
        center_pixel.y + std::max(2, static_cast<int>(std::round(std::max(0.1, box.x_width))) / 2));
    }

    const int xmin = std::clamp(std::min(min_pixel.x, max_pixel.x), 0, bird.cols - 1);
    const int ymin = std::clamp(std::min(min_pixel.y, max_pixel.y), 0, bird.rows - 1);
    const int xmax = std::clamp(std::max(min_pixel.x, max_pixel.x), 0, bird.cols - 1);
    const int ymax = std::clamp(std::max(min_pixel.y, max_pixel.y), 0, bird.rows - 1);
    const cv::Rect rect(
      cv::Point(xmin, ymin),
      cv::Point(std::max(xmin + 1, xmax + 1), std::max(ymin + 1, ymax + 1)));
    cv::rectangle(bird, rect, color, 3, cv::LINE_AA);
    cv::circle(bird, center_pixel, 3, color, -1, cv::LINE_AA);
  };

  for (std::size_t i = 0; i < state.filtered_bboxes.size(); ++i) {
    const auto * cluster =
      i < state.filtered_clusters.size() ? &state.filtered_clusters[i] : nullptr;
    draw_box(state.filtered_bboxes[i], cluster, cv::Scalar(0, 0, 255));
  }

  for (const auto & track : state.track_states) {
    cv::Point prev_pixel;
    bool prev_valid = false;
    for (auto it = track.box_history.rbegin(); it != track.box_history.rend(); ++it) {
      const double dx = it->x - state.current_position.x;
      const double dy = it->y - state.current_position.y;
      const double x_local = cyaw * dx + syaw * dy;
      const double y_local = -syaw * dx + cyaw * dy;
      cv::Point pixel;
      if (!local_point_to_bird_pixel(x_local, y_local, bird.cols, bird.rows, fx, px, pixel)) {
        prev_valid = false;
        continue;
      }
      if (prev_valid) {
        cv::line(bird, prev_pixel, pixel, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      }
      prev_pixel = pixel;
      prev_valid = true;
    }

    cv::Point center_pixel;
    if (local_point_to_bird_pixel(
        cyaw * (track.current_box.x - state.current_position.x) + syaw * (track.current_box.y - state.current_position.y),
        -syaw * (track.current_box.x - state.current_position.x) + cyaw * (track.current_box.y - state.current_position.y),
        bird.cols, bird.rows, fx, px, center_pixel))
    {
      const cv::Point velocity_tip(
        center_pixel.x + static_cast<int>(std::round(track.current_box.vy * 15.0)),
        center_pixel.y - static_cast<int>(std::round(track.current_box.vx * 15.0)));
      cv::circle(bird, center_pixel, 5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::line(bird, center_pixel, velocity_tip, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
  }

  cv::resize(bird, bird, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  return bird;
}

}  // namespace

std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::msg::MarkerArray make_box_markers(
  const std::vector<Box3D> & boxes,
  const std::string & frame_id,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = ns;
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);
  marker_array.markers.reserve(boxes.size() * 2);

  for (const auto & box : boxes) {
    const int box_marker_id = marker_id_from_box_id(box.id);
    const double sx = std::max(0.1, box.x_width) * 0.5;
    const double sy = std::max(0.1, box.y_width) * 0.5;
    const double sz = std::max(0.1, box.z_width) * 0.5;
    const double z_min = box.z - sz;
    const double z_max = box.z + sz;

    std::array<geometry_msgs::msg::Point, 8> corners{};
    corners[0].x = box.x - sx; corners[0].y = box.y - sy; corners[0].z = z_min;
    corners[1].x = box.x + sx; corners[1].y = box.y - sy; corners[1].z = z_min;
    corners[2].x = box.x + sx; corners[2].y = box.y + sy; corners[2].z = z_min;
    corners[3].x = box.x - sx; corners[3].y = box.y + sy; corners[3].z = z_min;
    corners[4].x = box.x - sx; corners[4].y = box.y - sy; corners[4].z = z_max;
    corners[5].x = box.x + sx; corners[5].y = box.y - sy; corners[5].z = z_max;
    corners[6].x = box.x + sx; corners[6].y = box.y + sy; corners[6].z = z_max;
    corners[7].x = box.x - sx; corners[7].y = box.y + sy; corners[7].z = z_max;

    static constexpr int kEdges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    visualization_msgs::msg::Marker wire;
    wire.header.frame_id = frame_id;
    wire.header.stamp = stamp;
    wire.ns = ns;
    wire.id = box_marker_id;
    wire.type = visualization_msgs::msg::Marker::LINE_LIST;
    wire.action = visualization_msgs::msg::Marker::ADD;
    wire.pose.orientation.w = 1.0;
    wire.scale.x = 0.05;
    wire.color = box.is_u_map_enhanced ?
      make_color(1.0f, 0.1f, 1.0f, color.a) : color;
    wire.lifetime = rclcpp::Duration::from_seconds(0.25);
    wire.points.reserve(24);
    for (const auto & edge : kEdges) {
      wire.points.push_back(corners[edge[0]]);
      wire.points.push_back(corners[edge[1]]);
    }
    marker_array.markers.push_back(wire);

    visualization_msgs::msg::Marker text;
    text.header = wire.header;
    text.ns = ns + "_label";
    text.id = box_marker_id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = box.x;
    text.pose.position.y = box.y;
    text.pose.position.z = z_max + std::max(0.1, z_max - z_min) * 0.35;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.2;
    text.color = make_color(1.0f, 1.0f, 1.0f, 0.95f);
    std::ostringstream label;
    label << "id=" << box.id;
    if (box.is_human) {
      label << " human";
    }
    if (box.is_dynamic) {
      label << " dyn";
    }
    if (box.is_u_map_enhanced) {
      label << " umap";
    }
    text.text = label.str();
    text.lifetime = wire.lifetime;
    marker_array.markers.push_back(text);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray make_history_markers(
  const std::vector<LVdotRuntimeState::TrackState> & tracks,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "history";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  int marker_id = 1;
  for (const auto & track : tracks) {
    if (track.box_history.size() < 2) {
      continue;
    }

    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = stamp;
    line.ns = "history";
    line.id = marker_id++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.06;
    line.color = make_color(0.9f, 0.9f, 0.1f, 0.95f);
    line.lifetime = rclcpp::Duration::from_seconds(0.3);
    line.points.reserve(track.box_history.size());

    for (auto it = track.box_history.rbegin(); it != track.box_history.rend(); ++it) {
      geometry_msgs::msg::Point point;
      point.x = it->x;
      point.y = it->y;
      point.z = it->z + std::max(0.15, it->z_width * 0.5);
      line.points.push_back(point);
    }
    marker_array.markers.push_back(line);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray make_velocity_markers(
  const std::vector<Box3D> & boxes,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "velocity";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  int marker_id = 1;
  for (const auto & box : boxes) {
    const double speed = std::hypot(box.vx, box.vy);
    if (speed <= 1e-3) {
      continue;
    }

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = stamp;
    arrow.ns = "velocity";
    arrow.id = marker_id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.06;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.14;
    arrow.color = make_color(0.2f, 1.0f, 0.2f, 0.95f);
    arrow.lifetime = rclcpp::Duration::from_seconds(0.3);

    geometry_msgs::msg::Point start;
    start.x = box.x;
    start.y = box.y;
    start.z = box.z + std::max(0.2, box.z_width * 0.55);
    geometry_msgs::msg::Point end = start;
    end.x += box.vx;
    end.y += box.vy;
    arrow.points = {start, end};
    marker_array.markers.push_back(arrow);
  }

  return marker_array;
}

sensor_msgs::msg::PointCloud2 make_pointcloud2(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

sensor_msgs::msg::PointCloud2 make_uniform_colored_pointcloud2(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  uint8_t r,
  uint8_t g,
  uint8_t b)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

  for (const auto & point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    *iter_r = r;
    *iter_g = g;
    *iter_b = b;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }

  return cloud;
}

sensor_msgs::msg::PointCloud2 make_colored_pointcloud2(
  const std::vector<std::vector<LVdotRuntimeState::DepthSample>> & clusters,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  bool use_unique_colors)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;

  std::size_t total_points = 0;
  for (const auto & cluster : clusters) {
    total_points += cluster.size();
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(total_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

  for (std::size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
    uint8_t r = 127;
    uint8_t g = 127;
    uint8_t b = 127;
    if (use_unique_colors) {
      const uint32_t seed = static_cast<uint32_t>(cluster_idx + 1U) * 2654435761U;
      r = static_cast<uint8_t>((seed >> 16) & 0xFF);
      g = static_cast<uint8_t>((seed >> 8) & 0xFF);
      b = static_cast<uint8_t>(seed & 0xFF);
    }

    for (const auto & sample : clusters[cluster_idx]) {
      *iter_x = static_cast<float>(sample.point.x);
      *iter_y = static_cast<float>(sample.point.y);
      *iter_z = static_cast<float>(sample.point.z);
      *iter_r = r;
      *iter_g = g;
      *iter_b = b;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }

  return cloud;
}

std::vector<geometry_msgs::msg::Point> samples_to_points(
  const std::vector<LVdotRuntimeState::DepthSample> & samples)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(samples.size());
  for (const auto & sample : samples) {
    points.push_back(sample.point);
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> clusters_to_points(
  const std::vector<std::vector<LVdotRuntimeState::DepthSample>> & clusters)
{
  std::vector<geometry_msgs::msg::Point> points;
  std::size_t total = 0;
  for (const auto & cluster : clusters) {
    total += cluster.size();
  }
  points.reserve(total);
  for (const auto & cluster : clusters) {
    for (const auto & sample : cluster) {
      points.push_back(sample.point);
    }
  }
  return points;
}

DynamicStaticPointSplit split_points_by_box_flags(
  const std::vector<std::vector<LVdotRuntimeState::DepthSample>> & clusters,
  const std::vector<Box3D> & boxes)
{
  DynamicStaticPointSplit out;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const bool dynamic_cluster =
      i < boxes.size() && (boxes[i].is_dynamic || boxes[i].is_human || boxes[i].is_dynamic_candidate);
    for (const auto & sample : clusters[i]) {
      geometry_msgs::msg::Point p;
      p.x = sample.point.x;
      p.y = sample.point.y;
      p.z = sample.point.z;
      if (dynamic_cluster) {
        out.dynamic_points.push_back(p);
      } else {
        out.static_points.push_back(p);
      }
    }
  }
  return out;
}

std::vector<geometry_msgs::msg::Point> points_in_boxes(
  const std::vector<LVdotRuntimeState::DepthSample> & samples,
  const std::vector<Box3D> & boxes)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & sample : samples) {
    for (const auto & box : boxes) {
      if (point_in_box(sample.point, box)) {
        points.push_back(sample.point);
        break;
      }
    }
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> points_outside_boxes(
  const std::vector<geometry_msgs::msg::Point> & source_points,
  const std::vector<Box3D> & boxes)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(source_points.size());
  for (const auto & point : source_points) {
    bool in_any_box = false;
    for (const auto & box : boxes) {
      if (point_in_box(point, box)) {
        in_any_box = true;
        break;
      }
    }
    if (!in_any_box) {
      points.push_back(point);
    }
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> points_in_boxes(
  const std::vector<geometry_msgs::msg::Point> & source_points,
  const std::vector<Box3D> & boxes)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & point : source_points) {
    for (const auto & box : boxes) {
      if (point_in_box(point, box)) {
        points.push_back(point);
        break;
      }
    }
  }
  return points;
}

std::optional<sensor_msgs::msg::Image> make_detected_color_image(
  const LVdotRuntimeState & state,
  const std::string & fallback_frame_id)
{
  if (!state.latest_color_image) {
    return std::nullopt;
  }

  auto color_bgr = color_image_to_bgr(*state.latest_color_image);
  if (color_bgr.empty()) {
    return std::nullopt;
  }

  draw_detections(color_bgr, state.latest_yolo_detections);
  return make_image_msg(
    color_bgr,
    state.latest_color_image->header.frame_id.empty() ? fallback_frame_id :
    state.latest_color_image->header.frame_id,
    state.latest_color_image->header.stamp,
    "bgr8");
}

std::optional<sensor_msgs::msg::Image> make_detected_depth_image(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::string & fallback_frame_id)
{
  if (!state.latest_depth_image) {
    return std::nullopt;
  }

  auto depth_vis = make_depth_visualization(state, config);
  if (depth_vis.empty()) {
    return std::nullopt;
  }

  draw_detections(depth_vis, state.latest_yolo_detections);
  return make_image_msg(
    depth_vis,
    state.latest_depth_image->header.frame_id.empty() ? fallback_frame_id :
    state.latest_depth_image->header.frame_id,
    state.latest_depth_image->header.stamp,
    "bgr8");
}

std::optional<sensor_msgs::msg::Image> make_detected_u_depth_image(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::string & fallback_frame_id)
{
  if (!state.latest_depth_image) {
    return std::nullopt;
  }

  auto u_map = make_u_depth_map(state, config);
  if (u_map.empty()) {
    return std::nullopt;
  }

  return make_image_msg(
    u_map,
    state.latest_depth_image->header.frame_id.empty() ? fallback_frame_id :
    state.latest_depth_image->header.frame_id,
    state.latest_depth_image->header.stamp,
    "bgr8");
}

std::optional<sensor_msgs::msg::Image> make_bird_view_image(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto bird_view = make_bird_view(state, config);
  if (bird_view.empty()) {
    return std::nullopt;
  }
  return make_image_msg(bird_view, frame_id, stamp, "bgr8");
}

}  // namespace lvdot_ros2
