/*
 * FILE: lidar_preprocess.hpp
 * -------------------------------------------------------------------------
 * ROS-free LiDAR preprocessing shared by the ROS2 wrapper:
 *   - body-frame point projection into world-frame ClusterPoint samples
 *   - simple stride downsampling used before onboardDetector::lidarDetector
 *
 * This keeps PointCloud2 decoding in the wrapper, but moves the geometric
 * preprocessing into lvdot_core so the wrapper does less algorithmic work.
 */
#ifndef LVDOT_CORE_LIDAR_PREPROCESS_HPP
#define LVDOT_CORE_LIDAR_PREPROCESS_HPP

#include <vector>

#include <Eigen/Eigen>

#include "lvdot_core/fusion_filter.hpp"

namespace onboardDetector {

struct LidarPreprocessConfig
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d orientationBody{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d sensorRotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d sensorTranslation{Eigen::Vector3d::Zero()};
    Eigen::Vector3d localSensorRange{Eigen::Vector3d(5.0, 5.0, 5.0)};
    double groundHeight{0.0};
    double roofHeight{3.0};
    int downsampleThreshold{1000};
    int gaussianDownsampleRate{2};
};

std::vector<ClusterPoint> projectLidarPoints(
    const std::vector<Eigen::Vector3d>& bodyPoints,
    const LidarPreprocessConfig& config);

std::vector<ClusterPoint> downsampleLidarPoints(
    const std::vector<ClusterPoint>& points,
    const LidarPreprocessConfig& config);

}  // namespace onboardDetector

#endif  // LVDOT_CORE_LIDAR_PREPROCESS_HPP
