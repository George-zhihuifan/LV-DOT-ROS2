/*
 * FILE: detection_filter.hpp
 * -------------------------------------------------------------------------
 * Core port of the ROS1 LV-DOT depth detection chain:
 *   - projectDepthImage()
 *   - filterPoints()
 *   - voxelFilter()
 *   - clusterPointsAndBBoxes()
 *   - uvDetect()
 *
 * The ROS2 wrapper should convert sensor messages into the plain inputs here
 * and then write the plain outputs back into runtime_state.
 */
#ifndef LVDOT_CORE_DETECTION_FILTER_HPP
#define LVDOT_CORE_DETECTION_FILTER_HPP

#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "lvdot_core/box3d.hpp"
#include "lvdot_core/fusion_filter.hpp"

namespace onboardDetector {

struct DetectionConfig
{
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double depthScale{1000.0};
    double depthMinValue{0.2};
    double depthMaxValue{5.0};
    double raycastMaxLength{5.0};
    int depthFilterMargin{0};
    int skipPixel{1};
    double groundHeight{0.0};
    double roofHeight{3.0};
    double voxelOccThresh{10.0};
    int dbMinPointsCluster{18};
    double dbEpsilon{0.3};
    Eigen::Vector3d localSensorRange{5.0, 5.0, 5.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d positionDepth{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d orientationDepth{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d minObjectSize{0.0, 0.0, 0.0};
    Eigen::Vector3d maxObjectSize{3.0, 3.0, 2.0};
    double depthBBoxQuantileXYLow{0.08};
    double depthBBoxQuantileXYHigh{0.92};
    double depthBBoxQuantileZLow{0.02};
    double depthBBoxQuantileZHigh{0.98};
    int depthBBoxQuantileMinPoints{30};
    double depthBBoxPaddingXY{0.05};
    double depthBBoxPaddingZ{0.03};

    int uMapRowDownsample{4};
    float uMapColScale{0.5f};
    float uMapThresholdPoint{3.0f};
    float uMapThresholdLine{2.0f};
    int uMapMinLengthLine{6};
};

struct DetectionInput
{
    cv::Mat depthImageMm;
    DetectionConfig config;
};

struct DetectionOutput
{
    std::vector<ClusterPoint> projectedDepthPoints;
    std::vector<ClusterPoint> filteredDepthPoints;
    std::vector<box3D> dbBBoxes;
    std::vector<std::vector<ClusterPoint>> dbClusters;
    std::vector<Eigen::Vector3d> dbClusterCenters;
    std::vector<Eigen::Vector3d> dbClusterStds;
    std::vector<box3D> uvBBoxes;
    cv::Mat depthShow;
    cv::Mat uMapShow;
    cv::Mat birdView;
};

DetectionOutput runDetection(const DetectionInput& input);

}  // namespace onboardDetector

#endif  // LVDOT_CORE_DETECTION_FILTER_HPP
