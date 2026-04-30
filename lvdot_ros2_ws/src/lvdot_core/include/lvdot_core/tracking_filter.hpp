/*
 * FILE: tracking_filter.hpp
 * -------------------------------------------------------------------------
 * Core port of the ROS1 LV-DOT dynamicDetector tracking path:
 *   - genFeatHelper()
 *   - findBestMatch()
 *   - kalmanFilterAndUpdateHist()
 *
 * The ROS2 wrapper should convert runtime state into these plain C++
 * structures and call this module, instead of reimplementing association
 * and Kalman-history updates in the node.
 */
#ifndef LVDOT_CORE_TRACKING_FILTER_HPP
#define LVDOT_CORE_TRACKING_FILTER_HPP

#include <array>
#include <deque>
#include <vector>

#include <Eigen/Eigen>

#include "lvdot_core/box3d.hpp"
#include "lvdot_core/fusion_filter.hpp"
#include "lvdot_core/kalman_filter.hpp"

namespace onboardDetector {

struct TrackingConfig
{
    double dt{0.033};
    double maxMatchRange{0.5};
    double maxMatchSizeRange{0.5};
    Eigen::VectorXd featureWeights{Eigen::VectorXd::Zero(9)};
    int histSize{100};
    int fixSizeHistThresh{10};
    double fixSizeDimThresh{0.4};
    double eP{0.25};
    double eQPos{0.01};
    double eQVel{0.05};
    double eQAcc{0.05};
    double eRPos{0.04};
    double eRVel{0.3};
    double eRAcc{0.6};
    int kfAvgFrames{10};
    int maxUnmatchedFrames{0};
};

struct TrackState
{
    kalman_filter kf;
    bool kf_initialized{false};
    std::array<double, 6> filter_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool filter_initialized{false};
    box3D currentBox;
    Eigen::Vector3d currentCenter{Eigen::Vector3d::Zero()};
    Eigen::Vector3d currentStd{Eigen::Vector3d::Zero()};
    std::deque<box3D> boxHistory;
    std::deque<std::vector<ClusterPoint>> clusterHistory;
    std::deque<Eigen::Vector3d> centerHistory;
    std::deque<Eigen::Vector3d> stdHistory;
    bool matchedInFrame{false};
    std::size_t age{0};
    std::size_t missedFrames{0};
};

struct TrackingInput
{
    std::vector<box3D> filteredBBoxes;
    std::vector<std::vector<ClusterPoint>> filteredPcClusters;
    std::vector<Eigen::Vector3d> filteredPcClusterCenters;
    std::vector<Eigen::Vector3d> filteredPcClusterStds;
    std::vector<TrackState> tracks;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    TrackingConfig config;
};

struct TrackingOutput
{
    std::vector<TrackState> tracks;
    std::vector<box3D> trackedBBoxes;
    std::size_t fixedSizeCount{0};
};

TrackingOutput runTracking(const TrackingInput& input);

}  // namespace onboardDetector

#endif  // LVDOT_CORE_TRACKING_FILTER_HPP
