/*
 * FILE: classification_filter.hpp
 * -------------------------------------------------------------------------
 * Core port of the ROS1 LV-DOT dynamicDetector::classificationCB() stage.
 */
#ifndef LVDOT_CORE_CLASSIFICATION_FILTER_HPP
#define LVDOT_CORE_CLASSIFICATION_FILTER_HPP

#include <vector>

#include <Eigen/Eigen>

#include "lvdot_core/box3d.hpp"
#include "lvdot_core/tracking_filter.hpp"

namespace onboardDetector {

struct ClassificationConfig
{
    double dt{0.033};
    int skipFrame{5};
    double dynamicVelocityThreshold{0.2};
    double dynamicVotingThreshold{0.8};
    int forceDynamicFrames{10};
    int forceDynamicCheckRange{30};
    int dynamicConsistencyThreshold{15};
    bool constrainSize{true};
    std::vector<Eigen::Vector3d> targetObjectSize;
};

struct ClassificationInput
{
    std::vector<TrackState> tracks;
    ClassificationConfig config;
};

struct ClassificationOutput
{
    std::vector<TrackState> tracks;
    std::vector<box3D> dynamicBBoxes;
    std::size_t dynamicRejectedBySize{0};
};

ClassificationOutput runClassification(const ClassificationInput& input);

}  // namespace onboardDetector

#endif  // LVDOT_CORE_CLASSIFICATION_FILTER_HPP
