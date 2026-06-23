/*
 * FILE: classification_filter.cpp
 * -------------------------------------------------------------------------
 * Core port of the ROS1 LV-DOT dynamicDetector::classificationCB() stage.
 */
#include "lvdot_core/classification_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace onboardDetector {

ClassificationOutput runClassification(const ClassificationInput& input)
{
    ClassificationOutput output;
    output.tracks = input.tracks;
    output.dynamicBBoxes.reserve(output.tracks.size());

    for (auto& track : output.tracks) {
        box3D box = track.currentBox;

        if (box.is_human) {
            track.currentBox = box;
            if (!track.boxHistory.empty()) {
                track.boxHistory.front() = box;
            }
            output.dynamicBBoxes.push_back(box);
            continue;
        }

        int curFrameGap = 0;
        if (static_cast<int>(track.clusterHistory.size()) < input.config.skipFrame + 1) {
            curFrameGap = static_cast<int>(track.clusterHistory.size()) - 1;
        } else {
            curFrameGap = input.config.skipFrame;
        }

        int dynaFrames = 0;
        if (static_cast<int>(track.boxHistory.size()) > input.config.forceDynamicCheckRange) {
            for (int j = 1; j < input.config.forceDynamicCheckRange + 1; ++j) {
                if (track.boxHistory[static_cast<std::size_t>(j)].is_dynamic) {
                    ++dynaFrames;
                }
            }
        }
        if (dynaFrames >= input.config.forceDynamicFrames) {
            box.is_dynamic = true;
            track.currentBox = box;
            if (!track.boxHistory.empty()) {
                track.boxHistory.front() = box;
            }
            output.dynamicBBoxes.push_back(box);
            continue;
        }

        if (curFrameGap > 0 &&
            static_cast<std::size_t>(curFrameGap) < track.clusterHistory.size() &&
            !track.clusterHistory.empty()) {
            const auto& currPc = track.clusterHistory.front();
            const auto& prevPc = track.clusterHistory[static_cast<std::size_t>(curFrameGap)];

            Eigen::Vector3d Vcur(0.0, 0.0, 0.0);
            Eigen::Vector3d Vbox(0.0, 0.0, 0.0);
            Eigen::Vector3d Vkf(0.0, 0.0, 0.0);
            int numPoints = static_cast<int>(currPc.size());
            int votes = 0;

            Vbox(0) = (track.boxHistory[0].x - track.boxHistory[static_cast<std::size_t>(curFrameGap)].x) /
                      (input.config.dt * static_cast<double>(curFrameGap));
            Vbox(1) = (track.boxHistory[0].y - track.boxHistory[static_cast<std::size_t>(curFrameGap)].y) /
                      (input.config.dt * static_cast<double>(curFrameGap));
            Vbox(2) = (track.boxHistory[0].z - track.boxHistory[static_cast<std::size_t>(curFrameGap)].z) /
                      (input.config.dt * static_cast<double>(curFrameGap));
            Vkf(0) = track.boxHistory[0].Vx;
            Vkf(1) = track.boxHistory[0].Vy;

            for (const auto& currPoint : currPc) {
                double minDist = 2.0;
                Eigen::Vector3d nearestVect = Eigen::Vector3d::Zero();
                bool found = false;
                for (const auto& prevPoint : prevPc) {
                    const double dist = (currPoint.point - prevPoint.point).norm();
                    if (std::abs(dist) < minDist) {
                        minDist = dist;
                        nearestVect = currPoint.point - prevPoint.point;
                        found = true;
                    }
                }
                if (!found) {
                    continue;
                }

                Vcur = nearestVect / (input.config.dt * static_cast<double>(curFrameGap));
                Vcur(2) = 0.0;
                const double denom = Vcur.norm() * Vbox.norm();
                const double velSim = denom > 1e-9 ?
                    Vcur.dot(Vbox) / denom :
                    std::numeric_limits<double>::quiet_NaN();

                if (std::isfinite(velSim) && velSim < 0.0) {
                    --numPoints;
                } else if (Vcur.norm() > input.config.dynamicVelocityThreshold) {
                    ++votes;
                }
            }

            const double voteRatio = numPoints > 0 ?
                static_cast<double>(votes) / static_cast<double>(numPoints) : 0.0;
            const double velNorm = Vkf.norm();

            if (voteRatio >= input.config.dynamicVotingThreshold &&
                velNorm >= input.config.dynamicVelocityThreshold) {
                box.is_dynamic_candidate = true;
                int dynaConsistCount = 0;
                if (static_cast<int>(track.boxHistory.size()) >= input.config.dynamicConsistencyThreshold) {
                    for (int j = 0; j < input.config.dynamicConsistencyThreshold; ++j) {
                        const auto& histBox = track.boxHistory[static_cast<std::size_t>(j)];
                        if (histBox.is_dynamic_candidate || histBox.is_human || histBox.is_dynamic) {
                            ++dynaConsistCount;
                        }
                    }
                }
                // ROS2 parity tuning: avoid requiring 100% consecutive dynamic flags.
                // In practice, sensor jitter makes strict equality too brittle and
                // suppresses most dynamic outputs. Use a ratio-based pass criterion.
                const int consistencyThreshold = input.config.dynamicConsistencyThreshold;
                const int minConsistentFrames = std::max(
                    1, static_cast<int>(std::ceil(0.6 * static_cast<double>(consistencyThreshold))));
                if (dynaConsistCount >= minConsistentFrames) {
                    box.is_dynamic = true;
                    output.dynamicBBoxes.push_back(box);
                }
            }
        }

        track.currentBox = box;
        if (!track.boxHistory.empty()) {
            track.boxHistory.front() = box;
        }
    }

    if (input.config.constrainSize && !input.config.targetObjectSize.empty()) {
        std::vector<box3D> constrained;
        constrained.reserve(output.dynamicBBoxes.size());
        for (const auto& ob : output.dynamicBBoxes) {
            bool findMatch = false;
            for (const auto& targetSize : input.config.targetObjectSize) {
                const double xdiff = std::abs(ob.x_width - targetSize(0));
                const double ydiff = std::abs(ob.y_width - targetSize(1));
                const double zdiff = std::abs(ob.z_width - targetSize(2));
                if (xdiff < 0.8 && ydiff < 0.8 && zdiff < 1.0) {
                    findMatch = true;
                    break;
                }
            }
            if (findMatch) {
                constrained.push_back(ob);
            } else {
                output.dynamicRejectedBySize += 1;
            }
        }
        output.dynamicBBoxes = std::move(constrained);
    }

    return output;
}

}  // namespace onboardDetector
