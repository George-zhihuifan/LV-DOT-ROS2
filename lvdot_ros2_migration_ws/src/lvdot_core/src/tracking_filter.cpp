/*
 * FILE: tracking_filter.cpp
 * -------------------------------------------------------------------------
 * Core port of the ROS1 LV-DOT dynamicDetector tracking path:
 *   - genFeatHelper()
 *   - findBestMatch()
 *   - kalmanFilterAndUpdateHist()
 */
#include "lvdot_core/tracking_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace onboardDetector {

namespace {

using FeatureVector = Eigen::VectorXd;

void kalmanFilterMatrixAcc(
    const box3D& currDetectedBBox,
    const TrackingConfig& config,
    Eigen::MatrixXd& states,
    Eigen::MatrixXd& A,
    Eigen::MatrixXd& B,
    Eigen::MatrixXd& H,
    Eigen::MatrixXd& P,
    Eigen::MatrixXd& Q,
    Eigen::MatrixXd& R)
{
    states.resize(6, 1);
    states(0) = currDetectedBBox.x;
    states(1) = currDetectedBBox.y;
    states(2) = 0.0;
    states(3) = 0.0;
    states(4) = 0.0;
    states(5) = 0.0;

    A.resize(6, 6);
    A << 1, 0, config.dt, 0, 0.5 * std::pow(config.dt, 2), 0,
         0, 1, 0, config.dt, 0, 0.5 * std::pow(config.dt, 2),
         0, 0, 1, 0, config.dt, 0,
         0, 0, 0, 1, 0, config.dt,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    B = Eigen::MatrixXd::Zero(6, 6);
    H = Eigen::MatrixXd::Identity(6, 6);
    P = Eigen::MatrixXd::Identity(6, 6) * config.eP;
    Q = Eigen::MatrixXd::Identity(6, 6);
    Q(0, 0) *= config.eQPos;
    Q(1, 1) *= config.eQPos;
    Q(2, 2) *= config.eQVel;
    Q(3, 3) *= config.eQVel;
    Q(4, 4) *= config.eQAcc;
    Q(5, 5) *= config.eQAcc;
    R = Eigen::MatrixXd::Identity(6, 6);
    R(0, 0) *= config.eRPos;
    R(1, 1) *= config.eRPos;
    R(2, 2) *= config.eRVel;
    R(3, 3) *= config.eRVel;
    R(4, 4) *= config.eRAcc;
    R(5, 5) *= config.eRAcc;
}

void genFeatHelper(
    const std::vector<box3D>& boxes,
    const std::vector<Eigen::Vector3d>& pcCenters,
    const Eigen::Vector3d& position,
    const TrackingConfig& config,
    std::vector<FeatureVector>& features)
{
    features.resize(boxes.size());
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        FeatureVector feature = FeatureVector::Zero(9);
        feature(0) = (boxes[i].x - position(0)) * config.featureWeights(0);
        feature(1) = (boxes[i].y - position(1)) * config.featureWeights(1);
        feature(2) = (boxes[i].z - position(2)) * config.featureWeights(2);
        feature(3) = boxes[i].x_width * config.featureWeights(3);
        feature(4) = boxes[i].y_width * config.featureWeights(4);
        feature(5) = boxes[i].z_width * config.featureWeights(5);
        if (i < pcCenters.size()) {
            feature(6) = pcCenters[i](0) * config.featureWeights(6);
            feature(7) = pcCenters[i](1) * config.featureWeights(7);
            feature(8) = pcCenters[i](2) * config.featureWeights(8);
        }
        for (int j = 0; j < feature.size(); ++j) {
            if (std::isnan(feature(j)) || std::isinf(feature(j))) {
                feature(j) = 0.0;
            }
        }
        features[i] = feature;
    }
}

void getPrevBBoxes(
    const std::vector<TrackState>& tracks,
    std::vector<box3D>& prevBoxes,
    std::vector<Eigen::Vector3d>& prevPcCenters)
{
    prevBoxes.clear();
    prevPcCenters.clear();
    prevBoxes.reserve(tracks.size());
    prevPcCenters.reserve(tracks.size());
    for (const auto& track : tracks) {
        if (track.boxHistory.empty()) {
            continue;
        }
        prevBoxes.push_back(track.boxHistory.front());
        if (!track.centerHistory.empty()) {
            prevPcCenters.push_back(track.centerHistory.front());
        } else {
            prevPcCenters.push_back(track.currentCenter);
        }
    }
}

void linearProp(
    const std::vector<TrackState>& tracks,
    double dt,
    std::vector<box3D>& propedBBoxes,
    std::vector<Eigen::Vector3d>& propedPcCenters)
{
    propedBBoxes.clear();
    propedPcCenters.clear();
    propedBBoxes.reserve(tracks.size());
    propedPcCenters.reserve(tracks.size());
    for (const auto& track : tracks) {
        if (track.boxHistory.empty()) {
            continue;
        }
        box3D propedBBox = track.boxHistory.front();
        propedBBox.x += propedBBox.Vx * dt;
        propedBBox.y += propedBBox.Vy * dt;
        propedBBoxes.push_back(propedBBox);

        Eigen::Vector3d propedPcCenter = track.centerHistory.empty() ?
            track.currentCenter : track.centerHistory.front();
        propedPcCenter(0) += propedBBox.Vx * dt;
        propedPcCenter(1) += propedBBox.Vy * dt;
        propedPcCenters.push_back(propedPcCenter);
    }
}

void findBestMatch(
    const std::vector<box3D>& currBBoxes,
    const std::vector<box3D>& /*prevBBoxes*/,
    const std::vector<FeatureVector>& prevBBoxesFeat,
    const std::vector<box3D>& propedBBoxes,
    const std::vector<FeatureVector>& propedBBoxesFeat,
    const std::vector<FeatureVector>& currBBoxesFeat,
    const TrackingConfig& config,
    std::vector<int>& bestMatch)
{
    const int numObjs = static_cast<int>(currBBoxes.size());
    bestMatch.assign(numObjs, -1);

    struct Candidate
    {
        int curr_idx;
        int prev_idx;
        double sim;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<std::size_t>(numObjs) * propedBBoxes.size());

    for (int i = 0; i < numObjs; ++i) {
        const box3D& currBBox = currBBoxes[static_cast<std::size_t>(i)];
        for (std::size_t j = 0; j < propedBBoxes.size(); ++j) {
            const box3D& propedBBox = propedBBoxes[j];
            const double propedWidth = std::max(propedBBox.x_width, propedBBox.y_width);
            const double currWidth = std::max(currBBox.x_width, currBBox.y_width);
            if (std::abs(propedWidth - currWidth) >= config.maxMatchSizeRange) {
                continue;
            }
            const double planar = std::sqrt(
                std::pow(propedBBox.x - currBBox.x, 2) +
                std::pow(propedBBox.y - currBBox.y, 2));
            if (planar >= config.maxMatchRange) {
                continue;
            }
            const double prevNorm = prevBBoxesFeat[j].norm() * currBBoxesFeat[i].norm();
            const double propNorm = propedBBoxesFeat[j].norm() * currBBoxesFeat[i].norm();
            const double simPrev = prevNorm > 1e-9 ?
                prevBBoxesFeat[j].dot(currBBoxesFeat[i]) / prevNorm : 0.0;
            const double simProped = propNorm > 1e-9 ?
                propedBBoxesFeat[j].dot(currBBoxesFeat[i]) / propNorm : 0.0;
            double sim = config.simPrevWeight * simPrev + config.simPropedWeight * simProped;
            if (config.adaptiveSimilarityWeight) {
                const double distanceNorm = std::max(config.similarityDistanceNorm, 1e-6);
                const double closeness = std::clamp(1.0 - planar / distanceNorm, 0.0, 1.0);
                const double adaptivePrevWeight = config.simPrevWeight * (1.5 - closeness);
                const double adaptivePropedWeight = config.simPropedWeight * (0.5 + closeness);
                sim = adaptivePrevWeight * simPrev + adaptivePropedWeight * simProped;
            }
            if (sim < config.minMatchSimilarity) {
                continue;
            }
            candidates.push_back(Candidate{i, static_cast<int>(j), sim});
        }
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [](const Candidate& lhs, const Candidate& rhs) { return lhs.sim > rhs.sim; });

    std::vector<bool> curr_used(static_cast<std::size_t>(numObjs), false);
    std::vector<bool> prev_used(propedBBoxes.size(), false);
    for (const Candidate& c : candidates) {
        if (curr_used[static_cast<std::size_t>(c.curr_idx)] ||
            prev_used[static_cast<std::size_t>(c.prev_idx)]) {
            continue;
        }
        bestMatch[static_cast<std::size_t>(c.curr_idx)] = c.prev_idx;
        curr_used[static_cast<std::size_t>(c.curr_idx)] = true;
        prev_used[static_cast<std::size_t>(c.prev_idx)] = true;
    }
}

Eigen::MatrixXd getKalmanObservationAcc(
    const box3D& currDetectedBBox,
    const TrackState& track,
    const TrackingConfig& config)
{
    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(6, 1);
    Z(0) = currDetectedBBox.x;
    Z(1) = currDetectedBBox.y;

    int k = std::max(1, config.kfAvgFrames);
    const int historySize = static_cast<int>(track.boxHistory.size());
    if (historySize < k) {
        k = historySize;
    }
    if (k <= 0) {
        return Z;
    }
    const box3D& prevMatchBBox = track.boxHistory[static_cast<std::size_t>(k - 1)];
    const double horizon = std::max(config.dt * static_cast<double>(k), 1e-6);
    Z(2) = (currDetectedBBox.x - prevMatchBBox.x) / horizon;
    Z(3) = (currDetectedBBox.y - prevMatchBBox.y) / horizon;
    Z(4) = (Z(2) - prevMatchBBox.Vx) / horizon;
    Z(5) = (Z(3) - prevMatchBBox.Vy) / horizon;
    return Z;
}

}  // namespace

TrackingOutput runTracking(const TrackingInput& input)
{
    TrackingOutput output;
    const int numObjs = static_cast<int>(input.filteredBBoxes.size());
    if (numObjs <= 0) {
        return output;
    }

    double nextTrackId = 0.0;
    for (const auto& prevTrack : input.tracks) {
        nextTrackId = std::max(nextTrackId, prevTrack.currentBox.id + 1.0);
    }

    if (input.tracks.empty()) {
        output.tracks.reserve(input.filteredBBoxes.size());
        output.trackedBBoxes.reserve(input.filteredBBoxes.size());
        for (std::size_t i = 0; i < input.filteredBBoxes.size(); ++i) {
            TrackState track;
            track.currentBox = input.filteredBBoxes[i];
            track.currentCenter = i < input.filteredPcClusterCenters.size() ?
                input.filteredPcClusterCenters[i] : Eigen::Vector3d(track.currentBox.x, track.currentBox.y, track.currentBox.z);
            track.currentStd = i < input.filteredPcClusterStds.size() ?
                input.filteredPcClusterStds[i] : Eigen::Vector3d::Zero();
            track.boxHistory.push_back(track.currentBox);
            if (i < input.filteredPcClusters.size()) {
                track.clusterHistory.push_back(input.filteredPcClusters[i]);
            } else {
                track.clusterHistory.push_back({});
            }
            track.centerHistory.push_back(track.currentCenter);
            track.stdHistory.push_back(track.currentStd);
            Eigen::MatrixXd states, A, B, H, P, Q, R;
            kalmanFilterMatrixAcc(track.currentBox, input.config, states, A, B, H, P, Q, R);
            track.kf.setup(states, A, B, H, P, Q, R);
            track.kf_initialized = true;
            track.filter_initialized = true;
            track.filter_state = {track.currentBox.x, track.currentBox.y, 0.0, 0.0, 0.0, 0.0};
            track.age = 1;
            track.currentBox.id = nextTrackId++;
            output.trackedBBoxes.push_back(track.currentBox);
            output.tracks.push_back(track);
        }
        return output;
    }

    std::vector<box3D> prevBBoxes;
    std::vector<Eigen::Vector3d> prevPcCenters;
    std::vector<FeatureVector> prevBBoxesFeat;
    std::vector<box3D> propedBBoxes;
    std::vector<Eigen::Vector3d> propedPcCenters;
    std::vector<FeatureVector> propedBBoxesFeat;
    std::vector<FeatureVector> currBBoxesFeat;

    genFeatHelper(input.filteredBBoxes, input.filteredPcClusterCenters, input.position, input.config, currBBoxesFeat);
    getPrevBBoxes(input.tracks, prevBBoxes, prevPcCenters);
    genFeatHelper(prevBBoxes, prevPcCenters, input.position, input.config, prevBBoxesFeat);
    linearProp(input.tracks, input.config.dt, propedBBoxes, propedPcCenters);
    genFeatHelper(propedBBoxes, propedPcCenters, input.position, input.config, propedBBoxesFeat);

    std::vector<int> bestMatch;
    findBestMatch(
        input.filteredBBoxes,
        prevBBoxes,
        prevBBoxesFeat,
        propedBBoxes,
        propedBBoxesFeat,
        currBBoxesFeat,
        input.config,
        bestMatch);

    std::vector<std::deque<box3D>> boxHistTemp;
    std::vector<std::deque<std::vector<ClusterPoint>>> pcHistTemp;
    std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
    std::vector<std::deque<Eigen::Vector3d>> pcStdHistTemp;
    std::vector<kalman_filter> filtersTemp;
    std::vector<std::array<double, 6>> filterStateTemp;
    std::vector<bool> filterInitializedTemp;
    std::vector<std::size_t> ageTemp;
    std::vector<std::size_t> missedFramesTemp;
    std::vector<box3D> trackedBBoxesTemp;
    std::vector<TrackState> tracksTemp;

    std::deque<box3D> newSingleBoxHist;
    std::deque<std::vector<ClusterPoint>> newSinglePcHist;
    std::deque<Eigen::Vector3d> newSinglePcCenterHist;
    std::deque<Eigen::Vector3d> newSinglePcStdHist;

    boxHistTemp.reserve(static_cast<std::size_t>(numObjs));
    pcHistTemp.reserve(static_cast<std::size_t>(numObjs));
    pcCenterHistTemp.reserve(static_cast<std::size_t>(numObjs));
    pcStdHistTemp.reserve(static_cast<std::size_t>(numObjs));
    filtersTemp.reserve(static_cast<std::size_t>(numObjs));
    filterStateTemp.reserve(static_cast<std::size_t>(numObjs));
    filterInitializedTemp.reserve(static_cast<std::size_t>(numObjs));
    ageTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    missedFramesTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    trackedBBoxesTemp.reserve(static_cast<std::size_t>(numObjs));
    tracksTemp.reserve(static_cast<std::size_t>(numObjs));

    for (int i = 0; i < numObjs; ++i) {
        box3D newEstimatedBBox;
        kalman_filter filter;
        std::array<double, 6> filterState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        bool filterInitialized = false;
        const box3D& currDetectedBBox = input.filteredBBoxes[static_cast<std::size_t>(i)];

        if (bestMatch[i] >= 0) {
            const std::size_t matchIdx = static_cast<std::size_t>(bestMatch[i]);
            boxHistTemp.push_back(input.tracks[matchIdx].boxHistory);
            pcHistTemp.push_back(input.tracks[matchIdx].clusterHistory);
            pcCenterHistTemp.push_back(input.tracks[matchIdx].centerHistory);
            pcStdHistTemp.push_back(input.tracks[matchIdx].stdHistory);
            filter = input.tracks[matchIdx].kf;
            const Eigen::MatrixXd Z = getKalmanObservationAcc(currDetectedBBox, input.tracks[matchIdx], input.config);
            filter.estimate(Z, Eigen::MatrixXd::Zero(6, 1));

            newEstimatedBBox.x = filter.output(0);
            newEstimatedBBox.y = filter.output(1);
            newEstimatedBBox.z = currDetectedBBox.z;
            newEstimatedBBox.Vx = filter.output(2);
            newEstimatedBBox.Vy = filter.output(3);
            newEstimatedBBox.Ax = filter.output(4);
            newEstimatedBBox.Ay = filter.output(5);
            newEstimatedBBox.x_width = currDetectedBBox.x_width;
            newEstimatedBBox.y_width = currDetectedBBox.y_width;
            newEstimatedBBox.z_width = currDetectedBBox.z_width;
            newEstimatedBBox.is_dynamic = currDetectedBBox.is_dynamic;
            newEstimatedBBox.is_human = currDetectedBBox.is_human;
            newEstimatedBBox.is_dynamic_candidate = currDetectedBBox.is_dynamic_candidate;
            newEstimatedBBox.is_u_map_enhanced = currDetectedBBox.is_u_map_enhanced;
            newEstimatedBBox.id = input.tracks[matchIdx].currentBox.id;

            filterState = {
                newEstimatedBBox.x,
                newEstimatedBBox.y,
                newEstimatedBBox.Vx,
                newEstimatedBBox.Vy,
                newEstimatedBBox.Ax,
                newEstimatedBBox.Ay
            };
            filterInitialized = true;
            ageTemp.push_back(input.tracks[matchIdx].age + 1);
            missedFramesTemp.push_back(0);
        } else {
            boxHistTemp.push_back(newSingleBoxHist);
            pcHistTemp.push_back(newSinglePcHist);
            pcCenterHistTemp.push_back(newSinglePcCenterHist);
            pcStdHistTemp.push_back(newSinglePcStdHist);

            Eigen::MatrixXd states, A, B, H, P, Q, R;
            kalmanFilterMatrixAcc(currDetectedBBox, input.config, states, A, B, H, P, Q, R);
            filter.setup(states, A, B, H, P, Q, R);
            newEstimatedBBox = currDetectedBBox;
            newEstimatedBBox.id = nextTrackId++;
            filterState = {
                newEstimatedBBox.x,
                newEstimatedBBox.y,
                0.0,
                0.0,
                0.0,
                0.0
            };
            filterInitialized = true;
            ageTemp.push_back(1);
            missedFramesTemp.push_back(0);
        }

        if (static_cast<int>(boxHistTemp[static_cast<std::size_t>(i)].size()) == input.config.histSize) {
            boxHistTemp[static_cast<std::size_t>(i)].pop_back();
            pcHistTemp[static_cast<std::size_t>(i)].pop_back();
            pcCenterHistTemp[static_cast<std::size_t>(i)].pop_back();
            pcStdHistTemp[static_cast<std::size_t>(i)].pop_back();
        }

        boxHistTemp[static_cast<std::size_t>(i)].push_front(newEstimatedBBox);
        if (static_cast<std::size_t>(i) < input.filteredPcClusters.size()) {
            pcHistTemp[static_cast<std::size_t>(i)].push_front(input.filteredPcClusters[static_cast<std::size_t>(i)]);
        } else {
            pcHistTemp[static_cast<std::size_t>(i)].push_front({});
        }
        if (static_cast<std::size_t>(i) < input.filteredPcClusterCenters.size()) {
            pcCenterHistTemp[static_cast<std::size_t>(i)].push_front(input.filteredPcClusterCenters[static_cast<std::size_t>(i)]);
        } else {
            pcCenterHistTemp[static_cast<std::size_t>(i)].push_front(Eigen::Vector3d(newEstimatedBBox.x, newEstimatedBBox.y, newEstimatedBBox.z));
        }
        if (static_cast<std::size_t>(i) < input.filteredPcClusterStds.size()) {
            pcStdHistTemp[static_cast<std::size_t>(i)].push_front(input.filteredPcClusterStds[static_cast<std::size_t>(i)]);
        } else {
            pcStdHistTemp[static_cast<std::size_t>(i)].push_front(Eigen::Vector3d::Zero());
        }

        trackedBBoxesTemp.push_back(newEstimatedBBox);
        filtersTemp.push_back(filter);
        filterStateTemp.push_back(filterState);
        filterInitializedTemp.push_back(filterInitialized);
    }

    if (input.config.maxUnmatchedFrames > 0) {
        std::vector<bool> matchedTracks(input.tracks.size(), false);
        for (int match : bestMatch) {
            if (match >= 0 && static_cast<std::size_t>(match) < matchedTracks.size()) {
                matchedTracks[static_cast<std::size_t>(match)] = true;
            }
        }

        for (std::size_t i = 0; i < input.tracks.size(); ++i) {
            if (matchedTracks[i]) {
                continue;
            }

            const auto& prevTrack = input.tracks[i];
            const std::size_t nextMissed = prevTrack.missedFrames + 1;
            if (nextMissed > static_cast<std::size_t>(input.config.maxUnmatchedFrames)) {
                continue;
            }
            if (prevTrack.boxHistory.empty()) {
                continue;
            }

            box3D propagatedBox = prevTrack.boxHistory.front();
            propagatedBox.x += propagatedBox.Vx * input.config.dt;
            propagatedBox.y += propagatedBox.Vy * input.config.dt;
            propagatedBox.is_estimated = true;
            propagatedBox.id = prevTrack.currentBox.id;

            std::deque<box3D> boxHistory = prevTrack.boxHistory;
            std::deque<std::vector<ClusterPoint>> clusterHistory = prevTrack.clusterHistory;
            std::deque<Eigen::Vector3d> centerHistory = prevTrack.centerHistory;
            std::deque<Eigen::Vector3d> stdHistory = prevTrack.stdHistory;

            if (static_cast<int>(boxHistory.size()) == input.config.histSize) {
                boxHistory.pop_back();
            }
            if (static_cast<int>(clusterHistory.size()) == input.config.histSize) {
                clusterHistory.pop_back();
            }
            if (static_cast<int>(centerHistory.size()) == input.config.histSize) {
                centerHistory.pop_back();
            }
            if (static_cast<int>(stdHistory.size()) == input.config.histSize) {
                stdHistory.pop_back();
            }

            boxHistory.push_front(propagatedBox);
            if (!prevTrack.clusterHistory.empty()) {
                clusterHistory.push_front(prevTrack.clusterHistory.front());
            } else {
                clusterHistory.push_front({});
            }
            if (!prevTrack.centerHistory.empty()) {
                centerHistory.push_front(prevTrack.centerHistory.front());
            } else {
                centerHistory.push_front(Eigen::Vector3d(propagatedBox.x, propagatedBox.y, propagatedBox.z));
            }
            if (!prevTrack.stdHistory.empty()) {
                stdHistory.push_front(prevTrack.stdHistory.front());
            } else {
                stdHistory.push_front(Eigen::Vector3d::Zero());
            }

            boxHistTemp.push_back(std::move(boxHistory));
            pcHistTemp.push_back(std::move(clusterHistory));
            pcCenterHistTemp.push_back(std::move(centerHistory));
            pcStdHistTemp.push_back(std::move(stdHistory));
            filtersTemp.push_back(prevTrack.kf);
            filterStateTemp.push_back(prevTrack.filter_state);
            filterInitializedTemp.push_back(prevTrack.filter_initialized);
            ageTemp.push_back(prevTrack.age + 1);
            missedFramesTemp.push_back(nextMissed);
            trackedBBoxesTemp.push_back(propagatedBox);
        }
    }

    if (!boxHistTemp.empty()) {
        for (std::size_t i = 0; i < trackedBBoxesTemp.size(); ++i) {
            if (static_cast<int>(boxHistTemp[i].size()) >= input.config.fixSizeHistThresh && boxHistTemp[i].size() >= 2) {
                if ((std::abs(trackedBBoxesTemp[i].x_width - boxHistTemp[i][1].x_width) /
                        std::max(boxHistTemp[i][1].x_width, 1e-6)) <= input.config.fixSizeDimThresh &&
                    (std::abs(trackedBBoxesTemp[i].y_width - boxHistTemp[i][1].y_width) /
                        std::max(boxHistTemp[i][1].y_width, 1e-6)) <= input.config.fixSizeDimThresh &&
                    (std::abs(trackedBBoxesTemp[i].z_width - boxHistTemp[i][1].z_width) /
                        std::max(boxHistTemp[i][1].z_width, 1e-6)) <= input.config.fixSizeDimThresh) {
                    trackedBBoxesTemp[i].x_width = boxHistTemp[i][1].x_width;
                    trackedBBoxesTemp[i].y_width = boxHistTemp[i][1].y_width;
                    trackedBBoxesTemp[i].z_width = boxHistTemp[i][1].z_width;
                    trackedBBoxesTemp[i].fix_size = true;
                    boxHistTemp[i][0].x_width = trackedBBoxesTemp[i].x_width;
                    boxHistTemp[i][0].y_width = trackedBBoxesTemp[i].y_width;
                    boxHistTemp[i][0].z_width = trackedBBoxesTemp[i].z_width;
                    boxHistTemp[i][0].fix_size = true;
                    output.fixedSizeCount += 1;
                }
            }
        }
    }

    output.tracks.reserve(trackedBBoxesTemp.size());
    output.trackedBBoxes.reserve(trackedBBoxesTemp.size());
    for (std::size_t i = 0; i < trackedBBoxesTemp.size(); ++i) {
        TrackState track;
        track.kf = filtersTemp[i];
        track.kf_initialized = true;
        track.filter_initialized = filterInitializedTemp[i];
        track.filter_state = filterStateTemp[i];
        track.currentBox = trackedBBoxesTemp[i];
        track.currentCenter = pcCenterHistTemp[i].empty() ?
            Eigen::Vector3d(track.currentBox.x, track.currentBox.y, track.currentBox.z) :
            pcCenterHistTemp[i].front();
        track.currentStd = pcStdHistTemp[i].empty() ? Eigen::Vector3d::Zero() : pcStdHistTemp[i].front();
        track.boxHistory = std::move(boxHistTemp[i]);
        track.clusterHistory = std::move(pcHistTemp[i]);
        track.centerHistory = std::move(pcCenterHistTemp[i]);
        track.stdHistory = std::move(pcStdHistTemp[i]);
        track.matchedInFrame = missedFramesTemp[i] == 0;
        track.age = ageTemp[i];
        track.missedFrames = missedFramesTemp[i];
        output.trackedBBoxes.push_back(track.currentBox);
        output.tracks.push_back(std::move(track));
    }

    return output;
}

}  // namespace onboardDetector
