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
#include <functional>
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
    const std::vector<TrackState>& tracks,
    const std::vector<box3D>& /*prevBBoxes*/,
    const std::vector<FeatureVector>& prevBBoxesFeat,
    const std::vector<box3D>& propedBBoxes,
    const std::vector<FeatureVector>& propedBBoxesFeat,
    const std::vector<FeatureVector>& currBBoxesFeat,
    const TrackingConfig& config,
    const std::vector<int>& currIndices,
    const std::vector<int>& trackIndices,
    std::vector<int>& bestMatch)
{
    bestMatch.assign(currBBoxes.size(), -1);
    if (currIndices.empty() || trackIndices.empty()) {
        return;
    }

    // Do not hard-bind to upstream marker IDs here. QC-GAF replacement IDs are
    // diagnostic tracklet hints, not a guaranteed identity source; using them
    // as a hard prior caused ID switches when two nearby targets crossed.
    // Association below is therefore driven by geometry and motion consistency.
    struct Candidate
    {
        int curr_idx;
        int prev_idx;
        double cost;
        double sim;
    };
    std::vector<std::vector<Candidate>> candidateLists(currIndices.size());

    for (std::size_t localDetIdx = 0; localDetIdx < currIndices.size(); ++localDetIdx) {
        const int i = currIndices[localDetIdx];
        const box3D& currBBox = currBBoxes[static_cast<std::size_t>(i)];
        for (const int trackIdx : trackIndices) {
            const std::size_t j = static_cast<std::size_t>(trackIdx);
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
            double directionPenalty = 0.0;
            if (j < prevBBoxesFeat.size() && j < propedBBoxes.size()) {
                const Eigen::Vector2d observedStep(
                    currBBox.x - propedBBox.x + propedBBox.Vx * config.dt,
                    currBBox.y - propedBBox.y + propedBBox.Vy * config.dt);
                const Eigen::Vector2d predictedStep(
                    propedBBox.Vx * config.dt,
                    propedBBox.Vy * config.dt);
                const double obsNorm = observedStep.norm();
                const double predNorm = predictedStep.norm();
                if (obsNorm > 1e-4 && predNorm > 1e-4) {
                    const double cosDir = std::clamp(observedStep.dot(predictedStep) / (obsNorm * predNorm), -1.0, 1.0);
                    directionPenalty = 0.15 * (1.0 - cosDir);
                }
            }
            const double sizePenalty = 0.25 * std::abs(propedWidth - currWidth);
            const double simBonus = 0.05 * sim;
            const double confirmedBonus =
                (j < tracks.size() && tracks[j].confirmed) ? 0.03 : 0.0;
            const double missPenalty =
                (j < tracks.size()) ? 0.03 * std::min<std::size_t>(tracks[j].missedFrames, 4) : 0.0;
            double gruPenalty = 0.0;
            if (config.enableGruAssociationCost && j < tracks.size() &&
                tracks[j].hasExternalPrediction && config.gruAssociationWeight > 0.0)
            {
                const Eigen::Vector3d& pred = tracks[j].externalPrediction;
                const double predDist = std::hypot(currBBox.x - pred.x(), currBBox.y - pred.y());
                const double gate = std::max(config.gruPredictionGate, 1e-6);
                if (std::isfinite(predDist)) {
                    // GRU is only an auxiliary soft prior.  Clamp the penalty so
                    // a domain-mismatched prediction cannot hard-reject a valid
                    // geometry/KF association.
                    gruPenalty = config.gruAssociationWeight * std::min(predDist, gate);
                }
            }
            const double cost =
                planar + sizePenalty + directionPenalty + gruPenalty + missPenalty - simBonus - confirmedBonus;
            candidateLists[localDetIdx].push_back(
                Candidate{i, static_cast<int>(j), cost, sim});
        }
    }

    for (auto& candidates : candidateLists) {
        std::sort(
            candidates.begin(), candidates.end(),
            [](const Candidate& lhs, const Candidate& rhs) {
                if (std::abs(lhs.cost - rhs.cost) > 1e-6) {
                    return lhs.cost < rhs.cost;
                }
                return lhs.sim > rhs.sim;
            });
        if (candidates.size() > 8) {
            candidates.resize(8);
        }
    }

    int bestMatches = -1;
    double bestTotalCost = std::numeric_limits<double>::infinity();
    std::vector<int> currentAssign(currIndices.size(), -1);
    std::vector<int> bestAssign(currIndices.size(), -1);
    std::vector<bool> usedTracks(propedBBoxes.size(), false);

    std::function<void(int, int, double)> dfs =
        [&](int detIdx, int matches, double totalCost) {
            const int remaining = static_cast<int>(currIndices.size()) - detIdx;
            if (matches + remaining < bestMatches) {
                return;
            }
            if (detIdx >= static_cast<int>(currIndices.size())) {
                if (matches > bestMatches ||
                    (matches == bestMatches && totalCost < bestTotalCost))
                {
                    bestMatches = matches;
                    bestTotalCost = totalCost;
                    bestAssign = currentAssign;
                }
                return;
            }

            currentAssign[static_cast<std::size_t>(detIdx)] = -1;
            dfs(detIdx + 1, matches, totalCost);

            for (const Candidate& c : candidateLists[static_cast<std::size_t>(detIdx)]) {
                if (usedTracks[static_cast<std::size_t>(c.prev_idx)]) {
                    continue;
                }
                usedTracks[static_cast<std::size_t>(c.prev_idx)] = true;
                currentAssign[static_cast<std::size_t>(detIdx)] = c.prev_idx;
                dfs(detIdx + 1, matches + 1, totalCost + c.cost);
                currentAssign[static_cast<std::size_t>(detIdx)] = -1;
                usedTracks[static_cast<std::size_t>(c.prev_idx)] = false;
            }
        };
    dfs(0, 0, 0.0);
    for (std::size_t localDetIdx = 0; localDetIdx < currIndices.size(); ++localDetIdx) {
        const int currIdx = currIndices[localDetIdx];
        if (currIdx >= 0 && static_cast<std::size_t>(currIdx) < bestMatch.size()) {
            bestMatch[static_cast<std::size_t>(currIdx)] = bestAssign[localDetIdx];
        }
    }

    std::vector<bool> curr_used(currBBoxes.size(), false);
    std::vector<bool> prev_used(propedBBoxes.size(), false);
    for (std::size_t localDetIdx = 0; localDetIdx < currIndices.size(); ++localDetIdx) {
        const int currIdx = currIndices[localDetIdx];
        const int match = bestMatch[static_cast<std::size_t>(currIdx)];
        if (match < 0 || static_cast<std::size_t>(match) >= prev_used.size()) {
            continue;
        }
        curr_used[static_cast<std::size_t>(currIdx)] = true;
        prev_used[static_cast<std::size_t>(match)] = true;
    }

    // Second-stage geometric rescue.  The original LV-DOT association rejects
    // any pair outside maxMatchRange before considering all other evidence.
    // With UAV ego-motion and asynchronous camera/LiDAR refinement, a valid
    // single object can occasionally jump just beyond that tight gate and
    // unnecessarily spawn a new track ID.  Keep the original match as stage 1,
    // then link remaining one-to-one candidates by nearest planar distance
    // under a bounded rescue gate.
    struct RescueCandidate
    {
        int curr_idx;
        int prev_idx;
        double planar;
    };
    std::vector<RescueCandidate> rescue_candidates;
    const double rescue_range = std::max(config.maxMatchRange * 2.0, config.maxMatchRange + 0.35);
    const double rescue_size_range = std::max(config.maxMatchSizeRange * 1.5, config.maxMatchSizeRange + 0.25);
    for (const int i : currIndices) {
        if (curr_used[static_cast<std::size_t>(i)]) {
            continue;
        }
        const box3D& currBBox = currBBoxes[static_cast<std::size_t>(i)];
        for (std::size_t j = 0; j < propedBBoxes.size(); ++j) {
            if (prev_used[j]) {
                continue;
            }
            const box3D& propedBBox = propedBBoxes[j];
            const double propedWidth = std::max(propedBBox.x_width, propedBBox.y_width);
            const double currWidth = std::max(currBBox.x_width, currBBox.y_width);
            if (std::abs(propedWidth - currWidth) >= rescue_size_range) {
                continue;
            }
            const double planar = std::sqrt(
                std::pow(propedBBox.x - currBBox.x, 2) +
                std::pow(propedBBox.y - currBBox.y, 2));
            if (planar >= rescue_range) {
                continue;
            }
            rescue_candidates.push_back(RescueCandidate{i, static_cast<int>(j), planar});
        }
    }
    std::sort(
        rescue_candidates.begin(), rescue_candidates.end(),
        [](const RescueCandidate& lhs, const RescueCandidate& rhs) {
            return lhs.planar < rhs.planar;
        });
    for (const RescueCandidate& c : rescue_candidates) {
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

Eigen::MatrixXd getReactivationObservationAcc(
    const box3D& currDetectedBBox,
    const TrackState& track,
    const TrackingConfig& config)
{
    if (!track.hasLastObservation) {
        return getKalmanObservationAcc(currDetectedBBox, track, config);
    }

    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(6, 1);
    Z(0) = currDetectedBBox.x;
    Z(1) = currDetectedBBox.y;

    const box3D& lastObserved = track.lastObservedBox;
    const double missFrames = static_cast<double>(std::max<std::size_t>(1, track.missedFrames));
    const double horizon = std::max(config.dt * (missFrames + 1.0), 1e-6);
    Z(2) = (currDetectedBBox.x - lastObserved.x) / horizon;
    Z(3) = (currDetectedBBox.y - lastObserved.y) / horizon;
    Z(4) = (Z(2) - lastObserved.Vx) / horizon;
    Z(5) = (Z(3) - lastObserved.Vy) / horizon;
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
            track.consecutiveHits = 1;
            track.confirmed = input.config.tentativeMinHits <= 1;
            track.currentBox.id = nextTrackId++;
            if (track.confirmed) {
                output.trackedBBoxes.push_back(track.currentBox);
            }
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

    std::vector<int> highScoreDetections;
    std::vector<int> lowScoreDetections;
    highScoreDetections.reserve(input.filteredBBoxes.size());
    lowScoreDetections.reserve(input.filteredBBoxes.size());
    for (std::size_t i = 0; i < input.filteredBBoxes.size(); ++i) {
        const double score = input.filteredBBoxes[i].score;
        if (score >= input.config.trackHighScoreThreshold) {
            highScoreDetections.push_back(static_cast<int>(i));
        } else if (score >= input.config.trackLowScoreThreshold) {
            lowScoreDetections.push_back(static_cast<int>(i));
        }
    }

    std::vector<int> allTrackIndices;
    std::vector<int> confirmedTrackIndices;
    allTrackIndices.reserve(input.tracks.size());
    confirmedTrackIndices.reserve(input.tracks.size());
    for (std::size_t i = 0; i < input.tracks.size(); ++i) {
        allTrackIndices.push_back(static_cast<int>(i));
        if (input.tracks[i].confirmed) {
            confirmedTrackIndices.push_back(static_cast<int>(i));
        }
    }

    std::vector<int> bestMatch;
    findBestMatch(
        input.filteredBBoxes,
        input.tracks,
        prevBBoxes,
        prevBBoxesFeat,
        propedBBoxes,
        propedBBoxesFeat,
        currBBoxesFeat,
        input.config,
        highScoreDetections,
        allTrackIndices,
        bestMatch);

    std::vector<int> unmatchedConfirmedTrackIndices;
    unmatchedConfirmedTrackIndices.reserve(confirmedTrackIndices.size());
    for (const int trackIdx : confirmedTrackIndices) {
        bool matched = false;
        for (int match : bestMatch) {
            if (match == trackIdx) {
                matched = true;
                break;
            }
        }
        if (!matched) {
            unmatchedConfirmedTrackIndices.push_back(trackIdx);
        }
    }
    if (!lowScoreDetections.empty() && !unmatchedConfirmedTrackIndices.empty()) {
        std::vector<int> lowScoreMatch;
        findBestMatch(
            input.filteredBBoxes,
            input.tracks,
            prevBBoxes,
            prevBBoxesFeat,
            propedBBoxes,
            propedBBoxesFeat,
            currBBoxesFeat,
            input.config,
            lowScoreDetections,
            unmatchedConfirmedTrackIndices,
            lowScoreMatch);
        for (const int detIdx : lowScoreDetections) {
            if (detIdx >= 0 &&
                static_cast<std::size_t>(detIdx) < lowScoreMatch.size() &&
                lowScoreMatch[static_cast<std::size_t>(detIdx)] >= 0 &&
                bestMatch[static_cast<std::size_t>(detIdx)] < 0)
            {
                bestMatch[static_cast<std::size_t>(detIdx)] =
                    lowScoreMatch[static_cast<std::size_t>(detIdx)];
            }
        }
    }

    std::vector<std::deque<box3D>> boxHistTemp;
    std::vector<std::deque<std::vector<ClusterPoint>>> pcHistTemp;
    std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
    std::vector<std::deque<Eigen::Vector3d>> pcStdHistTemp;
    std::vector<kalman_filter> filtersTemp;
    std::vector<std::array<double, 6>> filterStateTemp;
    std::vector<bool> filterInitializedTemp;
    std::vector<std::size_t> ageTemp;
    std::vector<std::size_t> consecutiveHitsTemp;
    std::vector<std::size_t> missedFramesTemp;
    std::vector<bool> confirmedTemp;
    std::vector<bool> hasLastObservationTemp;
    std::vector<box3D> lastObservedBoxTemp;
    std::vector<Eigen::Vector3d> lastObservedCenterTemp;
    std::vector<Eigen::Vector3d> lastObservedStdTemp;
    std::vector<box3D> trackedBBoxesTemp;

    boxHistTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    pcHistTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    pcCenterHistTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    pcStdHistTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    filtersTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    filterStateTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    filterInitializedTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    ageTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    consecutiveHitsTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    missedFramesTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    confirmedTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    hasLastObservationTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    lastObservedBoxTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    lastObservedCenterTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    lastObservedStdTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));
    trackedBBoxesTemp.reserve(static_cast<std::size_t>(numObjs + input.tracks.size()));

    auto trim_history = [&](auto& history) {
        if (static_cast<int>(history.size()) == input.config.histSize) {
            history.pop_back();
        }
    };

    for (int i = 0; i < numObjs; ++i) {
        box3D newEstimatedBBox;
        kalman_filter filter;
        std::array<double, 6> filterState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        bool filterInitialized = false;
        const box3D& currDetectedBBox = input.filteredBBoxes[static_cast<std::size_t>(i)];

        std::deque<box3D> boxHistory;
        std::deque<std::vector<ClusterPoint>> clusterHistory;
        std::deque<Eigen::Vector3d> centerHistory;
        std::deque<Eigen::Vector3d> stdHistory;
        std::size_t age = 1;
        std::size_t consecutiveHits = 1;
        bool confirmed = input.config.tentativeMinHits <= 1;
        bool hasLastObservation = false;
        box3D lastObservedBox;
        Eigen::Vector3d lastObservedCenter = Eigen::Vector3d::Zero();
        Eigen::Vector3d lastObservedStd = Eigen::Vector3d::Zero();

        if (bestMatch[i] >= 0) {
            const std::size_t matchIdx = static_cast<std::size_t>(bestMatch[i]);
            const auto& prevTrack = input.tracks[matchIdx];
            boxHistory = prevTrack.boxHistory;
            clusterHistory = prevTrack.clusterHistory;
            centerHistory = prevTrack.centerHistory;
            stdHistory = prevTrack.stdHistory;
            filter = prevTrack.kf;
            const bool isReactivation = prevTrack.confirmed && prevTrack.missedFrames > 0;
            const Eigen::MatrixXd Z = isReactivation ?
                getReactivationObservationAcc(currDetectedBBox, prevTrack, input.config) :
                getKalmanObservationAcc(currDetectedBBox, prevTrack, input.config);
            if (input.config.noiseAdaptationEnabled) {
                double H = 0.5 * (input.config.Hc + input.config.Hl);
                if (H < 0.0) H = 0.0;
                if (H > 1.0) H = 1.0;
                constexpr double kScaleMax = 1.5;
                const double q_scale = 1.0;
                double r_scale = 1.0 + input.config.alphaR * (1.0 - H);
                if (r_scale > kScaleMax) r_scale = kScaleMax;
                filter.setNoiseScales(q_scale, r_scale);
            }
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
            newEstimatedBBox.id = prevTrack.currentBox.id;

            filterState = {
                newEstimatedBBox.x,
                newEstimatedBBox.y,
                newEstimatedBBox.Vx,
                newEstimatedBBox.Vy,
                newEstimatedBBox.Ax,
                newEstimatedBBox.Ay
            };
            filterInitialized = true;
            age = prevTrack.age + 1;
            consecutiveHits = prevTrack.consecutiveHits + 1;
            confirmed = prevTrack.confirmed ||
                consecutiveHits >= static_cast<std::size_t>(input.config.tentativeMinHits);
            hasLastObservation = true;
        } else {
            if (currDetectedBBox.score < input.config.newTrackScoreThreshold) {
                continue;
            }
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
            hasLastObservation = true;
        }

        trim_history(boxHistory);
        trim_history(clusterHistory);
        trim_history(centerHistory);
        trim_history(stdHistory);

        boxHistory.push_front(newEstimatedBBox);
        if (static_cast<std::size_t>(i) < input.filteredPcClusters.size()) {
            clusterHistory.push_front(input.filteredPcClusters[static_cast<std::size_t>(i)]);
        } else {
            clusterHistory.push_front({});
        }
        if (static_cast<std::size_t>(i) < input.filteredPcClusterCenters.size()) {
            centerHistory.push_front(input.filteredPcClusterCenters[static_cast<std::size_t>(i)]);
        } else {
            centerHistory.push_front(Eigen::Vector3d(newEstimatedBBox.x, newEstimatedBBox.y, newEstimatedBBox.z));
        }
        if (static_cast<std::size_t>(i) < input.filteredPcClusterStds.size()) {
            stdHistory.push_front(input.filteredPcClusterStds[static_cast<std::size_t>(i)]);
        } else {
            stdHistory.push_front(Eigen::Vector3d::Zero());
        }
        lastObservedBox = newEstimatedBBox;
        lastObservedCenter = centerHistory.front();
        lastObservedStd = stdHistory.front();

        boxHistTemp.push_back(std::move(boxHistory));
        pcHistTemp.push_back(std::move(clusterHistory));
        pcCenterHistTemp.push_back(std::move(centerHistory));
        pcStdHistTemp.push_back(std::move(stdHistory));
        filtersTemp.push_back(filter);
        filterStateTemp.push_back(filterState);
        filterInitializedTemp.push_back(filterInitialized);
        ageTemp.push_back(age);
        consecutiveHitsTemp.push_back(consecutiveHits);
        missedFramesTemp.push_back(0);
        confirmedTemp.push_back(confirmed);
        hasLastObservationTemp.push_back(hasLastObservation);
        lastObservedBoxTemp.push_back(lastObservedBox);
        lastObservedCenterTemp.push_back(lastObservedCenter);
        lastObservedStdTemp.push_back(lastObservedStd);
        trackedBBoxesTemp.push_back(newEstimatedBBox);
    }

    if (input.config.maxUnmatchedFrames > 0 || input.config.tentativeMaxUnmatchedFrames > 0) {
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
            const std::size_t maxMissed =
                prevTrack.confirmed ?
                static_cast<std::size_t>(input.config.maxUnmatchedFrames) :
                static_cast<std::size_t>(input.config.tentativeMaxUnmatchedFrames);
            if (nextMissed > maxMissed || prevTrack.boxHistory.empty()) {
                continue;
            }

            kalman_filter propagatedFilter = prevTrack.kf;
            box3D propagatedBox = prevTrack.boxHistory.front();
            std::array<double, 6> propagatedFilterState = prevTrack.filter_state;
            if (prevTrack.filter_initialized && prevTrack.kf_initialized) {
                Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(6, 1);
                Z(0) = prevTrack.currentBox.x + prevTrack.currentBox.Vx * input.config.dt;
                Z(1) = prevTrack.currentBox.y + prevTrack.currentBox.Vy * input.config.dt;
                Z(2) = prevTrack.currentBox.Vx;
                Z(3) = prevTrack.currentBox.Vy;
                Z(4) = prevTrack.currentBox.Ax;
                Z(5) = prevTrack.currentBox.Ay;
                propagatedFilter.estimate(Z, Eigen::MatrixXd::Zero(6, 1));
                propagatedBox.x = propagatedFilter.output(0);
                propagatedBox.y = propagatedFilter.output(1);
                propagatedBox.Vx = propagatedFilter.output(2);
                propagatedBox.Vy = propagatedFilter.output(3);
                propagatedBox.Ax = propagatedFilter.output(4);
                propagatedBox.Ay = propagatedFilter.output(5);
                propagatedFilterState = {
                    propagatedBox.x,
                    propagatedBox.y,
                    propagatedBox.Vx,
                    propagatedBox.Vy,
                    propagatedBox.Ax,
                    propagatedBox.Ay
                };
            } else {
                propagatedBox.x += propagatedBox.Vx * input.config.dt;
                propagatedBox.y += propagatedBox.Vy * input.config.dt;
            }
            propagatedBox.is_estimated = true;
            propagatedBox.id = prevTrack.currentBox.id;

            auto boxHistory = prevTrack.boxHistory;
            auto clusterHistory = prevTrack.clusterHistory;
            auto centerHistory = prevTrack.centerHistory;
            auto stdHistory = prevTrack.stdHistory;
            trim_history(boxHistory);
            trim_history(clusterHistory);
            trim_history(centerHistory);
            trim_history(stdHistory);

            boxHistory.push_front(propagatedBox);
            clusterHistory.push_front(prevTrack.clusterHistory.empty() ? std::vector<ClusterPoint>{} : prevTrack.clusterHistory.front());
            centerHistory.push_front(prevTrack.centerHistory.empty() ?
                Eigen::Vector3d(propagatedBox.x, propagatedBox.y, propagatedBox.z) :
                prevTrack.centerHistory.front());
            stdHistory.push_front(prevTrack.stdHistory.empty() ? Eigen::Vector3d::Zero() : prevTrack.stdHistory.front());

            boxHistTemp.push_back(std::move(boxHistory));
            pcHistTemp.push_back(std::move(clusterHistory));
            pcCenterHistTemp.push_back(std::move(centerHistory));
            pcStdHistTemp.push_back(std::move(stdHistory));
            filtersTemp.push_back(propagatedFilter);
            filterStateTemp.push_back(propagatedFilterState);
            filterInitializedTemp.push_back(prevTrack.filter_initialized);
            ageTemp.push_back(prevTrack.age + 1);
            consecutiveHitsTemp.push_back(prevTrack.consecutiveHits);
            missedFramesTemp.push_back(nextMissed);
            confirmedTemp.push_back(prevTrack.confirmed);
            hasLastObservationTemp.push_back(prevTrack.hasLastObservation);
            lastObservedBoxTemp.push_back(prevTrack.lastObservedBox);
            lastObservedCenterTemp.push_back(prevTrack.lastObservedCenter);
            lastObservedStdTemp.push_back(prevTrack.lastObservedStd);
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

    // Bounded ID-consistency guard: guarantee each track ID maps to at most one
    // box per frame.  findBestMatch is injective on track INDEX, but `id` is a
    // copied payload with no cross-entry uniqueness guarantee; a transient
    // nextTrackId collision with a coasting track otherwise pins one ID onto
    // several distinct objects every frame (ID-collapse -> wrecks IDF1).  When a
    // duplicate ID appears, keep it on the first (matched-first ordering) entry
    // and reassign later duplicates a fresh ID, writing it back into currentBox
    // and boxHistory so the split persists via the track_states feedback loop.
    double maxAssignedId = -1.0;
    for (const auto & b : trackedBBoxesTemp) {
        maxAssignedId = std::max(maxAssignedId, b.id);
    }
    double nextFreeId = maxAssignedId + 1.0;
    std::vector<int> usedIds;
    usedIds.reserve(trackedBBoxesTemp.size());

    for (std::size_t i = 0; i < trackedBBoxesTemp.size(); ++i) {
        int idKey = static_cast<int>(std::llround(trackedBBoxesTemp[i].id));
        if (std::find(usedIds.begin(), usedIds.end(), idKey) != usedIds.end()) {
            const double freshId = nextFreeId;
            nextFreeId += 1.0;
            trackedBBoxesTemp[i].id = freshId;
            if (!boxHistTemp[i].empty()) {
                boxHistTemp[i].front().id = freshId;
            }
            idKey = static_cast<int>(std::llround(freshId));
        }
        usedIds.push_back(idKey);

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
        track.consecutiveHits = consecutiveHitsTemp[i];
        track.missedFrames = missedFramesTemp[i];
        track.confirmed = confirmedTemp[i];
        track.hasLastObservation = hasLastObservationTemp[i];
        track.lastObservedBox = lastObservedBoxTemp[i];
        track.lastObservedCenter = lastObservedCenterTemp[i];
        track.lastObservedStd = lastObservedStdTemp[i];
        if (track.confirmed && (track.matchedInFrame || input.config.publishEstimatedTracks)) {
            output.trackedBBoxes.push_back(track.currentBox);
        }
        output.tracks.push_back(std::move(track));
    }

    return output;
}

}  // namespace onboardDetector
