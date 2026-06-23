#include "lvdot_core/fusion_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace onboardDetector {

namespace {
constexpr double kMinYolo3DMatchIou = 0.15;
constexpr double kPedestrianThicknessPriorM = 0.25;
constexpr double kPedestrianCenterZOffsetM = 0.85;

Eigen::Vector3d point_vector(const ClusterPoint& point)
{
    return point.point;
}

box3D conservativeFuseBox(const box3D& lhs, const box3D& rhs)
{
    box3D bbox;
    const double xmax = std::max(lhs.x + lhs.x_width / 2.0, rhs.x + rhs.x_width / 2.0);
    const double xmin = std::min(lhs.x - lhs.x_width / 2.0, rhs.x - rhs.x_width / 2.0);
    const double ymax = std::max(lhs.y + lhs.y_width / 2.0, rhs.y + rhs.y_width / 2.0);
    const double ymin = std::min(lhs.y - lhs.y_width / 2.0, rhs.y - rhs.y_width / 2.0);
    const double zmax = std::max(lhs.z + lhs.z_width / 2.0, rhs.z + rhs.z_width / 2.0);
    const double zmin = std::min(lhs.z - lhs.z_width / 2.0, rhs.z - rhs.z_width / 2.0);
    bbox.x = (xmin + xmax) / 2.0;
    bbox.y = (ymin + ymax) / 2.0;
    bbox.z = (zmin + zmax) / 2.0;
    bbox.x_width = xmax - xmin;
    bbox.y_width = ymax - ymin;
    bbox.z_width = zmax - zmin;
    bbox.score = std::max(lhs.score, rhs.score);
    bbox.Vx = 0.0;
    bbox.Vy = 0.0;
    bbox.is_u_map_enhanced = lhs.is_u_map_enhanced || rhs.is_u_map_enhanced;
    return bbox;
}

void assignSequentialIds(std::vector<box3D>& boxes)
{
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        boxes[i].id = static_cast<double>(i);
    }
}

Eigen::Vector3d computeGeometryCompensatedCenter(
    const std::vector<ClusterPoint>& points,
    const Eigen::Vector3d& sensorPosition)
{
    const Eigen::Vector3d geometricCenter = computeCenter(points);
    if (points.empty()) {
        return geometricCenter;
    }

    double zMin = std::numeric_limits<double>::max();
    double zMax = std::numeric_limits<double>::lowest();
    double projMin = std::numeric_limits<double>::max();
    double projMax = std::numeric_limits<double>::lowest();
    bool hasPlanarDirection = false;

    Eigen::Vector3d compensatedCenter = geometricCenter;
    const Eigen::Vector2d planarObservation =
        (geometricCenter - sensorPosition).head<2>();
    Eigen::Vector2d planarDirection = planarObservation;
    const double planarNorm = planarDirection.norm();
    if (planarNorm > 1e-6) {
        planarDirection /= planarNorm;
        hasPlanarDirection = true;
    }

    for (const auto& sample : points) {
        const Eigen::Vector3d point = point_vector(sample);
        zMin = std::min(zMin, point.z());
        zMax = std::max(zMax, point.z());
        if (hasPlanarDirection) {
            const double projection = planarDirection.dot(point.head<2>());
            projMin = std::min(projMin, projection);
            projMax = std::max(projMax, projection);
        }
    }

    if (hasPlanarDirection && projMax >= projMin) {
        const double observedThickness = projMax - projMin;
        const double missingThickness =
            std::max(0.0, kPedestrianThicknessPriorM - observedThickness);
        compensatedCenter.x() += planarDirection.x() * (missingThickness * 0.5);
        compensatedCenter.y() += planarDirection.y() * (missingThickness * 0.5);
    }

    // UAV partial-view data in our diagnostics is dominated by top-visible-body observations.
    // A head-drop center estimate (z_max - half human height) is substantially less biased
    // than the previous foot-lift estimate (z_min + half human height).
    if (std::isfinite(zMax)) {
        compensatedCenter.z() = zMax - kPedestrianCenterZOffsetM;
    } else if (std::isfinite(zMin)) {
        compensatedCenter.z() = zMin + kPedestrianCenterZOffsetM;
    }

    return compensatedCenter;
}

void applyGeometryCompensationToBox(
    box3D& bbox,
    Eigen::Vector3d& center,
    const std::vector<ClusterPoint>& cluster,
    const Eigen::Vector3d& sensorPosition)
{
    if (cluster.empty()) {
        return;
    }
    center = computeGeometryCompensatedCenter(cluster, sensorPosition);
    bbox.x = center.x();
    bbox.y = center.y();
    bbox.z = center.z();
}

}  // namespace

Eigen::Vector3d computeCenter(const std::vector<ClusterPoint>& points)
{
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    if (points.empty()) {
        return center;
    }
    for (const auto& p : points) {
        center += point_vector(p);
    }
    center /= static_cast<double>(points.size());
    return center;
}

Eigen::Vector3d computeStd(const std::vector<ClusterPoint>& points, const Eigen::Vector3d& center)
{
    Eigen::Vector3d stds(0.0, 0.0, 0.0);
    if (points.empty()) {
        return stds;
    }
    for (const auto& p : points) {
        const Eigen::Vector3d diff = point_vector(p) - center;
        stds(0) += diff(0) * diff(0);
        stds(1) += diff(1) * diff(1);
        stds(2) += diff(2) * diff(2);
    }
    stds /= static_cast<double>(points.size());
    return stds.array().sqrt();
}

void calcPcFeat(
    const std::vector<ClusterPoint>& pcCluster,
    Eigen::Vector3d& pcClusterCenter,
    Eigen::Vector3d& pcClusterStd)
{
    pcClusterCenter = computeCenter(pcCluster);
    pcClusterStd = computeStd(pcCluster, pcClusterCenter);
}

double calBoxIOU(const box3D& box1, const box3D& box2, bool ignoreZmin)
{
    double box1Volume = box1.x_width * box1.y_width * box1.z_width;
    double box2Volume = box2.x_width * box2.y_width * box2.z_width;

    double l1Y = box1.y + box1.y_width / 2.0 - (box2.y - box2.y_width / 2.0);
    double l2Y = box2.y + box2.y_width / 2.0 - (box1.y - box1.y_width / 2.0);
    double l1X = box1.x + box1.x_width / 2.0 - (box2.x - box2.x_width / 2.0);
    double l2X = box2.x + box2.x_width / 2.0 - (box1.x - box1.x_width / 2.0);
    double l1Z = box1.z + box1.z_width / 2.0 - (box2.z - box2.z_width / 2.0);
    double l2Z = box2.z + box2.z_width / 2.0 - (box1.z - box1.z_width / 2.0);

    if (ignoreZmin) {
        const double zmin = std::max(box1.z - box1.z_width / 2.0, box2.z - box2.z_width / 2.0);
        const double zWidth1 = box1.z_width / 2.0 + (box1.z - zmin);
        const double zWidth2 = box2.z_width / 2.0 + (box2.z - zmin);
        box1Volume = box1.x_width * box1.y_width * zWidth1;
        box2Volume = box2.x_width * box2.y_width * zWidth2;

        l1Z = box1.z + box1.z_width / 2.0 - zmin;
        l2Z = box2.z + box2.z_width / 2.0 - zmin;
    }

    double overlapX = std::min(l1X, l2X);
    double overlapY = std::min(l1Y, l2Y);
    double overlapZ = std::min(l1Z, l2Z);

    if (std::max(l1X, l2X) <= std::max(box1.x_width, box2.x_width)) {
        overlapX = std::min(box1.x_width, box2.x_width);
    }
    if (std::max(l1Y, l2Y) <= std::max(box1.y_width, box2.y_width)) {
        overlapY = std::min(box1.y_width, box2.y_width);
    }
    if (std::max(l1Z, l2Z) <= std::max(box1.z_width, box2.z_width)) {
        overlapZ = std::min(box1.z_width, box2.z_width);
    }

    const double overlapVolume = overlapX * overlapY * overlapZ;
    double IOU = overlapVolume / (box1Volume + box2Volume - overlapVolume);
    if (overlapX <= 0.0 || overlapY <= 0.0 || overlapZ <= 0.0) {
        IOU = 0.0;
    }
    return IOU;
}

int getBestOverlapBBox(const box3D& currBBox, const std::vector<box3D>& targetBBoxes, double& bestIOU)
{
    bestIOU = 0.0;
    int bestIOUIdx = -1;
    for (std::size_t i = 0; i < targetBBoxes.size(); ++i) {
        const double IOU = calBoxIOU(currBBox, targetBBoxes[i]);
        if (IOU > bestIOU) {
            bestIOU = IOU;
            bestIOUIdx = static_cast<int>(i);
        }
    }
    return bestIOUIdx;
}

void transformBBox(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    const Eigen::Matrix3d& orientation,
    Eigen::Vector3d& newCenter,
    Eigen::Vector3d& newSize)
{
    const double x = center(0);
    const double y = center(1);
    const double z = center(2);
    const double xWidth = size(0);
    const double yWidth = size(1);
    const double zWidth = size(2);

    const Eigen::Vector3d p1(x + xWidth / 2.0, y + yWidth / 2.0, z + zWidth / 2.0);
    const Eigen::Vector3d p2(x + xWidth / 2.0, y + yWidth / 2.0, z - zWidth / 2.0);
    const Eigen::Vector3d p3(x + xWidth / 2.0, y - yWidth / 2.0, z + zWidth / 2.0);
    const Eigen::Vector3d p4(x + xWidth / 2.0, y - yWidth / 2.0, z - zWidth / 2.0);
    const Eigen::Vector3d p5(x - xWidth / 2.0, y + yWidth / 2.0, z + zWidth / 2.0);
    const Eigen::Vector3d p6(x - xWidth / 2.0, y + yWidth / 2.0, z - zWidth / 2.0);
    const Eigen::Vector3d p7(x - xWidth / 2.0, y - yWidth / 2.0, z + zWidth / 2.0);
    const Eigen::Vector3d p8(x - xWidth / 2.0, y - yWidth / 2.0, z - zWidth / 2.0);

    const Eigen::Vector3d p1m = orientation * p1 + position;
    const Eigen::Vector3d p2m = orientation * p2 + position;
    const Eigen::Vector3d p3m = orientation * p3 + position;
    const Eigen::Vector3d p4m = orientation * p4 + position;
    const Eigen::Vector3d p5m = orientation * p5 + position;
    const Eigen::Vector3d p6m = orientation * p6 + position;
    const Eigen::Vector3d p7m = orientation * p7 + position;
    const Eigen::Vector3d p8m = orientation * p8 + position;
    const std::vector<Eigen::Vector3d> pointsMap{p1m, p2m, p3m, p4m, p5m, p6m, p7m, p8m};

    double xmin = p1m(0);
    double xmax = p1m(0);
    double ymin = p1m(1);
    double ymax = p1m(1);
    double zmin = p1m(2);
    double zmax = p1m(2);
    for (const Eigen::Vector3d& pm : pointsMap) {
        xmin = std::min(xmin, pm(0));
        xmax = std::max(xmax, pm(0));
        ymin = std::min(ymin, pm(1));
        ymax = std::max(ymax, pm(1));
        zmin = std::min(zmin, pm(2));
        zmax = std::max(zmax, pm(2));
    }

    newCenter(0) = (xmin + xmax) / 2.0;
    newCenter(1) = (ymin + ymax) / 2.0;
    newCenter(2) = (zmin + zmax) / 2.0;
    newSize(0) = xmax - xmin;
    newSize(1) = ymax - ymin;
    newSize(2) = zmax - zmin;
}

FilterLVBBoxesOutput filterLVBBoxes(const FilterLVBBoxesInput& input)
{
    FilterLVBBoxesOutput output;
    output.stats.uv_input_count = input.uvBBoxes.size();
    output.stats.db_input_count = input.dbBBoxes.size();
    output.stats.yolo_input_count = input.yoloDetectionResults.size();

    // STEP 1: visual bbox = mutual-best overlap between uv and db bboxes.
    for (std::size_t i = 0; i < input.uvBBoxes.size(); ++i) {
        const box3D uvBBox = input.uvBBoxes[i];
        double bestIOUForUVBBox = 0.0;
        double bestIOUForDBBBox = 0.0;
        const int bestMatchForUVBBox = getBestOverlapBBox(uvBBox, input.dbBBoxes, bestIOUForUVBBox);
        if (bestMatchForUVBBox == -1) {
            output.stats.uv_no_db_candidate_count += 1;
            continue;
        }
        output.stats.uv_best_match_count += 1;

        const box3D matchedDBBBox = input.dbBBoxes[static_cast<std::size_t>(bestMatchForUVBBox)];
        const int bestMatchForDBBBox = getBestOverlapBBox(matchedDBBBox, input.uvBBoxes, bestIOUForDBBBox);
        if (bestMatchForDBBBox != -1) {
            output.stats.db_best_match_count += 1;
        }
        if (bestMatchForDBBBox != static_cast<int>(i)) {
            output.stats.uv_not_mutual_count += 1;
            continue;
        }
        if (!(bestIOUForUVBBox > input.boxIOUThresh &&
              bestIOUForDBBBox > input.boxIOUThresh))
        {
            output.stats.uv_mutual_iou_reject_count += 1;
            continue;
        }
        {
            output.stats.uv_db_mutual_match_count += 1;
            box3D bbox = conservativeFuseBox(uvBBox, matchedDBBBox);
            output.visualBBoxes.push_back(bbox);
            output.visualPcClusters.push_back(input.pcClustersVisual[static_cast<std::size_t>(bestMatchForUVBBox)]);
            output.visualPcClusterCenters.push_back(input.pcClusterCentersVisual[static_cast<std::size_t>(bestMatchForUVBBox)]);
            output.visualPcClusterStds.push_back(input.pcClusterStdsVisual[static_cast<std::size_t>(bestMatchForUVBBox)]);
        }
    }

    // STEP 2/3/4: lidar + visual fusion exactly following dynamicDetector::filterLVBBoxes.
    std::vector<bool> processedLidarBBoxes(input.lidarBBoxes.size(), false);
    std::vector<bool> processedVisualBBoxes(output.visualBBoxes.size(), false);
    for (std::size_t i = 0; i < output.visualBBoxes.size(); ++i) {
        if (processedVisualBBoxes[i]) {
            continue;
        }
        const box3D visualBBox = output.visualBBoxes[i];
        std::vector<int> overlappingLidarBBoxes;
        std::vector<int> overlappingVisualBBoxes;

        for (std::size_t j = 0; j < input.lidarBBoxes.size(); ++j) {
            if (processedLidarBBoxes[j]) {
                continue;
            }
            const box3D lidarBBox = input.lidarBBoxes[j];
            const double lvIOU = calBoxIOU(visualBBox, lidarBBox, true);
            if (lvIOU > input.boxIOUThresh) {
                overlappingLidarBBoxes.push_back(static_cast<int>(j));
                for (std::size_t k = 0; k < output.visualBBoxes.size(); ++k) {
                    if (processedVisualBBoxes[i] || i == k) {
                        continue;
                    }
                    const double lvIOUPotentialMatch = calBoxIOU(output.visualBBoxes[k], lidarBBox, true);
                    if (lvIOUPotentialMatch > input.boxIOUThresh) {
                        overlappingVisualBBoxes.push_back(static_cast<int>(k));
                    }
                }
            }
        }

        if (overlappingLidarBBoxes.empty()) {
            output.stats.visual_only_component_count += 1;
            output.stats.fusion_component_count += 1;
            output.filteredBBoxesBeforeYolo.push_back(visualBBox);
            output.filteredPcClustersBeforeYolo.push_back(output.visualPcClusters[i]);
            output.filteredPcClusterCentersBeforeYolo.push_back(output.visualPcClusterCenters[i]);
            output.filteredPcClusterStdsBeforeYolo.push_back(output.visualPcClusterStds[i]);
            processedVisualBBoxes[i] = true;
        } else {
            output.stats.fusion_component_count += 1;
            std::vector<ClusterPoint> fusedPcCluster = output.visualPcClusters[i];

            double xmax = visualBBox.x + visualBBox.x_width / 2.0;
            double xmin = visualBBox.x - visualBBox.x_width / 2.0;
            double ymax = visualBBox.y + visualBBox.y_width / 2.0;
            double ymin = visualBBox.y - visualBBox.y_width / 2.0;
            double zmax = visualBBox.z + visualBBox.z_width / 2.0;
            double zmin = visualBBox.z - visualBBox.z_width / 2.0;
            bool is_u_map_enhanced = visualBBox.is_u_map_enhanced;
            double fusedScore = visualBBox.score;

            for (const int lidarIdx : overlappingLidarBBoxes) {
                const box3D& lidarBox = input.lidarBBoxes[static_cast<std::size_t>(lidarIdx)];
                xmax = std::max(xmax, lidarBox.x + lidarBox.x_width / 2.0);
                xmin = std::min(xmin, lidarBox.x - lidarBox.x_width / 2.0);
                ymax = std::max(ymax, lidarBox.y + lidarBox.y_width / 2.0);
                ymin = std::min(ymin, lidarBox.y - lidarBox.y_width / 2.0);
                zmax = std::max(zmax, lidarBox.z + lidarBox.z_width / 2.0);
                zmin = std::min(zmin, lidarBox.z - lidarBox.z_width / 2.0);
                fusedPcCluster.insert(
                    fusedPcCluster.end(),
                    input.lidarPcClusters[static_cast<std::size_t>(lidarIdx)].begin(),
                    input.lidarPcClusters[static_cast<std::size_t>(lidarIdx)].end());
                processedLidarBBoxes[static_cast<std::size_t>(lidarIdx)] = true;
                is_u_map_enhanced = is_u_map_enhanced || lidarBox.is_u_map_enhanced;
                fusedScore = std::max(fusedScore, lidarBox.score);
            }

            for (const int visualIdx : overlappingVisualBBoxes) {
                const box3D& visualPotential = output.visualBBoxes[static_cast<std::size_t>(visualIdx)];
                xmax = std::max(xmax, visualPotential.x + visualPotential.x_width / 2.0);
                xmin = std::min(xmin, visualPotential.x - visualPotential.x_width / 2.0);
                ymax = std::max(ymax, visualPotential.y + visualPotential.y_width / 2.0);
                ymin = std::min(ymin, visualPotential.y - visualPotential.y_width / 2.0);
                zmax = std::max(zmax, visualPotential.z + visualPotential.z_width / 2.0);
                zmin = std::min(zmin, visualPotential.z - visualPotential.z_width / 2.0);
                fusedPcCluster.insert(
                    fusedPcCluster.end(),
                    output.visualPcClusters[static_cast<std::size_t>(visualIdx)].begin(),
                    output.visualPcClusters[static_cast<std::size_t>(visualIdx)].end());
                processedVisualBBoxes[static_cast<std::size_t>(visualIdx)] = true;
                is_u_map_enhanced = is_u_map_enhanced || visualPotential.is_u_map_enhanced;
            }

            Eigen::Vector3d fusedPcClusterCenter;
            Eigen::Vector3d fusedPcClusterStd;
            calcPcFeat(fusedPcCluster, fusedPcClusterCenter, fusedPcClusterStd);

            box3D fusedBBox;
            fusedBBox.x = (xmin + xmax) / 2.0;
            fusedBBox.y = (ymin + ymax) / 2.0;
            fusedBBox.z = (zmin + zmax) / 2.0;
            fusedBBox.x_width = xmax - xmin;
            fusedBBox.y_width = ymax - ymin;
            fusedBBox.z_width = zmax - zmin;
            fusedBBox.score = fusedScore;
            fusedBBox.Vx = 0.0;
            fusedBBox.Vy = 0.0;
            fusedBBox.is_u_map_enhanced = is_u_map_enhanced;

            output.filteredBBoxesBeforeYolo.push_back(fusedBBox);
            output.filteredPcClustersBeforeYolo.push_back(fusedPcCluster);
            output.filteredPcClusterCentersBeforeYolo.push_back(fusedPcClusterCenter);
            output.filteredPcClusterStdsBeforeYolo.push_back(fusedPcClusterStd);
            processedVisualBBoxes[i] = true;
        }
    }

    for (std::size_t i = 0; i < input.lidarBBoxes.size(); ++i) {
        if (processedLidarBBoxes[i]) {
            continue;
        }
        output.stats.lidar_only_component_count += 1;
        output.stats.fusion_component_count += 1;
        output.filteredBBoxesBeforeYolo.push_back(input.lidarBBoxes[i]);
        output.filteredPcClustersBeforeYolo.push_back(input.lidarPcClusters[i]);
        output.filteredPcClusterCentersBeforeYolo.push_back(input.lidarPcClusterCenters[i]);
        output.filteredPcClusterStdsBeforeYolo.push_back(input.lidarPcClusterStds[i]);
        processedLidarBBoxes[i] = true;
    }

    for (std::size_t i = 0; i < output.filteredBBoxesBeforeYolo.size(); ++i) {
        applyGeometryCompensationToBox(
            output.filteredBBoxesBeforeYolo[i],
            output.filteredPcClusterCentersBeforeYolo[i],
            output.filteredPcClustersBeforeYolo[i],
            input.positionColor);
    }

    assignSequentialIds(output.filteredBBoxesBeforeYolo);
    output.filteredBBoxes = output.filteredBBoxesBeforeYolo;
    output.filteredPcClusters = output.filteredPcClustersBeforeYolo;
    output.filteredPcClusterCenters = output.filteredPcClusterCentersBeforeYolo;
    output.filteredPcClusterStds = output.filteredPcClusterStdsBeforeYolo;
    output.stats.yolo_candidate_3d_count = output.filteredBBoxes.size();
    output.stats.split_source_boxes = output.filteredBBoxes.size();

    if (input.yoloDetectionResults.empty()) {
        output.stats.split_output_boxes = output.filteredBBoxes.size();
        return output;
    }

    std::vector<int> best3DBBoxForYOLO(input.yoloDetectionResults.size(), -1);
    std::vector<ImageBBox2D> selectedYoloBBoxes = input.yoloDetectionResults;
    std::vector<ImageBBox2D> filteredDetectionResults;
    filteredDetectionResults.reserve(output.filteredBBoxes.size());
    for (std::size_t j = 0; j < output.filteredBBoxes.size(); ++j) {
        const box3D& bbox = output.filteredBBoxes[j];
        const Eigen::Vector3d centerWorld(bbox.x, bbox.y, bbox.z);
        const Eigen::Vector3d sizeWorld(bbox.x_width, bbox.y_width, bbox.z_width);
        Eigen::Vector3d centerCam;
        Eigen::Vector3d sizeCam;
        transformBBox(
            centerWorld,
            sizeWorld,
            -input.orientationColor.inverse() * input.positionColor,
            input.orientationColor.inverse(),
            centerCam,
            sizeCam);

        const Eigen::Vector3d topleft(centerCam(0) - sizeCam(0) / 2.0, centerCam(1) - sizeCam(1) / 2.0, centerCam(2));
        const Eigen::Vector3d bottomright(centerCam(0) + sizeCam(0) / 2.0, centerCam(1) + sizeCam(1) / 2.0, centerCam(2));

        const int tlX = static_cast<int>((input.fxC * topleft(0) + input.cxC * topleft(2)) / topleft(2));
        const int tlY = static_cast<int>((input.fyC * topleft(1) + input.cyC * topleft(2)) / topleft(2));
        const int brX = static_cast<int>((input.fxC * bottomright(0) + input.cxC * bottomright(2)) / bottomright(2));
        const int brY = static_cast<int>((input.fyC * bottomright(1) + input.cyC * bottomright(2)) / bottomright(2));

        ImageBBox2D result;
        result.x = tlX;
        result.y = tlY;
        result.width = brX - tlX;
        result.height = brY - tlY;
        filteredDetectionResults.push_back(result);
    }

    auto calc_iou_2d = [](const ImageBBox2D& lhs, const ImageBBox2D& rhs) {
        const int lhsTlX = lhs.x;
        const int lhsTlY = lhs.y;
        const int lhsBrX = lhs.x + lhs.width;
        const int lhsBrY = lhs.y + lhs.height;
        const int rhsTlX = rhs.x;
        const int rhsTlY = rhs.y;
        const int rhsBrX = rhs.x + rhs.width;
        const int rhsBrY = rhs.y + rhs.height;

        const double xOverlap = std::max(0, std::min(lhsBrX, rhsBrX) - std::max(lhsTlX, rhsTlX));
        const double yOverlap = std::max(0, std::min(lhsBrY, rhsBrY) - std::max(lhsTlY, rhsTlY));
        const double intersection = xOverlap * yOverlap;
        const double areaLhs = static_cast<double>((lhsBrX - lhsTlX) * (lhsBrY - lhsTlY));
        const double areaRhs = static_cast<double>((rhsBrX - rhsTlX) * (rhsBrY - rhsTlY));
        const double unionArea = areaLhs + areaRhs - intersection;
        return unionArea == 0.0 ? 0.0 : intersection / unionArea;
    };

    for (std::size_t i = 0; i < input.yoloDetectionResults.size(); ++i) {
        const ImageBBox2D boxA = input.yoloDetectionResults[i];
        ImageBBox2D boxB = boxA;
        boxB.x = static_cast<int>(std::round(static_cast<double>(boxA.x) - static_cast<double>(boxA.width) * 0.5));
        boxB.y = static_cast<int>(std::round(static_cast<double>(boxA.y) - static_cast<double>(boxA.height) * 0.5));

        double bestIOU = 0.0;
        int bestIdx = -1;
        ImageBBox2D bestTarget = boxA;
        for (std::size_t j = 0; j < output.filteredBBoxes.size(); ++j) {
            const ImageBBox2D& projectedBox = filteredDetectionResults[j];
            const double iouA = calc_iou_2d(projectedBox, boxA);
            const double iouB = calc_iou_2d(projectedBox, boxB);
            if (iouA > bestIOU || iouB > bestIOU) {
                if (iouA >= iouB) {
                    bestIOU = iouA;
                    bestTarget = boxA;
                } else {
                    bestIOU = iouB;
                    bestTarget = boxB;
                }
                bestIdx = static_cast<int>(j);
            }
        }

        if (bestIOU >= kMinYolo3DMatchIou) {
            best3DBBoxForYOLO[i] = bestIdx;
            selectedYoloBBoxes[i] = bestTarget;
        }
    }

    std::map<int, std::vector<int>> box3DToYolo;
    for (std::size_t i = 0; i < best3DBBoxForYOLO.size(); ++i) {
        const int idx3D = best3DBBoxForYOLO[i];
        if (idx3D >= 0 && idx3D < static_cast<int>(output.filteredBBoxes.size())) {
            box3DToYolo[idx3D].push_back(static_cast<int>(i));
        }
    }
    output.stats.yolo_matched_3d_count = box3DToYolo.size();
    for (const auto& [_, yolo_indices] : box3DToYolo) {
        output.stats.yolo_matched_detection_count += yolo_indices.size();
    }

    std::vector<box3D> newFilteredBBoxes;
    std::vector<std::vector<ClusterPoint>> newFilteredPcClusters;
    std::vector<Eigen::Vector3d> newFilteredPcClusterCenters;
    std::vector<Eigen::Vector3d> newFilteredPcClusterStds;

    for (int idx3D = 0; idx3D < static_cast<int>(output.filteredBBoxes.size()); ++idx3D) {
        const auto it = box3DToYolo.find(idx3D);
        if (it == box3DToYolo.end()) {
            newFilteredBBoxes.push_back(output.filteredBBoxes[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusters.push_back(output.filteredPcClusters[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusterCenters.push_back(output.filteredPcClusterCenters[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusterStds.push_back(output.filteredPcClusterStds[static_cast<std::size_t>(idx3D)]);
            continue;
        }

        const std::vector<int>& yoloIndices = it->second;
        if (yoloIndices.size() == 1) {
            box3D marked = output.filteredBBoxes[static_cast<std::size_t>(idx3D)];
            marked.is_dynamic = true;
            marked.is_human = true;
            marked.score = std::max(
                marked.score,
                input.yoloDetectionResults[static_cast<std::size_t>(yoloIndices.front())].score);
            newFilteredBBoxes.push_back(marked);
            newFilteredPcClusters.push_back(output.filteredPcClusters[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusterCenters.push_back(output.filteredPcClusterCenters[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusterStds.push_back(output.filteredPcClusterStds[static_cast<std::size_t>(idx3D)]);
            output.stats.yolo_human_marked_count += 1;
            continue;
        }

        const std::vector<ClusterPoint>& cloudCluster = output.filteredPcClusters[static_cast<std::size_t>(idx3D)];
        const int allowMargin = 0;
        std::vector<int> assignment(cloudCluster.size(), -1);
        for (std::size_t i = 0; i < cloudCluster.size(); ++i) {
            const Eigen::Vector3d ptWorld = point_vector(cloudCluster[i]);
            const Eigen::Vector3d ptCam = input.orientationColor.inverse() * (ptWorld - input.positionColor);
            const int u = static_cast<int>((input.fxC * ptCam(0) + input.cxC * ptCam(2)) / ptCam(2));
            const int v = static_cast<int>((input.fyC * ptCam(1) + input.cyC * ptCam(2)) / ptCam(2));

            int closestDist = std::numeric_limits<int>::max();
            for (const int yidx : yoloIndices) {
                const int xMin = selectedYoloBBoxes[static_cast<std::size_t>(yidx)].x;
                const int xMax = xMin + selectedYoloBBoxes[static_cast<std::size_t>(yidx)].width;
                const int yMin = selectedYoloBBoxes[static_cast<std::size_t>(yidx)].y;
                const int yMax = yMin + selectedYoloBBoxes[static_cast<std::size_t>(yidx)].height;

                if (u >= xMin - allowMargin && u <= xMax + allowMargin &&
                    v >= yMin - allowMargin && v <= yMax + allowMargin)
                {
                    int horizontalDistance = 0;
                    if (u < xMin) {
                        horizontalDistance = xMin - u;
                    } else if (u > xMax) {
                        horizontalDistance = u - xMax;
                    } else {
                        horizontalDistance = std::max(xMin - u, u - xMax);
                    }

                    const int distance = horizontalDistance;
                    if (distance < closestDist) {
                        assignment[i] = yidx;
                        closestDist = distance;
                    }
                }
            }
        }

        std::vector<bool> flag(cloudCluster.size(), false);
        bool emitted_any_sub_box = false;
        for (const int yidx : yoloIndices) {
            std::vector<ClusterPoint> subCloud;
            for (std::size_t i = 0; i < cloudCluster.size(); ++i) {
                if (flag[i]) {
                    continue;
                }
                if (assignment[i] == yidx) {
                    subCloud.push_back(cloudCluster[i]);
                    flag[i] = true;
                }
            }

            if (!subCloud.empty()) {
                const Eigen::Vector3d geometricCenter = computeCenter(subCloud);
                double xMin = std::numeric_limits<double>::max();
                double xMax = std::numeric_limits<double>::lowest();
                double yMin = std::numeric_limits<double>::max();
                double yMax = std::numeric_limits<double>::lowest();
                double zMin = std::numeric_limits<double>::max();
                double zMax = std::numeric_limits<double>::lowest();

                for (const auto& pt : subCloud) {
                    const Eigen::Vector3d point = point_vector(pt);
                    xMin = std::min(xMin, point.x());
                    xMax = std::max(xMax, point.x());
                    yMin = std::min(yMin, point.y());
                    yMax = std::max(yMax, point.y());
                    zMin = std::min(zMin, point.z());
                    zMax = std::max(zMax, point.z());
                }

                const Eigen::Vector3d center =
                    computeGeometryCompensatedCenter(subCloud, input.positionColor);

                box3D newBox;
                newBox.x = center.x();
                newBox.y = center.y();
                newBox.z = center.z();
                newBox.x_width = xMax - xMin;
                newBox.y_width = yMax - yMin;
                newBox.z_width = zMax - zMin;
                newBox.score = std::max(
                    output.filteredBBoxes[static_cast<std::size_t>(idx3D)].score,
                    input.yoloDetectionResults[static_cast<std::size_t>(yidx)].score);
                newBox.is_dynamic = true;
                newBox.is_human = true;
                newBox.is_u_map_enhanced = output.filteredBBoxes[static_cast<std::size_t>(idx3D)].is_u_map_enhanced;

                if (newBox.x_width <= 0.0 || newBox.y_width <= 0.0 || newBox.z_width <= 0.0) {
                    continue;
                }

                const Eigen::Vector3d stddev = computeStd(subCloud, geometricCenter);
                newFilteredBBoxes.push_back(newBox);
                newFilteredPcClusters.push_back(subCloud);
                newFilteredPcClusterCenters.push_back(center);
                newFilteredPcClusterStds.push_back(stddev);
                emitted_any_sub_box = true;
                output.stats.yolo_human_marked_count += 1;
            }
        }
        if (!emitted_any_sub_box) {
            newFilteredBBoxes.push_back(output.filteredBBoxes[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusters.push_back(output.filteredPcClusters[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusterCenters.push_back(output.filteredPcClusterCenters[static_cast<std::size_t>(idx3D)]);
            newFilteredPcClusterStds.push_back(output.filteredPcClusterStds[static_cast<std::size_t>(idx3D)]);
        }
    }

    output.filteredBBoxes = std::move(newFilteredBBoxes);
    output.filteredPcClusters = std::move(newFilteredPcClusters);
    output.filteredPcClusterCenters = std::move(newFilteredPcClusterCenters);
    output.filteredPcClusterStds = std::move(newFilteredPcClusterStds);
    assignSequentialIds(output.filteredBBoxes);
    output.stats.split_success_boxes = 0;
    for (const auto& [_, yolo_indices] : box3DToYolo) {
        if (yolo_indices.size() > 1) {
            output.stats.split_success_boxes += 1;
        }
    }
    output.stats.split_output_boxes = output.filteredBBoxes.size();
    output.stats.yolo_fused_used_count = output.stats.yolo_human_marked_count;
    return output;
}

}  // namespace onboardDetector
