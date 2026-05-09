#include "lvdot_core/detection_filter.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>

#include "lvdot_core/dbscan.hpp"
#include "lvdot_core/uv_detector.hpp"

namespace onboardDetector {

namespace {

bool isInFilterRange(const Eigen::Vector3d& pos, const DetectionConfig& config)
{
    return (pos(0) >= config.position(0) - config.localSensorRange(0)) &&
           (pos(0) <= config.position(0) + config.localSensorRange(0)) &&
           (pos(1) >= config.position(1) - config.localSensorRange(1)) &&
           (pos(1) <= config.position(1) + config.localSensorRange(1)) &&
           (pos(2) >= config.position(2) - config.localSensorRange(2)) &&
           (pos(2) <= config.position(2) + config.localSensorRange(2));
}

int posToAddress(const Eigen::Vector3d& pos, double res, const DetectionConfig& config)
{
    Eigen::Vector3i idx;
    idx(0) = static_cast<int>(std::floor((pos(0) - config.position(0) + config.localSensorRange(0)) / res));
    idx(1) = static_cast<int>(std::floor((pos(1) - config.position(1) + config.localSensorRange(1)) / res));
    idx(2) = static_cast<int>(std::floor((pos(2) - config.position(2) + config.localSensorRange(2)) / res));

    return idx(0) * static_cast<int>(std::ceil(2.0 * config.localSensorRange(1) / res)) *
           static_cast<int>(std::ceil(2.0 * config.localSensorRange(2) / res)) +
           idx(1) * static_cast<int>(std::ceil(2.0 * config.localSensorRange(2) / res)) +
           idx(2);
}

void transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                   const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                   Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize)
{
    const double x = center(0);
    const double y = center(1);
    const double z = center(2);
    const double xWidth = size(0);
    const double yWidth = size(1);
    const double zWidth = size(2);

    Eigen::Vector3d p1(x + xWidth / 2.0, y + yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p2(x + xWidth / 2.0, y + yWidth / 2.0, z - zWidth / 2.0);
    Eigen::Vector3d p3(x + xWidth / 2.0, y - yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p4(x + xWidth / 2.0, y - yWidth / 2.0, z - zWidth / 2.0);
    Eigen::Vector3d p5(x - xWidth / 2.0, y + yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p6(x - xWidth / 2.0, y + yWidth / 2.0, z - zWidth / 2.0);
    Eigen::Vector3d p7(x - xWidth / 2.0, y - yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p8(x - xWidth / 2.0, y - yWidth / 2.0, z - zWidth / 2.0);

    std::vector<Eigen::Vector3d> pointsMap{
        orientation * p1 + position,
        orientation * p2 + position,
        orientation * p3 + position,
        orientation * p4 + position,
        orientation * p5 + position,
        orientation * p6 + position,
        orientation * p7 + position,
        orientation * p8 + position};

    double xmin = pointsMap.front()(0);
    double xmax = pointsMap.front()(0);
    double ymin = pointsMap.front()(1);
    double ymax = pointsMap.front()(1);
    double zmin = pointsMap.front()(2);
    double zmax = pointsMap.front()(2);
    for (const auto& pm : pointsMap) {
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

std::vector<ClusterPoint> projectDepthImage(const DetectionInput& input)
{
    std::vector<ClusterPoint> projected;
    const auto& depth = input.depthImageMm;
    const auto& config = input.config;
    if (depth.empty() || depth.type() != CV_16UC1) {
        return projected;
    }

    const int cols = depth.cols;
    const int rows = depth.rows;
    const double inv_factor = 1.0 / std::max(config.depthScale, 1e-6);
    const double inv_fx = 1.0 / std::max(config.fx, 1e-6);
    const double inv_fy = 1.0 / std::max(config.fy, 1e-6);

    projected.reserve(static_cast<std::size_t>(cols * rows) /
                      static_cast<std::size_t>(std::max(1, config.skipPixel * config.skipPixel)));

    for (int v = config.depthFilterMargin; v < rows - config.depthFilterMargin; v += config.skipPixel) {
        const uint16_t* rowPtr = depth.ptr<uint16_t>(v) + config.depthFilterMargin;
        for (int u = config.depthFilterMargin; u < cols - config.depthFilterMargin; u += config.skipPixel) {
            double depthValue = (*rowPtr) * inv_factor;
            if (*rowPtr == 0) {
                depthValue = config.raycastMaxLength + 0.1;
            } else if (depthValue < config.depthMinValue) {
                rowPtr += config.skipPixel;
                continue;
            } else if (depthValue > config.depthMaxValue) {
                depthValue = config.raycastMaxLength + 0.1;
            }
            rowPtr += config.skipPixel;

            Eigen::Vector3d currPointCam;
            currPointCam(0) = (u - config.cx) * depthValue * inv_fx;
            currPointCam(1) = (v - config.cy) * depthValue * inv_fy;
            currPointCam(2) = depthValue;
            const Eigen::Vector3d currPointMap = config.orientationDepth * currPointCam + config.positionDepth;

            ClusterPoint sample;
            sample.point = currPointMap;
            sample.depth = depthValue;
            sample.u = u;
            sample.v = v;
            sample.has_image_point = true;
            projected.push_back(sample);
        }
    }
    return projected;
}

std::vector<ClusterPoint> voxelFilter(const std::vector<ClusterPoint>& points, const DetectionConfig& config)
{
    constexpr double res = 0.1;
    const int xVoxels = static_cast<int>(std::ceil(2.0 * config.localSensorRange(0) / res));
    const int yVoxels = static_cast<int>(std::ceil(2.0 * config.localSensorRange(1) / res));
    const int zVoxels = static_cast<int>(std::ceil(2.0 * config.localSensorRange(2) / res));
    const int totalVoxels = xVoxels * yVoxels * zVoxels;
    std::vector<int> voxelOccupancyVec(static_cast<std::size_t>(std::max(0, totalVoxels)), 0);

    std::vector<ClusterPoint> filteredPoints;
    filteredPoints.reserve(points.size());
    for (const auto& p : points) {
        if (isInFilterRange(p.point, config) &&
            p.point(2) >= config.groundHeight &&
            p.depth <= config.raycastMaxLength)
        {
            const int pID = posToAddress(p.point, res, config);
            if (pID < 0 || pID >= totalVoxels) {
                continue;
            }
            voxelOccupancyVec[static_cast<std::size_t>(pID)] += 1;
            if (voxelOccupancyVec[static_cast<std::size_t>(pID)] ==
                static_cast<int>(std::round(config.voxelOccThresh)))
            {
                filteredPoints.push_back(p);
            }
        }
    }
    return filteredPoints;
}

std::vector<ClusterPoint> filterPoints(const std::vector<ClusterPoint>& points, const DetectionConfig& config)
{
    const auto voxelFilteredPoints = voxelFilter(points, config);
    std::vector<ClusterPoint> filteredPoints;
    filteredPoints.reserve(voxelFilteredPoints.size());
    for (const auto& point : voxelFilteredPoints) {
        if (point.point.z() <= config.roofHeight && point.point.z() >= config.groundHeight) {
            filteredPoints.push_back(point);
        }
    }
    return filteredPoints;
}

void clusterPointsAndBBoxes(const std::vector<ClusterPoint>& points, const DetectionConfig& config,
                            std::vector<box3D>& bboxes,
                            std::vector<std::vector<ClusterPoint>>& pcClusters,
                            std::vector<Eigen::Vector3d>& pcClusterCenters,
                            std::vector<Eigen::Vector3d>& pcClusterStds)
{
    std::vector<Point> pointsDB;
    pointsDB.reserve(points.size());
    for (const auto& p : points) {
        Point db;
        db.x = static_cast<float>(p.point.x());
        db.y = static_cast<float>(p.point.y());
        db.z = static_cast<float>(p.point.z());
        db.clusterID = UNCLASSIFIED;
        pointsDB.push_back(db);
    }

    DBSCAN dbCluster(config.dbMinPointsCluster, static_cast<float>(config.dbEpsilon), pointsDB);
    dbCluster.run();

    int clusterNum = 0;
    for (const auto& pDB : dbCluster.m_points) {
        if (pDB.clusterID > clusterNum) {
            clusterNum = pDB.clusterID;
        }
    }

    std::vector<std::vector<ClusterPoint>> pcClustersTemp(static_cast<std::size_t>(std::max(0, clusterNum)));
    for (std::size_t i = 0; i < dbCluster.m_points.size(); ++i) {
        const auto& pDB = dbCluster.m_points[i];
        if (pDB.clusterID > 0) {
            pcClustersTemp[static_cast<std::size_t>(pDB.clusterID - 1)].push_back(points[i]);
        }
    }

    pcClusters.clear();
    bboxes.clear();
    pcClusterCenters.clear();
    pcClusterStds.clear();
    for (std::size_t i = 0; i < pcClustersTemp.size(); ++i) {
        if (pcClustersTemp[i].empty()) {
            continue;
        }
        box3D box;
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        xs.reserve(pcClustersTemp[i].size());
        ys.reserve(pcClustersTemp[i].size());
        zs.reserve(pcClustersTemp[i].size());

        double xmin_raw = pcClustersTemp[i][0].point(0);
        double ymin_raw = pcClustersTemp[i][0].point(1);
        double zmin_raw = pcClustersTemp[i][0].point(2);
        double xmax_raw = pcClustersTemp[i][0].point(0);
        double ymax_raw = pcClustersTemp[i][0].point(1);
        double zmax_raw = pcClustersTemp[i][0].point(2);
        for (const auto& sample : pcClustersTemp[i]) {
            const double x = sample.point(0);
            const double y = sample.point(1);
            const double z = sample.point(2);
            xs.push_back(x);
            ys.push_back(y);
            zs.push_back(z);
            xmin_raw = std::min(xmin_raw, x);
            ymin_raw = std::min(ymin_raw, y);
            zmin_raw = std::min(zmin_raw, z);
            xmax_raw = std::max(xmax_raw, x);
            ymax_raw = std::max(ymax_raw, y);
            zmax_raw = std::max(zmax_raw, z);
        }

        auto quantile_bound = [](std::vector<double>& values, double q) {
            if (values.empty()) {
                return 0.0;
            }
            q = std::clamp(q, 0.0, 1.0);
            const std::size_t idx = static_cast<std::size_t>(q * static_cast<double>(values.size() - 1));
            std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx), values.end());
            return values[idx];
        };

        double xmin = xmin_raw;
        double ymin = ymin_raw;
        double zmin = zmin_raw;
        double xmax = xmax_raw;
        double ymax = ymax_raw;
        double zmax = zmax_raw;

        if (static_cast<int>(pcClustersTemp[i].size()) >= config.depthBBoxQuantileMinPoints) {
            const double qx_low = quantile_bound(xs, config.depthBBoxQuantileXYLow);
            const double qx_high = quantile_bound(xs, config.depthBBoxQuantileXYHigh);
            const double qy_low = quantile_bound(ys, config.depthBBoxQuantileXYLow);
            const double qy_high = quantile_bound(ys, config.depthBBoxQuantileXYHigh);
            const double qz_low = quantile_bound(zs, config.depthBBoxQuantileZLow);
            const double qz_high = quantile_bound(zs, config.depthBBoxQuantileZHigh);

            if (qx_high > qx_low) {
                xmin = qx_low;
                xmax = qx_high;
            }
            if (qy_high > qy_low) {
                ymin = qy_low;
                ymax = qy_high;
            }
            if (qz_high > qz_low) {
                zmin = qz_low;
                zmax = qz_high;
            }
        }

        xmin -= std::max(0.0, config.depthBBoxPaddingXY);
        xmax += std::max(0.0, config.depthBBoxPaddingXY);
        ymin -= std::max(0.0, config.depthBBoxPaddingXY);
        ymax += std::max(0.0, config.depthBBoxPaddingXY);
        zmin -= std::max(0.0, config.depthBBoxPaddingZ);
        zmax += std::max(0.0, config.depthBBoxPaddingZ);

        zmin = std::max(zmin, config.groundHeight);

        box.id = static_cast<double>(i);
        box.x = (xmax + xmin) / 2.0;
        box.y = (ymax + ymin) / 2.0;
        box.z = (zmax + zmin) / 2.0;
        box.x_width = (xmax - xmin) > 0.1 ? (xmax - xmin) : 0.1;
        box.y_width = (ymax - ymin) > 0.1 ? (ymax - ymin) : 0.1;
        box.z_width = (zmax - zmin);

        if (box.x_width < config.minObjectSize(0) ||
            box.y_width < config.minObjectSize(1) ||
            box.z_width < config.minObjectSize(2))
        {
            continue;
        }
        if (box.x_width > config.maxObjectSize(0) ||
            box.y_width > config.maxObjectSize(1) ||
            box.z_width > config.maxObjectSize(2))
        {
            continue;
        }
        bboxes.push_back(box);
        pcClusters.push_back(pcClustersTemp[i]);
    }

    for (const auto& cluster : pcClusters) {
        Eigen::Vector3d center(0.0, 0.0, 0.0);
        Eigen::Vector3d std(0.0, 0.0, 0.0);
        onboardDetector::calcPcFeat(cluster, center, std);
        pcClusterCenters.push_back(center);
        pcClusterStds.push_back(std);
    }
}

std::vector<box3D> transformUVBBoxes(const UVdetector& uvDetector, const DetectionConfig& config)
{
    std::vector<box3D> bboxes;
    bboxes.reserve(uvDetector.box3Ds.size());
    for (std::size_t i = 0; i < uvDetector.box3Ds.size(); ++i) {
        box3D bbox;
        const auto& uvBox = uvDetector.box3Ds[i];
        Eigen::Vector3d center(uvBox.x, uvBox.y, uvBox.z);
        Eigen::Vector3d size(uvBox.x_width, uvBox.y_width, uvBox.z_width);
        Eigen::Vector3d newCenter, newSize;
        transformBBox(center, size, config.positionDepth, config.orientationDepth, newCenter, newSize);
        bbox.x = newCenter(0);
        bbox.y = newCenter(1);
        bbox.z = newCenter(2);
        bbox.x_width = newSize(0);
        bbox.y_width = newSize(1);
        bbox.z_width = newSize(2);
        bbox.id = static_cast<double>(i);
        bboxes.push_back(bbox);
    }
    return bboxes;
}

}  // namespace

DetectionOutput runDetection(const DetectionInput& input)
{
    DetectionOutput output;
    if (input.depthImageMm.empty()) {
        return output;
    }

    output.projectedDepthPoints = projectDepthImage(input);
    output.filteredDepthPoints = filterPoints(output.projectedDepthPoints, input.config);
    clusterPointsAndBBoxes(
        output.filteredDepthPoints,
        input.config,
        output.dbBBoxes,
        output.dbClusters,
        output.dbClusterCenters,
        output.dbClusterStds);

    UVdetector uv;
    uv.fx = static_cast<float>(input.config.fx);
    uv.fy = static_cast<float>(input.config.fy);
    uv.px = static_cast<float>(input.config.cx);
    uv.py = static_cast<float>(input.config.cy);
    // input.depthImageMm is already normalized to uint16 millimeters in runtime_bridge.
    // Keep UV detector depth scale fixed to 1.0 to avoid double scaling semantics.
    constexpr double kMmPerMeter = 1000.0;
    uv.depthScale_ = 1.0f;
    uv.max_dist = static_cast<int>(std::max(1.0, input.config.raycastMaxLength * kMmPerMeter));
    uv.min_dist = static_cast<int>(std::max(0.0, input.config.depthMinValue * kMmPerMeter));
    uv.row_downsample = std::max(1, input.config.uMapRowDownsample);
    uv.col_scale = input.config.uMapColScale;
    uv.threshold_point = input.config.uMapThresholdPoint;
    uv.threshold_line = input.config.uMapThresholdLine;
    uv.min_length_line = input.config.uMapMinLengthLine;
    uv.depth = input.depthImageMm;
    uv.detect();
    uv.extract_3Dbox();
    uv.display_depth();
    uv.display_U_map();
    uv.display_bird_view();
    output.uvBBoxes = transformUVBBoxes(uv, input.config);
    output.depthShow = uv.depth_show.clone();
    output.uMapShow = uv.U_map_show.clone();
    output.birdView = uv.bird_view.clone();
    return output;
}

}  // namespace onboardDetector
