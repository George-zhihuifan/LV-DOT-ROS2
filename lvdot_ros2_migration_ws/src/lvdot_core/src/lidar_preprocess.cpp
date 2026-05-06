#include "lvdot_core/lidar_preprocess.hpp"

#include <cstdlib>
#include <cmath>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace onboardDetector {

std::vector<ClusterPoint> projectLidarPoints(
    const std::vector<Eigen::Vector3d>& bodyPoints,
    const LidarPreprocessConfig& config)
{
    std::vector<ClusterPoint> preTransformPoints;
    preTransformPoints.reserve(bodyPoints.size());

    for (const auto& bodyPoint : bodyPoints) {
        if (!std::isfinite(bodyPoint.x()) ||
            !std::isfinite(bodyPoint.y()) ||
            !std::isfinite(bodyPoint.z()))
        {
            continue;
        }

        if (std::abs(bodyPoint.x()) > config.localSensorRange.x() ||
            std::abs(bodyPoint.y()) > config.localSensorRange.y())
        {
            continue;
        }

        // Sensor-frame near-field rejection for self returns.
        if (bodyPoint.norm() < 0.42) {
            continue;
        }

        const Eigen::Vector3d pointBody =
            config.sensorRotation * bodyPoint + config.sensorTranslation;

        // Body-frame platform bbox rejection (body + arms).
        if (std::abs(pointBody.x()) < 0.36 &&
            std::abs(pointBody.y()) < 0.36 &&
            pointBody.z() > -0.16 &&
            pointBody.z() < 0.28)
        {
            continue;
        }

        preTransformPoints.push_back(ClusterPoint{pointBody, pointBody.norm(), 0, 0, false});
    }

    std::vector<ClusterPoint> points;
    points.reserve(preTransformPoints.size());
    for (const auto& sample : preTransformPoints) {
        const Eigen::Vector3d pointWorld =
            config.orientationBody * sample.point + config.position;

        // World-frame fallback self rejection to tolerate frame convention drift.
        const Eigen::Vector3d from_uav = pointWorld - config.position;
        if (from_uav.norm() < 0.45) {
            continue;
        }

        if (pointWorld.z() < config.groundHeight || pointWorld.z() > config.roofHeight) {
            continue;
        }

        ClusterPoint worldSample;
        worldSample.point = pointWorld;
        worldSample.depth = sample.depth;
        worldSample.u = 0;
        worldSample.v = 0;
        worldSample.has_image_point = false;
        points.push_back(worldSample);
    }

    return points;
}

std::vector<ClusterPoint> downsampleLidarPoints(
    const std::vector<ClusterPoint>& points,
    const LidarPreprocessConfig& config)
{
    if (points.empty()) {
        return {};
    }

    if (static_cast<int>(points.size()) <= config.downsampleThreshold) {
        return points;
    }

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(points.size());
    for (const auto& sample : points) {
        pcl::PointXYZ point;
        point.x = static_cast<float>(sample.point.x());
        point.y = static_cast<float>(sample.point.y());
        point.z = static_cast<float>(sample.point.z());
        cloud->push_back(point);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);

    auto downsampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    sor.filter(*downsampled);

    while (static_cast<int>(downsampled->size()) > config.downsampleThreshold) {
        const float leaf = sor.getLeafSize().x() * 1.1f;
        sor.setLeafSize(leaf, leaf, leaf);
        sor.filter(*downsampled);
    }

    std::vector<ClusterPoint> filtered;
    filtered.reserve(downsampled->size());
    for (const auto& point : downsampled->points) {
        ClusterPoint sample;
        sample.point = Eigen::Vector3d(point.x, point.y, point.z);
        sample.depth = sample.point.norm();
        sample.u = 0;
        sample.v = 0;
        sample.has_image_point = false;
        filtered.push_back(sample);
    }
    return filtered;
}

}  // namespace onboardDetector
