/*
 * FILE: lidar_detector.hpp
 * -------------------------------------------------------------------------
 * Ported from the ROS1 LV-DOT onboard_detector/lidarDetector.h.
 * The single `<ros/ros.h>` include was removed because the header did not
 * use any ROS API; only the includes were changed, the class API is kept
 * verbatim.
 */
#ifndef LVDOT_CORE_LIDAR_DETECTOR_HPP
#define LVDOT_CORE_LIDAR_DETECTOR_HPP

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>

#include "lvdot_core/box3d.hpp"
#include "lvdot_core/dbscan.hpp"

namespace onboardDetector {

struct Cluster
{
    int cluster_id;
    Eigen::Vector4f centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;

    Eigen::Vector3f dimensions;
    Eigen::Matrix3f eigen_vectors;
    Eigen::Vector3f eigen_values;

    Cluster()
        : cluster_id(-1),
          centroid(Eigen::Vector4f::Zero()),
          points(new pcl::PointCloud<pcl::PointXYZ>()) {}
};

class lidarDetector
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::vector<onboardDetector::Cluster> clusters_;
    std::vector<onboardDetector::box3D> bboxes_;
    double eps_;
    int minPts_;
    double groundHeight_;
    double roofHeight_;

public:
    lidarDetector();
    void setParams(double eps, int minPts);
    void getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void lidarDBSCAN();
    std::vector<onboardDetector::Cluster>& getClusters();
    std::vector<onboardDetector::box3D>& getBBoxes();
};

}  // namespace onboardDetector

#endif  // LVDOT_CORE_LIDAR_DETECTOR_HPP
