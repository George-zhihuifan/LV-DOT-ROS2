/*
 * FILE: fusion_filter.hpp
 * -------------------------------------------------------------------------
 * Core port of the ROS1 LV-DOT dynamicDetector::filterLVBBoxes() stage.
 * The ROS wrapper should only convert runtime messages into these plain C++
 * inputs/outputs and call this module, rather than reimplementing the logic.
 */
#ifndef LVDOT_CORE_FUSION_FILTER_HPP
#define LVDOT_CORE_FUSION_FILTER_HPP

#include <vector>

#include <Eigen/Eigen>

#include "lvdot_core/box3d.hpp"

namespace onboardDetector {

struct ClusterPoint
{
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    double depth{0.0};
    int u{0};
    int v{0};
    bool has_image_point{false};
};

struct ImageBBox2D
{
    int x{0};
    int y{0};
    int width{0};
    int height{0};
    bool is_human{false};
    double score{0.0};
};

struct FilterLVBBoxesStats
{
    std::size_t uv_input_count{0};
    std::size_t db_input_count{0};
    std::size_t uv_best_match_count{0};
    std::size_t db_best_match_count{0};
    std::size_t uv_db_mutual_match_count{0};
    std::size_t uv_no_db_candidate_count{0};
    std::size_t uv_not_mutual_count{0};
    std::size_t uv_mutual_iou_reject_count{0};
    std::size_t fusion_component_count{0};
    std::size_t visual_only_component_count{0};
    std::size_t lidar_only_component_count{0};
    std::size_t split_source_boxes{0};
    std::size_t split_success_boxes{0};
    std::size_t split_output_boxes{0};
    std::size_t yolo_input_count{0};
    std::size_t yolo_candidate_3d_count{0};
    std::size_t yolo_matched_3d_count{0};
    std::size_t yolo_matched_detection_count{0};
    std::size_t yolo_human_marked_count{0};
    std::size_t yolo_fused_used_count{0};
};

struct FilterLVBBoxesInput
{
    std::vector<box3D> uvBBoxes;
    std::vector<box3D> dbBBoxes;
    std::vector<std::vector<ClusterPoint>> pcClustersVisual;
    std::vector<Eigen::Vector3d> pcClusterCentersVisual;
    std::vector<Eigen::Vector3d> pcClusterStdsVisual;

    std::vector<box3D> lidarBBoxes;
    std::vector<std::vector<ClusterPoint>> lidarPcClusters;
    std::vector<Eigen::Vector3d> lidarPcClusterCenters;
    std::vector<Eigen::Vector3d> lidarPcClusterStds;

    std::vector<ImageBBox2D> yoloDetectionResults;

    Eigen::Vector3d positionColor{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d orientationColor{Eigen::Matrix3d::Identity()};
    double fxC{0.0};
    double fyC{0.0};
    double cxC{0.0};
    double cyC{0.0};
    double boxIOUThresh{0.0};
};

struct FilterLVBBoxesOutput
{
    std::vector<box3D> visualBBoxes;
    std::vector<std::vector<ClusterPoint>> visualPcClusters;
    std::vector<Eigen::Vector3d> visualPcClusterCenters;
    std::vector<Eigen::Vector3d> visualPcClusterStds;

    std::vector<box3D> filteredBBoxesBeforeYolo;
    std::vector<std::vector<ClusterPoint>> filteredPcClustersBeforeYolo;
    std::vector<Eigen::Vector3d> filteredPcClusterCentersBeforeYolo;
    std::vector<Eigen::Vector3d> filteredPcClusterStdsBeforeYolo;

    std::vector<box3D> filteredBBoxes;
    std::vector<std::vector<ClusterPoint>> filteredPcClusters;
    std::vector<Eigen::Vector3d> filteredPcClusterCenters;
    std::vector<Eigen::Vector3d> filteredPcClusterStds;

    FilterLVBBoxesStats stats;
};

Eigen::Vector3d computeCenter(const std::vector<ClusterPoint>& points);
Eigen::Vector3d computeStd(const std::vector<ClusterPoint>& points, const Eigen::Vector3d& center);
void calcPcFeat(const std::vector<ClusterPoint>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd);
double calBoxIOU(const box3D& box1, const box3D& box2, bool ignoreZmin = false);
int getBestOverlapBBox(const box3D& currBBox, const std::vector<box3D>& targetBBoxes, double& bestIOU);
void transformBBox(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    const Eigen::Matrix3d& orientation,
    Eigen::Vector3d& newCenter,
    Eigen::Vector3d& newSize);

FilterLVBBoxesOutput filterLVBBoxes(const FilterLVBBoxesInput& input);

}  // namespace onboardDetector

#endif  // LVDOT_CORE_FUSION_FILTER_HPP
