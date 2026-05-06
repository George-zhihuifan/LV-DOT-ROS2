/*
 * FILE: box3d.hpp
 * -------------------------------------------------------------------------
 * Ported verbatim from the ROS1 LV-DOT onboard_detector/utils.h.
 * Removed tf2 / geometry_msgs helpers so the struct has no ROS dependency
 * and can be shared between ROS1 and ROS2 wrappers.
 */
#ifndef LVDOT_CORE_BOX3D_HPP
#define LVDOT_CORE_BOX3D_HPP

#include <vector>

#include <Eigen/Eigen>

namespace onboardDetector {

constexpr double PI_const = 3.1415926;

struct box3D
{
    double x{0.0}, y{0.0}, z{0.0};
    double x_width{0.0}, y_width{0.0}, z_width{0.0};
    double id{0.0};
    double Vx{0.0}, Vy{0.0}, Vz{0.0};
    double Ax{0.0}, Ay{0.0}, Az{0.0};
    bool is_human{false};
    bool is_dynamic{false};
    bool fix_size{false};
    bool is_dynamic_candidate{false};
    bool is_estimated{false};
    bool is_u_map_enhanced{false};
};

inline double angleBetweenVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

inline Eigen::Vector3d computeCenter(const std::vector<Eigen::Vector3d>& points)
{
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    if (points.empty()) {
        return center;
    }
    for (const auto& p : points) {
        center += p;
    }
    center /= static_cast<double>(points.size());
    return center;
}

inline Eigen::Vector3d computeStd(const std::vector<Eigen::Vector3d>& points,
                                  const Eigen::Vector3d& center)
{
    Eigen::Vector3d stds(0.0, 0.0, 0.0);
    if (points.empty()) {
        return stds;
    }
    for (const auto& p : points) {
        Eigen::Vector3d diff = p - center;
        stds(0) += diff(0) * diff(0);
        stds(1) += diff(1) * diff(1);
        stds(2) += diff(2) * diff(2);
    }
    stds /= static_cast<double>(points.size());
    stds = stds.array().sqrt();
    return stds;
}

}  // namespace onboardDetector

#endif  // LVDOT_CORE_BOX3D_HPP
