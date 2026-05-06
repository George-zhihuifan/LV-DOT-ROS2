#pragma once

#include "lvdot_core/box3d.hpp"

namespace lvdot_ros2
{

struct Box3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double x_width{0.0};
  double y_width{0.0};
  double z_width{0.0};

  double id{0.0};

  double vx{0.0};
  double vy{0.0};
  double vz{0.0};

  double ax{0.0};
  double ay{0.0};
  double az{0.0};

  bool is_human{false};
  bool is_dynamic{false};
  bool fix_size{false};
  bool is_dynamic_candidate{false};
  bool is_estimated{false};
  bool is_u_map_enhanced{false};
};

inline onboardDetector::box3D to_core_box3d(const Box3D & src)
{
  onboardDetector::box3D dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  dst.x_width = src.x_width;
  dst.y_width = src.y_width;
  dst.z_width = src.z_width;
  dst.id = src.id;
  dst.Vx = src.vx;
  dst.Vy = src.vy;
  dst.Vz = src.vz;
  dst.Ax = src.ax;
  dst.Ay = src.ay;
  dst.Az = src.az;
  dst.is_human = src.is_human;
  dst.is_dynamic = src.is_dynamic;
  dst.fix_size = src.fix_size;
  dst.is_dynamic_candidate = src.is_dynamic_candidate;
  dst.is_estimated = src.is_estimated;
  dst.is_u_map_enhanced = src.is_u_map_enhanced;
  return dst;
}

inline Box3D from_core_box3d(const onboardDetector::box3D & src, bool is_u_map_enhanced = false)
{
  Box3D dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  dst.x_width = src.x_width;
  dst.y_width = src.y_width;
  dst.z_width = src.z_width;
  dst.id = src.id;
  dst.vx = src.Vx;
  dst.vy = src.Vy;
  dst.vz = src.Vz;
  dst.ax = src.Ax;
  dst.ay = src.Ay;
  dst.az = src.Az;
  dst.is_human = src.is_human;
  dst.is_dynamic = src.is_dynamic;
  dst.fix_size = src.fix_size;
  dst.is_dynamic_candidate = src.is_dynamic_candidate;
  dst.is_estimated = src.is_estimated;
  dst.is_u_map_enhanced = src.is_u_map_enhanced || is_u_map_enhanced;
  return dst;
}

}  // namespace lvdot_ros2
