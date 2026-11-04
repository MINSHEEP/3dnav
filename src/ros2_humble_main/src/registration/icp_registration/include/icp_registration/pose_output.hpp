#pragma once
#include <array>
#include <cmath>

namespace icp
{
inline constexpr const char * kRigidTransformModel = "se3_map_odom_v1";

struct PoseOutput
{
  double z;
  std::array<double, 4> rotation;
};

inline PoseOutput poseOutput(
  bool rigid, double z, std::array<double, 4> rotation, double yaw)
{
  if (rigid) {return {z, rotation};}
  return {0.0, {0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)}};
}

inline bool ambiguousRigidCandidate(
  double best_cost, double other_cost, double height_distance, double tilt_distance)
{
  return (height_distance > 0.25 || tilt_distance > 0.1396263402) &&
         other_cost <= best_cost * 1.15 + 0.01;
}
}  // namespace icp
