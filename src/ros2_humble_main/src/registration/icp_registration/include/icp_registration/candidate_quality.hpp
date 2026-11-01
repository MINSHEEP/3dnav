#pragma once
#include <algorithm>
#include <cmath>

namespace icp
{
inline double candidateCost(double rmse, double overlap)
{
  return rmse + 0.2 * (1.0 - overlap);
}
inline bool ambiguousCandidate(
  double best_cost, double other_cost, double distance, double yaw_distance)
{
  return (distance > 0.35 || yaw_distance > 0.2617993878) &&
         other_cost <= best_cost * 1.15 + 0.01;
}
}  // namespace icp
