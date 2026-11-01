#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace icp
{
struct SearchSeed
{
  double x;
  double y;
  double yaw;
};

// Offsets only: the caller composes these with the operator's rough pose.
inline std::vector<SearchSeed> makeSearchSeeds(double radius, double yaw, int yaw_count)
{
  std::vector<double> angles{0.0};
  if (yaw_count >= 2) {angles.push_back(-yaw);}
  if (yaw_count >= 3) {angles.push_back(yaw);}
  if (yaw_count >= 4) {angles.push_back(-yaw * 0.5);}
  if (yaw_count >= 5) {angles.push_back(yaw * 0.5);}
  std::vector<SearchSeed> seeds;
  for (double angle : angles) {seeds.push_back({0.0, 0.0, angle});}
  constexpr double pi = 3.14159265358979323846;
  for (int direction = 0; direction < 8; ++direction) {
    const double bearing = direction * pi / 4.0;
    for (double angle : angles) {
      seeds.push_back({radius * std::cos(bearing), radius * std::sin(bearing), angle});
    }
  }
  return seeds;
}

// A cheap score ranks seeds, never accepts a pose. Keep the operator's seed
// even when sampling noise ranks it poorly; bound expensive NDT evaluations.
inline std::vector<std::size_t> shortlistSeeds(
  const std::vector<double> & scores, std::size_t maximum)
{
  if (scores.empty() || maximum == 0) {return {};}
  std::vector<std::size_t> indices(scores.size() - 1);
  std::iota(indices.begin(), indices.end(), 1U);
  const auto score = [&scores](std::size_t i) {
      return std::isfinite(scores[i]) ? scores[i] : HUGE_VAL;
    };
  std::stable_sort(indices.begin(), indices.end(), [&](std::size_t a, std::size_t b) {
    return score(a) < score(b);
  });
  indices.resize(std::min(indices.size(), maximum - 1));
  indices.insert(indices.begin(), 0U);
  return indices;
}
}  // namespace icp
