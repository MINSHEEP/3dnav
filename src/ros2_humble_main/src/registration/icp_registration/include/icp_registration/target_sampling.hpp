#pragma once

#include <algorithm>
#include <cstddef>
#include <initializer_list>

namespace icp
{
inline constexpr const char * kTargetSamplingModel = "adaptive_target_sampling_v1";

struct TargetSampling
{
  std::size_t raw_count{0U};
  std::size_t fine_count{0U};
  std::size_t rough_count{0U};
  std::size_t initial_rough_count{0U};
  double fine_leaf{0.0};
  double rough_leaf{0.0};
  std::size_t refinements{0U};
  bool ready{false};
};

// The caller supplies the real voxel filter and point cap. Keeping the choice
// here lets portable tests drive the same preparation used by the PCL node.
template<typename Sampler>
TargetSampling prepareTargetSampling(
  bool adaptive, std::size_t raw_count, double map_leaf,
  double fine_leaf, double rough_leaf, std::size_t minimum_points,
  std::size_t maximum_fine, std::size_t maximum_rough, Sampler sample)
{
  TargetSampling result;
  result.raw_count = raw_count;
  result.fine_leaf = std::max(fine_leaf, map_leaf);
  result.rough_leaf = std::max(rough_leaf, map_leaf);
  if (raw_count < minimum_points) {return result;}
  result.fine_count = sample(false, result.fine_leaf, maximum_fine);
  result.rough_count = sample(true, result.rough_leaf, maximum_rough);
  result.initial_rough_count = result.rough_count;
  const auto minimum_rough = std::min(minimum_points, maximum_rough);
  // Do not lower the count gate or enlarge the search region. Preserve more
  // real target geometry when the coarse grid alone made a valid ROI sparse.
  // At most three retries, never denser than the fine/map voxel floor.
  if (adaptive && result.fine_count >= minimum_points) {
    for (double scale : {0.75, 0.5, 0.0}) {
      if (result.rough_count >= minimum_rough) {break;}
      const double leaf = std::max(result.fine_leaf, rough_leaf * scale);
      if (leaf >= result.rough_leaf) {continue;}
      result.rough_leaf = leaf;
      result.rough_count = sample(true, leaf, maximum_rough);
      ++result.refinements;
    }
  }
  result.ready = result.fine_count >= minimum_points &&
    result.rough_count >= minimum_rough;
  return result;
}
}  // namespace icp
