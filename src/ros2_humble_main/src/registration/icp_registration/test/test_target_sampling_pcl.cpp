#include "icp_registration/target_sampling.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#undef NDEBUG
#include <cassert>

int main()
{
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  Cloud::Ptr roi(new Cloud);
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 50; ++y) {
      pcl::PointXYZ point;
      point.x = -.1F + x * .16F;
      point.y = -2.F + y * .16F;
      point.z = .1F;
      roi->push_back(point);
    }
  }
  Cloud::Ptr rough, fine;
  auto sampler = [&](bool coarse, double leaf, std::size_t maximum) {
      Cloud::Ptr sampled(new Cloud);
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      const auto size = static_cast<float>(leaf);
      filter.setLeafSize(size, size, size);
      filter.setInputCloud(roi);
      filter.filter(*sampled);
      assert(sampled->size() <= maximum);
      if (coarse) {rough = sampled;} else {fine = sampled;}
      return sampled->size();
    };
  const auto legacy = icp::prepareTargetSampling(
    false, roi->size(), .1, .15, .45, 1200U, 30000U, 18000U, sampler);
  assert(!legacy.ready && legacy.fine_count == 1500U);
  const auto result = icp::prepareTargetSampling(
    true, roi->size(), .1, .15, .45, 1200U, 30000U, 18000U, sampler);
  assert(result.ready && result.refinements <= 3U);
  assert(rough->size() >= 1200U && fine->size() == 1500U);
  assert(result.rough_leaf >= .15);
}
