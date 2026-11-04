#include "icp_registration/target_sampling.hpp"
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstddef>

int main()
{
  // shiyanshi-wai PCD, near recorded last stop (-37.083, 19.393).
  // 14,589 real ROI points -> 7,280 fine points, but only 1,163 at 45 cm.
  // The 1,200-point gate must be met by preserving more measured points.
  auto sampler = [](bool rough, double leaf, std::size_t maximum) {
      const std::size_t count = !rough ? 7280U : (leaf > 0.34 ? 1163U : 1944U);
      return std::min(count, maximum);
    };
  const auto legacy = icp::prepareTargetSampling(
    false, 14589U, .10, .15, .45, 1200U, 30000U, 18000U, sampler);
  assert(!legacy.ready && legacy.rough_count == 1163U);
  const auto rigid = icp::prepareTargetSampling(
    true, 14589U, .10, .15, .45, 1200U, 30000U, 18000U, sampler);
  assert(rigid.ready);
  assert(rigid.fine_count == 7280U && rigid.rough_count >= 1200U);
  assert(rigid.rough_leaf < .45 && rigid.rough_leaf >= rigid.fine_leaf);
  assert(rigid.initial_rough_count == 1163U && rigid.refinements == 1U);

  // A healthy coarse cloud costs no extra sampling, for either mode.
  for (bool adaptive : {false, true}) {
    int calls = 0;
    const auto healthy = icp::prepareTargetSampling(
      adaptive, 10000U, .1, .15, .45, 1200U, 30000U, 18000U,
      [&](bool, double, std::size_t) {++calls; return 1500U;});
    assert(healthy.ready && healthy.refinements == 0U && calls == 2);
  }

  // Final quality gates cannot be replaced by repeated coarse sampling.
  int calls = 0;
  const auto missing = icp::prepareTargetSampling(
    true, 1199U, .1, .15, .45, 1200U, 30000U, 18000U,
    [&](bool, double, std::size_t) {++calls; return 1500U;});
  assert(!missing.ready && calls == 0);
  const auto sparse_fine = icp::prepareTargetSampling(
    true, 1500U, .1, .15, .45, 1200U, 30000U, 18000U,
    [](bool, double, std::size_t) {return 1100U;});
  assert(!sparse_fine.ready && sparse_fine.refinements == 0U);

  // Some geometry stays too sparse even at the floor. No raw-cloud bypass.
  const auto sparse_coarse = icp::prepareTargetSampling(
    true, 1500U, .1, .15, .45, 1200U, 30000U, 18000U,
    [](bool rough, double, std::size_t) {return rough ? 1100U : 1300U;});
  assert(!sparse_coarse.ready && sparse_coarse.refinements == 3U);
  assert(std::abs(sparse_coarse.rough_leaf - .15) < 1e-9);
  const auto map_floor = icp::prepareTargetSampling(
    true, 1500U, .4, .15, .45, 1200U, 30000U, 18000U,
    [](bool rough, double leaf, std::size_t) {
      assert(leaf >= .4);
      return rough ? 1100U : 1300U;
    });
  assert(!map_floor.ready && map_floor.refinements == 1U);
  assert(map_floor.rough_leaf == .4);

  // Every attempt uses the original point caps, including the last attempt.
  const auto capped = icp::prepareTargetSampling(
    true, 12000U, .1, .15, .45, 1200U, 5000U, 1800U,
    [](bool rough, double leaf, std::size_t maximum) {
      assert(maximum == (rough ? 1800U : 5000U));
      return rough && leaf > .34 ? 1163U : maximum;
    });
  assert(capped.ready && capped.rough_count == 1800U);
}
