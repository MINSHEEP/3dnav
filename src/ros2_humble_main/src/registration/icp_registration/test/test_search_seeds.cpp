#include "icp_registration/search_seeds.hpp"
#undef NDEBUG
#include <cassert>
#include <limits>

int main()
{
  constexpr double pi = 3.14159265358979323846;
  const auto seeds = icp::makeSearchSeeds(3.0, pi / 2.0, 5);
  assert(seeds.size() == 45);
  bool north = false, diagonal = false, intermediate_yaw = false;
  for (const auto & seed : seeds) {
    assert(std::hypot(seed.x, seed.y) <= 3.0 + 1e-9);
    assert(std::abs(seed.yaw) <= pi / 2.0);
    north |= std::abs(seed.x) < 1e-9 && std::abs(seed.y - 3.0) < 1e-9;
    diagonal |= seed.x > 2.0 && seed.y > 2.0;
    intermediate_yaw |= std::abs(seed.yaw - pi / 4.0) < 1e-9;
  }
  assert(north && diagonal && intermediate_yaw);
  // Simulate a good joint position + heading seed away from the rough pose.
  std::vector<double> scores;
  for (const auto & seed : seeds) {
    scores.push_back(std::pow(seed.x - 3.0, 2) + std::pow(seed.y, 2) +
      std::pow(seed.yaw - pi / 2.0, 2));
  }
  const auto selected = icp::shortlistSeeds(scores, 9);
  assert(selected.size() == 9 && selected[0] == 0);
  assert(scores[selected[1]] < 1e-9);
  auto tied = icp::shortlistSeeds({3.0, 1.0, 1.0, std::numeric_limits<double>::quiet_NaN()}, 3);
  assert((tied == std::vector<std::size_t>{0, 1, 2}));
  assert(icp::shortlistSeeds({}, 9).empty());
  assert(icp::shortlistSeeds({1}, 0).empty());
}
