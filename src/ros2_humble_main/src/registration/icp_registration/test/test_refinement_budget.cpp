#include "icp_registration/refinement_budget.hpp"
#undef NDEBUG
#include <cassert>
#include <initializer_list>

int main()
{
  const icp::RefinementBudget legacy{6, 6, 4}, initial{20, 40, 12};
  const auto full = icp::refinementBudget(true, false, legacy, initial);
  assert(full.ndt == 20 && full.gicp == 40 && full.optimizer == 12);
  for (bool tracking : {false, true}) {
    const auto original = icp::refinementBudget(false, tracking, legacy, initial);
    assert(original.ndt == 6 && original.gicp == 6 && original.optimizer == 4);
  }
  const auto running = icp::refinementBudget(true, true, legacy, initial);
  assert(running.ndt == 6 && running.gicp == 6 && running.optimizer == 4);
  assert(icp::iterationCeiling(true, 6, 6));
  assert(icp::iterationCeiling(true, 40, 40));
  assert(!icp::iterationCeiling(true, 5, 6));
  assert(!icp::iterationCeiling(false, 0, 6));
}
