#include "icp_registration/tracking_validation.hpp"
#undef NDEBUG
#include <cassert>
#include <initializer_list>

int main()
{
  const icp::ValidationLimits original{.20, .35}, continuous{.22, .45};
  const auto running = icp::validationLimits(true, true, original, continuous);
  // Actual full-3D candidates rejected at 20:43:29 in the supplied trace.
  for (double measured_z : {.357470, .397020, .347729}) {
    assert(measured_z <= running.vertical);
  }
  assert(.219 <= running.rmse);
  assert(.221 > running.rmse);
  assert(.451 > running.vertical);
  for (bool robust : {false, true}) {
    for (bool tracking : {false, true}) {
      if (robust && tracking) {continue;}
      const auto legacy = icp::validationLimits(robust, tracking, original, continuous);
      assert(legacy.rmse == .20 && legacy.vertical == .35);
      assert(.357470 > legacy.vertical);
    }
  }
}
