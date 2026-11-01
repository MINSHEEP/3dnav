#pragma once

namespace icp
{
struct ValidationLimits
{
  double rmse;
  double vertical;
};

inline ValidationLimits validationLimits(
  bool robust, bool tracking, ValidationLimits original, ValidationLimits continuous)
{
  return robust && tracking ? continuous : original;
}
}  // namespace icp
