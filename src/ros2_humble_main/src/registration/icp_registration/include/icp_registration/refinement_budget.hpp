#pragma once

namespace icp
{
struct RefinementBudget
{
  int ndt;
  int gicp;
  int optimizer;
};

inline RefinementBudget refinementBudget(
  bool robust, bool tracking, RefinementBudget legacy, RefinementBudget initial)
{
  return robust && !tracking ? initial : legacy;
}

// PCL 1.12 GICP sets converged_ on either a small delta OR its iteration
// ceiling. Do not present the latter as proven numerical convergence.
inline bool iterationCeiling(bool reported_converged, int used, int limit)
{
  return reported_converged && used >= limit;
}
}  // namespace icp
