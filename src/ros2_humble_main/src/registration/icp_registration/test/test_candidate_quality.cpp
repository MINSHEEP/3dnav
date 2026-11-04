#include "icp_registration/candidate_quality.hpp"
#undef NDEBUG
#include <cassert>

int main()
{
  assert(icp::candidateCost(.12, .8) < icp::candidateCost(.15, .6));
  assert(icp::ambiguousCandidate(.16, .17, 1.7, .92));
  assert(!icp::ambiguousCandidate(.16, .4, 1.7, .92));
  assert(!icp::ambiguousCandidate(.16, .16, .02, .01));
  assert(icp::ambiguousCandidate(.16, .16, 0., .3));
}
