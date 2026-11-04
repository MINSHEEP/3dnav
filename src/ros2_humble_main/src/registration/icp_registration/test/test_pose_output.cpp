#include "icp_registration/pose_output.hpp"
#undef NDEBUG
#include <cassert>

int main()
{
  // Recorded valid full-3D candidate before the old projection rejection.
  const std::array<double, 4> q{{-0.012, -0.013, -0.999, 0.026}};
  const auto rigid = icp::poseOutput(true, 0.434135, q, -3.089208062);
  assert(rigid.z == 0.434135 && rigid.rotation == q);
  const auto legacy = icp::poseOutput(false, 0.434135, q, -3.089208062);
  assert(legacy.z == 0.0 && legacy.rotation[0] == 0.0 && legacy.rotation[1] == 0.0);
  assert(icp::ambiguousRigidCandidate(.16, .17, .5, 0.0));
  assert(!icp::ambiguousRigidCandidate(.16, .4, .5, 0.0));
  assert(!icp::ambiguousRigidCandidate(.16, .17, .02, 0.0));
  assert(icp::ambiguousRigidCandidate(.16, .17, .02, 0.20));
  assert(!icp::ambiguousRigidCandidate(.16, .17, .02, 0.01));
}
