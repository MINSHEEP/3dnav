// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astrall_driver {
namespace {

bool Finite(VelocityCommand command) {
  return std::isfinite(command.forward_) && std::isfinite(command.lateral_) &&
         std::isfinite(command.yaw_);
}

}  // namespace

VelocityController::VelocityController(VelocityCommand limits,
    std::chrono::milliseconds watchdog) : limits_(limits), watchdog_(watchdog) {
  constexpr double kSdkMaximum = 1;
  if (!Finite(limits) || limits.forward_ < 0 || limits.forward_ > kSdkMaximum ||
      limits.lateral_ < 0 || limits.lateral_ > kSdkMaximum || limits.yaw_ < 0 ||
      limits.yaw_ > kSdkMaximum || watchdog.count() <= 0) {
    throw std::invalid_argument("Invalid velocity limits/watchdog");
  }
}

bool VelocityController::Accept(VelocityCommand command, Time now,
    bool allowed) {
  if (!Finite(command) || !allowed) {
    Clear();
    return false;
  }
  command_ =
  { std::clamp(command.forward_, -limits_.forward_, limits_.forward_),
    std::clamp(command.lateral_, -limits_.lateral_,
        limits_.lateral_),
    std::clamp(command.yaw_, -limits_.yaw_, limits_.yaw_) };
  received_ = now;
  has_command_ = true;
  zero_needed_ = false;
  ++revision_;
  return true;
}

VelocitySample VelocityController::Sample(Time now, bool allowed) {
  const bool expired = has_command_ && now - received_ >= watchdog_;
  if (has_command_ && (!allowed || expired)) {
    Clear();
  }
  return { command_, revision_, has_command_ || zero_needed_, expired };
}

void VelocityController::Clear() {
  if (has_command_ || !zero_needed_) {
    ++revision_;
  }
  command_ = {};
  has_command_ = false;
  zero_needed_ = true;
}

void VelocityController::MarkSent(uint64_t revision) {
  if (revision == revision_ && !has_command_) {
    zero_needed_ = false;
  }
}

bool VelocityController::HasCommand() const {
  return has_command_;
}

}  // namespace astrall_driver
