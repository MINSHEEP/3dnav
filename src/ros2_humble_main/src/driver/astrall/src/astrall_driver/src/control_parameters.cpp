// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <cctype>
#include <cmath>

#include "astrall_driver/driver_parameters.hpp"

namespace astrall_driver {

std::string DriverParameters::ValidateControl() const {
  constexpr int64_t kMinPeriodMs = 10;
  constexpr int64_t kMaxPeriodMs = 25;
  constexpr int64_t kMaxCallMs = kMaxSdkTimeoutMs;
  constexpr int64_t kMaxWatchdogMs = 5000;
  constexpr int64_t kMinModeTimeoutMs = 100;
  constexpr int64_t kMaxModeTimeoutMs = 30000;
  if (control_period_ms_ < kMinPeriodMs || control_period_ms_ > kMaxPeriodMs ||
      control_sdk_timeout_ms_ < 1 || control_sdk_timeout_ms_ > kMaxCallMs ||
      control_sdk_timeout_ms_ > control_period_ms_ ||
      cmd_vel_timeout_ms_ < control_period_ms_ ||
      cmd_vel_timeout_ms_ > kMaxWatchdogMs) {
    return "control period must be 10..25ms, motion SDK calls 1..20ms "
           "within control period, watchdog at "
           "least control period and at most 5000ms";
  }
  for (auto limit :
      { max_forward_velocity_, max_lateral_velocity_, max_yaw_velocity_ }) {
    if (!std::isfinite(limit) || limit < 0 || limit > 1) {
      return "velocity limits must be finite and in [0,1], manual 5.14";
    }
  }
  if (mode_poll_period_ms_ < control_period_ms_ || mode_poll_period_ms_ > 100 ||
      mode_state_freshness_ms_ < mode_poll_period_ms_ ||
      mode_state_freshness_ms_ > 1000 ||
      mode_switch_timeout_ms_ < kMinModeTimeoutMs ||
      mode_switch_timeout_ms_ > kMaxModeTimeoutMs) {
    return "mode poll must fit freshness, be at least control_period_ms "
           "and at most 100ms; "
           "switch timeout must be 100..30000ms";
  }
  if (!std::isfinite(text_rate_hz_) || text_rate_hz_ < 0.1 ||
      text_rate_hz_ >= 5) {
    return "text_rate_hz must be finite and in [0.1,5), manual 5.11";
  }
  if (cmd_vel_topic_.empty() || !std::all_of(cmd_vel_topic_.begin(),
      cmd_vel_topic_.end(), [](unsigned char value) {
      return std::isalnum(value) || value == '_' || value == '/';
    })) {
    return "cmd_vel_topic must be a nonempty plain ROS topic name";
  }
  return {};
}

}  // namespace astrall_driver
