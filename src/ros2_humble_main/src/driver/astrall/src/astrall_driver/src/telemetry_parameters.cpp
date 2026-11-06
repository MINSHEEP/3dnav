// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <array>
#include <cctype>

#include "astrall_driver/driver_parameters.hpp"
#include "astrall_driver/subscription_frequency.hpp"

namespace astrall_driver {
namespace {

constexpr int64_t kMaxTelemetryPeriodMs = 1000;
constexpr int64_t kMinQueryPeriodMs = 100;
constexpr int64_t kMaxSampleAgeMs = 5000;

bool PlainName(const std::string& value, bool frame) {
  if (value.empty() || (frame && value.front() == '/') || value.back() == '/') {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isalnum(character) || character == '_' || character == '/';
      });
}

}  // namespace

std::string DriverParameters::ValidateTelemetry() const {
  if (!SubscriptionFrequency(imu_frequency_hz_) ||
      !SubscriptionFrequency(wheel_speed_frequency_hz_) ||
      (joystick_frequency_hz_ != 0 && joystick_frequency_hz_ != 50)) {
    return "sensor frequencies must be SDK values 0/1/25/50/125/250 Hz; "
           "joystick_frequency_hz must be 0 or 50";
  }
  if (telemetry_publish_period_ms_ < 1 ||
      telemetry_publish_period_ms_ > kMaxTelemetryPeriodMs ||
      telemetry_query_period_ms_ < kMinQueryPeriodMs ||
      telemetry_query_period_ms_ > kMaxTelemetryPeriodMs ||
      telemetry_max_age_ms_ < telemetry_publish_period_ms_ ||
      telemetry_max_age_ms_ > kMaxSampleAgeMs) {
    return "telemetry publish period must be 1..1000ms, query 100..1000ms, "
           "max age at least publish period and at most 5000ms";
  }
  const std::array<std::string, 7> topics = { imu_topic_, wheel_speed_topic_,
                                              joystick_topic_,
                                              system_status_topic_,
                                              power_status_topic_,
                                              sport_status_topic_,
                                              device_info_topic_ };
  if (!std::all_of(topics.begin(), topics.end(), [](const auto& name) {
      return PlainName(name, false);
    }) || !PlainName(imu_frame_id_, true) || !PlainName(base_frame_id_, true)) {
    return "telemetry topics/frames must be nonempty plain names without "
           "whitespace; frames must not start with /";
  }
  return {};
}

}  // namespace astrall_driver
