// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_parameters.hpp"

#include <arpa/inet.h>

namespace astrall_driver {
namespace {

constexpr int64_t kMinInitTimeoutMs = 1;
constexpr int64_t kMaxInitTimeoutMs = 5000;
constexpr int64_t kMinPollPeriodMs = 100;
constexpr int64_t kMaxPollPeriodMs = 500;
constexpr int64_t kMinPublishPeriodMs = 50;
constexpr int64_t kMaxPublishPeriodMs = 1000;
constexpr uint32_t kFirstLocalAddress = 0x0A120AC8;
constexpr uint32_t kLastLocalAddress = 0x0A120AD1;
constexpr int64_t kSdkPort = 3600;
constexpr int64_t kSdkClientPort = 3601;

bool ValidLocalAddress(const std::string& value) {
  in_addr address{};
  if (inet_pton(AF_INET, value.c_str(), &address) != 1) {
    return false;
  }
  const auto host = ntohl(address.s_addr);
  return host >= kFirstLocalAddress && host <= kLastLocalAddress;
}

}  // namespace

std::string DriverParameters::Validate() const {
  if (robot_ip_ != "10.18.10.100" || !ValidLocalAddress(local_ip_) ||
      sdk_port_ != kSdkPort || sdk_client_port_ != kSdkClientPort) {
    return "robot_ip/local_ip/sdk_port/sdk_client_port must match the fixed "
           "SDK network; these parameters do not configure the SDK or NIC";
  }
  if (init_timeout_ms_ < kMinInitTimeoutMs ||
      init_timeout_ms_ > kMaxInitTimeoutMs) {
    return "init_timeout_ms must be in [1,5000]";
  }
  if (sdk_timeout_ms_ < 1 || sdk_timeout_ms_ > kMaxSdkTimeoutMs) {
    return "sdk_timeout_ms must be in [1,20] to reserve the heartbeat budget";
  }
  if (heartbeat_timeout_ms_ < 1 || heartbeat_timeout_ms_ > kMaxSdkTimeoutMs) {
    return "heartbeat_timeout_ms must be in [1,20]";
  }
  if (heartbeat_period_ms_ != kHeartbeatPeriodMs) {
    return "heartbeat_period_ms is fixed at 100ms (10Hz), see manual 5.5";
  }
  if (system_poll_period_ms_ < kMinPollPeriodMs ||
      system_poll_period_ms_ > kMaxPollPeriodMs) {
    return "system_poll_period_ms must be in [100,500]";
  }
  if (state_publish_period_ms_ < kMinPublishPeriodMs ||
      state_publish_period_ms_ > kMaxPublishPeriodMs) {
    return "state_publish_period_ms must be in [50,1000]";
  }
  const auto telemetry = ValidateTelemetry();
  if (!telemetry.empty()) {
    return telemetry;
  }
  const auto control = ValidateControl();
  return control.empty() ? ValidateLidar() : control;
}

}  // namespace astrall_driver
