// Copyright 2026 Hypertron ROS 2 contributors

#include <cmath>
#include <limits>

#include "astrall_driver/driver_parameters.hpp"
#include "astrall_driver/subscription_frequency.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

TEST(DriverParameters, DefaultsAndNetworkBoundariesAreConservative) {
  DriverParameters parameters;
  EXPECT_TRUE(parameters.use_fake_sdk_);
  EXPECT_TRUE(parameters.Validate().empty());
  parameters.local_ip_ = "10.18.10.209";
  EXPECT_TRUE(parameters.Validate().empty());
  parameters.local_ip_ = "10.18.10.210";
  EXPECT_FALSE(parameters.Validate().empty());
  parameters.local_ip_ = "not_an_ip";
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.robot_ip_ = "127.0.0.1";
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.sdk_port_ = 1;
  EXPECT_FALSE(parameters.Validate().empty());
}

TEST(DriverParameters, InvalidPeriodsCannotStarveHeartbeatOrOverflow) {
  DriverParameters parameters;
  parameters.heartbeat_period_ms_ = 101;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.sdk_timeout_ms_ = 21;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters.sdk_timeout_ms_ = -1;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.init_timeout_ms_ = std::numeric_limits<int64_t>::max();
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.system_poll_period_ms_ = 0;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.state_publish_period_ms_ = -1;
  EXPECT_FALSE(parameters.Validate().empty());
}

TEST(DriverParameters, HeartbeatAckBudgetIsIndependentAndBounded) {
  DriverParameters parameters;
  EXPECT_EQ(parameters.heartbeat_timeout_ms_, 5);
  for (int64_t timeout : { 1, 20 }) {
    parameters.heartbeat_timeout_ms_ = timeout;
    EXPECT_TRUE(parameters.Validate().empty());
    EXPECT_EQ(parameters.sdk_timeout_ms_, 5);
    EXPECT_EQ(parameters.control_sdk_timeout_ms_, 2);
  }
  for (int64_t timeout : { -1, 0, 21 }) {
    parameters.heartbeat_timeout_ms_ = timeout;
    EXPECT_FALSE(parameters.Validate().empty());
  }
}

TEST(DriverParameters, TelemetryFrequenciesFramesAndTimingAreValidated) {
  DriverParameters parameters;
  EXPECT_FALSE(parameters.enable_rgb_);
  EXPECT_FALSE(parameters.enable_lidar_);
  EXPECT_EQ(SubscriptionFrequency(250), ASTRALL_SUB_FREQ_250HZ);
  EXPECT_EQ(SubscriptionFrequency(25), ASTRALL_SUB_FREQ_25HZ);
  EXPECT_EQ(SubscriptionFrequency(0), ASTRALL_SUB_FREQ_CLOSE);
  EXPECT_FALSE(SubscriptionFrequency(42));
  parameters.imu_frequency_hz_ = 42;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.joystick_frequency_hz_ = 25;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters.joystick_frequency_hz_ = 0;
  EXPECT_TRUE(parameters.Validate().empty());
  parameters = {};
  parameters.telemetry_max_age_ms_ = parameters.telemetry_publish_period_ms_ -
      1;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.imu_frame_id_ = "/absolute_frame";
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.imu_topic_ = "invalid topic";
  EXPECT_FALSE(parameters.Validate().empty());
}

TEST(DriverParameters, ControlBudgetAndTextRateCannotViolateWireConstraints) {
  DriverParameters parameters;
  EXPECT_TRUE(parameters.Validate().empty());
  // Status queries use the vendor 20ms default independently of motion calls.
  parameters.sdk_timeout_ms_ = 20;
  EXPECT_TRUE(parameters.Validate().empty());
  parameters.control_sdk_timeout_ms_ = 20;
  EXPECT_TRUE(parameters.Validate().empty());
  parameters.control_sdk_timeout_ms_ = 21;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.control_period_ms_ = 50;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.max_yaw_velocity_ = 1.1;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters = {};
  parameters.text_rate_hz_ = 5;
  EXPECT_FALSE(parameters.Validate().empty());
  parameters.text_rate_hz_ = std::nextafter(5.0, 0.0);
  EXPECT_TRUE(parameters.Validate().empty());
}

}  // namespace
}  // namespace astrall_driver
