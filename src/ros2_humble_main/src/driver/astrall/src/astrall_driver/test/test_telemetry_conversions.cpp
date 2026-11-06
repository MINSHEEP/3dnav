// Copyright 2026 Hypertron ROS 2 contributors

#include <cmath>
#include <limits>
#include <string>

#include "astrall_driver/telemetry_conversions.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

constexpr int64_t kReceiveStamp = 1234567890;

TEST(TelemetryConversions, ImuUsesReceiveTimeSiValuesAndNormalizedXyzw) {
  AstrallImuData data;
  data.timestamp = 99999;
  data.quaternion[0] = 2;
  data.quaternion[3] = 2;
  data.accelerometer[2] = 9.8F;
  data.gyroscope[1] = 0.5F;
  data.yaw = std::numeric_limits<float>::quiet_NaN();
  auto message = ConvertImu(data, kReceiveStamp, "imu_frame");
  ASSERT_TRUE(message);
  EXPECT_EQ(message->header.stamp.sec, 1);
  EXPECT_EQ(message->header.stamp.nanosec, 234567890U);
  EXPECT_EQ(message->header.frame_id, "imu_frame");
  EXPECT_NEAR(message->orientation.w, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(message->orientation.z, std::sqrt(0.5), 1e-6);
  EXPECT_DOUBLE_EQ(message->angular_velocity.y, 0.5);
  EXPECT_FLOAT_EQ(message->linear_acceleration.z, 9.8F);
  EXPECT_DOUBLE_EQ(message->orientation_covariance[0], 0);
}

TEST(TelemetryConversions, InvalidImuWheelPowerOrHeaderDoesNotProduceMessage) {
  AstrallImuData imu;
  EXPECT_FALSE(ConvertImu(imu, kReceiveStamp, "base_link"));
  imu.quaternion[0] = 1;
  imu.gyroscope[0] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(ConvertImu(imu, kReceiveStamp, "base_link"));
  imu.gyroscope[0] = 0;
  imu.quaternion[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(ConvertImu(imu, kReceiveStamp, "base_link"));
  AstrallSportData wheel;
  wheel.wheelSpeed[2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(ConvertWheelSpeeds(wheel, kReceiveStamp, "base_link"));
  AstrallPowerStatus power;
  power.soc = 101;
  EXPECT_FALSE(ConvertPowerStatus(power, kReceiveStamp, "base_link"));
  power.soc = -1;
  EXPECT_FALSE(ConvertPowerStatus(power, kReceiveStamp, "base_link"));
  power.soc = 10;
  power.temp = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(ConvertPowerStatus(power, kReceiveStamp, "base_link"));
  AstrallDeviceInfo device;
  EXPECT_FALSE(ConvertDeviceInfo(device, -1, "base_link"));
  EXPECT_FALSE(ConvertDeviceInfo(device, std::numeric_limits<int64_t>::max(),
      "base_link"));
  EXPECT_FALSE(ConvertDeviceInfo(device, kReceiveStamp, ""));
}

TEST(TelemetryConversions, RawWheelsJoystickAndDeviceFieldsArePreserved) {
  AstrallSportData wheel;
  wheel.timestamp = -123;
  wheel.wheelSpeed[0] = 12.5F;
  wheel.wheelSpeed[3] = -3.25F;
  auto wheels = ConvertWheelSpeeds(wheel, kReceiveStamp, "base_link");
  ASSERT_TRUE(wheels);
  EXPECT_EQ(wheels->sdk_timestamp, -123);
  EXPECT_FLOAT_EQ(wheels->wheel_speed[0], 12.5F);
  EXPECT_FLOAT_EQ(wheels->wheel_speed[3], -3.25F);
  AstrallJoystickData data{};
  data.timestamp = 42;
  data.rocker[0] = std::numeric_limits<int16_t>::min();
  data.rocker[5] = std::numeric_limits<int16_t>::max();
  data.l3 = 1;
  data.r2 = 1;
  data.sw = 3;
  auto joystick = ConvertJoystick(data, kReceiveStamp, "base_link");
  ASSERT_TRUE(joystick);
  EXPECT_EQ(joystick->rocker[0], -32768);
  EXPECT_EQ(joystick->rocker[5], 32767);
  EXPECT_TRUE(joystick->l3);
  EXPECT_TRUE(joystick->r2);
  EXPECT_EQ(joystick->switch_position, 3);
  EXPECT_EQ(joystick->sdk_timestamp, 42);
  AstrallDeviceInfo device;
  device.version = "1.1.1";
  device.serialNumber = "fake_sn";
  device.model = 7;
  auto info = ConvertDeviceInfo(device, kReceiveStamp, "base_link");
  ASSERT_TRUE(info);
  EXPECT_EQ(info->version, "1.1.1");
  EXPECT_EQ(info->serial_number, "fake_sn");
  EXPECT_EQ(info->model, 7U);
}

TEST(TelemetryConversions, PowerFractionAndRawSystemAndModeStayDistinct) {
  AstrallPowerStatus power;
  power.soc = 75;
  power.voltage = 48.4F;
  power.temp = 20.5F;
  power.charged = 3;
  power.cycleCount = 12;
  auto battery = ConvertPowerStatus(power, kReceiveStamp, "base_link");
  ASSERT_TRUE(battery);
  EXPECT_FLOAT_EQ(battery->percentage, 0.75F);
  EXPECT_FLOAT_EQ(battery->voltage, 48.4F);
  EXPECT_FLOAT_EQ(battery->temperature, 20.5F);
  EXPECT_EQ(battery->cycle_count, 12);
  EXPECT_EQ(battery->charged, 3);
  AstrallSystemStatus system;
  system.errorCode = static_cast<AstrallErrorCode>(0x80000210U);
  system.warnCode = static_cast<AstrallWarnCode>(0x104);
  auto status = ConvertSystemStatus(system, kReceiveStamp, "base_link");
  ASSERT_TRUE(status);
  EXPECT_EQ(status->error_code, 0x80000210U);
  EXPECT_EQ(status->warn_code, 0x104U);
  EXPECT_EQ(status->error_messages.size(), 3U);
  EXPECT_EQ(status->warn_messages.size(), 2U);
  auto mode = ConvertSportStatus(static_cast<AstrallSportStatus>(0xBEEF),
          kReceiveStamp, "base_link");
  ASSERT_TRUE(mode);
  EXPECT_EQ(mode->status, 0xBEEF);
}

}  // namespace
}  // namespace astrall_driver
