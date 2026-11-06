// Copyright 2026 Hypertron ROS 2 contributors

#include <array>
#include <cstdint>
#include <string>

#include "astrall_driver/fault_bits.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

TEST(FaultBits, EveryKnownErrorAndWarningHasItsOwnReadableLabel) {
  const std::array<const char*, 10> errors = {
    "MOTOR_VERSION", "MOTOR_COMM_LOSE", "MOTOR_STATUS", "MOTOR_OVER_TEMP",
    "STO_TRIGGERED", "IMU_ERROR", "LOW_BATTERY_LEVEL", "OVER_LOAD",
    "MBUS_DRIVER_FAIL", "LOSE_CAMERA"
  };
  const std::array<const char*, 8> warnings = {
    "MOTOR_OVER_TEMP", "JOYSTICK_LOCK", "JOYSTICK_INTERVENTION",
    "MOTOR_COMM_UNSTABLE", "LOW_BATTERY_LEVEL", "JOYSTICK_LOSE",
    "MOTOR_OVER_LOAD", "CONNECT_SDK_LOSE"
  };
  for (size_t bit = 0; bit < errors.size(); ++bit) {
    const auto labels = DecodeErrorBits(uint32_t{ 1 } << bit);
    ASSERT_EQ(labels.size(), 1U);
    EXPECT_NE(labels.front().find(errors[bit]), std::string::npos);
  }
  for (size_t bit = 0; bit < warnings.size(); ++bit) {
    const auto labels = DecodeWarnBits(uint32_t{ 1 } << bit);
    ASSERT_EQ(labels.size(), 1U);
    EXPECT_NE(labels.front().find(warnings[bit]), std::string::npos);
  }
}

TEST(FaultBits, ZeroCombinationsAndUnknownBitsArePreserved) {
  EXPECT_NE(DecodeErrorBits(0).front().find("ASTRALL_ERR_NONE"),
      std::string::npos);
  EXPECT_NE(DecodeWarnBits(0).front().find("ASTRALL_WARN_NONE"),
      std::string::npos);
  const auto errors = DecodeErrorBits(0x80000211U);
  ASSERT_EQ(errors.size(), 4U);
  EXPECT_NE(errors[0].find("MOTOR_VERSION"), std::string::npos);
  EXPECT_NE(errors[1].find("STO_TRIGGERED"), std::string::npos);
  EXPECT_NE(errors[2].find("LOSE_CAMERA"), std::string::npos);
  EXPECT_EQ(errors[3], "UNKNOWN_ERROR_BITS (0x80000000)");
  EXPECT_EQ(DecodeWarnBits(0x100).front(), "UNKNOWN_WARN_BITS (0x00000100)");
  EXPECT_EQ(DecodeErrorBits(0xFFFFFFFF).size(), 11U);
  EXPECT_EQ(DecodeWarnBits(0xFFFFFFFF).size(), 9U);
}

}  // namespace
}  // namespace astrall_driver
