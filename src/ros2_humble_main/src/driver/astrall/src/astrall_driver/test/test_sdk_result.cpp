// Copyright 2026 Hypertron ROS 2 contributors

#include <array>
#include <cstdint>
#include <string>
#include <utility>

#include "astrall_driver/sdk_result.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

TEST(SdkResult, AllNineManualCodesAreReadable) {
  const std::array<std::pair<uint16_t, const char*>, 9> expected = { {
    { 0x8004, "ASTRALL_RES_FAILED" },
    { 0x8005, "ASTRALL_RES_TIMEOUT" },
    { 0x8006, "ASTRALL_RES_RUNNING" },
    { 0x8008, "ASTRALL_RES_SUCCESSED" },
    { 0x8010, "ASTRALL_RES_INVALID_PARAM" },
    { 0x8011, "ASTRALL_RES_NOT_INIT" },
    { 0x8020, "ASTRALL_RES_RC_NORELEASE" },
    { 0x8021, "ASTRALL_RES_BEEN_OBTAINED" },
    { 0x8022, "ASTRALL_RES_WITHOUT_AUTH" }
  } };
  for (const auto& item : expected) {
    const SdkResult result(item.first);
    EXPECT_EQ(result.Value(), item.first);
    EXPECT_STREQ(result.Name(), item.second);
    EXPECT_EQ(result.Succeeded(), item.first == 0x8008);
    EXPECT_NE(result.ToString().find(item.second), std::string::npos);
  }
}

TEST(SdkResult, UnknownAndStatusValuesAreNotSuccessCodes) {
  EXPECT_EQ(SdkResult(0x1234).ToString(), "UNKNOWN_SDK_RESULT (0x1234)");
  EXPECT_EQ(SdkResult(0).ToString(), "UNKNOWN_SDK_RESULT (0x0000)");
  EXPECT_FALSE(SdkResult(0xB104).Succeeded());
}

}  // namespace
}  // namespace astrall_driver
