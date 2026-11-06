// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <memory>
#include <stdexcept>

#include "astrall_driver/fake_sdk_client.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

constexpr uint32_t kTimeoutMs = 5;

size_t ModeCalls(const FakeSdkClient& client) {
  const auto calls = client.Calls();
  return std::count_if(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kSportModeControl;
      });
}

TEST(SdkModeGuard, RejectsWithoutRawCallAndAdmitsThroughInterfaceAndFake) {
  FakeSdkClient client;
  ISdkClient* interface = &client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  int guards = 0;
  EXPECT_EQ(interface->SportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE,
      kTimeoutMs, [&guards] {
        ++guards;
        return SdkResult(ASTRALL_RES_TIMEOUT);
      }).Value(), ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(guards, 1);
  EXPECT_EQ(ModeCalls(client), 0U);
  EXPECT_TRUE(interface->SportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE,
      kTimeoutMs, [&guards] {
        ++guards;
        return SdkResult(ASTRALL_RES_SUCCESSED);
      }).Succeeded());
  EXPECT_EQ(guards, 2);
  EXPECT_EQ(ModeCalls(client), 1U);
}

TEST(SdkModeGuard, InvalidOrUninitializedCallsDoNotRunGuard) {
  FakeSdkClient client;
  int guards = 0;
  ModeDispatchGuard guard = [&guards] {
        ++guards;
        return SdkResult(ASTRALL_RES_SUCCESSED);
      };
  EXPECT_EQ(client.SportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE, kTimeoutMs,
      guard).Value(), ASTRALL_RES_NOT_INIT);
  ASSERT_TRUE(client.Init({}).Succeeded());
  EXPECT_EQ(client.SportModeControl(static_cast<AstrallSportModeCmd>(0),
      kTimeoutMs, guard).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(guards, 0);
  EXPECT_EQ(ModeCalls(client), 0U);
}

/** @brief 最后一个 guard 捕获引用析构时重入 API，证明释放发生在锁外。 */
class ReentrantCapture {
 public:
  ReentrantCapture(FakeSdkClient* client, uint16_t* result)
    : client_(client), result_(result) {
  }
  ~ReentrantCapture() {
    *result_ = client_->Heartbeat(kTimeoutMs).Value();
  }

 private:
  FakeSdkClient* client_;
  uint16_t* result_;
};

TEST(SdkModeGuard, ThrowPropagatesAndCaptureDestructorCanReenterSdk) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  uint16_t reentry = ASTRALL_RES_FAILED;
  EXPECT_THROW(client.SportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE,
      kTimeoutMs,
      [capture = std::make_shared<ReentrantCapture>(&client, &reentry)]()
      -> SdkResult {
        throw std::runtime_error("mode dispatch guard failure");
      }), std::runtime_error);
  EXPECT_EQ(reentry, ASTRALL_RES_SUCCESSED);
  EXPECT_EQ(ModeCalls(client), 0U);
  EXPECT_TRUE(client.Heartbeat(kTimeoutMs).Succeeded());
}

TEST(SdkModeGuard, RejectionAndSuccessReleaseCapturedResourcesOutsideLock) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  for (const auto result : { ASTRALL_RES_TIMEOUT, ASTRALL_RES_SUCCESSED }) {
    uint16_t reentry = ASTRALL_RES_FAILED;
    EXPECT_EQ(client.SportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE, kTimeoutMs,
        [capture = std::make_shared<ReentrantCapture>(&client, &reentry),
        result] {
          return SdkResult(result);
        }).Value(), result);
    EXPECT_EQ(reentry, ASTRALL_RES_SUCCESSED);
  }
  EXPECT_EQ(ModeCalls(client), 1U);
}

}  // namespace
}  // namespace astrall_driver
