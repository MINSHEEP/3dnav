// Copyright 2026 Hypertron ROS 2 contributors

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "astrall_driver/fake_sdk_client.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

constexpr uint32_t kWaitMs = 2000;

TEST(FakeSdkClient, DefaultHasNoImplicitMotionAuthorityOrHeartbeat) {
  FakeSdkClient client;
  EXPECT_TRUE(client.Calls().empty());
  EXPECT_EQ(client.Move(0, 0, 0).Value(), ASTRALL_RES_NOT_INIT);
  std::atomic<int> statuses{ 0 };
  SdkCallbacks callbacks;
  callbacks.status_ = [&statuses](const auto& status) {
        EXPECT_FALSE(status.ctrlAuthority);
        ++statuses;
      };
  ASSERT_TRUE(client.Init(callbacks).Succeeded());
  ASSERT_EQ(client.Calls().size(), 1U);
  EXPECT_EQ(client.Calls().front().operation_, SdkOperation::kInit);
  EXPECT_EQ(client.GetSportStatus(), ASTRALL_SPORT_STATUS_UNKNOWN);
  EXPECT_TRUE(client.EmitStatus(AstrallSdkStatus{}));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(statuses, 1);
  EXPECT_TRUE(client.AuthControl(ASTRALL_AUTH_SDK).Succeeded());
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(statuses, 1);
}

TEST(FakeSdkClient, ForwardsAndRecordsEverySdkOperation) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}, 101).Succeeded());
  EXPECT_TRUE(client.Heartbeat(102).Succeeded());
  AstrallSystemStatus system;
  AstrallPowerStatus power;
  AstrallDeviceInfo device;
  AstrallImuData imu;
  EXPECT_TRUE(client.GetSystemStatus(&system).Succeeded());
  EXPECT_TRUE(client.GetPowerStatus(&power).Succeeded());
  EXPECT_EQ(client.GetSportStatus(), ASTRALL_SPORT_STATUS_UNKNOWN);
  EXPECT_TRUE(client.GetDeviceInfo(&device, 103).Succeeded());
  EXPECT_TRUE(client.GetImuData(&imu).Succeeded());
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_125HZ,
      [](const auto&) {
      }, 104).Succeeded());
  EXPECT_TRUE(client.AuthControl(ASTRALL_AUTH_JOYSTICK, 105).Succeeded());
  EXPECT_TRUE(client.SportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE_TANK,
      106).Succeeded());
  EXPECT_TRUE(client.Move(-1, 0.5F, 1, 107).Succeeded());
  EXPECT_TRUE(client.LightControl(ASTRALL_CMD_LIGHT_CLOSE, 108).Succeeded());
  EXPECT_TRUE(client.SendMessage("测试文本").Succeeded());
  client.Deinit();
  auto calls = client.Calls();
  ASSERT_EQ(calls.size(), 14U);
  EXPECT_EQ(calls[0].timeout_ms_, 101U);
  EXPECT_EQ(calls[1].timeout_ms_, 102U);
  EXPECT_EQ(calls[5].timeout_ms_, 103U);
  EXPECT_EQ(calls[7].operation_, SdkOperation::kSubscribe);
  EXPECT_EQ(calls[7].topic_, ASTRALL_SUB_TOPIC_ID_IMU);
  EXPECT_EQ(calls[7].frequency_, ASTRALL_SUB_FREQ_125HZ);
  EXPECT_EQ(calls[7].timeout_ms_, 104U);
  EXPECT_EQ(calls[8].argument_, ASTRALL_AUTH_JOYSTICK);
  EXPECT_EQ(calls[8].timeout_ms_, 105U);
  EXPECT_EQ(calls[9].argument_, ASTRALL_SPORT_MODE_CMD_MOVE_TANK);
  EXPECT_EQ(calls[9].timeout_ms_, 106U);
  EXPECT_FLOAT_EQ(calls[10].velocity_[0], -1);
  EXPECT_FLOAT_EQ(calls[10].velocity_[1], 0.5F);
  EXPECT_FLOAT_EQ(calls[10].velocity_[2], 1);
  EXPECT_EQ(calls[10].timeout_ms_, 107U);
  EXPECT_EQ(calls[11].argument_, ASTRALL_CMD_LIGHT_CLOSE);
  EXPECT_EQ(calls[11].timeout_ms_, 108U);
  EXPECT_EQ(calls[12].text_, "测试文本");
  EXPECT_EQ(calls[13].operation_, SdkOperation::kDeinit);
}

TEST(FakeSdkClient, ResultsAndSportStatusesHaveSeparateInjectionDomains) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  const std::array<uint16_t, 10> codes = {
    0x8004, 0x8005, 0x8006, 0x8008, 0x8010, 0x8011, 0x8020, 0x8021, 0x8022,
    0xFFFF
  };
  for (auto code : codes) {
    client.SetResult(SdkOperation::kHeartbeat, code);
    EXPECT_EQ(client.Heartbeat().Value(), code);
  }
  client.SetSportStatus(0xBEEF);
  EXPECT_EQ(static_cast<uint16_t>(client.GetSportStatus()), 0xBEEF);
  EXPECT_THROW(client.SetResult(SdkOperation::kGetSportStatus, 0x8008),
      std::invalid_argument);
  EXPECT_THROW(client.SetResult(SdkOperation::kDeinit, 0x8008),
      std::invalid_argument);
  EXPECT_THROW(client.SetResult(static_cast<SdkOperation>(999), 0x8008),
      std::invalid_argument);
}

TEST(FakeSdkClient, InjectedQueriesAreCopiedAndFailuresClearOldValues) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  AstrallDeviceInfo device;
  device.version = "1.1.1";
  client.SetDeviceInfo(device);
  device.version = "changed";
  ASSERT_TRUE(client.GetDeviceInfo(&device).Succeeded());
  EXPECT_EQ(device.version, "1.1.1");
  AstrallPowerStatus power;
  power.soc = 64.5F;
  client.SetPowerStatus(power);
  EXPECT_TRUE(client.GetPowerStatus(&power).Succeeded());
  EXPECT_FLOAT_EQ(power.soc, 64.5F);
  AstrallImuData imu;
  imu.timestamp = 12345;
  client.SetImuData(imu);
  EXPECT_TRUE(client.GetImuData(&imu).Succeeded());
  EXPECT_EQ(imu.timestamp, 12345);
  client.SetResult(SdkOperation::kGetDeviceInfo, ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(client.GetDeviceInfo(&device).Value(), ASTRALL_RES_TIMEOUT);
  EXPECT_TRUE(device.version.empty());
  client.SetResult(SdkOperation::kGetPowerStatus, ASTRALL_RES_FAILED);
  EXPECT_EQ(client.GetPowerStatus(&power).Value(), ASTRALL_RES_FAILED);
  EXPECT_FLOAT_EQ(power.soc, 0);
  client.SetResult(SdkOperation::kGetImuData, ASTRALL_RES_NOT_INIT);
  EXPECT_EQ(client.GetImuData(&imu).Value(), ASTRALL_RES_NOT_INIT);
  EXPECT_EQ(imu.timestamp, 0);
}

TEST(FakeSdkClient, InvalidNativeArgumentsDoNotReachBackend) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  client.ClearCalls();
  const auto invalid = ASTRALL_RES_INVALID_PARAM;
  EXPECT_EQ(client.Move(std::numeric_limits<float>::quiet_NaN(), 0, 0).Value(),
      invalid);
  EXPECT_EQ(client.Move(0, std::numeric_limits<float>::infinity(), 0).Value(),
      invalid);
  EXPECT_EQ(client.Move(0, 0, -1.01F).Value(), invalid);
  EXPECT_EQ(client.SendMessage("").Value(), invalid);
  EXPECT_EQ(client.SendMessage(std::string("a\0b", 3)).Value(), invalid);
  constexpr auto kMaxSize = std::numeric_limits<uint16_t>::max();
  EXPECT_EQ(client.SendMessage(std::string(kMaxSize + 1U, 'a')).Value(),
      invalid);
  EXPECT_EQ(client.AuthControl(static_cast<AstrallAuth>(0)).Value(), invalid);
  EXPECT_EQ(client.SportModeControl(
        static_cast<AstrallSportModeCmd>(0)).Value(),
      invalid);
  EXPECT_EQ(client.LightControl(static_cast<AstrallLightCmd>(0)).Value(),
      invalid);
  EXPECT_TRUE(client.Calls().empty());
  EXPECT_TRUE(client.SendMessage(std::string(kMaxSize, 'a')).Succeeded());
  ASSERT_EQ(client.Calls().size(), 1U);
  EXPECT_EQ(client.Calls().front().text_.size(), kMaxSize);
}

TEST(FakeSdkClient,
    SubscriptionValidationMatchesManualAndUdpHasNoTypedCallback) {
  FakeSdkClient client;
  ASSERT_TRUE(client.Init({}).Succeeded());
  auto callback = [](const SubscriptionData&) {
      };
  EXPECT_EQ(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_50HZ)
      .Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.Subscribe(ASTRALL_SUB_TOPIC_ID_JOYSTICK,
      ASTRALL_SUB_FREQ_1HZ, callback).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.Subscribe(ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
      ASTRALL_SUB_FREQ_1HZ, callback).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU,
      static_cast<AstrallSubscribeFreq>(999), callback).Value(),
      ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.Subscribe(static_cast<AstrallSubscribeTopicId>(999),
      ASTRALL_SUB_FREQ_50HZ, callback).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_JOYSTICK,
      ASTRALL_SUB_FREQ_50HZ, callback).Succeeded());
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
      ASTRALL_SUB_FREQ_1HZ).Succeeded());
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_LIDAR,
      ASTRALL_SUB_FREQ_1HZ).Succeeded());
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
      ASTRALL_SUB_FREQ_CLOSE).Succeeded());
}

TEST(FakeSdkClient, TypedAndMalformedCallbacksUseTheSameSafeBoundary) {
  FakeSdkClient client;
  std::atomic<int> heartbeats{ 0 };
  std::atomic<int> statuses{ 0 };
  std::atomic<int> samples{ 0 };
  SdkCallbacks callbacks;
  callbacks.heartbeat_ = [&heartbeats] {
        ++heartbeats;
      };
  callbacks.status_ = [&statuses](const auto& status) {
        EXPECT_TRUE(status.link);
        EXPECT_FALSE(status.ctrlAuthority);
        ++statuses;
      };
  ASSERT_TRUE(client.Init(callbacks).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  EXPECT_TRUE(client.EmitHeartbeat());
  EXPECT_TRUE(client.EmitStatus(status));
  auto callback = [&samples](const SubscriptionData&) {
        ++samples;
      };
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_250HZ,
      callback).Succeeded());
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_SPORT,
      ASTRALL_SUB_FREQ_125HZ,
      callback).Succeeded());
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_JOYSTICK,
      ASTRALL_SUB_FREQ_50HZ, callback).Succeeded());
  EXPECT_TRUE(client.EmitImu(AstrallImuData{}));
  EXPECT_TRUE(client.EmitSport(AstrallSportData{}));
  EXPECT_TRUE(client.EmitJoystick(AstrallJoystickData{}));
  EXPECT_TRUE(client.EmitRawStatus(nullptr, sizeof(status)));
  EXPECT_TRUE(client.EmitRawData(ASTRALL_SUB_TOPIC_ID_IMU, nullptr, 1));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(heartbeats, 1);
  EXPECT_EQ(statuses, 1);
  EXPECT_EQ(samples, 3);
  EXPECT_EQ(client.GetCallbackStatistics().invalid_payloads_, 2U);
}

TEST(FakeSdkClient, FailedInitAndLateRawCallbacksAreSafe) {
  FakeSdkClient client;
  client.SetResult(SdkOperation::kInit, ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(client.Init({}).Value(), ASTRALL_RES_TIMEOUT);
  ASSERT_EQ(client.Calls().size(), 2U);
  EXPECT_EQ(client.Calls().back().operation_, SdkOperation::kDeinit);
  client.SetResult(SdkOperation::kInit, ASTRALL_RES_SUCCESSED);
  ASSERT_TRUE(client.Init({}).Succeeded());
  auto late = client.RawStatusCallback();
  client.Deinit();
  EXPECT_FALSE(client.EmitStatus(AstrallSdkStatus{}));
  AstrallSdkStatus status;
  late(&status, sizeof(status));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(client.GetCallbackStatistics().callback_errors_, 0U);
}

TEST(FakeSdkClient, IndependentSessionsAndConcurrentRecordsAreSafe) {
  FakeSdkClient first;
  FakeSdkClient second;
  ASSERT_TRUE(first.Init({}).Succeeded());
  ASSERT_TRUE(second.Init({}).Succeeded());
  first.ClearCalls();
  std::vector<std::thread> threads;
  for (int index = 0; index < 4; ++index) {
    threads.emplace_back([&first] {
          for (int call = 0; call < 100; ++call) {
            EXPECT_TRUE(first.Heartbeat().Succeeded());
          }
        });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(first.Calls().size(), 400U);
  EXPECT_EQ(second.Calls().size(), 1U);
}

}  // namespace
}  // namespace astrall_driver
