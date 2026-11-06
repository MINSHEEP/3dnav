// Copyright 2026 Hypertron ROS 2 contributors

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "astrall_driver/sdk_client.hpp"
#include "gtest/gtest.h"
#include "sdk_test_backend.hpp"

namespace astrall_driver {
namespace {

constexpr auto kWait = std::chrono::seconds(2);
constexpr uint32_t kWaitMs = 2000;

using test::Backend;
using test::Functions;

TEST(SdkClient, NativeSymbolsLinkWithoutInitializingOrConnecting) {
  SdkClient client;
  EXPECT_EQ(client.Heartbeat().Value(), ASTRALL_RES_NOT_INIT);
  EXPECT_EQ(client.GetSportStatus(), ASTRALL_SPORT_STATUS_UNKNOWN);
  EXPECT_THROW(SdkClient(SdkFunctions{}), std::invalid_argument);
}

TEST(SdkClient, GlobalOwnerIsExclusiveAndNonOwnerCannotDeinitializeIt) {
  auto backend = std::make_shared<Backend>();
  auto functions = Functions(backend);
  functions.process_global_ = true;
  SdkClient first(functions);
  SdkClient second(functions);
  ASSERT_TRUE(first.Init({}).Succeeded());
  EXPECT_EQ(first.Init({}).Value(), ASTRALL_RES_RUNNING);
  EXPECT_EQ(second.Init({}).Value(), ASTRALL_RES_BEEN_OBTAINED);
  second.Deinit();
  EXPECT_EQ(backend->deinit_count_, 0);
  EXPECT_TRUE(first.Heartbeat().Succeeded());
  first.Deinit();
  first.Deinit();
  EXPECT_EQ(backend->deinit_count_, 1);
  EXPECT_TRUE(second.Init({}).Succeeded());
}

TEST(SdkClient, FailedInitCleansPartialSessionAndDropsPendingCallbacks) {
  auto backend = std::make_shared<Backend>();
  auto functions = Functions(backend);
  functions.process_global_ = true;
  functions.init_ = [backend](AstrallConfig& config, uint32_t) {
        backend->config_ = config;
        AstrallSdkStatus status;
        status.link = 1;
        config.sdkStatusCb(&status, sizeof(status));
        return ASTRALL_RES_TIMEOUT;
      };
  SdkClient client(functions);
  std::atomic<int> callbacks{ 0 };
  SdkCallbacks handlers;
  handlers.status_ = [&callbacks](const auto&) {
        ++callbacks;
      };
  EXPECT_EQ(client.Init(handlers).Value(), ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(backend->deinit_count_, 1);
  EXPECT_EQ(callbacks, 0);
  EXPECT_EQ(client.Heartbeat().Value(), ASTRALL_RES_NOT_INIT);
  auto other_functions = Functions(backend);
  other_functions.process_global_ = true;
  SdkClient other(other_functions);
  EXPECT_TRUE(other.Init({}).Succeeded());
}

TEST(SdkClient, QueriesClearOutputsOnFailureAndPreserveUnknownSportStatus) {
  auto functions = Functions(std::make_shared<Backend>());
  functions.get_system_ = [](AstrallSystemStatus& value) {
        value.errorCode = ASTRALL_ERR_STO_TRIGGERED;
        return ASTRALL_RES_TIMEOUT;
      };
  functions.get_sport_ = [] {
        return 0xBEEF;
      };
  SdkClient client(functions);
  ASSERT_TRUE(client.Init({}).Succeeded());
  AstrallSystemStatus status;
  status.errorCode = ASTRALL_ERR_OVER_LOAD;
  EXPECT_EQ(client.GetSystemStatus(&status).Value(), ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(status.errorCode, ASTRALL_ERR_NONE);
  EXPECT_EQ(static_cast<uint16_t>(client.GetSportStatus()), 0xBEEF);
  EXPECT_EQ(client.GetSystemStatus(nullptr).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.GetPowerStatus(nullptr).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.GetDeviceInfo(nullptr).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(client.GetImuData(nullptr).Value(), ASTRALL_RES_INVALID_PARAM);
}

TEST(SdkClient, CallbackRejectsNullShortOversizedAndCopiesUnalignedData) {
  auto backend = std::make_shared<Backend>();
  SdkClient client(Functions(backend));
  std::atomic<int> calls{ 0 };
  SdkCallbacks callbacks;
  callbacks.status_ = [&calls](const auto&) {
        ++calls;
      };
  ASSERT_TRUE(client.Init(callbacks).Succeeded());
  AstrallSdkStatus status;
  backend->config_.sdkStatusCb(nullptr, sizeof(status));
  backend->config_.sdkStatusCb(&status, 0);
  backend->config_.sdkStatusCb(&status, sizeof(status) + 1);
  ASSERT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_250HZ,
      [&calls](const SubscriptionData& value) {
        EXPECT_EQ(std::get<AstrallImuData>(value).timestamp, 42);
        ++calls;
      }).Succeeded());
  AstrallImuData imu;
  imu.timestamp = 42;
  std::array<unsigned char, sizeof(imu) + 1> bytes{};
  std::memcpy(bytes.data() + 1, &imu, sizeof(imu));
  backend->subscription_(nullptr, sizeof(imu));
  backend->subscription_(&imu, sizeof(imu) - 1);
  backend->subscription_(bytes.data() + 1, sizeof(imu));
  bytes.fill(0);
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(client.GetCallbackStatistics().invalid_payloads_, 5U);
}

TEST(SdkClient, FailedReplacementPreservesOldSubscriptionAndCancelStopsIt) {
  auto backend = std::make_shared<Backend>();
  SdkClient client(Functions(backend));
  ASSERT_TRUE(client.Init({}).Succeeded());
  std::atomic<int> old_calls{ 0 };
  std::atomic<int> new_calls{ 0 };
  ASSERT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_SPORT,
      ASTRALL_SUB_FREQ_50HZ,
      [&old_calls](const auto&) {
        ++old_calls;
      }).Succeeded());
  auto late = backend->subscription_;
  backend->subscribe_result_ = ASTRALL_RES_TIMEOUT;
  EXPECT_EQ(client.Subscribe(ASTRALL_SUB_TOPIC_ID_SPORT, ASTRALL_SUB_FREQ_50HZ,
      [&new_calls](const auto&) {
        ++new_calls;
      }).Value(), ASTRALL_RES_TIMEOUT);
  AstrallSportData data;
  late(&data, sizeof(data));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(old_calls, 1);
  EXPECT_EQ(new_calls, 0);
  backend->subscribe_result_ = ASTRALL_RES_SUCCESSED;
  EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_SPORT,
      ASTRALL_SUB_FREQ_CLOSE).Succeeded());
  late(&data, sizeof(data));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(old_calls, 1);
}

TEST(SdkClient,
    CallbackCanDeinitializeAndReleaseClientAndLateCallbacksAreSafe) {
  auto backend = std::make_shared<Backend>();
  auto client = std::make_unique<SdkClient>(Functions(backend));
  std::promise<void> released;
  auto done = released.get_future();
  std::atomic<int> calls{ 0 };
  SdkCallbacks callbacks;
  callbacks.status_ = [&client, &released, &calls](const auto&) {
        client->Deinit();
        client.reset();
        ++calls;
        released.set_value();
      };
  ASSERT_TRUE(client->Init(callbacks).Succeeded());
  auto late = backend->config_.sdkStatusCb;
  AstrallSdkStatus status;
  late(&status, sizeof(status));
  ASSERT_EQ(done.wait_for(kWait), std::future_status::ready);
  late(&status, sizeof(status));
  late(nullptr, sizeof(status));
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(backend->deinit_count_, 1);
}

TEST(SdkClient, CallbackExceptionsAreRetainedForDiagnostics) {
  auto backend = std::make_shared<Backend>();
  SdkClient client(Functions(backend));
  SdkCallbacks callbacks;
  callbacks.heartbeat_ = [] {
        throw std::runtime_error("callback failure");
      };
  ASSERT_TRUE(client.Init(callbacks).Succeeded());
  backend->config_.heartbeatCb(nullptr, 0);
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  auto statistics = client.GetCallbackStatistics();
  EXPECT_EQ(statistics.callback_errors_, 1U);
  ASSERT_NE(statistics.last_exception_, nullptr);
  EXPECT_THROW(std::rethrow_exception(statistics.last_exception_),
      std::runtime_error);
}

TEST(SdkClient, ClosedCallbackDoesNotReadRetiredSdkPayload) {
  auto backend = std::make_shared<Backend>();
  SdkClient client(Functions(backend));
  ASSERT_TRUE(client.Init({}).Succeeded());
  auto late = backend->config_.sdkStatusCb;
  client.Deinit();
  // 使用不可读地址证明关闭检查早于解码，模拟 SDK 缓冲区已经回收。
  auto* retired = reinterpret_cast<void*>(static_cast<uintptr_t>(1));
  late(retired, sizeof(AstrallSdkStatus));
  EXPECT_EQ(client.GetCallbackStatistics().invalid_payloads_, 0U);
}

TEST(SdkClient, NativeApiCallsAreSerializedAcrossCallerThreads) {
  auto functions = Functions(std::make_shared<Backend>());
  std::atomic<int> active{ 0 };
  std::atomic<int> calls{ 0 };
  std::atomic<bool> overlap{ false };
  functions.heartbeat_ = [&active, &calls, &overlap](uint32_t) {
        if (active.fetch_add(1) != 0) {
          overlap = true;
        }
        std::this_thread::yield();
        ++calls;
        --active;
        return ASTRALL_RES_SUCCESSED;
      };
  SdkClient client(functions);
  ASSERT_TRUE(client.Init({}).Succeeded());
  std::vector<std::thread> threads;
  for (int index = 0; index < 4; ++index) {
    threads.emplace_back([&client] {
          for (int call = 0; call < 100; ++call) {
            EXPECT_TRUE(client.Heartbeat().Succeeded());
          }
        });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(calls, 400);
  EXPECT_FALSE(overlap);
}

}  // namespace
}  // namespace astrall_driver
