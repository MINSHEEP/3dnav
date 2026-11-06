// Copyright 2026 Hypertron ROS 2 contributors

#include <atomic>
#include <chrono>
#include <future>
#include <memory>

#include "astrall_driver/sdk_client.hpp"
#include "gtest/gtest.h"
#include "sdk_test_backend.hpp"

namespace astrall_driver {
namespace {

constexpr auto kWait = std::chrono::seconds(2);
constexpr uint32_t kWaitMs = 2000;

TEST(SdkCallbacks,
    ReentryWhileAnotherThreadCancelsOrDeinitializesDoesNotDeadlock) {
  for (bool deinitialize : { false, true }) {
    auto backend = std::make_shared<test::Backend>();
    auto functions = test::Functions(backend);
    std::promise<void> callback_entered;
    auto entered = callback_entered.get_future();
    std::promise<void> release_callback;
    auto callback_gate = release_callback.get_future().share();
    std::promise<void> closing_entered;
    auto closing = closing_entered.get_future();
    std::promise<void> release_closing;
    auto closing_gate = release_closing.get_future().share();
    std::promise<void> reentry_started;
    auto reentering = reentry_started.get_future();
    std::atomic<uint16_t> reentry_result{ 0 };
    auto block_closing = [&closing_entered, closing_gate] {
          closing_entered.set_value();
          closing_gate.wait();
        };
    functions.deinit_ = [backend, &block_closing, deinitialize] {
          ++backend->deinit_count_;
          if (deinitialize) {
            block_closing();
          }
        };
    functions.subscribe_ = [backend, &block_closing](AstrallSubscribeTopicId,
            AstrallSubscribeFreq frequency, RawSdkCallback callback, uint32_t) {
          if (frequency == ASTRALL_SUB_FREQ_CLOSE) {
            block_closing();
          } else {
            backend->subscription_ = std::move(callback);
          }
          return ASTRALL_RES_SUCCESSED;
        };
    SdkClient client(functions);
    ASSERT_TRUE(client.Init({}).Succeeded());
    ASSERT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU,
        ASTRALL_SUB_FREQ_50HZ,
        [&](const auto&) {
          callback_entered.set_value();
          callback_gate.wait();
          reentry_started.set_value();
          reentry_result = client.Heartbeat().Value();
        }).Succeeded());
    AstrallImuData data;
    backend->subscription_(&data, sizeof(data));
    auto entered_status = entered.wait_for(kWait);
    auto finished = std::async(std::launch::async, [&client, deinitialize] {
          if (deinitialize) {
            client.Deinit();
          } else {
            EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU,
            ASTRALL_SUB_FREQ_CLOSE).Succeeded());
          }
        });
    auto closing_status = closing.wait_for(kWait);
    release_callback.set_value();
    auto reentry_status = reentering.wait_for(kWait);
    // 先释放所有门闩再断言，避免失败路径把工作线程永远留在门闩上。
    release_closing.set_value();
    EXPECT_EQ(entered_status, std::future_status::ready);
    EXPECT_EQ(closing_status, std::future_status::ready);
    EXPECT_EQ(reentry_status, std::future_status::ready);
    ASSERT_EQ(finished.wait_for(kWait), std::future_status::ready);
    finished.get();
    EXPECT_EQ(reentry_result, deinitialize ? ASTRALL_RES_NOT_INIT :
        ASTRALL_RES_SUCCESSED);
  }
}

TEST(SdkCallbacks, SubscriptionCanCancelItselfAndRejectLateData) {
  auto backend = std::make_shared<test::Backend>();
  SdkClient client(test::Functions(backend));
  ASSERT_TRUE(client.Init({}).Succeeded());
  std::atomic<int> calls{ 0 };
  ASSERT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_50HZ,
      [&client, &calls](const auto&) {
        ++calls;
        EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU,
        ASTRALL_SUB_FREQ_CLOSE).Succeeded());
      }).Succeeded());
  auto late = backend->subscription_;
  AstrallImuData data;
  late(&data, sizeof(data));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  late(&data, sizeof(data));
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_EQ(calls, 1);
}

TEST(SdkCallbacks, QueueIsBoundedAndOverflowIsObservable) {
  auto backend = std::make_shared<test::Backend>();
  SdkClient client(test::Functions(backend));
  ASSERT_TRUE(client.Init({}).Succeeded());
  std::promise<void> first_entered;
  auto entered = first_entered.get_future();
  std::promise<void> release;
  auto gate = release.get_future().share();
  std::atomic<int> calls{ 0 };
  ASSERT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_250HZ,
      [&calls, &first_entered, gate](const auto&) {
        if (calls.fetch_add(1) == 0) {
          first_entered.set_value();
          gate.wait();
        }
      }).Succeeded());
  AstrallImuData data;
  backend->subscription_(&data, sizeof(data));
  auto started = entered.wait_for(kWait);
  for (int index = 0; index < 300; ++index) {
    backend->subscription_(&data, sizeof(data));
  }
  auto statistics = client.GetCallbackStatistics();
  release.set_value();
  EXPECT_EQ(started, std::future_status::ready);
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  EXPECT_GE(statistics.dropped_events_, 44U);
  EXPECT_LE(calls, 257);
}

TEST(SdkCallbacks, CapturedResourceDestructionCanReenterOnCancelOrDeinit) {
  for (bool deinitialize : { false, true }) {
    auto backend = std::make_shared<test::Backend>();
    SdkClient client(test::Functions(backend));
    ASSERT_TRUE(client.Init({}).Succeeded());
    std::promise<uint16_t> destroyed;
    auto result = destroyed.get_future();
    auto captured = std::shared_ptr<int>(new int{},
            [&client, &destroyed](int* p) {
          delete p;
          destroyed.set_value(client.Heartbeat().Value());
        });
    ASSERT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU,
        ASTRALL_SUB_FREQ_50HZ,
        [captured](const auto&) {
        }).Succeeded());
    captured.reset();
    auto done = std::async(std::launch::async, [&client, deinitialize] {
          if (deinitialize) {
            client.Deinit();
          } else {
            EXPECT_TRUE(client.Subscribe(ASTRALL_SUB_TOPIC_ID_IMU,
            ASTRALL_SUB_FREQ_CLOSE).Succeeded());
          }
        });
    ASSERT_EQ(result.wait_for(kWait), std::future_status::ready);
    EXPECT_EQ(result.get(), deinitialize ? ASTRALL_RES_NOT_INIT :
        ASTRALL_RES_SUCCESSED);
    ASSERT_EQ(done.wait_for(kWait), std::future_status::ready);
    done.get();
  }
}

}  // namespace
}  // namespace astrall_driver
