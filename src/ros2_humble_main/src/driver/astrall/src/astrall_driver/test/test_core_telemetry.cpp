// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "astrall_driver/driver_core.hpp"
#include "astrall_driver/fake_sdk_client.hpp"
#include "gtest/gtest.h"
#include "sdk_test_backend.hpp"

namespace astrall_driver {
namespace {

constexpr auto kWait = std::chrono::seconds(3);

bool WaitUntil(const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

template <typename T>
bool Has(const std::vector<TelemetrySample>& samples) {
  return std::any_of(samples.begin(), samples.end(), [](const auto& sample) {
        return std::holds_alternative<T>(sample.data_);
      });
}

TEST(CoreTelemetry, NoAuthorityIsRequiredAndLossKeepsExistingDataAlive) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client, [] {
      return int64_t{ 1234567890 };
        });
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().subscriptions_active_ == 3;
      }));
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  AstrallImuData imu;
  imu.timestamp = 42;
  ASSERT_TRUE(client->EmitImu(imu));
  ASSERT_TRUE(client->EmitSport(AstrallSportData{}));
  ASSERT_TRUE(client->EmitJoystick(AstrallJoystickData{}));
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  auto samples = core.TakeTelemetry();
  EXPECT_TRUE(Has<AstrallImuData>(samples));
  EXPECT_TRUE(Has<AstrallSportData>(samples));
  EXPECT_TRUE(Has<AstrallJoystickData>(samples));
  for (const auto& sample : samples) {
    EXPECT_EQ(sample.received_ros_ns_, 1234567890);
  }
  EXPECT_FALSE(Has<AstrallImuData>(core.TakeTelemetry()));
  ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  client->EmitStatus(status);
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  status.ctrlAuthority = 0;
  client->EmitStatus(status);
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  ASSERT_TRUE(core.Snapshot().faulted_);
  EXPECT_FALSE(core.Snapshot().active_);
  EXPECT_TRUE(client->EmitImu(imu));
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  EXPECT_TRUE(Has<AstrallImuData>(core.TakeTelemetry()));
}

TEST(CoreTelemetry, DisconnectDeactivateAndOldCallbacksCannotReplaySamples) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().subscriptions_active_ == 3;
      }));
  auto old = client->RawSubscriptionCallback(ASTRALL_SUB_TOPIC_ID_IMU);
  AstrallImuData imu;
  client->EmitImu(imu);
  client->EmitStatus(AstrallSdkStatus{});
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  EXPECT_TRUE(core.TakeTelemetry().empty());
  core.Deactivate();
  EXPECT_EQ(core.Snapshot().subscriptions_active_, 0U);
  EXPECT_FALSE(client->EmitImu(imu));
  old(&imu, sizeof(imu));
  EXPECT_TRUE(core.TakeTelemetry().empty());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().subscriptions_active_ == 3;
      }));
  old(&imu, sizeof(imu));
  EXPECT_FALSE(Has<AstrallImuData>(core.TakeTelemetry()));
  ASSERT_TRUE(client->EmitImu(imu));
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  EXPECT_TRUE(Has<AstrallImuData>(core.TakeTelemetry()));
  const auto calls = client->Calls();
  EXPECT_EQ(std::count_if(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kSubscribe &&
        call.frequency_ == ASTRALL_SUB_FREQ_CLOSE;
      }), 3);
}

TEST(CoreTelemetry, FreshnessQueriesAndRawLengthFailuresNeverRepeatOldData) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverParameters parameters;
  parameters.telemetry_max_age_ms_ = 20;
  parameters.telemetry_query_period_ms_ = 100;
  AstrallPowerStatus power;
  power.soc = 75;
  client->SetPowerStatus(power);
  DriverCore core(client);
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().subscriptions_active_ == 3;
      }));
  AstrallImuData imu;
  imu.timestamp = 1;
  client->EmitImu(imu);
  imu.timestamp = 2;
  client->EmitImu(imu);
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  auto samples = core.TakeTelemetry();
  for (const auto& sample : samples) {
    if (const auto* value = std::get_if<AstrallImuData>(&sample.data_)) {
      EXPECT_EQ(value->timestamp, 2);
    }
  }
  client->EmitImu(imu);
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(Has<AstrallImuData>(core.TakeTelemetry()));
  EXPECT_GT(core.Snapshot().telemetry_dropped_, 0U);
  client->EmitRawData(ASTRALL_SUB_TOPIC_ID_IMU, nullptr, sizeof(imu));
  EXPECT_FALSE(Has<AstrallImuData>(core.TakeTelemetry()));
  client->SetResult(SdkOperation::kGetPowerStatus, ASTRALL_RES_TIMEOUT);
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().reason_.find("TELEMETRY_QUERY") !=
        std::string::npos ||
        core.Snapshot().faulted_;
      }));
  core.TakeTelemetry();
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  EXPECT_FALSE(Has<AstrallPowerStatus>(core.TakeTelemetry()));
}

TEST(CoreTelemetry,
    StreamFlagsUseSingleSessionAndCloseEverySuccessfulSubscription) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  DriverParameters parameters;
  parameters.imu_frequency_hz_ = 0;
  parameters.wheel_speed_frequency_hz_ = 0;
  parameters.joystick_frequency_hz_ = 0;
  parameters.enable_rgb_ = true;
  parameters.enable_lidar_ = true;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().subscriptions_active_ == 2;
      }));
  core.Shutdown();
  const auto calls = client->Calls();
  EXPECT_EQ(std::count_if(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kInit;
      }), 1);
  for (auto topic :
      { ASTRALL_SUB_TOPIC_ID_CAMERA_RGB, ASTRALL_SUB_TOPIC_ID_LIDAR }) {
    EXPECT_EQ(std::count_if(calls.begin(), calls.end(), [&](const auto& call) {
          return call.operation_ == SdkOperation::kSubscribe &&
          call.topic_ == topic;
        }), 2);
  }
}

/** @brief 直接保留核心回调，避免 S1 弱寿命保护掩盖核心自己的旧代次检查。 */
class TelemetryClockClient : public SdkClient {
 public:
  explicit TelemetryClockClient(SdkFunctions functions) : SdkClient(std::move(
          functions)) {
  }
  SdkResult Subscribe(AstrallSubscribeTopicId topic,
      AstrallSubscribeFreq frequency,
      SubscriptionCallback callback = {},
      uint32_t timeout = kSdkTimeoutMs) override {
    if (topic == ASTRALL_SUB_TOPIC_ID_IMU &&
        frequency != ASTRALL_SUB_FREQ_CLOSE) {
      saved_ = callback;
    }
    return SdkClient::Subscribe(topic, frequency, std::move(callback), timeout);
  }
  SubscriptionCallback saved_;
};

TEST(CoreTelemetry,
    ThrowingReceiveClockIsDiagnosedAndInvalidSessionsDoNotCallIt) {
  std::atomic<bool> throwing{ false };
  std::atomic<int> calls{ 0 };
  auto clock = [&] {
        ++calls;
        if (throwing) {
          throw std::runtime_error("synthetic receive clock failure");
        }
        return int64_t{ 1234567890 };
      };
  auto backend = std::make_shared<test::Backend>();
  auto client =
      std::make_shared<TelemetryClockClient>(test::Functions(backend));
  DriverCore core(client, clock);
  DriverParameters parameters;
  parameters.wheel_speed_frequency_hz_ = 0;
  parameters.joystick_frequency_hz_ = 0;
  parameters.system_poll_period_ms_ = 100;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(calls, 0);
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().subscriptions_active_ == 1;
      }));
  auto old = client->saved_;
  AstrallImuData imu;
  old(SubscriptionData(imu));
  throwing = true;
  // 查询线程也会调用 clock，此路径必须自行捕获，不能依赖 SDK 分发器。
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().telemetry_clock_errors_ > 0;
      }));
  EXPECT_TRUE(core.Snapshot().faulted_);
  EXPECT_NE(core.Snapshot().telemetry_last_error_.find("RECEIVE_CLOCK"),
      std::string::npos);
  EXPECT_FALSE(Has<AstrallImuData>(core.TakeTelemetry()));
  core.Deactivate();
  const auto stopped_calls = calls.load();
  old(SubscriptionData(imu));
  EXPECT_EQ(calls, stopped_calls);
  EXPECT_TRUE(core.TakeTelemetry().empty());
}

TEST(CoreTelemetry, InFlightSubscribeIsAlsoCancelledDuringClose) {
  auto backend = std::make_shared<test::Backend>();
  auto functions = test::Functions(backend);
  std::promise<void> entered;
  auto started = entered.get_future();
  std::promise<void> allow;
  auto gate = allow.get_future().share();
  std::atomic<int> closed{ 0 };
  functions.subscribe_ =
      [&](AstrallSubscribeTopicId, AstrallSubscribeFreq frequency,
          RawSdkCallback, uint32_t) {
        if (frequency == ASTRALL_SUB_FREQ_CLOSE) {
          ++closed;
        } else {
          entered.set_value();
          gate.wait();
        }
        return ASTRALL_RES_SUCCESSED;
      };
  auto client = std::make_shared<SdkClient>(functions);
  DriverCore core(client);
  DriverParameters parameters;
  parameters.wheel_speed_frequency_hz_ = 0;
  parameters.joystick_frequency_hz_ = 0;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  const auto status = started.wait_for(kWait);
  auto stopping = std::async(std::launch::async, [&] {
        return core.Deactivate();
      });
  const bool closing = WaitUntil([&] {
        return core.Snapshot().phase_ == DriverPhase::kClosing;
      });
  allow.set_value();
  EXPECT_EQ(status, std::future_status::ready);
  EXPECT_TRUE(closing);
  ASSERT_EQ(stopping.wait_for(kWait), std::future_status::ready);
  EXPECT_TRUE(stopping.get().Succeeded());
  EXPECT_EQ(closed, 1);
}

TEST(CoreTelemetry, ShortSubscriptionsAndDeviceQueriesPreserveTenHzHeartbeat) {
  auto backend = std::make_shared<test::Backend>();
  auto functions = test::Functions(backend);
  auto trace_mutex = std::make_shared<std::mutex>();
  auto times =
      std::make_shared<std::vector<std::chrono::steady_clock::time_point>>();
  std::atomic<int> queries{ 0 };
  functions.heartbeat_ = [trace_mutex, times](uint32_t timeout) {
        EXPECT_LE(timeout, 20U);
        std::lock_guard<std::mutex> lock(*trace_mutex);
        times->push_back(std::chrono::steady_clock::now());
        return ASTRALL_RES_SUCCESSED;
      };
  functions.subscribe_ = [](AstrallSubscribeTopicId, AstrallSubscribeFreq,
          RawSdkCallback, uint32_t timeout) {
        EXPECT_LE(timeout, 20U);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return ASTRALL_RES_SUCCESSED;
      };
  functions.get_device_ = [&queries](AstrallDeviceInfo&, uint32_t timeout) {
        EXPECT_LE(timeout, 20U);
        ++queries;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return ASTRALL_RES_SUCCESSED;
      };
  auto client = std::make_shared<SdkClient>(functions);
  DriverCore core(client);
  DriverParameters parameters;
  parameters.enable_rgb_ = true;
  parameters.enable_lidar_ = true;
  parameters.telemetry_query_period_ms_ = 100;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        std::lock_guard<std::mutex> lock(*trace_mutex);
        return times->size() >= 12;
      }));
  EXPECT_EQ(core.Snapshot().subscriptions_active_, 5U);
  core.Shutdown();
  EXPECT_GE(queries, 2);
  std::cout << "S3 SDK heartbeat intervals under 20ms subscriptions/queries:";
  for (size_t index = 1; index < times->size(); ++index) {
    const auto interval = std::chrono::duration<double, std::milli>(
      (*times)[index] - (*times)[index - 1]).count();
    std::cout << ' ' << interval;
    EXPECT_GE(interval, 60.0);
    EXPECT_LE(interval, 180.0);
  }
  const auto mean = std::chrono::duration<double, std::milli>(
    times->back() - times->front()).count() / (times->size() - 1);
  std::cout << "; mean=" << mean << '\n';
  EXPECT_NEAR(mean, 100.0, 10.0);
}

}  // namespace
}  // namespace astrall_driver
