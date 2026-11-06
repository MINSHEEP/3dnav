// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "astrall_driver/driver_core.hpp"
#include "astrall_driver/fake_sdk_client.hpp"
#include "gtest/gtest.h"
#include "sdk_test_backend.hpp"

namespace astrall_driver {
namespace {

constexpr auto kWait = std::chrono::seconds(2);

bool WaitUntil(const std::function<bool()>& condition) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

void ConfirmAuthority(DriverCore* core, FakeSdkClient* client) {
  ASSERT_TRUE(core->RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  EXPECT_FALSE(core->Snapshot().has_control_authority_);
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  ASSERT_TRUE(client->EmitStatus(status));
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  EXPECT_TRUE(core->Snapshot().has_control_authority_);
}

TEST(DriverCore, DefaultsDoNotAcquireAuthorityAndLifecycleIsIdempotent) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  EXPECT_FALSE(core.Snapshot().configured_);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  EXPECT_TRUE(core.Snapshot().connected_);
  EXPECT_FALSE(core.Snapshot().active_);
  ASSERT_TRUE(core.Activate().Succeeded());
  const auto calls = client->Calls();
  EXPECT_TRUE(std::none_of(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kAuthControl ||
        call.operation_ == SdkOperation::kMove;
      }));
  ConfirmAuthority(&core, client.get());
  ASSERT_TRUE(core.Deactivate().Succeeded());
  EXPECT_FALSE(core.Snapshot().connected_);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  EXPECT_TRUE(core.Snapshot().zero_attempted_);
  EXPECT_TRUE(core.Snapshot().release_attempted_);
  const auto stopped = client->Calls().size();
  EXPECT_TRUE(core.Deactivate().Succeeded());
  EXPECT_TRUE(core.Cleanup().Succeeded());
  EXPECT_TRUE(core.Cleanup().Succeeded());
  EXPECT_TRUE(core.Shutdown().Succeeded());
  EXPECT_TRUE(core.Shutdown().Succeeded());
  EXPECT_EQ(client->Calls().size(), stopped);
  EXPECT_EQ(core.Configure({}).Value(), ASTRALL_RES_NOT_INIT);
  for (const auto& call : client->Calls()) {
    if (call.operation_ == SdkOperation::kMove) {
      EXPECT_FLOAT_EQ(call.velocity_[0], 0);
      EXPECT_FLOAT_EQ(call.velocity_[1], 0);
      EXPECT_FLOAT_EQ(call.velocity_[2], 0);
    }
  }
}

TEST(DriverCore,
    InvalidConfigurationAndInitFailureNeverLeaveThreadsOrAuthority) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  DriverParameters parameters;
  parameters.sdk_timeout_ms_ = 999;
  EXPECT_EQ(core.Configure(parameters).Value(), ASTRALL_RES_INVALID_PARAM);
  EXPECT_TRUE(client->Calls().empty());
  client->SetResult(SdkOperation::kInit, ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(core.Configure({}).Value(), ASTRALL_RES_TIMEOUT);
  EXPECT_FALSE(core.Snapshot().configured_);
  EXPECT_FALSE(core.Snapshot().connected_);
  EXPECT_FALSE(core.Snapshot().zero_attempted_);
  EXPECT_NE(core.Snapshot().reason_.find("ASTRALL_RES_TIMEOUT"),
      std::string::npos);
  const auto stopped = client->Calls().size();
  EXPECT_TRUE(core.Cleanup().Succeeded());
  EXPECT_EQ(client->Calls().size(), stopped);
  EXPECT_THROW(DriverCore(nullptr), std::invalid_argument);
}

TEST(DriverCore, AuthorityDenialAndAckWithoutConfirmationDoNotGrantControl) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  EXPECT_EQ(core.RequestAuthority(ASTRALL_AUTH_SDK).Value(),
      ASTRALL_RES_NOT_INIT);
  ASSERT_TRUE(core.Activate().Succeeded());
  EXPECT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  client->SetResult(SdkOperation::kAuthControl, ASTRALL_RES_RC_NORELEASE);
  EXPECT_EQ(core.RequestAuthority(ASTRALL_AUTH_SDK).Value(),
      ASTRALL_RES_RC_NORELEASE);
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().zero_attempted_;
      }));
  EXPECT_TRUE(core.Snapshot().faulted_);
  EXPECT_FALSE(core.Snapshot().active_);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  EXPECT_NE(core.Snapshot().reason_.find("ASTRALL_RES_RC_NORELEASE"),
      std::string::npos);
}

TEST(DriverCore, DisconnectLossHeartbeatAndSystemErrorsCauseObservableZero) {
  for (int cause = 0; cause < 6; ++cause) {
    auto client = std::make_shared<FakeSdkClient>();
    DriverCore core(client);
    ASSERT_TRUE(core.Configure({}).Succeeded());
    ASSERT_TRUE(core.Activate().Succeeded());
    ConfirmAuthority(&core, client.get());
    if (cause < 2) {
      AstrallSdkStatus status;
      status.link = cause == 1;
      client->EmitStatus(status);
    } else if (cause == 2) {
      client->SetResult(SdkOperation::kHeartbeat, ASTRALL_RES_TIMEOUT);
    } else if (cause == 3) {
      AstrallSystemStatus status;
      status.errorCode = ASTRALL_ERR_STO_TRIGGERED;
      client->SetSystemStatus(status);
    } else if (cause == 4) {
      client->SetResult(SdkOperation::kGetSystemStatus, ASTRALL_RES_FAILED);
    } else {
      client->EmitRawStatus(nullptr, sizeof(AstrallSdkStatus));
    }
    ASSERT_TRUE(WaitUntil([&] {
          return core.Snapshot().zero_attempted_;
        }));
    const auto snapshot = core.Snapshot();
    EXPECT_TRUE(snapshot.faulted_);
    EXPECT_FALSE(snapshot.active_);
    EXPECT_FALSE(snapshot.has_control_authority_);
    EXPECT_TRUE(snapshot.release_attempted_);
    const auto calls = client->Calls();
    EXPECT_EQ(std::count_if(calls.begin(), calls.end(), [](const auto& call) {
          return call.operation_ == SdkOperation::kAuthControl &&
          call.argument_ == ASTRALL_AUTH_SDK;
        }), 1);
  }
}

TEST(DriverCore, HeartbeatOnlyBudgetReachesSdkAndFailureStillLatches) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  DriverParameters parameters;
  parameters.heartbeat_timeout_ms_ = 20;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().heartbeat_count_ > 0;
      }));
  client->SetResult(SdkOperation::kHeartbeat, ASTRALL_RES_TIMEOUT);
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().zero_attempted_;
      }));
  EXPECT_TRUE(core.Snapshot().faulted_);
  EXPECT_FALSE(core.Snapshot().active_);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  ASSERT_TRUE(WaitUntil([&] {
        const auto calls = client->Calls();
        return std::any_of(calls.begin(), calls.end(), [](const auto& call) {
          return call.operation_ == SdkOperation::kMove;
        });
      }));
  bool heartbeat = false;
  bool zero = false;
  for (const auto& call : client->Calls()) {
    if (call.operation_ == SdkOperation::kHeartbeat) {
      heartbeat = true;
      EXPECT_EQ(call.timeout_ms_, 20U);
    } else if (call.operation_ == SdkOperation::kMove) {
      zero = true;
      EXPECT_EQ(call.timeout_ms_, 2U);
      EXPECT_EQ(call.velocity_, (std::array<float, 3>{ 0, 0, 0 }));
    } else if (call.operation_ != SdkOperation::kInit &&
        call.operation_ != SdkOperation::kDeinit) {
      EXPECT_LE(call.timeout_ms_, 5U);
    }
  }
  EXPECT_TRUE(heartbeat);
  EXPECT_TRUE(zero);
}

TEST(DriverCore, FailedZeroIsNotPresentedAsDeliveredOrAcknowledged) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  client->SetResult(SdkOperation::kMove, ASTRALL_RES_WITHOUT_AUTH);
  core.Deactivate();
  const auto snapshot = core.Snapshot();
  EXPECT_TRUE(snapshot.zero_attempted_);
  EXPECT_EQ(snapshot.zero_result_, ASTRALL_RES_WITHOUT_AUTH);
  EXPECT_FALSE(snapshot.connected_);
  EXPECT_FALSE(snapshot.has_control_authority_);
}

/** @brief 包装真实封装的状态回调，验证回调等待期间关闭不持 core 状态锁。 */
class ObservedClient : public SdkClient {
 public:
  explicit ObservedClient(SdkFunctions functions)
    : SdkClient(std::move(functions)) {
  }
  SdkResult Init(const SdkCallbacks& callbacks, uint32_t timeout) override {
    old_status_ = callbacks.status_;
    auto decorated = callbacks;
    decorated.status_ =
        [this, callback = callbacks.status_](const auto& status) {
          if (hook_) {
            hook_();
          }
          callback(status);
          if (after_status_) {
            after_status_();
          }
        };
    auto result = SdkClient::Init(decorated, timeout);
    if (result.Succeeded() && emit_initial_) {
      callbacks.status_(initial_status_);
    }
    return result;
  }
  std::function<void()> hook_;
  std::function<void()> after_status_;
  std::function<void(const AstrallSdkStatus&)> old_status_;
  bool emit_initial_ = false;
  AstrallSdkStatus initial_status_;
};

TEST(DriverCore,
    StatusDuringInitCannotLeaveUnexpectedAuthorityOrFalseConnection) {
  for (bool authority : { false, true }) {
    auto backend = std::make_shared<test::Backend>();
    auto client = std::make_shared<ObservedClient>(test::Functions(backend));
    client->emit_initial_ = true;
    client->initial_status_.link = authority;
    client->initial_status_.ctrlAuthority = authority;
    DriverCore core(client);
    EXPECT_FALSE(core.Configure({}).Succeeded());
    const auto state = core.Snapshot();
    EXPECT_FALSE(state.connected_);
    EXPECT_FALSE(state.configured_);
    EXPECT_FALSE(state.has_control_authority_);
    EXPECT_TRUE(state.zero_attempted_);
    EXPECT_EQ(state.release_attempted_, authority);
    EXPECT_TRUE(state.faulted_);
    EXPECT_EQ(backend->deinit_count_, 1);
  }
}

TEST(DriverCore, ReentrantStatusSnapshotDuringCloseAndOldSessionAreSafe) {
  auto backend = std::make_shared<test::Backend>();
  auto functions = test::Functions(backend);
  std::promise<void> deinit_entered;
  auto deinitializing = deinit_entered.get_future();
  std::atomic<bool> notified{ false };
  functions.deinit_ = [&] {
        if (!notified.exchange(true)) {
          deinit_entered.set_value();
        }
      };
  auto client = std::make_shared<ObservedClient>(functions);
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  auto old = client->old_status_;
  std::promise<void> callback_entered;
  auto entered = callback_entered.get_future();
  std::promise<void> release;
  auto gate = release.get_future().share();
  client->hook_ = [&] {
        callback_entered.set_value();
        gate.wait();
        EXPECT_EQ(core.Snapshot().phase_, DriverPhase::kClosing);
      };
  AstrallSdkStatus status;
  status.link = 1;
  backend->config_.sdkStatusCb(&status, sizeof(status));
  const auto callback_status = entered.wait_for(kWait);
  auto stopped = std::async(std::launch::async, [&] {
        return core.Deactivate();
      });
  const auto close_status = deinitializing.wait_for(kWait);
  release.set_value();
  EXPECT_EQ(callback_status, std::future_status::ready);
  EXPECT_EQ(close_status, std::future_status::ready);
  ASSERT_EQ(stopped.wait_for(kWait), std::future_status::ready);
  EXPECT_TRUE(stopped.get().Succeeded());
  client->hook_ = {};
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  status.ctrlAuthority = 1;
  old(status);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
}

TEST(DriverCore, OldHeldStatusDuringExplicitReleaseDoesNotFaultOrRegrant) {
  auto backend = std::make_shared<test::Backend>();
  auto functions = test::Functions(backend);
  std::promise<void> release_entered;
  auto releasing_sdk = release_entered.get_future();
  std::promise<void> allow_ack;
  auto ack_gate = allow_ack.get_future().share();
  std::promise<void> old_processed;
  auto old_seen = old_processed.get_future();
  std::promise<void> false_processed;
  auto false_seen = false_processed.get_future();
  std::atomic<bool> observe{ false };
  std::atomic<int> statuses{ 0 };
  functions.auth_ = [&](AstrallAuth authority, uint32_t) {
        if (authority == ASTRALL_AUTH_JOYSTICK) {
          release_entered.set_value();
          ack_gate.wait();
        }
        return ASTRALL_RES_SUCCESSED;
      };
  auto client = std::make_shared<ObservedClient>(functions);
  client->after_status_ = [&] {
        if (observe) {
          const auto count = ++statuses;
          if (count == 1) {
            old_processed.set_value();
          } else if (count == 2) {
            false_processed.set_value();
          }
        }
      };
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  backend->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(client->WaitForCallbacks(1000));
  ASSERT_TRUE(core.Snapshot().has_control_authority_);
  auto release_result = std::async(std::launch::async, [&] {
        return core.RequestAuthority(ASTRALL_AUTH_JOYSTICK);
      });
  const auto entered = releasing_sdk.wait_for(kWait);
  observe = true;
  backend->config_.sdkStatusCb(&status, sizeof(status));
  const auto old_done = old_seen.wait_for(kWait);
  const auto pending = core.Snapshot();
  status.ctrlAuthority = 0;
  backend->config_.sdkStatusCb(&status, sizeof(status));
  const auto false_done = false_seen.wait_for(kWait);
  allow_ack.set_value();
  EXPECT_EQ(entered, std::future_status::ready);
  EXPECT_EQ(old_done, std::future_status::ready);
  EXPECT_EQ(false_done, std::future_status::ready);
  EXPECT_TRUE(pending.zero_attempted_);
  EXPECT_TRUE(pending.authority_release_pending_);
  EXPECT_FALSE(pending.has_control_authority_);
  EXPECT_FALSE(pending.faulted_);
  ASSERT_EQ(release_result.wait_for(kWait), std::future_status::ready);
  EXPECT_TRUE(release_result.get().Succeeded());
  EXPECT_FALSE(core.Snapshot().authority_release_pending_);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  EXPECT_FALSE(core.Snapshot().faulted_);
  EXPECT_TRUE(core.Snapshot().active_);
}

TEST(DriverCore, CloseCancelsInFlightGrantAndRejectsItsLateResult) {
  auto backend = std::make_shared<test::Backend>();
  auto functions = test::Functions(backend);
  std::promise<void> grant_entered;
  auto entered = grant_entered.get_future();
  std::promise<void> allow_grant;
  auto gate = allow_grant.get_future().share();
  std::atomic<int> releases{ 0 };
  functions.auth_ = [&](AstrallAuth authority, uint32_t) {
        if (authority == ASTRALL_AUTH_SDK) {
          grant_entered.set_value();
          gate.wait();
        } else {
          ++releases;
        }
        return ASTRALL_RES_SUCCESSED;
      };
  auto client = std::make_shared<ObservedClient>(functions);
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  const auto old = client->old_status_;
  auto granting = std::async(std::launch::async, [&] {
        return core.RequestAuthority(ASTRALL_AUTH_SDK);
      });
  const auto grant_status = entered.wait_for(kWait);
  auto closing = std::async(std::launch::async, [&] {
        return core.Deactivate();
      });
  const bool closing_seen = WaitUntil([&] {
        return core.Snapshot().phase_ == DriverPhase::kClosing;
      });
  allow_grant.set_value();
  EXPECT_EQ(grant_status, std::future_status::ready);
  EXPECT_TRUE(closing_seen);
  ASSERT_EQ(granting.wait_for(kWait), std::future_status::ready);
  EXPECT_EQ(granting.get().Value(), ASTRALL_RES_NOT_INIT);
  ASSERT_EQ(closing.wait_for(kWait), std::future_status::ready);
  EXPECT_TRUE(closing.get().Succeeded());
  EXPECT_EQ(releases, 1);
  EXPECT_TRUE(core.Activate().Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  old(status);
  EXPECT_FALSE(core.Snapshot().has_control_authority_);
  EXPECT_FALSE(core.Snapshot().faulted_);
}

TEST(DriverCore, ZeroFailureDuringExplicitReleaseLatchesFaultAndStillReleases) {
  auto client = std::make_shared<FakeSdkClient>();
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ConfirmAuthority(&core, client.get());
  client->SetResult(SdkOperation::kMove, ASTRALL_RES_FAILED);
  EXPECT_FALSE(core.RequestAuthority(ASTRALL_AUTH_JOYSTICK).Succeeded());
  const auto state = core.Snapshot();
  EXPECT_TRUE(state.faulted_);
  EXPECT_FALSE(state.active_);
  EXPECT_FALSE(state.has_control_authority_);
  EXPECT_EQ(state.zero_result_, ASTRALL_RES_FAILED);
  EXPECT_NE(state.reason_.find("ZERO_MOVE ASTRALL_RES_FAILED"),
      std::string::npos);
  const auto calls = client->Calls();
  EXPECT_TRUE(std::any_of(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kAuthControl &&
        call.argument_ == ASTRALL_AUTH_JOYSTICK;
      }));
}

}  // namespace
}  // namespace astrall_driver
