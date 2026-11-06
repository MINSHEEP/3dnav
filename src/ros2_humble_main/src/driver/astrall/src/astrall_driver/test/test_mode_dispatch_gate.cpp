// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "astrall_driver/driver_core.hpp"
#include "astrall_driver/sdk_client.hpp"
#include "sdk_test_backend.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

using namespace std::chrono_literals;
using Time = std::chrono::steady_clock::time_point;
constexpr uint32_t kRawTimeoutMs = 5;
constexpr int kModeTimeoutMs = 100;
constexpr int kCancellationTimeoutMs = 500;

enum class Cancellation { kNone, kDamping, kDisconnect, kDeactivate };

struct RawEvent {
  std::string name_;
  Time entry_;
  Time exit_;
  std::thread::id thread_;
};

struct GateTrace {
  std::mutex mutex_;
  std::vector<RawEvent> events_;
  std::atomic<bool> hold_next_{ false };
  std::atomic<uint16_t> actual_{ ASTRALL_SPORT_STATUS_FIXEDSTAND };
  std::promise<void> dispatch_;
  std::promise<void> locked_;
  std::shared_future<void> dispatch_ready_ = dispatch_.get_future().share();
  std::shared_future<void> lock_ready_ = locked_.get_future().share();
  Time wrapper_entry_;
  std::thread::id mode_thread_;
};

/** @brief 计时所有原始函数，验证竞争没有突破配置的 5ms 原始调用预算。 */
class RawSpan {
 public:
  RawSpan(GateTrace* trace, const std::string& name)
    : trace_(trace), event_{name, std::chrono::steady_clock::now(), {},
        std::this_thread::get_id()} {
  }
  ~RawSpan() {
    event_.exit_ = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(trace_->mutex_);
    trace_->events_.push_back(event_);
  }

 private:
  GateTrace* trace_;
  RawEvent event_;
};

template <typename Function>
auto RecordRaw(const std::shared_ptr<GateTrace>& trace,
    const std::string& name, Function function) {
  return [trace, name, function](auto&& ... args) -> decltype(auto) {
    RawSpan span(trace.get(), name);
    return function(std::forward<decltype(args)>(args)...);
  };
}

SdkFunctions GateFunctions(const std::shared_ptr<test::Backend>& backend,
    const std::shared_ptr<GateTrace>& trace) {
  auto functions = test::Functions(backend);
  functions.heartbeat_ = [trace](uint32_t timeout) {
        EXPECT_EQ(timeout, kRawTimeoutMs);
        if (trace->hold_next_.exchange(false)) {
          trace->locked_.set_value();
          std::this_thread::sleep_for(4ms);
        }
        return ASTRALL_RES_SUCCESSED;
      };
  functions.get_sport_ = [trace] {
        return trace->actual_.load();
      };
  functions.mode_ = [trace](AstrallSportModeCmd command, uint32_t timeout) {
        EXPECT_EQ(timeout, kRawTimeoutMs);
        RawSpan span(trace.get(), command == ASTRALL_SPORT_MODE_CMD_MOVE ?
            "mode_move" : "mode_damping");
        return ASTRALL_RES_SUCCESSED;
      };
  functions.init_ = RecordRaw(trace, "init", functions.init_);
  functions.deinit_ = RecordRaw(trace, "deinit", functions.deinit_);
  functions.heartbeat_ = RecordRaw(trace, "heartbeat", functions.heartbeat_);
  functions.get_system_ = RecordRaw(trace, "system", functions.get_system_);
  functions.get_power_ = RecordRaw(trace, "power", functions.get_power_);
  functions.get_sport_ = RecordRaw(trace, "sport", functions.get_sport_);
  functions.get_device_ = RecordRaw(trace, "device", functions.get_device_);
  functions.get_imu_ = RecordRaw(trace, "imu", functions.get_imu_);
  functions.subscribe_ = RecordRaw(trace, "subscribe", functions.subscribe_);
  functions.auth_ = RecordRaw(trace, "auth", functions.auth_);
  functions.move_ = RecordRaw(trace, "move_zero", functions.move_);
  functions.light_ = RecordRaw(trace, "light", functions.light_);
  functions.send_message_ = RecordRaw(trace, "text", functions.send_message_);
  return functions;
}

/** @brief 保留 FINAL-001 探针的包装层 hook；实际持锁仅发生在原始心跳内。 */
class GateClient : public SdkClient {
 public:
  explicit GateClient(SdkFunctions functions)
    : SdkClient(std::move(functions)) {
  }
  SdkResult SportModeControl(AstrallSportModeCmd command,
      uint32_t timeout, ModeDispatchGuard guard = {}) override {
    if (command == ASTRALL_SPORT_MODE_CMD_MOVE) {
      before_mode_();
    }
    return SdkClient::SportModeControl(command, timeout, std::move(guard));
  }
  std::function<void()> before_mode_;
};

bool Wait(const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50us);
  }
  return predicate();
}

double Ms(Time time, Time origin) {
  return std::chrono::duration<double, std::milli>(time - origin).count();
}

/** @brief 用真实 SdkClient 串行锁及完整注入表复现最终派发窗口。 */
class ModeDispatchGate : public testing::Test {
 protected:
  void Start(int mode_timeout_ms = kModeTimeoutMs) {
    DriverParameters parameters;
    parameters.imu_frequency_hz_ = 0;
    parameters.wheel_speed_frequency_hz_ = 0;
    parameters.joystick_frequency_hz_ = 0;
    parameters.control_period_ms_ = 25;
    parameters.control_sdk_timeout_ms_ = kRawTimeoutMs;
    parameters.mode_poll_period_ms_ = 100;
    parameters.mode_switch_timeout_ms_ = mode_timeout_ms;
    ASSERT_TRUE(parameters.Validate().empty());
    ASSERT_TRUE(core_.Configure(parameters).Succeeded());
    ASSERT_TRUE(core_.Activate().Succeeded());
    ASSERT_TRUE(Wait([&] {
          return core_.Snapshot().mode_actual_ ==
          ASTRALL_SPORT_STATUS_FIXEDSTAND;
        }));
    ASSERT_TRUE(core_.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
    AstrallSdkStatus status;
    status.link = 1;
    status.ctrlAuthority = 1;
    backend_->config_.sdkStatusCb(&status, sizeof(status));
    ASSERT_TRUE(Wait([&] {
          return core_.Snapshot().has_control_authority_;
        }));
    generation_ = core_.Snapshot().session_generation_;
    ASSERT_TRUE(core_.RequestLight(ASTRALL_CMD_LIGHT_OPEN,
        generation_).Succeeded());
  }

  void Run(bool hold, Cancellation cancellation = Cancellation::kNone,
      bool observe_completion = false) {
    client_->before_mode_ = [this, hold] {
          trace_->wrapper_entry_ = std::chrono::steady_clock::now();
          trace_->mode_thread_ = std::this_thread::get_id();
          if (hold) {
            trace_->hold_next_ = true;
            trace_->dispatch_.set_value();
            EXPECT_EQ(trace_->lock_ready_.wait_for(1s),
                std::future_status::ready);
          }
        };
    auto holder = std::async(std::launch::async, [this, hold] {
          if (hold && trace_->dispatch_ready_.wait_for(1s) ==
          std::future_status::ready) {
            EXPECT_TRUE(client_->Heartbeat(kRawTimeoutMs).Succeeded());
          }
        });
    auto canceller = std::async(std::launch::async, [this, cancellation] {
          if (cancellation == Cancellation::kNone) {
            return SdkResult(ASTRALL_RES_SUCCESSED);
          }
          if (trace_->lock_ready_.wait_for(1s) != std::future_status::ready) {
            return SdkResult(ASTRALL_RES_TIMEOUT);
          }
          if (cancellation == Cancellation::kDamping) {
            return core_.RequestMode(ASTRALL_SPORT_MODE_CMD_DAMPING, false,
            generation_);
          }
          if (cancellation == Cancellation::kDeactivate) {
            return core_.Deactivate();
          }
          AstrallSdkStatus status;
          backend_->config_.sdkStatusCb(&status, sizeof(status));
          return SdkResult(ASTRALL_RES_SUCCESSED);
        });
    std::this_thread::sleep_for(1ms);
    auto request = std::async(std::launch::async, [this] {
          request_entry_ = std::chrono::steady_clock::now();
          return core_.RequestMode(ASTRALL_SPORT_MODE_CMD_MOVE, false,
          generation_);
        });
    ASSERT_TRUE(Wait([this] {
          return core_.Snapshot().mode_pending_;
        }));
    pending_seen_ = std::chrono::steady_clock::now();
    old_ticket_ = core_.Snapshot().mode_ticket_;
    if (cancellation != Cancellation::kNone) {
      ASSERT_EQ(trace_->lock_ready_.wait_for(1s), std::future_status::ready);
      ASSERT_TRUE(Wait([this, cancellation] {
            const auto snapshot = core_.Snapshot();
            if (cancellation == Cancellation::kDamping) {
              return snapshot.mode_ticket_ != old_ticket_;
            }
            if (cancellation == Cancellation::kDisconnect) {
              return !snapshot.connected_;
            }
            return snapshot.session_generation_ != generation_;
          }));
      cancellation_seen_ = std::chrono::steady_clock::now();
    }
    EXPECT_EQ(request.wait_for(1s), std::future_status::ready);
    response_ = request.get().Value();
    holder.get();
    EXPECT_EQ(canceller.wait_for(1s), std::future_status::ready);
    cancellation_result_ = canceller.get().Value();
    if (observe_completion) {
      ASSERT_EQ(response_, ASTRALL_RES_RUNNING);
      EXPECT_TRUE(core_.Snapshot().mode_pending_);
      trace_->actual_ = ASTRALL_SPORT_STATUS_MOVE;
      ASSERT_TRUE(Wait([this] {
            return !core_.Snapshot().mode_pending_;
          }));
    }
    after_ = core_.Snapshot();
    core_.Shutdown();
  }

  void CheckCancellation() {
    EXPECT_EQ(Count("mode_move"), 0U);
    bool observed = false;
    for (const auto& event : trace_->events_) {
      EXPECT_LT(Ms(event.exit_, event.entry_), kRawTimeoutMs) << event.name_;
      if (event.name_ == "heartbeat" && event.exit_ - event.entry_ >= 3ms) {
        observed = true;
        EXPECT_GE(cancellation_seen_, event.entry_);
        EXPECT_LT(cancellation_seen_, event.exit_);
        std::cout << "FINAL-001 cancellation_seen=" <<
          Ms(cancellation_seen_, pending_seen_) << " held_raw_exit=" <<
          Ms(event.exit_, pending_seen_) << " held_raw_duration=" <<
          Ms(event.exit_, event.entry_) << " old_mode_calls=" <<
          Count("mode_move") << " damping_calls=" <<
          Count("mode_damping") << '\n';
      }
    }
    EXPECT_TRUE(observed);
    EXPECT_FALSE(after_.command_active_);
  }

  size_t Count(const std::string& name) const {
    return std::count_if(trace_->events_.begin(), trace_->events_.end(),
               [&name](const auto& event) {
          return event.name_ == name;
        });
  }

  void CheckTrace(bool hold) {
    double maximum = 0;
    Time zero_entry;
    Time zero_exit;
    std::cout << "FINAL-001 hold=" << hold << " request_entry=" <<
      Ms(request_entry_, pending_seen_) << " deadline_interval=[" <<
      Ms(request_entry_, pending_seen_) + kModeTimeoutMs << ",100]";
    for (const auto& event : trace_->events_) {
      maximum = std::max(maximum, Ms(event.exit_, event.entry_));
      EXPECT_LT(Ms(event.exit_, event.entry_), kRawTimeoutMs) << event.name_;
      if (event.name_ == "mode_move") {
        std::cout << " raw_mode_entry=" << Ms(event.entry_, pending_seen_);
        EXPECT_LT(event.entry_, request_entry_ + 100ms);
      }
      if (event.name_ == "move_zero" &&
          event.thread_ == trace_->mode_thread_ &&
          event.exit_ < trace_->wrapper_entry_) {
        zero_entry = event.entry_;
        zero_exit = event.exit_;
      }
    }
    EXPECT_NE(zero_entry, Time{});
    EXPECT_LT(zero_exit, trace_->wrapper_entry_);
    EXPECT_LT(trace_->wrapper_entry_, request_entry_ + 100ms);
    std::cout << " zero_entry=" << Ms(zero_entry, pending_seen_) <<
      " zero_exit=" << Ms(zero_exit, pending_seen_) <<
      " wrapper_entry=" << Ms(trace_->wrapper_entry_, pending_seen_) <<
      " mode_calls=" << Count("mode_move") << " max_raw_call_ms=" <<
      maximum << " response=" << SdkResult(response_).ToString() << '\n';
  }

  std::shared_ptr<test::Backend> backend_ = std::make_shared<test::Backend>();
  std::shared_ptr<GateTrace> trace_ = std::make_shared<GateTrace>();
  std::shared_ptr<GateClient> client_ = std::make_shared<GateClient>(
    GateFunctions(backend_, trace_));
  DriverCore core_{ client_ };
  uint64_t generation_ = 0;
  uint64_t old_ticket_ = 0;
  Time request_entry_;
  Time pending_seen_;
  Time cancellation_seen_;
  DriverSnapshot after_;
  uint16_t response_ = ASTRALL_RES_FAILED;
  uint16_t cancellation_result_ = ASTRALL_RES_FAILED;
};

TEST_F(ModeDispatchGate, UncontendedRequestEntersRawSdkBeforeDeadline) {
  ASSERT_NO_FATAL_FAILURE(Start());
  ASSERT_NO_FATAL_FAILURE(Run(false));
  EXPECT_EQ(response_, ASTRALL_RES_RUNNING);
  EXPECT_EQ(Count("mode_move"), 1U);
  CheckTrace(false);
}

TEST_F(ModeDispatchGate, ApiLockWaitPastDeadlineNeverEntersRawMode) {
  ASSERT_NO_FATAL_FAILURE(Start());
  ASSERT_NO_FATAL_FAILURE(Run(true));
  EXPECT_EQ(response_, ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(Count("mode_move"), 0U);
  CheckTrace(true);
}

TEST_F(ModeDispatchGate, DampingDuringApiWaitCancelsOnlyOldModeTicket) {
  ASSERT_NO_FATAL_FAILURE(Start(kCancellationTimeoutMs));
  ASSERT_NO_FATAL_FAILURE(Run(true, Cancellation::kDamping));
  EXPECT_EQ(response_, ASTRALL_RES_FAILED);
  EXPECT_EQ(cancellation_result_, ASTRALL_RES_RUNNING);
  EXPECT_EQ(Count("mode_damping"), 1U);
  EXPECT_GT(after_.mode_ticket_, old_ticket_);
  EXPECT_EQ(after_.mode_ack_, ASTRALL_RES_SUCCESSED);
  EXPECT_EQ(after_.mode_result_, ASTRALL_RES_RUNNING);
  EXPECT_TRUE(after_.mode_pending_);
  CheckCancellation();
}

TEST_F(ModeDispatchGate, DisconnectDuringApiWaitNeverEntersOldRawMode) {
  ASSERT_NO_FATAL_FAILURE(Start(kCancellationTimeoutMs));
  ASSERT_NO_FATAL_FAILURE(Run(true, Cancellation::kDisconnect));
  EXPECT_EQ(response_, ASTRALL_RES_NOT_INIT);
  EXPECT_TRUE(after_.faulted_);
  EXPECT_FALSE(after_.connected_);
  EXPECT_EQ(Count("mode_damping"), 0U);
  CheckCancellation();
}

TEST_F(ModeDispatchGate, DeactivateDuringApiWaitNeverEntersOldRawMode) {
  ASSERT_NO_FATAL_FAILURE(Start(kCancellationTimeoutMs));
  ASSERT_NO_FATAL_FAILURE(Run(true, Cancellation::kDeactivate));
  EXPECT_EQ(response_, ASTRALL_RES_NOT_INIT);
  EXPECT_EQ(cancellation_result_, ASTRALL_RES_SUCCESSED);
  EXPECT_GT(after_.session_generation_, generation_);
  EXPECT_FALSE(after_.active_);
  EXPECT_EQ(Count("mode_damping"), 0U);
  CheckCancellation();
}

TEST_F(ModeDispatchGate, AdmittedModeStillWaitsForObservedPhysicalState) {
  ASSERT_NO_FATAL_FAILURE(Start(kCancellationTimeoutMs));
  ASSERT_NO_FATAL_FAILURE(Run(false, Cancellation::kNone, true));
  EXPECT_EQ(response_, ASTRALL_RES_RUNNING);
  EXPECT_EQ(Count("mode_move"), 1U);
  EXPECT_EQ(after_.mode_actual_, ASTRALL_SPORT_STATUS_MOVE);
  EXPECT_EQ(after_.mode_result_, ASTRALL_RES_SUCCESSED);
  EXPECT_FALSE(after_.mode_pending_);
}

}  // namespace
}  // namespace astrall_driver
