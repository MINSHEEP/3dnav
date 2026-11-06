// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "astrall_driver/driver_core.hpp"
#include "astrall_driver/sdk_client.hpp"
#include "gtest/gtest.h"
#include "control_test_backend.hpp"

namespace astrall_driver {
namespace {

using namespace std::chrono_literals;
constexpr auto kWait = 3s;

bool WaitUntil(const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

bool Nonzero(const test::MoveEntry& entry) {
  return entry.command_.forward_ != 0 || entry.command_.lateral_ != 0 ||
         entry.command_.yaw_ != 0;
}

void DriveReady(DriverCore* core,
    const std::shared_ptr<test::ControlBackend>& trace,
    const DriverParameters& parameters = {}) {
  ASSERT_TRUE(core->Configure(parameters).Succeeded());
  ASSERT_TRUE(core->Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core->Snapshot().mode_can_move_;
      }));
  ASSERT_TRUE(core->RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  trace->base_->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(WaitUntil([&] {
        return core->Snapshot().has_control_authority_;
      }));
}

SdkResult Submit(DriverCore* core, VelocityCommand command) {
  const auto state = core->Snapshot();
  return core->SubmitVelocity(command,
             state.session_generation_,
             state.motion_epoch_);
}

TEST(CoreControl, FixedResendUsesSiLimitsAndWatchdogNeedsFreshCommand) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  DriverCore core(client);
  DriveReady(&core, trace);
  EXPECT_TRUE(trace->Moves().empty());
  ASSERT_TRUE(Submit(&core, { 2, -3, 4 }).Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return trace->Moves().size() >= 6;
      }));
  auto moves = trace->Moves();
  for (const auto& entry : moves) {
    EXPECT_DOUBLE_EQ(entry.command_.forward_, 1);
    EXPECT_DOUBLE_EQ(entry.command_.lateral_, -1);
    EXPECT_DOUBLE_EQ(entry.command_.yaw_, 1);
  }
  std::cout << "S4 fixed Move intervals(ms):";
  for (size_t index = 1; index < moves.size(); ++index) {
    const auto gap = std::chrono::duration<double, std::milli>(
      moves[index].time_ - moves[index - 1].time_).count();
    std::cout << ' ' << gap;
    EXPECT_GT(gap, 5);
    EXPECT_LT(gap, 50);
  }
  std::cout << '\n';
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().watchdog_stops_ == 1;
      }));
  ASSERT_TRUE(WaitUntil([&] {
        return !trace->Moves().empty() && !Nonzero(trace->Moves().back());
      }));
  const auto count = trace->Moves().size();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(trace->Moves().size(), count);
  EXPECT_FALSE(core.Snapshot().command_active_);
  ASSERT_TRUE(Submit(&core, { 0.3, 0, 0 }).Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return Nonzero(trace->Moves().back());
      }));
}

TEST(CoreControl,
    InvalidInputLossAndSdkFailureClearCacheWithoutAutomaticReplay) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  DriverCore core(client);
  DriveReady(&core, trace);
  ASSERT_TRUE(Submit(&core, { 0.5, 0, 0 }).Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return !trace->Moves().empty();
      }));
  EXPECT_EQ(Submit(&core,
      { std::numeric_limits<double>::quiet_NaN(), 0, 0 }).Value(),
      ASTRALL_RES_INVALID_PARAM);
  ASSERT_TRUE(WaitUntil([&] {
        return !Nonzero(trace->Moves().back());
      }));
  EXPECT_FALSE(core.Snapshot().command_active_);
  ASSERT_TRUE(Submit(&core, { 0.4, 0, 0 }).Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return Nonzero(trace->Moves().back());
      }));
  AstrallSdkStatus status;
  status.link = 1;
  trace->base_->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().faulted_;
      }));
  ASSERT_TRUE(WaitUntil([&] {
        return !Nonzero(trace->Moves().back());
      }));
  const auto old = core.Snapshot();
  core.Deactivate();
  ASSERT_TRUE(core.Activate().Succeeded());
  EXPECT_EQ(core.SubmitVelocity({ 0.9, 0, 0 }, old.session_generation_,
      old.motion_epoch_)
      .Value(), ASTRALL_RES_NOT_INIT);
  EXPECT_FALSE(core.Snapshot().command_active_);
}

TEST(CoreControl, ModeAckRunningWaitsForPhysicalStateAndDampingPreempts) {
  auto trace = std::make_shared<test::ControlBackend>();
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDDOWN;
  trace->mode_result_ = ASTRALL_RES_RUNNING;
  auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().mode_actual_ == ASTRALL_SPORT_STATUS_FIXEDDOWN;
      }));
  ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  trace->base_->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().has_control_authority_;
      }));
  auto generation = core.Snapshot().session_generation_;
  EXPECT_EQ(core.RequestMode(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false,
      generation)
      .Value(), ASTRALL_RES_RUNNING);
  EXPECT_TRUE(core.Snapshot().mode_pending_);
  EXPECT_FALSE(core.Snapshot().faulted_);
  EXPECT_FALSE(core.RequestMode(ASTRALL_SPORT_MODE_CMD_MOVE, false,
      generation).Succeeded());
  EXPECT_EQ(core.RequestMode(ASTRALL_SPORT_MODE_CMD_DAMPING, false, generation)
      .Value(), ASTRALL_RES_RUNNING);
  trace->actual_ = ASTRALL_SPORT_STATUS_DAMPING;
  ASSERT_TRUE(WaitUntil([&] {
        return !core.Snapshot().mode_pending_;
      }));
  EXPECT_EQ(core.Snapshot().mode_result_, ASTRALL_RES_SUCCESSED);
  EXPECT_FALSE(core.Snapshot().command_active_);
}

TEST(CoreControl, LightTextUnicodeAndActualSendFrequencyAreBounded) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  DriverCore core(client);
  DriveReady(&core, trace);
  const auto generation = core.Snapshot().session_generation_;
  EXPECT_TRUE(core.RequestLight(ASTRALL_CMD_LIGHT_OPEN,
      generation).Succeeded());
  EXPECT_EQ(core.RequestLight(0, generation).Value(),
      ASTRALL_RES_INVALID_PARAM);
  EXPECT_TRUE(core.RequestText("测试消息", generation).Succeeded());
  EXPECT_FALSE(core.RequestText("too soon", generation).Succeeded());
  std::this_thread::sleep_for(260ms);
  EXPECT_TRUE(core.RequestText("第二条", generation).Succeeded());
  const auto texts = trace->Texts();
  ASSERT_EQ(texts.size(), 2U);
  EXPECT_EQ(texts[0].text_, "测试消息");
  EXPECT_GT(texts[1].time_ - texts[0].time_, 200ms);
  EXPECT_EQ(core.RequestText(std::string(65536, 'x'), generation).Value(),
      ASTRALL_RES_INVALID_PARAM);
  EXPECT_EQ(core.RequestText(std::string("a\0b", 3), generation).Value(),
      ASTRALL_RES_INVALID_PARAM);
}

/** @brief 在进入 S1 API 锁之前暂停已选中的非零，专门验证 core 发送门。 */
class SelectedMoveClient : public SdkClient {
 public:
  explicit SelectedMoveClient(SdkFunctions functions) : SdkClient(std::move(
          functions)) {
  }
  SdkResult Move(float x, float y, float yaw, uint32_t timeout) override {
    if (x != 0 && !paused_.exchange(true)) {
      entered_.set_value();
      gate_.wait();
    }
    return SdkClient::Move(x, y, yaw, timeout);
  }
  std::atomic<bool> paused_{ false };
  std::promise<void> entered_;
  std::shared_future<void> gate_;
};

TEST(CoreControl, SelectedOldNonzeroCannotWriteAfterModeOrLifecycleZero) {
  for (bool lifecycle : { false, true }) {
    auto trace = std::make_shared<test::ControlBackend>();
    std::promise<void> allow;
    auto client =
        std::make_shared<SelectedMoveClient>(test::ControlFunctions(trace));
    client->gate_ = allow.get_future().share();
    auto entered = client->entered_.get_future();
    DriverCore core(client);
    DriveReady(&core, trace);
    ASSERT_TRUE(Submit(&core, { 0.5, 0, 0 }).Succeeded());
    const auto selected = entered.wait_for(kWait);
    const auto generation = core.Snapshot().session_generation_;
    auto stopping = std::async(std::launch::async, [&] {
          return lifecycle ? core.Deactivate() :
          core.RequestMode(ASTRALL_SPORT_MODE_CMD_DAMPING, false, generation);
        });
    const bool cleared = WaitUntil([&] {
          return lifecycle ? core.Snapshot().phase_ == DriverPhase::kClosing :
          core.Snapshot().mode_pending_;
        });
    std::this_thread::sleep_for(30ms);
    const auto before = trace->Moves();
    allow.set_value();
    EXPECT_EQ(selected, std::future_status::ready);
    EXPECT_TRUE(cleared);
    EXPECT_TRUE(before.empty());
    ASSERT_EQ(stopping.wait_for(kWait), std::future_status::ready);
    stopping.get();
    ASSERT_TRUE(WaitUntil([&] {
          return trace->Moves().size() >= 2;
        }));
    std::this_thread::sleep_for(60ms);
    const auto moves = trace->Moves();
    ASSERT_TRUE(Nonzero(moves.front()));
    EXPECT_TRUE(std::none_of(moves.begin() + 1, moves.end(), Nonzero));
    EXPECT_FALSE(core.Snapshot().command_active_);
  }
}

TEST(CoreControl, FailedPreZeroDoesNotEnterNonDampingSdkMode) {
  auto trace = std::make_shared<test::ControlBackend>();
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDSTAND;
  auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  DriverCore core(client);
  ASSERT_TRUE(core.Configure({}).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().mode_actual_ == ASTRALL_SPORT_STATUS_FIXEDSTAND;
      }));
  ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  trace->base_->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().has_control_authority_;
      }));
  trace->move_result_ = ASTRALL_RES_FAILED;
  EXPECT_FALSE(core.RequestMode(ASTRALL_SPORT_MODE_CMD_MOVE, false,
      core.Snapshot().session_generation_).Succeeded());
  std::lock_guard<std::mutex> lock(trace->mutex_);
  EXPECT_TRUE(trace->modes_.empty());
}

TEST(CoreControl, DampingCancelsOldModeWhileItsPreZeroWaitsForSelectedMove) {
  auto trace = std::make_shared<test::ControlBackend>();
  std::promise<void> allow;
  auto client =
      std::make_shared<SelectedMoveClient>(test::ControlFunctions(trace));
  client->gate_ = allow.get_future().share();
  auto entered = client->entered_.get_future();
  DriverCore core(client);
  DriveReady(&core, trace);
  ASSERT_TRUE(Submit(&core, { 0.5, 0, 0 }).Succeeded());
  const auto selected = entered.wait_for(kWait);
  const auto generation = core.Snapshot().session_generation_;
  auto stand = std::async(std::launch::async, [&] {
        return core.RequestMode(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false,
        generation);
      });
  const bool standing = WaitUntil([&] {
        return core.Snapshot().mode_pending_;
      });
  std::this_thread::sleep_for(20ms);
  auto damping = std::async(std::launch::async, [&] {
        return core.RequestMode(ASTRALL_SPORT_MODE_CMD_DAMPING, false,
        generation);
      });
  std::this_thread::sleep_for(10ms);
  allow.set_value();
  EXPECT_EQ(selected, std::future_status::ready);
  EXPECT_TRUE(standing);
  ASSERT_EQ(stand.wait_for(kWait), std::future_status::ready);
  EXPECT_FALSE(stand.get().Succeeded());
  ASSERT_EQ(damping.wait_for(kWait), std::future_status::ready);
  damping.get();
  std::lock_guard<std::mutex> lock(trace->mutex_);
  ASSERT_EQ(trace->modes_.size(), 1U);
  EXPECT_EQ(trace->modes_[0], ASTRALL_SPORT_MODE_CMD_DAMPING);
}

class TextDispatchClient : public SdkClient {
 public:
  explicit TextDispatchClient(SdkFunctions functions) : SdkClient(std::move(
          functions)) {
  }
  SdkResult SendMessage(const std::string& text) override {
    if (!seen_.exchange(true)) {
      dispatched_.set_value(std::chrono::steady_clock::now());
      if (enter_base_.valid()) {
        enter_base_.wait();
      }
    }
    return SdkClient::SendMessage(text);
  }
  std::atomic<bool> seen_{ false };
  std::promise<std::chrono::steady_clock::time_point> dispatched_;
  std::shared_future<void> enter_base_;
};

TEST(CoreControl,
    NearFiveHzUsesSdkEntrySpacingDespiteFirstSendWaitingForSdkLock) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto functions = test::ControlFunctions(trace);
  std::atomic<bool> armed{ false };
  std::atomic<bool> blocked{ false };
  std::promise<void> lock_entered;
  auto entered = lock_entered.get_future();
  std::promise<void> allow_lock;
  auto gate = allow_lock.get_future().share();
  functions.heartbeat_ = [&](uint32_t) {
        if (armed && !blocked.exchange(true)) {
          lock_entered.set_value();
          gate.wait();
        }
        return ASTRALL_RES_SUCCESSED;
      };
  auto client = std::make_shared<TextDispatchClient>(functions);
  std::promise<void> enter_base;
  client->enter_base_ = enter_base.get_future().share();
  auto dispatched = client->dispatched_.get_future();
  DriverCore core(client);
  DriverParameters parameters;
  parameters.text_rate_hz_ = 4.999;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  const auto generation = core.Snapshot().session_generation_;
  auto sending = std::async(std::launch::async, [&] {
        return core.RequestText("first", generation);
      });
  const auto started = dispatched.wait_for(kWait);
  armed = true;
  auto holding = std::async(std::launch::async, [&] {
        return client->Heartbeat(5);
      });
  const auto locked = entered.wait_for(kWait);
  enter_base.set_value();
  std::this_thread::sleep_for(30ms);
  allow_lock.set_value();
  EXPECT_EQ(locked, std::future_status::ready);
  ASSERT_EQ(started, std::future_status::ready);
  holding.get();
  ASSERT_TRUE(sending.get().Succeeded());
  const auto dispatch_time = dispatched.get();
  std::this_thread::sleep_until(dispatch_time + 201ms);
  EXPECT_FALSE(core.RequestText("must reject", generation).Succeeded());
  auto texts = trace->Texts();
  ASSERT_EQ(texts.size(), 1U);
  std::this_thread::sleep_until(texts.front().time_ + 202ms);
  EXPECT_TRUE(core.RequestText("second", generation).Succeeded());
  texts = trace->Texts();
  ASSERT_EQ(texts.size(), 2U);
  const auto gap = texts[1].time_ - texts[0].time_;
  std::cout << "S4 near-5Hz actual send_message entry gap(ms): "
            << std::chrono::duration<double, std::milli>(gap).count() << '\n';
  EXPECT_GT(gap, 200ms);
}

TEST(CoreControl, ModeTimeoutAndLatePhysicalCompletionCannotRestoreCommands) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  DriverCore core(client);
  DriverParameters parameters;
  parameters.mode_switch_timeout_ms_ = 150;
  DriveReady(&core, trace, parameters);
  EXPECT_EQ(core.RequestMode(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false,
      core.Snapshot().session_generation_).Value(), ASTRALL_RES_RUNNING);
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().mode_result_ == ASTRALL_RES_TIMEOUT;
      }));
  EXPECT_TRUE(core.Snapshot().faulted_);
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDSTAND;
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(core.Snapshot().mode_result_, ASTRALL_RES_TIMEOUT);
  EXPECT_FALSE(core.Snapshot().command_active_);
  EXPECT_FALSE(Submit(&core, { 0.4, 0, 0 }).Succeeded());
}

TEST(CoreControl, QueuedModeExpiringDuringBudgetedPreZeroNeverEntersSdk) {
  auto trace = std::make_shared<test::ControlBackend>();
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDSTAND;
  auto functions = test::ControlFunctions(trace);
  auto move = functions.move_;
  double max_zero_ms = 0;
  functions.move_ = [&, move](float x, float y, float yaw, uint32_t timeout) {
        const auto start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(4ms);
        const auto elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        {
          std::lock_guard<std::mutex> lock(trace->mutex_);
          max_zero_ms = std::max(max_zero_ms, elapsed);
        }
        return move(x, y, yaw, timeout);
      };
  auto client = std::make_shared<SdkClient>(functions);
  DriverCore core(client);
  DriverParameters parameters;
  parameters.imu_frequency_hz_ = parameters.wheel_speed_frequency_hz_ =
      parameters.joystick_frequency_hz_ = 0;
  parameters.control_period_ms_ = 25;
  parameters.control_sdk_timeout_ms_ = 5;
  parameters.mode_poll_period_ms_ = 100;
  parameters.mode_switch_timeout_ms_ = 100;
  ASSERT_TRUE(core.Configure(parameters).Succeeded());
  ASSERT_TRUE(core.Activate().Succeeded());
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().mode_actual_ == ASTRALL_SPORT_STATUS_FIXEDSTAND;
      }));
  ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  trace->base_->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(WaitUntil([&] {
        return core.Snapshot().has_control_authority_;
      }));
  const auto generation = core.Snapshot().session_generation_;
  // 普通工作槽间隔 100ms，加预算内预零会跨过合法的最短模式 deadline。
  ASSERT_TRUE(core.RequestLight(ASTRALL_CMD_LIGHT_OPEN,
      generation).Succeeded());
  std::this_thread::sleep_for(1ms);
  auto pending = std::async(std::launch::async, [&] {
        return core.RequestMode(ASTRALL_SPORT_MODE_CMD_MOVE, false, generation);
      });
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (!core.Snapshot().mode_pending_ &&
      std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(100us);
  }
  ASSERT_TRUE(core.Snapshot().mode_pending_);
  const auto pending_seen = std::chrono::steady_clock::now();
  ASSERT_EQ(pending.wait_for(kWait), std::future_status::ready);
  const auto result = pending.get();
  const auto state = core.Snapshot();
  EXPECT_EQ(result.Value(), ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(state.mode_result_, ASTRALL_RES_TIMEOUT);
  EXPECT_EQ(state.last_result_, ASTRALL_RES_TIMEOUT);
  EXPECT_TRUE(state.faulted_);
  EXPECT_FALSE(state.mode_pending_);
  EXPECT_FALSE(state.command_active_);
  std::lock_guard<std::mutex> lock(trace->mutex_);
  EXPECT_TRUE(trace->modes_.empty());
  EXPECT_GT(max_zero_ms, 0);
  EXPECT_LT(max_zero_ms, parameters.control_sdk_timeout_ms_);
  std::cout << "S4 queued deadline: mode_calls=" << trace->modes_.size()
            << " response_after_pending_seen_ms="
            << std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - pending_seen).count()
            << " max_zero_ms=" << max_zero_ms
            << " result=" << result.ToString() << '\n';
}

TEST(CoreControl, MoveSdkFailureAndDisconnectCannotReplayAfterNewSession) {
  for (bool disconnect : { false, true }) {
    auto trace = std::make_shared<test::ControlBackend>();
    auto client = std::make_shared<SdkClient>(test::ControlFunctions(trace));
    DriverCore core(client);
    DriveReady(&core, trace);
    ASSERT_TRUE(Submit(&core, { 0.4, 0, 0 }).Succeeded());
    ASSERT_TRUE(WaitUntil([&] {
          return !trace->Moves().empty();
        }));
    if (disconnect) {
      AstrallSdkStatus status;
      trace->base_->config_.sdkStatusCb(&status, sizeof(status));
    } else {
      trace->move_result_ = ASTRALL_RES_FAILED;
    }
    ASSERT_TRUE(WaitUntil([&] {
          return core.Snapshot().faulted_;
        }));
    ASSERT_TRUE(WaitUntil([&] {
          return !Nonzero(trace->Moves().back());
        }));
    EXPECT_FALSE(core.Snapshot().command_active_);
    core.Deactivate();
    trace->move_result_ = ASTRALL_RES_SUCCESSED;
    ASSERT_TRUE(core.Activate().Succeeded());
    ASSERT_TRUE(core.RequestAuthority(ASTRALL_AUTH_SDK).Succeeded());
    AstrallSdkStatus status;
    status.link = 1;
    status.ctrlAuthority = 1;
    trace->base_->config_.sdkStatusCb(&status, sizeof(status));
    const auto count = trace->Moves().size();
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(trace->Moves().size(), count);
  }
}

TEST(CoreControl, ActualMoveAndHeartbeatTimingSurviveBoundedRpcLoad) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto functions = test::ControlFunctions(trace);
  const auto move = functions.move_;
  const auto heart = functions.heartbeat_;
  const auto light = functions.light_;
  const auto mode = functions.mode_;
  functions.move_ = [move](float x, float y, float yaw, uint32_t timeout) {
        auto result = move(x, y, yaw, timeout);
        std::this_thread::sleep_for(2ms);
        return result;
      };
  functions.heartbeat_ = [heart](uint32_t timeout) {
        auto result = heart(timeout);
        std::this_thread::sleep_for(5ms);
        return result;
      };
  functions.light_ = [light](AstrallLightCmd command, uint32_t timeout) {
        std::this_thread::sleep_for(5ms);
        return light(command, timeout);
      };
  functions.mode_ = [mode](AstrallSportModeCmd command, uint32_t timeout) {
        std::this_thread::sleep_for(5ms);
        return mode(command, timeout);
      };
  functions.get_device_ = [](AstrallDeviceInfo&, uint32_t) {
        std::this_thread::sleep_for(5ms);
        return ASTRALL_RES_SUCCESSED;
      };
  auto client = std::make_shared<SdkClient>(functions);
  DriverCore core(client);
  DriverParameters parameters;
  parameters.cmd_vel_timeout_ms_ = 2000;
  DriveReady(&core, trace, parameters);
  ASSERT_TRUE(Submit(&core, { 0.3, 0, 0 }).Succeeded());
  const auto generation = core.Snapshot().session_generation_;
  auto rpc = std::async(std::launch::async, [&] {
        for (int index = 0; index < 6; ++index) {
          EXPECT_TRUE(core.RequestLight(ASTRALL_CMD_LIGHT_OPEN,
          generation).Succeeded());
          std::this_thread::sleep_for(80ms);
        }
      });
  ASSERT_TRUE(WaitUntil([&] {
        std::lock_guard<std::mutex> lock(trace->mutex_);
        return trace->hearts_.size() >= 12;
      }));
  rpc.get();
  const auto moves = trace->Moves();
  ASSERT_GT(moves.size(), 20U);
  double max_move = 0;
  for (size_t index = 1; index < moves.size(); ++index) {
    const auto gap = std::chrono::duration<double, std::milli>(
      moves[index].time_ - moves[index - 1].time_).count();
    max_move = std::max(max_move, gap);
    EXPECT_LT(gap, 50);
  }
  EXPECT_EQ(core.RequestMode(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false,
      generation)
      .Value(), ASTRALL_RES_RUNNING);
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDSTAND;
  ASSERT_TRUE(WaitUntil([&] {
        return !core.Snapshot().mode_pending_;
      }));
  core.Shutdown();
  std::lock_guard<std::mutex> lock(trace->mutex_);
  for (size_t index = 1; index < trace->hearts_.size(); ++index) {
    const auto gap = std::chrono::duration<double, std::milli>(
      trace->hearts_[index] - trace->hearts_[index - 1]).count();
    EXPECT_GE(gap, 60);
    EXPECT_LE(gap, 180);
  }
  const auto mean = std::chrono::duration<double, std::milli>(
    trace->hearts_.back() - trace->hearts_.front()).count() /
      (trace->hearts_.size() - 1);
  std::cout << "S4 loaded Move max-gap(ms)=" << max_move
            << "; heartbeat mean(ms)=" << mean << '\n';
  EXPECT_NEAR(mean, 100.0, 10.0);
}

}  // namespace
}  // namespace astrall_driver
