// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_core.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <utility>

#include "driver_core_state.hpp"

namespace astrall_driver {
namespace {

constexpr auto kControlResponseWait = std::chrono::milliseconds(250);

bool Zero(VelocityCommand command) {
  return command.forward_ == 0 && command.lateral_ == 0 && command.yaw_ == 0;
}

}  // namespace

SdkResult DriverCore::SubmitVelocity(VelocityCommand command,
    uint64_t generation, uint64_t motion_epoch) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (generation != state_->snapshot_.session_generation_ ||
      motion_epoch != state_->snapshot_.motion_epoch_ || state_->stop_) {
    return SdkResult(ASTRALL_RES_NOT_INIT);
  }
  const bool allowed = state_->initialized_ && state_->snapshot_.active_ &&
      state_->snapshot_.connected_ &&
      state_->snapshot_.has_control_authority_ &&
      !state_->snapshot_.faulted_ && state_->mode_.CanMove(now);
  const bool finite = std::isfinite(command.forward_) &&
      std::isfinite(command.lateral_) && std::isfinite(command.yaw_);
  if (!state_->velocity_.Accept(command, now, allowed)) {
    ++state_->snapshot_.motion_epoch_;
    const auto code =
        finite ? ASTRALL_RES_WITHOUT_AUTH : ASTRALL_RES_INVALID_PARAM;
    state_->snapshot_.control_reason_ = finite ?
        "cmd_vel gate closed; old command cleared" :
        "nonfinite cmd_vel rejected; zero required";
    state_->snapshot_.control_result_ = code;
    state_->condition_.notify_all();
    return SdkResult(code);
  }
  state_->snapshot_.control_reason_ =
      "fresh cmd_vel accepted with per-axis limits";
  state_->snapshot_.control_result_ = ASTRALL_RES_SUCCESSED;
  state_->condition_.notify_all();
  return SdkResult(ASTRALL_RES_SUCCESSED);
}

void DriverCore::FinishControl(const std::shared_ptr<ControlRequest>& request,
    uint16_t result) {
  if (request && !request->completed_.exchange(true)) {
    request->promise_.set_value(SdkResult(result));
  }
}

SdkResult DriverCore::WaitControl(
  const std::shared_ptr<ControlRequest>& request) {
  auto future = request->promise_.get_future();
  if (future.wait_for(kControlResponseWait) == std::future_status::ready) {
    return future.get();
  }
  request->cancelled_ = true;
  Fault(state_, request->generation_, "CONTROL_RPC " +
      SdkResult(ASTRALL_RES_TIMEOUT).ToString(), ASTRALL_RES_TIMEOUT);
  return SdkResult(ASTRALL_RES_TIMEOUT);
}

SdkResult DriverCore::RequestMode(uint16_t command, bool confirmation,
    uint64_t generation) {
  auto request = std::make_shared<ControlRequest>();
  request->kind_ = ControlRequest::Kind::kMode;
  request->command_ = command;
  std::shared_ptr<ControlRequest> previous;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    const bool damping = command == ASTRALL_SPORT_MODE_CMD_DAMPING;
    if (state_->stop_ || !state_->initialized_ ||
        !state_->snapshot_.connected_ ||
        generation != state_->snapshot_.session_generation_) {
      return SdkResult(ASTRALL_RES_NOT_INIT);
    }
    if (!damping && (!state_->snapshot_.active_ || state_->snapshot_.faulted_ ||
        !state_->snapshot_.has_control_authority_)) {
      return SdkResult(ASTRALL_RES_WITHOUT_AUTH);
    }
    if (state_->control_request_ && !damping) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    const auto now = std::chrono::steady_clock::now();
    if (state_->mode_.Tick(now)) {
      state_->FaultLocked("MODE " + SdkResult(ASTRALL_RES_TIMEOUT).ToString(),
          ASTRALL_RES_TIMEOUT);
      if (!damping) {
        return SdkResult(ASTRALL_RES_TIMEOUT);
      }
    }
    state_->ClearMotionLocked();
    auto decision = state_->mode_.Begin(command, confirmation, now);
    state_->snapshot_.control_reason_ = decision.reason_;
    if (!decision.accepted_) {
      return SdkResult(decision.result_);
    }
    previous = std::move(state_->control_request_);
    if (previous) {
      previous->cancelled_ = true;
    }
    request->generation_ = generation;
    request->epoch_ = ++state_->control_epoch_;
    request->mode_ticket_ = decision.ticket_;
    state_->control_request_ = request;
    state_->condition_.notify_all();
  }
  FinishControl(previous, ASTRALL_RES_FAILED);
  return WaitControl(request);
}

SdkResult DriverCore::RequestLight(uint16_t command, uint64_t generation) {
  if (command != ASTRALL_CMD_LIGHT_OPEN && command != ASTRALL_CMD_LIGHT_CLOSE) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  auto request = std::make_shared<ControlRequest>();
  request->kind_ = ControlRequest::Kind::kLight;
  request->command_ = command;
  request->generation_ = generation;
  return QueueAuxiliary(request);
}

SdkResult DriverCore::RequestText(const std::string& text,
    uint64_t generation) {
  if (text.empty() || text.size() > std::numeric_limits<uint16_t>::max() ||
      text.find('\0') != std::string::npos) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  auto request = std::make_shared<ControlRequest>();
  request->kind_ = ControlRequest::Kind::kText;
  request->text_ = text;
  request->generation_ = generation;
  return QueueAuxiliary(request);
}

SdkResult DriverCore::QueueAuxiliary(
  const std::shared_ptr<ControlRequest>& request) {
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    if (state_->stop_ || !state_->initialized_ || !state_->snapshot_.active_ ||
        !state_->snapshot_.connected_ || state_->snapshot_.faulted_ ||
        request->generation_ != state_->snapshot_.session_generation_) {
      return SdkResult(ASTRALL_RES_NOT_INIT);
    }
    if (state_->control_request_) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    if (request->kind_ == ControlRequest::Kind::kText &&
        !state_->text_limiter_.Ready(std::chrono::steady_clock::now())) {
      state_->snapshot_.control_reason_ =
          "text rate limited locally; no SDK call";
      return SdkResult(ASTRALL_RES_FAILED);
    }
    request->epoch_ = ++state_->control_epoch_;
    state_->control_request_ = request;
    state_->condition_.notify_all();
  }
  return WaitControl(request);
}

void DriverCore::ProcessControl(const std::shared_ptr<State>& state,
    const std::shared_ptr<ControlRequest>& request) {
  const bool mode = request->kind_ == ControlRequest::Kind::kMode;
  uint32_t timeout;
  bool rejected = false;
  uint16_t rejection = ASTRALL_RES_FAILED;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    rejected = state->stop_ || request->cancelled_ ||
        request->generation_ != state->snapshot_.session_generation_ ||
        request->epoch_ != state->control_epoch_;
    if (mode) {
      rejection = state->CheckModeDispatchLocked(request);
      rejected = rejection != ASTRALL_RES_SUCCESSED;
    }
    if (!rejected && request->kind_ == ControlRequest::Kind::kText &&
        !state->text_limiter_.TrySend(std::chrono::steady_clock::now())) {
      rejected = true;
      state->snapshot_.control_reason_ =
          "text rate limited at dispatch; no SDK call";
    }
    if (rejected && state->control_request_ == request) {
      state->control_request_.reset();
    }
    timeout = static_cast<uint32_t>(state->parameters_.sdk_timeout_ms_);
  }
  if (rejected) {
    FinishControl(request, rejection);
    return;
  }
  if (mode) {
    SafetyStop(state, request->generation_, false);
    // 等待发送门/归零会消耗剩余时间；重查 deadline 和取消，过期不产生模式 I/O。
    {
      std::lock_guard<std::mutex> lock(state->mutex_);
      rejection = state->CheckModeDispatchLocked(request);
      if (rejection != ASTRALL_RES_SUCCESSED &&
          state->control_request_ == request) {
        state->control_request_.reset();
      }
    }
    if (rejection != ASTRALL_RES_SUCCESSED) {
      FinishControl(request, rejection);
      return;
    }
  }
  uint16_t admission = ASTRALL_RES_SUCCESSED;
  auto result = ExecuteSdk(state, request->generation_, "CONTROL_RPC", [&] {
      if (mode) {
        return state->client_->SportModeControl(
          static_cast<AstrallSportModeCmd>(request->command_), timeout,
          [state, request, &admission] {
          // 此处已取得 SDK API 锁；只读写本地状态，绝不再次进入 SDK。
          std::lock_guard<std::mutex> lock(state->mutex_);
          admission = state->CheckModeDispatchLocked(request);
          return SdkResult(admission);
        });
      }
      if (request->kind_ == ControlRequest::Kind::kLight) {
        return state->client_->LightControl(
          static_cast<AstrallLightCmd>(request->command_), timeout);
      }
      return state->client_->SendMessage(request->text_);
    });
  if (admission != ASTRALL_RES_SUCCESSED) {
    {
      std::lock_guard<std::mutex> lock(state->mutex_);
      if (state->control_request_ == request) {
        state->control_request_.reset();
      }
    }
    // 拒绝没有产生 ACK；保留 TIMEOUT，也不覆盖抢占者的新 ticket/诊断。
    FinishControl(request, admission);
    return;
  }
  uint16_t response = result.Value();
  bool current;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    if (request->kind_ == ControlRequest::Kind::kText) {
      // 完成时刻晚于实际 SDK 入口，作为下次间隔的保守基准，不会因锁等待缩短。
      state->text_limiter_.RecordCompleted(std::chrono::steady_clock::now());
    }
    current = !state->stop_ && !request->cancelled_ &&
        request->generation_ == state->snapshot_.session_generation_ &&
        request->epoch_ == state->control_epoch_;
    if (state->control_request_ == request) {
      state->control_request_.reset();
    }
    if (current && mode) {
      state->snapshot_.mode_ack_ = result.Value();
      state->mode_.Acknowledge(request->mode_ticket_, result.Value(),
          std::chrono::steady_clock::now());
      response = state->mode_.Result();
      state->snapshot_.control_reason_ = state->mode_.Reason();
    } else if (current) {
      state->snapshot_.control_reason_ = "SDK request completed: " +
          result.ToString();
    }
    state->snapshot_.control_result_ =
        current ? response : ASTRALL_RES_NOT_INIT;
  }
  const bool in_progress = mode && result.Value() == ASTRALL_RES_RUNNING;
  if (current && ((!result.Succeeded() && !in_progress) ||
      (mode && response == ASTRALL_RES_TIMEOUT))) {
    Fault(state,
        request->generation_,
        "CONTROL_RPC " + SdkResult(response).ToString(),
        response);
  }
  FinishControl(request, current ? response : ASTRALL_RES_NOT_INIT);
}

void DriverCore::PollControlMode(const std::shared_ptr<State>& state,
    uint64_t generation) {
  auto actual = ASTRALL_SPORT_STATUS_UNKNOWN;
  auto result = ExecuteSdk(state, generation, "MODE_STATUS", [&] {
      actual = state->client_->GetSportStatus();
      return SdkResult(ASTRALL_RES_SUCCESSED);
    });
  if (!result.Succeeded()) {
    Fault(state, generation, "MODE_STATUS " + result.ToString(),
        result.Value());
    return;
  }
  std::lock_guard<std::mutex> lock(state->mutex_);
  if (state->stop_ || generation != state->snapshot_.session_generation_) {
    return;
  }
  const bool pending = state->mode_.Pending();
  state->mode_.Observe(actual, std::chrono::steady_clock::now());
  if (pending && !state->mode_.Pending()) {
    state->ClearMotionLocked();
    if (state->mode_.Result() == ASTRALL_RES_TIMEOUT) {
      state->FaultLocked("MODE " + SdkResult(ASTRALL_RES_TIMEOUT).ToString(),
          ASTRALL_RES_TIMEOUT);
    }
  }
}

void DriverCore::ControlLoop(const std::shared_ptr<State>& state,
    uint64_t generation) {
  auto next = std::chrono::steady_clock::now();
  auto next_mode = next;
  while (true) {
    int64_t period;
    {
      std::unique_lock<std::mutex> lock(state->mutex_);
      state->condition_.wait_until(lock, next, [&] {
          return state->stop_;
        });
      if (state->stop_ || generation != state->snapshot_.session_generation_) {
        return;
      }
      period = state->parameters_.control_period_ms_;
    }
    if (std::chrono::steady_clock::now() >= next_mode) {
      PollControlMode(state, generation);
      std::lock_guard<std::mutex> lock(state->mutex_);
      next_mode = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(state->parameters_.mode_poll_period_ms_);
    }
    {
      // 发送门覆盖最后一次状态取样和实际 I/O，安全零速使用相同门。
      std::lock_guard<std::mutex> io(state->motion_io_mutex_);
      VelocitySample sample;
      uint32_t timeout;
      bool send = false;
      {
        std::lock_guard<std::mutex> lock(state->mutex_);
        if (state->stop_ ||
            generation != state->snapshot_.session_generation_) {
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (state->mode_.Tick(now)) {
          state->FaultLocked("MODE " + SdkResult(
                ASTRALL_RES_TIMEOUT).ToString(),
              ASTRALL_RES_TIMEOUT);
        }
        const bool allowed = state->snapshot_.active_ &&
            state->snapshot_.connected_ &&
            state->snapshot_.has_control_authority_ &&
            !state->snapshot_.faulted_ &&
            state->mode_.CanMove(now);
        const bool previously_active = state->velocity_.HasCommand();
        sample = state->velocity_.Sample(now, allowed);
        if (sample.expired_ || (previously_active && !allowed)) {
          ++state->snapshot_.motion_epoch_;
          state->snapshot_.control_reason_ = sample.expired_ ?
              "steady cmd_vel watchdog expired; fresh command required" :
              "velocity gate closed; fresh command required";
          if (sample.expired_) {
            ++state->snapshot_.watchdog_stops_;
          }
        }
        send = sample.send_ && state->snapshot_.has_control_authority_;
        timeout =
            static_cast<uint32_t>(state->parameters_.control_sdk_timeout_ms_);
        if (sample.send_ && !send) {
          state->velocity_.MarkSent(sample.revision_);
        }
      }
      if (send) {
        auto result = ExecuteSdk(state, generation, "CONTROL_MOVE", [&] {
            return state->client_->Move(static_cast<float>(sample.command_.
            forward_),
            static_cast<float>(sample.command_.lateral_),
            static_cast<float>(sample.command_.yaw_), timeout);
          });
        if (!result.Succeeded()) {
          Fault(state,
              generation,
              "CONTROL_MOVE " + result.ToString(),
              result.Value());
        }
        std::lock_guard<std::mutex> lock(state->mutex_);
        if (generation == state->snapshot_.session_generation_) {
          ++state->snapshot_.control_send_count_;
          state->snapshot_.control_result_ = result.Value();
          state->velocity_.MarkSent(sample.revision_);
          if (Zero(sample.command_)) {
            state->snapshot_.zero_attempted_ = true;
            state->snapshot_.zero_result_ = result.Value();
          }
        }
      }
    }
    next += std::chrono::milliseconds(period);
    if (next < std::chrono::steady_clock::now()) {
      next = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(period);
    }
  }
}

}  // namespace astrall_driver
