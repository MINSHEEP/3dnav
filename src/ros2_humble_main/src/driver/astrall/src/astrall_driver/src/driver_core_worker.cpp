// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_core.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <optional>
#include <utility>

#include "driver_core_state.hpp"

namespace astrall_driver {
namespace {

constexpr auto kHeartbeatPeriod = std::chrono::milliseconds(kHeartbeatPeriodMs);
constexpr auto kWorkSpacing = std::chrono::milliseconds(kHeartbeatPeriodMs);

}  // namespace

void DriverCore::Fault(const std::shared_ptr<State>& state, uint64_t generation,
    const std::string& reason, uint16_t result) {
  std::lock_guard<std::mutex> lock(state->mutex_);
  if (state->snapshot_.session_generation_ != generation) {
    return;
  }
  if (state->stop_) {
    state->snapshot_.faulted_ = true;
    state->snapshot_.last_result_ = result;
    state->snapshot_.reason_ = reason;
    return;
  }
  state->FaultLocked(reason, result);
}

SdkResult DriverCore::ExecuteSdk(const std::shared_ptr<State>& state,
    uint64_t generation,
    const std::string& operation,
    const std::function<SdkResult()>& action) {
  try {
    return action();
  } catch (const std::exception& error) {
    Fault(state, generation, operation + " exception: " + error.what() + "; " +
        SdkResult(ASTRALL_RES_FAILED).ToString(), ASTRALL_RES_FAILED);
  } catch (...) {
    Fault(state, generation, operation + " unknown exception; " +
        SdkResult(ASTRALL_RES_FAILED).ToString(), ASTRALL_RES_FAILED);
  }
  return SdkResult(ASTRALL_RES_FAILED);
}

void DriverCore::ObserveStatus(const std::weak_ptr<State>& weak,
    uint64_t generation,
    const AstrallSdkStatus& status) {
  auto state = weak.lock();
  if (!state) {
    return;
  }
  std::lock_guard<std::mutex> lock(state->mutex_);
  if (!state->accept_callbacks_ ||
      generation != state->snapshot_.session_generation_) {
    return;
  }
  state->link_observed_ = true;
  state->sdk_authority_observed_ = status.ctrlAuthority;
  const bool previously_held = state->snapshot_.has_control_authority_;
  const bool previously_releasing = state->snapshot_.authority_release_pending_;
  state->snapshot_.connected_ = status.link;
  if (!status.ctrlAuthority) {
    state->snapshot_.authority_release_pending_ = false;
  }
  state->snapshot_.has_control_authority_ = status.link &&
      status.ctrlAuthority &&
      state->grant_allowed_ && state->snapshot_.active_ &&
      !state->snapshot_.faulted_;
  if (!state->initialized_) {
    return;
  }
  if (!status.link) {
    ++state->snapshot_.telemetry_epoch_;
    state->telemetry_.clear();
    state->FaultLocked("SDK disconnected; " +
        SdkResult(ASTRALL_RES_NOT_INIT).ToString(), ASTRALL_RES_NOT_INIT);
  } else if (previously_held && !state->snapshot_.has_control_authority_) {
    state->FaultLocked("SDK control authority lost; " +
        SdkResult(ASTRALL_RES_WITHOUT_AUTH).ToString(),
        ASTRALL_RES_WITHOUT_AUTH);
  } else if (status.ctrlAuthority && !state->grant_allowed_ &&
      !state->snapshot_.authority_release_pending_) {
    state->release_needed_ = true;
    state->FaultLocked("unsolicited SDK authority; " +
        SdkResult(ASTRALL_RES_WITHOUT_AUTH).ToString(),
        ASTRALL_RES_WITHOUT_AUTH);
  } else if (state->snapshot_.has_control_authority_) {
    state->snapshot_.reason_ = "SDK confirmed control authority";
  } else if (previously_releasing && !status.ctrlAuthority) {
    state->snapshot_.reason_ = "SDK confirmed control release";
  }
}

void DriverCore::HeartbeatLoop(const std::shared_ptr<State>& state,
    uint64_t generation) {
  auto next = std::chrono::steady_clock::now();
  while (true) {
    uint32_t timeout;
    {
      std::unique_lock<std::mutex> lock(state->mutex_);
      state->condition_.wait_until(lock, next, [&] {
          return state->stop_;
        });
      if (state->stop_ || generation != state->snapshot_.session_generation_) {
        return;
      }
      timeout = static_cast<uint32_t>(state->parameters_.heartbeat_timeout_ms_);
    }
    auto result = ExecuteSdk(state, generation, "HEARTBEAT", [&] {
        return state->client_->Heartbeat(timeout);
      });
    if (!result.Succeeded()) {
      Fault(state, generation, "HEARTBEAT " + result.ToString(),
          result.Value());
    }
    {
      std::lock_guard<std::mutex> lock(state->mutex_);
      if (generation == state->snapshot_.session_generation_) {
        ++state->snapshot_.heartbeat_count_;
      }
    }
    next += kHeartbeatPeriod;
    if (next < std::chrono::steady_clock::now()) {
      // SDK 若超期，避免补发突发流；实际时间间隔由测试直接记录验证。
      next = std::chrono::steady_clock::now() + kHeartbeatPeriod;
    }
  }
}

void DriverCore::SafetyStop(const std::shared_ptr<State>& state,
    uint64_t generation, bool release) {
  uint32_t timeout;
  uint64_t revision;
  std::unique_lock<std::mutex> io(state->motion_io_mutex_);
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    state->velocity_.Clear();
    revision = state->velocity_.Sample(std::chrono::steady_clock::now(), false)
        .revision_;
    timeout = static_cast<uint32_t>(state->parameters_.control_sdk_timeout_ms_);
  }
  auto zero = ExecuteSdk(state, generation, "ZERO_MOVE", [&] {
      return state->client_->Move(0.0F, 0.0F, 0.0F, timeout);
    });
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    if (generation == state->snapshot_.session_generation_) {
      state->velocity_.MarkSent(revision);
    }
    timeout = static_cast<uint32_t>(state->parameters_.sdk_timeout_ms_);
  }
  io.unlock();
  SdkResult released(ASTRALL_RES_NOT_INIT);
  if (release) {
    released = ExecuteSdk(state, generation, "RELEASE_AUTHORITY", [&] {
        return state->client_->AuthControl(ASTRALL_AUTH_JOYSTICK, timeout);
      });
  }
  if (!zero.Succeeded()) {
    Fault(state, generation, "ZERO_MOVE " + zero.ToString(), zero.Value());
  }
  if (release && !released.Succeeded()) {
    Fault(state, generation, "RELEASE_AUTHORITY " + released.ToString(),
        released.Value());
  }
  std::lock_guard<std::mutex> lock(state->mutex_);
  if (generation == state->snapshot_.session_generation_) {
    state->snapshot_.zero_attempted_ = true;
    state->snapshot_.zero_result_ = zero.Value();
    if (release) {
      state->snapshot_.release_attempted_ = true;
      state->snapshot_.release_result_ = released.Value();
      if (released.Succeeded()) {
        state->release_needed_ = false;
      }
    }
  }
}

void DriverCore::WorkLoop(const std::shared_ptr<State>& state,
    uint64_t generation) {
  auto next_poll = std::chrono::steady_clock::now();
  auto next_work = next_poll;
  auto next_query = next_poll;
  while (true) {
    bool safety = false;
    bool release = false;
    bool poll = false;
    bool query = false;
    std::optional<SubscriptionRequest> subscription;
    std::shared_ptr<AuthorityRequest> request;
    std::shared_ptr<ControlRequest> control;
    {
      std::unique_lock<std::mutex> lock(state->mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (state->stop_ || generation != state->snapshot_.session_generation_) {
        return;
      }
      const bool damping = state->control_request_ &&
          state->control_request_->kind_ == ControlRequest::Kind::kMode &&
          state->control_request_->command_ == ASTRALL_SPORT_MODE_CMD_DAMPING;
      if (damping) {
        control = state->control_request_;
      } else if (now >= next_work) {
        safety = state->safety_pending_;
        if (safety) {
          state->safety_pending_ = false;
          release = state->release_needed_;
        } else {
          request = state->request_;
          if (!request) {
            control = state->control_request_;
          }
          if (!request && !control && !state->subscription_requests_.empty()) {
            subscription = state->subscription_requests_.front();
            state->subscription_requests_.pop_front();
          }
        }
      }
      poll = now >= next_poll;
      query = now >= next_query;
      if (!safety && !request && !control && !subscription && !poll && !query) {
        auto deadline = std::min(next_poll, next_query);
        if (state->safety_pending_ || state->request_ ||
            state->control_request_ ||
            !state->subscription_requests_.empty()) {
          deadline = std::min(deadline, next_work);
        }
        state->condition_.wait_until(lock, deadline);
        continue;
      }
      if (poll) {
        next_poll = now + std::chrono::milliseconds(
          state->parameters_.system_poll_period_ms_);
      }
      if (query) {
        next_query = now + std::chrono::milliseconds(
          state->parameters_.telemetry_query_period_ms_);
      }
    }
    if (safety) {
      SafetyStop(state, generation, release);
    } else if (request) {
      ProcessAuthority(state, request);
    } else if (control) {
      ProcessControl(state, control);
    } else if (subscription) {
      ProcessSubscription(state, generation, *subscription);
    }
    if (safety || request || control || subscription) {
      next_work = std::chrono::steady_clock::now() + kWorkSpacing;
    }
    if (poll) {
      PollSystem(state, generation);
    }
    if (query) {
      PollTelemetry(state, generation);
    }
  }
}

void DriverCore::PollSystem(const std::shared_ptr<State>& state,
    uint64_t generation) {
  AstrallSystemStatus system;
  auto result = ExecuteSdk(state, generation, "SYSTEM_STATUS", [&] {
      return state->client_->GetSystemStatus(&system);
    });
  StoreTelemetry(state, generation, system, result.Succeeded());
  if (!result.Succeeded() || system.errorCode != ASTRALL_ERR_NONE ||
      system.sysStatus == ASTRALL_SYSTEM_CODE_ERROR) {
    const auto code = result.Succeeded() ? ASTRALL_RES_FAILED : result.Value();
    Fault(state, generation, "SYSTEM_STATUS " + SdkResult(code).ToString() +
        "; error_bits=" + std::to_string(system.errorCode) +
        "; system_status=" + std::to_string(system.sysStatus), code);
  }
  CallbackStatistics statistics;
  auto callbacks = ExecuteSdk(state, generation, "SDK_CALLBACKS", [&] {
      statistics = state->client_->GetCallbackStatistics();
      return SdkResult(statistics.callback_errors_ ||
      statistics.invalid_payloads_ ||
      statistics.dropped_events_ ? ASTRALL_RES_FAILED : ASTRALL_RES_SUCCESSED);
    });
  if (!callbacks.Succeeded()) {
    Fault(state, generation, "SDK_CALLBACKS " + callbacks.ToString(),
        callbacks.Value());
  }
  const auto diagnostic_count = statistics.callback_errors_ +
      statistics.invalid_payloads_ + statistics.dropped_events_;
  std::lock_guard<std::mutex> lock(state->mutex_);
  if (generation == state->snapshot_.session_generation_ &&
      diagnostic_count != state->callback_diagnostics_seen_) {
    state->snapshot_.telemetry_dropped_ += state->telemetry_.size();
    state->telemetry_.clear();
    state->callback_diagnostics_seen_ = diagnostic_count;
  }
}

}  // namespace astrall_driver
