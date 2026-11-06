// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_core.hpp"

#include <chrono>
#include <future>
#include <utility>

#include "driver_core_state.hpp"

namespace astrall_driver {
namespace {

constexpr auto kAuthorityWait = std::chrono::milliseconds(250);

}  // namespace

void DriverCore::FinishRequest(const std::shared_ptr<AuthorityRequest>& request,
    uint16_t result) {
  if (request && !request->completed_.exchange(true)) {
    request->promise_.set_value(SdkResult(result));
  }
}

SdkResult DriverCore::RequestAuthority(AstrallAuth authority) {
  if (authority != ASTRALL_AUTH_SDK && authority != ASTRALL_AUTH_JOYSTICK) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  auto request = std::make_shared<AuthorityRequest>();
  auto future = request->promise_.get_future();
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    if (!state_->initialized_ || !state_->snapshot_.active_ ||
        !state_->snapshot_.connected_ || state_->snapshot_.faulted_) {
      return SdkResult(ASTRALL_RES_NOT_INIT);
    }
    if (state_->request_) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    if (authority == ASTRALL_AUTH_SDK &&
        state_->snapshot_.authority_release_pending_) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    request->authority_ = authority;
    request->generation_ = state_->snapshot_.session_generation_;
    request->epoch_ = ++state_->authority_epoch_;
    state_->request_ = request;
    state_->grant_allowed_ = authority == ASTRALL_AUTH_SDK;
    state_->release_needed_ = state_->release_needed_ ||
        authority == ASTRALL_AUTH_SDK ||
        state_->snapshot_.has_control_authority_;
    if (authority == ASTRALL_AUTH_JOYSTICK) {
      state_->ClearMotionLocked();
      state_->snapshot_.has_control_authority_ = false;
      state_->snapshot_.authority_release_pending_ = true;
    }
    state_->condition_.notify_all();
  }
  if (future.wait_for(kAuthorityWait) == std::future_status::ready) {
    return future.get();
  }
  request->cancelled_ = true;
  Fault(state_, request->generation_, "AUTH_CONTROL " +
      SdkResult(ASTRALL_RES_TIMEOUT).ToString(), ASTRALL_RES_TIMEOUT);
  return SdkResult(ASTRALL_RES_TIMEOUT);
}

void DriverCore::ProcessAuthority(const std::shared_ptr<State>& state,
    const std::shared_ptr<AuthorityRequest>& request) {
  uint32_t timeout;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    if (request->cancelled_ || state->stop_ ||
        request->generation_ != state->snapshot_.session_generation_ ||
        request->epoch_ != state->authority_epoch_) {
      if (state->request_ == request) {
        state->request_.reset();
      }
      FinishRequest(request, ASTRALL_RES_NOT_INIT);
      return;
    }
    timeout = static_cast<uint32_t>(state->parameters_.sdk_timeout_ms_);
  }
  if (request->authority_ == ASTRALL_AUTH_JOYSTICK) {
    SafetyStop(state, request->generation_, false);
  }
  auto result = ExecuteSdk(state, request->generation_, "AUTH_CONTROL", [&] {
      return state->client_->AuthControl(request->authority_, timeout);
    });
  bool current;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    current = !request->cancelled_ && !state->stop_ &&
        request->generation_ == state->snapshot_.session_generation_ &&
        request->epoch_ == state->authority_epoch_;
    if (state->request_ == request) {
      state->request_.reset();
    }
    if (current && result.Succeeded()) {
      state->snapshot_.last_result_ = result.Value();
      state->snapshot_.reason_ = state->snapshot_.has_control_authority_ ?
          "SDK confirmed control authority" :
          "control request ACK; awaiting SDK state confirmation";
      if (request->authority_ == ASTRALL_AUTH_JOYSTICK) {
        state->release_needed_ = false;
        state->snapshot_.reason_ = state->snapshot_.authority_release_pending_ ?
            "release ACK; awaiting SDK state confirmation" :
            "SDK confirmed control release";
      }
    }
  }
  if (current && !result.Succeeded()) {
    Fault(state, request->generation_, "AUTH_CONTROL " + result.ToString(),
        result.Value());
  }
  FinishRequest(request, current ? result.Value() : ASTRALL_RES_NOT_INIT);
}

}  // namespace astrall_driver
