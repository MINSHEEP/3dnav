// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_core.hpp"

#include <stdexcept>
#include <chrono>
#include <utility>

#include "driver_core_state.hpp"

namespace astrall_driver {
namespace {

std::function<int64_t()> ReceiveClock(std::function<int64_t()> clock) {
  if (clock) {
    return clock;
  }
  return [] {
           return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
  };
}

}  // namespace

const char* DriverPhaseName(DriverPhase phase) {
  switch (phase) {
    case DriverPhase::kUnconfigured: return "unconfigured";
    case DriverPhase::kConnecting: return "connecting";
    case DriverPhase::kInactive: return "inactive";
    case DriverPhase::kActive: return "active";
    case DriverPhase::kFault: return "fault";
    case DriverPhase::kClosing: return "closing";
    case DriverPhase::kFinalized: return "finalized";
  }
  return "unknown";
}

DriverCore::DriverCore(std::shared_ptr<ISdkClient> client,
    std::function<int64_t()> receive_clock)
  : state_(std::make_shared<State>(std::move(client),
      ReceiveClock(std::move(receive_clock)))) {
  if (!state_->client_) {
    throw std::invalid_argument("DriverCore requires an SDK client");
  }
}

DriverCore::~DriverCore() {
  Shutdown();
}

DriverSnapshot DriverCore::Snapshot() const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  auto snapshot = state_->snapshot_;
  snapshot.command_active_ = state_->velocity_.HasCommand();
  snapshot.mode_pending_ = state_->mode_.Pending();
  snapshot.mode_can_move_ = state_->mode_.CanMove(
    std::chrono::steady_clock::now());
  snapshot.mode_actual_ = state_->mode_.Actual();
  snapshot.mode_result_ = state_->mode_.Result();
  snapshot.mode_ticket_ = state_->mode_.Ticket();
  snapshot.mode_reason_ = state_->mode_.Reason();
  return snapshot;
}

SdkResult DriverCore::Configure(const DriverParameters& parameters) {
  const auto error = parameters.Validate();
  std::lock_guard<std::mutex> transition(transition_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    if (state_->snapshot_.phase_ == DriverPhase::kFinalized) {
      return SdkResult(ASTRALL_RES_NOT_INIT);
    }
    if (state_->snapshot_.configured_) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    if (!error.empty()) {
      state_->snapshot_.reason_ = error;
      state_->snapshot_.last_result_ = ASTRALL_RES_INVALID_PARAM;
      return SdkResult(ASTRALL_RES_INVALID_PARAM);
    }
    state_->parameters_ = parameters;
  }
  return Connect();
}

SdkResult DriverCore::Connect() {
  uint64_t generation;
  uint32_t timeout;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    generation = state_->snapshot_.session_generation_ + 1;
    state_->snapshot_ = {};
    state_->snapshot_.session_generation_ = generation;
    state_->snapshot_.phase_ = DriverPhase::kConnecting;
    state_->snapshot_.reason_ = "connecting";
    state_->stop_ = false;
    state_->session_attempted_ = true;
    state_->accept_callbacks_ = true;
    state_->link_observed_ = false;
    state_->sdk_authority_observed_ = false;
    state_->grant_allowed_ = false;
    state_->release_needed_ = false;
    state_->safety_pending_ = false;
    state_->PrepareTelemetryLocked();
    state_->PrepareControlLocked();
    timeout = static_cast<uint32_t>(state_->parameters_.init_timeout_ms_);
  }
  SdkCallbacks callbacks;
  callbacks.status_ = [weak = std::weak_ptr<State>(state_), generation](
    const AstrallSdkStatus& status) {
        ObserveStatus(weak, generation, status);
      };
  auto result = ExecuteSdk(state_, generation, "SDK_INIT", [&] {
      return state_->client_->Init(callbacks, timeout);
    });
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    if (result.Succeeded()) {
      state_->initialized_ = true;
      if (!state_->link_observed_) {
        // 手册 5.2.2：Init 成功表示已建立连接，控制权仍需独立确认。
        state_->snapshot_.connected_ = true;
      }
      if (state_->sdk_authority_observed_) {
        state_->release_needed_ = true;
        state_->FaultLocked("unsolicited SDK authority at init; " +
            SdkResult(
              ASTRALL_RES_WITHOUT_AUTH).ToString(),
            ASTRALL_RES_WITHOUT_AUTH);
      } else if (!state_->snapshot_.faulted_) {
        state_->snapshot_.configured_ = true;
        state_->snapshot_.phase_ = DriverPhase::kInactive;
        state_->snapshot_.reason_ = "connected; no control authority requested";
        state_->snapshot_.last_result_ = result.Value();
      }
    }
  }
  const auto connected = Snapshot();
  if (!result.Succeeded() || connected.faulted_ || !connected.connected_) {
    const auto code = !result.Succeeded() ? result.Value() :
        (connected.faulted_ ? connected.last_result_ : ASTRALL_RES_NOT_INIT);
    CloseSession("SDK initialization failed: " + SdkResult(code).ToString() +
        "; " + connected.reason_);
    std::lock_guard<std::mutex> lock(state_->mutex_);
    state_->snapshot_.configured_ = false;
    state_->snapshot_.faulted_ = true;
    state_->snapshot_.phase_ = DriverPhase::kUnconfigured;
    state_->snapshot_.last_result_ = code;
    return SdkResult(state_->snapshot_.last_result_);
  }
  try {
    heartbeat_thread_ = std::thread(HeartbeatLoop, state_, generation);
    worker_thread_ = std::thread(WorkLoop, state_, generation);
    control_thread_ = std::thread(ControlLoop, state_, generation);
  } catch (const std::exception& error) {
    CloseSession(std::string("worker creation failed: ") + error.what());
    std::lock_guard<std::mutex> lock(state_->mutex_);
    state_->snapshot_.configured_ = false;
    state_->snapshot_.phase_ = DriverPhase::kUnconfigured;
    state_->snapshot_.faulted_ = true;
    state_->snapshot_.last_result_ = ASTRALL_RES_FAILED;
    return SdkResult(ASTRALL_RES_FAILED);
  }
  return result;
}

SdkResult DriverCore::Activate() {
  std::lock_guard<std::mutex> transition(transition_mutex_);
  bool reconnect;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    if (!state_->snapshot_.configured_) {
      return SdkResult(ASTRALL_RES_NOT_INIT);
    }
    reconnect = !state_->initialized_;
  }
  if (reconnect) {
    auto result = Connect();
    if (!result.Succeeded()) {
      return result;
    }
  }
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (state_->snapshot_.faulted_ || !state_->snapshot_.connected_) {
    return SdkResult(ASTRALL_RES_FAILED);
  }
  state_->snapshot_.active_ = true;
  if (!state_->telemetry_active_) {
    state_->telemetry_.clear();
    state_->telemetry_active_ = true;
  }
  state_->snapshot_.phase_ = DriverPhase::kActive;
  state_->snapshot_.reason_ =
      "active; control authority requires explicit request";
  return SdkResult(ASTRALL_RES_SUCCESSED);
}

void DriverCore::CloseSession(const std::string& reason) {
  bool initialized;
  bool attempted;
  bool release;
  uint64_t generation;
  std::shared_ptr<AuthorityRequest> request;
  std::shared_ptr<ControlRequest> control;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    initialized = state_->initialized_;
    attempted = state_->session_attempted_;
    state_->session_attempted_ = false;
    release = state_->release_needed_;
    state_->initialized_ = false;
    state_->stop_ = true;
    state_->accept_callbacks_ = false;
    state_->telemetry_active_ = false;
    state_->telemetry_.clear();
    state_->subscription_requests_.clear();
    state_->grant_allowed_ = false;
    state_->release_needed_ = false;
    state_->safety_pending_ = false;
    state_->ClearMotionLocked();
    state_->mode_.Cancel(ASTRALL_RES_NOT_INIT, reason);
    ++state_->control_epoch_;
    control = std::move(state_->control_request_);
    if (control) {
      control->cancelled_ = true;
    }
    state_->snapshot_.active_ = false;
    state_->snapshot_.connected_ = false;
    state_->snapshot_.has_control_authority_ = false;
    state_->snapshot_.authority_release_pending_ = false;
    generation = ++state_->snapshot_.session_generation_;
    ++state_->authority_epoch_;
    state_->snapshot_.phase_ = DriverPhase::kClosing;
    state_->snapshot_.reason_ = reason;
    request = std::move(state_->request_);
    if (request) {
      request->cancelled_ = true;
    }
    state_->condition_.notify_all();
  }
  FinishRequest(request, ASTRALL_RES_NOT_INIT);
  FinishControl(control, ASTRALL_RES_NOT_INIT);
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  if (initialized) {
    SafetyStop(state_, generation, release);
  }
  CancelSubscriptions(generation);
  if (attempted) {
    ExecuteSdk(state_, generation, "SDK_DEINIT", [&] {
        state_->client_->Deinit();
        return SdkResult(ASTRALL_RES_SUCCESSED);
      });
  }
}

SdkResult DriverCore::Deactivate() {
  std::lock_guard<std::mutex> transition(transition_mutex_);
  if (Snapshot().phase_ == DriverPhase::kFinalized) {
    return SdkResult(ASTRALL_RES_SUCCESSED);
  }
  CloseSession("deactivated; zero command is best effort");
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->snapshot_.phase_ = state_->snapshot_.configured_ ?
      DriverPhase::kInactive : DriverPhase::kUnconfigured;
  return SdkResult(ASTRALL_RES_SUCCESSED);
}

SdkResult DriverCore::Cleanup() {
  std::lock_guard<std::mutex> transition(transition_mutex_);
  if (Snapshot().phase_ == DriverPhase::kFinalized) {
    return SdkResult(ASTRALL_RES_SUCCESSED);
  }
  CloseSession("cleaned up");
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->snapshot_.configured_ = false;
  state_->snapshot_.faulted_ = false;
  state_->snapshot_.phase_ = DriverPhase::kUnconfigured;
  return SdkResult(ASTRALL_RES_SUCCESSED);
}

SdkResult DriverCore::Shutdown() {
  std::lock_guard<std::mutex> transition(transition_mutex_);
  if (Snapshot().phase_ == DriverPhase::kFinalized) {
    return SdkResult(ASTRALL_RES_SUCCESSED);
  }
  CloseSession("shutdown; zero command is best effort");
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->snapshot_.configured_ = false;
  state_->snapshot_.phase_ = DriverPhase::kFinalized;
  return SdkResult(ASTRALL_RES_SUCCESSED);
}

}  // namespace astrall_driver
