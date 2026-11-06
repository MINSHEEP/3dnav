// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/sdk_client.hpp"

#include <exception>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "sdk_client_state.hpp"

namespace astrall_driver {
namespace {

std::mutex process_mutex;
std::weak_ptr<void> process_session;

std::shared_ptr<void> AcquireProcessSession() {
  std::lock_guard<std::mutex> lock(process_mutex);
  if (!process_session.expired()) {
    return {};
  }
  auto session = std::make_shared<uint8_t>();
  process_session = session;
  return session;
}

bool Complete(const SdkFunctions& functions) {
  return functions.init_ && functions.deinit_ && functions.heartbeat_ &&
         functions.get_system_ && functions.get_power_ &&
         functions.get_sport_ &&
         functions.get_device_ && functions.get_imu_ && functions.subscribe_ &&
         functions.auth_ && functions.mode_ && functions.move_ &&
         functions.light_ && functions.send_message_;
}

}  // namespace

SdkClient::SdkClient() : SdkClient(MakeNativeSdkFunctions()) {
}

SdkClient::SdkClient(SdkFunctions functions)
  : state_(std::make_unique<State>(std::move(functions))) {
  if (!Complete(state_->functions_)) {
    throw std::invalid_argument("SDK function table is incomplete");
  }
}

SdkClient::~SdkClient() {
  Deinit();
}

SdkResult SdkClient::Init(const SdkCallbacks& callbacks, uint32_t timeout_ms) {
  auto owned_callbacks = callbacks;
  std::shared_ptr<SdkCallbackDispatcher> dispatcher;
  std::shared_ptr<SdkCallbackDispatcher> previous_dispatcher;
  std::vector<SdkCallbackDispatcher::CallbackHandle> retired;
  std::exception_ptr exception;
  SdkResult result(ASTRALL_RES_FAILED);
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    if (state_->initialized_ || state_->closing_) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    if (state_->dispatcher_ && !state_->dispatcher_->WaitIdle(0)) {
      return SdkResult(ASTRALL_RES_RUNNING);
    }
    if (state_->functions_.process_global_) {
      state_->process_session_ = AcquireProcessSession();
      if (!state_->process_session_) {
        return SdkResult(ASTRALL_RES_BEEN_OBTAINED);
      }
    }
    bool attempted = false;
    try {
      dispatcher = std::make_shared<SdkCallbackDispatcher>();
      previous_dispatcher = std::move(state_->dispatcher_);
      state_->dispatcher_ = dispatcher;
      state_->config_ = dispatcher->MakeConfig(std::move(owned_callbacks));
      attempted = true;
      result = SdkResult(state_->functions_.init_(state_->config_, timeout_ms));
      state_->initialized_ = result.Succeeded();
      if (state_->initialized_) {
        dispatcher->ActivateConfig();
        return result;
      }
    } catch (...) {
      exception = std::current_exception();
    }
    state_->closing_ = true;
    if (dispatcher) {
      retired = dispatcher->Disable();
    }
    if (attempted) {
      state_->functions_.deinit_();
    }
    state_->config_ = {};
    state_->process_session_.reset();
  }
  FinishClose(dispatcher);
  if (exception) {
    std::rethrow_exception(exception);
  }
  return result;
}

void SdkClient::FinishClose(
  const std::shared_ptr<SdkCallbackDispatcher>& dispatcher) {
  if (dispatcher) {
    dispatcher->Drain();
  }
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (state_->dispatcher_ == dispatcher) {
    state_->closing_ = false;
  }
}

void SdkClient::Deinit() {
  std::shared_ptr<SdkCallbackDispatcher> dispatcher;
  std::vector<SdkCallbackDispatcher::CallbackHandle> retired;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    dispatcher = state_->dispatcher_;
    if (state_->initialized_) {
      state_->initialized_ = false;
      state_->closing_ = true;
      retired = dispatcher->Disable();
      state_->functions_.deinit_();
      state_->subscriptions_.clear();
      state_->config_ = {};
      state_->process_session_.reset();
    }
  }
  // 不持 SDK 调用锁等待用户回调；否则回调重入会死锁。
  FinishClose(dispatcher);
}

SdkResult SdkClient::Call(const std::function<uint16_t()>& function,
    const ModeDispatchGuard& guard) {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (!state_->initialized_) {
    return SdkResult(ASTRALL_RES_NOT_INIT);
  }
  if (guard) {
    const auto result = guard();
    if (!result.Succeeded()) {
      return result;
    }
  }
  return SdkResult(function());
}

SdkResult SdkClient::Heartbeat(uint32_t timeout_ms) {
  return Call([this, timeout_ms] {
      return state_->functions_.heartbeat_(timeout_ms);
    });
}

SdkResult SdkClient::GetSystemStatus(AstrallSystemStatus* status) {
  if (status == nullptr) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  *status = {};
  AstrallSystemStatus value;
  auto result = Call([this, &value] {
      return state_->functions_.get_system_(value);
    });
  if (result.Succeeded()) {
    *status = value;
  }
  return result;
}

SdkResult SdkClient::GetPowerStatus(AstrallPowerStatus* power) {
  if (power == nullptr) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  *power = {};
  AstrallPowerStatus value;
  auto result = Call([this, &value] {
      return state_->functions_.get_power_(value);
    });
  if (result.Succeeded()) {
    *power = value;
  }
  return result;
}

AstrallSportStatus SdkClient::GetSportStatus() {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (!state_->initialized_) {
    return ASTRALL_SPORT_STATUS_UNKNOWN;
  }
  return static_cast<AstrallSportStatus>(state_->functions_.get_sport_());
}

SdkResult SdkClient::GetDeviceInfo(AstrallDeviceInfo* info,
    uint32_t timeout_ms) {
  if (info == nullptr) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  *info = {};
  AstrallDeviceInfo value;
  auto result = Call([this, &value, timeout_ms] {
      return state_->functions_.get_device_(value, timeout_ms);
    });
  if (result.Succeeded()) {
    *info = std::move(value);
  }
  return result;
}

SdkResult SdkClient::GetImuData(AstrallImuData* data) {
  if (data == nullptr) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  *data = {};
  AstrallImuData value;
  auto result = Call([this, &value] {
      return state_->functions_.get_imu_(value);
    });
  if (result.Succeeded()) {
    *data = value;
  }
  return result;
}

CallbackStatistics SdkClient::GetCallbackStatistics() const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  return state_->dispatcher_ ? state_->dispatcher_->Statistics() :
         CallbackStatistics{};
}

bool SdkClient::WaitForCallbacks(uint32_t timeout_ms) {
  std::shared_ptr<SdkCallbackDispatcher> dispatcher;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    dispatcher = state_->dispatcher_;
  }
  return !dispatcher || dispatcher->WaitIdle(timeout_ms);
}

}  // namespace astrall_driver
