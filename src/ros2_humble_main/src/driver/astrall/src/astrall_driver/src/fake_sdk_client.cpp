// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/fake_sdk_client.hpp"

#include <mutex>
#include <stdexcept>
#include <utility>

#include "fake_sdk_state.hpp"

namespace astrall_driver {

FakeSdkClient::FakeSdkClient() : FakeSdkClient(std::make_shared<State>()) {
}

FakeSdkClient::FakeSdkClient(std::shared_ptr<State> state)
  : SdkClient(MakeFunctions(state)), state_(std::move(state)) {
}

void FakeSdkClient::SetResult(SdkOperation operation, uint16_t result) {
  if (operation == SdkOperation::kDeinit ||
      operation == SdkOperation::kGetSportStatus ||
      static_cast<unsigned int>(operation) >
      static_cast<unsigned int>(SdkOperation::kSendMessage)) {
    throw std::invalid_argument("Operation does not return an SDK result");
  }
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->results_[operation] = result;
}

std::vector<SdkCall> FakeSdkClient::Calls() const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  return state_->calls_;
}

void FakeSdkClient::ClearCalls() {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->calls_.clear();
}

void FakeSdkClient::SetSystemStatus(const AstrallSystemStatus& status) {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->system_ = status;
}

void FakeSdkClient::SetPowerStatus(const AstrallPowerStatus& power) {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->power_ = power;
}

void FakeSdkClient::SetSportStatus(uint16_t status) {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->sport_ = status;
}

void FakeSdkClient::SetDeviceInfo(const AstrallDeviceInfo& info) {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->device_ = info;
}

void FakeSdkClient::SetImuData(const AstrallImuData& data) {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  state_->imu_ = data;
}

bool FakeSdkClient::EmitHeartbeat() {
  RawSdkCallback callback;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    callback = state_->config_.heartbeatCb;
  }
  if (!callback) {
    return false;
  }
  callback(nullptr, 0);
  return true;
}

bool FakeSdkClient::EmitStatus(AstrallSdkStatus status) {
  return EmitRawStatus(&status, sizeof(status));
}

bool FakeSdkClient::EmitImu(AstrallImuData data) {
  return EmitRawData(ASTRALL_SUB_TOPIC_ID_IMU, &data, sizeof(data));
}

bool FakeSdkClient::EmitSport(AstrallSportData data) {
  return EmitRawData(ASTRALL_SUB_TOPIC_ID_SPORT, &data, sizeof(data));
}

bool FakeSdkClient::EmitJoystick(AstrallJoystickData data) {
  return EmitRawData(ASTRALL_SUB_TOPIC_ID_JOYSTICK, &data, sizeof(data));
}

bool FakeSdkClient::EmitRawStatus(void* data, uint16_t length) {
  auto callback = RawStatusCallback();
  if (!callback) {
    return false;
  }
  callback(data, length);
  return true;
}

bool FakeSdkClient::EmitRawData(AstrallSubscribeTopicId topic, void* data,
    uint16_t length) {
  auto callback = RawSubscriptionCallback(topic);
  if (!callback) {
    return false;
  }
  callback(data, length);
  return true;
}

RawSdkCallback FakeSdkClient::RawStatusCallback() const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  return state_->config_.sdkStatusCb;
}

RawSdkCallback FakeSdkClient::RawSubscriptionCallback(
  AstrallSubscribeTopicId topic) const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  const auto found = state_->subscriptions_.find(topic);
  return found ==
         state_->subscriptions_.end() ? RawSdkCallback{} : found->second;
}

}  // namespace astrall_driver
