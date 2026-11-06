// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/fake_sdk_client.hpp"

#include <mutex>
#include <utility>

#include "fake_sdk_state.hpp"

namespace astrall_driver {

SdkFunctions FakeSdkClient::MakeFunctions(const std::shared_ptr<State>& state) {
  SdkFunctions functions;
  functions.process_global_ = false;
  MakeLifecycle(state, &functions);
  MakeQueries(state, &functions);
  MakeControls(state, &functions);
  MakeSubscription(state, &functions);
  return functions;
}

void FakeSdkClient::MakeLifecycle(const std::shared_ptr<State>& state,
    SdkFunctions* functions) {
  functions->init_ = [state](AstrallConfig& config, uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        state->config_ = config;
        return state->Record({ SdkOperation::kInit, timeout });
      };
  functions->deinit_ = [state] {
        std::lock_guard<std::mutex> lock(state->mutex_);
        state->Record({ SdkOperation::kDeinit });
        state->config_ = {};
        state->subscriptions_.clear();
      };
  functions->heartbeat_ = [state](uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        return state->Record({ SdkOperation::kHeartbeat, timeout });
      };
}

void FakeSdkClient::MakeQueries(const std::shared_ptr<State>& state,
    SdkFunctions* functions) {
  functions->get_system_ = [state](AstrallSystemStatus& status) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        status = state->system_;
        return state->Record({ SdkOperation::kGetSystemStatus });
      };
  functions->get_power_ = [state](AstrallPowerStatus& power) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        power = state->power_;
        return state->Record({ SdkOperation::kGetPowerStatus });
      };
  functions->get_sport_ = [state] {
        std::lock_guard<std::mutex> lock(state->mutex_);
        state->Record({ SdkOperation::kGetSportStatus });
        return state->sport_;
      };
  functions->get_device_ = [state](AstrallDeviceInfo& info, uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        info = state->device_;
        return state->Record({ SdkOperation::kGetDeviceInfo, timeout });
      };
  functions->get_imu_ = [state](AstrallImuData& data) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        data = state->imu_;
        return state->Record({ SdkOperation::kGetImuData });
      };
}

void FakeSdkClient::MakeControls(const std::shared_ptr<State>& state,
    SdkFunctions* functions) {
  functions->auth_ = [state](AstrallAuth authority, uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        return state->Record({ SdkOperation::kAuthControl, timeout,
                               authority });
      };
  functions->mode_ = [state](AstrallSportModeCmd mode, uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        return state->Record({ SdkOperation::kSportModeControl, timeout,
                               mode });
      };
  functions->move_ = [state](float vx, float vy, float vyaw, uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        SdkCall call;
        call.operation_ = SdkOperation::kMove;
        call.timeout_ms_ = timeout;
        call.velocity_ = { vx, vy, vyaw };
        return state->Record(std::move(call));
      };
  functions->light_ = [state](AstrallLightCmd command, uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        return state->Record({ SdkOperation::kLightControl, timeout, command });
      };
  functions->send_message_ = [state](char* data, uint16_t length) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        SdkCall call;
        call.operation_ = SdkOperation::kSendMessage;
        call.text_.assign(data, length);
        return state->Record(std::move(call));
      };
}

void FakeSdkClient::MakeSubscription(const std::shared_ptr<State>& state,
    SdkFunctions* functions) {
  functions->subscribe_ = [state](AstrallSubscribeTopicId topic,
          AstrallSubscribeFreq frequency, RawSdkCallback callback,
          uint32_t timeout) {
        std::lock_guard<std::mutex> lock(state->mutex_);
        SdkCall call;
        call.operation_ = SdkOperation::kSubscribe;
        call.timeout_ms_ = timeout;
        call.topic_ = topic;
        call.frequency_ = frequency;
        const auto result = state->Record(std::move(call));
        if (result == ASTRALL_RES_SUCCESSED) {
          if (frequency == ASTRALL_SUB_FREQ_CLOSE) {
            state->subscriptions_.erase(topic);
          } else {
            state->subscriptions_[topic] = std::move(callback);
          }
        }
        return result;
      };
}

}  // namespace astrall_driver
