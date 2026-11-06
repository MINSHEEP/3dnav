// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_core.hpp"

#include <chrono>
#include <exception>
#include <utility>
#include <vector>

#include "driver_core_state.hpp"

namespace astrall_driver {

std::optional<uint64_t> DriverCore::TelemetryEpoch(uint64_t generation) const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (state_->stop_ || !state_->initialized_ || !state_->telemetry_active_ ||
      !state_->snapshot_.connected_ ||
      generation != state_->snapshot_.session_generation_) {
    return std::nullopt;
  }
  return state_->snapshot_.telemetry_epoch_;
}

bool DriverCore::TelemetryCurrent(uint64_t generation) const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  return !state_->stop_ && state_->initialized_ && state_->telemetry_active_ &&
         state_->snapshot_.connected_ &&
         generation == state_->snapshot_.session_generation_;
}

void DriverCore::StoreTelemetry(const std::shared_ptr<State>& state,
    uint64_t generation, TelemetryData data,
    bool valid) {
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    if (state->stop_ || !state->initialized_ || !state->telemetry_active_ ||
        !state->snapshot_.connected_ ||
        generation != state->snapshot_.session_generation_) {
      return;
    }
  }
  int64_t stamp = 0;
  std::string error;
  try {
    if (valid) {
      stamp = state->receive_clock_();
    }
  } catch (const std::exception& exception) {
    error = std::string("RECEIVE_CLOCK exception: ") + exception.what();
  } catch (...) {
    error = "RECEIVE_CLOCK unknown exception";
  }
  if (!error.empty()) {
    error += "; " + SdkResult(ASTRALL_RES_FAILED).ToString();
    {
      std::lock_guard<std::mutex> lock(state->mutex_);
      if (generation != state->snapshot_.session_generation_ || state->stop_) {
        return;
      }
      ++state->snapshot_.telemetry_clock_errors_;
      state->snapshot_.telemetry_last_error_ = error;
      state->snapshot_.telemetry_dropped_ += state->telemetry_.size() + 1;
      state->telemetry_.clear();
    }
    Fault(state, generation, error, ASTRALL_RES_FAILED);
    return;
  }
  const auto received = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(state->mutex_);
  if (state->stop_ || !state->initialized_ || !state->telemetry_active_ ||
      !state->snapshot_.connected_ ||
      generation != state->snapshot_.session_generation_) {
    return;
  }
  const auto kind = data.index();
  if (!valid) {
    state->telemetry_.erase(kind);
    return;
  }
  if (state->telemetry_.count(kind)) {
    ++state->snapshot_.telemetry_dropped_;
  }
  state->telemetry_.insert_or_assign(kind,
      TelemetrySample{ std::move(data), stamp, generation, received });
}

std::vector<TelemetrySample> DriverCore::TakeTelemetry() {
  std::vector<TelemetrySample> result;
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(state_->mutex_);
  if (!state_->stop_ && state_->initialized_ && state_->telemetry_active_ &&
      state_->snapshot_.connected_) {
    const auto max_age = std::chrono::milliseconds(
      state_->parameters_.telemetry_max_age_ms_);
    for (auto& entry : state_->telemetry_) {
      if (entry.second.session_generation_ ==
          state_->snapshot_.session_generation_ &&
          now - entry.second.received_steady_ <= max_age) {
        result.push_back(std::move(entry.second));
      } else {
        ++state_->snapshot_.telemetry_dropped_;
      }
    }
  }
  state_->telemetry_.clear();
  return result;
}

void DriverCore::ProcessSubscription(const std::shared_ptr<State>& state,
    uint64_t generation,
    const SubscriptionRequest& request) {
  uint32_t timeout;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    if (state->stop_ || generation != state->snapshot_.session_generation_) {
      return;
    }
    timeout = static_cast<uint32_t>(state->parameters_.sdk_timeout_ms_);
  }
  SubscriptionCallback callback;
  if (request.topic_ == ASTRALL_SUB_TOPIC_ID_IMU ||
      request.topic_ == ASTRALL_SUB_TOPIC_ID_SPORT ||
      request.topic_ == ASTRALL_SUB_TOPIC_ID_JOYSTICK) {
    callback =
        [weak = std::weak_ptr<State>(state), generation](const auto& data) {
          if (auto current = weak.lock()) {
            std::visit([&](const auto& value) {
            StoreTelemetry(current, generation, value);
          }, data);
          }
        };
  }
  auto result = ExecuteSdk(state, generation, "SUBSCRIBE", [&] {
      return state->client_->Subscribe(request.topic_, request.frequency_,
      callback, timeout);
    });
  if (result.Succeeded()) {
    std::lock_guard<std::mutex> lock(state->mutex_);
    // 关闭已改变代次时也登记资源，join 后的取消流程必须覆盖在途成功订阅。
    state->subscriptions_.insert(request.topic_);
    state->snapshot_.subscriptions_active_ = state->subscriptions_.size();
  } else {
    Fault(state,
        generation,
        "SUBSCRIBE topic=" + std::to_string(request.topic_) +
        "; " + result.ToString(),
        result.Value());
  }
}

void DriverCore::CancelSubscriptions(uint64_t generation) {
  std::vector<AstrallSubscribeTopicId> topics;
  uint32_t timeout;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    topics.assign(state_->subscriptions_.begin(), state_->subscriptions_.end());
    state_->subscriptions_.clear();
    state_->snapshot_.subscriptions_active_ = 0;
    timeout = static_cast<uint32_t>(state_->parameters_.sdk_timeout_ms_);
  }
  for (auto topic : topics) {
    auto result = ExecuteSdk(state_, generation, "UNSUBSCRIBE", [&] {
        return state_->client_->Subscribe(topic, ASTRALL_SUB_FREQ_CLOSE, {},
        timeout);
      });
    if (!result.Succeeded()) {
      Fault(state_, generation, "UNSUBSCRIBE topic=" + std::to_string(topic) +
          "; " + result.ToString(), result.Value());
    }
  }
}

void DriverCore::PollTelemetry(const std::shared_ptr<State>& state,
    uint64_t generation) {
  size_t index;
  uint32_t timeout;
  {
    std::lock_guard<std::mutex> lock(state->mutex_);
    if (state->stop_ || generation != state->snapshot_.session_generation_) {
      return;
    }
    constexpr size_t kQueryKinds = 3;
    index = state->telemetry_query_index_++ % kQueryKinds;
    timeout = static_cast<uint32_t>(state->parameters_.sdk_timeout_ms_);
  }
  SdkResult result(ASTRALL_RES_SUCCESSED);
  if (index == 0) {
    AstrallPowerStatus power;
    result = ExecuteSdk(state, generation, "POWER_STATUS", [&] {
        return state->client_->GetPowerStatus(&power);
      });
    StoreTelemetry(state, generation, power, result.Succeeded());
  } else if (index == 1) {
    auto sport = ASTRALL_SPORT_STATUS_UNKNOWN;
    result = ExecuteSdk(state, generation, "SPORT_STATUS", [&] {
        sport = state->client_->GetSportStatus();
        return SdkResult(ASTRALL_RES_SUCCESSED);
      });
    StoreTelemetry(state, generation, sport, result.Succeeded());
  } else {
    AstrallDeviceInfo device;
    result = ExecuteSdk(state, generation, "DEVICE_INFO", [&] {
        return state->client_->GetDeviceInfo(&device, timeout);
      });
    StoreTelemetry(state, generation, device, result.Succeeded());
  }
  if (!result.Succeeded()) {
    const auto error = "TELEMETRY_QUERY " + result.ToString();
    {
      std::lock_guard<std::mutex> lock(state->mutex_);
      if (generation == state->snapshot_.session_generation_) {
        state->snapshot_.telemetry_last_error_ = error;
      }
    }
    Fault(state, generation, error, result.Value());
  }
}

}  // namespace astrall_driver
