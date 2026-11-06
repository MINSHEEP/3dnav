// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/sdk_client.hpp"

#include <cmath>
#include <limits>
#include <mutex>
#include <type_traits>
#include <utility>

#include "sdk_client_state.hpp"

namespace astrall_driver {
namespace {

constexpr float kSdkMaxVelocity = 1.0F;

bool ValidSubscription(AstrallSubscribeTopicId topic,
    AstrallSubscribeFreq frequency, bool has_callback) {
  if (frequency > ASTRALL_SUB_FREQ_250HZ) {
    return false;
  }
  if (topic == ASTRALL_SUB_TOPIC_ID_CAMERA_RGB ||
      topic == ASTRALL_SUB_TOPIC_ID_LIDAR) {
    return !has_callback;
  }
  if (topic != ASTRALL_SUB_TOPIC_ID_IMU &&
      topic != ASTRALL_SUB_TOPIC_ID_SPORT &&
      topic != ASTRALL_SUB_TOPIC_ID_JOYSTICK) {
    return false;
  }
  if (frequency == ASTRALL_SUB_FREQ_CLOSE) {
    return true;
  }
  return has_callback && (topic != ASTRALL_SUB_TOPIC_ID_JOYSTICK ||
         frequency == ASTRALL_SUB_FREQ_50HZ);
}

SdkCallbackDispatcher::Kind CallbackKind(AstrallSubscribeTopicId topic) {
  if (topic == ASTRALL_SUB_TOPIC_ID_IMU) {
    return SdkCallbackDispatcher::Kind::kImu;
  }
  if (topic == ASTRALL_SUB_TOPIC_ID_SPORT) {
    return SdkCallbackDispatcher::Kind::kSport;
  }
  return SdkCallbackDispatcher::Kind::kJoystick;
}

void Deliver(const SubscriptionCallback& callback,
    const SdkCallbackDispatcher::Value& value) {
  std::visit([&callback](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, AstrallImuData> ||
        std::is_same_v<T, AstrallSportData> ||
        std::is_same_v<T, AstrallJoystickData> ) {
          callback(SubscriptionData(data));
        }
      }, value);
}

bool ValidMode(AstrallSportModeCmd mode) {
  switch (mode) {
    case ASTRALL_SPORT_MODE_CMD_DAMPING:
    case ASTRALL_SPORT_MODE_CMD_FIXEDSTAND:
    case ASTRALL_SPORT_MODE_CMD_FIXEDDOWN:
    case ASTRALL_SPORT_MODE_CMD_MOVE:
    case ASTRALL_SPORT_MODE_CMD_AUTOCHARGE:
    case ASTRALL_SPORT_MODE_CMD_EXITCHARGE:
    case ASTRALL_SPORT_MODE_CMD_MOVE_TANK: return true;
    default: return false;
  }
}

bool ValidVelocity(float value) {
  return std::isfinite(value) && std::abs(value) <= kSdkMaxVelocity;
}

}  // namespace

SdkResult SdkClient::Subscribe(AstrallSubscribeTopicId topic,
    AstrallSubscribeFreq frequency,
    SubscriptionCallback callback,
    uint32_t timeout_ms) {
  if (!ValidSubscription(topic, frequency, static_cast<bool>(callback))) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  // 延迟到 API 锁释放后销毁用户捕获对象，其析构函数也可能重入 SDK。
  SdkCallbackDispatcher::CallbackHandle retired_previous;
  SdkCallbackDispatcher::CallbackHandle retired_rejected;
  std::unique_lock<std::mutex> lock(state_->mutex_);
  if (!state_->initialized_) {
    return SdkResult(ASTRALL_RES_NOT_INIT);
  }
  SdkCallbackDispatcher::Handle slot;
  RawSdkCallback raw = [](void*, uint16_t) {
      };
  if (callback && frequency != ASTRALL_SUB_FREQ_CLOSE) {
    slot = state_->dispatcher_->Add(
      [callback = std::move(callback)](const auto& value) {
        Deliver(callback, value);
      });
    raw = state_->dispatcher_->Bind(slot, CallbackKind(topic));
  }
  uint16_t code;
  try {
    code = state_->functions_.subscribe_(topic, frequency, raw, timeout_ms);
  } catch (...) {
    if (slot) {
      retired_rejected = SdkCallbackDispatcher::Disable(slot);
    }
    throw;
  }
  SdkResult result(code);
  if (!result.Succeeded()) {
    if (slot) {
      retired_rejected = SdkCallbackDispatcher::Disable(slot);
    }
    return result;
  }
  auto previous = state_->subscriptions_[topic];
  if (previous) {
    retired_previous = SdkCallbackDispatcher::Disable(previous);
  }
  state_->subscriptions_.erase(topic);
  if (slot) {
    state_->subscriptions_[topic] = slot;
    SdkCallbackDispatcher::Activate(slot);
  }
  lock.unlock();
  if (previous) {
    SdkCallbackDispatcher::Drain(previous);
  }
  return result;
}

SdkResult SdkClient::AuthControl(AstrallAuth authority, uint32_t timeout_ms) {
  if (authority != ASTRALL_AUTH_SDK && authority != ASTRALL_AUTH_JOYSTICK) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  return Call([this, authority, timeout_ms] {
      return state_->functions_.auth_(authority, timeout_ms);
    });
}

SdkResult SdkClient::SportModeControl(AstrallSportModeCmd mode,
    uint32_t timeout_ms, ModeDispatchGuard guard) {
  if (!ValidMode(mode)) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  return Call([this, mode, timeout_ms] {
      return state_->functions_.mode_(mode, timeout_ms);
    }, guard);
}

SdkResult SdkClient::Move(float vx, float vy, float vyaw, uint32_t timeout_ms) {
  if (!ValidVelocity(vx) || !ValidVelocity(vy) || !ValidVelocity(vyaw)) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  return Call([this, vx, vy, vyaw, timeout_ms] {
      return state_->functions_.move_(vx, vy, vyaw, timeout_ms);
    });
}

SdkResult SdkClient::LightControl(AstrallLightCmd command,
    uint32_t timeout_ms) {
  if (command != ASTRALL_CMD_LIGHT_OPEN && command != ASTRALL_CMD_LIGHT_CLOSE) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  return Call([this, command, timeout_ms] {
      return state_->functions_.light_(command, timeout_ms);
    });
}

SdkResult SdkClient::SendMessage(const std::string& text) {
  if (text.empty() || text.size() > std::numeric_limits<uint16_t>::max() ||
      text.find('\0') != std::string::npos) {
    return SdkResult(ASTRALL_RES_INVALID_PARAM);
  }
  std::string copy = text;
  return Call([this, &copy] {
      return state_->functions_.send_message_(
        copy.data(), static_cast<uint16_t>(copy.size()));
    });
}

}  // namespace astrall_driver
