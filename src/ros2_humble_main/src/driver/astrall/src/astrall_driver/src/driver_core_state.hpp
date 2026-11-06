// Copyright 2026 Hypertron ROS 2 contributors

#ifndef DRIVER_CORE_STATE_HPP_
#define DRIVER_CORE_STATE_HPP_

#include <atomic>
#include <array>
#include <condition_variable>
#include <future>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <set>
#include <utility>

#include "astrall_driver/driver_core.hpp"
#include "astrall_driver/subscription_frequency.hpp"
#include "astrall_driver/mode_state_machine.hpp"
#include "astrall_driver/text_rate_limiter.hpp"

namespace astrall_driver {

struct DriverCore::AuthorityRequest {
  AstrallAuth authority_ = ASTRALL_AUTH_JOYSTICK;
  uint64_t generation_ = 0;
  uint64_t epoch_ = 0;
  std::atomic<bool> cancelled_{ false };
  std::atomic<bool> completed_{ false };
  std::promise<SdkResult> promise_;
};

struct DriverCore::SubscriptionRequest {
  AstrallSubscribeTopicId topic_;
  AstrallSubscribeFreq frequency_;
};

struct DriverCore::ControlRequest {
  enum class Kind { kMode, kLight, kText };
  Kind kind_ = Kind::kMode;
  uint16_t command_ = 0;
  std::string text_;
  uint64_t generation_ = 0;
  uint64_t epoch_ = 0;
  uint64_t mode_ticket_ = 0;
  std::atomic<bool> cancelled_{ false };
  std::atomic<bool> completed_{ false };
  std::promise<SdkResult> promise_;
};

struct DriverCore::State {
  State(std::shared_ptr<ISdkClient> client, std::function<int64_t()> clock)
    : client_(std::move(client)), receive_clock_(std::move(clock)) {
  }

  // 只构造有限的订阅计划，实际 SDK 调用由工作线程逐项执行。
  void PrepareTelemetryLocked() {
    telemetry_active_ = false;
    telemetry_.clear();
    subscription_requests_.clear();
    subscriptions_.clear();
    telemetry_query_index_ = 0;
    callback_diagnostics_seen_ = 0;
    const std::array<std::pair<AstrallSubscribeTopicId, int64_t>,
        3> sensors = { {
      { ASTRALL_SUB_TOPIC_ID_IMU, parameters_.imu_frequency_hz_ },
      { ASTRALL_SUB_TOPIC_ID_SPORT, parameters_.wheel_speed_frequency_hz_ },
      { ASTRALL_SUB_TOPIC_ID_JOYSTICK, parameters_.joystick_frequency_hz_ }
    } };
    for (const auto& sensor : sensors) {
      if (sensor.second != 0) {
        subscription_requests_.push_back({ sensor.first,
                                           SubscriptionFrequency(
                                             sensor.second).value() });
      }
    }
    if (parameters_.enable_rgb_) {
      subscription_requests_.push_back({ ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
                                         ASTRALL_SUB_FREQ_1HZ });
    }
    if (parameters_.enable_lidar_) {
      subscription_requests_.push_back({ ASTRALL_SUB_TOPIC_ID_LIDAR,
                                         ASTRALL_SUB_FREQ_1HZ });
    }
  }

  void PrepareControlLocked() {
    velocity_ = VelocityController({ parameters_.max_forward_velocity_,
                                     parameters_.max_lateral_velocity_,
                                     parameters_.max_yaw_velocity_ },
            std::chrono::milliseconds(parameters_.cmd_vel_timeout_ms_));
    mode_ =
        ModeStateMachine(std::chrono::milliseconds(
              parameters_.mode_switch_timeout_ms_),
            std::chrono::milliseconds(parameters_.mode_state_freshness_ms_),
            parameters_.allow_reboot_on_dock_stand_);
    text_limiter_.Configure(parameters_.text_rate_hz_);
    control_request_.reset();
    ++control_epoch_;
  }

  void ClearMotionLocked() {
    velocity_.Clear();
    ++snapshot_.motion_epoch_;
  }

  // 调用者持 mutex_；SDK 最终门也调用此函数，禁止 SDK I/O/等待/线程 join。
  // 排队、预零、SDK 锁等待后须推进 deadline，不能仅读 Pending。
  uint16_t CheckModeDispatchLocked(
    const std::shared_ptr<ControlRequest>& request) {
    if (stop_ || request->generation_ != snapshot_.session_generation_ ||
        control_request_ != request) {
      return ASTRALL_RES_NOT_INIT;
    }
    if (mode_.Tick(std::chrono::steady_clock::now())) {
      FaultLocked("MODE " + SdkResult(ASTRALL_RES_TIMEOUT).ToString(),
          ASTRALL_RES_TIMEOUT);
    }
    // Tick 也可能已由控制线程执行；保留该请求的 TIMEOUT，不能改为取消码。
    if (mode_.Result() == ASTRALL_RES_TIMEOUT) {
      snapshot_.control_result_ = ASTRALL_RES_TIMEOUT;
      snapshot_.control_reason_ = mode_.Reason();
      return ASTRALL_RES_TIMEOUT;
    }
    if (request->cancelled_ || request->epoch_ != control_epoch_ ||
        request->mode_ticket_ != mode_.Ticket() || !mode_.Pending()) {
      return ASTRALL_RES_NOT_INIT;
    }
    return ASTRALL_RES_SUCCESSED;
  }

  // 调用者持 mutex_；仅更新本地安全状态，不在回调中进入 SDK 或转换门。
  void FaultLocked(const std::string& reason, uint16_t result) {
    if (!snapshot_.faulted_) {
      ClearMotionLocked();
    } else {
      velocity_.Clear();
    }
    const bool damping = mode_.Pending() &&
        mode_.TargetStatus() == ASTRALL_SPORT_STATUS_DAMPING;
    if (!damping) {
      mode_.Cancel(result, reason);
      ++control_epoch_;
      if (control_request_) {
        control_request_->cancelled_ = true;
      }
    }
    if (!snapshot_.faulted_) {
      snapshot_.reason_ = reason;
      snapshot_.last_result_ = result;
    }
    snapshot_.faulted_ = true;
    snapshot_.active_ = false;
    snapshot_.has_control_authority_ = false;
    snapshot_.authority_release_pending_ = false;
    snapshot_.phase_ = DriverPhase::kFault;
    grant_allowed_ = false;
    ++authority_epoch_;
    safety_pending_ = true;
    condition_.notify_all();
  }

  const std::shared_ptr<ISdkClient> client_;
  const std::function<int64_t()> receive_clock_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::mutex motion_io_mutex_;
  DriverParameters parameters_;
  DriverSnapshot snapshot_;
  bool initialized_ = false;
  bool session_attempted_ = false;
  bool stop_ = true;
  bool accept_callbacks_ = false;
  bool link_observed_ = false;
  bool sdk_authority_observed_ = false;
  bool grant_allowed_ = false;
  bool release_needed_ = false;
  bool safety_pending_ = false;
  uint64_t authority_epoch_ = 0;
  std::shared_ptr<AuthorityRequest> request_;
  bool telemetry_active_ = false;
  std::map<size_t, TelemetrySample> telemetry_;
  std::deque<SubscriptionRequest> subscription_requests_;
  std::set<AstrallSubscribeTopicId> subscriptions_;
  size_t telemetry_query_index_ = 0;
  uint64_t callback_diagnostics_seen_ = 0;
  VelocityController velocity_;
  ModeStateMachine mode_;
  TextRateLimiter text_limiter_;
  uint64_t control_epoch_ = 0;
  std::shared_ptr<ControlRequest> control_request_;
};

}  // namespace astrall_driver

#endif  // DRIVER_CORE_STATE_HPP_
