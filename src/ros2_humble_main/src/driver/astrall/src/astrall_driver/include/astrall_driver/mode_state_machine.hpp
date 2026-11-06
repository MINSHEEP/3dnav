// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__MODE_STATE_MACHINE_HPP_
#define ASTRALL_DRIVER__MODE_STATE_MACHINE_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "astrall_sdk/interface.h"

namespace astrall_driver {

struct ModeDecision {
  bool accepted_ = false;
  uint16_t result_ = ASTRALL_RES_FAILED;
  uint64_t ticket_ = 0;
  std::string reason_;
};

/** @brief 手册 5.12.3 的纯状态机；外部串行化，所有时间为稳态时间。 */
class ModeStateMachine {
 public:
  using Time = std::chrono::steady_clock::time_point;
  /** @brief 重启在桩站立例外须参数启用并由每次请求显式确认，默认关闭。 */
  explicit ModeStateMachine(std::chrono::milliseconds timeout =
      std::chrono::milliseconds(3000), std::chrono::milliseconds freshness =
      std::chrono::milliseconds(250), bool allow_reboot_stand = false);
  /** @brief 更新实际完成状态；只有新观察匹配目标且有 ACK 才完成。 */
  void Observe(AstrallSportStatus actual, Time now);
  /** @brief 校验十项规则；阻尼可抢占，自动充电/退桩仅检查后报告不支持。 */
  ModeDecision Begin(uint16_t command, bool confirm_reboot_on_dock, Time now);
  /** @brief 仅当前 ticket 生效；SUCCESS/RUNNING 均等待实际状态，不立即完成。 */
  bool Acknowledge(uint64_t ticket, uint16_t result, Time now);
  /** @brief 安全事件废弃旧 ticket，不恢复运动。 */
  void Cancel(uint16_t result, const std::string& reason);
  /** @brief pending 到期记 TIMEOUT；返回 true 表示本次新发生超时。 */
  bool Tick(Time now);
  /** @brief 状态足够新且实际在 Move/Tank、无切换时才允许平面速度。 */
  bool CanMove(Time now) const;
  /** @brief 当前是否等待模式完成。 */
  bool Pending() const;
  /** @brief 保留最新实际状态，不把 ACK 当模式值。 */
  AstrallSportStatus Actual() const;
  /** @brief 当前切换结果；pending=RUNNING，完成=SUCCESS，失败保留 SDK 原码。 */
  uint16_t Result() const;
  /** @brief 当前有效 ticket，用于拒绝抢占前的旧 ACK。 */
  uint64_t Ticket() const;
  /** @brief 目标实际状态，用于阻尼抢占和诊断，不是 SDK opcode。 */
  uint16_t TargetStatus() const;
  /** @brief 可读前置失败、等待或超时说明。 */
  const std::string& Reason() const;

 private:
  ModeDecision Check(uint16_t command, bool confirmation, Time now) const;
  void CompleteIfObserved();

  std::chrono::milliseconds timeout_;
  std::chrono::milliseconds freshness_;
  bool allow_reboot_stand_;
  AstrallSportStatus actual_ = ASTRALL_SPORT_STATUS_UNKNOWN;
  Time observed_at_;
  Time deadline_;
  uint64_t observations_ = 0;
  uint64_t request_observation_ = 0;
  uint64_t ticket_ = 0;
  uint16_t target_ = 0;
  uint16_t result_ = ASTRALL_RES_NOT_INIT;
  bool pending_ = false;
  bool acknowledged_ = false;
  bool failed_ = false;
  std::string reason_ = "no mode request";
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__MODE_STATE_MACHINE_HPP_
