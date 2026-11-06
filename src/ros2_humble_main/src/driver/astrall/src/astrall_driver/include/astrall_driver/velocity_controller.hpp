// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__VELOCITY_CONTROLLER_HPP_
#define ASTRALL_DRIVER__VELOCITY_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>

namespace astrall_driver {

/** @brief 手册 5.14：前进/平移 m/s、转向 rad/s。 */
struct VelocityCommand {
  double forward_ = 0;
  double lateral_ = 0;
  double yaw_ = 0;
};

struct VelocitySample {
  VelocityCommand command_;
  uint64_t revision_ = 0;
  bool send_ = false;
  bool expired_ = false;
};

/** @brief 纯稳态时间速度缓存，无内部锁；调用方串行化，不依赖 ROS 时间。 */
class VelocityController {
 public:
  using Time = std::chrono::steady_clock::time_point;
  /** @brief 限幅必须有限且在 [0,1]，watchdog 为正；非法配置抛异常。 */
  explicit VelocityController(VelocityCommand limits = { 1, 1, 1 },
      std::chrono::milliseconds watchdog = std::chrono::milliseconds(250));
  /** @brief 非有限/不允许运动时清缓存并要求零速；新命令逐轴限幅。 */
  bool Accept(VelocityCommand command, Time now, bool allowed);
  /** @brief 每个固定发送周期取样；超时或门关闭使旧命令失效。 */
  VelocitySample Sample(Time now, bool allowed);
  /** @brief 显式清除，不保留恢复后可重放的速度。 */
  void Clear();
  /** @brief 仅确认同一版本的零速尝试，不能消掉较新的归零要求。 */
  void MarkSent(uint64_t revision);
  /** @brief 是否仍有当前命令，过期判断由 Sample 执行。 */
  bool HasCommand() const;

 private:
  VelocityCommand limits_;
  std::chrono::milliseconds watchdog_;
  VelocityCommand command_;
  Time received_;
  uint64_t revision_ = 0;
  bool has_command_ = false;
  bool zero_needed_ = false;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__VELOCITY_CONTROLLER_HPP_
