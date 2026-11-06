// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__TEXT_RATE_LIMITER_HPP_
#define ASTRALL_DRIVER__TEXT_RATE_LIMITER_HPP_

#include <chrono>
#include <optional>

namespace astrall_driver {

/** @brief 手册 5.11：实际发送严格 <5Hz；不累积令牌、不补发突发队列。 */
class TextRateLimiter {
 public:
  using Time = std::chrono::steady_clock::time_point;
  /** @brief 0.1<=rate<5，向上取整到时钟 tick，非法值抛异常。 */
  explicit TextRateLimiter(double rate = 4);
  /** @brief 更新频率但保留发送窗口，生命周期重连不能形成突发。 */
  void Configure(double rate);
  /** @brief 只检查，不预留；实际 SDK 发送前必须再次 TrySend。 */
  bool Ready(Time now) const;
  /** @brief SDK 发送时调用；过频返回 false 且不改变上次发送时刻。 */
  bool TrySend(Time now);
  /** @brief 同步 SDK 返回后登记完成时间，覆盖等待 S1 锁造成的入口延迟。 */
  void RecordCompleted(Time now);

 private:
  std::chrono::steady_clock::duration interval_;
  std::optional<Time> last_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__TEXT_RATE_LIMITER_HPP_
