// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__SDK_RESULT_HPP_
#define ASTRALL_DRIVER__SDK_RESULT_HPP_

#include <cstdint>
#include <string>

namespace astrall_driver {

/** @brief 手册 5.1 调用结果的值类型；不用于模式状态，复制和读取线程安全。 */
class SdkResult final {
 public:
  /** @brief 保留未经修改的 uint16 原码，允许未知返回值。 */
  explicit SdkResult(uint16_t value);
  /** @brief 返回原码供 ROS 消息和诊断保留数值。 */
  uint16_t Value() const;
  /** @brief 仅 0x8008 表示完成成功，RUNNING 不视为成功。 */
  bool Succeeded() const;
  /** @brief 返回 SDK 常量名；未知码使用 UNKNOWN_SDK_RESULT。 */
  const char* Name() const;
  /** @brief 常量名及四位十六进制原码，避免日志仅有数字。 */
  std::string ToString() const;

 private:
  uint16_t value_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__SDK_RESULT_HPP_
