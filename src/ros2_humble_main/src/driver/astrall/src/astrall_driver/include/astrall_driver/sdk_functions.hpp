// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__SDK_FUNCTIONS_HPP_
#define ASTRALL_DRIVER__SDK_FUNCTIONS_HPP_

#include <cstdint>
#include <functional>

#include "astrall_sdk/interface.h"

namespace astrall_driver {

using RawSdkCallback = std::function<void (void*, uint16_t)>;

/**
 * @brief SDK 原始调用表，为离线测试注入后端；各函数须遵循返回码契约。
 * 客户端串行调用，后端不得直接执行用户回调；只调用原始 trampoline。
 * process_global_ 为 true 时执行进程级独占，真实 SDK 必须启用。
 */
struct SdkFunctions {
  bool process_global_ = true;
  std::function<uint16_t(AstrallConfig&, uint32_t)> init_;
  std::function<void()> deinit_;
  std::function<uint16_t(uint32_t)> heartbeat_;
  std::function<uint16_t(AstrallSystemStatus&)> get_system_;
  std::function<uint16_t(AstrallPowerStatus&)> get_power_;
  std::function<uint16_t()> get_sport_;
  std::function<uint16_t(AstrallDeviceInfo&, uint32_t)> get_device_;
  std::function<uint16_t(AstrallImuData&)> get_imu_;
  std::function<uint16_t(AstrallSubscribeTopicId, AstrallSubscribeFreq,
      RawSdkCallback, uint32_t)> subscribe_;
  std::function<uint16_t(AstrallAuth, uint32_t)> auth_;
  std::function<uint16_t(AstrallSportModeCmd, uint32_t)> mode_;
  std::function<uint16_t(float, float, float, uint32_t)> move_;
  std::function<uint16_t(AstrallLightCmd, uint32_t)> light_;
  std::function<uint16_t(char*, uint16_t)> send_message_;
};

/** @brief 仅绑定真实符号，不连接、不调用 SDK；实例化客户端也不建连接。 */
SdkFunctions MakeNativeSdkFunctions();

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__SDK_FUNCTIONS_HPP_
