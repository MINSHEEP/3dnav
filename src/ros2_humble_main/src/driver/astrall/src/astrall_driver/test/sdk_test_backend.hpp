// Copyright 2026 Hypertron ROS 2 contributors

#ifndef SDK_TEST_BACKEND_HPP_
#define SDK_TEST_BACKEND_HPP_

#include <atomic>
#include <memory>
#include <utility>

#include "astrall_driver/sdk_functions.hpp"

namespace astrall_driver {
namespace test {

/** @brief 只替换 SDK 原始函数，不访问网络；保存 trampoline 测试晚到回调。 */
struct Backend {
  AstrallConfig config_;
  RawSdkCallback subscription_;
  uint16_t init_result_ = ASTRALL_RES_SUCCESSED;
  uint16_t subscribe_result_ = ASTRALL_RES_SUCCESSED;
  std::atomic<int> init_count_{ 0 };
  std::atomic<int> deinit_count_{ 0 };
};

inline SdkFunctions Functions(const std::shared_ptr<Backend>& backend) {
  SdkFunctions functions;
  functions.process_global_ = false;
  functions.init_ = [backend](AstrallConfig& config, uint32_t) {
        backend->config_ = config;
        ++backend->init_count_;
        return backend->init_result_;
      };
  functions.deinit_ = [backend] {
        ++backend->deinit_count_;
      };
  functions.heartbeat_ = [](uint32_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.get_system_ = [](AstrallSystemStatus&) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.get_power_ = [](AstrallPowerStatus&) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.get_sport_ = [] {
        return ASTRALL_SPORT_STATUS_MOVE;
      };
  functions.get_device_ = [](AstrallDeviceInfo&, uint32_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.get_imu_ = [](AstrallImuData&) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.subscribe_ = [backend](AstrallSubscribeTopicId,
          AstrallSubscribeFreq frequency, RawSdkCallback callback, uint32_t) {
        if (backend->subscribe_result_ == ASTRALL_RES_SUCCESSED) {
          backend->subscription_ = frequency == ASTRALL_SUB_FREQ_CLOSE ?
              RawSdkCallback{} : std::move(callback);
        }
        return backend->subscribe_result_;
      };
  functions.auth_ = [](AstrallAuth, uint32_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.mode_ = [](AstrallSportModeCmd, uint32_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.move_ = [](float, float, float, uint32_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.light_ = [](AstrallLightCmd, uint32_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  functions.send_message_ = [](char*, uint16_t) {
        return ASTRALL_RES_SUCCESSED;
      };
  return functions;
}

}  // namespace test
}  // namespace astrall_driver

#endif  // SDK_TEST_BACKEND_HPP_
