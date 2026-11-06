// Copyright 2026 Hypertron ROS 2 contributors

#ifndef FAKE_SDK_STATE_HPP_
#define FAKE_SDK_STATE_HPP_

#include <map>
#include <mutex>
#include <utility>
#include <vector>

#include "astrall_driver/fake_sdk_client.hpp"

namespace astrall_driver {

struct FakeSdkClient::State {
  // 调用方必须持有 mutex_，以原子记录调用及读取注入返回码。
  uint16_t Record(SdkCall call) {
    auto operation = call.operation_;
    calls_.push_back(std::move(call));
    const auto found = results_.find(operation);
    return found == results_.end() ? ASTRALL_RES_SUCCESSED : found->second;
  }

  mutable std::mutex mutex_;
  std::map<SdkOperation, uint16_t> results_;
  std::vector<SdkCall> calls_;
  AstrallConfig config_;
  std::map<AstrallSubscribeTopicId, RawSdkCallback> subscriptions_;
  AstrallSystemStatus system_;
  AstrallPowerStatus power_;
  uint16_t sport_ = ASTRALL_SPORT_STATUS_UNKNOWN;
  AstrallDeviceInfo device_;
  AstrallImuData imu_;
};

}  // namespace astrall_driver

#endif  // FAKE_SDK_STATE_HPP_
