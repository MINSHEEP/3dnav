// Copyright 2026 Hypertron ROS 2 contributors

#ifndef SDK_CLIENT_STATE_HPP_
#define SDK_CLIENT_STATE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <utility>

#include "astrall_driver/sdk_client.hpp"
#include "sdk_callback_dispatcher.hpp"

namespace astrall_driver {

struct SdkClient::State {
  explicit State(SdkFunctions functions) : functions_(std::move(functions)) {
  }

  const SdkFunctions functions_;
  mutable std::mutex mutex_;
  bool initialized_ = false;
  bool closing_ = false;
  std::shared_ptr<void> process_session_;
  std::shared_ptr<SdkCallbackDispatcher> dispatcher_;
  AstrallConfig config_;
  std::map<AstrallSubscribeTopicId,
      SdkCallbackDispatcher::Handle> subscriptions_;
};

}  // namespace astrall_driver

#endif  // SDK_CLIENT_STATE_HPP_
