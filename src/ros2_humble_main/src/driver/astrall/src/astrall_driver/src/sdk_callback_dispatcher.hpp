// Copyright 2026 Hypertron ROS 2 contributors

#ifndef SDK_CALLBACK_DISPATCHER_HPP_
#define SDK_CALLBACK_DISPATCHER_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <variant>
#include <vector>

#include "astrall_driver/i_sdk_client.hpp"
#include "astrall_driver/sdk_functions.hpp"

namespace astrall_driver {

/** @brief 内部有界回调分发；原始 trampoline 只复制数据，不执行用户代码。 */
class SdkCallbackDispatcher {
 public:
  using Value = std::variant<std::monostate, AstrallSdkStatus,
          AstrallImuData, AstrallSportData, AstrallJoystickData>;
  using Callback = std::function<void (const Value&)>;
  using CallbackHandle = std::shared_ptr<const Callback>;
  enum class Kind { kHeartbeat, kStatus, kImu, kSport, kJoystick };

  struct Slot {
    std::mutex mutex_;
    std::condition_variable condition_;
    bool ready_ = false;
    bool enabled_ = true;
    bool running_ = false;
    std::thread::id thread_id_;
    CallbackHandle callback_;
  };
  using Handle = std::shared_ptr<Slot>;

  SdkCallbackDispatcher();
  ~SdkCallbackDispatcher();
  SdkCallbackDispatcher(const SdkCallbackDispatcher&) = delete;
  SdkCallbackDispatcher& operator=(const SdkCallbackDispatcher&) = delete;

  AstrallConfig MakeConfig(SdkCallbacks callbacks);
  Handle Add(std::function<void(const Value&)> callback);
  RawSdkCallback Bind(const Handle& slot, Kind kind);
  void ActivateConfig();
  static void Activate(const Handle& slot);
  static CallbackHandle Disable(const Handle& slot);
  static void Drain(const Handle& slot);
  std::vector<CallbackHandle> Disable();
  void Drain();
  bool WaitIdle(uint32_t timeout_ms);
  CallbackStatistics Statistics() const;

 private:
  struct State;
  static void Run(const std::shared_ptr<State>& state);
  static void Invoke(const std::shared_ptr<State>& state,
      const Handle& slot, const Value& value);

  std::shared_ptr<State> state_;
  std::thread worker_;
  std::vector<Handle> config_slots_;
};

}  // namespace astrall_driver

#endif  // SDK_CALLBACK_DISPATCHER_HPP_
