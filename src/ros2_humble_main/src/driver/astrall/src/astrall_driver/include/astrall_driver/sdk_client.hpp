// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__SDK_CLIENT_HPP_
#define ASTRALL_DRIVER__SDK_CLIENT_HPP_

#include <functional>
#include <memory>
#include <string>

#include "astrall_driver/i_sdk_client.hpp"
#include "astrall_driver/sdk_functions.hpp"

namespace astrall_driver {

class SdkCallbackDispatcher;

/**
 * @brief 真实 SDK 封装；SDK 调用串行化，用户回调在独立线程和锁外执行。
 * 真实会话进程独占，未显式 Init 前不触碰 SDK。不得绕过本类直调原 SDK。
 * 回调队列最多 256 项，溢出计入诊断；关闭时丢弃待执行项并排空正在执行项。
 */
class SdkClient : public ISdkClient {
 public:
  /** @brief 绑定厂商符号但不初始化，不连接网络。 */
  SdkClient();
  /** @brief 注入完整离线调用表；缺少任一函数抛 invalid_argument。 */
  explicit SdkClient(SdkFunctions functions);
  /** @brief 关闭会话；可从用户回调内释放实例，不等待自身回调。 */
  ~SdkClient() override;
  SdkClient(const SdkClient&) = delete;
  SdkClient& operator=(const SdkClient&) = delete;

  /// @copydoc ISdkClient::Init
  SdkResult Init(const SdkCallbacks& callbacks,
      uint32_t timeout_ms = kSdkInitTimeoutMs) override;
  /// @copydoc ISdkClient::Deinit
  void Deinit() override;
  /// @copydoc ISdkClient::Heartbeat
  SdkResult Heartbeat(uint32_t timeout_ms = kSdkTimeoutMs) override;
  /// @copydoc ISdkClient::GetSystemStatus
  SdkResult GetSystemStatus(AstrallSystemStatus* status) override;
  /// @copydoc ISdkClient::GetPowerStatus
  SdkResult GetPowerStatus(AstrallPowerStatus* power) override;
  /// @copydoc ISdkClient::GetSportStatus
  AstrallSportStatus GetSportStatus() override;
  /// @copydoc ISdkClient::GetDeviceInfo
  SdkResult GetDeviceInfo(AstrallDeviceInfo* info,
      uint32_t timeout_ms = kSdkTimeoutMs) override;
  /// @copydoc ISdkClient::GetImuData
  SdkResult GetImuData(AstrallImuData* data) override;
  /// @copydoc ISdkClient::Subscribe
  SdkResult Subscribe(AstrallSubscribeTopicId topic,
      AstrallSubscribeFreq frequency,
      SubscriptionCallback callback = {},
      uint32_t timeout_ms = kSdkTimeoutMs) override;
  /// @copydoc ISdkClient::AuthControl
  SdkResult AuthControl(AstrallAuth authority,
      uint32_t timeout_ms = kSdkTimeoutMs) override;
  /// @copydoc ISdkClient::SportModeControl
  SdkResult SportModeControl(AstrallSportModeCmd mode,
      uint32_t timeout_ms = kSdkTimeoutMs,
      ModeDispatchGuard guard = {}) override;
  /// @copydoc ISdkClient::Move
  SdkResult Move(float vx, float vy, float vyaw,
      uint32_t timeout_ms = kSdkTimeoutMs) override;
  /// @copydoc ISdkClient::LightControl
  SdkResult LightControl(AstrallLightCmd command,
      uint32_t timeout_ms = kSdkTimeoutMs) override;
  /// @copydoc ISdkClient::SendMessage
  SdkResult SendMessage(const std::string& text) override;
  /// @copydoc ISdkClient::GetCallbackStatistics
  CallbackStatistics GetCallbackStatistics() const override;
  /** @brief 离线测试等待队列排空，超时单位 ms；回调线程内返回 false。 */
  bool WaitForCallbacks(uint32_t timeout_ms);

 private:
  struct State;
  SdkResult Call(const std::function<uint16_t()>& function,
      const ModeDispatchGuard& guard = {});
  void FinishClose(const std::shared_ptr<SdkCallbackDispatcher>& dispatcher);

  std::unique_ptr<State> state_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__SDK_CLIENT_HPP_
