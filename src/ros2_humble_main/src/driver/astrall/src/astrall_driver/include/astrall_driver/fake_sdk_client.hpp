// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__FAKE_SDK_CLIENT_HPP_
#define ASTRALL_DRIVER__FAKE_SDK_CLIENT_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "astrall_driver/sdk_client.hpp"

namespace astrall_driver {

/** @brief 离线后端的可观测操作；模式状态必须通过单独接口注入。 */
enum class SdkOperation {
  kInit, kDeinit, kHeartbeat, kGetSystemStatus, kGetPowerStatus,
  kGetSportStatus, kGetDeviceInfo, kGetImuData, kSubscribe,
  kAuthControl, kSportModeControl, kMove, kLightControl, kSendMessage
};

/** @brief 原始 SDK 调用快照；timeout 为 ms，velocity 为 m/s、m/s、rad/s。 */
struct SdkCall {
  SdkOperation operation_ = SdkOperation::kInit;
  uint32_t timeout_ms_ = 0;
  uint16_t argument_ = 0;
  uint16_t topic_ = 0;
  uint16_t frequency_ = 0;
  std::array<float, 3> velocity_{};
  std::string text_{};
};

/**
 * @brief 不联网的 SDK 后端；复用真实封装的参数校验、会话和回调寿命策略。
 * 所有注入与快照接口均线程安全，回调在后端锁外进入分发队列。
 * 默认查询为零/UNKNOWN，命令 ACK 不隐式改变控制权或模式。
 */
class FakeSdkClient final : public SdkClient {
 public:
  /** @brief 创建相互独立的 Fake，不占用真实 SDK 的进程会话。 */
  FakeSdkClient();
  /** @brief 注入原始结果；Deinit 无返回码，GetSportStatus 使用独立 setter。 */
  void SetResult(SdkOperation operation, uint16_t result);
  /** @brief 查询快照含所有原始调用；不含在 SDK 边界被拒绝的调用。 */
  std::vector<SdkCall> Calls() const;
  /** @brief 仅清除记录，不改变订阅、会话或已注入状态。 */
  void ClearCalls();
  /** @brief 设置原始系统位域，不自动触发驱动安全策略。 */
  void SetSystemStatus(const AstrallSystemStatus& status);
  /** @brief 原始电量 soc 为百分比，保持 SDK 单位供转换测试。 */
  void SetPowerStatus(const AstrallPowerStatus& power);
  /** @brief 原始状态值允许未知 uint16，避免与结果码混用。 */
  void SetSportStatus(uint16_t status);
  /** @brief 复制设备字段；调用方修改原值不会影响后端。 */
  void SetDeviceInfo(const AstrallDeviceInfo& info);
  /** @brief 设置查询值：加速度 m/s²、角速度 rad/s、四元数 wxyz。 */
  void SetImuData(const AstrallImuData& data);
  /** @brief 模拟心跳 ACK；不自动调度或发送心跳。 */
  bool EmitHeartbeat();
  /** @brief 显式通知连接/控制权状态，不把命令 ACK 当状态确认。 */
  bool EmitStatus(AstrallSdkStatus status);
  /** @brief 复制并注入 IMU，只有成功订阅后才有接收者。 */
  bool EmitImu(AstrallImuData data);
  /** @brief 复制并注入原始轮速，单位不作猜测。 */
  bool EmitSport(AstrallSportData data);
  /** @brief 复制并注入原始摇杆结构和按钮位。 */
  bool EmitJoystick(AstrallJoystickData data);
  /** @brief 注入空/短状态；true 仅表示调用了 trampoline，不代表数据有效。 */
  bool EmitRawStatus(void* data, uint16_t length);
  /** @brief 注入空/短订阅数据；内存须至少含 length 字节。 */
  bool EmitRawData(AstrallSubscribeTopicId topic, void* data, uint16_t length);
  /** @brief 保留弱寿命 trampoline 供晚到回调测试，允许在 Fake 析构后调用。 */
  RawSdkCallback RawStatusCallback() const;
  /** @brief 取消订阅后返回空函数；之前取得的副本可测试晚到数据。 */
  RawSdkCallback RawSubscriptionCallback(AstrallSubscribeTopicId topic) const;

 private:
  struct State;
  explicit FakeSdkClient(std::shared_ptr<State> state);
  static SdkFunctions MakeFunctions(const std::shared_ptr<State>& state);
  static void MakeLifecycle(const std::shared_ptr<State>& state,
      SdkFunctions* functions);
  static void MakeQueries(const std::shared_ptr<State>& state,
      SdkFunctions* functions);
  static void MakeControls(const std::shared_ptr<State>& state,
      SdkFunctions* functions);
  static void MakeSubscription(const std::shared_ptr<State>& state,
      SdkFunctions* functions);

  std::shared_ptr<State> state_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__FAKE_SDK_CLIENT_HPP_
