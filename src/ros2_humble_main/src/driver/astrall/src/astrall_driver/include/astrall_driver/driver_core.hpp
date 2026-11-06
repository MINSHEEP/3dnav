// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__DRIVER_CORE_HPP_
#define ASTRALL_DRIVER__DRIVER_CORE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "astrall_driver/driver_parameters.hpp"
#include "astrall_driver/i_sdk_client.hpp"
#include "astrall_driver/telemetry_data.hpp"
#include "astrall_driver/velocity_controller.hpp"

namespace astrall_driver {

enum class DriverPhase {
  kUnconfigured, kConnecting, kInactive, kActive, kFault, kClosing, kFinalized
};

/** @brief 可跨线程读取的值快照；零速结果只代表 SDK ACK，不证明物理停止。 */
struct DriverSnapshot {
  uint64_t telemetry_epoch_ = 0;
  DriverPhase phase_ = DriverPhase::kUnconfigured;
  bool configured_ = false;
  bool active_ = false;
  bool connected_ = false;
  bool has_control_authority_ = false;
  bool authority_release_pending_ = false;
  bool faulted_ = false;
  bool zero_attempted_ = false;
  uint16_t zero_result_ = ASTRALL_RES_NOT_INIT;
  bool release_attempted_ = false;
  uint16_t release_result_ = ASTRALL_RES_NOT_INIT;
  uint16_t last_result_ = ASTRALL_RES_NOT_INIT;
  uint64_t heartbeat_count_ = 0;
  uint64_t session_generation_ = 0;
  uint64_t telemetry_dropped_ = 0;
  uint64_t subscriptions_active_ = 0;
  uint64_t telemetry_clock_errors_ = 0;
  std::string telemetry_last_error_;
  uint64_t motion_epoch_ = 0;
  bool command_active_ = false;
  uint64_t control_send_count_ = 0;
  uint64_t watchdog_stops_ = 0;
  uint16_t control_result_ = ASTRALL_RES_NOT_INIT;
  std::string control_reason_;
  bool mode_pending_ = false;
  bool mode_can_move_ = false;
  uint16_t mode_actual_ = ASTRALL_SPORT_STATUS_UNKNOWN;
  uint16_t mode_result_ = ASTRALL_RES_NOT_INIT;
  uint16_t mode_ack_ = ASTRALL_RES_NOT_INIT;
  uint64_t mode_ticket_ = 0;
  std::string mode_reason_;
  std::string reason_ = "unconfigured";
};

/** @brief 返回稳定的阶段名称供日志/诊断使用，不访问共享状态。 */
const char* DriverPhaseName(DriverPhase phase);

/**
 * @brief 生命周期/遥测/控制核心；新命令须经过控制权、模式与时效门。
 * 状态互斥锁内不调用 SDK、不等待 future、不 join；转换由独立门串行化。
 * SDK 回调只更新弱寿命会话状态；心跳和 SDK 工作线程不依赖 ROS executor。
 * 析构前外部须停止成员调用；注入后端须遵守 SDK 的有界调用/返回码契约。
 */
class DriverCore {
 public:
  /**
   * @brief 注入 SDK 与接收时间函数，不自动连接；空客户端抛 invalid_argument。
   * clock 返回 ROS 接收时间 ns，须线程安全、快速且不阻塞；空函数用系统时钟。
   * clock 可由 SDK 分发线程和查询线程并发调用；异常会诊断、丢弃样本并撤权。
   */
  explicit DriverCore(std::shared_ptr<ISdkClient> client,
      std::function<int64_t()> receive_clock = {});
  /** @brief 幂等安全关闭并 join 所有工作线程。 */
  ~DriverCore();
  DriverCore(const DriverCore&) = delete;
  DriverCore& operator=(const DriverCore&) = delete;

  /** @brief 校验配置、连接、启动独立 10Hz 心跳；不申请控制权。 */
  SdkResult Configure(const DriverParameters& parameters);
  /** @brief 激活当前连接；停用后显式再激活会创建新会话。 */
  SdkResult Activate();
  /** @brief 撤销本地权限、尽力归零/释放、关闭会话，保留配置便于再激活。 */
  SdkResult Deactivate();
  /** @brief 幂等关闭并清除配置；未连接时不发送虚假的零速 ACK。 */
  SdkResult Cleanup();
  /** @brief 幂等终结；终结后禁止重新配置。 */
  SdkResult Shutdown();
  /**
   * @brief 手册 5.13；显式申请/释放，ACK 不设置确认权限。
   * 至多一个请求；等待有界且不持状态锁，关闭/故障会取消旧请求。
   */
  SdkResult RequestAuthority(AstrallAuth authority);
  /** @brief 只复制状态，不调用 SDK，可从 SDK 回调及 ROS 线程重入。 */
  DriverSnapshot Snapshot() const;
  /** @brief 每种数据仅交付一次最新样本，TTL 使用稳态时间；不依赖持权。 */
  std::vector<TelemetrySample> TakeTelemetry();
  /** @brief 发布前再核对生命周期、连接和代次；故障/失权不阻断已有遥测。 */
  bool TelemetryCurrent(uint64_t generation) const;
  /** @brief S5接收门；断线推进epoch，即使同会话恢复也不能发布断线前缓存。 */
  std::optional<uint64_t> TelemetryEpoch(uint64_t generation) const;
  /** @brief 带会话/运动代次接收新命令；旧回调不影响当前缓存。 */
  SdkResult SubmitVelocity(VelocityCommand command, uint64_t generation,
      uint64_t motion_epoch);
  /** @brief 只允许五种公开模式；ACK 后仍 pending 时返回 RUNNING。 */
  SdkResult RequestMode(uint16_t command, bool confirm_reboot_on_dock,
      uint64_t generation);
  /** @brief 低频灯光请求；由唯一 SDK 工作线程执行，返回可读原码。 */
  SdkResult RequestLight(uint16_t command, uint64_t generation);
  /** @brief 发送前实际时间限流，不因排队延迟补发成突发。 */
  SdkResult RequestText(const std::string& text, uint64_t generation);

 private:
  struct State;
  struct AuthorityRequest;
  struct SubscriptionRequest;
  struct ControlRequest;
  SdkResult Connect();
  void CloseSession(const std::string& reason);
  static SdkResult ExecuteSdk(const std::shared_ptr<State>& state,
      uint64_t generation, const std::string& operation,
      const std::function<SdkResult()>& action);
  static void ObserveStatus(const std::weak_ptr<State>& state,
      uint64_t generation, const AstrallSdkStatus& status);
  static void Fault(const std::shared_ptr<State>& state, uint64_t generation,
      const std::string& reason, uint16_t result);
  static void HeartbeatLoop(const std::shared_ptr<State>& state,
      uint64_t generation);
  static void WorkLoop(const std::shared_ptr<State>& state,
      uint64_t generation);
  static void PollSystem(const std::shared_ptr<State>& state,
      uint64_t generation);
  static void PollTelemetry(const std::shared_ptr<State>& state,
      uint64_t generation);
  static void StoreTelemetry(const std::shared_ptr<State>& state,
      uint64_t generation, TelemetryData data,
      bool valid = true);
  static void ProcessSubscription(const std::shared_ptr<State>& state,
      uint64_t generation,
      const SubscriptionRequest& request);
  void CancelSubscriptions(uint64_t generation);
  static void ControlLoop(const std::shared_ptr<State>& state,
      uint64_t generation);
  static void PollControlMode(const std::shared_ptr<State>& state,
      uint64_t generation);
  static void ProcessControl(const std::shared_ptr<State>& state,
      const std::shared_ptr<ControlRequest>& request);
  static void FinishControl(const std::shared_ptr<ControlRequest>& request,
      uint16_t result);
  SdkResult WaitControl(const std::shared_ptr<ControlRequest>& request);
  SdkResult QueueAuxiliary(const std::shared_ptr<ControlRequest>& request);
  static void SafetyStop(const std::shared_ptr<State>& state,
      uint64_t generation, bool release);
  static void ProcessAuthority(const std::shared_ptr<State>& state,
      const std::shared_ptr<AuthorityRequest>& request);
  static void FinishRequest(const std::shared_ptr<AuthorityRequest>& request,
      uint16_t result);

  std::mutex transition_mutex_;
  std::shared_ptr<State> state_;
  std::thread heartbeat_thread_;
  std::thread worker_thread_;
  std::thread control_thread_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__DRIVER_CORE_HPP_
