// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__I_SDK_CLIENT_HPP_
#define ASTRALL_DRIVER__I_SDK_CLIENT_HPP_

#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <variant>

#include "astrall_driver/sdk_result.hpp"
#include "astrall_sdk/interface.h"

namespace astrall_driver {

inline constexpr uint32_t kSdkTimeoutMs = 20;
inline constexpr uint32_t kSdkInitTimeoutMs = 60000;

/** @brief 已校验长度并复制的数据；引用仅在用户回调执行期间有效。 */
using SubscriptionData =
    std::variant<AstrallImuData, AstrallSportData, AstrallJoystickData>;
using SubscriptionCallback = std::function<void (const SubscriptionData&)>;

/**
 * @brief 模式最终派发门；在 SDK 串行锁内、原始模式调用前同步执行一次。
 * SUCCESS 放行，其他结果原样返回且不调用原始 SDK；异常向调用方传播。
 * 必须快速返回，不得重入任何 SDK 方法、等待 future/回调或 join 线程。
 * 可短暂取得上层状态锁，但该锁不得覆盖任何 SDK 调用或阻塞等待。
 * 客户端不保存该函数；捕获资源在 API 锁外释放，生命周期须覆盖本次调用。
 */
using ModeDispatchGuard = std::function<SdkResult()>;

/** @brief 回调在专用分发线程执行，可重入客户端；不得等待其他回调。 */
struct SdkCallbacks {
  std::function<void()> heartbeat_;
  std::function<void(const AstrallSdkStatus&)> status_;
};

/** @brief 非法数据、队列溢出和用户异常可观测，避免静默丢弃。 */
struct CallbackStatistics {
  uint64_t invalid_payloads_ = 0;
  uint64_t dropped_events_ = 0;
  uint64_t callback_errors_ = 0;
  std::exception_ptr last_exception_;
};

/**
 * @brief S1 SDK 边界；方法可并发调用，超时单位均为 ms。
 * 不创建心跳定时器，不申请控制权，不自动输出运动指令。
 * 析构前调用方须停止并发方法调用；SDK 延迟回调可安全晚于析构到达。
 */
class ISdkClient {
 public:
  virtual ~ISdkClient() = default;
  /** @brief 见手册 5.2.2；阻塞连接，重复初始化返回 RUNNING。 */
  virtual SdkResult Init(const SdkCallbacks& callbacks,
      uint32_t timeout_ms = kSdkInitTimeoutMs) = 0;
  /** @brief 见 5.4；幂等关闭并排空回调，可从用户回调内调用。 */
  virtual void Deinit() = 0;
  /** @brief 见 5.5；只发送一次，10Hz 调度由后续驱动负责。 */
  virtual SdkResult Heartbeat(uint32_t timeout_ms = kSdkTimeoutMs) = 0;
  /** @brief 见 5.7；输出指针不能为空，失败时输出恢复默认值。 */
  virtual SdkResult GetSystemStatus(AstrallSystemStatus* status) = 0;
  /** @brief 见 5.8；保留 SDK 单位：soc 为百分比，电压为 V。 */
  virtual SdkResult GetPowerStatus(AstrallPowerStatus* power) = 0;
  /** @brief 见 5.9；返回独立状态枚举，未初始化时为 UNKNOWN。 */
  virtual AstrallSportStatus GetSportStatus() = 0;
  /** @brief 见 5.6；失败不保留上次设备信息，输出不能为空。 */
  virtual SdkResult GetDeviceInfo(AstrallDeviceInfo* info,
      uint32_t timeout_ms = kSdkTimeoutMs) = 0;
  /** @brief 见 5.10；保留加速度 m/s²、角速度 rad/s、四元数 wxyz。 */
  virtual SdkResult GetImuData(AstrallImuData* data) = 0;
  /**
   * @brief 见 5.10；枚举频率，摇杆仅 50Hz；CLOSE 取消订阅。
   * RGB/LIDAR 仅启停 UDP，不传用户回调。成功替换/取消后旧回调排空。
   */
  virtual SdkResult Subscribe(AstrallSubscribeTopicId topic,
      AstrallSubscribeFreq frequency,
      SubscriptionCallback callback = {},
      uint32_t timeout_ms = kSdkTimeoutMs) = 0;
  /** @brief 见 5.13；ACK 不代表状态回调确认的控制权。 */
  virtual SdkResult AuthControl(AstrallAuth authority,
      uint32_t timeout_ms = kSdkTimeoutMs) = 0;
  /**
   * @brief 见 5.12；可选 guard 在取得 SDK 锁后复查 S4 截止时间及取消状态。
   * guard 放行是软件派发点；已经进入原始 SDK 的调用不能撤回。
   */
  virtual SdkResult SportModeControl(AstrallSportModeCmd mode,
      uint32_t timeout_ms = kSdkTimeoutMs, ModeDispatchGuard guard = {}) = 0;
  /** @brief 见 5.14；vx/vy 为 m/s、vyaw 为 rad/s；各轴须在 ±1 内。 */
  virtual SdkResult Move(float vx, float vy, float vyaw,
      uint32_t timeout_ms = kSdkTimeoutMs) = 0;
  /** @brief 见 5.15；只接受 OPEN/CLOSE，超时为 ms。 */
  virtual SdkResult LightControl(AstrallLightCmd command,
      uint32_t timeout_ms = kSdkTimeoutMs) = 0;
  /** @brief 见 5.11；检查非空 uint16 字节长度，<5Hz 调度属于 S4。 */
  virtual SdkResult SendMessage(const std::string& text) = 0;
  /** @brief 返回当前或最近会话的诊断快照；异常保留供上层报告。 */
  virtual CallbackStatistics GetCallbackStatistics() const = 0;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__I_SDK_CLIENT_HPP_
