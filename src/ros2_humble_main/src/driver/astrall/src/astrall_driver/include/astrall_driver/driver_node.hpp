// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__DRIVER_NODE_HPP_
#define ASTRALL_DRIVER__DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "astrall_driver/driver_core.hpp"
#include "astrall_driver/lidar_publisher.hpp"
#include "astrall_interfaces/msg/connection_status.hpp"
#include "astrall_interfaces/srv/set_control_authority.hpp"
#include "astrall_interfaces/srv/set_mode.hpp"
#include "astrall_interfaces/srv/set_light.hpp"
#include "astrall_interfaces/srv/send_message.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace astrall_driver {

class TelemetryPublisher;

/**
 * @brief 生命周期、控制与遥测/雷达 ROS 适配；默认 Fake，构造不连接 SDK。
 * 状态发布器在 inactive 时仍可发布失败诊断，心跳独立于 ROS executor。
 * 参数只读，通过启动覆盖/YAML 配置；注入客户端仅用于离线测试。
 */
class DriverNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  /** @brief 参数类型错误由 ROS 拒绝；范围错误在 configure 中报告。 */
  explicit DriverNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    std::shared_ptr<ISdkClient> client = nullptr);
  /** @brief 在销毁 ROS 接口前关闭核心、尽力归零并等待工作线程退出。 */
  ~DriverNode() override;
  /** @brief 非阻塞读取核心快照，可由测试或监控线程调用。 */
  DriverSnapshot DriverState() const;
  /** @brief 与生命周期操作串行调用；可与UDP线程并发读取实际绑定端口/诊断。 */
  LidarSnapshot LidarState() const;
  /** @brief 正常线程中的退出入口，不能从 POSIX 信号处理函数调用。 */
  void Stop();

 protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  /// @brief 校验参数并连接，不申请控制权，见手册 5.2/5.13。
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /// @brief 激活读状态和显式授权入口，心跳不由此回调驱动。
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /// @brief 归零、释放、关闭会话；下次 activate 使用新的会话代次。
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  /// @brief 清理配置和后台资源，重复调用核心清理安全。
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /// @brief 显式终结与程序退出共用相同安全路径。
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  /// @brief 生命周期内部失败后仍尝试关闭并保留可读诊断日志。
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  using AuthorityService = astrall_interfaces::srv::SetControlAuthority;
  void DeclareParameters();
  DriverParameters ReadParameters() const;
  void PublishState();
  void PublishTelemetry();
  void DeclareTelemetryParameters();
  void ReadTelemetryParameters(DriverParameters* parameters) const;
  void DeclareControlParameters();
  void ReadControlParameters(DriverParameters* parameters) const;
  void CreateControlServices();
  void ResetControlInterfaces();
  void UpdateCommandSubscription(const DriverSnapshot& snapshot);
  void DeclareLidarParameters();
  void ReadLidarParameters(DriverParameters* parameters) const;
  void CreateLidarPublisher();
  bool StartLidar();
  void StopLidar();
  void AddLidarDiagnostics(
    diagnostic_msgs::msg::DiagnosticStatus* status) const;
  CallbackReturn TransitionResult(const SdkResult& result);
  void HandleAuthority(
    const AuthorityService::Request::SharedPtr request,
    AuthorityService::Response::SharedPtr response);

  DriverParameters parameters_;
  std::unique_ptr<DriverCore> core_;
  std::unique_ptr<TelemetryPublisher> telemetry_publisher_;
  std::unique_ptr<LidarPublisher> lidar_publisher_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::Publisher<astrall_interfaces::msg::ConnectionStatus>::SharedPtr
      connection_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::Service<astrall_interfaces::srv::SetControlAuthority>::SharedPtr
      authority_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::string last_log_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      command_subscription_;
  rclcpp::Service<astrall_interfaces::srv::SetMode>::SharedPtr mode_service_;
  rclcpp::Service<astrall_interfaces::srv::SetLight>::SharedPtr light_service_;
  rclcpp::Service<astrall_interfaces::srv::SendMessage>::SharedPtr
      text_service_;
  uint64_t services_generation_ = 0;
  uint64_t command_generation_ = 0;
  uint64_t command_epoch_ = 0;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__DRIVER_NODE_HPP_
