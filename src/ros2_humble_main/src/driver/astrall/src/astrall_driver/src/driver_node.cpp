// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_node.hpp"

#include <chrono>
#include <utility>

#include "astrall_driver/fake_sdk_client.hpp"
#include "astrall_driver/sdk_client.hpp"
#include "astrall_driver/telemetry_publisher.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/create_publisher.hpp"

namespace astrall_driver {
namespace {

constexpr size_t kStateHistoryDepth = 1;

rcl_interfaces::msg::ParameterDescriptor ReadOnly(
  const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = description;
  return descriptor;
}

diagnostic_msgs::msg::DiagnosticStatus Diagnostic(const DriverSnapshot& state) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  const bool failed = state.faulted_ ||
      (state.last_result_ != ASTRALL_RES_SUCCESSED &&
      state.last_result_ != ASTRALL_RES_NOT_INIT) ||
      (state.zero_attempted_ && state.zero_result_ != ASTRALL_RES_SUCCESSED);
  status.level = failed ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (state.connected_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = std::string(DriverPhaseName(state.phase_)) + "; " +
      state.reason_ + "; " + SdkResult(state.last_result_).ToString();
  if (state.zero_attempted_) {
    status.message += "; zero ACK only=" +
        SdkResult(state.zero_result_).ToString();
  }
  auto add = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue field;
        field.key = key;
        field.value = value;
        status.values.push_back(std::move(field));
      };
  add("configured", std::to_string(state.configured_));
  add("operational_active", std::to_string(state.active_));
  add("connected", std::to_string(state.connected_));
  add("has_control_authority", std::to_string(state.has_control_authority_));
  add("release_pending_confirmation",
      std::to_string(state.authority_release_pending_));
  add("session_generation", std::to_string(state.session_generation_));
  add("heartbeat_attempts", std::to_string(state.heartbeat_count_));
  add("zero_attempted", std::to_string(state.zero_attempted_));
  add("zero_result", SdkResult(state.zero_result_).ToString());
  add("release_attempted", std::to_string(state.release_attempted_));
  add("release_result", SdkResult(state.release_result_).ToString());
  add("sdk_subscriptions_active", std::to_string(state.subscriptions_active_));
  add("telemetry_dropped", std::to_string(state.telemetry_dropped_));
  add("telemetry_clock_errors", std::to_string(state.telemetry_clock_errors_));
  add("telemetry_last_error", state.telemetry_last_error_);
  add("command_active", std::to_string(state.command_active_));
  add("motion_epoch", std::to_string(state.motion_epoch_));
  add("control_send_count", std::to_string(state.control_send_count_));
  add("watchdog_stops", std::to_string(state.watchdog_stops_));
  add("control_result", SdkResult(state.control_result_).ToString());
  add("control_reason", state.control_reason_);
  add("mode_pending", std::to_string(state.mode_pending_));
  add("mode_actual", std::to_string(state.mode_actual_));
  add("mode_result", SdkResult(state.mode_result_).ToString());
  add("mode_ack", SdkResult(state.mode_ack_).ToString());
  add("mode_ticket", std::to_string(state.mode_ticket_));
  add("mode_reason", state.mode_reason_);
  return status;
}

}  // namespace

DriverNode::DriverNode(const rclcpp::NodeOptions& options,
    std::shared_ptr<ISdkClient> client)
  : LifecycleNode("astrall_driver", options) {
  DeclareParameters();
  DeclareTelemetryParameters();
  DeclareControlParameters();
  DeclareLidarParameters();
  parameters_ = ReadParameters();
  if (!client) {
    client = parameters_.use_fake_sdk_ ?
        std::static_pointer_cast<ISdkClient>(std::make_shared<FakeSdkClient>())
          :
        std::static_pointer_cast<ISdkClient>(std::make_shared<SdkClient>());
  }
  const auto clock = get_clock();
  core_ = std::make_unique<DriverCore>(std::move(client), [clock] {
      return clock->now().nanoseconds();
    });
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(kStateHistoryDepth))
      .reliable().transient_local();
  // 普通 Publisher 保证 inactive/configure 失败时也有连接与失败诊断。
  connection_publisher_ = rclcpp::create_publisher<
    astrall_interfaces::msg::ConnectionStatus>(*this, "connection_status", qos);
  diagnostic_publisher_ = rclcpp::create_publisher<
    diagnostic_msgs::msg::DiagnosticArray>(*this, "driver_status", qos);
  authority_service_ =
      create_service<astrall_interfaces::srv::SetControlAuthority>(
    "set_control_authority",
    [this](AuthorityService::Request::SharedPtr request,
    AuthorityService::Response::SharedPtr response) {
      HandleAuthority(request, response);
    });
  state_timer_ = create_wall_timer(std::chrono::milliseconds(
            kHeartbeatPeriodMs),
          [this] {
      PublishState();
    });
  PublishState();
}

DriverNode::~DriverNode() {
  Stop();
}

void DriverNode::DeclareParameters() {
  const DriverParameters defaults;
  declare_parameter("use_fake_sdk",
      defaults.use_fake_sdk_,
      ReadOnly(
        "Default true; false explicitly enables native SDK on configure"));
  declare_parameter("robot_ip", defaults.robot_ip_,
      ReadOnly("Fixed SDK address; validation only, not a setter"));
  declare_parameter("local_ip",
      defaults.local_ip_,
      ReadOnly(
        "SDK host NIC requirement; validation only, not NIC configuration"));
  declare_parameter("sdk_port", defaults.sdk_port_,
      ReadOnly("Fixed SDK port 3600; validation only"));
  declare_parameter("sdk_client_port", defaults.sdk_client_port_,
      ReadOnly("Fixed SDK port 3601; validation only"));
  declare_parameter("init_timeout_ms", defaults.init_timeout_ms_,
      ReadOnly("Connection timeout before heartbeat starts, 1..5000 ms"));
  declare_parameter("sdk_timeout_ms", defaults.sdk_timeout_ms_,
      ReadOnly("Runtime SDK timeout 1..5 ms; native library must honor it"));
  declare_parameter("heartbeat_timeout_ms", defaults.heartbeat_timeout_ms_,
      ReadOnly("Heartbeat ACK timeout 1..20 ms; shares the SDK API lock"));
  declare_parameter("heartbeat_period_ms",
      defaults.heartbeat_period_ms_,
      ReadOnly("Fixed 100 ms / 10 Hz, manual 5.5"));
  declare_parameter("system_poll_period_ms",
      defaults.system_poll_period_ms_,
      ReadOnly("Safety-only system error polling, 100..500 ms"));
  declare_parameter("state_publish_period_ms",
      defaults.state_publish_period_ms_,
      ReadOnly("ROS state publication period, 50..1000 ms"));
}

DriverParameters DriverNode::ReadParameters() const {
  DriverParameters parameters;
  parameters.use_fake_sdk_ = get_parameter("use_fake_sdk").as_bool();
  parameters.robot_ip_ = get_parameter("robot_ip").as_string();
  parameters.local_ip_ = get_parameter("local_ip").as_string();
  parameters.sdk_port_ = get_parameter("sdk_port").as_int();
  parameters.sdk_client_port_ = get_parameter("sdk_client_port").as_int();
  parameters.init_timeout_ms_ = get_parameter("init_timeout_ms").as_int();
  parameters.sdk_timeout_ms_ = get_parameter("sdk_timeout_ms").as_int();
  parameters.heartbeat_timeout_ms_ =
      get_parameter("heartbeat_timeout_ms").as_int();
  parameters.heartbeat_period_ms_ =
      get_parameter("heartbeat_period_ms").as_int();
  parameters.system_poll_period_ms_ =
      get_parameter("system_poll_period_ms").as_int();
  parameters.state_publish_period_ms_ =
      get_parameter("state_publish_period_ms").as_int();
  ReadTelemetryParameters(&parameters);
  ReadControlParameters(&parameters);
  ReadLidarParameters(&parameters);
  return parameters;
}

DriverSnapshot DriverNode::DriverState() const {
  return core_->Snapshot();
}

void DriverNode::Stop() {
  StopLidar();
  ResetControlInterfaces();
  if (telemetry_timer_) {
    telemetry_timer_->cancel();
  }
  if (state_timer_) {
    state_timer_->cancel();
  }
  if (core_) {
    const bool closing = core_->Snapshot().phase_ != DriverPhase::kFinalized;
    core_->Shutdown();
    if (closing) {
      const auto state = core_->Snapshot();
      const auto message = "shutdown; zero attempted=" +
          std::to_string(state.zero_attempted_) + "; ACK only=" +
          SdkResult(state.zero_result_).ToString() + "; release attempted=" +
          std::to_string(state.release_attempted_) + "; release=" +
          SdkResult(state.release_result_).ToString() + "; " + state.reason_;
      if (state.faulted_) {
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
      }
    }
  }
}

auto DriverNode::TransitionResult(const SdkResult& result) -> CallbackReturn {
  PublishState();
  if (!result.Succeeded()) {
    RCLCPP_ERROR(get_logger(), "%s: %s", result.ToString().c_str(),
        core_->Snapshot().reason_.c_str());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_configure(
  const rclcpp_lifecycle::State&) {
  parameters_ = ReadParameters();
  if (!parameters_.Validate().empty()) {
    return TransitionResult(core_->Configure(parameters_));
  }
  try {
    telemetry_publisher_ = std::make_unique<TelemetryPublisher>(*this,
            parameters_);
    CreateLidarPublisher();
  } catch (const std::exception& error) {
    lidar_publisher_.reset();
    telemetry_publisher_.reset();
    RCLCPP_ERROR(get_logger(), "TELEMETRY_PUBLISHERS %s: %s",
        SdkResult(ASTRALL_RES_INVALID_PARAM).ToString().c_str(), error.what());
    return CallbackReturn::FAILURE;
  }
  auto result = core_->Configure(parameters_);
  if (result.Succeeded()) {
    CreateControlServices();
    state_timer_->cancel();
    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(parameters_.state_publish_period_ms_),
      [this] {
        PublishState();
      });
    telemetry_timer_ = create_wall_timer(
      std::chrono::milliseconds(parameters_.telemetry_publish_period_ms_),
      [this] {
        PublishTelemetry();
      });
  } else {
    lidar_publisher_.reset();
    telemetry_publisher_.reset();
  }
  return TransitionResult(result);
}

DriverNode::CallbackReturn DriverNode::on_activate(
  const rclcpp_lifecycle::State& previous) {
  auto result = core_->Activate();
  if (result.Succeeded() && !StartLidar()) {
    ResetControlInterfaces();
    const auto stopped = core_->Deactivate();
    return TransitionResult(stopped.Succeeded() ?
               SdkResult(ASTRALL_RES_FAILED) : stopped);
  }
  if (result.Succeeded()) {
    CreateControlServices();
    LifecycleNode::on_activate(previous);
    if (telemetry_timer_) {
      telemetry_timer_->reset();
    }
  }
  return TransitionResult(result);
}

DriverNode::CallbackReturn DriverNode::on_deactivate(
  const rclcpp_lifecycle::State& previous) {
  StopLidar();
  ResetControlInterfaces();
  if (telemetry_timer_) {
    telemetry_timer_->cancel();
  }
  auto result = core_->Deactivate();
  LifecycleNode::on_deactivate(previous);
  return TransitionResult(result);
}

DriverNode::CallbackReturn DriverNode::on_cleanup(
  const rclcpp_lifecycle::State&) {
  StopLidar();
  ResetControlInterfaces();
  if (telemetry_timer_) {
    telemetry_timer_->cancel();
    telemetry_timer_.reset();
  }
  auto result = core_->Cleanup();
  lidar_publisher_.reset();
  telemetry_publisher_.reset();
  return TransitionResult(result);
}

DriverNode::CallbackReturn DriverNode::on_shutdown(
  const rclcpp_lifecycle::State&) {
  StopLidar();
  ResetControlInterfaces();
  if (telemetry_timer_) {
    telemetry_timer_->cancel();
  }
  return TransitionResult(core_->Shutdown());
}

auto DriverNode::on_error(const rclcpp_lifecycle::State&) -> CallbackReturn {
  StopLidar();
  ResetControlInterfaces();
  if (telemetry_timer_) {
    telemetry_timer_->cancel();
  }
  auto result = core_->Cleanup();
  lidar_publisher_.reset();
  telemetry_publisher_.reset();
  return TransitionResult(result);
}

void DriverNode::HandleAuthority(
  const AuthorityService::Request::SharedPtr request,
  AuthorityService::Response::SharedPtr response) {
  const auto result =
      core_->RequestAuthority(static_cast<AstrallAuth>(request->authority));
  response->result.code = result.Value();
  response->result.success = result.Succeeded();
  response->result.message = result.ToString();
  PublishState();
}

void DriverNode::PublishState() {
  if (!get_node_base_interface()->get_context()->is_valid()) {
    return;
  }
  const auto snapshot = core_->Snapshot();
  UpdateCommandSubscription(snapshot);
  astrall_interfaces::msg::ConnectionStatus connection;
  connection.header.stamp = now();
  connection.connected = snapshot.connected_;
  connection.has_control_authority = snapshot.has_control_authority_;
  connection_publisher_->publish(connection);
  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = connection.header.stamp;
  auto status = Diagnostic(snapshot);
  AddLidarDiagnostics(&status);
  if (telemetry_publisher_) {
    diagnostic_msgs::msg::KeyValue invalid;
    invalid.key = "telemetry_invalid_samples";
    invalid.value = std::to_string(telemetry_publisher_->InvalidSamples());
    status.values.push_back(std::move(invalid));
  }
  status.name = get_node_base_interface()->get_fully_qualified_name();
  status.hardware_id = parameters_.use_fake_sdk_ ? "fake" : "astrall_sdk";
  if (status.message != last_log_) {
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      RCLCPP_ERROR(get_logger(), "%s", status.message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", status.message.c_str());
    }
    last_log_ = status.message;
  }
  diagnostics.status.push_back(std::move(status));
  diagnostic_publisher_->publish(diagnostics);
}

}  // namespace astrall_driver
