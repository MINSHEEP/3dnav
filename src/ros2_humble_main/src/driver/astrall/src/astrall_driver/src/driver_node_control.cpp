// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/driver_node.hpp"

#include <cmath>
#include <limits>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace astrall_driver {

void DriverNode::DeclareControlParameters() {
  const DriverParameters defaults;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description =
      "S4 control startup configuration; see package README";
  declare_parameter("control_period_ms", defaults.control_period_ms_,
      descriptor);
  declare_parameter("control_sdk_timeout_ms",
      defaults.control_sdk_timeout_ms_,
      descriptor);
  declare_parameter("cmd_vel_timeout_ms",
      defaults.cmd_vel_timeout_ms_,
      descriptor);
  declare_parameter("max_forward_velocity",
      defaults.max_forward_velocity_,
      descriptor);
  declare_parameter("max_lateral_velocity",
      defaults.max_lateral_velocity_,
      descriptor);
  declare_parameter("max_yaw_velocity", defaults.max_yaw_velocity_, descriptor);
  declare_parameter("mode_switch_timeout_ms",
      defaults.mode_switch_timeout_ms_,
      descriptor);
  declare_parameter("mode_state_freshness_ms",
      defaults.mode_state_freshness_ms_,
      descriptor);
  declare_parameter("mode_poll_period_ms",
      defaults.mode_poll_period_ms_,
      descriptor);
  declare_parameter("allow_reboot_on_dock_stand",
      defaults.allow_reboot_on_dock_stand_,
      descriptor);
  declare_parameter("text_rate_hz", defaults.text_rate_hz_, descriptor);
  declare_parameter("cmd_vel_topic", defaults.cmd_vel_topic_, descriptor);
}

void DriverNode::ReadControlParameters(DriverParameters* parameters) const {
  parameters->control_period_ms_ = get_parameter("control_period_ms").as_int();
  parameters->control_sdk_timeout_ms_ =
      get_parameter("control_sdk_timeout_ms").as_int();
  parameters->cmd_vel_timeout_ms_ =
      get_parameter("cmd_vel_timeout_ms").as_int();
  parameters->max_forward_velocity_ =
      get_parameter("max_forward_velocity").as_double();
  parameters->max_lateral_velocity_ =
      get_parameter("max_lateral_velocity").as_double();
  parameters->max_yaw_velocity_ = get_parameter("max_yaw_velocity").as_double();
  parameters->mode_switch_timeout_ms_ =
      get_parameter("mode_switch_timeout_ms").as_int();
  parameters->mode_state_freshness_ms_ =
      get_parameter("mode_state_freshness_ms").as_int();
  parameters->mode_poll_period_ms_ =
      get_parameter("mode_poll_period_ms").as_int();
  parameters->allow_reboot_on_dock_stand_ = get_parameter(
    "allow_reboot_on_dock_stand").as_bool();
  parameters->text_rate_hz_ = get_parameter("text_rate_hz").as_double();
  parameters->cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
}

void DriverNode::CreateControlServices() {
  const auto generation = core_->Snapshot().session_generation_;
  if (services_generation_ == generation && mode_service_) {
    return;
  }
  ResetControlInterfaces();
  services_generation_ = generation;
  using Mode = astrall_interfaces::srv::SetMode;
  mode_service_ = create_service<Mode>("set_mode",
          [this, generation](Mode::Request::SharedPtr request,
          Mode::Response::SharedPtr response) {
      const auto result = core_->RequestMode(request->mode,
      request->confirm_reboot_on_dock, generation);
      const auto state = core_->Snapshot();
      response->result.code = result.Value();
      response->result.success = result.Succeeded();
      response->result.message = result.ToString();
      response->result.message += "; " + state.control_reason_;
      response->result.message += "; " + state.mode_reason_;
      response->sport_status = state.mode_actual_;
      PublishState();
    });
  using Light = astrall_interfaces::srv::SetLight;
  light_service_ = create_service<Light>("set_light",
          [this, generation](Light::Request::SharedPtr request,
          Light::Response::SharedPtr response) {
      const auto result = core_->RequestLight(request->command, generation);
      response->result.code = result.Value();
      response->result.success = result.Succeeded();
      response->result.message = result.ToString();
      response->result.message += "; " + core_->Snapshot().control_reason_;
      PublishState();
    });
  using Text = astrall_interfaces::srv::SendMessage;
  text_service_ = create_service<Text>("send_message",
          [this, generation](Text::Request::SharedPtr request,
          Text::Response::SharedPtr response) {
      const auto result = core_->RequestText(request->text, generation);
      response->result.code = result.Value();
      response->result.success = result.Succeeded();
      response->result.message = result.ToString();
      response->result.message += "; " + core_->Snapshot().control_reason_;
      PublishState();
    });
}

void DriverNode::ResetControlInterfaces() {
  command_subscription_.reset();
  mode_service_.reset();
  light_service_.reset();
  text_service_.reset();
  services_generation_ = 0;
  command_generation_ = 0;
  command_epoch_ = 0;
}

void DriverNode::UpdateCommandSubscription(const DriverSnapshot& state) {
  const bool allowed = state.active_ && state.connected_ &&
      state.has_control_authority_ &&
      !state.faulted_ && state.mode_can_move_ && !state.mode_pending_;
  if (!allowed) {
    command_subscription_.reset();
    return;
  }
  if (command_subscription_ &&
      command_generation_ == state.session_generation_ &&
      command_epoch_ == state.motion_epoch_) {
    return;
  }
  command_subscription_.reset();
  command_generation_ = state.session_generation_;
  command_epoch_ = state.motion_epoch_;
  const auto generation = command_generation_;
  const auto epoch = command_epoch_;
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    parameters_.cmd_vel_topic_, rclcpp::QoS(1).reliable(),
    [this, generation, epoch](
      const geometry_msgs::msg::Twist& twist) {
      VelocityCommand command;
      command.forward_ = twist.linear.x;
      command.lateral_ = twist.linear.y;
      command.yaw_ = twist.angular.z;
      if (!std::isfinite(twist.linear.z) || !std::isfinite(twist.angular.x) ||
      !std::isfinite(twist.angular.y)) {
        command.forward_ = std::numeric_limits<double>::quiet_NaN();
      }
      const auto result = core_->SubmitVelocity(command, generation, epoch);
      if (!result.Succeeded()) {
        RCLCPP_WARN(get_logger(), "CMD_VEL %s", result.ToString().c_str());
      }
    });
}

}  // namespace astrall_driver
