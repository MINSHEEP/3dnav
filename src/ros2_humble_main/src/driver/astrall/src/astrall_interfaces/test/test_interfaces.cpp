// Copyright 2026 Hypertron ROS 2 contributors

#include <cstdint>
#include <limits>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "astrall_interfaces/msg/connection_status.hpp"
#include "astrall_interfaces/msg/joystick_state.hpp"
#include "astrall_interfaces/msg/sport_status.hpp"
#include "astrall_interfaces/msg/system_status.hpp"
#include "astrall_interfaces/srv/send_message.hpp"
#include "astrall_interfaces/srv/set_control_authority.hpp"
#include "astrall_interfaces/srv/set_light.hpp"
#include "astrall_interfaces/srv/set_mode.hpp"
#include "gtest/gtest.h"

namespace astrall_interfaces {
namespace {

TEST(Interfaces, DefaultStateHasNoConnectionOrAuthority) {
  const msg::ConnectionStatus state;
  EXPECT_FALSE(state.connected);
  EXPECT_FALSE(state.has_control_authority);
  EXPECT_FALSE(srv::SetMode::Request().confirm_reboot_on_dock);
}

TEST(Interfaces, ProtocolConstantsPreserveValuesAndDomains) {
  EXPECT_EQ(msg::SportStatus::MOVE, 0xB104);
  EXPECT_EQ(msg::SportStatus::MOVE_TANK, 0xB114);
  EXPECT_EQ(srv::SetMode::Request::MOVE, 0xA104);
  EXPECT_EQ(srv::SetMode::Request::MOVE_TANK, 0xA108);
  EXPECT_EQ(srv::SetControlAuthority::Request::SDK, 1);
  EXPECT_EQ(srv::SetControlAuthority::Request::JOYSTICK, 2);
  EXPECT_EQ(srv::SetLight::Request::OPEN, 1);
  EXPECT_EQ(srv::SetLight::Request::CLOSE, 2);
}

TEST(Interfaces, SerializationRetainsUnknownBitsAndUnicode) {
  msg::SystemStatus original;
  original.error_code = std::numeric_limits<uint32_t>::max();
  original.warn_code = 1U << 31;
  original.error_messages = { "未知错误位" };
  rclcpp::Serialization<msg::SystemStatus> serializer;
  rclcpp::SerializedMessage bytes;
  serializer.serialize_message(&original, &bytes);
  msg::SystemStatus restored;
  serializer.deserialize_message(&bytes, &restored);
  EXPECT_EQ(restored.error_code, original.error_code);
  EXPECT_EQ(restored.warn_code, original.warn_code);
  EXPECT_EQ(restored.error_messages, original.error_messages);
}

TEST(Interfaces, RawJoystickAndTextAreNotTruncated) {
  msg::JoystickState joystick;
  joystick.rocker.front() = std::numeric_limits<int16_t>::min();
  joystick.rocker.back() = std::numeric_limits<int16_t>::max();
  EXPECT_EQ(joystick.rocker.size(), 6U);
  srv::SendMessage::Request request;
  request.text = "遥控器消息";
  EXPECT_GT(request.text.size(), 5U);
}

}  // namespace
}  // namespace astrall_interfaces
