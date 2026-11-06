// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <thread>

#include "astrall_driver/driver_node.hpp"
#include "astrall_driver/sdk_client.hpp"
#include "gtest/gtest.h"
#include "control_test_backend.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace astrall_driver {
namespace {

using namespace std::chrono_literals;
constexpr auto kWait = 3s;
using Authority = astrall_interfaces::srv::SetControlAuthority;
using Mode = astrall_interfaces::srv::SetMode;
using Light = astrall_interfaces::srv::SetLight;
using Text = astrall_interfaces::srv::SendMessage;
using Trigger = std_srvs::srv::Trigger;

bool SpinUntil(rclcpp::executors::SingleThreadedExecutor* executor,
    const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

void Authorize(rclcpp::executors::SingleThreadedExecutor* executor,
    const std::shared_ptr<DriverNode>& node,
    const rclcpp::Node::SharedPtr& observer,
    const std::shared_ptr<test::ControlBackend>& trace) {
  auto client = observer->create_client<Authority>("set_control_authority");
  ASSERT_TRUE(client->wait_for_service(kWait));
  auto request = std::make_shared<Authority::Request>();
  request->authority = Authority::Request::SDK;
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor->spin_until_future_complete(future, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->result.success);
  EXPECT_FALSE(node->DriverState().has_control_authority_);
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  trace->base_->config_.sdkStatusCb(&status, sizeof(status));
  ASSERT_TRUE(SpinUntil(executor, [&] {
        return node->DriverState().has_control_authority_;
      }));
}

class ControlRosTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

 protected:
  rclcpp::NodeOptions Options() {
    static std::atomic<uint64_t> count{ 0 };
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__ns:=/astrall_s4_" +
                        std::to_string(stamp) + "_" +
                        std::to_string(++count) });
    return options;
  }
};

TEST_F(ControlRosTest,
    TwistLimitsResendAndSteadyWatchdogSurviveBlockedExecutor) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto sdk = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  auto options = Options();
  options.parameter_overrides({ rclcpp::Parameter("use_sim_time", true),
                                rclcpp::Parameter("max_forward_velocity", 0.5),
                                rclcpp::Parameter("max_lateral_velocity", 0.6),
                                rclcpp::Parameter("max_yaw_velocity", 0.7) });
  auto node = std::make_shared<DriverNode>(options, sdk);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  auto publisher = observer->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    1);
  EXPECT_TRUE(trace->Moves().empty());
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  Authorize(&executor, node, observer, trace);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return publisher->get_subscription_count() > 0;
      }));
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 2;
  twist.linear.y = -3;
  twist.angular.z = 4;
  publisher->publish(twist);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return trace->Moves().size() >= 5;
      }));
  auto moves = trace->Moves();
  EXPECT_FLOAT_EQ(moves.front().command_.forward_, 0.5F);
  EXPECT_FLOAT_EQ(moves.front().command_.lateral_, -0.6F);
  EXPECT_FLOAT_EQ(moves.front().command_.yaw_, 0.7F);
  std::chrono::steady_clock::time_point start;
  std::chrono::steady_clock::time_point end;
  auto service = observer->create_service<Trigger>("slow_service",
          [&](Trigger::Request::SharedPtr,
          Trigger::Response::SharedPtr response) {
        start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(350ms);
        end = std::chrono::steady_clock::now();
        response->success = true;
      });
  auto client = observer->create_client<Trigger>("slow_service");
  ASSERT_TRUE(client->wait_for_service(kWait));
  auto future =
      client->async_send_request(std::make_shared<Trigger::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);
  EXPECT_GE(node->DriverState().watchdog_stops_, 1U);
  moves = trace->Moves();
  EXPECT_TRUE(std::any_of(moves.begin(), moves.end(), [&](const auto& entry) {
        return entry.time_ >= start && entry.time_ <= end &&
        entry.command_.forward_ == 0 && entry.command_.lateral_ == 0 &&
        entry.command_.yaw_ == 0;
      }));
  {
    std::lock_guard<std::mutex> lock(trace->mutex_);
    EXPECT_GE(std::count_if(trace->hearts_.begin(), trace->hearts_.end(),
        [&](auto time) {
          return time >= start && time <= end;
        }), 3);
  }
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return publisher->get_subscription_count() > 0;
      }));
  twist.linear.x = 0.2;
  twist.linear.y = 0;
  twist.angular.z = 0;
  publisher->publish(twist);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return !trace->Moves().empty() &&
        trace->Moves().back().command_.forward_ > 0;
      }));
  twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  publisher->publish(twist);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return !node->DriverState().command_active_ &&
        trace->Moves().back().command_.forward_ == 0;
      }));
}

TEST_F(ControlRosTest, ActualModeServiceRunningPreemptionLightAndUnicodeText) {
  auto trace = std::make_shared<test::ControlBackend>();
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDDOWN;
  trace->mode_result_ = ASTRALL_RES_RUNNING;
  auto sdk = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  auto node = std::make_shared<DriverNode>(Options(), sdk);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  Authorize(&executor, node, observer, trace);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return node->DriverState().mode_actual_ ==
        ASTRALL_SPORT_STATUS_FIXEDDOWN;
      }));
  auto mode = observer->create_client<Mode>("set_mode");
  ASSERT_TRUE(mode->wait_for_service(kWait));
  auto request = std::make_shared<Mode::Request>();
  request->mode = Mode::Request::FIXEDSTAND;
  auto standing = mode->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(standing, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  auto response = standing.get();
  EXPECT_EQ(response->result.code, ASTRALL_RES_RUNNING);
  EXPECT_FALSE(response->result.success);
  EXPECT_TRUE(node->DriverState().mode_pending_);
  request = std::make_shared<Mode::Request>();
  request->mode = Mode::Request::DAMPING;
  auto damping = mode->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(damping, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(damping.get()->result.code, ASTRALL_RES_RUNNING);
  trace->actual_ = ASTRALL_SPORT_STATUS_DAMPING;
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return !node->DriverState().mode_pending_;
      }));
  EXPECT_EQ(node->DriverState().mode_result_, ASTRALL_RES_SUCCESSED);
  request = std::make_shared<Mode::Request>();
  request->mode = Mode::Request::FIXEDSTAND;
  auto restanding = mode->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(restanding, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(restanding.get()->result.code, ASTRALL_RES_RUNNING);
  trace->actual_ = ASTRALL_SPORT_STATUS_FIXEDSTAND;
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return !node->DriverState().mode_pending_;
      }));
  request = std::make_shared<Mode::Request>();
  request->mode = Mode::Request::MOVE;
  auto moving = mode->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(moving, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(moving.get()->result.code, ASTRALL_RES_RUNNING);
  trace->actual_ = ASTRALL_SPORT_STATUS_MOVE;
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return node->DriverState().mode_can_move_;
      }));
  request = std::make_shared<Mode::Request>();
  request->mode = ASTRALL_SPORT_MODE_CMD_AUTOCHARGE;
  auto unsupported = mode->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(unsupported, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(unsupported.get()->result.code, ASTRALL_RES_INVALID_PARAM);
  auto light = observer->create_client<Light>("set_light");
  ASSERT_TRUE(light->wait_for_service(kWait));
  auto lamp = std::make_shared<Light::Request>();
  lamp->command = Light::Request::OPEN;
  auto lit = light->async_send_request(lamp);
  ASSERT_EQ(executor.spin_until_future_complete(lit, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(lit.get()->result.success);
  auto text = observer->create_client<Text>("send_message");
  ASSERT_TRUE(text->wait_for_service(kWait));
  auto message = std::make_shared<Text::Request>();
  message->text = "机器人状态正常";
  auto sent = text->async_send_request(message);
  ASSERT_EQ(executor.spin_until_future_complete(sent, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(sent.get()->result.success);
  auto rejected = text->async_send_request(message);
  ASSERT_EQ(executor.spin_until_future_complete(rejected, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(rejected.get()->result.success);
  ASSERT_EQ(trace->Texts().size(), 1U);
  EXPECT_EQ(trace->Texts()[0].text_, "机器人状态正常");
}

TEST_F(ControlRosTest, MovingNodeDestructionZerosAndNewNodeDoesNotReplay) {
  auto trace = std::make_shared<test::ControlBackend>();
  auto sdk = std::make_shared<SdkClient>(test::ControlFunctions(trace));
  auto node = std::make_shared<DriverNode>(Options(), sdk);
  const auto options = node->get_node_options();
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  Authorize(&executor, node, observer, trace);
  auto publisher = observer->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    1);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return publisher->get_subscription_count() > 0;
      }));
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.4;
  publisher->publish(command);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        const auto moves = trace->Moves();
        return !moves.empty() && moves.back().command_.forward_ > 0;
      }));
  executor.remove_node(node->get_node_base_interface());
  node.reset();
  const auto stopped = trace->Moves();
  ASSERT_FALSE(stopped.empty());
  EXPECT_DOUBLE_EQ(stopped.back().command_.forward_, 0);
  auto next = std::make_shared<DriverNode>(options, sdk);
  executor.add_node(next->get_node_base_interface());
  ASSERT_EQ(next->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(next->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  Authorize(&executor, next, observer, trace);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return publisher->get_subscription_count() > 0;
      }));
  const auto count = trace->Moves().size();
  const auto deadline = std::chrono::steady_clock::now() + 150ms;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(trace->Moves().size(), count);
  EXPECT_FALSE(next->DriverState().command_active_);
}

}  // namespace
}  // namespace astrall_driver
