// Copyright 2026 Hypertron ROS 2 contributors

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "astrall_driver/driver_node.hpp"
#include "astrall_driver/fake_sdk_client.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "sdk_test_backend.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace astrall_driver {
namespace {

constexpr auto kWait = std::chrono::seconds(3);
using ChangeState = lifecycle_msgs::srv::ChangeState;
using Authority = astrall_interfaces::srv::SetControlAuthority;
using Trigger = std_srvs::srv::Trigger;

bool SpinUntil(rclcpp::executors::SingleThreadedExecutor* executor,
    const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

bool Change(rclcpp::executors::SingleThreadedExecutor* executor,
    const rclcpp::Client<ChangeState>::SharedPtr& client, uint8_t id) {
  if (!client->wait_for_service(kWait)) {
    return false;
  }
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = id;
  auto future = client->async_send_request(request);
  return executor->spin_until_future_complete(future, kWait) ==
         rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
}

/** @brief CTest 限制 localhost/domain，所有节点另加唯一命名空间。 */
class DriverNodeTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

 protected:
  rclcpp::NodeOptions Options() {
    static std::atomic<uint64_t> counter{ 0 };
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    const auto name = "/astrall_s2_" + std::to_string(stamp) + "_" +
        std::to_string(++counter);
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__ns:=" + name });
    return options;
  }
};

TEST_F(DriverNodeTest, HeartbeatTimeoutOverrideReachesConfiguredCore) {
  auto fake = std::make_shared<FakeSdkClient>();
  auto options = Options();
  options.parameter_overrides({ rclcpp::Parameter("heartbeat_timeout_ms",
                                20) });
  auto node = std::make_shared<DriverNode>(options, fake);
  node->configure();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        const auto calls = fake->Calls();
        return std::any_of(calls.begin(), calls.end(), [](const auto& call) {
          return call.operation_ == SdkOperation::kHeartbeat &&
          call.timeout_ms_ == 20;
        });
      }));
  EXPECT_FALSE(node->set_parameter(
        rclcpp::Parameter("heartbeat_timeout_ms", 5)).successful);
}

TEST_F(DriverNodeTest, RealLifecycleServicesAndStateConfirmAuthority) {
  EXPECT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
  auto fake = std::make_shared<FakeSdkClient>();
  auto node = std::make_shared<DriverNode>(Options(), fake);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  std::atomic<bool> held{ false };
  auto subscription = observer->create_subscription<
    astrall_interfaces::msg::ConnectionStatus>("connection_status",
          rclcpp::QoS(1).reliable().transient_local(),
          [&](const astrall_interfaces::msg::ConnectionStatus& message) {
        held = message.has_control_authority;
      });
  auto lifecycle = observer->create_client<ChangeState>(
    std::string(
      node->get_node_base_interface()->get_fully_qualified_name()) +
    "/change_state");
  EXPECT_EQ(node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_TRUE(Change(&executor, lifecycle,
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  ASSERT_TRUE(Change(&executor, lifecycle,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  EXPECT_FALSE(node->DriverState().has_control_authority_);
  auto authority = observer->create_client<Authority>("set_control_authority");
  ASSERT_TRUE(authority->wait_for_service(kWait));
  auto request = std::make_shared<Authority::Request>();
  request->authority = Authority::Request::SDK;
  auto future = authority->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->result.success);
  EXPECT_FALSE(node->DriverState().has_control_authority_);
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  ASSERT_TRUE(fake->EmitStatus(status));
  ASSERT_TRUE(fake->WaitForCallbacks(1000));
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return held.load();
      }));
  ASSERT_TRUE(Change(&executor, lifecycle,
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return !held.load();
      }));
  EXPECT_TRUE(node->DriverState().zero_attempted_);
  EXPECT_TRUE(node->DriverState().release_attempted_);
  ASSERT_TRUE(Change(&executor, lifecycle,
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
  ASSERT_TRUE(Change(&executor, lifecycle,
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN));
  EXPECT_EQ(node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(DriverNodeTest, InvalidRangeTypeAndReadonlyParametersAreRejected) {
  auto options = Options();
  options.parameter_overrides({ rclcpp::Parameter("sdk_timeout_ms", 21) });
  auto fake = std::make_shared<FakeSdkClient>();
  auto node = std::make_shared<DriverNode>(options, fake);
  EXPECT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(fake->Calls().empty());
  EXPECT_FALSE(node->set_parameter(
        rclcpp::Parameter("sdk_timeout_ms", 10)).successful);
  options = Options();
  options.parameter_overrides({ rclcpp::Parameter("sdk_timeout_ms",
                                "bad_type") });
  EXPECT_THROW(std::make_shared<DriverNode>(options, fake),
      rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(DriverNodeTest, DefaultFactoryAndDestructionUseFakeAndSendZero) {
  {
    auto node = std::make_shared<DriverNode>(Options());
    EXPECT_TRUE(node->get_parameter("use_fake_sdk").as_bool());
    EXPECT_EQ(node->configure().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node->activate().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
  auto fake = std::make_shared<FakeSdkClient>();
  {
    auto node = std::make_shared<DriverNode>(Options(), fake);
    ASSERT_EQ(node->configure().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
  const auto calls = fake->Calls();
  ASSERT_FALSE(calls.empty());
  EXPECT_EQ(calls.back().operation_, SdkOperation::kDeinit);
  EXPECT_TRUE(std::any_of(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kMove &&
        call.velocity_[0] == 0 &&
        call.velocity_[1] == 0 && call.velocity_[2] == 0;
      }));
}

TEST_F(DriverNodeTest, ActiveDestructionZerosAndReleasesConfirmedAuthority) {
  auto fake = std::make_shared<FakeSdkClient>();
  auto node = std::make_shared<DriverNode>(Options(), fake);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto client = observer->create_client<Authority>("set_control_authority");
  ASSERT_TRUE(client->wait_for_service(kWait));
  auto request = std::make_shared<Authority::Request>();
  request->authority = Authority::Request::SDK;
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->result.success);
  AstrallSdkStatus status;
  status.link = 1;
  status.ctrlAuthority = 1;
  ASSERT_TRUE(fake->EmitStatus(status));
  ASSERT_TRUE(fake->WaitForCallbacks(1000));
  ASSERT_TRUE(node->DriverState().has_control_authority_);
  executor.remove_node(node->get_node_base_interface());
  node.reset();
  const auto calls = fake->Calls();
  ASSERT_FALSE(calls.empty());
  EXPECT_EQ(calls.back().operation_, SdkOperation::kDeinit);
  EXPECT_TRUE(std::any_of(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kMove &&
        call.velocity_[0] == 0 &&
        call.velocity_[1] == 0 && call.velocity_[2] == 0;
      }));
  EXPECT_TRUE(std::any_of(calls.begin(), calls.end(), [](const auto& call) {
        return call.operation_ == SdkOperation::kAuthControl &&
        call.argument_ == ASTRALL_AUTH_JOYSTICK;
      }));
}

struct HeartbeatTrace {
  std::mutex mutex_;
  std::vector<std::chrono::steady_clock::time_point> times_;

  auto Times() -> std::vector<std::chrono::steady_clock::time_point> {
    std::lock_guard<std::mutex> lock(mutex_);
    return times_;
  }
};

TEST_F(DriverNodeTest, SlowRosServiceDoesNotBlockActualTenHzSdkHeartbeat) {
  auto trace = std::make_shared<HeartbeatTrace>();
  auto functions = test::Functions(std::make_shared<test::Backend>());
  functions.heartbeat_ = [trace](uint32_t timeout) {
        EXPECT_LE(timeout, 20U);
        std::lock_guard<std::mutex> lock(trace->mutex_);
        trace->times_.push_back(std::chrono::steady_clock::now());
        return ASTRALL_RES_SUCCESSED;
      };
  auto sdk = std::make_shared<SdkClient>(functions);
  auto node = std::make_shared<DriverNode>(Options(), sdk);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  std::chrono::steady_clock::time_point start;
  std::chrono::steady_clock::time_point end;
  auto service = observer->create_service<std_srvs::srv::Trigger>(
    "slow_service",
    [&](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        end = std::chrono::steady_clock::now();
        response->success = true;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return trace->Times().size() >= 3;
      }));
  auto client = observer->create_client<std_srvs::srv::Trigger>("slow_service");
  ASSERT_TRUE(client->wait_for_service(kWait));
  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return trace->Times().size() >= 8;
      }));
  node->Stop();
  const auto times = trace->Times();
  const auto during = std::count_if(times.begin(), times.end(), [&](auto time) {
        return time >= start && time <= end;
      });
  EXPECT_GE(during, 3);
  std::cout << "Heartbeat intervals (ms):";
  for (size_t index = 1; index < times.size(); ++index) {
    const auto interval = std::chrono::duration<double, std::milli>(
      times[index] - times[index - 1]).count();
    std::cout << ' ' << interval;
    EXPECT_GE(interval, 60.0);
    EXPECT_LE(interval, 180.0);
  }
  const auto mean = std::chrono::duration<double, std::milli>(
    times.back() - times.front()).count() / (times.size() - 1);
  EXPECT_NEAR(mean, 100.0, 10.0);
  const auto blocked_ms = std::chrono::duration<double, std::milli>(end - start)
      .count();
  std::cout << "; mean=" << mean << "; blocked executor ms=" << blocked_ms
            << "; SDK calls during blocked executor=" << during << '\n';
}

}  // namespace
}  // namespace astrall_driver
