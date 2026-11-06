// Copyright 2026 Hypertron ROS 2 contributors

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "astrall_driver/driver_node.hpp"
#include "astrall_driver/fake_sdk_client.hpp"
#include "astrall_driver/telemetry_conversions.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"

namespace astrall_driver {
namespace {

constexpr auto kWait = std::chrono::seconds(3);

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

void SpinFor(rclcpp::executors::SingleThreadedExecutor* executor,
    std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

class TelemetryRosTest : public ::testing::Test {
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
    options.arguments({ "--ros-args", "-r", "__ns:=/astrall_s3_" +
                        std::to_string(stamp) + "_" +
                        std::to_string(++count) });
    options.parameter_overrides({ rclcpp::Parameter("telemetry_query_period_ms",
                                  100) });
    return options;
  }
};

TEST_F(TelemetryRosTest,
    SubscribersReceiveAllTypesUnitsAndRosReceiveTimeWithoutAuthority) {
  auto fake = std::make_shared<FakeSdkClient>();
  AstrallPowerStatus power;
  power.soc = 73;
  power.temp = 26;
  power.voltage = 48;
  power.cycleCount = 17;
  power.charged = 2;
  fake->SetPowerStatus(power);
  AstrallDeviceInfo device;
  device.version = "test_version";
  device.serialNumber = "fake_serial";
  device.model = 99;
  fake->SetDeviceInfo(device);
  fake->SetSportStatus(ASTRALL_SPORT_STATUS_MOVE);
  AstrallSystemStatus system;
  system.sysStatus = ASTRALL_SYSTEM_CODE_RUNNING;
  system.warnCode = static_cast<AstrallWarnCode>(0x102);
  fake->SetSystemStatus(system);
  auto options = Options();
  options.parameter_overrides({
        rclcpp::Parameter("telemetry_query_period_ms", 100),
        rclcpp::Parameter("imu_topic", "custom_imu"),
        rclcpp::Parameter("imu_frame_id", "imu_body"),
        rclcpp::Parameter("base_frame_id", "robot_base"),
        rclcpp::Parameter("use_sim_time", true) });
  auto node = std::make_shared<DriverNode>(options, fake);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  std::optional<sensor_msgs::msg::Imu> imu;
  std::optional<astrall_interfaces::msg::WheelSpeeds> wheels;
  std::optional<astrall_interfaces::msg::JoystickState> joystick;
  std::optional<astrall_interfaces::msg::SystemStatus> status;
  std::optional<astrall_interfaces::msg::PowerStatus> battery;
  std::optional<astrall_interfaces::msg::SportStatus> mode;
  std::optional<astrall_interfaces::msg::DeviceInfo> info;
  auto imu_sub = observer->create_subscription<sensor_msgs::msg::Imu>(
    "custom_imu",
    rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Imu& message) {
        imu = message;
      });
  auto wheel_sub =
      observer->create_subscription<astrall_interfaces::msg::WheelSpeeds>(
    "wheel_speeds", rclcpp::SensorDataQoS(),
    [&](const astrall_interfaces::msg::WheelSpeeds& message) {
        wheels = message;
      });
  auto joy_sub =
      observer->create_subscription<astrall_interfaces::msg::JoystickState>(
    "joystick", rclcpp::SensorDataQoS(),
    [&](const astrall_interfaces::msg::JoystickState& message) {
        joystick = message;
      });
  auto sys_sub =
      observer->create_subscription<astrall_interfaces::msg::SystemStatus>(
    "system_status",
    1,
    [&](const astrall_interfaces::msg::SystemStatus& message) {
        status = message;
      });
  auto power_sub =
      observer->create_subscription<astrall_interfaces::msg::PowerStatus>(
    "power_status",
    1,
    [&](const astrall_interfaces::msg::PowerStatus& message) {
        battery = message;
      });
  auto mode_sub =
      observer->create_subscription<astrall_interfaces::msg::SportStatus>(
    "sport_status",
    1,
    [&](const astrall_interfaces::msg::SportStatus& message) {
        mode = message;
      });
  auto device_sub =
      observer->create_subscription<astrall_interfaces::msg::DeviceInfo>(
    "device_info", 1, [&](const astrall_interfaces::msg::DeviceInfo& message) {
        info = message;
      });
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  AstrallImuData input;
  input.timestamp = 999999999;
  input.quaternion[0] = 2;
  input.quaternion[3] = 2;
  input.accelerometer[0] = 1.25F;
  input.gyroscope[1] = 0.3F;
  AstrallSportData wheel;
  wheel.timestamp = -123;
  wheel.wheelSpeed[0] = 12.5F;
  AstrallJoystickData joy{};
  joy.timestamp = 456;
  joy.rocker[0] = -32768;
  joy.l3 = 1;
  joy.sw = 3;
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        if (node->DriverState().subscriptions_active_ == 3) {
          fake->EmitImu(input);
          fake->EmitSport(wheel);
          fake->EmitJoystick(joy);
        }
        return imu && wheels && joystick && status && battery && mode && info;
      }));
  EXPECT_FALSE(node->DriverState().has_control_authority_);
  EXPECT_EQ(imu->header.stamp.sec, 0);
  EXPECT_EQ(imu->header.stamp.nanosec, 0U);
  EXPECT_EQ(imu->header.frame_id, "imu_body");
  EXPECT_NEAR(imu->orientation.z, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(imu->orientation.w, std::sqrt(0.5), 1e-6);
  EXPECT_FLOAT_EQ(imu->linear_acceleration.x, 1.25F);
  EXPECT_FLOAT_EQ(imu->angular_velocity.y, 0.3F);
  EXPECT_EQ(wheels->header.frame_id, "robot_base");
  EXPECT_FLOAT_EQ(wheels->wheel_speed[0], 12.5F);
  EXPECT_EQ(wheels->sdk_timestamp, -123);
  EXPECT_EQ(joystick->rocker[0], -32768);
  EXPECT_EQ(joystick->sdk_timestamp, 456);
  EXPECT_TRUE(joystick->l3);
  EXPECT_EQ(joystick->switch_position, 3);
  EXPECT_FLOAT_EQ(battery->percentage, 0.73F);
  EXPECT_FLOAT_EQ(battery->voltage, 48);
  EXPECT_EQ(battery->cycle_count, 17);
  EXPECT_EQ(status->warn_code, 0x102U);
  EXPECT_EQ(status->warn_messages.size(), 2U);
  EXPECT_EQ(mode->status, 0xB104);
  EXPECT_EQ(info->version, "test_version");
  EXPECT_EQ(info->serial_number, "fake_serial");
  EXPECT_EQ(info->model, 99U);
  // 暂停 ROS 时钟不会影响稳态心跳，数据接收时间仍真实采用 ROS 的零时刻。
  EXPECT_GT(node->DriverState().heartbeat_count_, 1U);
  using Authority = astrall_interfaces::srv::SetControlAuthority;
  auto authority = observer->create_client<Authority>("set_control_authority");
  ASSERT_TRUE(authority->wait_for_service(kWait));
  auto request = std::make_shared<Authority::Request>();
  request->authority = Authority::Request::SDK;
  auto future = authority->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, kWait),
      rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->result.success);
  AstrallSdkStatus connection;
  connection.link = 1;
  connection.ctrlAuthority = 1;
  fake->EmitStatus(connection);
  ASSERT_TRUE(fake->WaitForCallbacks(1000));
  ASSERT_TRUE(node->DriverState().has_control_authority_);
  connection.ctrlAuthority = 0;
  fake->EmitStatus(connection);
  ASSERT_TRUE(fake->WaitForCallbacks(1000));
  EXPECT_FALSE(node->DriverState().active_);
  EXPECT_FALSE(node->DriverState().has_control_authority_);
  imu.reset();
  input.accelerometer[0] = 42;
  fake->EmitImu(input);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return imu && imu->linear_acceleration.x == 42 &&
        node->DriverState().zero_attempted_;
      }));
}

TEST_F(TelemetryRosTest,
    InvalidPacketsFailedQueriesAndOldSessionsDoNotPublishCachedValues) {
  auto fake = std::make_shared<FakeSdkClient>();
  AstrallPowerStatus power;
  power.soc = 60;
  fake->SetPowerStatus(power);
  auto node = std::make_shared<DriverNode>(Options(), fake);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  size_t imu_count = 0;
  size_t power_count = 0;
  auto imu_sub = observer->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data",
    rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Imu&) {
        ++imu_count;
      });
  auto power_sub =
      observer->create_subscription<astrall_interfaces::msg::PowerStatus>(
    "power_status", 1, [&](const astrall_interfaces::msg::PowerStatus&) {
        ++power_count;
      });
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return node->DriverState().subscriptions_active_ == 3 &&
        imu_sub->get_publisher_count() > 0 && power_count > 0;
      }));
  AstrallImuData imu;
  imu.quaternion[0] = 1;
  ASSERT_TRUE(fake->EmitImu(imu));
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return imu_count > 0;
      }));
  const auto valid_count = imu_count;
  auto old = fake->RawSubscriptionCallback(ASTRALL_SUB_TOPIC_ID_IMU);
  imu.gyroscope[0] = std::numeric_limits<float>::quiet_NaN();
  fake->EmitImu(imu);
  fake->EmitRawData(ASTRALL_SUB_TOPIC_ID_IMU, nullptr, sizeof(imu));
  imu = {};
  fake->EmitImu(imu);
  ASSERT_TRUE(fake->WaitForCallbacks(1000));
  SpinFor(&executor, std::chrono::milliseconds(150));
  EXPECT_EQ(imu_count, valid_count);
  fake->SetResult(SdkOperation::kGetPowerStatus, ASTRALL_RES_TIMEOUT);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return node->DriverState().telemetry_last_error_.find("TELEMETRY_QUERY")
        !=
        std::string::npos;
      }));
  SpinFor(&executor, std::chrono::milliseconds(100));
  const auto stopped_power = power_count;
  SpinFor(&executor, std::chrono::milliseconds(350));
  EXPECT_EQ(power_count, stopped_power);
  node->deactivate();
  old(&imu, sizeof(imu));
  SpinFor(&executor, std::chrono::milliseconds(100));
  EXPECT_EQ(imu_count, valid_count);
  EXPECT_EQ(node->DriverState().subscriptions_active_, 0U);
  fake->SetResult(SdkOperation::kGetPowerStatus, ASTRALL_RES_SUCCESSED);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return node->DriverState().subscriptions_active_ == 3;
      }));
  old(&imu, sizeof(imu));
  SpinFor(&executor, std::chrono::milliseconds(100));
  EXPECT_EQ(imu_count, valid_count);
  imu.quaternion[0] = 1;
  imu.gyroscope[0] = 0;
  fake->EmitImu(imu);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return imu_count > valid_count;
      }));
}

TEST_F(TelemetryRosTest, SystemErrorsArePublishedAndStillCauseSafetyZero) {
  auto fake = std::make_shared<FakeSdkClient>();
  auto node = std::make_shared<DriverNode>(Options(), fake);
  auto observer = std::make_shared<rclcpp::Node>("observer",
          node->get_namespace());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  std::optional<astrall_interfaces::msg::SystemStatus> received;
  auto subscription =
      observer->create_subscription<astrall_interfaces::msg::SystemStatus>(
    "system_status",
    1,
    [&](const astrall_interfaces::msg::SystemStatus& message) {
        received = message;
      });
  ASSERT_EQ(node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  AstrallSystemStatus status;
  status.sysStatus = ASTRALL_SYSTEM_CODE_ERROR;
  status.errorCode = ASTRALL_ERR_STO_TRIGGERED;
  fake->SetSystemStatus(status);
  ASSERT_TRUE(SpinUntil(&executor, [&] {
        return received && received->error_code == ASTRALL_ERR_STO_TRIGGERED &&
        node->DriverState().zero_attempted_;
      }));
  EXPECT_FALSE(node->DriverState().active_);
  EXPECT_FALSE(node->DriverState().has_control_authority_);
  ASSERT_EQ(received->error_messages.size(), 1U);
  EXPECT_NE(received->error_messages[0].find("STO_TRIGGERED"),
      std::string::npos);
}

}  // namespace
}  // namespace astrall_driver
