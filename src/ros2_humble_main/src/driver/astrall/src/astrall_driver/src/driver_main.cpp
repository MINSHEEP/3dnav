// Copyright 2026 Hypertron ROS 2 contributors

#include <exception>
#include <memory>

#include "astrall_driver/driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<astrall_driver::DriverNode>();
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    // rclcpp 信号处理关闭 ROS 上下文并唤醒 executor；
    // SDK 的关闭在普通线程中完成。
    node->Stop();
  } catch (const rclcpp::exceptions::RCLError& error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("astrall_driver"), "%s", error.what());
      result = 1;
    } else {
      // SIGINT 可在构造/等待 ROS 实体时关闭上下文；栈展开已安全关闭节点。
      RCLCPP_INFO(rclcpp::get_logger("astrall_driver"),
          "ROS shutdown interrupted startup/spin: %s", error.what());
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("astrall_driver"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
