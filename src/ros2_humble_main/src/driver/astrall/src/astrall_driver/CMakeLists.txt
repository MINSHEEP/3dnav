cmake_minimum_required(VERSION 3.16)
project(astrall_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(astrall_sdk_vendor REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(astrall_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(astrall_sdk_client SHARED
  src/sdk_result.cpp
  src/sdk_functions.cpp
  src/sdk_callback_dispatcher.cpp
  src/sdk_client.cpp
  src/sdk_client_commands.cpp
  src/fake_sdk_client.cpp
  src/fake_sdk_functions.cpp
)
add_library(astrall_driver_core SHARED
  src/lidar_parameters.cpp
  src/udp_receiver.cpp
  src/lidar_packet_parser.cpp src/point_cloud_assembler.cpp
  src/velocity_controller.cpp src/mode_state_machine.cpp src/text_rate_limiter.cpp
  src/control_parameters.cpp
  src/driver_core_control.cpp
  src/fault_bits.cpp
  src/subscription_frequency.cpp
  src/telemetry_parameters.cpp
  src/driver_core_telemetry.cpp
  src/driver_parameters.cpp
  src/driver_core.cpp
  src/driver_core_worker.cpp
  src/driver_core_authority.cpp
)
target_include_directories(astrall_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(astrall_driver_core PUBLIC astrall_sdk_client)
target_compile_options(astrall_driver_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(astrall_sdk_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(astrall_sdk_client PUBLIC astrall_sdk::sdk Threads::Threads)
target_compile_options(astrall_sdk_client PRIVATE -Wall -Wextra -Wpedantic)
add_library(astrall_driver_ros SHARED
  src/lidar_publisher.cpp src/driver_node_lidar.cpp
  src/lidar_conversions.cpp
  src/driver_node.cpp src/driver_node_telemetry.cpp
  src/driver_node_control.cpp
  src/telemetry_conversions.cpp src/telemetry_publisher.cpp)
target_link_libraries(astrall_driver_ros PUBLIC astrall_driver_core)
ament_target_dependencies(astrall_driver_ros PUBLIC
  rclcpp rclcpp_lifecycle rcl_interfaces diagnostic_msgs astrall_interfaces sensor_msgs geometry_msgs nav_msgs tf2_ros)
target_compile_options(astrall_driver_ros PRIVATE -Wall -Wextra -Wpedantic)
add_executable(astrall_driver_node src/driver_main.cpp)
target_link_libraries(astrall_driver_node PRIVATE astrall_driver_ros)
ament_target_dependencies(astrall_driver_node PUBLIC rclcpp)
install(TARGETS astrall_driver_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS astrall_sdk_client astrall_driver_core astrall_driver_ros
  EXPORT astrall_driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(astrall_driverTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(astrall_sdk_vendor Threads)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle rcl_interfaces diagnostic_msgs astrall_interfaces sensor_msgs geometry_msgs nav_msgs tf2_ros)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/astrall_testing.cmake")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_conversions test/test_lidar_conversions.cpp)
  target_link_libraries(test_lidar_conversions astrall_driver_ros)
  ament_add_gtest(test_udp_receiver test/test_udp_receiver.cpp TIMEOUT 10)
  target_link_libraries(test_udp_receiver astrall_driver_core)
  ament_add_gtest(test_lidar_packets test/test_lidar_packets.cpp)
  target_link_libraries(test_lidar_packets astrall_driver_core)
  ament_add_gtest(test_control_policies test/test_control_policies.cpp)
  target_link_libraries(test_control_policies astrall_driver_core)
  ament_add_gtest(test_core_control test/test_core_control.cpp TIMEOUT 30)
  target_link_libraries(test_core_control astrall_driver_core)
  ament_add_gtest(test_mode_dispatch_gate
    test/test_mode_dispatch_gate.cpp TIMEOUT 15)
  target_link_libraries(test_mode_dispatch_gate astrall_driver_core)
  ament_add_gtest(test_telemetry_conversions test/test_telemetry_conversions.cpp)
  target_link_libraries(test_telemetry_conversions astrall_driver_ros)
  ament_add_gtest(test_fault_bits test/test_fault_bits.cpp)
  target_link_libraries(test_fault_bits astrall_driver_core)
  find_package(lifecycle_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  set(ros_test_log_dir "${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  file(MAKE_DIRECTORY "${ros_test_log_dir}")
  find_package(tf2_msgs REQUIRED)
  ament_add_gtest(test_control_ros test/test_control_ros.cpp TIMEOUT 30)
  target_link_libraries(test_control_ros astrall_driver_ros)
  ament_target_dependencies(test_control_ros lifecycle_msgs std_srvs)
  set_tests_properties(test_control_ros PROPERTIES
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=91;ROS_LOG_DIR=${ros_test_log_dir}")
  ament_add_gtest(test_telemetry_ros test/test_telemetry_ros.cpp TIMEOUT 30)
  target_link_libraries(test_telemetry_ros astrall_driver_ros)
  ament_target_dependencies(test_telemetry_ros lifecycle_msgs)
  set_tests_properties(test_telemetry_ros PROPERTIES
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=91;ROS_LOG_DIR=${ros_test_log_dir}")
  ament_add_gtest(test_driver_node test/test_driver_node.cpp TIMEOUT 30)
  target_link_libraries(test_driver_node astrall_driver_ros)
  ament_target_dependencies(test_driver_node lifecycle_msgs std_srvs)
  set_tests_properties(test_driver_node PROPERTIES
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=91;ROS_LOG_DIR=${ros_test_log_dir}")
  ament_add_gtest(test_driver_parameters test/test_driver_parameters.cpp)
  target_link_libraries(test_driver_parameters astrall_driver_core)
  ament_add_gtest(test_driver_core test/test_driver_core.cpp TIMEOUT 20)
  target_link_libraries(test_driver_core astrall_driver_core)
  ament_add_gtest(test_core_telemetry test/test_core_telemetry.cpp TIMEOUT 30)
  target_link_libraries(test_core_telemetry astrall_driver_core)
  ament_add_gtest(test_sdk_result test/test_sdk_result.cpp)
  target_link_libraries(test_sdk_result astrall_sdk_client)
  ament_add_gtest(test_sdk_client test/test_sdk_client.cpp)
  target_link_libraries(test_sdk_client astrall_sdk_client)
  ament_add_gtest(test_sdk_mode_guard test/test_sdk_mode_guard.cpp TIMEOUT 10)
  target_link_libraries(test_sdk_mode_guard astrall_sdk_client)
  ament_add_gtest(test_sdk_callbacks test/test_sdk_callbacks.cpp)
  target_link_libraries(test_sdk_callbacks astrall_sdk_client)
  ament_add_gtest(test_fake_sdk_client test/test_fake_sdk_client.cpp)
  target_link_libraries(test_fake_sdk_client astrall_sdk_client)
  # 诊断死锁会阻塞进程；独立套件设置进程级超时，防止回归永久挂起。
  ament_add_gtest(test_sdk_exception_reentry
    test/test_sdk_exception_reentry.cpp TIMEOUT 10)
  target_link_libraries(test_sdk_exception_reentry astrall_sdk_client)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_driver_exit
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/tests/test_driver_exit.py"
      "$<TARGET_FILE:astrall_driver_node>"
  )
  set_tests_properties(test_driver_exit PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=91;ROS_LOG_DIR=${ros_test_log_dir}")
  get_target_property(sdk_library astrall_sdk::sdk IMPORTED_LOCATION)
  get_filename_component(sdk_library_dir "${sdk_library}" DIRECTORY)
  get_filename_component(sdk_prefix "${sdk_library_dir}" DIRECTORY)
  add_test(NAME test_sdk_install
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/tests/test_sdk_install.py"
      --vendor-prefix "${sdk_prefix}"
      --driver-prefix "${CMAKE_INSTALL_PREFIX}"
      --interfaces-prefix "${astrall_interfaces_DIR}/../../.."
  )
  set_tests_properties(test_sdk_install PROPERTIES TIMEOUT 60)
endif()

ament_package()
