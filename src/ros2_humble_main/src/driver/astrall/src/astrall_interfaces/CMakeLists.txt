cmake_minimum_required(VERSION 3.16)
project(astrall_interfaces LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SdkResult.msg"
  "msg/ConnectionStatus.msg"
  "msg/SystemStatus.msg"
  "msg/PowerStatus.msg"
  "msg/SportStatus.msg"
  "msg/DeviceInfo.msg"
  "msg/WheelSpeeds.msg"
  "msg/JoystickState.msg"
  "srv/SetControlAuthority.srv"
  "srv/SetMode.srv"
  "srv/SetLight.srv"
  "srv/SendMessage.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime std_msgs)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/astrall_testing.cmake")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  ament_add_gtest(test_interfaces test/test_interfaces.cpp)
  rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME}
    rosidl_typesupport_cpp)
  target_link_libraries(test_interfaces "${typesupport_target}")
  ament_target_dependencies(test_interfaces rclcpp)
endif()

ament_package()
