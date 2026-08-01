cmake_minimum_required(VERSION 3.8)
project(lzp_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Relocalize.action"
  "msg/CurrentMap.msg"
  "msg/RelocalizationRequest.msg"
  "msg/Waypoint.msg"
  "msg/NavigationStatus.msg"
  "msg/LaunchStatus.msg"
  "msg/RoutePoint.msg"
  "msg/Route.msg"
  "msg/RouteExecutionStatus.msg"
  "srv/CreateWaypoint.srv"
  "srv/ReadWaypoint.srv"
  "srv/ListWaypoints.srv"
  "srv/UpdateWaypoint.srv"
  "srv/DeleteWaypoint.srv"
  "srv/NavigateToWaypoint.srv"
  "srv/MultiNavigate.srv"
  "srv/PauseNavigation.srv"
  "srv/ResumeNavigation.srv"
  "srv/SwitchMap.srv"
  "srv/ControlLaunch.srv"
  "srv/ArmControl.srv"
  "srv/SetInitialPose.srv"
  "srv/MapSave.srv"
  "srv/GetMapList.srv"
  "srv/GetCurrentMap.srv"
  "srv/DeleteMap.srv"
  "srv/EditMap.srv"
  "srv/GetEditedMap.srv"
  "srv/SetMapPixelIndices.srv"
  "srv/SaveRoute.srv"
  "srv/ListRoutes.srv"
  "srv/GetRoute.srv"
  "srv/DeleteRoute.srv"
  "srv/StartRoute.srv"
  "srv/ControlRoute.srv"
  "srv/GetRouteExecutionState.srv"
  "srv/DeleteRoutesByMap.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
