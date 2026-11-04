cmake_minimum_required(VERSION 3.8)
project(icp_registration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lzp_interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# These are system CMake packages, not ament packages. Keep their exact CMake
# names here instead of passing the lower-case rosdep keys through ament_auto.
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/icp_registration.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  geometry_msgs
  lzp_interface
  pcl_conversions
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN icp::IcpNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  lzp_interface
  pcl_conversions
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_search_seeds test/test_search_seeds.cpp)
  target_include_directories(test_search_seeds PRIVATE include)
  add_test(NAME search_seeds COMMAND test_search_seeds)
  add_executable(test_candidate_quality test/test_candidate_quality.cpp)
  target_include_directories(test_candidate_quality PRIVATE include)
  add_test(NAME candidate_quality COMMAND test_candidate_quality)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
