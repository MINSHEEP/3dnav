cmake_minimum_required(VERSION 3.16)
project(astrall_sdk_vendor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR
   NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  message(FATAL_ERROR "此工作空间仅验证 Linux x86_64 SDK。")
endif()

set(sdk_root "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/astrall_sdk/C++")
set(sdk_library "${sdk_root}/lib/linux/x86_64/libASTRALL_SDK.so.1.1.1")
if(NOT EXISTS "${sdk_library}")
  message(FATAL_ERROR "缺少原始 SDK 库：${sdk_library}")
endif()

# 原件只读；安装副本使用 ELF SONAME，避免依赖源码目录中的软链接。
install(FILES "${sdk_library}" DESTINATION lib RENAME libASTRALL_SDK.so.1)
install(FILES "${sdk_root}/include/interface.h" DESTINATION include/astrall_sdk)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env/library_path.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env/library_path.dsv.in"
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/astrall_testing.cmake")
endif()

ament_package(CONFIG_EXTRAS cmake/astrall_sdk_vendor-extras.cmake)
