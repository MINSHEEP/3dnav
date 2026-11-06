# 使用配置文件位置计算前缀，支持安装树重定位；不引用原 SDK 的路径。
if(NOT TARGET astrall_sdk::sdk)
  get_filename_component(_astrall_sdk_prefix
    "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
  add_library(astrall_sdk::sdk SHARED IMPORTED GLOBAL)
  set_target_properties(astrall_sdk::sdk PROPERTIES
    IMPORTED_LOCATION "${_astrall_sdk_prefix}/lib/libASTRALL_SDK.so.1"
    IMPORTED_SONAME "libASTRALL_SDK.so.1"
    INTERFACE_INCLUDE_DIRECTORIES "${_astrall_sdk_prefix}/include"
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_astrall_sdk_prefix}/include"
  )
  unset(_astrall_sdk_prefix)
endif()
