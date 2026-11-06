find_package(ament_cmake_cpplint REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(ament_cmake_cppcheck REQUIRED)

# 空壳阶段仍编译 C++17 检查，并为三项 lint 提供真实输入。
add_library(${PROJECT_NAME}_cxx17_check OBJECT test/cxx17_check.cpp)

# 恢复 ament 默认放宽的 Google 空白规则，不屏蔽任何项目告警。
ament_cpplint(
  MAX_LINE_LENGTH 80
  FILTERS
    +whitespace/braces
    +whitespace/indent
    +whitespace/parens
    +whitespace/semicolon
)
ament_uncrustify(
  CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/../config/uncrustify.cfg"
  MAX_LINE_LENGTH 80
)
# 使用系统自带的 gtest 宏模型解析行为测试，不关闭任何诊断。
ament_cppcheck(LANGUAGE "c++" LIBRARIES googletest)

# Humble 默认跳过 cppcheck 2.x；必须强制实际运行本机分析器。
set_tests_properties(cppcheck PROPERTIES
  ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1"
)
