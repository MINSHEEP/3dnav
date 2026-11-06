# astrall_sdk_vendor

为Linux x86_64导出官方SDK 1.1.1的 `astrall_sdk::sdk` imported target，不编译SDK实现。原始目录与alias只读；安装生成头文件 `include/astrall_sdk/interface.h` 和按ELF SONAME命名的 `lib/libASTRALL_SDK.so.1` 副本。下游不需要源码路径，CMake按安装前缀定位，安装树重定位已纳入回归。

## 构建与导出

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to astrall_sdk_vendor --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select astrall_sdk_vendor --return-code-on-test-failure
```

下游CMake：

```cmake
find_package(astrall_sdk_vendor REQUIRED)
target_link_libraries(my_target PRIVATE astrall_sdk::sdk)
```

C++使用 `#include "astrall_sdk/interface.h"`。运行下游前加载工作空间 `install/setup.bash`；vendor环境hook把安装库目录加入动态加载路径。链接或构造封装不会初始化SDK，真实连接只能通过显式Init/driver配置。

| 运行接口 | 本包职责 |
| --- | --- |
| 可执行节点、话题、服务、ROS参数 | 不适用；纯库导出包 |
| CMake target | astrall_sdk::sdk |
| 编译平台 | 已验证Linux x86_64；其他SDK原件仅保留，不声称支持 |
| 网络/单位/时钟/TF | 本包不配置网络、不转换数据、不发布TF；由driver与接口文档定义 |

厂商库和头文件随本仓库管理，具体目录和构建方式见 ../../README.md。链接库不会自动申请控制权。
