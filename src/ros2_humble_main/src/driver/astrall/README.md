# 璇玑 SDK 集成

本目录由原 SDK 工作空间迁入，仅保留 3dnav 使用的三个包：`astrall_sdk_vendor`、`astrall_interfaces`、`astrall_driver`。厂商 SDK 1.1.1 的 x86_64 库与头文件保持原件，位于 `third_party/astrall_sdk/C++`，直接随本仓库管理，不使用子模块或外部工作空间。

从 3dnav 根目录执行 `./scripts/build_workspace.sh --packages-select astrall_sdk_vendor astrall_interfaces astrall_driver car_vel --cmake-args -DBUILD_TESTING=OFF`。之后只加载 `install/setup.bash`。支持 Linux x86_64、ROS 2 Humble。

入口是本体选择管理器 → `car_vel.xuanji_bridge` → 进程守护器 → 本工作空间安装的 `astrall_driver_node` → 本工作空间安装的 `libASTRALL_SDK.so.1`。SDK 相机和雷达禁用；未引入相机、键盘 Demo、假雷达发包脚本。驱动内部保留原有协议与点云解析代码以避免改变既有驱动实现；不用于主机建图雷达。

验证用例保留接口、参数、控制权、模式确认、速度看门狗和安装路径检查。删除了依赖外部假雷达脚本的两组用例，未引入对应发包工具。运行 `colcon test --packages-select astrall_driver --ctest-args -R "test_driver_parameters|test_control_policies|test_core_control|test_sdk_install" --output-on-failure`（先以 BUILD_TESTING=ON 编译）。

网络约定：机器人 10.18.10.100，主机 10.18.10.201（现场自行配置），SDK 3600/3601。不要同时启动其他 SDK 控制进程。操作与验收范围见主仓库 README 和 car_vel/ROBOT_BODY_SELECTION.md。

导入基线：原独立驱动仓库提交 `b31e339`（含 20 ms 调用预算修正）。厂商库 SHA-256：`7d50e5da195f3c5c0f6cf2e1239ab31d59310bc24f6461809db7c14bf6212da8`。后续改动仅在本仓库维护。
