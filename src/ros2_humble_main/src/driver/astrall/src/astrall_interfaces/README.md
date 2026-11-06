# astrall_interfaces

导出驱动、键盘和离线探针使用的8种消息与4种服务定义。本包不运行节点或执行服务；节点、话题、服务实例和ROS参数不适用于纯接口包，实际端点由driver/keyboard提供。

## 构建与下游使用

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to astrall_interfaces --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select astrall_interfaces --return-code-on-test-failure
```

CMake使用 `find_package(astrall_interfaces REQUIRED)` 并将其加入 `ament_target_dependencies`。C++示例头为 `astrall_interfaces/srv/set_mode.hpp`，Python为 `from astrall_interfaces.srv import SetMode`；运行下游先source安装后的setup。全仓 `scripts/check_all.sh` 包括接口序列化、协议常量、中文与未知位回归和三项实际lint。

| 消息 | 字段约定 |
| --- | --- |
| ConnectionStatus | ROS接收时间、连接及回调确认持权；默认false |
| SystemStatus | 原始error/warn位域与可读文本，保留未知位 |
| PowerStatus | 电量[0,1]、原始温度/电压/循环/charged；温度单位待厂商确认 |
| SportStatus | 手册0xBxxx状态，区别于0x8xxx返回码 |
| DeviceInfo | 固件版本、序列号、型号 |
| WheelSpeeds | 左前/右前/左后/右后原始值与SDK时戳，轮速单位未确认 |
| JoystickState | 六路原始摇杆、按钮与三档开关，不猜归一化范围 |
| SdkResult | uint16原始码、success、可读message，未知码保留 |

| 服务定义 | 约定与实际执行者 |
| --- | --- |
| SetControlAuthority | SDK=1、JOYSTICK=2，driver执行；ACK不等于确认持权 |
| SetMode | Damping/Stand/Down/Move/Tank，driver按实际状态校验；RUNNING只是ACK，confirm_reboot_on_dock默认false |
| SetLight | OPEN=1、CLOSE=2，由driver返回原码和可读结果 |
| SendMessage | driver验证UTF-8字节长度/NUL边界并严格限频<5Hz |

IMU由driver发布标准sensor_msgs/Imu；雷达发布标准PointCloud2、Odometry和同时间TF，相机发布标准Image。消息header采用ROS接收时间，原始SDK时戳不假定为ROS采样时刻。单位/坐标/未知协方差由相应数据包README与OPEN_QUESTIONS说明，接口包不补TF、不改网络、不调用SDK。启动与安全控制步骤见driver文档。
