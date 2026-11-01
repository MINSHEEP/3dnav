# 雷达网络设置（MID360s）

设置页可修改雷达驱动使用的雷达 IPv4 地址与本机 IPv4 地址。建图和导航共用，独立于星玑/钢镚本体配置。首次使用从驱动包的 MID360s_config.json 读取默认值（当前 192.168.1.130 / 192.168.1.50）。

## 部署

安全停止机器人和任务、退出旧后台，同步代码后，在小电脑执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_interface livox_ros_driver2 car_vel --cmake-args -DROS_EDITION=ROS2
source install/setup.bash
```

按原方式启动主后台并刷新设置页。在“雷达网络（MID360s）”填写实际地址并保存。不会立即重启雷达；需安全停止当前任务，重新启动包含雷达的建图/导航进程。仅重新下发导航目标不会重启雷达。若原后台保留雷达进程，请安全退出后重启后台。

## 生效范围

- 不修改雷达硬件自身的 IP，也不修改 Linux 网卡、路由或掩码。雷达地址应已在硬件上配置好；本机地址应已配置在连接雷达的网卡上。
- 页面通过本机 UDP 地址绑定检查给出提示，但不发送探测包；绑定成功不等于雷达可达。掩码、路由和网络连通性需要部署现场确认，不假设所有网络都是 /24。
- 保存于后台运行用户的 `~/.ros/robot_navigation/lidar_network.json`。服务和建图/导航必须使用同一 Linux 用户；不要混用普通用户和 sudo 启动。
- 两个项目启动文件都在每次启动时生成内容寻址的独立 JSON 快照，然后传给 `msg_MID360s_launch.py` 的 `user_config_path` 参数。保存不会改动正在运行的快照，不改端口、外参、点云类型等其他参数。
- 单独运行厂商 `msg_MID360s_launch.py`（未传参数）、RViz 示例、MID360/HAP 示例仍使用各自原配置；设置页只控制本项目建图和导航流程。
- 页面显示的是已保存值，不声称是当前驱动实际地址。新版本配置不依赖修改源码 JSON 或重编译；首次部署新增服务/启动参数需要上述编译。

## 故障与验证

非法 IPv4、相同雷达/本机地址、回环/组播/有限广播等地址被拒绝。保存采用原子替换和版本检查，失败保留旧配置，多页面旧版本提交需刷新后重试。保存文件损坏时拒绝启动雷达，不静默使用默认值；应在停止任务后备份损坏文件并恢复正确配置，或移走该文件后从默认值重新设置。

状态话题 `/lidar/network_settings`，服务 `/lidar/set_network`（`lzp_interface/srv/SetLidarNetwork`，get/save）。建议真机验证：读取默认值、保存新地址、当前任务不中断、安全重启后核对驱动 user_config_path 与 `/livox/lidar`、`/livox/imu` 数据；分别验证建图和导航启动。可用 `ros2 param get /livox_lidar_publisher user_config_path` 查实际加载文件。

本地自动化覆盖地址校验、保存失败、版本冲突、快照隔离、启动参数接线和页面离线/过期状态；没有替代 Ubuntu 实机 ROS 编译和网络验收。
