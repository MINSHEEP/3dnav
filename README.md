# nav_ws 部署、编译与运行手册

本工作空间运行于 ROS 2 Humble，面向 Livox MID-360S 雷达和机械狗，提供点云采集、Small Point-LIO、OctoMap 建图、ICP 重定位、Nav2 导航、地图与路线管理、ROSBridge 以及 Web 前端。

本文以当前目标机器为准：

| 项目 | 当前配置 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS |
| 架构 | x86_64 / amd64 |
| ROS 2 | Humble |
| 工作空间 | `/home/a/nav_ws` |
| 主机雷达网卡 | `192.168.1.50/24` |
| Livox 雷达 | `192.168.1.192` |
| 底盘/机器狗 | `192.168.96.2` |

项目核心算法不依赖 CUDA。Intel 核显只影响 RViz 显示，不影响雷达驱动、LIO、ICP 或 Nav2。

## 1. 工作空间结构

```text
/home/a/nav_ws/
├── src/
│   ├── ros2_humble_main/
│   ├── Livox_SDK2/ 或 livox_sdk2/
│   ├── gtsam/              # 使用预编译包时不参与 colcon 构建
│   └── Sophus/             # 使用预编译包时不参与 colcon 构建
├── build/                  # colcon 生成
├── install/                # colcon 生成
├── log/                    # colcon 生成
├── nav.db
├── mapping.rviz
└── navigation.rviz
```

不要从其他机器复制 `build/`、`install/` 和 `log/`。Small Point-LIO 使用 `-march=native`，必须在目标机器上重新编译。

## 2. 首次部署

### 2.1 加载 ROS 2 Humble

以下步骤假定 ROS 2 Humble 已经通过官方软件源安装：

```bash
source /opt/ros/humble/setup.bash
```

如果该文件不存在，应先安装 ROS 2 Humble Desktop，再继续部署项目。

### 2.2 安装基础工具和运行依赖

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential cmake git ripgrep mesa-utils \
  libpcl-dev libeigen3-dev libyaml-cpp-dev liboctomap-dev libapr1-dev \
  libpcap-dev \
  python3-numpy python3-pil python3-yaml python3-serial python3-websockets \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-rotation-shim-controller \
  ros-humble-octomap \
  ros-humble-octomap-msgs \
  ros-humble-rosbridge-server \
  ros-humble-tf-transformations \
  ros-humble-sensor-msgs-py
```

其中几个容易遗漏的依赖是：

- `ros-humble-navigation2`：Nav2 核心节点与插件。
- `ros-humble-nav2-bringup`：Nav2 标准启动文件，本项目会直接调用。
- `liboctomap-dev`：提供 `octomap-config.cmake`。
- `ros-humble-octomap-msgs`：提供 OctoMap ROS 消息。
- `ros-humble-rosbridge-server`：启动管理后端所需的 WebSocket 服务。

### 2.3 安装 GTSAM 和 Sophus 预编译包

当前 `ros2_humble_main` 没有直接使用 GTSAM 或 Sophus，但可以安装 ROS Humble 提供的预编译包：

```bash
sudo apt install -y \
  ros-humble-gtsam \
  ros-humble-sophus
```


### 2.4 检查 Livox SDK2

Livox ROS 驱动需要 `liblivox_lidar_sdk_shared.so`。先检查系统是否已经安装：

```bash
ldconfig -p | grep livox_lidar_sdk
```

如果能看到该共享库，可跳过本节。如果找不到，并且工作空间中的 SDK 不是可由 colcon 安装的 ROS 包，则手动安装：

```bash
cd ~/nav_ws/src/Livox_SDK2
cmake -S . -B build-sdk -DCMAKE_BUILD_TYPE=Release
cmake --build build-sdk -j8
sudo cmake --install build-sdk
sudo ldconfig
```

实际目录名如果是 `livox_sdk2`，相应修改 `cd` 路径。

### 2.5 使用 rosdep 补齐包依赖

```bash
source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update

cd /home/a/nav_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

参数说明：

- `--from-paths src`：扫描 `src` 下的 `package.xml`。
- `--ignore-src`：依赖已在当前源码工作空间中时，不再安装同名系统包。
- `--rosdistro humble`：明确使用 Humble 的依赖规则。
- `-r`：某项解析失败时继续处理其余依赖。
- `-y`：自动确认 apt 安装。

`rosdep` 只能安装已正确声明在 `package.xml` 中的依赖。若依赖声明遗漏，仍需使用前述 `apt install` 手动安装。

## 3. 项目配置

### 3.1 固定路径

当前项目已按照以下位置配置：

```text
/home/a/nav_ws
```

检查是否还有其他机器遗留的绝对路径：

```bash
cd ~/nav_ws
rg -n '/home/(cat|zy|openarm|hsh|b2w)' src/ros2_humble_main
```

当前地图与点云地图目录：

```text
/home/a/nav_ws/src/ros2_humble_main/src/navigation/robot_navigation2/maps
/home/a/nav_ws/src/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps
```


### 3.2 雷达网络

当前 MID-360S 配置使用：

```text
主机 IP：192.168.1.50
雷达 IP：192.168.1.192
```

查看网卡和路由：

```bash
nmcli device status
ip -br address
ip route
```

为专用雷达网卡设置静态 IP：

```bash
sudo nmcli connection modify "<连接名称>" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.50/24 \
  ipv4.gateway "" \
  ipv4.never-default yes

sudo nmcli connection up "<连接名称>"
```

不要远程修改当前 SSH 正在使用的网卡，否则可能断开连接。

验证网络：

```bash
ping -c 4 192.168.1.192
ping -c 4 192.168.96.2
```

当前建图和导航启动文件调用 `msg_MID360s_launch.py`。如果实际设备是普通 MID-360，应改用 `msg_MID360_launch.py` 和 `MID360_config.json`。

### 3.3 串口权限（仅串口底盘需要）

```bash
sudo usermod -aG dialout "$USER"
```

执行后注销并重新登录。建议使用稳定的 `/dev/serial/by-id/...`，而不是依赖可能变化的 `/dev/ttyUSB0`。

## 4. 编译工作空间

不要使用 `sudo colcon build`。

### 4.1 正常编译

```bash
cd /home/a/nav_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --parallel-workers 4 \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble \
    -DCMAKE_BUILD_TYPE=Release

source /home/a/nav_ws/install/setup.bash
```

如果内存不足，将 `--parallel-workers 4` 改成 `2`。

### 4.2 从其他机器复制源码后的首次编译

如果复制内容中包含旧的 `build/`、`install/` 或 `log/`，应先删除这些生成物：

```bash
cd /home/a/nav_ws
rm -rf \
  /home/a/nav_ws/build \
  /home/a/nav_ws/install \
  /home/a/nav_ws/log
```

该操作不会删除 `src/`、地图、PCD 或顶层数据库。清理后重新打开终端，再执行正常编译。

### 4.3 自动加载环境

完成首次编译后，可以加入 `~/.bashrc`：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo '[ -f /home/a/nav_ws/install/setup.bash ] && source /home/a/nav_ws/install/setup.bash' >> ~/.bashrc
```

每个新终端都应先加载系统 ROS，再加载本工作空间。

## 5. 运行

### 5.1 启动常驻管理后端

```bash
source /opt/ros/humble/setup.bash
source /home/a/nav_ws/install/setup.bash
ros2 launch car_vel launch_manager.launch.py
```

该启动文件包含启动管理、地图/路线服务、底盘桥接和 ROSBridge。ROSBridge 默认监听 `9090` 端口；建图和导航模式由前端或控制服务按需启动。

### 5.2 启动并保存全部日志

只保存完整日志、不生成筛选日志，推荐每次使用独立时间戳文件：

```bash
source /opt/ros/humble/setup.bash && \
source /home/a/nav_ws/install/setup.bash && \
RCUTILS_COLORIZED_OUTPUT=0 \
ros2 launch car_vel launch_manager.launch.py 2>&1 | \
tee "/home/a/nav_debug_$(date +%Y%m%d_%H%M%S).log"
```

这个命令会把全部标准输出和错误输出同时显示在终端并保存到日志。按 `Ctrl+C` 停止。

如需始终覆盖同一个文件：

```bash
source /home/a/nav_ws/install/setup.bash && \
RCUTILS_COLORIZED_OUTPUT=0 \
ros2 launch car_vel launch_manager.launch.py 2>&1 | \
tee /home/a/nav_debug_full.log
```

实时查看日志：

```bash
tail -f /home/a/nav_debug_full.log
```

ROS 2 自身也会把每次运行的节点日志保存在 `/home/a/.ros/log/`。

### 5.3 直接启动建图

```bash
source /opt/ros/humble/setup.bash
source /home/a/nav_ws/install/setup.bash
ros2 launch car_vel mapping.launch.py
```

### 5.4 直接启动导航

```bash
source /opt/ros/humble/setup.bash
source /home/a/nav_ws/install/setup.bash
ros2 launch car_vel car_vel.launch.py
```

### 5.5 启动 Web 前端

```bash
cd ~/nav_ws/src/ros2_humble_main/src/car_vel/car_vel/web_frontend
python3 -m http.server 8080 --bind 0.0.0.0
```

浏览器访问：

```text
http://<M9-PRO局域网IP>:8080
```

## 6. 分阶段验证

### 6.1 检查关键包

```bash
source /opt/ros/humble/setup.bash
source /home/a/nav_ws/install/setup.bash

ros2 pkg prefix lzp_interface
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix octomap_server2
ros2 pkg prefix nav2_bringup
ros2 pkg prefix rosbridge_server
```

项目包应位于 `/home/a/nav_ws/install/...`，系统预编译包通常位于 `/opt/ros/humble`。

### 6.2 单独验证雷达

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/a/nav_ws/install/setup.bash

ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### 6.3 验证 LIO 和建图

启动建图后检查：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /projected_map
ros2 run tf2_ros tf2_echo odom base_link
```

### 6.4 导航前检查

```bash
ros2 topic hz /scan
ros2 topic hz /Odometry
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

同时确认：

- `map.yaml` 及其引用的图片存在。
- ICP 使用的 PCD 地图存在。
- 雷达和底盘网络可达。
- `/scan`、`/Odometry` 和 TF 持续更新。

## 7. 常见问题

### 7.1 lzp_interface 无法创建符号链接

错误：

```text
failed to create symbolic link ... because existing path cannot be removed:
Is a directory
```

原因是旧构建目录与 `--symlink-install` 冲突。只清理该包并重新构建：

```bash
cd ~/nav_ws
rm -rf build/lzp_interface install/lzp_interface

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_interface
```

成功后重新执行全量 `colcon build --symlink-install --parallel-workers 4`。

### 7.2 找不到 OctoMap

错误：

```text
Could not find octomapConfig.cmake
```

处理：

```bash
sudo apt update
sudo apt install -y \
  liboctomap-dev \
  ros-humble-octomap \
  ros-humble-octomap-msgs

cd ~/nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --symlink-install \
  --packages-select octomap_server2 \
  --cmake-clean-cache
```

`io features related to pcap will be disabled` 只是警告；安装 `libpcap-dev` 后可消除。

### 7.3 找不到 rosbridge_server

错误：

```text
package 'rosbridge_server' not found
```

处理：

```bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-server

source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
ros2 pkg prefix rosbridge_server
```

这是运行时预编译包，安装后不需要重新编译项目。

### 7.4 找不到 Nav2

错误通常会包含 `nav2_bringup`、`controller_server`、`bt_navigator` 或其他 Nav2 包未找到。

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

安装后重新加载环境即可，不需要重新编译 Nav2。

### 7.5 其他包显示 Aborted

并行构建时，一个包 `Failed` 会使其他包显示 `Aborted`。首先处理第一个 `Failed` 包，然后再次运行全量构建。`Aborted` 不等于这些包自身存在代码错误。

### 7.6 权限错误

不要使用 `sudo colcon build`。如果历史构建导致文件归 root 所有：

```bash
sudo chown -R a:a /home/a/nav_ws
```

### 7.7 没有 Livox 话题

依次检查：

1. 实际设备是否为 MID-360S。
2. 启动文件是否为 `msg_MID360s_launch.py`。
3. 主机网卡是否配置为 `192.168.1.50/24`。
4. 雷达是否为 `192.168.1.192`。
5. `ldconfig -p | grep livox_lidar_sdk` 是否能找到 SDK。
6. 防火墙、网线和路由是否正常。

## 8. 最短部署流程

完成系统 ROS 安装、网络配置和源码复制后，可按以下顺序执行：

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-octomap \
  ros-humble-octomap-msgs \
  liboctomap-dev \
  ros-humble-rosbridge-server

cd /home/a/nav_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

colcon build --symlink-install --parallel-workers 4
source install/setup.bash

ros2 launch car_vel launch_manager.launch.py
```

首次部署建议按“雷达 → LIO → 建图 → 定位 → Nav2 → 后端 → Web 前端”的顺序逐级验证，确认硬件、话题和 TF 正常后再配置开机自启。
