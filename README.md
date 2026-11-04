# 3dnav 部署、编译与运行手册

更新日期：2026-10-28。

本工作空间运行于 ROS 2 Humble，支持星玑与钢镚 ZSL-1 点足机械狗，提供 Livox 点云采集、Small Point-LIO、OctoMap 建图、三维定位、Nav2 导航、地图和路线管理、巡检任务、视觉/行为识别、USB 热成像、M701 环境检测及 Web 前端。

本文以 M9 Pro 小电脑的目标部署为例，网卡名称和 IP 应按现场实际配置：

| 项目 | 当前配置 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS |
| 架构 | x86_64 / amd64 |
| ROS 2 | Humble |
| 工作空间 | `/home/a/3dnav` |
| 主机雷达网卡 | `192.168.1.50/24` |
| Livox 雷达 | MID360s 配置默认 `192.168.1.130` |
| 星玑本体 | `192.168.96.2:8080`；小电脑示例 `192.168.96.3/24` |
| 钢镚 ZSL-1 点足 | 本体示例 `192.168.168.168`；小电脑示例 `192.168.168.15`；本地接收端口 `43988` |
| Python | 系统 Python 3.10；识别依赖按下文隔离安装 |

项目核心导航算法不依赖 CUDA，当前识别部署也使用 CPU 运行库，不要求 NVIDIA 显卡。Intel 核显用于界面显示；实际导航与多模型识别并行负载仍需在小电脑上测量。

## 功能与文档入口

| 模块 | 当前功能 |
| --- | --- |
| 建图 / 修图 | 二维栅格、全局三维点云、地图保存；支持配置建图动态障碍过滤，以及按区域/高度编辑对应点云 |
| 地形导航 | 星玑三维模式下在点云俯视图设置初始点、目标点、多点路线；预览与启动分开，显示全局规划路径 |
| 导航模式 | 设置页选择二维或三维地形增强模式；三维模式仍使用现有 Nav2 平面控制接口，视角切换不等于模式切换 |
| 园区 / 巡检 | 路线执行、检测点、任务进度、告警、证据图片和历史记录 |
| 遥控 / 本体 | 独立遥控；设置页选择星玑或钢镚 ZSL-1 点足，各自保存连接参数 |
| 普通摄像头 | 设置页选择 USB 或本体 HTTP 图传，供遥控、识别和巡检共享 |
| 视觉识别 | 火焰、烟雾、香烟/烟头目标、人脸及车牌检测/匹配，具体结果依赖模型与登记库 |
| 行为识别 | 疑似跌倒、躺卧姿态、场景级疑似打架；没有接入抽烟行为识别 |
| 环境检测 | 海康 USB 热成像、高温阈值与告警；M701 气体/环境数据及 CO₂ 阈值语音告警 |
| 设置 | 本体、导航模式、障碍膨胀半径、雷达网络参数、普通摄像头、气体连接、告警阈值、建图过滤开关、系统电源操作 |

星玑自动导航与巡检沿用固定越障模式；进入越障模式不代表已实现任意楼梯、坑洞或复杂地形的可靠自主通过。三维定位、地形判断、规划和本体能力需要分别验证。详见[星玑三维定位与防跳变](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)及[固定越障策略](src/ros2_humble_main/src/car_vel/XINGJI_FIXED_OBSTACLE.md)。

完整专题文档见[第 9 节](#9-专题文档)。

## 1. 工作空间结构

```text
/home/a/3dnav/
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
  build-essential cmake git ripgrep mesa-utils usbutils v4l-utils \
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
cd /home/a/3dnav/src/Livox_SDK2
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

cd /home/a/3dnav
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

参数说明：

- `--from-paths src`：扫描 `src` 下的 `package.xml`。
- `--ignore-src`：依赖已在当前源码工作空间中时，不再安装同名系统包。
- `--rosdistro humble`：明确使用 Humble 的依赖规则。
- `-r`：某项解析失败时继续处理其余依赖。
- `-y`：自动确认 apt 安装。

`rosdep` 只能安装已正确声明在 `package.xml` 中的依赖。若依赖声明遗漏，仍需使用前述 `apt install` 手动安装。

### 2.6 普通视觉与热成像依赖

首次使用普通摄像头、YOLO 或热成像时，安装相机运行依赖：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_usb_camera_deps.sh
sudo usermod -aG video,plugdev "$USER"
```

组权限变更后注销并重新登录。脚本把 Python 依赖安装到 `~/.local/share/hikvision-camera/python`，启动包装脚本加载该目录；不要通过全局升降 NumPy/OpenCV 来修复相机，从而影响 ROS 环境。

当前锁定的普通识别依赖包括 CPU 版 PyTorch 2.10、Ultralytics 8.4.104、NumPy 2.2.6 和 OpenCV 4.13，具体以仓库的 requirements 和安装脚本为准。普通识别默认主模型是 `hikvision_camera/models/cigarette_multiclass_best.pt`，检测烟雾、火焰及香烟/烟头目标；它不能单独证明某个人正在抽烟。

热成像另需完整的外部海康 bundle：

```text
/opt/hik_thermal_linux64_bundle/PythonDemo/thermal_max_cv.py
/opt/hik_thermal_linux64_bundle/library/linux64/libHCUSBSDK.so
```

必须保留配套动态库和适配脚本，不能只复制主 `.so`。USB 权限、校验和部署步骤见[普通摄像头](src/ros2_humble_main/src/car_vel/USB_CAMERA.md)和[热成像](src/ros2_humble_main/src/car_vel/THERMAL_CAMERA.md)。

### 2.7 跌倒与打架识别依赖

行为识别使用单独的 CPU 环境，首次部署执行：

```bash
sudo apt-get install python3-venv libgl1 libglib2.0-0 espeak-ng ffmpeg
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_behavior_deps.sh
```

| 运行范围 | 默认依赖位置 | 版本约束 |
| --- | --- | --- |
| ROS 节点 | 系统 Python / ROS 环境 | 使用 ROS Humble 对应系统依赖 |
| 普通视觉、热成像 | `~/.local/share/hikvision-camera/python` | NumPy 2.2.6、OpenCV 4.13、CPU PyTorch |
| 跌倒与打架 | `~/.local/share/robot-behavior/venv` | NumPy 1.26.4、OpenCV 4.6、Paddle 2.6.2、ONNX Runtime 1.18 |

行为依赖不能装进系统 Python 或普通相机目录，也不需要在 ROS 终端手动激活这个环境。网关分别启动跌倒与打架进程，每个进程只加载自己的模型，默认各使用两个 CPU 线程。

模型和厂商源码归档需随 `car_vel/person_behavior/` 完整同步；安装脚本执行资产校验和模型加载自检。加载成功不等于实际帧率或识别准确率达标。使用方法见[行为识别说明](src/ros2_humble_main/src/car_vel/BEHAVIOR_RECOGNITION.md)，最新独立采样机制见[行为运行时说明](src/ros2_humble_main/src/car_vel/person_behavior/README.md)。

## 3. 项目配置

### 3.1 固定路径

当前项目已按照以下位置配置：

```text
/home/a/3dnav
```

检查是否还有其他机器遗留的绝对路径：

```bash
cd /home/a/3dnav
rg -n '/home/(cat|zy|openarm|hsh|b2w)' src/ros2_humble_main
```

当前地图与点云地图目录：

```text
/home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/maps
/home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps
```


### 3.2 雷达网络

当前 MID-360S 配置使用：

```text
主机 IP：192.168.1.50
雷达 IP：192.168.1.130
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
ping -c 4 192.168.1.130
ping -c 4 192.168.96.2
```

当前建图和导航启动文件调用 `msg_MID360s_launch.py`。如果实际设备是普通 MID-360，应改用 `msg_MID360_launch.py` 和 `MID360_config.json`。

设置页的“雷达网络”保存驱动使用的雷达/本机 IP；它不会配置 Linux 网卡、路由，也不会修改雷达硬件地址。保存后要停止并重新启动包含雷达的建图/导航进程才会使用新配置。独立启动厂商示例时仍使用该示例自己的配置。详见[雷达网络设置](src/ros2_humble_main/src/car_vel/LIDAR_NETWORK.md)。

若现场使用 `robot-network.service` 和 `/usr/local/sbin/setup-robot-network.sh` 开机配网，应核对脚本与页面的地址一致；该脚本属于现场系统配置，不能只改网页就认为网卡也已修改。

### 3.3 串口与 USB 权限

```bash
sudo usermod -aG dialout "$USER"
```

执行后注销并重新登录。建议使用稳定的 `/dev/serial/by-id/...`，而不是依赖可能变化的 `/dev/ttyUSB0`。

M701 经 USB-RS485 接入，同样需要 `dialout` 权限。兼容 CH340/CH341、CH348 对应驱动生成的串口命名，以实际 USB 身份、驱动和设备节点为准；已有正常串口时无需重装驱动。普通 USB 相机使用 `video` 权限，热成像按实际 udev 规则使用 `plugdev`/访问授权。

### 3.4 本体选择与持久配置

设置页切换本体前，结束建图、导航和巡检，退出遥控会话并按页面提示确认本体已安全停放。新本体连接成功不自动站立，也不恢复旧任务。星玑与钢镚使用不同通信接口，不要同时运行厂商键盘 demo、旧版 UDP 桥或另一套控制后台。完整步骤见[本体选择](src/ros2_humble_main/src/car_vel/ROBOT_BODY_SELECTION.md)。

下列文件默认属于启动后台的 Linux 用户：

| 路径 | 用途 |
| --- | --- |
| `~/.ros/robot_navigation/robot_body.json` | 当前本体及两款本体各自的网络/限速配置 |
| `~/.ros/robot_navigation/lidar_network.json` | 已保存的雷达/本机 IP |
| `~/.ros/robot_navigation/ordinary_camera.json` | 普通摄像头来源及采集配置 |
| `~/.ros/robot_navigation/behavior.json` | 跌倒、打架开关 |
| `~/.ros/robot_navigation/routes.db` | 多点路线 |
| `~/.ros/robot_navigation/inspection.db` | 巡检任务与记录 |
| `~/.ros/robot_navigation/recognition.db` | 人脸/车牌识别登记与相关数据 |

请同时备份这些配置、数据库及地图文件，不要把工作空间顶层旧数据库当成当前路线库。不要混用普通用户和 `sudo` 启动后台，否则会读取不同的用户配置。

### 3.5 设置页重启与关机权限

若设置页提示“系统权限未配置”，可在小电脑为运行账号安装固定电源权限：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
sudo bash scripts/install_system_power_permissions.sh a
```

用户名不是 `a` 时替换为实际账号。脚本只授权固定的 `systemctl reboot` 和 `systemctl poweroff`。安装权限本身不重启、不关机；之后执行页面电源操作前，先结束任务并确认地图和数据已保存。

## 4. 编译工作空间

不要使用 `sudo colcon build`。

### 4.1 正常编译

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --parallel-workers 4 \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble \
    -DCMAKE_BUILD_TYPE=Release

source /home/a/3dnav/install/setup.bash
```

首次完整部署涉及 `lzp_interface`、`lzp_service`、`car_vel`、`small_point_lio`、`icp_registration`、`robot_navigation2`、`octomap_server2` 和 `livox_ros_driver2`。更换机器或一次同步多项功能时，优先使用上面的全量编译；增量更新按第 4.4 节选择相关包。

如果内存不足，将 `--parallel-workers 4` 改成 `2`。

### 4.2 从其他机器复制源码后的首次编译

如果复制内容中包含旧的 `build/`、`install/` 或 `log/`，应先删除这些生成物：

```bash
cd /home/a/3dnav
rm -rf \
  /home/a/3dnav/build \
  /home/a/3dnav/install \
  /home/a/3dnav/log
```

该操作不会删除 `src/`、地图、PCD 或顶层数据库。清理后重新打开终端，再执行正常编译。

### 4.3 自动加载环境

完成首次编译后，可以加入 `~/.bashrc`：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo '[ -f /home/a/3dnav/install/setup.bash ] && source /home/a/3dnav/install/setup.bash' >> ~/.bashrc
```

每个新终端都应先加载系统 ROS，再加载本工作空间。

### 4.4 已部署主机的增量更新

先结束任务、正常停止旧后台，完整同步相关源码。仅更新相机页面和本次打架采样修复，且之前已完成完整部署时：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select car_vel
source /home/a/3dnav/install/setup.bash
```

相机网关与 `person_behavior` 子进程必须一起同步，不能只替换网页或单个脚本。已正常加载模型的主机无需为此次采样修复重新下载模型和依赖。

若尚未部署星玑三维定位与防跳变更新，因为涉及 Action 接口，至少四个包一起编译：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential \
  --packages-select lzp_interface icp_registration lzp_service car_vel
source /home/a/3dnav/install/setup.bash
```

编译完成后重新启动原后台，保留现场需要的 launch 参数；浏览器按 Ctrl+F5 强制刷新。只刷新网页不能更新运行中的 Python/C++ 服务。

## 5. 运行

### 5.1 启动常驻管理后端

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
ros2 launch car_vel launch_manager.launch.py
```

该启动文件包含启动管理、地图/路线/巡检服务、所选本体桥接、普通相机网关、独立 USB 热成像服务、M701、完整保存点云 HTTP 服务及 ROSBridge。建图和导航由对应页面按需启动，地图由用户选择加载。

| 端口 | 默认用途 |
| --- | --- |
| `8080` | 按第 5.5 节单独启动的前端静态页面 |
| `9090` | ROSBridge WebSocket |
| `8766` | 普通相机、视觉/行为识别 API，以及热成像代理 |
| `8767` | 热成像服务，仅本机回环访问 |
| `8769` | 保存地图完整 PCD 的只读 HTTP 服务 |
| `8081` | 星玑本体相机本地代理，随本体桥接运行 |

热成像不由浏览器直接打开 USB；YOLO 和行为识别共享网关取得的普通画面，不另开摄像头。未使用的硬件可通过 `camera_enabled:=false`、`thermal_enabled:=false`、`m701_enabled:=false` 关闭对应服务。

### 5.2 启动并保存全部日志

只保存完整日志、不生成筛选日志，推荐每次使用独立时间戳文件：

```bash
source /opt/ros/humble/setup.bash && \
source /home/a/3dnav/install/setup.bash && \
RCUTILS_COLORIZED_OUTPUT=0 \
ros2 launch car_vel launch_manager.launch.py 2>&1 | \
tee "/home/a/nav_debug_$(date +%Y%m%d_%H%M%S).log"
```

这个命令会把全部标准输出和错误输出同时显示在终端并保存到日志。按 `Ctrl+C` 停止。

如需始终覆盖同一个文件：

```bash
source /home/a/3dnav/install/setup.bash && \
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

以下直接启动方式用于独立调试；不要与页面已启动的建图/导航重复运行。日常操作优先使用常驻后端加页面入口。

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
ros2 launch car_vel mapping.launch.py
```

### 5.4 直接启动导航

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
ros2 launch car_vel car_vel.launch.py
```

### 5.5 启动 Web 前端

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel/car_vel/web_frontend
python3 -m http.server 8080 --bind 0.0.0.0
```

浏览器访问：

```text
http://<M9-PRO局域网IP>:8080
```

页面与 ROSBridge 的主机地址应指向小电脑可达地址。浏览器运行在另一台电脑时，不能用浏览器电脑的 `127.0.0.1` 代替小电脑地址。API 示例中的 `127.0.0.1` 命令均在小电脑终端执行。

### 5.6 推荐操作顺序

1. 启动常驻后台和前端，在设置页确认当前本体、导航模式、雷达网络和普通相机来源。
2. 建图时从建图页面启动对应流程；保存后检查地图文件与页面结果，退出建图再进入导航。
3. 选择并加载地图，等待二维栅格和三维点云就绪。保存地图的三维视图读取完整 PCD 中的有效坐标点，不使用建图实时预览的抽稀点云代替。
4. 三维模式下在点云俯视图设置初始位置与朝向、确认定位，再设置目标或编辑路线。
5. 单点或整条路线先预览，再人工点击开始；定位刷新会使旧预览作废时，应重新预览，系统不会自行恢复旧执行。
6. 巡检选择地图、路线及检测点，设置识别项目、高温和 CO₂ 阈值，确认各检测分支状态后启动任务。

三维显示与导航策略独立；三维增强模式仍使用二维代价地图和 Nav2 规划/控制，并结合三维定位与地形检查。二维图中的障碍、膨胀半径和路线可达性仍会影响是否能规划，不能仅凭点云看起来空旷判断一定可通行。

## 6. 分阶段验证

### 6.1 检查关键包

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash

ros2 pkg prefix lzp_interface
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix octomap_server2
ros2 pkg prefix nav2_bringup
ros2 pkg prefix rosbridge_server
```

项目包应位于 `/home/a/3dnav/install/...`，系统预编译包通常位于 `/opt/ros/humble`。

### 6.2 单独验证雷达

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash

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

星玑三维模式还应在初始点确认后检查：

```bash
timeout 8s ros2 topic echo /localization/status --once --full-length
```

应确认 `scope=xingji_3d`、`ready=true`，地图身份正确、传感器新鲜且状态没有错误；地图上的实际位置与朝向也应与现场一致。不能用重复时间戳或单纯收到消息代替新鲜定位。统一定位失效时自动导航零速停车并撤销执行，不自动续走；恢复后重新确认、预览并人工启动。星玑二维和钢镚沿用原定位路径，完整行为见[三维定位说明](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。

### 6.5 普通相机、热成像与识别

在小电脑分别执行以下只读检查：

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/status | python3 -m json.tool --no-ensure-ascii
```

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/thermal/status | python3 -m json.tool --no-ensure-ascii
```

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/behavior/status | python3 -m json.tool --no-ensure-ascii
```

在相机页面启用所需项目并点击“启动识别”后，分别核对：

- 普通相机实际出帧；YOLO、人脸/车牌匹配层按各自模型、登记库和错误状态判断是否可用。
- 热成像 `available/running/ready=true`、`stale=false`，帧号持续增长；仅看到温度数字不代表高温检测开关已启用。
- 行为识别查看 `workers.fall`、`workers.fight` 和对应 `branches`；`running=true` 仅代表进程运行，仍需看分支状态、采样和新鲜度。
- 打架需要 8 个真实采样帧，相邻间隔不得超过 0.6 秒；正常连续采集约需 1.6 秒窗口，再加模型加载/推理时间。低分结果仍可能是 `unknown`，不能当作无异常。
- 跌倒依靠连续姿态规则；躺卧姿态不等于已确认跌倒。打架是场景级判断，不直接认定某个人参与。

已有依赖时，只检查普通识别导入可用性应通过相机包装脚本：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/run_usb_camera.sh /usr/bin/python3 -c 'import cv2, torch, ultralytics; print(cv2.__version__, torch.__version__, ultralytics.__version__)'
```

没有走包装脚本的系统 `python3` 报缺包，不一定意味着相机实际运行环境缺包。行为模型加载自检和性能回放方法见[行为运行时说明](src/ros2_humble_main/src/car_vel/person_behavior/README.md)。

### 6.6 M701 环境数据

```bash
lsusb -d 1a86:7523
```

```bash
ls -l /dev/serial/by-id/
```

```bash
timeout 10s ros2 topic echo /air_quality/m701 --once --full-length
```

`1a86:7523` 是当前 CH340 转换器示例，其他型号按实际 USB 身份检查。默认配置为 9600、8N1、地址 1；应核对 `online`、`valid`、`warming_up`、`values` 和 `error`。收到话题不等于获得有效数据，预热、超时或失联应按具体状态处理。串口选择、供电与 BRLTTY 冲突见[M701 接入说明](src/ros2_humble_main/src/car_vel/M701_RS485.md)。

### 6.7 地图与诊断资料

导航页未显示地图时，先检查 map_server 是否已有栅格：

```bash
timeout 8s ros2 topic echo /map --once --field info --qos-durability transient_local --qos-reliability reliable
```

保存地图通常包括 `maps/<名称>.yaml/.pgm` 和 `pcdmaps/<名称>.pcd/.terrain.npz/.localization.yaml`，以实际保存结果为准。排查地图与路线问题时，提供同一地图的这些文件、对应 `routes.db`、完整后台日志、页面操作记录及故障时间；不要混用不同建图批次的文件。备份正在使用的 SQLite 数据库前应先正常停止后台，或使用 SQLite 一致性备份方式。

## 7. 常见问题

### 7.1 建图页障碍图和二维图正常，但全局三维图一直等待

如果日志中能看到网页订阅 `/global_mapping_cloud`，却看不到下面这行：

```text
Global mapping cloud preview enabled on /global_mapping_cloud
```

说明前端已更新，但运行中的 Small Point-LIO 仍是旧安装产物。若日志路径还出现 `/home/a/nav_ws`，必须停止旧的常驻后端，在一个未加载旧工作空间的新终端中重新构建并启动：

```bash
# 先在原终端按 Ctrl+C，确保旧 launch_manager 已停止。
cd /home/a/3dnav
source /opt/ros/humble/setup.bash

# 不要在这里 source /home/a/nav_ws/install/setup.bash。
colcon build --symlink-install --parallel-workers 4 \
  --packages-select \
  lzp_interface small_point_lio lzp_service icp_registration \
  robot_navigation2 octomap_server2 car_vel

source /home/a/3dnav/install/setup.bash

ros2 pkg prefix car_vel
ros2 pkg prefix lzp_service
ros2 pkg prefix small_point_lio
python3 -c 'import lzp_service.launch_manager_service as m; print(m.__file__)'

# 还要确认安装产物确实包含本次源码，而不是同一路径下的旧构建。
LIO_PREFIX="$(ros2 pkg prefix small_point_lio)"
strings "$LIO_PREFIX/lib/small_point_lio/small_point_lio_node" | \
  grep -F 'Global mapping cloud preview enabled'
CAR_PREFIX="$(ros2 pkg prefix car_vel)"
grep -n 'terrain_grid_node' "$CAR_PREFIX/share/car_vel/launch/mapping.launch.py"
cat "$LIO_PREFIX/share/small_point_lio/global_mapping_contract.txt"
FILTER_PREFIX="$(ros2 pkg prefix octomap_server2)"
cat "$FILTER_PREFIX/share/octomap_server2/mapping_filter_contract.txt"
```

以上前缀和模块路径都必须位于 `/home/a/3dnav/install`。两个 contract
必须分别输出 `global_mapping_cloud_temporal_filter_v2` 和
`mapping_cloud_temporal_filter_v1`。再启动：

```bash
ros2 launch car_vel launch_manager.launch.py
```

进入建图页后确认：

```bash
ros2 topic info /global_mapping_cloud --verbose
timeout 8 ros2 topic echo /global_mapping_cloud --once --field width
ros2 topic hz /cloud_mapping_dynamic
ros2 topic echo /mapping_cloud_filter/status --once
```

日志还应出现 `terrain_grid_node` 进程和全局点云发布器。启用动态过滤时应出现
`Temporal mapping filter enabled` 启动提示。`/mapping_cloud_filter/status`
中的 `static_points`、`dynamic_points`、`filtered_ratio` 应持续更新；容量正常时
`capacity_exceeded` 必须为 `false`。若已在设置页关闭对应过滤，不应只凭缺少启用提示判为故障。仅刷新网页无法替换已经运行的旧 ROS 后端。

若重启后又回到旧路径，继续检查 shell 或 systemd 的自动加载项：

```bash
grep -Rsn -- '/home/a/nav_ws' \
  ~/.bashrc ~/.profile ~/.config/systemd/user /etc/systemd/system 2>/dev/null
```

### 7.2 lzp_interface 无法创建符号链接

错误：

```text
failed to create symbolic link ... because existing path cannot be removed:
Is a directory
```

原因是旧构建目录与 `--symlink-install` 冲突。只清理该包并重新构建：

```bash
cd /home/a/3dnav
rm -rf build/lzp_interface install/lzp_interface

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_interface
```

成功后重新执行全量 `colcon build --symlink-install --parallel-workers 4`。

### 7.3 找不到 OctoMap

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

cd /home/a/3dnav
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --symlink-install \
  --packages-select octomap_server2 \
  --cmake-clean-cache
```

`io features related to pcap will be disabled` 只是警告；安装 `libpcap-dev` 后可消除。

### 7.4 找不到 rosbridge_server

错误：

```text
package 'rosbridge_server' not found
```

处理：

```bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-server

source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
ros2 pkg prefix rosbridge_server
```

这是运行时预编译包，安装后不需要重新编译项目。

### 7.5 找不到 Nav2

错误通常会包含 `nav2_bringup`、`controller_server`、`bt_navigator` 或其他 Nav2 包未找到。

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

安装后重新加载环境即可，不需要重新编译 Nav2。

### 7.6 其他包显示 Aborted

并行构建时，一个包 `Failed` 会使其他包显示 `Aborted`。首先处理第一个 `Failed` 包，然后再次运行全量构建。`Aborted` 不等于这些包自身存在代码错误。

### 7.7 权限错误

不要使用 `sudo colcon build`。如果历史构建导致文件归 root 所有：

```bash
sudo chown -R a:a /home/a/3dnav
```

### 7.8 没有 Livox 话题

依次检查：

1. 实际设备是否为 MID-360S。
2. 启动文件是否为 `msg_MID360s_launch.py`。
3. 主机网卡是否配置为 `192.168.1.50/24`。
4. 雷达实际地址是否与当前驱动配置一致（MID360s 默认 `192.168.1.130`）。
5. `ldconfig -p | grep livox_lidar_sdk` 是否能找到 SDK。
6. 防火墙、网线和路由是否正常。

### 7.9 打架检测一直显示等待，窗口只有 1/8 帧

先读取第 6.5 节的行为状态，区分“未加载”和“未形成连续样本”。已知旧问题是把打架采样放在耗时姿态推理后：姿态每次约 1 秒，超过打架的 0.6 秒采样间隔，窗口反复清空。

当前网关使用独立跌倒/打架进程，并在采集时缓存最多 8 帧真实画面。按第 4.4 节同步网关和整个行为运行时、重建 `car_vel` 并重启，不能仅刷新网页。

| 状态 | 排查方向 |
| --- | --- |
| 没有 `workers` 字段 | 可能仍在运行更新前的网关，核对源码、安装路径与后台进程 |
| 分支 `error` / 不可用 | 查看错误、`diagnostics_by_branch` 和后台 `[行为识别]` 日志，核对独立环境及模型文件 |
| `warming` 且帧数不足 | 查看真实相机采样是否连续、是否切换来源或频繁重置 |
| `stale_results` 持续增长 | 模型返回超过有效期；查看分支 `timing_ms` 与 `last_result_latency_seconds`，测量 CPU 负载 |
| `unknown` | 有结果但证据/分数不足；不能改成正常或靠重复帧凑满窗口 |

本次采样修复不放宽模型阈值、视频断帧限制或结果有效期；跌倒分支是否获得足够连续姿态仍取决于实际帧率。

### 7.10 热成像重启后无图，插拔 USB 后才出现

先单独执行：

```bash
lsusb -d 2bdf:0102
```

若没有输出，说明该 VID:PID 的设备尚未被系统枚举；先检查供电、数据线、USB 端口及开机内核日志。不能靠重装 Python、刷新网页或仅重启 SDK 保证恢复未枚举的硬件。保存当前启动日志用于对比：

```bash
sudo journalctl -k -b --no-pager > "$HOME/thermal-usb-boot.log"
```

若 USB 已识别，再查看第 6.5 节热成像状态：分别排查 bundle/适配脚本缺失、udev 权限、其他程序占用和 SDK 错误。不要同时开启厂商预览与项目采集进程，具体见[热成像部署与恢复](src/ros2_humble_main/src/car_vel/THERMAL_CAMERA.md)。

### 7.11 M701 串口消失或导航显示运行却不移动

M701 的串口错误与机器人控制分别排查：

- USB 有转换器但没有串口，检查 `lsusb -t` 和内核日志；如果日志明确是 BRLTTY 抢占，且设备不依赖盲文辅助功能，可按 M701 专题处理服务屏蔽，不能只反复安装 CH341 驱动。
- `Permission denied` 检查运行账号的 `dialout` 组及后台实际继承的组；加组后要重新登录并重启旧后台。
- 串口打开后超时，检查传感器供电、RS485 接线、地址和波特率，不再把它当成驱动未加载。

导航方面，“目标已接受”“路径已显示”或 `/cmd_vel_safe` 有 20 Hz 消息，都不能证明有非零速度发送给本体。先记录定位、速度仲裁与本体状态，不要通过换页面或重复启动来代替诊断：

```bash
timeout 8s ros2 topic echo /localization/status --once --full-length
```

```bash
timeout 8s ros2 topic echo /navigation/velocity_guard --once --full-length
```

```bash
timeout 8s ros2 topic echo /dog_status --once --full-length
```

保留完整日志中的 `NAV_GOAL`、`NAV_BODY`、定位和地形错误；结合[NAV 运动诊断说明](src/ros2_humble_main/src/car_vel/NAVIGATION_MOTION_LOGS.md)判断卡在规划、定位保护、速度仲裁还是本体执行。不要只凭单一提示判断为 YOLO 导致停车。

### 7.12 初始精修成功，但一直等待定位或 Nav2

若日志先出现精修成功，再出现 `定位TF权限同步失败：'dict' object is not callable`，
是新增统一定位节点与 ROS 节点内部参数字典的名称冲突。2026-10-28 的修复已将
自定义方法改名；已部署过前一版三维定位的主机按第 4.4 节重建 `car_vel` 并重启后台即可，
不能只刷新页面。若首次部署相关 Action 接口，仍需按第 4.4 节编译四个包。

地图/定位服务已启动且地图加载成功后，可以先设置并确认初始点，再等待 Nav2 全部激活。
精修成功还需统一定位输出确认，不能把单条“GICP 成功”当作整个初始化已完成。
保持静止检查 `/localization/status` 的 `ready`、`error` 和本次 `seed_request_id`，
完整步骤见[三维定位修复与验证](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。

### 7.13 多个初始配准候选都被质量检查拒绝

这与第 7.12 节的字典调用错误不同。先查看各候选的 RMSE、重叠率、垂直/倾斜修正和
搜索边界；接近门限不等于可以直接放行。当前星玑三维初始精修采用独立的
NDT/GICP 预算（20 / 40 次外层、12 次 GICP 内部优化），原二维、钢镚和跟踪预算保持原值。
总时间、点数、匹配质量及歧义检查没有放宽。

此次更新涉及 C++ 配准节点和统一定位状态提示，已部署主机需要同时重建
`icp_registration`、`car_vel` 并重启。新日志记录初始坐标、候选姿态、实际迭代次数和
具体拒绝原因；仅提高预算不能保证错误地图、初始点或外参不一致时匹配成功。
详见[初始精修预算与记录方法](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。
后续 bag 应包含 `/relocalization/request` 和 `/relocalization/status`；只录雷达、TF
而没有初始请求坐标，无法精确复现同一次配准。

### 7.14 初始定位成功、已行走，随后因弱扫描永久停车

若日志先出现可恢复的 `scan_matching_quality` 回退，约半秒恢复 healthy，
但统一定位随后变为 `initialization_required` 并锁定零速，是两个保护环节对
“暂停”和“定位重置”的处理冲突。明确已回退且无独立运动异常的质量告警，
现在有最多 1 秒的锚点连续性窗口；期间仍由 LIO/导航暂停独立输出零速，
健康后接纳新速度继续原任务。真正重置、跳变、TF/匹配过期和急停仍保持保护。
此处不改变星玑二维及钢镚的定位输出逻辑，也不降低地形或配准门限。

同步源码后停止后台，重建 `car_vel lzp_service` 并重启。检查
`/localization/status` 的 `lio_quality_pause` 与 `/lio/motion_paused`，
前者说明定位连续性窗口，后者决定 LIO 是否允许运动。
路线预览过期提示也会显示具体变化项或起点偏移；保持静止重新预览，不能复用失效预览。
具体更新命令与逐条检查步骤见[三维定位文档](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。

### 7.15 三维跟踪高度偏差触发停车，稍后匹配成功却无法继续

20:43:30 的记录属于三维匹配校验/期限问题，LIO 本身保持 healthy。
星玑三维连续跟踪现有独立阈值：RMSE 0.22 m、高度修正 0.45 m；初始定位和
原有模式仍为 0.20 / 0.35 m。正常匹配期限从 2 s 改为 3 s，之后最多再零速
复核 2 s；当前一次新的完整可信匹配即可恢复原任务（原两次确认已调整）。窗口不滑动延期，真正跳变、
故障、重置或超窗仍锁停，晚到结果不自动重启。最终平面变换的校验仍保留。

本次需停止旧后台并重建 `icp_registration lzp_service car_vel`，刷新前端。
检查 `/localization/status` 的 `ready`、`recovering`、`recovery_remaining_sec`
及 `recovery_matches`；恢复时需新控制器速度，人工暂停/取消仍有效。
详细参数、更新和现场检查见[三维定位说明](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。
0.45 m 是配准容差，不是可跨台阶高度；持续高度不一致仍需排查，离线回归不能替代实机验收。

### 7.16 初始点和预览成功，但巡检开始时报 TF/里程计不连续

短暂 TF 变旧、采样间歇和查找失败已合并为一条有界零速恢复路径，清除了
“一次超过 0.3 s 就取消”和“样本间隔超过 0.5 s 就清锚”的冲突处理。
最长 1.5 s 的短间歇需核验位移/朝向，并在固定窗口内得到 3 个新鲜 TF 样本
和 2 次新三维匹配；原任务保留，但不会使用旧 TF 伪装恢复或输出旧速度。
定位心跳短时延迟也先零速等待，持续掉流/真实跳变仍停车。

当前显式路线开始和有效巡检授权支持有界等待并完整复核；停止或替换预览仍能
阻止晚到启动。连续匹配预算为 2.0 s，上一版质量阈值不变。
新状态包含 `sensor_recovering`、`sensor_stream_ready`、`lio_timing` 的实际时间/位移数值。
更新步骤和已清理规则见[三维定位说明](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。

### 7.17 匹配恢复后仍未就绪，或轻微定位修正就使路线作废

2026-10-29 的后续修复针对星玑三维导航，地形导航的单点/多点和巡检共用：

- 本体/模式心跳短暂迟到时保留三维定位归属和锚点，零速等待；明确的模式、地图、本体切换仍使旧证据失效。
- LIO数据流在最多3 s内用3个真实新TF独立完成复核；身份/地图匹配复核仍有限时约束。地图等待不再使已恢复的LIO误报超时，真实采样间隔后的地图复核仍受原锚点总期限约束（见7.25）。
- 三维匹配请求范围与接受范围统一为 0.20 m / 6°。超出范围的候选不写入锚点；轻微超限先零速复核，一次新合格匹配后恢复，大幅跳变或超时仍停止任务。
- 修复“匹配 accepted、定位却未就绪”的矛盾。已确认的不可自动恢复故障不会被晚到结果覆盖。
- 启动仍需人工确认；使用当前可信位置检查到预览路径的连接段（最长 0.35 m），并逐格检查连接段和整条路线的 0.45 m 占地范围。地图/地形/本体/配置改变、朝向变化超过 15°、连接段不安全仍拒绝，并显示具体原因。

人工停止、暂停和取消始终优先；恢复不会重新执行已取消的任务。星玑二维与钢镚配置不变。
本轮只改 Python 包，更新源码并停止旧后台后执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select car_vel lzp_service
source /home/a/3dnav/install/setup.bash
```

按原方式重启后台，重新加载地图、确认初始点并预览。先验证短平地路线，再验证多点和巡检。
状态和录包检查步骤见[三维定位说明](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)。
离线测试验证逻辑，不等同于小电脑上的 ROS/PCL 与实机稳定性验收。

### 7.18 预览后必须18秒内开始，或巡检报“路线预约超时”

当前版本已取消前端18秒、后台20秒预览倒计时和待启动巡检授权的120秒期限。
预览按地图加载批次、地形地图版本、本体、路线和配置变化失效；等待检查路线本身不会作废预览。
点击开始时仍检查当前定位、朝向与起点连接。定位需刷新时由本次人工开始请求进行有界刷新和复核，无需单纯因等待而重新预览。

“路线预约超时”是另一个问题：后台请求最多等待12秒，当前已改为锁外等待，避免状态回调占满线程后无法处理回复。
地图版本相同的预览复用已验证静态路径，只重验当前位置到路径的连接段及占地；新规划仍检查整条路径。
等待预约期间可以点击“取消启动”；地图、路线或检测点配置变化也会阻止提交，晚到预约仅用于清理，不创建任务。
日志 `INSPECTION_START` 给出 reserving/committed/rejected/late_reservation_revoke_requested 与实际耗时。

### 7.19 多点路线走到一半，LIO临时隔离后却永久取消

2026-10-29 12:38 的日志中，低速行走在约0.499秒扫描间隔触发一次 position_step 隔离，
LIO计划验证健康扫描后恢复，但统一三维定位曾立即撤销锚点，先取消了路线；随后成功的配准也无法接回。
当前对明确隔离、低速且诊断完整的短暂 step 警告保留锚点最多1秒，运动仍保持零速；原始健康状态、
实际TF连续性和可信匹配恢复后继续原目标。高速、大跳变、重置、故障和超时不使用这项宽限。

对仅有短暂TF变旧、采样连续、原匹配仍有效的情况，恢复3个真实新TF即可复核，不再额外强求两次新地图匹配。
较大采样间隔或真正匹配失效仍需匹配复核；人工停止、取消和急停始终优先。
地形导航和巡检共用同一个路线执行、定位/速度判断；巡检仅保留检测点停留、识别和报告等任务行为。
两页三维点云视图的机器人/候选箭头缩放已统一为0.25。

本轮更新 `car_vel`、`lzp_service`，无C++/消息接口或本体步态配置变更。按7.17节重建并重启后，浏览器强制刷新。
先测试短平地多点，再测试同一路线巡检；离线回归不能替代现场验证。

### 7.20 启动后一直等待 Nav2，存在两个 nav2_container

2026-10-29 14:05 的运行同时存在旧容器 PID975382（已脱离原启动树）和新容器 PID980502。
规划器、AMCL和导航器没有就绪；这次尚未加载地图和发送目标，不是地形限速或LIO中途保护。
旧实现仅等待 `ros2 launch` 主进程退出，会丢弃仍有子进程的记录；主进程消失后再查询其进程组也会失败。

现在由专用进程组看护启动树：主进程退出或后台意外结束时清理其子进程；正常停止先发送SIGINT，
等待整组退出，超时才清理自己拥有的组。退出检查忽略僵尸进程，用所属标记避免PID/组号复用误伤。
启动前检查同用户、同ROS域的旧 `nav2_container`；未确认所属的容器只报告PID并阻止叠加启动，不自动终止。
只读启动诊断增加 `process_conflict` 和 `container_processes`，不再等60秒才说明存在容器冲突。

同步代码后停止旧后台，重建 `lzp_service car_vel`。本次已存在的旧版残留可在确认所有任务结束后重启小电脑一次清除；
以后只启动一套后台，不叠加运行 `navigation2.launch.py` 或另一套 Nav2。具体命令和验收见 [启动恢复](src/ros2_humble_main/src/car_vel/STARTUP_RECOVERY.md)。
这不改变导航速度、地形阈值、二维规划或本体步态。Linux实际进程/信号验收仍需在小电脑执行。

### 7.21 运行中三维匹配恢复超时，不能只看最终“已锁定零速”

2026-10-29 15:34—15:39的日志与之前15:25—15:27的粘贴记录是不同运行。
新日志15:39:17最终为`recovery_timeout`：LIO/TF仍新鲜，但约5秒没有可接受的地图匹配。
一项完整三维候选曾通过RMSE0.158m/重叠率90.6%，投影成Z=0的yaw-only变换后重检同一三维点云，
RMSE升至0.223965m而超过0.22m；另一些候选的垂直修正0.47—0.57m超过0.45m。
这是三维校验与平面变换之间的模型不一致/质量拒绝证据，不等于机器人真实垂直跳动了该距离。
地图高度基准、LIO高度漂移与外参各占多少，仍需同步点云/TF数据核对。

本轮仅修复故障原因丢失：`/localization/status.failure`和`LOCALIZATION_FAILURE`冻结首次失效原因、
当时有效匹配时间/质量、LIO状态和地图批次；后续锚点重置或LIO恢复不会覆盖。
最终“锁定零速”提示包含原始拒绝详情，只有新一次通过校验的人工初始定位才清除此记录。
当时尚未完成模型改造；后续已按7.22完成刚体几何链路修改。此前的LIO短时隔离/残留Nav2容器是另两类问题。

### 7.22 星玑三维匹配保留完整变换，修复投影后的重复拒绝

仅星玑`terrain_3d`启用`se3_map_odom_v1`：通过校验的完整三维候选直接保留，
初始结果、后续粗位姿、LIO组合、定位消息与唯一`map→odom`均保留Z及姿态。
Nav2继续使用XY/朝向做平面控制，原星玑二维/钢镚AMCL路径和速度配置保持原样。
地形导航和巡检共用后端路线/定位/速度恢复链；地形页不再另用旧AMCL协方差重复否决统一三维定位。
地图先验对照使用点云采集时刻的完整TF。星玑三维把小范围缺失回波与实测沟槽分开，
较大未知区、实测坑洞、真实跳变、人工停止和恢复超时仍停车。

此次没有继续放宽RMSE、重叠率、XY/朝向修正或恢复时间门槛。新增Action结果`transform_model`，
初始及跟踪均检查模型版本。**必须停止旧后台并重建以下四个包，不能仅刷新网页或只重建car_vel。**

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select lzp_interface icp_registration car_vel lzp_service
source /home/a/3dnav/install/setup.bash
python3 src/ros2_humble_main/src/car_vel/scripts/check_3d_navigation_install.py
```

自检同时检查当前工作区、生成接口、原生动态库中的模型标记和Python几何；它不启动节点或发送运动。
通过后只启动一套后台，浏览器Ctrl+F5，重新加载地图、确认初始点、预览并人工启动短平地路线，再验证同路线巡检。
启动日志应有`REGISTRATION_CONTRACT model=se3_map_odom_v1`；三维定位状态应有相同`transform_model`。
已有地图/路线格式未改；不需要为了此次修复重新建图。

离线已覆盖3D结果→初始确认→唯一TF→下一次请求、连续台阶相对运动、路线/最终速度恢复、版本混用拒绝、
二维/钢镚隔离及地形证据。完整Linux ROS/PCL构建与真实机器人效果仍须在小电脑验收，不能据此承诺所有场地永不停。

### 7.23 局部地图粗采样不足导致行进中定位超时

17:35这次日志与7.22不同：新三维模型已生效，前面的LIO复核均已恢复，路线推进到第10/19点；
最后是原生节点在地图ROI准备阶段连续失败，5秒未能产生新匹配而停止。
`shiyanshi-wai`同一PCD在停止区域约14,589个原始点、7,280个15cm细采样点，
45cm粗采样仅1,163个点，刚好低于1,200门槛。不能将这个错误等同于地图没有覆盖。

星玑三维的原生robust请求现在对**同一ROI**进行有界重采样：粗点不足而细点充足时，
依次尝试原粗网格的75%、50%和细网格下限，最多三次；不低于地图/细采样分辨率，保持原点数上限。
原点数门槛、细配准RMSE/重叠率/修正量/歧义校验和真实失配后的停车流程均保持。
二维和钢镚的原生legacy路径不启用该重试。

同步完整源码、停止旧后台后，按7.22重建四个包并运行安装检查。
安装检查现在也校验原生动态库中的`adaptive_target_sampling_v1`，旧SE3版本仅刷新网页不能通过。
启动日志应有`REGISTRATION_TARGET_POLICY model=adaptive_target_sampling_v1`；
每次准备的`REGISTRATION_TARGET`记录原始/细/粗点数、重试前点数、网格尺寸、范围和重采样次数，
最终失败信息包含这些点数，便于区分过度抽稀和真实缺图。

Linux编译完成后可只运行本次采样测试（其中一个使用真实PCL体素过滤）：

```bash
ctest --test-dir build/icp_registration -R '^target_sampling(_pcl)?$' --output-on-failure
```

无需重新建图或修改路线。验证仍先用短平地路线经过原停止区域，再验证同路线巡检。
短时LIO数据延迟、真实地形危险或错误匹配仍可能停车；此次不以延长超时或放行坏定位掩盖准备阶段错误。

### 7.24 星玑三维恢复改为一次完整可信匹配

按用户要求，星玑三维导航的地图匹配复核从连续两次调整为**一次新的完整匹配通过即允许恢复**。
这一次仍包括原生多时间戳一致性、RMSE/重叠率/修正范围/歧义和统一定位的时间/身份/位姿校验；
不是取消校验，也不是只看一帧。数据流连续性检查、原固定截止时间、零速等待和真实跳变停车保留。
恢复后仍只接受新的控制器速度，人工暂停/停止、已取消任务和超时后的锁定不被单次结果解除。
地形导航和巡检共用同一定位链；二维与钢镚流程保持原样。

本次仅修改Python定位策略，无需修改地图或路线。同步源码、停止旧后台后重建`car_vel`并重新加载环境、启动后台。
确认星玑三维`/localization/status`中的`recovery_matches_required`为1。
这项调整减少等待第二次匹配造成的中断，也减少了时间上的重复确认；不能保证解决地图与现场本身不一致的问题。

### 7.25 巡检预热期间已恢复的LIO被误报超时

19:19这次失败中，单次匹配规则已生效；但旧LIO计时器仍在等地图匹配完成。
LIO经历约1.3s短采样间隔后已经恢复9个新样本，数据年龄约0.14s，却在16.777秒被判
`sensor_recovery_timeout`并清除锚点；17.078秒地图匹配RMSE0.172m、重叠率87.4%通过，结果已无法接回。

现将两种证据分开：3个真实连续新TF完成LIO复核后立即结束其计时器；实际大采样间隔仍要求一次
新的完整合格地图匹配。匹配复核使用自己的固定窗口，每次不超过原3s上限，也不超过原锚点
`matching_coast_sec + matching_recovery_sec`总边界（默认5s），失败结果/心跳不续期。
0.3s新鲜度、1.5s实际掉流、真实位姿跳变、质量和人工停止检查保持。
等待地图匹配时保持零速，恢复只接受新控制器速度；地形导航与巡检共用此流程。

另修正两处诊断：已确认初始点但尚未导航时，真实失效也保存首个原因；启动失败后本来没有活动路线的
清理响应改为普通提示，其他停止/权限失败仍报警。YOLO负载是否导致采样间歇尚需现场CPU/调度证据，不能仅凭同时发生认定。

本次同步源码、停止旧后台后重建`car_vel lzp_service`并重新加载环境。既有地图/路线和原生接口不变。
地图20s补收超时属于另一条栅格接收链路；本次日志中后续加载成功，不能把它归为LIO或配准失败。

### 7.26 巡检启动报“等待导航空闲后同步控制器计算配置”

2026-10-30 14:22这次启动被控制器计算量前置检查拒绝。实际`FollowPath.batch_size`已经回读为400，
参数回读年龄0.75s且controller_server代际未变；问题是检查节点把固定导航模式当成2s心跳，
短暂未收到重复事件就将`terrain_3d`清空成未知，进而把已确认参数标为未就绪。
14:22:08.534拒绝后约0.16s，同一400参数又恢复ready，没有证据表明需要重新写入。

现改为缓存有效的持久导航模式，只有显式新模式或无效配置消息才改变它。
实际本体状态/活动状态仍检查新鲜度，真正的模式切换仍必须恢复正确参数；
未决远端写入、参数回读和运动中禁止改参的限制保留。没有修改速度、指令期限或关闭启动检查。
状态新增`mode_context_source=latched_configuration`、`mode_message_age_sec`及`activity_confirmed`，
明确区分配置消息年龄和控制器活动证据。未知身份的提示也不再一律误称“等待空闲”。

同步源码、停止旧后台后重建`car_vel`并重新加载环境即可；无需重建地图、修改路线或手动删除参数权限记录。
在`/navigation/controller_timing`确认正确的模式、本体和ready，之后重新预览并启动巡检。

### 7.27 巡检反复显示不同“零速原因”

2026-10-30 14:39/14:40日志中，多个提示属于同一停车链：LIO/TF数据过期→定位复核停车→清除旧速度→等待新控制器速度。
其中也有真正的控制器速度中断和地形暂未放行。首轮数据年龄1.509s触发原1.5s源流边界；第二轮有多次5Hz控制循环超时，
五帧配准因点云时间戳/跨度不满足要求被拒绝，最后5s没有新合格锚点而停车。不同于之前“新数据已恢复却被旧计时器误杀”的问题。
日志不足以区分CPU/线程争用、驱动/传输或调度延迟，不能仅凭停车后的低负载断言运行中没有性能问题。

已修正显示误判：旧速度被主动清除执行资格、但平滑速度仍按时到达时，不再附加“平滑器未及时输出”。
同一次静止阶段的复核/速度清理/零速等重复原因，操作记录按类型10s去重；实时状态仍每次更新。
新的危险/定位失效立即记录，实际下发非零速度或任务结束后重置去重，不改变任何运动门禁或速度有效期。

后台已关时不用为采集重新启动。下次正常使用，可在机器狗保持停止的新终端运行只读采集，再照常操作识别/巡检：

```bash
cd /home/a/3dnav
bash src/ros2_humble_main/src/car_vel/scripts/collect_navigation_load.sh
```

默认采集60次1秒负载快照，保留CPU/内存和高占用进程，文件保存在`~/nav_runtime_load_*.log`；可Ctrl+C提前结束。
与同一时段`nav_debug_full.log`一起判断负载和数据延迟来源。采集不调用ROS服务或发送运动，显示修复也不等于数据中断已解决。
本次更新源码、重建`car_vel`、重启后台并强制刷新网页即可更新显示；地图/路线不用重做。

### 7.28 巡检启动在定位复核完成前提前失败

2026-10-30 15:01这次日志没有最终定位失效：路线在15:01:19.445被启动调用方的独立3.1s等待拒绝，
当时统一定位仍处于有效复核窗口内，约0.372s后恢复ready。启动失败不是前一次15.9cm起点偏移的延续；
15:01:09的新预览已经通过。反复出现的短时TF新鲜度延迟仍需现场负载/传输证据定位。

星玑三维的共享路线启动现在参考定位模块的有效剩余复核时间，扣除状态接收后的时间，
在原3.1s等待之上最多保留3s尾窗，累计复核等待不超过约6.1s。这里只延长待确认启动请求的等待，
不延长定位模块自身的恢复截止时间；定位真正失效时立即拒绝，未就绪前不会提交运动目标。
重复进入复核也不能无限占用启动请求，巡检暂停/取消、授权代际、地图身份和手动预览替换仍重新检查。
等待用尽时明确提示“路线尚未启动、未创建活动路线”，不会暗示不存在的路线将自动恢复。
地形导航多点和巡检使用相同实现；二维和钢镚分支不变。

同步源码、停止旧后台后，在小电脑执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_service
source /home/a/3dnav/install/setup.bash
```

之后只启动一套后台，重新加载地图、确认初始点并预览，再人工启动短平地路线及同路线巡检。
地图、路线和原生接口均未改变。本修复覆盖启动等待与定位恢复的时序冲突，不能代替小电脑运行负载和真实掉流的验收。

### 7.29 星玑三维行进中反复零速与五帧恢复超窗

2026-10-30 15:18–15:20记录中，99次TF过期状态有98次源数据年龄不超过0.36s，
旧0.3s边界导致反复停车复核。最后一次position_step的dt约0.5s，位移约0.209m，
超过固定0.15m门槛；五帧健康扫描用了1.484s，而定位锚点的1s保留窗口已提前结束。

星玑三维运行策略现为：

| 项目 | 当前行为 |
| --- | --- |
| LIO源TF新鲜度 | 0.45s；保留真实测量时间戳，不重发旧速度 |
| 已识别短暂LIO健康恢复的锚点保留 | 固定最多3s，重复心跳不续期；期间仍零速 |
| 原生平面位移检查 | 活动星玑三维内为max(原0.15m, min(配置线速度上限,0.6m/s)×dt)，dt≤0.5s，故门槛最多0.3m |
| 模式限定 | 仅已确认定位、身份新鲜且有活动任务时发补偿状态；状态中断1s或显式退出即恢复原位移门槛 |

独立线速度/转速/垂向/数值故障检查、1.5s真实掉流、三帧流恢复、一次完整可信地图匹配、
单一TF发布权、人工停止/取消和实时避障保留。定位状态心跳与控制器速度的0.3s期限不变。
地形导航多点和巡检共用该流程；二维和钢镚的原生门槛及定位转发保持原样。
日志新增`runtime_timing_model`、`lio_timing.freshness_limit_sec`、`lio_health_pause_limit_sec`和
`interval_compensation`；首因记录保留最近LIO健康明细，避免仅看到null。

**本次有原生C++改动，必须编译small_point_lio，不能只刷新网页或仅更新Python。**
同步完整源码、停止任务和旧后台后，在小电脑执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select small_point_lio car_vel
source /home/a/3dnav/install/setup.bash
python3 src/ros2_humble_main/src/car_vel/scripts/check_3d_navigation_install.py --runtime-timing
```

安装检查只读，核对SmallPointLIO动态库含`xingji_3d_interval_v1`及Python对应版本。
随后启动一套后台，强制刷新网页、重新加载地图并确认初始点，预览后人工启动短平地路线；
再在巡检页验证同一路线。现有地图和路线不需重做，已取消的任务不自动复活。

启动日志应有`ODOMETRY_INTERVAL_POLICY model=xingji_3d_interval_v1`。
星玑三维状态应有`runtime_timing_model=xingji_3d_runtime_timing_v1`、
`lio_timing.freshness_limit_sec=0.45`、`lio_health_pause_limit_sec=3.0`；
`interval_compensation`只在满足上述活动任务条件时为true。若再次停车，保留同一时段完整日志与7.27的负载采集文件。

Linux编译后可验证原生门槛与租约：

```bash
ctest --test-dir build/small_point_lio -R '^test_interval_guard$' --output-on-failure
```

Windows离线回归不能替代完整Linux ROS/PCL构建和实际负载/机器人验收；当前仍未确认源流间歇的CPU、调度或驱动原因。

### 7.30 巡检最后一点已完成，页面仍显示导航中且跳过失败

2026-10-30本次日志的19个点（名称目标点3–21）均已由后台判定到达；最后一点在15:55:49.240到达，
15:55:49.277路线执行完成。15:57:02再点击跳过时，路线后台已空闲，因此返回`No route navigation task is active`。
问题是路线完成发布前清除了owner_token，巡检管理器因身份不匹配忽略了完成消息；
而最后一个点晚到的release回复还可能把完成状态改回navigating。

现使用既有状态字段发布带任务标识和序号的`route_completed`事件，活动控制权仍正常释放。
巡检按任务身份及本次路线的事件基线消费完成，重复或旧事件不重复结算；短路线先完成、Start回复后到也能收尾。
最后release/skip/resume回复不能覆盖终态或轮次间隔；暂停期间完成仍等待人工继续，取消屏障保持优先。
无活动路线时不再发送虚假的跳过请求，结束后已有页面逻辑会禁用控制按钮。
本次没有取消三维在途跳点的路径复核限制；有效检测点的跳过仍走原流程。

本次无需改变接口或重建地图。先终止这次旧任务、停止旧后台并同步源码，再执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_service
source /home/a/3dnav/install/setup.bash
```

重启一套后台并刷新页面，验证2–3个普通点的短巡检：最后点释放后应显示“巡检任务全部完成”、
进度100%，跳过/暂停/继续禁用且可新建任务；再验证包含检测点和多轮间隔的任务。
旧后台已经丢失的完成事件不能在重启后凭空恢复，不要直接继续旧的异常任务。
离线覆盖生产者到巡检消费者、异步回复顺序、重复/其他任务事件、暂停/取消及原有页面按钮逻辑；
目标小电脑仍需实际验收。

## 8. 核心导航的最短部署流程

完成系统 ROS、基础构建依赖、Livox SDK、网络配置和源码复制后，可按以下顺序执行。此流程不包含相机 SDK、视觉/行为依赖和 M701 硬件准备；使用这些功能还需完成第 2.6、2.7、3.3 节。

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

cd /home/a/3dnav
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

colcon build --symlink-install --parallel-workers 4
source install/setup.bash

ros2 launch car_vel launch_manager.launch.py
```

首次部署建议按“雷达 → LIO → 建图 → 定位 → Nav2 → 后端 → Web 前端”的顺序逐级验证，确认硬件、话题和 TF 正常后再配置开机自启。

## 9. 专题文档

| 文档 | 适用内容 |
| --- | --- |
| [本体选择](src/ros2_humble_main/src/car_vel/ROBOT_BODY_SELECTION.md) | 星玑 / 钢镚 ZSL-1 点足连接、切换与 SDK 边界 |
| [雷达网络](src/ros2_humble_main/src/car_vel/LIDAR_NETWORK.md) | 页面设置、实际网卡与驱动配置的关系 |
| [星玑三维定位](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md) | 初始精修、持续定位、防跳变、TF 交接和验收步骤 |
| [固定越障策略](src/ros2_humble_main/src/car_vel/XINGJI_FIXED_OBSTACLE.md) | 星玑导航/巡检模式与速度策略 |
| [转向连续性](src/ros2_humble_main/src/car_vel/TURN_CONTINUITY.md) | 转弯、步态与相关控制检查 |
| [运动诊断日志](src/ros2_humble_main/src/car_vel/NAVIGATION_MOTION_LOGS.md) | 路线启动、零速与本体执行链排查 |
| [地图点云编辑](src/ros2_humble_main/src/car_vel/MAP_CLOUD_EDIT.md) | 二维修图与对应三维点云删除、恢复 |
| [点云显示](src/ros2_humble_main/src/car_vel/WEB_PREVIEW.md) | 实时预览与完整保存 PCD 的显示机制 |
| [普通摄像头](src/ros2_humble_main/src/car_vel/USB_CAMERA.md) | USB / 本体图传切换、依赖与设备选择 |
| [热成像](src/ros2_humble_main/src/car_vel/THERMAL_CAMERA.md) | SDK bundle、权限、高温检测和 USB 恢复 |
| [M701 环境检测](src/ros2_humble_main/src/car_vel/M701_RS485.md) | CH341/CH348、RS485、串口权限与数据状态 |
| [行为识别](src/ros2_humble_main/src/car_vel/BEHAVIOR_RECOGNITION.md) | 跌倒/打架开关、告警与部署 |
| [行为运行时](src/ros2_humble_main/src/car_vel/person_behavior/README.md) | 独立进程、连续采样修复、协议和性能限制 |
| [启动恢复](src/ros2_humble_main/src/car_vel/STARTUP_RECOVERY.md) | 后台启动、服务恢复与部署排查 |

文档中的命令以小电脑 Ubuntu 环境为目标；Windows 开发机的离线回归不能代替 Linux 完整编译、真实模型性能测试、雷达/相机验证及机器人现场验收。
