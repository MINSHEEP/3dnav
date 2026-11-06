# 3dnav 部署、编译与运行手册

更新日期：2026-11-06。

交付与复测入口：[全功能验收清单](FIELD_ACCEPTANCE.md) · [排查、备份与恢复手册](TROUBLESHOOTING.md)。先按验收清单中的时间预算选择测试，再逐项记录未测范围。

## 当前璇玑集成与构建入口

三种底盘（星玑、钢镚 ZSL-1 点足、璇玑 Hypertron 轮式四足）由同一个 3dnav 仓库管理。璇玑 SDK 1.1.1 的头文件、x86_64 厂商库和三个 ROS 包已纳入 `src/ros2_humble_main/src/driver/astrall/`，不再依赖旧 SDK 工作空间；无需子模块或另行下载。SDK 库不支持本次未验证的 ARM/Windows 运行环境。

在 3dnav 根目录构建：

```bash
./scripts/build_workspace.sh --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

只更新璇玑接口和控制代码时：

```bash
./scripts/build_workspace.sh --packages-select astrall_sdk_vendor astrall_interfaces astrall_driver car_vel --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 pkg prefix astrall_driver
ros2 pkg prefix astrall_interfaces
ros2 pkg prefix astrall_sdk_vendor
```

三个前缀均应位于当前 3dnav/install 下。源码路径含中文时，Humble 接口生成器不能使用中文构建路径，脚本自动将可重建的 build 产物放到 `/tmp/3dnav-build-<路径摘要>`；源码和 install 仍在本仓库内。构建脚本会先清除外部工作空间路径，再仅加载 `/opt/ros/humble`，防止生成的 setup 文件继续串联旧 SDK 环境。不要复制其他机器的 build/install，也不要在后台启动前加载外部 SDK 工作空间。若命令行之前加载过旧环境，使用新终端。后台更新后重启 `3dnav-backend.service`；重启前保存地图、停止任务并确认机器狗安全停稳。

调用链：网页本体选择 → `/robot_body/select` → 唯一本体 worker → `XuanjiBridge` → 守护进程 → `3dnav/install/astrall_driver` 中的节点 → `3dnav/install/astrall_sdk_vendor` 中的厂商库。桥接层拒绝加载其他安装目录的璇玑驱动。

- 独立遥控：`/dog_remote/cmd_vel`，W/S 前后、A/D 横移（上限 0.2）、Q/E 原地转向。
- 移动建图：`/dog_remote/ensure_mapping_motion` 确认准备完成后，`/cmd_vel_teleop` → 速度仲裁器 → `/cmd_vel_safe` → 本体；支持 W/S 前后与 Q/E 转向，定位暂停时停车，需人工重新开启。
- 导航：现有规划器 → 速度仲裁器 → `/cmd_vel_safe`；璇玑使用单独生成的 DiffDrive 参数，禁止横移、倒车，机身轮廓与雷达安装参数须现场标定确认。
- 雷达与相机使用现有主机设备，璇玑 SDK 自带雷达和相机输出关闭；不要并行启动旧 SDK Demo/WS 程序。

遥控开始：手柄三档开关回 1 档 → 9 取得控制权 → Y 等待固定站立 0xB102 → 4 等待 MOVE 0xB104。R 在停稳后重启 SDK 会话、解除锁定，不自动站立或恢复移动。电量来自 SDK 实际反馈。详见[本体与操作指南](src/ros2_humble_main/src/car_vel/ROBOT_BODY_SELECTION.md)和[SDK 集成目录](src/ros2_humble_main/src/driver/astrall/README.md)。

安装路径检查：

```bash
ps -eo pid,args | grep '[a]strall_driver_node'
ldd install/astrall_driver/lib/libastrall_sdk_client.so | grep ASTRALL
rg 'sw01_ws' install/setup.*
```

进程和动态库必须指向当前 3dnav/install；最后一条命令应没有匹配。只有完成实际路径核验，才能确认当前后台已使用新构建。控制权、站立/MOVE、横移和建图运动仍需现场验收，软件检查不能代替真机效果确认。

## 导航中的人工接管与转向

巡检任务页中央地图上方、“定位准备”下方有“人工遥控”栏；地形导航页也提供同一入口。旧页面请强制刷新。点击“接管（暂停自动执行）”，等待暂停确认后使用 W/S 前后、Q/E 转向，按住移动、松开停止。调整结束后先“退出接管”，再“继续原任务”；地图、目标和巡检进度保留。切页或失焦会退出接管，任务保持暂停。

设置页的 0.25/0.45 m/s 档位只决定前进速度上限，不提高转向速度。星矶在 2D/3D 下均使用最多 400 条 MPPI 候选轨迹（仅空闲时应用，原配置更小时保留），避免切回 2D 后恢复较大的计算量。星矶有效原地转向从 SDK 归一化指令 0.30 起步；反向时先输出零，再按新方向起步。指令过期仍停车，不能用持续运动掩盖控制器断流。钢镚和璇玑不使用此星矶配置。真机转向效果和控制器延迟仍需现场验证。

## 二维导航初始位置与匹配阈值

1. 地形导航页与巡检任务页均支持以下功能。巡检页入口在地图/路线选择下方的常驻“定位准备”栏，可直接显示点云、确认/取消候选、读取/应用匹配阈值。
2. 启动二维导航、选择并加载地图，等待雷达和 LIO 正常。
3. 点击地图上方的“显示二维点云”，再选“设置初始点”。在地图上按住左键拖拽：按下的位置为机器人位置，拖拽方向为朝向。
4. 红色激光点按雷达到机身的实际 TF 投影；拖拽时预览与墙体的对齐。松开仅保存候选，点击“确认二维初始位置”才提交原有重定位流程；“取消候选”恢复当前定位显示。关闭点云仍需确认候选后才提交。
5. 地图交互区或巡检定位准备栏可读取、应用二维初始匹配阈值（1%–100%，默认 50%）。两页复用同一个阈值接口，设置写入后端并读回确认；巡检任务运行/暂停或定位处理中禁止修改初始位置和阈值，导航重启后恢复启动配置。阈值仅影响二维激光与地图匹配比例，不改变三维精修参数、AMCL 协方差、稳定样本数、激光数量或时间同步要求。

ROS 接口使用标准参数服务 `/relocalization_coordinator/get_parameters` 与 `/relocalization_coordinator/set_parameters`，参数名 `initial_scan_match_ratio_2d`，double 值 0.01–1.0。例如在二维模式、初始定位未执行时：

```bash
ros2 param get /relocalization_coordinator initial_scan_match_ratio_2d
ros2 param set /relocalization_coordinator initial_scan_match_ratio_2d 0.50
```

显示话题 `/localization/scan_preview`（String JSON）为按需生成的二维激光预览，坐标系 `base_footprint`，携带点坐标与扫描时间。只在前端订阅时发布，最多 4 Hz、720 点；过期点自动隐藏。更新后重新编译 car_vel、重启导航，并强制刷新网页以加载新脚本。

## 历史：test 分支整合说明（2026-11-04）

本分支以 `dev` 的 `580f543` 为现场基线，整合 `master` 的 `68bf09c` 三维导航实现。`master` 已是 `dev` 的祖先，普通 Git 合并不会恢复被现场版本替换的实现，因此本次按功能整合源文件；不改写两个原分支的历史。

- 三维定位、LIO 连续性、ICP 刚体变换、路径预览、启动复核与导航速度挡位采用 master 的一致接口。
- 二维导航保留现场初始定位质量检查、静止定位更新，以及显式目标在刷新成功后的重新预检与执行。取消、换图、刷新失败都会清除待执行目标。
- 保留现场相机链路、巡检界面右侧 C12 云台、手动双路 MP4 录像及设置中的录像显示开关。录像默认隐藏，机器狗用户为 `a` 时保存到 `/home/a/巡检录像`。
- 录像停止等待已作最小修正：HTTP 立即返回正在保存，原录像线程在状态锁之外等待 FFmpeg 退出；完成后页面恢复按钮。旧响应防覆盖保留，重复判断和未引用的检测设置模块已移除。双路预览中断及电脑重启仍未确定根因，录像并发需按验收清单实测。

本地验证包含导航/巡检与相机 Python 回归、前端 Node 测试、模拟通信的真实浏览器测试，以及 `lzp_interface`、`icp_registration`、`small_point_lio`、`lzp_service` 编译。ICP 与 LIO 的原生行为测试通过。`car_vel` 的 Python 包构建通过；完整工作空间构建在本机缺少 `octomap_msgs` 时停止，未验证整套后台启动。

本次只整合本地 test，未向机器狗部署或重启服务。部署前必须按下文重新构建接口及依赖包，并完成二维/三维模式切换、运动与录像并发实机验收，不能仅覆盖网页。

本工作空间运行于 ROS 2 Humble，支持星玑、钢镚 ZSL-1 点足及璇玑轮式四足，提供 Livox 点云采集、Small Point-LIO、OctoMap 建图、三维定位、Nav2 导航、地图和路线管理、巡检任务、视觉/行为识别、USB 热成像、M701 环境检测及 Web 前端。

本文以 M9 Pro 小电脑的目标部署为例，网卡名称和 IP 应按现场实际配置：

| 项目 | 当前配置 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS |
| 架构 | x86_64 / amd64 |
| ROS 2 | Humble |
| 工作空间 | `/home/a/3dnav` |
| 主机雷达网卡 | `192.168.1.50/24` |
| Livox 雷达 | MID360s 现场模板 `192.168.1.117` |
| 星玑本体 | `192.168.96.2:8080`；小电脑示例 `192.168.96.3/24` |
| 钢镚 ZSL-1 点足 | 本体示例 `192.168.168.168`；小电脑示例 `192.168.168.15`；本地接收端口 `43988` |
| 璇玑轮式四足 | 本体 `10.18.10.100`；主机 `10.18.10.201/24`；SDK 端口 `3600/3601` |
| Python | 系统 Python 3.10；识别依赖按下文隔离安装 |

项目核心导航算法不依赖 CUDA，当前识别部署也使用 CPU 运行库，不要求 NVIDIA 显卡。Intel 核显用于界面显示；实际导航与多模型识别并行负载仍需在小电脑上测量。

## 功能与文档入口

| 模块 | 当前功能 |
| --- | --- |
| 建图 / 修图 | 二维栅格、全局三维点云、地图保存；支持配置建图动态障碍过滤，以及按区域/高度编辑对应点云 |
| 地形导航 | 星玑三维模式下在点云俯视图设置初始点、目标点、多点路线；预览与启动分开，显示全局规划路径 |
| 导航模式 | 设置页选择二维或三维地形增强模式；三维模式仍使用现有 Nav2 平面控制接口，视角切换不等于模式切换 |
| 园区 / 巡检 | 路线执行、检测点、任务进度、告警、证据图片和历史记录 |
| 遥控 / 本体 | 独立遥控；设置页选择星玑、钢镚 ZSL-1 点足或璇玑，各自保存连接参数 |
| 普通摄像头 | 设置页选择原链路 USB/本体 HTTP 图传或 C12 双光链路，供遥控、识别和巡检共享 |
| C12 云台与录像 | 巡检页右侧云台、手动双路 MP4、浏览器录像显示开关；C12 红外测温尚未接入 |
| 视觉识别 | 火焰、烟雾、香烟/烟头目标、人脸及车牌检测/匹配，具体结果依赖模型与登记库 |
| 行为识别 | 疑似跌倒、躺卧姿态、场景级疑似打架及疑似吸烟；吸烟默认关闭 |
| 环境检测 | 海康 USB 热成像、高温阈值与告警；M701 气体/环境数据及 CO₂ 阈值语音告警 |
| 设置 | 本体、导航模式、障碍膨胀半径、雷达网络参数、普通摄像头、气体连接、告警阈值、建图过滤开关、系统电源操作 |

星玑自动导航与巡检沿用固定越障模式；进入越障模式不代表已实现任意楼梯、坑洞或复杂地形的可靠自主通过。三维定位、地形判断、规划和本体能力需要分别验证。详见[星玑三维定位与防跳变](src/ros2_humble_main/src/car_vel/XINGJI_3D_LOCALIZATION.md)及[固定越障策略](src/ros2_humble_main/src/car_vel/XINGJI_FIXED_OBSTACLE.md)。

完整专题文档见[第 9 节](#9-专题文档)。

## 1. 工作空间结构

```text
/home/a/3dnav/
├── src/
│   ├── ros2_humble_main/ # driver/astrall 下含璇玑 SDK 与 ROS 包
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
| 行为识别 | `~/.local/share/robot-behavior/venv` | NumPy 1.26.4、OpenCV 4.6、Paddle 2.6.2、OpenVINO 2024.6（FP32 姿态、INT8 打架及 FP32 吸烟） |

行为依赖不能装进系统 Python 或普通相机目录，也不需要在 ROS 终端手动激活这个环境。网关调用完整新版项目的单个 worker，共享姿态推理并输出跌倒、吸烟和全场景打架结果；当前工控机使用原始 CPU 推理。

模型和厂商源码归档需随 `car_vel/person-behavior-recognition/` 完整同步；安装脚本校验资产并使用包内固定版本依赖源码。加载成功不等于实际帧率或识别准确率达标。使用方法见[行为识别说明](src/ros2_humble_main/src/car_vel/BEHAVIOR_RECOGNITION.md)，原项目启动方式见[行为运行时说明](src/ros2_humble_main/src/car_vel/person-behavior-recognition/README.md)。

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

./scripts/build_workspace.sh \
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

先结束任务、正常停止旧后台，完整同步相关源码。更新完整行为项目与相机网关，且之前已完成完整部署时：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh --packages-select car_vel
source /home/a/3dnav/install/setup.bash
```

相机网关与 `person-behavior-recognition` 子进程必须一起同步，不能只替换网页或单个脚本。新版项目的全部模型资产必须同步；已有独立环境可复用。

若尚未部署星玑三维定位与防跳变更新，因为涉及 Action 接口，至少四个包一起编译：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh --executor sequential \
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

没有走包装脚本的系统 `python3` 报缺包，不一定意味着相机实际运行环境缺包。行为模型加载自检和性能回放方法见[行为运行时说明](src/ros2_humble_main/src/car_vel/person-behavior-recognition/README.md)。

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
./scripts/build_workspace.sh --parallel-workers 4 \
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
./scripts/build_workspace.sh --packages-select lzp_interface
```

成功后重新执行全量 `./scripts/build_workspace.sh --parallel-workers 4`。

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

./scripts/build_workspace.sh \
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

./scripts/build_workspace.sh --parallel-workers 4
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
| [行为运行时](src/ros2_humble_main/src/car_vel/person-behavior-recognition/README.md) | 原项目部署、模型和行为规则说明 |
| [启动恢复](src/ros2_humble_main/src/car_vel/STARTUP_RECOVERY.md) | 后台启动、服务恢复与部署排查 |

文档中的命令以小电脑 Ubuntu 环境为目标；Windows 开发机的离线回归不能代替 Linux 完整编译、真实模型性能测试、雷达/相机验证及机器人现场验收。
