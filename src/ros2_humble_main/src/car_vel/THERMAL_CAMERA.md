# USB 热成像与高温检测部署

本功能使用用户提供的海康 Linux64 HCUSBSDK bundle。厂商动态库不复制进本仓库，
默认由独立进程从 `/opt/hik_thermal_linux64_bundle` 加载。该进程是热成像 USB
设备的唯一持有者，只监听 `127.0.0.1:8767`；浏览器统一通过普通相机网关的
`8766/api/thermal/*` 同源接口访问，因此页面不会直接加载 SDK 或争抢设备。

## 1. 安放并核验厂商 bundle

把压缩包解压后的根目录完整放到：

```text
/opt/hik_thermal_linux64_bundle
├── PythonDemo/thermal_max_cv.py
├── library/linux64/libHCUSBSDK.so
└── ...
```

不要只复制主动态库；SDK 还依赖同目录的 PlayCtrl、格式转换、OpenSSL、libusb
等运行库。若 bundle 内带 `SHA256SUMS`，部署后应在 bundle 根目录执行：

```bash
sha256sum -c SHA256SUMS
```

## 2. USB 权限

先连接设备并核对实际 VID:PID：

```bash
lsusb
```

用户附件的规则针对 `2bdf:0102`。只有实际设备一致时，才把 bundle 内的 udev
规则安装到 `/etc/udev/rules.d/`，随后重新加载规则并重新插拔设备。若型号不同，
必须按实际 VID:PID 修改规则，不能照搬。运行 ROS 的用户需属于规则指定的
`plugdev` 组；组变更后注销并重新登录。

## 3. Python 依赖、构建与启动

热成像服务复用普通 USB 相机的 NumPy/OpenCV 运行环境：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_usb_camera_deps.sh
cd /home/a/3dnav
colcon build --symlink-install --packages-select car_vel lzp_interface lzp_service
source install/setup.bash
ros2 launch car_vel launch_manager.launch.py
```

默认启动参数：

- `thermal_enabled:=true`
- `thermal_bundle_dir:=/opt/hik_thermal_linux64_bundle`
- `thermal_device_index:=0`
- `thermal_serial:=`（多设备时建议指定准确序列号）
- `thermal_web_port:=8767`，仅监听本机回环地址
- `thermal_api_url:=http://127.0.0.1:8767`
- `thermal_default_threshold_c:=60.0`

自定义示例：

```bash
ros2 launch car_vel launch_manager.launch.py \
  thermal_bundle_dir:=/data/hik_thermal_linux64_bundle \
  thermal_serial:=设备序列号 \
  thermal_default_threshold_c:=55.0
```

如设备启用了登录密码，通过启动环境提供 `HIK_THERMAL_USERNAME` 和
`HIK_THERMAL_PASSWORD`；不要把密码写进 launch 参数、脚本或进程命令行。

## 4. 页面和检测行为

- 遥控页、USB 相机页、巡检页均为普通画面在左、热成像在右；窄屏自动上下排列。
- USB 相机页“启动识别”同时协调原有 YOLO 和高温检测；“查看识别画面”仍只在
  普通原图与原有 YOLO 标注图之间切换，不改变旧功能。
- USB 相机页修改全局默认阈值；巡检任务运行时，巡检页修改当前任务专用阈值。
  新阈值从后续温度帧开始生效，不追溯修改已形成的告警证据。
- 高温判定使用 SDK 同帧给出的全屏最高温和坐标，默认需持续超阈值 0.8 秒；
  解除告警需低于阈值 3 ℃，同一次连续超温只生成一条事件。
- 巡检报告保存峰值温度、当时阈值、热成像原图和标注图；温度从不作为 YOLO
  置信度处理。

当前红框表示 SDK 报告的最高温点附近，不等同于高温目标轮廓分割，也不代表
普通图与热图已经做像素级配准。测温准确性仍取决于设备标定、发射率、距离、
反射温度、外部光学透过率和现场环境。

## 5. 状态检查和故障隔离

```bash
curl http://127.0.0.1:8767/healthz
curl http://127.0.0.1:8766/api/thermal/status
curl http://127.0.0.1:8766/api/thermal/config
```

未发现设备时，优先检查 USB 连接、udev 权限、bundle 完整性、CPU 架构以及设备
是否被其他 SDK 程序占用。热成像服务会保留 HTTP 状态并自动重连；其故障不会
停止普通摄像头、遥控或原有 YOLO。若巡检业务将热成像定义为强制覆盖，应在现场
验收时明确“不可用时禁止开始/暂停任务”的策略；当前实现按可选能力继续任务并在
状态中标记热成像覆盖不完整。

## 6. 缺少整合包且 USB 尚未枚举时的现场恢复

2026-10-15 现场状态为缺少
`/opt/hik_thermal_linux64_bundle/PythonDemo/thermal_max_cv.py`，同时
`lsusb -d 2bdf:0102` 无输出。这是两个独立条件：先恢复文件不等于相机已经连接，
普通 USB 识别不到设备时，不能靠安装 SDK 或更改网页解决。

使用用户原先的 `hik_thermal_linux64_bundle.zip`，不要用仅含 C++ 示例和动态库的
`HCUSBSDKV2.5.0.3_build20240423_Linux64_ZH_20240506165039.zip` 替代。
已只读核验整合包 CRC 和适配脚本、主库、头文件、权限规则的 SHA256。

### 6.1 恢复完整文件

先将整合 ZIP 传到小电脑 `/home/a/hik_thermal_linux64_bundle.zip`。
让机器狗保持静止，结束任务，在原启动终端用 Ctrl+C 正常停止后台；不要在采集进程运行时
写入动态库，也不要另开第二个 SDK 预览/采集进程。停止后台不会撤销已经持久保存的
BRLTTY 屏蔽和 M701 连接配置；再次启动后气体模块会按既有规则重新计时预热。

先检查 ZIP，成功后再执行下一段：

```bash
ls -lh /home/a/hik_thermal_linux64_bundle.zip
unzip -t /home/a/hik_thermal_linux64_bundle.zip
```

解压到 `/opt`。ZIP 自带顶层目录，`-n` 不覆盖任何现有文件：

```bash
sudo unzip -n /home/a/hik_thermal_linux64_bundle.zip -d /opt
```

从原 ZIP 读取校验清单，在目标目录核验；不使用可能遗留的旧清单：

```bash
cd /opt/hik_thermal_linux64_bundle
unzip -p /home/a/hik_thermal_linux64_bundle.zip hik_thermal_linux64_bundle/SHA256SUMS | sha256sum --check -
```

每项应成功；如果缺文件、校验不一致或没有 unzip 命令，先反馈完整输出，不继续、
不加覆盖参数、不删除现有目录。当前错误不要求重新编译 ROS 或重新安装 NumPy/OpenCV。

### 6.2 单独确认并授权热成像 USB

只检查热成像相机的供电和 USB 数据线。需要重新插接或换到小电脑空闲 USB 口时，
仅操作热成像设备，不拔插气体转换器、雷达网卡或整个集线器。

```bash
lsusb -d 2bdf:0102
```

应出现已确认的 HIK HikCamera。没有输出时先停止本阶段，提供完整 `lsusb` 和
`sudo dmesg --ctime | tail -n 40`，区分未枚举、型号改变和反复断连；暂不安装驱动或权限规则。

确认相同 VID:PID 后才安装附件规则：

```bash
sudo install -m 0644 /opt/hik_thermal_linux64_bundle/PythonDemo/70-hik-thermal.rules /etc/udev/rules.d/70-hik-thermal.rules
sudo udevadm control --reload-rules
```

只重新插拔热成像相机使规则生效，再执行：

```bash
sudo udevadm settle --timeout=10
lsusb -d 2bdf:0102
id -nG
```

本现场用户此前已有 plugdev 组；不要为此修改已正常的 dialout/气体权限。若新会话没有
plugdev，先反馈而不是用 sudo 运行 ROS 或把 USB 节点改成所有人可写。

### 6.3 原方式启动并检查实帧

保持原启动命令的其他参数；若原来使用默认主启动命令：

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
ros2 launch car_vel launch_manager.launch.py
```

另开终端检查：

```bash
curl -sS --max-time 5 http://127.0.0.1:8767/api/status | python3 -m json.tool --no-ensure-ascii
curl -sS --max-time 5 http://127.0.0.1:8766/api/thermal/status | python3 -m json.tool --no-ensure-ascii
```

需要有 `ready:true`、持续更新的 `frame_sequence` 和合理帧龄、有效温度、非过期状态，
再刷新页面验证热图。仅 HTTP 200、ZIP 校验通过或 SDK 离线自检通过不能代替实际出图。
仍有错误时用返回的完整 `error` 定位依赖、权限、SDK 枚举或采集阶段，不扩大修改到 M701、
机器狗或导航配置。

## 7. 原生 SDK abort 的证据记录（2026-10-15）

本次现场日志为 `libusb_ref_device` 的 `refcnt >= 2` 断言后进程退出 -6。
这与缺附件、NumPy 导入失败、网页黑屏或气体串口占用不同；代理 HTTP 503 是
热成像进程退出之后的结果。该时段内核日志为 No entries，系统无 coredumpctl，
目前不能判断 SDK 内部引用计数异常的根因，也不能宣称底层崩溃已修复。

`thermal_web.py` 现在在启动采集线程之前启用 `faulthandler`，并在已有后台日志中
输出 `[ThermalSDK] pid=... epoch=... phase=...`，区分加载适配模块、SDK 初始化、
枚举、登录、配置、启动取流、等待帧和关闭阶段。发生原生 abort 时，Python 运行时
可输出当时各 Python 线程的调用栈；这不是原生 C/C++ 堆栈，不保证能单独定位 SDK 缺陷。
不抑制 SIGABRT，不循环拉起崩溃进程，不替换 SDK/libusb 或升级全局 Python 库。

随本次主后台正常更新一起加载即可；不要启动第二个示例同时占用相机。
若自然运行中再次丢图，先保留完整启动日志，再单独执行以下只读检查：

```bash
curl -sS --max-time 5 http://127.0.0.1:8767/api/status | python3 -m json.tool --no-ensure-ascii
```

```bash
lsusb -d 2bdf:0102
```

提供日志中的 `[ThermalSDK]` 阶段、断言及其后的调用栈；无需为了复现故障再次导航，
也无需仅为这一步安装崩溃工具。后续原生根因修复需这些现场证据，当前仅完成可诊断性改进。
