# 普通摄像头（USB / 本体图传）部署

普通摄像头右侧的 USB 热成像、高温阈值和巡检告警部署见
[`THERMAL_CAMERA.md`](THERMAL_CAMERA.md)。

## 在设置页切换普通摄像头

设置页新增“普通摄像头来源”，统一决定遥控页左侧、相机页普通画面、
巡检普通画面以及视觉识别的输入。热成像和 M701 独立，不随此选择改变。
导航和机器人本体配置不变。

- **USB 摄像头**：保留原 Sonix 默认路径；可刷新设备列表选择另一台普通相机，
  或填写稳定的 `/dev/v4l/by-id/…-video-index0` 路径。高级选项可设置请求的
  分辨率和帧率，实际格式以设备协商为准。不会自动改选其他 USB 相机，
  海康热成像及元数据接口不列入候选，打开设备前还会校验实际节点身份。
- **狗本体摄像头**：星玑默认使用小电脑的
  `http://127.0.0.1:8081/camera`，沿用现有上游 `http://192.168.96.2:8080`。
  不需要把 `127.0.0.1` 改成浏览器电脑的 IP，这个地址由小电脑后台访问。
  该代理随星玑桥接进程运行；本体未连接或图传不可用会切换失败并给出错误。
  其他本体只能填写实际可用的 HTTP/HTTPS JPEG 或 MJPEG 地址；不支持直接填 RTSP，
  不会凭空为钢镚创建图传接口。

先结束巡检任务（暂停仍算任务中）并停止视觉识别，再点“保存并切换”。
切换先释放旧源，只在新来源实际出帧后保存；失败尝试恢复原来源，
不会把旧缓存图像当成新相机画面。若旧采集线程未退出，则不打开第二个来源；
此时可能没有画面，需按错误提示恢复或重启相机后台，不能保证故障硬件一定恢复。

选择保存在小电脑 `~/.ros/robot_navigation/ordinary_camera.json`，
重启后沿用；没有配置文件时才采用原 USB 启动默认值。
配置文件损坏时不会静默改选来源，设置页可显式修复并保留损坏文件备份。
多个页面同时编辑时按配置版本校验，不允许覆盖另一页面刚保存的选择。

本体画面请求独立于遥控：仅申请图传命令，不开启遥控、不站立、不发送速度。
退出遥控时，正在使用的共享本体图传保持开启；切走本体来源会释放该请求，
取流进程异常退出后请求会过期。USB 设备和本体 HTTP 流均由普通相机网关共享，
YOLO 不会另开硬件。原“启动识别 / 查看识别画面”行为保持不变。

### 本次更新（已安装相机依赖的主机）

先把本次修改后的项目同步到小电脑 `/home/a/3dnav`。
让机器人保持静止、结束任务，正常停止原启动终端，再执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select car_vel
source /home/a/3dnav/install/setup.bash
ros2 launch car_vel launch_manager.launch.py
```

如原启动命令带现场参数，请保留原参数。需要重启整个后台以更新本体图传代理，
只刷新网页不够；无需重新安装已正常工作的热成像 SDK、Python 依赖或气体驱动。
浏览器强制刷新设置页，选择来源并保存，再到遥控/相机页核对图像。
最后反向切回 USB 验证，并在下一次正常启动后确认已保存来源仍被沿用。

只读检查当前普通来源与画面状态：

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/camera/source \
  | python3 -m json.tool --no-ensure-ascii
curl -sS --max-time 5 http://127.0.0.1:8766/api/status \
  | python3 -m json.tool --no-ensure-ascii
```

`config.source` 为 `usb` 或 `body`；`video.error` 应为空，
`video.encoded_frames` 应增长。`locked_reason` 提示需要先停止的识别/巡检操作。
普通来源切换不应让热成像的帧序号停止更新。

## Fast DDS 共享内存启动保护

主启动入口会在每次创建建图或导航进程树前执行官方维护命令
`fastdds shm clean`，清理由异常退出遗留且已经不再使用的共享内存对象。
该命令最多等待 3 秒，不会由项目代码直接删除 `/dev/shm` 文件。

页面驱动的建图/导航进程树默认设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，
从而避免频繁切换页面时复用旧 SHM 端口；这个环境
只作用于新进程树，不会修改主进程或系统全局环境。运行日志会明确显示
“cleanup completed”和“Starting ... with UDPv4 only”。可通过
`fastdds_force_udp:=false`恢复默认传输；此时若维护命令失败，仍由
`fastdds_udp_fallback_enabled`决定是否回退到 UDPv4。

可通过 launch 参数调整：

- `fastdds_shm_cleanup_enabled:=true`
- `fastdds_shm_cleanup_timeout_sec:=3.0`
- `fastdds_udp_fallback_enabled:=true`

建议保持三个默认值。若主机已安装 Fast DDS CLI，正常情况下仍使用默认的
UDPv4+SHM 高性能传输；只有清理不可用时才进入 UDPv4 安全降级。

当前部署主机已经按USB身份确认：海康热成像 `2bdf:0102` 占用
`/dev/video0`、`/dev/video1`，普通Sonix USB相机 `2bc5:050f` 占用
`/dev/video2`、`/dev/video3`。其中普通相机的 `video-index0` 才是视频采集节点，
`video-index1` 是元数据节点。项目默认使用以下稳定路径，以 MJPG
1280×720、30 FPS采集：

```text
/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
```

相机主服务是该V4L2设备的唯一持有者；遥控页、巡检页和YOLO都通过主服务的
MJPEG流读取。普通相机后端不得选择海康热成像的 `/dev/video0` 或
`usb-HIK_HikCamera_...` 路径，否则会导致普通画面、热成像和识别同时失败。

## 首次安装

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_usb_camera_deps.sh
sudo usermod -aG video "$USER"
```

安装脚本同时安装 Ubuntu 22.04 官方的 `fastdds-tools`，供主程序在模式启动前
执行 `fastdds shm clean`。已经完成过旧版安装的主机可以重新运行一次脚本；
即使暂时没有安装该工具，程序也会自动使用 UDPv4 安全降级。

安装脚本固定使用 CPU 版 PyTorch。完整依赖解析同时应用
`requirements-pytorch-cpu.txt`，防止 Ultralytics 从通用 PyPI 镜像重新选择
CUDA 版 `torch`。无 NVIDIA 显卡的主机不需要安装任何 `nvidia-*`、
`cuda-toolkit` 或 CUDA 运行库。

加入 `video` 用户组后需注销并重新登录。然后确认设备：

```bash
v4l2-ctl -d \
/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
--list-formats-ext
```

## 构建和启动

```bash
cd /home/a/3dnav
colcon build --symlink-install --packages-select car_vel lzp_interface lzp_service
source install/setup.bash
ros2 launch car_vel launch_manager.launch.py
```

默认参数：

- `camera_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0`
- `camera_width:=1280`
- `camera_height:=720`
- `camera_fps:=30`
- 主模型检测烟雾、火焰和烟头，`yolo_imgsz:=640`
- 到点精检主模型使用训练分辨率，`yolo_inspection_imgsz:=1280`
- 行进中人脸/车牌限频检测，`yolo_aux_transit_fps:=3.0`
- 手动预览人脸/车牌限频检测，`yolo_aux_manual_fps:=5.0`
- 到点精检时主模型、人脸和车牌均逐帧运行
- 相机 API/MJPEG：`http://<机器人IP>:8766`

更换普通相机后，应先通过 `v4l2-ctl --list-devices` 和 `/dev/v4l/by-id/`
确认新的非海康 `video-index0` 稳定路径，再通过参数覆盖：

```bash
ros2 launch car_vel launch_manager.launch.py \
camera_device:=/dev/v4l/by-id/实际普通相机-video-index0
```

USB 摄像头不需要 IP、用户名或密码，也不提供云台、光学变倍、红外和雨刮控制。
独立 USB 相机页保留实时画面、YOLO、截图、重连和设备状态；巡检检测点固定执行
“稳定 1 秒、连续精检 2 秒、按配置保存原图与标注图”。

人脸检测后使用 OpenCV SFace 特征与人员库比对；车牌检测后使用 HyperLPR3
执行字符 OCR，并用 HSV 主色分类为黄底黑字、白底黑字、蓝底白字、绿底黑字
或未知。识别结果分为“已登记 / 未登记 / 不确定”，三种结果都只记录，均不会
触发语音或异常告警；烟雾、火焰和烟头继续沿用原有告警策略。

人员、车辆和识别事件使用独立数据库
`~/.ros/robot_navigation/recognition.db`，与巡检安全告警库分离。人员和车辆各
最多登记 50 条；同一任务中目标需连续稳定 2 帧才记录，持续出现不会重复记账，
离开超过 120 秒后再次出现才创建新事件。网页管理入口为：

```text
http://<当前上位机前端地址>/recognition_records.html
```

巡检页显示当前任务最近 5 条识别记录，巡检报告显示识别汇总，但这些信息不改变
任务正常/异常结论。结构化事件保留 180 天，已登记/未登记证据保留 90 天，
不确定证据保留 30 天，档案操作与复核审计保留 365 天。

安装脚本会从 OpenCV 官方模型仓库下载并校验 SFace 模型，同时自检
OpenCV SFace、HyperLPR3 和 ONNX Runtime。完成安装后必须重新执行下面的
`colcon build --symlink-install`，让新模型进入安装目录。

若现场 CPU 负载过高，可以降低 `yolo_aux_transit_fps`，设为 `0` 可关闭行进中
辅助检测，到点精检仍会逐帧运行已启用的模型。

## 巡检可靠性说明

- 相机服务默认允许 6 条 MJPEG 长连接，其中 YOLO 默认占用 1 条。多个浏览器标签页同时打开视频时，页面会提示关闭多余视频页面。
- 巡检页会先启动导航系统；Nav2 确认就绪后才允许加载地图，初始定位确认成功后才允许启动任务。
- AMCL 位姿是更新驱动消息，不是固定频率心跳。路线下发前若位姿接收时间超过 3 秒，后台会保留当前点并在最多 8 秒内请求无运动更新；刷新成功后重试同一点，失败则把巡检安全暂停，绝不会把剩余点全部标记不可达。现场可通过 `goal_preflight_wait_timeout_sec`、`goal_preflight_nomotion_interval_sec` 和 `goal_preflight_nomotion_max_requests` launch 参数调整等待策略，但不建议通过无限增大位姿年龄阈值绕过定位健康检查。
- 平面地形重建后，机器人中心若恰好落在雷达近场造成的单格未知孔内，预检默认只允许在 0.35 m 内解析到已经验证的可通行连通域（`goal_preflight_start_tolerance_m`）；致命障碍格、地图外或更大未知区仍会拒绝并安全暂停路线，不能用增大该参数绕过地形门禁。
- 路线或巡检节点异常退出后会自动拉起；任务会接管同一 owner 路线或进入安全暂停，路线节点丢失时同时撤销 Nav2 中的旧目标。
- ROS2 普通网页服务默认 15 秒超时；地图切换、地图保存和导航启动使用 60–120 秒的独立期限。
- 若本地缺少“检测到.mp3”，程序会首次使用 edge-tts 生成并缓存到 `~/.cache/robot_inspection_speech`；生成失败只省略前缀，不影响类别播报和检测。
- 巡检页显示现场图片磁盘剩余容量；默认低于 5 GB 时告警，历史和图片仍由操作员手动删除。
