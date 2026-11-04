# 人物行为识别

版本：`cpu-int8-8x224-async-20261102`。

本文件是 **CPU INT8 部署包**的使用说明。适用 Ubuntu 22.04 x86_64、ROS 2 Humble、Intel i5-13500H 等支持 AVX2 的 CPU；不需要 NVIDIA 显卡或 CUDA。不是 Windows 直接运行包，也不适用于 ARM/Jetson。

包内已包含源码、固定版本的 PaddleDetection 推理代码和三份实际使用的模型：

- 人体框/骨架：YOLOv8m-pose OpenVINO INT8 混合精度，约 41.2MB。
- 打架：ppTSM OpenVINO INT8 混合精度，8 帧、224×224，约 24.9MB，后台单任务推理。
- 吸烟：原 PP-YOLOE Paddle FP32，保持近脸 0.65/0.30 和已有侧脸补检规则。

默认 `config/inference.yaml` 已选择上述优化模型。无需下载模型、使用 Git LFS 或复制其他项目文件。骨架输出头、打架分类头保留浮点；跌倒几何规则、行为阈值和连续观测时间条件未放宽。包内没有旧骨架/打架 FP32 权重、测试录像、数据集、校准工具或已有 Python 环境。

相较上一轮 8 帧/320 同步版本，打架输入缩小为 224，缩放短边为 238，并使用独立 2 线程的 OpenVINO 异步请求。忙时继续采样、不排队旧窗口；过期或断流前的结果会丢弃。主识别流程不等待打架计算完成，但仍可能受 CPU 争用及其他分支耗时影响，流畅度和准确率需要实测。

**交付状态：模型已导出、CPU 接口已接入、文件已打包。按项目负责人要求，尚未执行优化版的安装验收、实际加载/测速、摄像头实测或 50 段回测。不能据此保证实时帧率、准确率不变或已解决“画面已过期”。**

## 首次配置

先在收件人的 Ubuntu 上安装 ROS 2 Humble 和 Miniconda/Anaconda/Miniforge。环境和系统依赖首次安装需要联网；安装完成后识别本身不依赖网络服务，模型不会在线下载。

1. 将 ZIP 解压到一个新目录，例如 `~/PP-HumanV2_CPU_INT8`，不要覆盖已有部署。进入包含本说明及 `scripts/` 的目录。路径不要求特定用户名。

2. 安装系统依赖（已有 ROS 2 Humble）：

   ```bash
   sudo apt update
   sudo apt install python3-opencv python3-numpy python3-pil python3-yaml \
     python3-gi gir1.2-gtk-3.0 fonts-noto-cjk v4l-utils libgl1 libgomp1 \
     ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs
   ```

3. 创建独立模型环境，随后启动：

   ```bash
   bash scripts/setup_ubuntu_cpu.sh
   bash scripts/start_all.sh
   ```

   默认创建 Conda 环境 `pp_humanv2`，安装 Python 3.10、Paddle CPU 2.6.2、OpenVINO 2024.6.0 等版本清单中的依赖。ROS 节点使用系统 Python，模型使用该独立环境；无需 `colcon build`，不要在 Conda 里安装 ROS。

   如果同名环境已存在，脚本会停止，**不会安装、升级或覆盖现有环境**。可显式选择新的目录，或只读复用已经配置好的环境：

   ```bash
   # 自行指定 Conda 与新的环境目录
   bash scripts/setup_ubuntu_cpu.sh --conda ~/miniconda3/bin/conda \
     --prefix ~/miniconda3/envs/pp_humanv2_cpu

   # 已有兼容 Conda 环境：只检查依赖并记录路径，不安装包
   bash scripts/setup_ubuntu_cpu.sh --reuse-env --prefix /path/to/envs/pp_humanv2
   ```

初始化会在本机项目内生成 `config/local.env.sh` 与 `config/ros.local.yaml`。这两份文件记录本机路径，换机器时重新初始化即可，不要从别人电脑复制过来。脚本不会修改 shell 启动文件、安装驱动、启动系统服务或自动开始摄像头检测。

## 摄像头与 ROS 2

默认读取 `/dev/video0`、1280×720、30 FPS。设备不同可指定：

```bash
bash scripts/start_all.sh --device /dev/video2
```

摄像头设备可用 `v4l2-ctl --list-devices` 查找。遇到权限不足时，将当前用户加入 `video` 组后重新登录：`sudo usermod -aG video "$USER"`。

无图形界面运行：

```bash
bash scripts/start_all.sh --no-view
```

已有 ROS 图像源时，不重复打开相机：

```bash
bash scripts/start_all.sh --no-camera --image-topic /camera/image_raw
```

结果由 `pphuman_recognition` 节点发布：

- `/pp_human/results`：`std_msgs/msg/String`，JSON，包含人物框、跟踪 ID、跌倒/吸烟以及全场景打架。
- `/pp_human/annotated_image`：标注画面。
- `/pp_human/status`、`/pp_human/preview_status`：运行及预览状态。

另开终端查看结果：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /pp_human/results
```

读取行为时同时检查 `state`、`label`、`updated` 和证据时间戳。`warming`/`unknown` 表示尚无有效结论，不应当作正常状态或警报。打架属于场景分类，不绑定到某个个人。预览显示速度不等于模型推理速度。

结果中的 `scheduling.fight` 可查看是否仍在后台计算、已完成多少窗口、是否因过期或断流丢弃结果，以及最近一次模型耗时。异步结果的 `prediction_stamp_ns` 是原窗口结束时间，正常情况下早于当前结果包的图像时间；`updated: true` 表示这次首次取到该结果，不能把该时间戳改成当前时间。

前台按 Ctrl+C 停止本次启动的进程，也可执行 `bash scripts/stop_all.sh`。关闭预览窗口只关闭显示，识别仍会继续。

## 参数与文件

- `config/inference.yaml`：默认优化版模型路径、CPU 线程数和行为参数。此包已启用骨架与打架 OpenVINO，吸烟仍为 Paddle。
- `config/ros.yaml`：ROS 话题、刷新及过期参数；模型解释器路径由初始化生成的本机配置覆盖。
- `config/camera.yaml`：独立相机入口参数；`start_all.sh` 的相机参数可用命令行指定。
- `output_inference/`：三份模型的完整文件；两个 INT8 模型目录附 `quantization.json`。
- `environment/requirements-cpu.lock.txt`：从目标 CPU 机器现有环境记录的包版本；本次未在新环境执行安装验收。
- `assets/runtime_assets.json`：模型文件的 SHA256 和大小。可用 `python3 scripts/verify_runtime_assets.py` 检查传输完整性，这不执行模型推理。
- `release_manifest.json`：打包时全部文件的指纹与待验收状态。
- `models_manifest.json`：历史模型来源记录，其中旧模型不属于本 CPU 包；实际随包资产以 `assets/runtime_assets.json` 为准。

本包只支持这一套 CPU 模型组合；不包含 legacy/GPU 回退模型。本说明仅针对完整 CPU 部署 ZIP；Gitee 源码需先取得 Git LFS 文件，按仓库 README 初始化。当前源码默认也已切到 CPU 配置；发布和打包没有修改远端现有环境或正式服务。完整优化记录见 `docs/CPU_INT8候选说明.md`，第三方来源见 `THIRD_PARTY_NOTICES.md`。
