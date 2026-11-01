# 海康相机控制与 YOLO 迁移包

> 当前副本已由 ROS2 `car_vel` 包管理。正式部署请优先阅读包根目录的
> `HIKVISION_CAMERA.md`；集成方式使用系统 Python，不再要求 Conda。

本目录是独立运行副本，不依赖原主机的绝对路径。包内 HCNetSDK 仅适用于 Linux x86_64。

## 安装

```bash
cd src
conda env create -f environment.yml
conda activate hikvision-camera
```

`environment.yml` 同时包含网页控制、Tk 预览、键盘控制、YOLO 和本地语音播放依赖。Torch/Torchvision 版本与现有 `yolo_v8` 环境一致，默认命令仍使用 CPU。

Ubuntu/Debian 最小系统还需要 SDK/PlayM4 的系统动态库：

```bash
sudo apt update
sudo apt install libuuid1 libgl1 libx11-6 libbz2-1.0 zlib1g libfreetype6 libstdc++6 libgcc-s1
```

PyPI 的 Linux PyTorch 2.10 轮子会安装 CUDA 12.8 运行库，无 NVIDIA GPU 时仍可使用 CPU，但下载体积较大。需要 GPU 时确保驱动兼容 CUDA 12.8，然后使用 `--yolo-device 0`；其他 CUDA 版本请按目标机驱动替换 `torch` 和 `torchvision`。

## 运行

建议通过环境变量传入密码，避免密码进入 Shell 历史：

```bash
read -rsp 'Hikvision password: ' HIKVISION_PASSWORD
echo
export HIKVISION_PASSWORD
python camera_web.py --host 192.168.1.19
```

集成后的 ROS2 工程请按上方 `HIKVISION_CAMERA.md` 使用系统 Python 启动，浏览器/API
默认端口为 `8766`。原始 Conda 启动方式仅供压缩包程序单独调试时参考。

其他入口：

```bash
python preview_window.py --host 192.168.1.19
python ptz_keyboard.py --host 192.168.1.19
python yolo_hk.py --host 192.168.1.19 --no-window
python yolo_hk_voice.py --host 192.168.1.19 --no-window
```

`preview_window.py` 和 `ptz_keyboard.py` 需要 Linux X11 图形会话。本地语音需要目标机有可用的音频输出；无声卡或音箱时可使用 `--yolo-voice-output none`。

## 目录

```text
camera_web.py             网页控制服务
camera_control.html       单页控制界面
preview_window.py         Tk + PlayM4 独立预览
ptz_keyboard.py           无 GUI 键盘云台控制
yolo_hk.py                独立 YOLO 取流与推理
yolo_hk_voice.py          YOLO 推理与语音告警
models/                   两个入口的默认模型
sdk/                      HCNetSDK/PlayM4 运行库
*.mp3                     固定告警语音
environment.yml           统一 Conda 环境
```
