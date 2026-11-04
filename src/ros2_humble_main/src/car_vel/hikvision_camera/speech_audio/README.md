# 固定离线播报

这七条普通话提示使用统一女声音色（Microsoft Huihui），于 2026-11-04 提前合成。
格式为 22,050 Hz、16 位、单声道 PCM WAV，总大小约 1 MB。Linux 使用已有的
`ffplay` 播放，不需要安装 Windows 语音引擎，不需要联网合成。

| 文件 | 播报内容 |
| --- | --- |
| `behavior_patrol_safety.wav` | 正在执行巡检任务，请注意安全 |
| `behavior_fall.wav` | 检测到疑似人员跌倒 |
| `behavior_fight.wav` | 检测到疑似打架 |
| `behavior_smoking.wav` | 检测到疑似人员吸烟 |
| `behavior_smoking_public_notice_v1.wav` | 吸烟有害健康，请不要在公共场合吸烟，谢谢配合 |
| `behavior_thermal_high.wav` | 环境温度异常 |
| `behavior_co2_high.wav` | 二氧化碳浓度异常 |

`manifest.json` 记录对应文案、文件大小、时长和 SHA-256。素材与文案一起更新。

## 安装和播放

同步整个 `car_vel` 包，保留本目录。`setup.py` 会将它递归安装到
`share/car_vel/hikvision_camera/speech_audio`，无需再复制到用户缓存目录。

在小电脑上停止旧后台后执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --packages-select car_vel --symlink-install
source install/setup.bash
```

如果系统没有 `ffplay`，安装 `ffmpeg`：

```bash
sudo apt-get install ffmpeg
```

无需启动导航即可试听安装后的巡检提示：

```bash
ffplay -hide_banner -loglevel error -nodisp -autoexit "$(ros2 pkg prefix --share car_vel)/hikvision_camera/speech_audio/behavior_patrol_safety.wav"
```

## 播放规则

- 默认优先播放随项目附带的音频，旧缓存不会覆盖这七条默认提示。
- 显式提供自定义素材目录时，保留自定义 WAV/MP3 的兼容支持。
- 缺少可用录音或播放器时仍可使用 `espeak-ng` 本地回退；回退音色可能不同。
- 网页主程序的高温、CO₂ 告警与行为提示共用本地播报队列，无论普通 YOLO
  是否运行，都不再转发到 YOLO 的联网 TTS 链路。
- 原有检测判定、告警重复间隔、巡检启动及检测点触发时机不变；播放在后台执行。
- 火焰、烟雾仍使用原来的录音及播放逻辑，独立 YOLO 脚本的“检测到”前缀缓存
  机制也保持原样。本次新增素材覆盖上表七条提示。
