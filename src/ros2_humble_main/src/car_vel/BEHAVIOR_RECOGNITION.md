# 跌倒与打架识别（M9 Pro CPU）

本次接入只使用附件中的人体姿态模型、连续姿态跌倒规则和场景级打架模型。原有火焰、烟雾、香烟、人脸、车牌、高温和气体功能保持原有入口。附件中的吸烟模型没有加载。

## 使用方式

- USB 相机页面及巡检任务页面增加“跌倒”“打架”独立开关，设置保存在小电脑 `~/.ros/robot_navigation/behavior.json`。初始为两项开启。
- “启动识别”启动原有视觉、高温及已勾选的行为分支。模型加载与连续采样需要时间；页面分别显示启动、积累样本、有效、过期或错误，不把启动进程当作识别已成功。
- 普通摄像头区域可切换到“行为画面”。这里显示实际参与该次推理的帧及标注；热成像仍在右侧。它不一定与原始视频同一时刻，页面显示帧龄；过期帧会清空。
- 巡检进入途中/检测模式时启用行为识别；暂停、切换相机/任务、分辨率变化或重新启动后重新积累证据。行为模块异常只影响自身识别状态，不触发机器人运动或急停。
- 疑似跌倒和疑似打架可通过小电脑本地音响播报，同一类别至少间隔 20 秒。使用独立语音队列，不依赖原 YOLO 进程是否可用。
- 巡检告警保存该次行为推理的原图、标注图及来源时间戳，不借用另一帧 YOLO 图片。报告中保留分支不可判断的状态。

## 结果的含义

| 显示 | 含义 |
| --- | --- |
| 疑似跌倒 | 连续姿态规则观察到跌倒过程；没有概率分数 |
| 躺卧姿态（未确认跌倒） | 看到躺卧，但没有足够动作过程证明跌倒；不按跌倒播报 |
| 疑似打架 | 整段场景的打架模型达到阈值；不认定具体某个人为参与者 |
| 样本不足/过期/不可用 | 不能判断，不能解读为“没有异常” |

当前阈值沿用附件：跌倒要求相邻有效观测间隔不超过 0.3 秒；打架使用 8 帧、约 1.63 秒窗口，采样断裂会重新积累。没有为了适配 CPU 而伪造帧时间、重复补帧或放宽动作规则。结果为辅助巡检线索，需要人工复核。

## 在小电脑部署

先把更新后的项目同步到 `/home/a/3dnav`，包括 `car_vel/person_behavior/` 下约 247 MB 模型及厂商源码归档。不要只复制 Python 文件。先停止现有后台，再执行下面各组命令。

首次准备 Python 的独立运行环境和本地语音：

```bash
sudo apt-get install python3-venv libgl1 libglib2.0-0 espeak-ng ffmpeg
```

安装行为模块专用依赖：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_behavior_deps.sh
```

此脚本仅适用于 Ubuntu 22.04 x86_64、Python 3.10。安装位置是 `~/.local/share/robot-behavior/venv`，与现有 `~/.local/share/hikvision-camera/python` 分离。行为模块需要 NumPy 1.26/Paddle 2.6；不要把这份 requirements 安装到系统 Python、ROS 或原相机依赖目录。脚本校验模型 SHA256，并仅安装 CPU 版运行库，不安装 CUDA。最后会在 90 秒期限内启动模型加载自检，失败时会直接显示底层错误；该自检不访问相机，也不测试机器人运动。

若 PyPI 下载慢，可在同一次安装命令前设置已有可信镜像，例如：

```bash
PIP_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple bash scripts/install_behavior_deps.sh
```

重新构建更新的两个 ROS 包：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --packages-select car_vel lzp_service --symlink-install
source install/setup.bash
```

随后按原有方式启动后台，在浏览器强制刷新页面。无需改机器人 SDK、导航模式或普通相机配置。

## 验证顺序

1. 打开 USB 相机页面，确认普通摄像头原图正常，再勾选两项行为功能并点击“启动识别”。
2. 等待各分支加载完成，查看具体状态与错误；切到行为画面，确认帧龄会更新。没有人时不能用它验证跌倒动作。
3. 使用已有、授权的录制视频验证，先不启动机器人导航，不让人员为了测试真实摔倒。可将相机对准测试视频画面做界面联调，但准确率应以原视频回放评估。
4. 停止识别、切换本体/USB 视频源，再重新启动。旧画面与旧告警不应串入新来源。
5. 巡检报告中分别查看告警类别和行为专属证据图片；准备好识别后再单独验证巡检流程。

只读检查行为服务：

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/behavior/status | python3 -m json.tool --no-ensure-ascii
```

关注 `running`、`branches.fall`、`branches.fight`、`fresh`、`frame_age_seconds`、`error` 和 `speech`。普通 YOLO 或热成像可用，不代表行为分支已可用。

## 在 M9 Pro 上测量实际性能

本地开发已做协议、状态、证据与页面回归；未在 M9 Pro 上实测真实模型速度和识别准确率。i5-13500H 没有 NVIDIA 显卡，不能套用附件中 GPU 的帧率。

停止行为识别后，使用本地视频进行有界回放，避免与线上识别同时占用 CPU：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/benchmark_behavior.sh --video /home/a/behavior-test.mp4 --output /home/a/behavior-benchmark.json
```

把实际视频放在命令所写位置，或修改为真实绝对路径。脚本保留视频采样时间、按实际处理能力跳过来不及处理的帧，报告推理耗时和有效/不可判断的覆盖情况。CPU 来不及时应先只开一项降低负载，再测量；不可通过改时间戳或忽略断帧把结果强行变为有效。如果仍达不到连续观测要求，需要更轻模型/经验证的加速后端，不能承诺当前 CPU 上稳定实时。

## 模型与代码来源

来源、模型摘要和第三方许可随 `person_behavior/THIRD_PARTY_NOTICES.md`、`person_behavior/assets/runtime_assets.json` 及 `person_behavior/licenses/` 一起交付。运行时只使用必要资产，未执行原附件部署脚本。现有相机、热成像、ROS 环境不导入 Paddle；专用子进程清理继承的 Python/CUDA 库路径，默认两线程，退出只回收自身进程。
