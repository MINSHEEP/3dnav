# 跌倒、打架与吸烟行为识别（M9 Pro CPU）

行为识别使用附件默认版本 `cpu-yolo26s-fight-capture-20261102`。跌倒与吸烟共用 YOLO26s-pose OpenVINO FP32 姿态模型，吸烟使用 PP-YOLOE OpenVINO FP32，打架使用异步 ppTSM 混合 INT8（8 帧 × 224）。三路行为共用一个进程，普通 YOLO 可同时运行。火焰、烟雾、人脸、车牌、高温和气体走各自原有流程。

## 使用与结果

- 设置页统一管理检测开关，USB 相机和巡检任务页面的“疑似跌倒”“疑似打架”“疑似吸烟”与它同步，保存在 `~/.ros/robot_navigation/detection_settings.json`。首次迁移保留旧 `behavior.json` 的选择，详见 [检测开关说明](DETECTION_SETTINGS.md)。
- “启动识别”启用已勾选项目；巡检途中和检测阶段同样启用。跌倒与吸烟共享一次人物姿态推理，打架从相机采集端独立保留 8 帧，再交给同一进程异步处理。姿态和吸烟各配置 4 个 OpenVINO 线程，打架配置 2 个。开关改变时重建这个行为进程以加载或释放相应模型。
- “查看行为画面”优先显示实际推理帧的人物姿态与吸烟标注；人物分支没有新鲜画面时可显示独立打架结果。右侧热成像独立显示。
- 巡检途中和检测点发现有效疑似吸烟结果时，播报“吸烟有害健康，请不要在公共场合吸烟，谢谢配合”；手动识别仍播报“检测到疑似人员吸烟”。同类别至少间隔 20 秒。巡检记录人物、检测分数、关联证据、来源时间戳和对应原图/标注图。过期、缓存及短暂遮挡结果不会产生新告警。
- 巡检吸烟语音使用独立缓存 `~/.cache/robot_inspection_speech/behavior_smoking_public_notice_v1.mp3`，不会沿用旧 `behavior_smoking.mp3`。没有新音频时由已安装的 `espeak-ng` 离线播报；长句播放超时上限为 30 秒。
- 暂停、来源/任务/分辨率变化和重新启用清理历史。行为识别不控制机器人运动或急停。

| 显示 | 含义 |
| --- | --- |
| 疑似跌倒 | 连续姿态规则观察到跌倒过程，无概率分数 |
| 躺卧姿态（未确认跌倒） | 看到躺卧但缺少跌倒过程，不按跌倒播报 |
| 疑似打架 | ppTSM 场景级判断，不认定具体参与人员 |
| 疑似吸烟 | PPYOLOE 候选与人物嘴部/手腕关联；分数为检测分数，非吸烟行为概率 |
| 等待连续观测/不可用 | 无法判断，不代表没有异常 |

吸烟保留附件的嘴部空间关联、持烟手靠近嘴部和近脸连续检测规则：基础阈值 0.5，近脸强证据 0.65、后续关联 0.30。后者必须有真实强证据及同一人物/手腕的连续关联，单个低分框不能触发。侧脸头部补检默认启用，每轮最多补检 2 个头部区域，强证据和后续阈值分别为 0.50 与 0.30。脸部不可观测时跳过推理并标为无法判断。

吸烟最多按 2 Hz 公平轮询每轮 8 个人物区域，实际频率取决于 CPU。姿态跟踪仍要求不超过 0.3 秒的有效观测间隔；遮挡、小目标和低帧率可能使动作无法确认。打架继续要求 8 张真实采集帧，相邻采样最大间隔 1.0 秒，正例最低分 0.80，整个打架窗口最多 3 秒，结果从实际窗口末帧起有效 2 秒。吸烟结果从实际判定帧起有效 2 秒。准确率和 M9 Pro 速度需用已有授权视频实测。

## 更新小电脑

停止后台，将 `car_vel` 的更新同步到小电脑，包括网关、`person_behavior/config/inference.yaml` 和 `assets/runtime_assets.json` 列出的全部文件。新版三组权重约 96 MB，XML 和 BIN 需成对复制。

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_behavior_deps.sh
```

安装脚本复用原有独立环境，安装锁定的 OpenVINO 2024.6.0，校验资产、准备 PaddleDetection 预处理代码并检查三个模型能否加载。没有独立环境时先安装 `python3-venv libgl1 libglib2.0-0`。语音可使用 `espeak-ng` 或已有音频与 `ffmpeg`。

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --packages-select car_vel --symlink-install
source install/setup.bash
```

按原方法启动后台，在 USB 相机页面验证三路，再检查巡检语音和报告。模型加载自检只覆盖初始化。

## 检查与回放

先在 USB 相机页面启动识别、验证三个开关、行为画面和语音，无需启动导航。

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/behavior/status | python3 -m json.tool --no-ensure-ascii
```

查看 `enabled.smoking`、`branches.smoking`、`workers.shared`、`fresh`、耗时与 `speech`。没有人物时的就绪状态不能验证吸烟效果。

停止线上行为识别后，用已有本地视频回放（修改为实际视频路径）：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/benchmark_behavior.sh --branches smoking \
  --video /home/a/behavior-test.mp4 --output /home/a/smoking-benchmark.json
```

`--branches all` 检查三分支；`--branches both` 保留仅跌倒/打架的旧含义。统计时间、丢帧和有效/不可判断覆盖，不输出未经标注验证的准确率。先用已有授权视频，不要求人员真实摔倒或打架来验证。

模型来源和许可见 `person_behavior/THIRD_PARTY_NOTICES.md`、`assets/runtime_assets.json` 和 `environment/smoking_source_provenance.json`。Windows 的规则/协议/页面回归与完整性校验不能代替小电脑的原生模型加载、视频推理和运行负载验证。
