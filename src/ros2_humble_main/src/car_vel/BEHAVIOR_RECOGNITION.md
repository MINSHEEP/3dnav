# 跌倒、打架与吸烟行为识别（M9 Pro CPU）

当前版本接入附件 `v4-smoking-near-face-conservative-20261030` 的吸烟模型与人物关联规则，替代旧的香烟/烟头物体告警。火焰、烟雾、人脸、车牌、高温和气体功能保留。`cigarette_multiclass_best.pt` 是共用多类别权重，仍用于火焰和烟雾；其香烟框已从画面、识别输出、语音和新巡检告警中移除。历史报告不删除。

## 使用与结果

- 设置页统一管理检测开关，USB 相机和巡检任务页面的“疑似跌倒”“疑似打架”“疑似吸烟”与它同步，保存在 `~/.ros/robot_navigation/detection_settings.json`。首次迁移保留旧 `behavior.json` 的选择，详见 [检测开关说明](DETECTION_SETTINGS.md)。
- “启动识别”启用已勾选项目；巡检途中和检测阶段同样启用。跌倒与吸烟共用一个 CPU 人物姿态进程，打架保留独立进程及真实 8 帧采样。默认每进程两线程。开关改变仅重建受影响进程以加载/释放模型。
- “查看行为画面”优先显示实际推理帧的人物姿态与吸烟标注；人物分支没有新鲜画面时可显示独立打架结果。右侧热成像独立显示。
- 有效吸烟结果播报“检测到疑似人员吸烟”，同类别至少间隔 20 秒。巡检记录人物、检测分数、关联证据、来源时间戳和对应原图/标注图。过期、缓存及短暂遮挡结果不会产生新告警。
- 暂停、来源/任务/分辨率变化和重新启用清理历史。行为识别不控制机器人运动或急停。

| 显示 | 含义 |
| --- | --- |
| 疑似跌倒 | 连续姿态规则观察到跌倒过程，无概率分数 |
| 躺卧姿态（未确认跌倒） | 看到躺卧但缺少跌倒过程，不按跌倒播报 |
| 疑似打架 | ppTSM 场景级判断，不认定具体参与人员 |
| 疑似吸烟 | PPYOLOE 候选与人物嘴部/手腕关联；分数为检测分数，非吸烟行为概率 |
| 等待连续观测/不可用 | 无法判断，不代表没有异常 |

吸烟保留附件的嘴部空间关联、持烟手靠近嘴部和近脸连续检测规则：基础阈值 0.5，近脸强证据 0.65、后续关联 0.30。后者必须有真实强证据及同一人物/手腕的连续关联，单个低分框不能触发。可选侧脸头部补检默认不启用。

吸烟最多按 2 Hz 公平轮询每轮 8 个人物区域，实际频率取决于 CPU。姿态跟踪仍要求不超过 0.3 秒的有效观测间隔；遮挡、小目标和低帧率可能使动作无法确认。跌倒/打架模型、阈值和时间要求没有放宽，也不重复补帧。准确率和 M9 Pro 速度需用已有授权视频实测。

## 更新小电脑

正常停止后台，完整同步源码到 `/home/a/3dnav`，包括整个 `person_behavior/` 和新增约 32 MB 的 `output_inference/ppyoloe_crn_s_80e_smoking_visdrone/`。不要只复制页面或 Python 文件。

此前跌倒、打架环境已安装时，依赖版本不变，无需重装整个环境。校验模型：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel/person_behavior
/usr/bin/python3 -s -m pphuman_runtime.assets
```

用已有独立环境检查三个模型加载，不打开相机：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel/person_behavior
env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
  PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' \
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
  "$HOME/.local/share/robot-behavior/venv/bin/python" -s \
  -m pphuman_runtime.self_check --timeout 90
```

应列出 `yolo_pose`、`fight`、`smoking`。自定义 `BEHAVIOR_HOME` 时换成实际解释器。自检只验证加载，不验证速度或准确率。

重建并按原方法启动后台，浏览器强制刷新：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --packages-select car_vel lzp_service --symlink-install
source install/setup.bash
```

若独立环境尚未安装，先执行以下命令，再构建：

```bash
sudo apt-get install python3-venv libgl1 libglib2.0-0 espeak-ng ffmpeg
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/install_behavior_deps.sh
```

不要把行为 requirements 装到系统 Python、ROS 或普通相机依赖目录。行为仍使用独立的 NumPy 1.26/Paddle 2.6 CPU 环境。

## 检查与回放

先在 USB 相机页面启动识别、验证三个开关、行为画面和语音，无需启动导航。

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/behavior/status | python3 -m json.tool --no-ensure-ascii
```

查看 `enabled.smoking`、`branches.smoking`、`workers.pose`、`workers.fight`、`fresh`、耗时与 `speech`。没有人物时的就绪状态不能验证吸烟效果。

停止线上行为识别后，用已有本地视频回放（修改为实际视频路径）：

```bash
cd /home/a/3dnav/src/ros2_humble_main/src/car_vel
bash scripts/benchmark_behavior.sh --branches smoking \
  --video /home/a/behavior-test.mp4 --output /home/a/smoking-benchmark.json
```

`--branches all` 检查三分支；`--branches both` 保留仅跌倒/打架的旧含义。统计时间、丢帧和有效/不可判断覆盖，不输出未经标注验证的准确率。先用已有授权视频，不要求人员真实摔倒或打架来验证。

模型来源和许可见 `person_behavior/THIRD_PARTY_NOTICES.md`、`assets/runtime_assets.json` 和 `environment/smoking_source_provenance.json`。Windows 的规则/协议/页面回归与完整性校验不能代替小电脑的原生模型加载、视频推理和运行负载验证。
