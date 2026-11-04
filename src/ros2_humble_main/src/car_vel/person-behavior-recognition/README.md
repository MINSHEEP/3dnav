# 人物行为识别

版本：`cpu-yolo26s-fp32-20261102`。这是 YOLO26s-pose 的隔离试用包，适用 Ubuntu 22.04 x86_64 / ROS 2 Humble / i5-13500H 等 CPU。包内默认模型已换为 YOLO26s-pose；源码仓库和机器上正在运行的正式服务仍保留原配置。

## 替换内容

- 骨架换为官方 `yolo26s-pose.pt` 导出的 OpenVINO FP32，640×640、COCO17 关键点、端到端输出。XML+BIN 约 42.5 MB。本轮没有对新骨架量化，便于区分模型更换和量化带来的效果变化。
- 新增 `[1,300,57]` 输出适配，读取人物框、分数、类别和关键点，不再额外执行 NMS。原 YOLOv8 `[1,56,8400]` 路径仍可用。
- 沿用当前吸烟 OpenVINO FP32、打架 8 帧/224 混合 INT8 异步计算，以及线程设置、旋转补检、连续观测和有效期规则。近脸吸烟 0.65/0.30 等阈值未改变。
- 导出图将 batch=1 的 GatherElements 改写为等价行索引，兼容已有 OpenVINO 2024.6；PyTorch 原输出与改写后输出逐元素一致。单张参考图的高置信度 OpenVINO 输出最大绝对差约 0.000122（含像素坐标）。这只是转换一致性检查，不是准确率测试。
- ROS 输入输出协议保持不变；模型身份会报告 `yolo26s_pose_openvino_cpu_candidate`，并附实际 XML/BIN 指纹。

官方权重、导出工具版本和图变换记录见 `output_inference/yolo26s_pose_openvino_fp32/export.json`。导出工具只下载、解压在开发机器的隔离目录；没有升级目标机环境，目标机运行本包不需要安装 PyTorch 或 Ultralytics。

## 验证与限制

对照基线是上一轮 `cpu-openvino-speed-20261102`，即 YOLOv8m-pose 混合 INT8 + 相同的吸烟/打架分支。在目标 i5-13500H 上保持正式任务运行，仅回测原来的 6 段素材，没有增加视频或跑 50 段。

固定约 8 Hz 的相同源帧对比：跌倒正提示从 54 帧减为 48 帧，首次提示从约 1.89 秒延后到 2.29 秒，中途还少了约 0.4 秒提示；多人打架仍为 27 帧。侧脸片段旧版约 51.07–51.47 秒的 4 帧吸烟提示，新版未检出。跌倒片段两版仍有 4 帧打架误报。

本轮实时回放合计：当前 YOLOv8m 提速版 127 次/55.702 秒（2.28 次/秒），YOLO26s 浮点版 130 次/56.551 秒（2.30 次/秒），约 1.008 倍，基本持平。本轮两版均未形成跌倒、吸烟或打架的实时正提示，连续观测仍受限。这个结果不能与上一轮其他负载时测得的 5.60 次/秒直接比较；本次始终保留正式任务，但没有控制后台负载波动。

88 项代码检查通过，3 项平台相关检查跳过。实际 worker 通过 12 个图像请求、3 个异步打架窗口提交、reset/shutdown 响应和正常退出检查；不是用强制退出替代验收。

因此不能将这次模型更换称为全面升级，当前证据不支持覆盖正式默认配置。本包保留为可独立试用的 YOLO26s 版本；旧版包和模型仍保留。完整六段与实时速度的机器可读记录在 `assets/yolo26s_validation.json`。固定采样的处理耗时不作为实时速度；实时对比按源时间取最新帧，但不含解码、ROS 和显示开销。

这些是重复使用的开发素材，不是独立准确率集；侧脸输入还是带旧标注的屏幕录像。未进行新环境安装验收或 USB/ROS 整链现场测试。

## 使用本 ZIP

本包包含完整模型和固定的 PaddleDetection 推理代码，不含录像、数据集、导出工具或 Python 环境，不需要另找模型，也不依赖 Git LFS。

1. 解压到新目录，例如 `~/PP-HumanV2_YOLO26s`。已有正式识别任务运行时，不要同时启动另一个相机或同话题的识别节点。
2. 在已经安装 ROS 2 Humble、Conda 的 Ubuntu 上准备系统依赖：

   ```bash
   sudo apt update
   sudo apt install python3-opencv python3-numpy python3-pil python3-yaml \
     python3-gi gir1.2-gtk-3.0 fonts-noto-cjk v4l-utils libgl1 libgomp1 \
     ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs
   ```

3. 进入含本 README 的目录，校验模型并配置环境：

   ```bash
   python3 scripts/verify_runtime_assets.py
   bash scripts/setup_ubuntu_cpu.sh
   ```

   脚本默认创建新的 Conda 环境 `pp_humanv2`。同名环境已存在时停止，不会升级它。已有兼容 Conda 环境可只读复用：`bash scripts/setup_ubuntu_cpu.sh --reuse-env --prefix /path/to/envs/pp_humanv2`。本机路径只写入本项目的 `config/local.env.sh`、`config/ros.local.yaml`。

4. 准备进行现场试用时运行 `bash scripts/start_all.sh`；更换相机加 `--device /dev/video2`；使用现有 ROS 图像源可运行 `bash scripts/start_all.sh --no-camera --image-topic /camera/image_raw`；不显示预览加 `--no-view`。

首次准备依赖需要联网；环境和模型齐备后可以离线推理。节点仍为 `pphuman_recognition`，结果 `/pp_human/results`（JSON String），标注画面 `/pp_human/annotated_image`。判断行为时同时检查 `state`、`label`、`updated` 和证据时间戳。

## 源码目录与回退

源码目录通过 `config/inference_yolo26s_candidate.yaml` 选择此模型。已有兼容环境可用 `python -m pphuman_runtime.worker --config config/inference_yolo26s_candidate.yaml` 启动管道协议 worker；它本身不显示窗口。ROS 入口为 `bash scripts/start_recognition.sh --params-file config/ros_yolo26s_candidate.yaml`。

源码的原 `config/inference.yaml`、上一轮提速候选 `config/inference_cpu_speed_candidate.yaml` 和对应权重均保留。独立 YOLO26s ZIP 只带本版三模型，回退时请使用保留的上一版完整包，不能只改一个不存在的模型路径。

重新构建本包：`python scripts/package_cpu_candidate.py --profile yolo26`。本轮没有提交或推送 Gitee。
