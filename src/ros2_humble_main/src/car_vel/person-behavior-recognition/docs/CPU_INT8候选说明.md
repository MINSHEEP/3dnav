# i5-13500H 纯 CPU 优化说明

当前源码默认版本：`cpu-int8-8x224-async-20261102`。代码与量化产物已准备，**按用户要求暂停验证**。没有执行 INT8 的性能/效果验收或之前 50 段回测，没有运行新增测试，也没有切换正式服务。不要把“已导出”理解为“已证明实时可用”。

## 本次改动

1. 人体与骨架模型改为可选的 OpenVINO CPU INT8 混合精度版本。主干和特征融合部分量化，28 个检测/关键点输出头卷积保留浮点，输入输出仍为原来的 FP32 张量，640×640 输入、COCO 17 点与后处理坐标约定不变。
2. 原 ONNX 为 106,128,038 字节；新 OpenVINO XML 与 BIN 合计 41,212,699 字节，减少约 61.2%。这不是全网络 INT8，也不是整个三模型项目的大小减少比例。尚未测定实际内存、速度和精度变化。
3. 增加 `pphuman_runtime/openvino_cpu.py`，只启用 CPU、单推理流，候选配置限制为 4 线程。返回数组独立复制，避免连续推理及旋转补检重用输出缓冲区。
4. 打架 ppTSM 改为 OpenVINO INT8 混合精度、固定单窗口 `[1,8,3,224,224]` 输入、两个 FP32 logits 输出。卷积主干量化，分类头保留浮点；8 帧时序结构不变。缩放短边同步从 340 改为 238，裁剪由 320 改为 224，尽量保持原裁剪比例；仍复用官方预处理与 softmax/top-1。此分支为 CPU 单推理流、2 线程，主链路的骨架与吸烟仍各自保持原候选的线程配置。
5. 模型身份记录同时包含 OpenVINO XML 和 BIN 的 SHA256，运行信息明确标注 OpenVINO CPU，不冒充 ONNX Runtime 或 CUDA。
6. 本次不继续优化吸烟：模型权重、分支代码、阈值与候选配置保持上轮状态，仍为 Paddle FP32。上轮 CPU 候选已启用的 MKLDNN/oneDNN、4 线程与有界缓存保留。打架 OpenVINO 预测器单独构造，不替换吸烟预测器。

打架原 Paddle 的 `.pdmodel` 与 `.pdiparams` 合计 96,306,459 字节；224 版 XML 与 BIN 合计 24,923,256 字节（约 24.9MB）。上一轮 320 版为 24,919,125 字节。缩小输入主要减少计算量，权重体积基本不变；224 输入像素数是 320 的 49%，不代表已实测两倍速度。原模型及上一轮 ZIP 保留供回退。

打架使用 OpenVINO 原生 `start_async`，每次轮询只调用 `wait_for(0)`，不在主识别链路等待整组计算。最多一个在途窗口，没有等待任务队列；忙时继续更新 8 帧滑动窗口，空闲后只提交最新完整窗口，保持原 0.4667 秒最小提交间隔。它不保证 CPU 无争用，也没有解除吸烟/骨架本身的耗时限制。

新结果只把 `updated` 置真一次，`prediction_stamp_ns` 始终是原窗口结束的图像时间。源变化、时间倒退、尺寸变化、断流或采样间隔超限后，旧任务即使完成也不参与判断。超过原 1 秒有效期的结果会丢弃；没有把完成时间冒充采样时间，也没有延长 TTL。后台异常会向 worker 抛出，持续运行超时为 5 秒；退出时取消并回收自己的推理请求。

结果增加 `scheduling.fight`：`pending`、`submitted_windows`、`completed_windows`、`discarded_windows`、`last_drop_reason`、`last_model_inference_ms` 和当前窗口信息。`timing_ms.fight_async_model` 是刚完成的后台模型耗时，与主链路可能重叠，不应与 `total` 相加。已有 ROS/预览按真实证据时间继续执行过期判断。

没有放宽行为阈值、人物关联间隔、连续观测窗口或结果过期时间。原来的近脸 0.65/0.30 和侧脸补检规则保留。是否能解决“等待连续观测 / 画面已过期”，需要等用户通知后测整条链路，当前不能确认。

## 文件

- `config/inference.yaml`：当前源码默认配置，基于侧脸补检配置。`config/inference_cpu_int8_candidate.yaml` 保留为相同内容的兼容入口。
- `config/ros.yaml`：默认使用 CPU 环境脚本；`config/ros_cpu_int8_candidate.yaml` 保留为显式覆盖入口。
- `output_inference/yolov8m_pose_int8_cpu_candidate/model.xml`、`model.bin`：量化产物。
- `output_inference/ppTSM_int8_224_cpu_candidate/model.xml`、`model.bin`：最新 8 帧/224 打架量化产物，由 `fight.backend: openvino` 与 `models.fight` 指定。不调用加载 Paddle 权重的构造器，因此部署包不需要旧打架 FP32 权重。`fight.async_inference: true` 启用异步，设为 `false` 可在相同 224 模型上回到同步执行，用于日后定位对比。
- 同目录 `quantization.json`：原模型指纹、工具版本、校准信息及待验证状态。
- `tests/test_cpu_int8_candidate.py`、`tests/test_fight_async.py`：已准备、未运行的配置、张量接口、单任务限制、时间戳、断流/过期丢弃和超时检查。

本次 Gitee 发布已把源码默认 `config/inference.yaml`、`config/ros.yaml` 和 `VERSION` 切到本 CPU 版本。原 FP32 模型保留，原默认推理配置另存为 `config/inference_gpu.yaml`，GPU 启动使用 `config/ros_gpu.yaml`。两个 INT8 模型及其元数据一并纳入 Git，BIN 使用 Git LFS；测试录像、校准数据和量化工具不纳入 Git。发布不代表已通过效果验收，也没有自动切换远端正式服务。

## 环境边界

目标机是 Ubuntu 22.04、Intel i5-13500H，已存在独立行为识别解释器及 OpenVINO 2024.6.0、Paddle 2.6.2、ONNX Runtime 1.18.0。**本次没有安装、升级、卸载或修改目标机的任何依赖，也没有停止其机器人或摄像头服务。**

Windows 上仅把 OpenVINO 2024.6.0、NNCF 2.14.0 等量化工具放入项目 `verification/cpu_int8_20261102/tools`，由量化进程临时加载，没有安装到原 Conda/Python 环境。目标运行时不需要 NNCF 或校准图片。当前 Windows 正式环境没有自动新增 OpenVINO，不能假设直接切候选配置就已具备依赖。

此前尝试同时转换辅助模型时，吸烟图在目标机 OpenVINO 2024.6 的动态维度 Squeeze 上编译失败。该吸烟实验适配器没有纳入本候选。本轮只处理打架：使用单窗口静态输入导出量化图，吸烟保持 Paddle。目标机上的打架新后端仍未启动验收。

## 校准与待验证工作

量化使用 64 个视频的 256 帧，固定随机种子 `20261102`。通过原路径与 SHA256 排除之前 50 段回归文件；AIRTLab 同事件的不同摄像头按组排除。校准与回归不是同一批输入。这些都是已收集的数据，不能宣称为全新独立测试集。

校准过程是生成 INT8 模型所需的离线统计，不是准确率评测。生成记录在 `verification/cpu_int8_20261102/quantization.json`，校准清单在同目录 `calibration.json`。

打架使用上述来源中可提供完整时间窗口的 63 个视频、126 个窗口，每个窗口为 8 个不重复的真实视频帧，按约 4.286Hz 采样。224 版从同一批原始帧重新做 238 缩放与 224 裁剪，再重新校准，没有缩放已经裁好的 320 图片。脚本只改导出图中 33 个空间 Reshape 的 H/W 和输入空间维度，保留时间维 8、时移 Slice 边界和卷积/分类参数。原图和改图的指纹、变更列表在产物 `quantization.json`；脚本与清单是 `verification/cpu_int8_20261102/calibrate_fight_224.py`、`fight_224_calibration.json`。这不是重新训练，也不是精度等价性验证。校准不读取之前 50 段回测视频。

用户通知后再进行：模型实际加载与算子精度检查；目标机整链路耗时和连续观测稳定性；50 段与侧脸视频的漏检/误报对比；新增测试；最后再决定是否切换正式项目。禁止为得到连续提示而把旧检测结果刷新成新证据。

日后做离线逐帧精度比较时，使用同一模型的同步模式，或让异步回放遵循源视频的真实时间。不能让离线时间戳高速跳过后台计算，再把因此过期的结果当成模型漏检；实时链路验证则必须保留异步和真实时钟。

独立的 `scripts/setup_ubuntu_gpu.sh` 会检查 NVIDIA/CUDA，**不要在纯 CPU 目标机运行它**。默认 `scripts/setup_ubuntu.sh` 已转到 CPU 安装入口；它只在新环境中安装依赖。待验证时应复用现有行为识别环境，仅在独立候选目录启动，随后再与现有网关或 ROS 入口对接。

目标机 `verification/cpu_int8_20261102/final_candidate` 仍是依赖既有环境与上游源码/原始权重链接的候选目录，不是独立部署压缩包；其中没有完整的摄像头/ROS 启动脚本。此次同步仅保存文件，没有运行该目录、修改网关配置或切换正式版本。

另按用户要求制作 Windows 上的 CPU 部署 ZIP：该包物化保存所需上游源码与三模型，不使用目标机目录链接，带完整 USB/ROS 入口、独立 CPU Conda 安装脚本和说明。压缩包和本次发布的源码均以该 CPU 优化配置为默认；远端正式服务没有因此切换。包的构建与文件完整性检查不是模型验收；安装与实际识别仍待用户通知后验证。
