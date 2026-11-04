# 机器狗巡检项目排查与恢复手册

适用于本地 `test` 整合版本。人工验收顺序见 [功能验收清单](FIELD_ACCEPTANCE.md)，完整依赖和编译说明见 [README](README.md)。命令在机器狗小电脑终端执行；浏览器所在电脑的 `127.0.0.1` 不是机器狗。本文不会要求直接发布速度或绕过启动条件来排障。

## 1. 先区分四种状态

1. **页面加载成功**：只证明静态文件可读。
2. **ROSBridge 已连接**：只证明 WebSocket 通信建立。
3. **导航/相机/任务就绪**：要看对应服务、模式、地图、定位和硬件状态。
4. **实际运动/识别/录像成功**：要核对实物、最新结果或可播放文件。

绿色“开始”按钮只表示页面认为可以提交，不等于后端已经创建任务。故障时记录点击时间、页面日志和接口响应；不要用不停重试覆盖第一次错误。

## 2. 系统入口与数据位置

| 入口 | 默认位置 | 用途与区别 |
| --- | --- | --- |
| 正式页面 | `http://机器狗IP:8080/inspection.html` 等 | 静态页面，升级后 Ctrl+F5 |
| ROSBridge | `ws://机器狗IP:9090` | 导航/地图/巡检 ROS 服务与状态 |
| 相机网关 | `http://机器狗IP:8766` | C12、普通视频、识别、录像、热成像代理 |
| USB 热相机服务 | 小电脑 `127.0.0.1:8767` | 海康 SDK 测温，与 C12 红外预览不同 |
| 保存点云服务 | `8769` | 完整保存 PCD 读取，与实时点云话题不同 |
| 星玑本体图传代理 | `8081` | 原链路中的本体相机来源 |
| C12 上游 | 可见光 `192.168.144.108:554/stream=1`；红外 `:555/stream=2` | 实际地址以启动参数为准；不在排障中随意改相机网络 |
| 用户持久数据 | `~/.ros/robot_navigation/` | 本体、导航、相机、行为、阈值配置和数据库 |
| 路线/巡检库 | `routes.db` / `inspection.db` | 位于上述用户数据目录；SQLite 复制前结束写入服务 |
| 识别库/证据 | `recognition.db` / `recognition_evidence/` | 人员/车辆/事件；可能含个人信息 |
| 巡检证据 | `~/.ros/robot_navigation/inspection_data/` | 报告关联图片等 |
| 双路录像 | `~/巡检录像/` | 机器狗用户为 a 时是 `/home/a/巡检录像` |
| 地图资源 | `src/ros2_humble_main/src/navigation/robot_navigation2/maps/`、`pcdmaps/` | 以实际启动参数为准；PGM/YAML/PCD/地形资源应一并保存 |
| ROS 日志 | `~/.ros/log/` | 配合本次启动的完整终端日志 |
| 录像显示/预览档位 | 当前浏览器 localStorage | 不随服务器源码同步；不同电脑需分别设置 |

后台启动参数可覆盖默认目录和端口。切换 Linux 用户会改变 `~`，这可能表现为“路线/配置突然不见了”，先查运行用户和参数再恢复数据。

## 3. 三分钟快速定位

在故障现场保留页面和后台状态，按顺序做：

- 页面空白/按钮全没反应：浏览器 F12 → Console 看首个脚本错误；Network 看 JS/CSS 是否 404；确认 8080 服务目录指向本版本。
- 页面能用但全部 ROS 操作失败：看 9090 连接，执行 Q02/Q03。
- 导航能读状态但不能开始：执行 Q04–Q06，并核对模式、地图、定位、当前任务占用。
- 视频或识别问题：先执行 Q07/Q08，分清原图、识别结果、C12 红外和 USB 热图。
- 录像停不下来/其他页面停帧：执行 Q09/Q10，先取状态和系统资源，不直接杀整套后台。
- 电脑重启：执行 Q11；区分正常 reboot、OOM、内核/硬件故障和断电，不能只凭“录像之后发生”认定原因。

### 绿色开始按钮点击无反应

1. **记录输入**：地图、路线 ID、点数/检测点数、轮次、模式、本体、任务名。无检测点路线仍可作为经过点巡检，不能仅以检测点数为零断言不允许启动。
2. **看浏览器**：F12 Network 的 WebSocket 消息里查 `/inspection/start`。没有发送则看 Console 和页面提示；有发送则保存 `service_response` 的 `success/message/task_id`。不要只截按钮颜色。
3. **看服务**：Q03 查 `/inspection/start` 是否存在；`ros2 service type /inspection/start` 应与前端的 `lzp_interface/srv/StartInspectionTask` 一致。接口报字段/类型错误时按第 8 节重编译接口，不能只更新 HTML。
4. **看任务**：Q05 的 `/inspection/status` 是否创建任务、是否已终止/失败、是否正在预约路线；查 `/route_execution_status` 是否被别的任务占用。
5. **看导航**：Q04 的模式、当前地图与初始定位是否一致，三维是否完成对应路线预览；Q06 查零速原因。完成初始定位不能代替 Nav2 就绪或三维路线预览。
6. **收到 success 但不动**：沿“巡检 → 路线 → Nav2 → 速度仲裁 → 本体”逐层定位；查看 `NAV_GOAL`、`NAV_VELOCITY`、`NAV_BODY`。不要直接发布速度验证。
7. **停止未完成**：等待取消确认并保留最早错误。确认任务空闲后再重新预览/开始，禁止通过移除任务归属判断强行运行。

## 4. 只读诊断命令

先在实际运行工作空间中加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

`timeout` 到期只能说明此时没有及时收到结果，不能单凭超时证明节点死亡。不同 shell 请重新加载环境；同一个 ROS_DOMAIN_ID 和 RMW 配置才可正确观察后台。

### Q01 版本与环境

```bash
pwd
git branch --show-current
git rev-parse HEAD
git status --short
ros2 pkg prefix car_vel
ros2 pkg prefix lzp_service
ros2 pkg prefix lzp_interface
ros2 pkg prefix icp_registration
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION
```

检查包路径是否来自同一工作空间。不要在日志里粘贴完整环境变量、密码或相机访问凭据。

### Q02 端口与资源

```bash
ss -ltnp | rg ':(8080|9090|8766|8767|8769|8081)\b'
free -h
df -h "$HOME"
uptime
```

端口占用不代表服务健康；打开页面和查询状态还需实际成功。磁盘满会同时影响录像、报告、数据库和日志。

### Q03 ROS 节点与接口

```bash
ros2 node list
ros2 service list | rg 'inspection|start_route|navigation|relocal'
ros2 action list -t
ros2 service type /inspection/start
ros2 interface show lzp_interface/srv/StartInspectionTask
ros2 interface show lzp_interface/srv/Preview3DRoute
ros2 interface show lzp_interface/action/Relocalize
```

三维结果应包含 `transform_model`，路线预览包含当前版本字段。发现包路径混用先修安装环境。

### Q04 导航启动、地图和定位

```bash
timeout 8s ros2 topic echo /navigation/startup/status --once --full-length
timeout 8s ros2 topic echo /navigation/mode --once --full-length
timeout 8s ros2 topic echo /current_map_info --once --full-length
timeout 8s ros2 topic echo /relocalization/status --once --full-length
timeout 8s ros2 topic echo /localization/status --once --full-length
```

三维核对 `scope`、`transform_model`、`map_name`、`map_load_id`、`body_generation`、`ready` 和原因字段。二维不能照搬三维 `scope` 标准；应看当前模式、AMCL 和交接状态。

### Q05 巡检与路线

```bash
timeout 8s ros2 topic echo /inspection/status --once --full-length
timeout 8s ros2 topic echo /route_execution_status --once --full-length
timeout 8s ros2 topic echo /goal_pose_status --once --full-length
timeout 8s ros2 topic echo /nav2_ready --once
timeout 8s ros2 topic echo /navigation_active --once
```

同时核对 task_id/owner_token、state、paused、当前点和轮次。不同时间采集的状态可能跨越同一次转换，分析时保留各自时间。

### Q06 速度链与本体

```bash
timeout 8s ros2 topic echo /navigation/velocity_guard --once --full-length
timeout 8s ros2 topic echo /robot_body/status --once --full-length
timeout 8s ros2 topic echo /lio_health --once --full-length
timeout 8s ros2 topic hz /Odometry
```

`commanded_velocity` 是已发布指令，不是实测移动。`udp_dispatched_unconfirmed` 只证明发送记录。字段解释见 [运动日志](src/ros2_humble_main/src/car_vel/NAVIGATION_MOTION_LOGS.md)。

### Q07 相机与显示来源

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/status
curl -sS --max-time 5 http://127.0.0.1:8766/api/camera/source
curl -sS --max-time 5 http://127.0.0.1:8766/api/display/status
curl -sS --max-time 5 http://127.0.0.1:8766/api/c12/status
```

先看 pipeline 和所选显示通道。网页选择“只看普通光”时红外隐藏是设置结果。若需要读状态 JSON，可追加 `| python3 -m json.tool --no-ensure-ascii`。

### Q08 识别、热成像和环境

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/yolo
curl -sS --max-time 5 http://127.0.0.1:8766/api/behavior/status
curl -sS --max-time 5 http://127.0.0.1:8766/api/recognition/status
curl -sS --max-time 5 http://127.0.0.1:8766/api/thermal/status
curl -sS --max-time 5 http://127.0.0.1:8767/api/status
timeout 8s ros2 topic echo /air_quality/m701 --once --full-length
timeout 8s ros2 topic echo /inspection/environment_config --once --full-length
```

原链路的 USB 测温异常与 C12 无测温能力分别处理；不能通过调阈值修相机断流。

### Q09 录像状态与文件

先确认小电脑具备运行依赖；缺少程序时按部署依赖流程安装，不把模拟进程测试当作真实录像验证。

```bash
command -v ffmpeg
command -v ffprobe
```

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/c12/recording
ls -lh "$HOME/巡检录像"
```

`starting` 表示等待两路文件产生数据；`recording` 表示文件在写入；停止后 `stopping` 表示收尾，最终 `idle` 或带原因的 `error`。正常停止无需等 HTTP 请求一直占用到保存结束。开始后当前实现最多等待约 20 秒取得两路数据；结束时每路有有限退出等待，保存数秒是允许的。

把下例文件名替换成真实文件，停止完成后逐个检查：

```bash
ffprobe -v error -show_entries format=duration,size:stream=codec_name,width,height -of json "$HOME/巡检录像/实际文件名_visible.mp4"
ffprobe -v error -show_entries format=duration,size:stream=codec_name,width,height -of json "$HOME/巡检录像/实际文件名_infrared.mp4"
```

还要实际播放两个文件。碎片 MP4 不等于零丢帧保证；断流/被强杀的文件需单独检查，不能删掉失败样本再宣称通过。

### Q10 视频并发与进程负载

```bash
ps -eo pid,ppid,comm,%cpu,%mem,etime --sort=-%cpu | head -25
free -h
df -h "$HOME"
```

本版录像额外建立两路 RTSP 连接并 stream-copy，不改变现有预览流，也不转码。若录像开始后两路预览一起消失，依次保存 Q07/Q09 和资源状态，再测试“停止录像后预览是否恢复”。这用于区分相机并发连接、网络、解码资源和后台问题；尚未确定其中哪项是之前现场异常的根因。不要同时开多个外部播放器去增加 RTSP 连接数。

### Q11 重启或进程被杀

```bash
journalctl --list-boots
journalctl -b -1 -k --no-pager | tail -200
journalctl -b -1 --no-pager | rg -i 'oom|out of memory|killed process|thermal|watchdog|reboot|shutdown' | tail -100
last -x | head -20
```

上一轮日志不可用时，说明未持久化或无读取权限，不能得出“未发生 OOM/硬件故障”。保存重启前完整应用日志、系统日志、供电情况和发生时间；缺少内核/关机记录时继续排查电源，不由相关性断言录像触发系统重启。

### Q12 模型完整性

```bash
python3 src/ros2_humble_main/src/car_vel/person-behavior-recognition/scripts/verify_runtime_assets.py
```

失败时先区分缺文件、Git LFS 指针、大小/哈希不符。用匹配版本的交付资源恢复，不能换任意同名模型。校验通过后再检查识别独立环境、模型实际加载、采样和准确率。

### Q13 日志查找

将路径替换为本次运行日志：

```bash
rg -n 'Traceback|ERROR|NAV_LAUNCH_CONFLICT|NAV_GOAL|NAV_VELOCITY|NAV_BODY|NAV_PROGRESS|GLOBAL_LOCALIZATION|REGISTRATION_CONTRACT|C12|录像' /实际路径/本次后台.log
```

先保留完整日志，再做筛选。提交给开发人员时附完整问题前后片段，隐去凭据和无关个人信息。

## 5. 常见现象与下一步

| 现象 | 先看 | 处理方向 |
| --- | --- | --- |
| 地图加载后仍空白 | Q04、浏览器 Network、8769 | 区分栅格没收到和完整点云没下载；核对地图资源及加载批次 |
| 设置初始位姿一直等待 | Q04/Q06、`/scan`、地图与实景 | 区分二维质量不通过、三维精修未返回、TF/接口不匹配；不伪造 ready |
| 一直等 Nav2 | Q04 startup 的 lifecycle/缺失节点 | 查看原始启动错误；进程冲突按空闲恢复，不重复 launch |
| 三维预览失效 | 当前地图、定位批次、本体/路线是否变更 | 确实变更就重新预览；纯等待不应倒计时失效，检查前端是否旧缓存 |
| 导航显示运行但不动 | Q05/Q06 和三个 NAV 日志层 | 分开查未发目标、输出零速、本体拒绝、已发指令但未动 |
| 二维刷新后继续了错误目标 | N03/N04/N06、目标日志 | 保存第一次点击和取消时间；不要再发新目标掩盖错误 |
| 巡检暂停后又走 | Q05 中巡检和路线是否同一归属 | 保存控制服务响应和晚到启动响应；作为 P0 失败处理 |
| 巡检结束但下一轮不启动 | state、轮次间隔、当前点事件 | 区分等待间隔、返回起点、完成事件丢失；用两点短路线重现 |
| 云台不动 | C12 pipeline、控制接口响应、网络 | HTTP 发送成功不等于固件执行；先用方向控制，不先改安装/网络参数 |
| 云台松手不停止 | 页面事件、断连状态、控制租约 | 停止操作并检查相机，保存保持按键/失焦顺序；不得继续演示 |
| 录像按钮消失 | 设置显示开关、Q09 | 空闲默认隐藏正常；录像中必须保留停止入口，换浏览器需重设 |
| 点击停止一直保存 | Q09/Q10、FFmpeg 是否退出 | 新版状态应可查询；记录持续时间与进程，再做正常后台关闭；不要先重启电脑 |
| 两个 MP4 一个打不开 | 文件大小/ffprobe/上游断流 | 保留两文件及错误状态，不能用另一个复制替代 |
| 原图有，YOLO 画面无 | Q08 | 行为识别正在运行时普通 YOLO 可能按互斥策略停止；查 running/ready/error |
| 行为一直预热/过期 | Q08、帧时间与CPU | 实际采样不足或处理慢；不能重复帧凑窗口、延长有效期冒充实时 |
| C12 红外有图但无温度 | Q07/Q08 | 当前未接入测温时正常；温度验收要使用具备测温能力的原链路 |
| USB 热图全无 | `lsusb -d 2bdf:0102`、Q08 | 先查枚举/权限/占用，再查SDK；不直接卸载驱动或重置USB总线 |
| M701 失联 | 串口实际设备、Q08 | `ls -l /dev/serial/by-id/` 核对设备；不把旧值当实时数据 |
| 人脸/车牌失败 | 登记库、模型、Q08 | 先查模型加载与图像来源，再看阈值和样本；只改一项并记录 |
| 报告缺图/删除不一致 | 数据库与证据目录是否同一备份 | 停写后恢复同一批数据，不能只替换 DB 或只替换图片 |
| 本体切换失败 | 旧任务停止结果、旧本体反馈 | 结束任务并按确认流程操作；不得把断连自动解释为已安全停放 |
| 修改已生效但网页仍旧 | Q01、静态服务目录、缓存 | 检查实际运行目录与已安装包路径；Ctrl+F5，避免仅更新源码未构建 |

## 6. 正常停止与恢复

1. 导航/巡检异常时按现场规程先让机器人安全停止。页面终止任务、停止录像、松开云台，并确认后台状态。
2. 录像可用时通过页面停止。若页面损坏但网关可用，可在小电脑执行下面的**有状态操作**，它只停止录像：

```bash
curl -sS --max-time 5 -H 'Content-Type: application/json' -d '{"operation":"stop"}' http://127.0.0.1:8766/api/c12/recording
```

3. 确认任务和录像结束后，在原后台启动终端 Ctrl+C。等待进程退出，再查看端口和进程，不使用 `pkill -9 python`、`killall` 或清空 ROS 配置作为常规恢复。
4. 重新启动一套后台。重启后必须重新核对本体、模式、地图、初始定位；不直接恢复此前未确认的运动。
5. 原页面断开不是后台停止证明；没有原启动终端时先查服务管理方式和进程归属，不启动第二套“覆盖”旧后台。

## 7. 升级前备份与回退

### 必须一起保留

- 原工作目录、提交号、源码/模型、构建和启动命令、当前依赖/识别虚拟环境位置。
- `~/.ros/robot_navigation/` 全目录，含配置、SQLite 数据库及相关 WAL/SHM、证据。
- 当前实际地图目录的栅格、点云、定位和地形文件；录像与完整日志按需要另存。
- 本体、雷达、相机实际地址和设备标识；浏览器里的录像显示、预览档位另外记录。

后台停止后，示例备份用户数据；目录以实际部署为准：

```bash
snapshot="$HOME/3dnav-backup-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$snapshot"
cp -a "$HOME/.ros/robot_navigation" "$snapshot/"
git rev-parse HEAD > "$snapshot/source-commit.txt"
```

若源目录不存在，不创建一个空目录冒充备份；先核对运行用户和自定义路径。地图和整个旧工作目录另外保留，不能靠这三条命令代替完整备份。

### 候选部署

将交付源码包解压到**新的候选目录**，先补齐本机依赖，复制现场需要的地图副本，在候选目录重编译。源码包中的示例地图不能直接覆盖现场生产地图。前端服务和 ROS overlay 必须一起指向候选版本；不要让网页来自 test 而后端来自旧 dev。

确认后台空闲并停止旧实例后才切换；先只读状态，再按验收表运行。回退时停止候选实例，使用保留的旧工作目录及其对应构建环境；必要时恢复同一批用户数据。不要只回退 HTML，也不要将新 Action 接口与旧原生库混用。修改前已有未提交内容时先保存，不用 reset/clean 抹掉现场修改。

## 8. 构建与安装一致性

当前代码包含三维接口、原生节点和 Python 适配；必须共同构建。已有依赖的工作空间可按 README 的全量构建流程；低内存机器先采用顺序构建：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 src/ros2_humble_main/src/car_vel/scripts/check_3d_navigation_install.py --workspace "$PWD" --runtime-timing
```

安装检查是只读检查；通过后仍要观察启动期 `REGISTRATION_CONTRACT` 和实际定位状态。若报 `octomap_msgs` 缺失，按 README 的 ROS 依赖安装流程补齐后重编译；若 Livox 报 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES` 未找到，先核对 Humble 参数。不要通过关闭相关包或跳过接口生成来把构建显示成成功。

本地本轮没有远程部署、后台重启、系统依赖安装或阈值调整。

## 9. 提交给开发人员的最小证据

用以下内容提交一次问题，避免只发“按钮没反应”：

```text
问题编号/发生时间（含时区）：
机器狗实际 IP / 本体 / 软件提交号 / 工作目录：
页面 / 导航模式 / 地图 / 路线 / 轮次 / 检测点数：
相机来源 / 视频通道 / 普通或行为识别 / 是否录像：
复现步骤（从上一个正常状态开始）：
预期 / 实际 / 是否涉及运动 / 如何停止：
浏览器 Console 首个错误与接口响应：
相关 Q 编号输出、完整后台日志、系统日志路径：
录像双文件或检测证据路径：
一次恢复或回退后的结果：
```

三维问题交给三维开发负责人时保留 master 基线和本地整合提交号；C12/录像问题附并发页面数和先后顺序；重启问题附上一启动周期日志。这样可将问题定位到改动模块，而不是重新复测整个系统。
