# 星玑三维定位输出与防跳变

本次改动针对星玑 `terrain_3d`：原始 AMCL 不再直接驱动该范围内的导航位置/TF。原星玑二维与钢镚保留 AMCL 算法、导航参数、步态与速度配置。为统一页面和后端的位置来源，各导航消费者订阅 `/localization/navigation_pose`；在原有模式下，节点原样转发 AMCL 消息，包括原时间戳与协方差，不伪造定位心跳。

## 输出流程

1. 初始定位仍使用已保存的定位 PCD、按采集时间查询的 LIO TF 和 NDT/GICP。只对星玑三维请求启用新增 Action 字段 `robust_validation`，并核对 `map_load_id`。
2. 最多精修三个空间不同的粗配准候选；分别验证高度、倾斜、RMSE、重叠率、特征退化和多帧一致性，再比较合格候选。空间分离而质量接近时拒绝确认。初始/原有模式保持原门限，星玑三维连续跟踪采用本文末尾的独立适度放宽参数。
3. 星玑三维保留**通过校验的完整刚体变换**，结果、点云校验、LIO组合和唯一`map→odom`使用同一几何；不再丢弃roll/pitch/z后二次拒绝。Action结果`transform_model=se3_map_odom_v1`，旧平面模型不能静默初始化新定位。Nav2仍读取XY/朝向做平面路径与速度控制，没有改成六自由度路径控制。
4. `global_pose_authority` 持续用完整LIO位姿推算位置，单线程 Action 进行局部三维匹配。局部请求搜索范围与接收界限统一为0.20 m、6°，预算2.0 s，两个请求启动之间至少0.75 s，不运行两个配准工作线程。粗位姿保留已验证高度/姿态，并按请求时间对应的LIO变换预测到最后一帧点云时间。
5. 局部结果必须通过服务器的候选质量检查，并在**相同采样时间的机器人位置**处检查创新量：XY最多0.20 m、朝向6°，高度修正0.45 m、倾斜创新8°；真实楼梯高度变化由LIO连续传递，不与常量0或整段累计上升高度比较。新结果最大年龄1 s，重复/倒序/未来时间戳拒绝。不得把不合格结果插值或钳位成合格位姿。
6. 正常匹配锚点期限为 3 s；超时后最多再有 2 s 的零速复核窗口，一次新的完整可信匹配通过即可继续运动。LIO流在3个真实新TF后独立完成恢复，实际采样间隔后的地图复核每次不超过3s且不超过原锚点5s总边界；地图等待不会再次使已恢复LIO误报掉流。允许运动仍要求新鲜LIO/TF，窗口不因失败结果或心跳延长。真实重置、地图重新加载、本体代际改变后旧锚点与旧Action结果作废。

这些是保守的初始工程门限，需要在目标机器上测量匹配耗时和误差后评估。重复结构仍可能产生小幅错误匹配；不能据此保证任何环境下永不误定位。

## TF 发布权和停车

- AMCL 运行参数 `tf_broadcast` 支持动态修改。进入已确认的星玑三维范围时，必须先确认原值、写入关闭、独立回读，并等待原 TF 流安静后，统一定位才可发布 `map→odom`。
- 写入前保存权限记录 `~/.ros/robot_navigation/global_pose_authority.json`。记录包含实际原值、AMCL 发布者 GID 和未决写入标记。超时写入继续等待原响应；不能假定取消本地 Future 就取消了远端操作。记录异常或未决状态不允许猜测发布权。此时停止整个后台再检查记录/重启。
- 不要手工同时启动另一个 `map→odom` 发布程序。检测到外部同名 TF 后统一定位停止输出。
- Python TF 接收使用 Humble 的单消息回调；通过有界缓存排除本节点发出的完整 TF 回显（含时间戳），其他同名变换会阻断输出。Humble 的普通 Python 回调不提供新版的发布者标识接口，不能依赖该接口判断来源。[Humble 执行器实现](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py)
- 退出星玑三维前先停止任务。恢复原 AMCL TF 前还要核对位置：有新鲜、同本体锚点时在静止状态下同步 AMCL；需要三个新时间戳样本与初始化位置一致，并通过协方差及静止检查。没有有效锚点、已换本体或同步失败时，在原模式重新设置初始点。原生二维启动不增加这一步。
- 行进中短暂匹配过期，速度仲裁器输出零速，路线管理器保留当前任务限时复核；期间不启动新目标或复用旧预览。窗口内一次新的完整可信匹配通过即可恢复，并要求新的控制器速度。真实失效、跳变、重置或窗口耗尽则取消任务、保持零速锁定；晚到的匹配不会自动续走，必须停止任务、重新确认初始点、预览并人工启动。
- 本模块不发站立、步态、速度补偿或硬急停指令。原有遥控、急停按键和其他保护仍按现有逻辑工作。星玑二维、钢镚不采用新增的三维匹配门限。

## 小电脑更新步骤

**最新部署以本文末尾“2026-10-29 刚体三维模型”章节为准。** 此次新增Action结果字段，历史仅重建Python包的命令不足。

### 2026-10-28 初始精修成功但输出未确认的修复

若日志先显示 `NDT/GICP 精修成功`，随后出现
`定位TF权限同步失败：'dict' object is not callable` 和
`统一三维定位输出未确认`，这是统一定位节点的方法名与
`rclpy.Node._parameters` 内部参数字典重名，导致权限确认流程未执行。
不是这条错误证明地图损坏或 NDT/GICP 搜索范围不足。

已将自定义方法改名为 `_sync_tf_parameters`，保留原参数字典、独立回读、
TF 安静交接和质量校验。回归覆盖了真实节点会持有参数字典的场景，以及
“参数读写/回读 → 合格精修结果 → 定位/TF 输出 → 同请求确认”的串联流程。
页面在地图/定位服务已启动且已加载地图时，会明确提示先设置初始点；
这条提示不会把 Nav2 未就绪状态改成允许执行目标。

若已部署前一版三维定位及 Action 接口，这次仅需同步更新并重建 `car_vel`：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select car_vel
source /home/a/3dnav/install/setup.bash
```

停止旧后台后重启，并在浏览器强制刷新。保持机器人静止：加载地图，切到三维俯视，
拖拽位置/朝向并点击“确认初始点”，再读取 `/localization/status`。
确认没有上述字典调用错误，`scope=xingji_3d`、`ready=true`、
`tf_broadcast_actual=false`、地图身份和 `seed_request_id` 对应本次请求。
现场核对位置/朝向之前不要开始导航。若有新的匹配/时间错误，应保留完整日志继续排查。

### 首次部署整套三维定位更新

先停止当前整个导航后台。同步修改后的代码后，在小电脑执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select lzp_interface icp_registration lzp_service car_vel
```

本次修改了 Action 接口，必须四个包一起更新，不能只重建前端或 car_vel，也不能保留旧的配准服务进程。

新终端加载环境，再用原有命令启动整套后台：

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
```

浏览器强制刷新。先保持机器狗停止，选择星玑和三维模式，加载地图、确认初始点，先不要点击开始导航。

以下命令每条单独执行：

```bash
ros2 param get /amcl tf_broadcast
```

已进入星玑三维且完成发布权交接后应为 `False`。

```bash
timeout 8s ros2 topic echo /localization/status --once --full-length
```

确认 `scope=xingji_3d`、`ready=true`、地图与 load_id 正确、`sensor_fresh=true`；正常时 `age_sec` 应周期性回落并在 3 s 内。`recovering=true` 时属于零速复核，不是允许行走。查看 `rmse_m`、`overlap` 和 `error`，不能只看 ready 字样。

```bash
timeout 8s ros2 topic echo /localization/navigation_pose --once
```

将地图上的机器人位置、朝向与现场核对，静止观察至少 30 s。再进行低风险短距离单点测试，之后才测试多点与巡检。现有越障模式和速度上限没有改动；若停止，先读定位状态，不要放宽门限绕过。

切回二维后检查原值已恢复（默认配置通常为 `True`），且 `/localization/status` 为 `scope=legacy, ready=true`。原值来自实际读取，不能一律强制写 True。若 `legacy_initialization_required=true`，先保持停止并在二维模式重新设置初始点。

建议后续 bag 同时记录 `/localization/status`、`/localization/navigation_pose`、原始 `/amcl_pose`、`/tf`、`/tf_static`、`/Odometry`、`/cloud_registered`、`/lio_health`、`/navigation/velocity_guard` 和原有控制/任务话题，用于区分匹配质量、输出接纳与执行保护。

## 已验证与尚未验证

本地回归包含实际 bag 解码得到的 70 次 TF/LIO 样本：跳变前 30 次正常修正接纳，1.7 m / 53° 跳变后 40 次修正拒绝，锚点失效后停车。该回放故意假设候选质量合格，以单独检验创新量保护；**没有把旧 AMCL 输出当成新 PCL 配准结果**。

还验证参数写入/回读/晚响应、本体与模式隔离、运行中失效锁定、旧 Action 作废、二维消息原样转发和 AMCL 交接检查。C++ 候选打分与歧义辅助函数已编译运行。

已隔离且明确 `temporary_stop=false, hard_stop=false` 的单个可疑 LIO 采样不会单独撤销原锚点；新鲜的已接纳 TF、独立连续性与三维匹配检查仍需满足。另有下述明确回退的扫描质量短暂恢复窗口；真正重置、故障或持续无新 TF 仍会阻断输出，不会把重复 TF 当成新测量。

当前开发机没有 Linux ROS/PCL 运行环境，新增完整 PCL 配准链尚未在该 bag 上完成实际匹配和耗时测量；也没有进行实机运动测试。因此不能将上述回归视为现场可靠性验收。必须在小电脑完成完整编译、静止检查和逐步实测，再评估 CPU 负载及 3 s 正常匹配期限是否能持续满足。出现初始化失败应检查 PCD/坐标、高度与重力，而非恢复粗定位自动放行。

### 初始配准质量拒绝与独立优化预算

若字典调用错误已消失，但所有候选因 RMSE、垂直/倾斜修正或搜索边界被拒绝，
应与“精修成功但输出确认失败”分开处理。2026-10-28 提供的 21.3 秒 bag 中，
23 条 LIO 健康消息均为 healthy，209 个 odom→base_footprint TF 样本的相邻步长
都通过当前连续性检查；这不证明地图/外参正确，也不证明未记录的时段没有问题。
该 bag 未记录初始点请求，原始粗位姿也未保留，因此没有精确重放当时 NDT/GICP。

代码为 `robust_validation=true && tracking=false` 请求拆分初始优化预算：

| 请求 | NDT 外层 | GICP 外层 | GICP 内部优化 |
| --- | --- | --- | --- |
| 原二维/钢镚及连续跟踪配置 | 原配置 6 | 原配置 6 | 原配置 4 |
| 星玑三维初始精修 | 20 | 40 | 12 |

上限参数分别是 `initial_ndt_maximum_iterations`、
`initial_gicp_maximum_iterations` 和 `initial_gicp_maximum_optimizer_iterations`。
只改变初始优化机会，不扩大 3 m / ±90° 自动接受范围，不修改 RMSE 0.20 m、
重叠率 55%、垂直修正 0.35 m、多帧一致性、候选歧义及输出平面变换校验。
初始请求仍受 30 秒总预算和取消检查约束。

[PCL 1.12 的 GICP 实现](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/registration/include/pcl/registration/impl/gicp.hpp)
在小变化或达到迭代上限时都可能设置收敛标志。新增日志输出实际迭代次数；
初始精修达到次数上限时不当作已经证明稳定收敛。最终仍必须通过原质量检查。

日志新增 `REGISTRATION_REQUEST`（地图、加载身份、粗位姿/朝向、请求时间和范围）
和 `REGISTRATION_CANDIDATE`（各候选的粗/精修位置与姿态）。页面失败原因包含
各候选的拒绝详情；统一定位状态也显示本次匹配错误，不继续沿用旧的 LIO 错误提示。
旧地图/旧请求的失败消息不能覆盖当前请求。

已部署过完整三维定位接口的主机，同步代码后停止旧后台，更新两个包：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select icp_registration car_vel
source /home/a/3dnav/install/setup.bash
```

按原命令重启后，先保持静止，核对参数：

```bash
ros2 param get /icp_registration initial_gicp_maximum_iterations
```

默认应为 40；本次星玑三维初始请求日志应显示 GICP `max_iterations=40`、
`optimizer_iterations=12`。重新确认初始点后检查 `/localization/status`，
未就绪时不要反复发送目标或降低门限。提高优化预算不能保证错误初始点、
错误地图或外参不一致的数据匹配成功；当前仅完成预算策略/诊断的离线验证，
完整 PCL 构建及原始场景匹配需要小电脑实测。

若需要再录一次，加入原始请求和状态（只记录，不会启动运动）：

```bash
ros2 bag record -o "$HOME/initial_pose_$(date +%Y%m%d_%H%M%S)" \
  /relocalization/request /relocalization/status /localization/status \
  /current_map_info /robot_body/status /navigation/mode \
  /tf /tf_static /Odometry /cloud_registered /lio_health /rosout
```

记录期间只执行加载地图和确认初始点，结束后 Ctrl+C 停止录制。保存完整后台日志及
同名 PCD/定位元数据。没有录原始请求时，不能用猜测的初始坐标冒充原场景回放。

### 成功初始化后，弱扫描被回退却永久停车

2026-10-28 两份运行记录均复现同一冲突：PointLIO 发布
`odometry_sample_quarantined`，说明 `scan_matching_quality` 扫描已回退、
`recovery_only=true`、`hard_stop=false`、没有独立平面/转向异常证据；
约 0.5～0.6 秒连续健康扫描复核后恢复。原统一定位节点却在 warning 时立即
清空全局锚点，继而触发导航取消和零速锁定。第二份记录在已经行走、连续
前往几个目标后才出现，所以不是初始点未生效或 SDK 不能行走。

现将“定位锚点是否连续”与“当前是否允许运动”分开：

- 只有上述明确回退的质量告警，才允许保留已验证锚点进入有界连续性窗口；星玑三维已于2026-10-30协调为最多3秒，见本文末节。
  后续 `lio_auto_recovery` 必须来自该窗口；重复告警和恢复心跳不会延长窗口。
- 这段时间 `/lio/motion_paused` 及导航恢复暂停仍独立阻断速度；健康后只接纳
  新控制器指令继续原任务，不重发旧速度、不解除真实急停。
- 新鲜 TF、独立里程计连续性、地图/本体身份和单一 TF 发布权检查保持原值。
  匹配期限已更新为下节的 3+2 秒策略。超窗、真正跳变、重置、故障或未知恢复仍撤销信任并停车。
- `/localization/status` 的 `lio_quality_pause` 表示上述已识别的短暂质量窗口。
  `ready=true` 是定位输出可信，不代表绕过 `/lio/motion_paused` 的运动许可。
- 整条路线预览仍保持 0.15 m / 15° 的起点变化限制；失败提示说明地图/地形/
  本体/配置变化项，或具体起点平移和转角（也包括定位校正），不能直接放行旧路线。

更新代码后，停止旧后台，再在小电脑执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select car_vel lzp_service
source /home/a/3dnav/install/setup.bash
```

按原方式重启后台，刷新页面，重新加载地图并确认初始点。先在平地做短路线检查。
若再次停车，在另一个已加载 ROS 环境的终端**逐条执行**以下只读命令：

```bash
timeout 5s ros2 topic echo /localization/status --once --full-length
```

```bash
timeout 5s ros2 topic echo /lio_health --once --full-length
```

```bash
timeout 5s ros2 topic echo /lio/motion_paused --once
```

健康复核结束后应保持同一初始请求和有效 `accepted_stamp`，恢复期 `motion_paused`
应先为 true、健康后为 false；真实失效则继续保持停车。上面是静态检查，不需要
为采集而主动制造弱扫描，也不要带着活动任务反复复位。

已用生产回调的离线测试复现原锚点被清空，并验证两份记录的健康恢复时序、
恢复期间输出零速、健康后新指令通行，以及持续 TF 不更新、里程计跳变、匹配过期、
超窗、重置和故障继续停车。离线测试不能替代小电脑 ROS 联调和现场运动验收。

### 20:43:30 高度校验与匹配过期的稳定性修复

这次 LIO 保持 healthy。20:43:29 的全三维匹配高度修正为 0.357470 / 0.397020 m，
超过旧 0.35 m 上限。旧策略在最后有效样本年龄达到 2.015565 s 时永久锁停；
约 0.126 s 后下一轮匹配已通过，但无法恢复原任务。

本次只调整星玑三维连续跟踪，初始定位和原有模式保持原阈值：

| 检查 | 初始/原有模式 | 星玑三维跟踪 |
| --- | --- | --- |
| 最终 RMSE 上限 | 0.20 m | 0.22 m |
| 相对粗位姿的高度修正上限 | 0.35 m | 0.45 m |
| 重叠率下限 | 55% | 55% |
| 单次平面修正/朝向上限 | 原路径不变 | 0.20 m / 6°，不变 |
| 新结果最大年龄 | 原路径不变 | 1 s，不变 |

0.45 m 是定位配准的高度修正容差，**不是**机器狗可跨越台阶高度。机械能力、
地形危险、碰撞及人工急停检查没有因此取消。全三维候选和最终平面变换仍分别
经过质量校验；没有把未经验证的高度写入 Nav2 TF，也没有宣称竖向漂移已消失。
若持续超过新限值，应继续核对地图基准、外参和 LIO 高度一致性。

正常锚点有效期从 2 s 增至 3 s；之后最多再有 2 s 的**零速复核**：

- `/localization/status` 输出 `ready=false, recovering=true, reason=matching_recovery`，
  并提供 `recovery_remaining_sec`、`recovery_matches` 与当前要求 `recovery_matches_required=1`。
- 新鲜且连续的 LIO 仍可维持 TF/相对位置供 Nav2 使用，速度仲裁器独立阻断运动；
  不把相对位置继续更新当作全局匹配已恢复。
- 一次新的、通过全部跟踪检查的匹配即可恢复（后续已按用户要求由两次调整）；重复/倒序、失败结果不计为通过。
  窗口固定，不随心跳或第一条成功结果延期。真正跳变、重置、故障、TF 发布权冲突立即终止；短暂数据延迟见下节。
- 多点路线和正在执行的巡检保留当前目标。若正处于巡检目标交接，后端有界等待并
  重新核验完整授权；暂停、取消、地图/本体变化仍能阻止续跑。新手动目标仍需重新预览。
- 复核通过后丢弃复核前的速度缓存，只接受新的控制器速度；不自动解除人工暂停或急停。
  窗口耗尽后仍取消任务并锁定零速，晚到结果不能重启任务。

同步源码并停止旧后台，在小电脑更新三个包（本次没有改 Action 接口）：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select icp_registration lzp_service car_vel
source /home/a/3dnav/install/setup.bash
```

按原方式重启后台，浏览器强制刷新。以下命令逐条执行：

```bash
ros2 param get /icp_registration tracking_maximum_vertical_correction
```

应为 0.45。初始定位参数 `maximum_vertical_correction` 仍为 0.35。

```bash
ros2 param get /icp_registration tracking_maximum_rmse
```

应为 0.22，与统一定位节点的 `tracking_maximum_rmse` 默认值一致。

```bash
ros2 param get /global_pose_authority matching_coast_sec
```

应为 3.0，`matching_recovery_sec` 为 2.0。它们以启动配置为准；运行时写入参数
不会重建策略。需要调整时应修改启动参数/配置并重启。节点拒绝无限期限或无效数值，复核最长 2 s。

重新加载地图、确认初始点和朝向后，先验证短平地路线，再逐步验证多点及巡检。
正常时 `ready=true`；短暂复核时保持站立零速、任务未取消，连续匹配恢复后继续；
真实故障或超过总窗口仍应停车。保留完整后台日志；如再录 bag，增加
`/localization/status /cmd_vel_nav /cmd_vel /cmd_vel_safe /navigation_active`
及原有 `/lio_health /Odometry /tf /tf_static /cloud_registered /rosout`。

离线回归覆盖了原 2.015565 s 锁停、单次完整恢复匹配、速度缓存清理、路线/巡检保留与
取消屏障；可独立编译的 C++ 测试验证记录中的高度候选和阈值作用域。Windows
无法运行完整 ROS/PCL 配准，本次未完成原 bag 的整条 PCL 重算或实机验收，
不能据此保证所有环境不再停车或已消除地图/高度漂移。

### 2026-10-29：清理短暂数据延迟引起的重复锁停

10:41 的记录中，初始定位和整条巡检路线预览均通过；开始时三维匹配年龄
2.31 s，尚未到 3 s，但 `sensor_fresh=false` 导致启动被拒。随后旧代码把
大于 0.5 s 的采样间隔直接当成里程计不连续，清空锚点。参与配准的点云
时间戳可见约 0.503 / 0.799 s 间隔；日志缺少完整 TF 位姿，不能据此断言
现场一定没有物理跳变。新增回归使用相同间隔和受控小位移复现处理缺陷。

本次清除/替换的重复处理：

| 原处理 | 当前处理 |
| --- | --- |
| 一次 TF 数据超过 0.3 s 就取消任务/锁定 | 立即零速，进入统一数据流复核，暂保留任务 |
| 相邻样本超过 0.5 s 就无条件清空锚点 | 对最长 1.5 s 的间隔核验实际位移与朝向，再复核恢复 |
| TF 查找暂时失败落入通用异常并立即锁定 | 与旧 TF/采样间歇使用同一个有界等待状态 |
| 定位状态心跳一次超过 0.3 s 就取消路线 | 运动仍立即阻断；同身份的最后可信状态可保留任务至最长 1.5 s |
| 启动恰逢短暂延迟，巡检立即失败 | 当前显式开始请求/有效巡检授权参考剩余复核时间，累计等待最多6.1s，再完整复核 |

星玑三维源TF新鲜度已更新为0.45s，定位状态心跳仍为0.3s；真实测量时间戳保持。缺失/过期 TF
不会被重复广播来伪装恢复。普通相邻样本的运动连续性限制保留；跨间隔重新出现
的数据还必须满足位移不超过 0.30 m、朝向变化不超过 20°，不能按缺失时间无限
放大运动容差。大位移、反向时间、非有限数值、实际 LIO 重置/故障、超过 1.5 s
的持续掉流仍撤销锚点并停车。

数据流恢复使用固定最长 3 s 的窗口，若原匹配复核窗口更早到期，以更早期限为准：

1. 必须看到 **3 个不同时间戳且新鲜的 TF 样本**；重复 TF 不累计。
2. 随后需要 **2 次新的合格三维匹配**，采样时间必须覆盖数据恢复阶段。
   恢复开始前采集的晚到结果不能凑数；心跳、重复结果和新一轮间歇不会延长窗口。
3. 整段过程保持零速；恢复后清理旧速度缓存，只接受新控制器指令。
   暂停、停止、预览替换、地图/本体变化和巡检授权撤销优先于等待结果。
4. 窗口耗尽或证据不成立就停止任务，不用无限重试掩盖故障。

连续匹配请求预算从 1.5 s 调整为 **2.0 s**，启动参数为
`/global_pose_authority.tracking_timeout_sec`；不会增加并行配准工作线程。
上一节的 RMSE 0.22 m、高度修正 0.45 m、平面创新量 0.20 m / 6°、重叠率、
候选歧义和实际输出变换校验均不变。人工急停、碰撞/地形危险、本体身份和单一
TF 发布权校验保留；星玑二维和钢镚输出流程不变。

诊断增加 `sensor_recovering`、`sensor_stream_ready` 和 `lio_timing`：
`source_age_sec` / `receipt_age_sec`、`step_dt_sec`、`step_xy_m`、`step_yaw_deg`、
`fresh_samples` 及查找错误。故障日志会带明确原因和数值；短暂复核不再提示
“必须重新确认初始点”。`reason=sensor_recovery` 表示数据流复核，
`reason=matching_recovery` 表示匹配复核，二者都不允许运动。

更新源码、停止旧后台后，重建并重新加载环境：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select icp_registration lzp_service car_vel
source /home/a/3dnav/install/setup.bash
```

本次主要修改 `car_vel` 和 `lzp_service`，同时列出 `icp_registration` 以保持上一版
跟踪阈值一致；没有修改 Action 接口。按原方式重启，浏览器强制刷新，先在静止状态
重新加载地图、确认初始点和预览。逐条检查：

```bash
ros2 param get /global_pose_authority tracking_timeout_sec
```

应为 2.0。读取完整状态：

```bash
timeout 5s ros2 topic echo /localization/status --once --full-length
```

正常时应为 `ready=true`；延迟时为 `ready=false, recovering=true`，任务保留但
机器人零速；恢复条件满足后才重新就绪。先验证短平地路线及巡检，再逐步增加负载。
不要为复现主动拔雷达或制造位姿重置。若仍失败，保留新数值日志及
`/tf /tf_static /Odometry /lio_health /localization/status /rosout` 录包，以区分延迟和真跳变。

离线测试覆盖了两类原误停、查找异常、时钟冻结、真实跳变/掉流、连续证据恢复、
显式启动等待、人工取消和巡检授权。当前 Windows 环境未执行原始场景的完整
ROS/PCL 重算或实机运动，现场稳定性仍需小电脑验证，不能承诺任何场地永不停车。

### 2026-10-29 后续修复：分离身份、候选异常与启动路径证据

11:19–11:20 的日志暴露了旧实现的三个状态问题：2 s 心跳失效被误作切换到
legacy；0.3–0.4 s 的抖动反复移动恢复起点；一次创新量越界永久置故障标记，
后续匹配却又报告 accepted。最后一次路线还被泛化的 preview_stale 拒绝。

当前统一处理如下：

| 情况 | 当前行为 |
| --- | --- |
| 星玑/三维模式身份消息暂时迟到 | 保留归属和锚点，零速、固定最长 3 s 复核；真实身份/模式代际改变才清理旧证据 |
| 同一窗口中出现短暂数据抖动 | 保留已验证的新鲜样本和新匹配；旧 TF 仍不许可运动，窗口不延长 |
| 身份/数据流等待进入匹配复核 | 由 authority 的同一固定截止时间控制，下游不再额外套一个 2 s 期限 |
| 匹配候选超过 0.20 m / 6°，但不超过 0.40 m / 12° | 拒绝该候选，锚点不变，最多 2 s 零速复核；一次新的完整合格匹配后恢复 |
| 超过 0.40 m / 12°、非有限数值或明确候选歧义 | 定位故障不可自动恢复，零速停止；晚到匹配不能改写故障原因 |
| 地图/本体/LIO 身份变化或超过连续性/恢复时限 | 旧锚点/旧任务授权失效，停止任务，需重新确认 |

这不是把 40 cm 当成可接受修正：真正允许写入锚点的范围仍为 **20 cm / 6°**。
局部匹配请求现在直接使用同一范围，服务端会先筛掉越界候选再排序。
星玑三维源TF新鲜度现为0.45s，状态心跳仍为0.3s；1.5s数据连续性上限和TF发布权核验保留。
恢复期间可以更新真实定位供规划，但速度输出保持为零；恢复后必须收到新的速度指令。
`hard_failure` 只表示定位不可自动恢复状态，不是调用厂商硬急停。

单点、多点与巡检使用同一个启动证据检查：稳定地图/地形/本体/配置身份不变、
起点朝向变化不超过 15°，从**当前可信位置**到预览路径起点的连接不超过原有
路径校验边界 0.35 m。连接段也逐格检查，和路线一样使用 0.45 m 占地半径。
不再另用 0.15 m 预览快照偏移量直接否定这类已重新验证的连接；真正地图版本
变化或连接不安全仍拒绝。新预览计算期间的大幅变化和下发前票据检查仍保留。
无变化的重复状态消息不使预览失效，真实地形地图版本改变会撤销预览和巡检授权。

诊断中的 `lio_timing.failure_reason` 保留 source_gap_exceeded、source_timeout 等
实际原因。路线拒绝分别显示地图加载批次、地形地图版本、本体/配置变化、
朝向差值或不安全路径段，不再统一报“地图、地形、本体或位姿变化”。

本轮仅修改 car_vel / lzp_service，无 Action 接口、星玑二维、钢镚或本体步态配置改动。
停止旧后台、同步源码后：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select car_vel lzp_service
source /home/a/3dnav/install/setup.bash
```

按原方式重启，重新加载地图并设置初始点，人工预览与开始。先测短平地单点、
再测地形导航多点和巡检。无需为了复现拔雷达。只读状态检查（逐条执行）：

```bash
timeout 5s ros2 topic echo /localization/status --once --full-length
```

```bash
timeout 5s ros2 topic echo /navigation/velocity_guard --once --full-length
```

正常应 ready=true；暂时复核应 ready=false/recovering=true、速度为零且原任务保留；
到期失败必须停止。人工停止/暂停/急停或撤销巡检授权后，晚到恢复不能重新启动任务。
Windows 离线测试不包含完整 ROS/PCL 重算或实机运动，这些仍需小电脑验证。

### 2026-10-29 12:38 后续：保留临时隔离目标，取消人的预览倒计时

本节更新前面“所有数据恢复都需要两次全局匹配”的旧规则：只有真实匹配失效、
实际大采样间隔或独立可疑校正需要这项复核。仅短暂TF变旧、没有连续性异常、
原匹配仍在有效期时，3个不同时间戳的新鲜TF即可恢复；底层LIO暂停仍独立阻断速度。
后续单次匹配调整进一步将需要地图匹配时的次数改为1次完整可信匹配，以上TF样本要求不变。
复核固定截止时间、1.5秒实际掉流边界和人工停止优先级保留；源TF新鲜度于2026-10-30更新为0.45秒。

现场12:38:03出现 position_step 1/8：扫描间隔0.499秒、位姿线速度0.344m/s、
估计器0.256m/s，原LIO临时隔离并很快恢复；authority过去却将非scan_matching_quality
警告全部撤销锚点，12:38:03.602取消路线。12:38:04.063原匹配已验证通过，
但身份被提前重置导致结果无法接回。本次移除这层重复的立即清锚行为。

仅当 position_step/yaw_step 警告明确声明 sample_quarantined=true、temporary_stop=true、
hard_stop=false，隔离计数未达故障数、6项速度诊断均有限且在保守范围内，才允许
保留先前已确认锚点最多3秒（2026-10-30协调更新）。线速度/估计线速度≤0.6m/s、角速度/估计角速度≤0.8rad/s、
垂直速度/估计垂直速度≤0.3m/s是这项宽限的诊断条件，不是提高机器人运动速度上限。
仍核验后续真实TF位姿、地图匹配、原LIO健康恢复；不允许在暂停期间运动。
缺少诊断、高速或大跳变、持续失联、估计器重置/故障仍停车；不会自动解除人工急停。

巡检与地形多点使用同一 route_navigation_manager -> Nav2 -> velocity_arbiter -> 本体SDK。
巡检外层保留检测点停留、识别、告警和报告，导航判断不另设一套阈值。
两页PointCloudViewer均采用 markerScale=0.25，实际位置和选点箭头一致。

人的操作不再受18秒/20秒预览有效期和120秒待启动租约限制；同地图加载批次、
同地形/本体/配置和路线的预览可等待后再人工开始。真实证据变化、新预览、取消或任务开始会撤销旧凭证。
启动使用当前可信位置复核原路径连接（仍≤0.35m并检查0.45m占地），需要新规划时验证完整路径。
相同静态地图上的预览不再重复全路线栅格遍历：3233点回归中由972111次读取降为302次连接检查。

巡检预约请求仍最多等待12秒，但已拆成“锁内准备、锁外等待、锁内复核并提交”。
冻结路线、检测点模板、本体和导航模式；期间可取消启动，迟到授权仅发送撤销，不创建任务。
后台 INSPECTION_START 记录阶段和耗时；页面显示中文原因，不把RPC无响应误说成预览过期。

同步源码，停止旧后台后重建 car_vel 与 lzp_service（当时版本的命令）；若更新到下面的刚体三维版本，必须重建四个包。

## 2026-10-29 刚体三维模型（含后续局部ROI修复）

15:39的完整日志显示：某候选完整三维校验RMSE0.158m、重叠率90.6%通过；强制将变换的Z、roll、pitch清零后，
对同一三维点云重检RMSE变成0.223965m而被拒绝。随后垂直候选持续超过以零高度为基准的0.45m限制，5秒失配后停止。
本轮修复几何链路而不继续抬高质量门槛：

- 原生robust候选保留通过验证的完整SE(3)，结果位置/四元数与其质量指标一致。默认legacy结果仍按原逻辑输出平面位姿。
- 初始协调状态保留Z/四元数；统一定位按真实点云时间的完整LIO位姿计算`map→odom`。后续粗位姿继承此变换，不再写Z=0。
- 只有统一定位发布该完整变换，不增加第二个TF发布者。Nav2仍使用同一地图坐标中的XY/yaw控制平面速度。
  已核对[Humble代价地图实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp)：
  footprint与地图更新从TF位姿取XY及yaw，不要求将上游匹配变换人为清零。
- 相对预测的高度/姿态创新及原XY/朝向边界仍检查。真实楼梯高度由LIO传递，累计上升不当成一次校正量。
  高度分离但质量相近的候选也纳入歧义拒绝。
- 初始和跟踪Action结果必须声明`transform_model=se3_map_odom_v1`。旧节点的平面结果不能静默进入新模型；
  后台启动日志新增`REGISTRATION_CONTRACT`用于确认真正运行的二进制版本。
- 地形地图先验使用同一采集时刻的完整TF转换现场点云，避免只转XY而留下错位高度。
- 地形页星玑三维健康判断使用同身份的后台统一状态，不再用旧AMCL协方差重复否决；二维/钢镚保持旧健康流程。
- 星玑三维把小范围缺失回波与实测沟槽分开。较大缺失区仍是unknown，实测坑洞/障碍仍阻止运动；
  没有取消物理危险、定位新鲜度或手动急停要求。

### 更新与验收

保持机器人停止，结束任务及旧后台，将完整更新源码同步到小电脑。在新终端执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --packages-select lzp_interface icp_registration car_vel lzp_service
source /home/a/3dnav/install/setup.bash
python3 src/ros2_humble_main/src/car_vel/scripts/check_3d_navigation_install.py
```

自检只读：检查四个包前缀、生成Action字段、原生动态库和安装标记、Python几何与所属工作区。不启动节点、不操作设备。
未通过时先修复部署，不用反复设置初始点。

然后启动单套后台并留日志：

```bash
RCUTILS_COLORIZED_OUTPUT=0 ros2 launch car_vel launch_manager.launch.py 2>&1 | tee "$HOME/nav_rigid3d.log"
```

浏览器Ctrl+F5，重新加载地图、确认初始点并预览一次。现有地图/路线格式未变，保存路线可继续使用。
先在平地执行短多点路线，经过本次停止位置，再测同一路线巡检；不会自动恢复之前取消的任务。

只读状态检查：

```bash
timeout 8s ros2 topic echo /localization/status --once --full-length
```

星玑三维应有`transform_model: se3_map_odom_v1`，确认定位后`ready:true`、`sensor_fresh:true`。
`map→odom`或机器人地图位姿的高度/roll/pitch可非零，不要为了让界面数值归零手工添加静态TF。
若停止，`LOCALIZATION_FAILURE`和`status.failure`保存首个原因及原生候选细节。

离线测试覆盖了刚体往返、初始结果到下一次请求、连续15cm相对台阶、唯一TF一致性、
共享路线/最终速度恢复、旧版本拒绝、地图先验高度转换、缺失回波与实测危险以及二维/钢镚隔离。
Windows上可编译ROS无关C++单元；完整Linux ROS/PCL构建、实际雷达/负载/机器狗效果仍需现场验收。
既有地图高度基准或雷达外参本身有误时，保留几何不能替代对这些数据的核验。
先验证短平地路线和同一路线巡检。原始PCL配准和实际机器人运动仍需小电脑验收，
没有取消真实几何质量、硬件故障、人工急停或实时避障检查。

### 17:35 局部地图采样修复

本次`shiyanshi-wai`行进到第10/19点停止，首因是局部地图粗采样不足，继而`matching_recovery_timeout`。
17:33/17:34的LIO隔离均成功恢复，不能把最后失败归因于这些已经结束的恢复。
停车区域PCD原始点约14,589，15cm细采样约7,280，45cm粗采样只有1,163，而门槛为1,200。
原算法把粗网格的抽稀结果直接当作不可精修，即使细配准真实点仍充足。

robust三维请求现在在同ROI、同上限、同门槛内，最多三次缩小粗网格保留更多真实点。
不扩大搜索半径，不降低RMSE/重叠率/歧义要求，不用原始未采样云绕过检查。
最终仍稀疏的地图会明确报告原始/细/粗点数，并保持停车。二维/钢镚legacy请求保持原行为。

本页上方四包重建和安装检查仍适用；checker同时拒绝未包含本次重采样修复的旧SE3动态库。
启动应有`REGISTRATION_TARGET_POLICY model=adaptive_target_sampling_v1`；匹配准备应有
`REGISTRATION_TARGET ... raw=... fine=... rough=... initial_rough=... refinements=... ready=true`。

在小电脑编译后执行：

```bash
ctest --test-dir build/icp_registration -R '^target_sampling(_pcl)?$' --output-on-failure
```

Windows已覆盖同一生产采样选择函数及原始地图点数核对；真实PCL测试和完整ROS原生构建必须在Linux完成。
地图/路线无需重做；重新确认初始点、预览后验证原停止处及同路线巡检。

### 19:19 巡检启动：分离已恢复的LIO与地图匹配等待

本次9个新TF、source_age0.14s却被旧3s流恢复计时器判失败；约0.3s后的地图结果
RMSE0.172m/overlap87.4%正常。该计时器过去等到地图匹配确认才清理，误把地图等待算作LIO不连续。

现在3个真实新TF即完成流恢复，真实>.5s采样间隔仍触发独立地图匹配要求；地图复核最多3s且
不能超过此前已接受锚点的coast+recovery总期限（默认5s），不无限续期。缺流1.5s、时钟倒退、
实际跳变、定位质量和人工暂停/停止规则保持。等待匹配期间零速，恢复只接受新的控制器速度。

更新源码、停止旧后台后重建`car_vel lzp_service`。初始点已确认但尚未运动时的真实失效也保存首因，
启动失败后“没有活动路线”的停止清理只显示普通提示，不覆盖启动失败原因。
这次地图补收超时和识别预热同时出现，日志不足以证明具体网络或CPU原因；不要据此认定YOLO使LIO崩溃。

### 2026-10-30 控制器计算配置误判

14:22日志中控制器已确认batch_size400，但固定模式消息暂时迟到超过2s，被计算量配置节点清空成unknown。
随后新路线因“等待空闲同步配置”被拒绝；约0.16s后相同参数恢复ready。该次并非LIO或地图匹配终止。
固定模式改为持久配置事件，显式切换/无效消息仍更新；动态本体/活动状态的失效边界保留。
不绕过远端写入确认、回读、控制器代际和模式/本体匹配要求，不改变二维/钢镚的参数值。

本次只需同步并重建`car_vel`、重启后台。查看`/navigation/controller_timing`应有
`mode_context_source=latched_configuration`，且模式/本体匹配、ready=true；再重新预览并启动任务。

### 2026-10-30 15:20 运行时稳定性协调

源TF在0.305s附近反复进入复核，以及五帧健康恢复需1.484s而锚点只等1s，是本次不同的两处问题。
活动星玑三维源TF容许年龄现为0.45s；独立的定位状态心跳/速度消息0.3s期限不变。
明确可恢复的LIO健康暂停保留锚点最多3s，心跳不续期、期间零速；真实故障/重置仍直接作废锚点。
原生平面步长按采样间隔有界补偿：max(原门槛, min(配置速度,0.6m/s)×dt)，仅在dt≤0.5s生效。
补偿要求`/localization/interval_compensation`的新鲜状态，1s无消息、任务结束或切出星玑三维即恢复原门槛。
这个状态不授权运动，不改变独立的速度/转速/高度/质量检查。二维和钢镚不启用补偿。

先停止旧后台、同步源码，重建`small_point_lio car_vel`并重新加载环境；
安装检查增加`--runtime-timing`，防止Python已更新但原生LIO仍为旧版本。
完整命令、状态字段与短路线验收步骤见项目README第7.29节。地图和路线格式不变。
源数据间歇的计算负载/调度/驱动原因仍待运行证据定位；此次不承诺所有场地持续通行。
