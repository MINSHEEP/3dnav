# 导航不动：速度链路诊断日志

本次只增强诊断，不修改导航参数、地形保护、步态、停车方式或运动授权。不能补回旧日志中未记录的速度，也不等于已修复实际不动的原因。

## 部署和采集

先让机器狗停止，结束任务并关闭旧后台；不要同时启动两套程序。将本次 `car_vel` 包的修改同步到小电脑后，在终端执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select car_vel
```

构建成功后，使用项目原有启动入口并保存一份新的完整日志：

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
RCUTILS_COLORIZED_OUTPUT=0 ros2 launch car_vel launch_manager.launch.py 2>&1 | tee "/home/a/nav_debug_$(date +%Y%m%d_%H%M%S).log"
```

这里使用的 `RouteExecutionStatus` 消息已经存在，本次没有新增或修改接口字段。不需要因为本次日志增强重新建图。没有 Linux/ROS 和机器狗的离线环境只能验证诊断逻辑及输出不变，部署后仍需现场核对日志。

不要专门复现摔倒或解除停车锁来采集。下次正常使用若遇到不动，保持停止，把本次完整日志发回即可，不要只保留筛选后的几行。

## 三个诊断入口

| 标记 | 记录什么 | 能证明什么 |
| --- | --- | --- |
| 原有 `NAV_GOAL` | 目标发送、接受、失败等导航事件 | 目标是否进入导航链路 |
| 新增 `NAV_VELOCITY` | 控制器原始速度、仲裁后的最终速度、来源、路线点、数据龄期、地形原因 | 速度是否产生，以及在哪种保护下被清零或限幅 |
| 新增 `NAV_BODY` | 星玑收到的最终速度、站立/步态反馈、授权、发送结果 | 本体层是否拒绝；只证明观察到的发送，不证明机械狗实际执行 |

每条新日志是带中文 `reason` 的 JSON，`reason_code` 可用于筛选。`stamp_ns` 是 ROS 时间；`*_age_sec` 是基于本机单调时钟计算的龄期。`route.point_number` 从 1 开始，`route_age_sec` 表示路线上下文多久未更新，不能把旧路线上下文当成当前任务确认。

`NAV_VELOCITY.input/output` 均为 `[前后速度m/s, 左右速度m/s, 转向速度rad/s]`。`NAV_BODY.input` 同样是上述物理单位；`sent_sdk_velocity` 则是整形、缩放后的厂商归一化参数，不能直接与 m/s 等同。零输入时整形器仍可能短暂输出减速指令，应同时查看输入和实际发送值。

## 如何定位不动的层级

| `reason_code` | 解读 |
| --- | --- |
| `upstream_missing / upstream_stale / upstream_discarded` | 未收到控制器速度、速度超时，或状态切换已清除旧速度执行资格 |
| `upstream_zero` | 控制器当前输出零速；继续对照控制器、TF、扫描和代价地图日志，不单凭零速认定其中某一个根因 |
| `stop_latched / lio_paused_or_stale / navigation_paused_or_stale` | 停车锁、LIO 或导航恢复保护在阻止输出 |
| `terrain_unsafe / terrain_status_stale / ground_evidence_unanchored` | 地形未放行、保护数据过期或地面高度证据不合格；查看同条 `terrain`、`observer` 及其龄期 |
| `direction_not_authorized / speed_limit_zero / limited` | 方向不允许、零限速，或速度被限幅 |
| `other_source_owns_output` | 遥控或园区连接段占用速度输出，不能把最终输出归给 Nav2 |
| `motion_not_armed / body_status_stale / body_not_standing / gait_feedback_invalid` | 星玑本体未获授权，或本体/站立/越障反馈未满足条件 |
| `safe_command_missing / safe_command_stale / safe_command_discarded / received_zero` | 本体未收到有效最终速度、速度过期/执行资格已清除，或收到零速 |
| `udp_send_failed / udp_send_not_observed` | 最近发送明确返回失败，或没有近期发送记录 |
| `udp_dispatched_unconfirmed` | 已观察到 UDP 发送；协议没有逐条运动执行确认，不能据此宣布机器人已运动 |

原因有优先级；同一时间多项条件可能同时失败，应查看完整字段。`permitted_by_body_tick` 来自本次实际控制判断，而非额外计算授权；提前返回时为 `null`。地形 `observer.message` 可保留变换/解析错误，`near_field_return_count` 等只是所有高度回波覆盖统计，不是地面已安全确认。

原因或路线点变化时限频记录，最多每秒 2 条；导航活动期间状态不变每 2 秒汇总一次。`window_reason_counts` 是两次日志之间控制周期采样的原因计数，**不是独立故障次数，也不是完整速度轨迹**；`window_sample_count/window_duration_sec` 给出统计窗口。短暂原因可能只出现在计数中。稳定无活动时不连续刷日志。

速度仲裁日志适用于现有共用链路；详细 `NAV_BODY` 是星玑本体诊断，本次未改钢镚 SDK 或配置。`/robot_body/status` 中新增星玑授权、最近原因及其时间戳仅供观测，不作为新控制条件。

原来累计第 5 次正常 PING 也会出现的 “No PONG” 提示已修正为仅握手未就绪时提示，并明确计数是启动以来累计数，而非连续失败次数。PING 发送行为本身没有改变。

## 星玑 3D 持续输出但无移动进展（2026-10-27）

`/navigation/velocity_guard` 除原 `blocked/reason` 外还包含 `warning/warning_code/warning_reason`
与 `motion_progress`。`blocked` 仍只表示原速度保护原因；`warning` 是只读运动诊断，不新增任何控制门禁。
地形导航与巡检页面均显示诊断，操作记录只在提示类型改变或解除时写入，持续秒数变化不会刷屏。
保护停车原因优先显示，诊断数据停止更新时显示已过期，不继续声称当前不动。

诊断只在星玑 3D 活动导航且仲裁器实际输出非零指令时计算，使用 `/Odometry` 位姿和推进的源时间戳，
而不是用 AMCL 静止时的消息频率判断运动。持续约 5 秒净位移不足 3 cm 且净转角不足 0.05 rad 才报告无进展；
纯转向已有角度进展不会被当作前进失败。里程计无数据/过期/重复时间戳时单独报告无法判断。
停速、保护接管、任务/本体/导航模式改变或里程计时间/坐标系重置后，旧计时不能沿用。

后端 `NAV_PROGRESS` 日志与同话题中的 `motion_progress` 用于对齐持续输出时长、平移/转角及里程计龄期。
“已输出速度”仅指速度仲裁器的实际发布；仍需结合 `NAV_BODY` 核对末端是否有权发送及发送结果，
不能据此认定本体已经收到或执行 UDP。诊断不主动取消任务、增速、站立、复位或急停；原有 Nav2 无进展处理仍有效。

## 速度快照与断流层级（2026-10-28）

`/navigation/velocity_guard` 的 `schema_version=2` 增加只读速度快照：

- `request_velocity`：新鲜的平滑后 Nav2 输入 `[vx,vy,wz]`，过期/执行资格清除后为 null。
- `commanded_velocity`：本次仲裁器已经发布的速度指令，不是本体实测速度。
- `speed_limit_mps`：当前有效地形状态给出的导航允许上限。数据/本体身份无效时为 null；不能把上限当成实际指令。
- `speed_policy / terrain_speed_limit_enabled / speed_limit_source / terrain_type`：区分跟随导航速度、受限地图证据限制和危险停车。
- `controller_stream`：只读 `/cmd_vel_nav` 原始消息接收龄期、近期接收频率、最长间隔及有效时的原始指令。
- `stream_reason_code`：`controller_stream_stale` 表示原始消息中断；`smoother_output_stale` 表示原始消息仍更新但平滑后输入过期。

`reason_code=upstream_stale`、明确地形危险等此前可能只在日志里体现的零速原因，现在也进入网页停车提示。
`upstream_zero` 本身不说明 TF 或障碍根因。原始指令采集没有发布权限，不能延长 `/cmd_vel` 的有效期。
UI 状态超过两秒未更新会清空速度数字，不继续展示旧值为当前速度。

只读查看（单条执行）：

```bash
timeout 5s ros2 topic echo /navigation/velocity_guard --once --full-length
```

导航和遥控控制值不是同一单位。同步记录指令和里程计、核验实际速度的步骤见 `XINGJI_FIXED_OBSTACLE.md`；
没有实测证据时，不把 0.15 不能起步当作已知厂商死区，也不把指令强制提高到 0.5。
