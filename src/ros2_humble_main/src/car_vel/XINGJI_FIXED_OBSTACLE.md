# 星玑 3D 固定越障执行与雷达安装核对

## 改动边界

仅对已确认身份的星玑、`terrain_3d` 生效。配置参数
`/automatic_terrain_navigation_node.xingji_3d_fixed_obstacle_enabled` 默认为 true。
首次启动执行任务仍需新观测及本体越障模式成功确认；成功后可通行地形变化不再使执行阶段在
normal/obstacle/ramp/rough_ground 之间来回切换。

2026-10-28 按用户要求，星玑 3D 取消可通行地形类别带来的额外速度限制：平地、可通行台阶、坡道、粗糙地面
统一沿用原 `normal_speed_mps` 导航上限，默认 0.25 m/s。删除原先 .10/.15/.25 的地形档位切换和五次提速确认，
不新增地形驱动的提速或降速过渡。实际指令仍由 Nav2 决定；转弯、到点、避障可能主动变速，系统不会强行补足速度。
本体上限和原有速度单位映射继续生效。

此调整只取消额外地形调速，不会把不可通行地形变成可通行。未知、明确危险、过期、几何超限和步态未确认仍零速停车。
受限地图先验与未通过高度锚定的弱地面证据保留独立不超过 0.10 m/s 的证据限制；真实当前/近期高度锚定证据有效时，
不会因估计高差跨越 5 cm 分类边界而反复切换速度。

巡检任务同样通过共用 `StartRoute` 链路执行：开始巡检、检测点停留后继续、返回起点及下一轮巡检
沿用任务冻结的 3D 导航模式，不附加巡检专用的地形调速或提速确认。检测点采集、手动暂停和危险保护
仍按原任务逻辑零速停车；继续行进后恢复同一导航速度上限，不切换高速步态。

关闭某类自动地形仍禁止通过该类地形；关闭自动地形步态仍仅允许原普通地面范围，不能解锁复杂地形。
空闲不为了固定策略发站立/模式请求。本体身份失效、代次变化需要重新确认，旧确认结果不能跨本体/2D策略使用。
园区连接器保持原来的独占控制，原本体桥接仍约束星玑自主执行使用越障模式。连接器释放不强制恢复 normal 阶段。

星玑 2D、钢镚、手动遥控、SDK、地图和雷达外参沿用原配置。Nav2 计算量优化仅针对星玑 3D，
离开该配置时恢复原参数，详见下方速度链诊断和运行配置说明。未实机测量前，不宣称解决所有原地踏步或计算卡顿。

状态新增 `execution_policy` 和 `observed_terrain_type`：前者应为 `fixed_obstacle`，后者保留真实地形。
`current_mode=obstacle` 表示执行阶段已经服务确认，不表示所有道路被重新分类为障碍地形。

## 部署

机器人安全停止后结束后台，先同步修改后的完整源码，再逐行执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select car_vel lzp_service robot_navigation2
source /home/a/3dnav/install/setup.bash
```

使用原来的启动方式只启动一套后台，不同时手工再启动桥接/SDK。
新终端先加载上述两份环境，再检查：

```bash
ros2 param get /automatic_terrain_navigation_node xingji_3d_fixed_obstacle_enabled
timeout 5s ros2 topic echo /automatic_terrain_navigation/status --once --full-length
```

无任务时 idle 正常，不能为了看到 ready 而复位或启动运动。任务实际执行且地形通过检查、首次确认完成后，
应显示 fixed_obstacle / current_mode obstacle / requested_mode 空。
真实高度锚定地面可通行、步态确认后，默认 speed_limit_mps 为 0.25，包括可通行台阶和坡道，且
`speed_policy=nav2_body_limits`、`terrain_speed_limit_enabled=false`。
仅使用受限地图证据时仍不高于 0.10。
本机离线验证不能替代实机安全验收；有 unsafe/TF/定位异常应先停止排查，不关闭保护。

## 导航低速与遥控速度的区别（2026-10-27）

遥控页面滑块 0.5 直接对应厂商归一化前进指令 0.5，不能视为已标定的 0.5 m/s。
导航最终线速度继续沿用原换算系数 1.5：0.10 m/s 对应 0.15，0.25 m/s 对应 0.375。
此次没有设定未经实测的最小前进指令，没有强制把小指令抬到 0.5，也没有改星玑 2D 或钢镚的速度映射。
因此，解除平地额外限速不等于已经验证本体的最低起步值或真实速度；仍需现场验收。

地形导航（单点、多点）与巡检页会展示只读运动诊断：连续输出非零导航速度但新鲜里程计未见明显移动时提示；
无里程计、重复时间戳或数据过期时提示无法判断，而不是断言机器人静止。提示不触发自动增速、复位或硬急停。
详细字段和收集方式见 `NAVIGATION_MOTION_LOGS.md`。

## 速度指令链与运行计算配置（2026-10-28）

实际启动链使用系统 Nav2 Humble bringup：`/cmd_vel_nav` 是原始导航指令，`/cmd_vel` 是平滑后的指令，
`/cmd_vel_safe` 是仲裁器实际发布给本体层的指令。新增只读原始输入观测，不会把收到原始指令当作最终速度授权，
也不会通过状态心跳刷新已经过期的运动指令。现有 0.3 秒指令有效期保留。

网页区分“Nav2 请求”“最终下发指令”和“导航上限”，这些都是指令/约束，不能当成机身实测速度。
原始指令中断与平滑后指令中断分别提示；状态过期时清空旧速度数字。

仅星玑 3D 的 MPPI 候选轨迹数由原运行基线（项目默认 800）降为 400，减少每轮计算量；
控制频率 5 Hz、25 个预测步、原预测时域、速度上限、碰撞检查和 0.3 秒指令有效期不变。
这不保证控制器在小电脑上已经达到所需周期，需结合实际断流诊断验证。

`nav2_timing_profile` 只在导航空闲、本体身份与导航模式有效时修改参数，并回读确认。
切回星玑 2D 或钢镚时恢复修改前的实际基线；共享 `nav2_params.yaml` 不因这项优化而改变。
原值和未决写入记录在 `~/.ros/robot_navigation/nav2_timing_profile.json`，请勿为解除提示手工删除。
新任务在参数未确认时会明确拒绝启动；已运行的任务不被这项计算优化检查中断。
远端写入超时不能用本地取消来证明没有生效：系统保留未决状态，等待原请求确认，
或确认控制器进程已重启后恢复原配置。出现“上次未决参数写入”提示时，先停止任务并重启整套导航后台，
仅重启这个辅助节点不能清除不确定性。

若提示记录损坏或配置不匹配，先停止整个导航后台，保留记录备份后核对原配置和当前配置；
运行中不要删除记录。关闭优化开关也不能绕过未决写入的确认。

启动参数 `nav2_timing_profile_enabled:=false` 可关闭计算量优化，仍需等待原基线恢复确认；
`nav2_mppi_optimized_batch_size` 默认 400。此开关不取消地形危险、数据过期或本体保护。

按原方式只启动一套后台，强制刷新浏览器。只读核查命令逐条执行：

```bash
timeout 5s ros2 topic echo /automatic_terrain_navigation/status --once --full-length
```

```bash
timeout 5s ros2 topic echo /navigation/velocity_guard --once --full-length
```

```bash
timeout 5s ros2 topic echo /navigation/controller_timing --once --full-length
```

出现明确地形危险、无有效地面证据或定位故障时仍需先停车处理。本地回放/配置测试只验证软件行为；
对实际控制周期、路线通过率和本体动作的改善需要小电脑运行后复核。

### 只读采集：为实际速度标定留下依据

在小电脑的新终端，先加载 ROS 和项目环境，再执行下面的采集命令。它只记录数据，不会控制机器狗；
45 秒结束或按 Ctrl+C 只停止采集，不会替你停止遥控。遥控的移动/停止仍由页面按键控制。

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
```

```bash
timeout --signal=INT 45s ros2 bag record -o "$HOME/xingji-motion-check-$(date +%Y%m%d-%H%M%S)" /Odometry /dog_remote/cmd_vel /cmd_vel_safe /dog_status /robot_body/status /navigation/mode /navigation_active /automatic_terrain_navigation/status /navigation/velocity_guard
```

需要标定时，在已停止导航/巡检的可控平地使用现有遥控页面、保持越障模式，记录使用的滑块数值、
是否起步以及实际直线移动距离/用时，每次松开按键停止后再调整。先从较小值观察，不为了取数持续顶住障碍或关闭停障。
仅里程计有漂移不算实际起步；配合现场距离/视频核验，才可拟合“厂商指令—实际速度”并确定最低有效范围。
将生成的文件夹连同数值记录保留。未完成标定前，不调整共享系数或增加强制起步下限。

## 雷达静止核对步骤

以下只是安装几何和数据一致性初查，不等于完整外参标定认证。命令只读，不修改参数或发送运动命令。
选择已知平整地面，机器人正常站立且无导航/巡检任务，确认稳定和安全后测量；若当前趴着或不稳定，先不测站立高度。

### 1. 实测三个不同量

- 雷达坐标原点/厂家规定测量基准到地面的垂直距离：本项目暂按 0.52 m；不是量支架顶部。
- 雷达相对模型 base_link 原点的位置：配置为前方 0.23 m、左侧 0 m、上方 0.38 m；base_link 原点不是随意选的机身中心。
- 雷达相对 base_link 的安装旋转：roll=0、pitch=0.523599 rad（30°）、yaw=0。需要核对方向和符号，而不只是角度绝对值。

不要把 0.38 m 直接改成离地高度，或给现有 LIO 的雷达—IMU外参再叠加这组机身外参。
如果不知道物理坐标基准，先保留照片/测量值核对，不凭外壳尺寸改配置。

### 2. 查实际运行的固定外参

```bash
source /opt/ros/humble/setup.bash
source /home/a/3dnav/install/setup.bash
timeout 8s ros2 run tf2_ros tf2_echo base_link livox_frame
```

预期 Translation 约 [0.230, 0, 0.380]，RPY 角度约 [0, 30, 0]。
这只证明软件加载值与源码一致；仍需和第1步的实际安装相符。持续 frame 不存在不能认为标定通过。

### 3. 查实际使用的地面高度基准

```bash
ros2 param get /terrain_observer_node nominal_sensor_height_m
```

当前预期 Double value: 0.52。与正常站立实测雷达离地高度比较。
如果明显不同，记录实测值，不先靠增大地形容差来掩盖。正常姿态和机身高度变化也要记录。

### 4. 检查姿态和实时点云

```bash
timeout 8s ros2 run tf2_ros tf2_echo odom base_link
timeout 8s ros2 run tf2_ros tf2_echo odom livox_frame
timeout 5s ros2 topic echo /cloud_registered --once --field header
timeout 5s ros2 topic echo /terrain_observation/status --once --full-length
```

正常站立时 odom→base_link 的 roll/pitch 应和实测机身相对于水平面的倾斜一致；不是要求雷达姿态角为0。
cloud_registered 应为 odom 系、时间持续推进；若节点报采样时刻无匹配TF/里程计，先修数据链，不改几何标定。

可在 RViz 中以 odom 为 Fixed Frame，只显示实时 /cloud_registered（选择 Best Effort QoS 以匹配传感数据），
观察已知平地、前方墙面与机身坐标轴是否对应；不使用初始点/目标点工具，不执行导航。
地板在 odom 中不必是 z=0；项目 LIO 的高度原点不保证是地面。
同一采样时刻应满足近似关系 `真实地板z = 雷达原点z - 实测离地高度`。
最终精确检查需同时保存点云、TF和IMU按采集时刻分析，不能用陈旧点云配最新TF算误差。

静止平地不足以完整验证所有轴向平移和偏航外参；完整雷达/IMU标定还需要独立参考和受控运动采样。

### 5. 提交核对材料

发回第2～4步输出、站立离地高度、可辨认安装朝向的侧面/正面照片及RViz实时地面截图。
地图中初始点错误、GICP垂直修正异常，不等同于雷达固定外参必然错误；需分开检查。

## 多点路线与巡检的静止定位刷新（2026-10-23）

AMCL 在静止时可能不持续发布新位姿；编辑较多路线点后，旧采样会超过现有3秒有效期。
日志中的21点路线曾在约50秒和重新定位后约5秒被拒绝。单点已有刷新，但共用多点预览入口此前没有。

现将无运动刷新接入地形导航与巡检共用的3D路线入口：

- 逐段规划前与规划完成后均检查定位，必要时限时请求 AMCL 无运动更新。只有真正推进的源时间戳、有效采样和新鲜接收时间才能通过；不伪造时间、不放宽3秒有效期。
- 巡检开始时的预览预约、相机预热后的正式路线启动也检查；已由用户明确启动的当前巡检任务可在授权仍有效时刷新、重新规划。暂停、取消、地图/本体/配置变化或旧授权代次不能被刷新绕过。
- 普通多点导航若在点击开始时才发现过期，刷新后废弃旧预览，要求重新预览并再次手动开始；预览按钮本身不触发运动。
- 定位修正超过原预览允许的0.15m或15°时拒绝旧预览。刷新超时/数据失效时保留路线点，不启动导航；传感器、扫描、TF真实故障仍需排查。
- 前端预览等待上限调整为90秒，覆盖有界定位刷新及逐段规划；这不是允许定位陈旧90秒。巡检3D预约等待12秒，页面3D任务启动等待30秒；2D等待保持原值。
- 不改变2D和钢镚的原执行分支、不修改雷达标定、障碍检查或本体SDK。

### 小电脑部署

先把更新后的完整 `lzp_service` 和 `car_vel` 源码同步到小电脑对应目录。
保持机器人停止，结束现有导航/巡检并停止后台后执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_service car_vel
source /home/a/3dnav/install/setup.bash
```

按原来的方式只启动一套后台，浏览器 Ctrl+F5 强制刷新。

### 先静止验收

1. 选择星玑3D模式，加载地图并完成初始定位；机器人保持停止。
2. 在地形导航多点路线中编辑路线，静止等待30秒，再点预览。应出现无运动刷新，再计算路线；不得自行开始。
3. 在巡检页面同样等待后预览，确认成功和失败时路线点都保留。先不点开始巡检；实际巡检会运动，需另行人工确认场地并监护。
4. 如果刷新失败，保留后台日志与 `/amcl_pose`、`/scan`、TF状态。不要通过重复标初始点或扩大过期阈值掩盖数据链故障。

离线覆盖了真实后端方法、浏览器导航/巡检两页面和模拟定位/授权变更；不等于已在小电脑或机器人上验收。
