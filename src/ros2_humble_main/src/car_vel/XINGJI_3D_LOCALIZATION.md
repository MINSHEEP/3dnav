# 星玑三维定位输出与防跳变

本次改动针对星玑 `terrain_3d`：原始 AMCL 不再直接驱动该范围内的导航位置/TF。原星玑二维与钢镚保留 AMCL 算法、导航参数、步态与速度配置。为统一页面和后端的位置来源，各导航消费者订阅 `/localization/navigation_pose`；在原有模式下，节点原样转发 AMCL 消息，包括原时间戳与协方差，不伪造定位心跳。

## 输出流程

1. 初始定位仍使用已保存的定位 PCD、按采集时间查询的 LIO TF 和 NDT/GICP。只对星玑三维请求启用新增 Action 字段 `robust_validation`，并核对 `map_load_id`。
2. 最多精修三个空间不同的粗配准候选；分别验证高度、倾斜、RMSE、重叠率、特征退化和多帧一致性，再比较合格候选。空间分离而质量接近时拒绝确认。原有质量门限没有放宽。
3. 完整六自由度结果验证通过后，还要验证**最终会输出的重力对齐平面变换**。不能把一个仅在倾斜状态下匹配良好的结果去掉 roll/pitch/z 后直接交给 Nav2。本项目控制接口仍是现有平面 Nav2 接口，没有改成六自由度路径控制。
4. `global_pose_authority` 持续用 LIO 推算位置，单线程 Action 进行局部三维匹配。局部请求搜索范围为 0.25 m、8°，预算 1.5 s，两个请求启动之间至少 0.75 s，不运行两个配准工作线程。粗位姿先按请求时间对应的 LIO 变换预测到最后一帧点云时间。
5. 局部结果必须通过服务器的候选质量检查，并在**相同采样时间的机器人位置**处检查创新量：最多 0.20 m、6°。不能直接比较远离地图原点时的 `map→odom` 平移量。新结果最大年龄 1 s，重复/倒序/未来时间戳拒绝。
6. 最多保留 2 s 的已验证全局锚点；期间必须有新鲜 LIO/TF。独立检查 LIO 位姿/时间连续性，LIO 重置、地图重新加载、本体代际改变后旧锚点与旧 Action 结果作废。新初始点请求也撤销旧锚点，并等待局部匹配真正结束后再使用配准线程。

这些是保守的初始工程门限，需要在目标机器上测量匹配耗时和误差后评估。重复结构仍可能产生小幅错误匹配；不能据此保证任何环境下永不误定位。

## TF 发布权和停车

- AMCL 运行参数 `tf_broadcast` 支持动态修改。进入已确认的星玑三维范围时，必须先确认原值、写入关闭、独立回读，并等待原 TF 流安静后，统一定位才可发布 `map→odom`。
- 写入前保存权限记录 `~/.ros/robot_navigation/global_pose_authority.json`。记录包含实际原值、AMCL 发布者 GID 和未决写入标记。超时写入继续等待原响应；不能假定取消本地 Future 就取消了远端操作。记录异常或未决状态不允许猜测发布权。此时停止整个后台再检查记录/重启。
- 不要手工同时启动另一个 `map→odom` 发布程序。检测到外部同名 TF 后统一定位停止输出。
- Python TF 接收使用 Humble 的单消息回调；通过有界缓存排除本节点发出的完整 TF 回显（含时间戳），其他同名变换会阻断输出。Humble 的普通 Python 回调不提供新版的发布者标识接口，不能依赖该接口判断来源。[Humble 执行器实现](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py)
- 退出星玑三维前先停止任务。恢复原 AMCL TF 前还要核对位置：有新鲜、同本体锚点时在静止状态下同步 AMCL；需要三个新时间戳样本与初始化位置一致，并通过协方差及静止检查。没有有效锚点、已换本体或同步失败时，在原模式重新设置初始点。原生二维启动不增加这一步。
- 行进中统一定位过期/失效，速度仲裁器对 Nav2/园区自动速度输出零速，路线管理器取消任务、撤销预览。运行中失效状态保持锁定；局部匹配重新成功不会自动续走，必须停止任务、重新确认初始点、预览并人工启动。
- 本模块不发站立、步态、速度补偿或硬急停指令。原有遥控、急停按键和其他保护仍按现有逻辑工作。星玑二维、钢镚不采用新增的三维匹配门限。

## 小电脑更新步骤

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

确认 `scope=xingji_3d`、`ready=true`、地图与 load_id 正确、`sensor_fresh=true`；`age_sec` 应周期性回落且始终在 2 s 内。查看 `rmse_m`、`overlap` 和 `error`，不能只看 ready 字样。

```bash
timeout 8s ros2 topic echo /localization/navigation_pose --once
```

将地图上的机器人位置、朝向与现场核对，静止观察至少 30 s。再进行低风险短距离单点测试，之后才测试多点与巡检。现有越障模式和速度上限没有改动；若停止，先读定位状态，不要放宽门限绕过。

切回二维后检查原值已恢复（默认配置通常为 `True`），且 `/localization/status` 为 `scope=legacy, ready=true`。原值来自实际读取，不能一律强制写 True。若 `legacy_initialization_required=true`，先保持停止并在二维模式重新设置初始点。

建议后续 bag 同时记录 `/localization/status`、`/localization/navigation_pose`、原始 `/amcl_pose`、`/tf`、`/tf_static`、`/Odometry`、`/cloud_registered`、`/lio_health`、`/navigation/velocity_guard` 和原有控制/任务话题，用于区分匹配质量、输出接纳与执行保护。

## 已验证与尚未验证

本地回归包含实际 bag 解码得到的 70 次 TF/LIO 样本：跳变前 30 次正常修正接纳，1.7 m / 53° 跳变后 40 次修正拒绝，锚点失效后停车。该回放故意假设候选质量合格，以单独检验创新量保护；**没有把旧 AMCL 输出当成新 PCL 配准结果**。

还验证参数写入/回读/晚响应、本体与模式隔离、运行中失效锁定、旧 Action 作废、二维消息原样转发和 AMCL 交接检查。C++ 候选打分与歧义辅助函数已编译运行。

已隔离且明确 `temporary_stop=false, hard_stop=false` 的单个可疑 LIO 采样不会单独撤销原锚点；新鲜的已接纳 TF、独立连续性与三维匹配检查仍需满足。真正重置、故障、暂停或持续无新 TF 仍会阻断输出，不会把重复 TF 当成新测量。

当前开发机没有 Linux ROS/PCL 运行环境，新增完整 PCL 配准链尚未在该 bag 上完成实际匹配和耗时测量；也没有进行实机运动测试。因此不能将上述回归视为现场可靠性验收。必须在小电脑完成完整编译、静止检查和逐步实测，再评估 CPU 负载及 2 s 锚点期限是否能持续满足。出现初始化失败应检查 PCD/坐标、高度与重力，而非恢复粗定位自动放行。
