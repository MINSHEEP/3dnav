# 导航启动与热成像恢复（2026-10-26）

## 改动边界

- 导航增加常驻、只读 `navigation_startup_monitor`。每秒发布 `/navigation/startup/status`，包含当前导航进程PID、启动经过时间、缺少节点、生命周期状态和动作服务可用性。
- 查询对象：map_server、amcl、planner_server、controller_server、bt_navigator。状态查询无响应超过3秒会取消本次查询并重新检查，不激活节点、不设置初始点、不发送运动命令。
- 默认60秒未就绪显示超时原因，仍继续观察，后来恢复会自动更新状态。进程PID改变后清除旧节点查询和旧计时。
- 地形导航页面显示诊断；同连接重复启动请求合并；重连前的异步结果不再覆盖新连接。已有导航进程由原启动管理器复用。
- 人工“重试导航服务（不运动）”只在诊断超时/已停止、无已知运行任务且导航尚未就绪时可用。使用现有服务重启导航树，保留停止保护；页面暂停自动站立重试，地图和定位需要重新确认。它不是恢复任务或重新授权运动。
- 保留当前建图/遥控/导航互斥关系。进入遥控会停止导航；导航启动期间不要通过反复切换页面尝试加速。

## 热成像

HTTP服务与原生SDK采集隔离：Linux sysfs确认2bdf:0102、所选序号/序列号及USB节点读写权限后，才创建全新的spawn子进程。SDK只在该子进程执行一次会话，不在崩溃过的进程里重复初始化。

- 同用户的采集子进程通过文件锁限制并发，不能取代对其他SDK程序占用设备的检查。
- 子进程异常退出/SDK卡住时，HTTP服务保留；清除上一会话帧、温度及当前告警状态，保留阈值、任务模式和事件序号。
- 初始SDK无输出45秒、采集有过帧后无输出至少10秒会回收该子进程。
- 同一次USB接入连续失败最多5次，退避3、6、12、24、30秒；之后等待重新接入或热服务重启。连续正常采集60秒后重置失败预算。
- USB缺席或权限不足时每秒检查，不消耗SDK重试次数；重新接入后无需重启整个机器人后台即可开始尝试采集。
- 不卸载uvcvideo、不修改udev、不重置USB控制器、不切断集线器供电。

**这不证明开机USB不枚举已解决。** 如果 `lsusb -d 2bdf:0102` 无输出，程序只能等待。需取得未拔插时的完整 `thermal-usb-boot.log` 继续分析相机上电、固件、线材及主机USB初始化。不要仅因拔插有效就断定供电不足。

## 部署

先同步本次代码到小电脑 `/home/a/3dnav`，机器人停止、结束当前任务和旧后台后，在终端依次执行：

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lzp_service car_vel
source /home/a/3dnav/install/setup.bash
```

启动单套后台并保留日志：

```bash
RCUTILS_COLORIZED_OUTPUT=0 ros2 launch car_vel launch_manager.launch.py 2>&1 | tee "$HOME/nav_startup_recovery.log"
```

浏览器强制刷新（Ctrl+F5）。先只检查状态，不发送目标。

## 验收

1. 打开地形导航，停留页面。应按实际状态显示等待节点/激活/动作服务/就绪；故障时60秒后给出具体缺失项，而不是永远只显示“等待Nav2”。
2. 已就绪时刷新页面，后端应复用相同导航PID；旧启动请求完成不能把就绪改回等待。切到遥控后返回则会新建PID，这是互斥模式的正常行为。
3. 无运动任务时测试重试：不得发送站立、复位、目标；重新加载地图和确认定位后才继续人工操作。
4. 热相机未连接时 `/api/status` 应仍可访问，返回“USB尚未识别”；只接入热相机后应自动尝试采集。SDK崩溃时网页状态服务应保留，日志包含 `[ThermalRecovery]` 和子进程退出信息。
5. 拔掉热相机后旧温度/帧不可作为当前有效数据，高温阈值和任务模式保持原值。

只读诊断命令（每条单独执行）：

```bash
timeout 8s ros2 topic echo /navigation/startup/status --once --full-length
```

```bash
curl -sS --max-time 5 http://127.0.0.1:8767/api/status | python3 -m json.tool --no-ensure-ascii
```

本地离线验证不能替代Linux ROS节点生命周期、实际SDK或机器狗现场验收。fastdds维护命令的Exec format error没有被认定为本次唯一根因，本次不自动覆盖系统安装的命令。
