# 海康云台相机兼容说明（已停用）

当前产品默认使用 USB 固定摄像头，部署步骤请查看 `USB_CAMERA.md`。默认 launch
不再包含海康 IP、用户名或密码，也不会启动云台、光学变倍、红外或雨刮功能。

源码目录 `hikvision_camera/` 仍保留部分 HCNetSDK/RTSP 实现，是为了兼容旧巡检历史、
既有 API 路径和必要时的旧硬件回退；USB 模式不会加载这些原生库。

如以后确实需要恢复海康硬件，应显式使用：

```bash
python3 hikvision_camera/camera_web.py \
  --camera-type hikvision \
  --video-backend jpeg \
  --host <相机IP> \
  --username <用户名> \
  --password <密码>
```

不要把真实设备密码写回源码或 launch 文件。恢复旧硬件还需要重新评估巡检页面的
方向/倍率配置，因为当前产品界面和任务执行器已经按固定视角设计。
