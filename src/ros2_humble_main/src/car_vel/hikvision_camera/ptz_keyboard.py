#!/usr/bin/env python3
"""海康网络 SDK 云台键盘控制器，无 GUI 窗口。

控制链路：电脑通过 TCP 网络连接设备 ``--host:--port``（默认端口 8000），
调用 HCNetSDK 的 ``NET_DVR_PTZControlWithSpeed_Other`` 控制云台。
本脚本不打开串口、不使用 USB 转 RS485，也不发送 Pelco-D 命令。

按键：W/A/S/D 或方向键为上/左/下/右；+/- 调速；Q 或 Esc 退出。
按住方向键运动，松开停止；切换方向、按下其他键和退出时均会停止当前动作。
"""

import argparse
import os
import queue
from ctypes import CDLL, Structure, byref, c_bool, c_byte, c_char, c_char_p, c_int, c_uint, c_ushort, c_void_p, create_string_buffer, sizeof
from pathlib import Path

from pynput import keyboard


# SDK 随项目分发，避免依赖终端的当前工作目录。
SDK_DIR = Path(__file__).resolve().parent / "sdk"
# HCNetSDK 云台方向命令和 PTZ 坐标读取命令。
TILT_UP, TILT_DOWN, PAN_LEFT, PAN_RIGHT = 21, 22, 23, 24
NET_DVR_GET_PTZPOS = 293


class LocalSdkPath(Structure):
    """HCNetSDK 初始化结构 ``NET_DVR_LOCAL_SDK_PATH``。

    ``sPath`` 指向 SDK 组件目录；``byRes`` 是 SDK 保留字段，必须保留其长度。
    """

    _fields_ = [("sPath", c_char * 256), ("byRes", c_byte * 128)]


class PtzPosition(Structure):
    """HCNetSDK 的 ``NET_DVR_PTZPOS`` 坐标结构。

    本机芯的 ``wTiltPos`` 按默认映射为 ``原始值 / 100 - 90`` 度。
    """

    _fields_ = [("wAction", c_ushort), ("wPanPos", c_ushort), ("wTiltPos", c_ushort), ("wZoomPos", c_ushort)]


class HikvisionClient:
    """封装海康网络 SDK 登录、云台控制及坐标查询。

    参数：
        host: 设备 IP 地址或主机名。
        port: HCNetSDK 服务端口，通常为 8000，不是本机串口号。
        username: 设备登录用户名。
        password: 设备登录密码，仅保存在运行内存。
        tilt_coordinate_scale: 俯仰原始坐标每度的单位数，默认 100。
        tilt_zero_offset: 原始坐标对应 0 度前的偏移，默认 90。
    """

    def __init__(self, host: str, port: int, username: str, password: str, tilt_coordinate_scale: float, tilt_zero_offset: float):
        self.host, self.port = host, port
        self.username, self.password = username, password
        self.tilt_coordinate_scale = tilt_coordinate_scale
        self.tilt_zero_offset = tilt_zero_offset
        self.sdk = None
        self.user_id = -1

    def _error(self) -> int:
        """返回最近一次 HCNetSDK 调用的错误码。"""
        return int(self.sdk.NET_DVR_GetLastError())

    def connect(self) -> None:
        """加载 SDK、配置依赖库路径并通过网络登录设备。"""
        if not SDK_DIR.is_dir():
            raise RuntimeError(f"未找到 SDK 库目录: {SDK_DIR}")
        self.sdk = CDLL(str(SDK_DIR / "libhcnetsdk.so"))
        self.sdk.NET_DVR_SetSDKInitCfg.argtypes = [c_int, c_void_p]
        self.sdk.NET_DVR_SetSDKInitCfg.restype = c_bool
        self.sdk.NET_DVR_Init.restype = c_bool
        self.sdk.NET_DVR_Login_V30.argtypes = [c_char_p, c_ushort, c_char_p, c_char_p, c_void_p]
        self.sdk.NET_DVR_Login_V30.restype = c_int
        self.sdk.NET_DVR_GetLastError.restype = c_uint
        self.sdk.NET_DVR_PTZControlWithSpeed_Other.argtypes = [c_int, c_int, c_uint, c_uint, c_uint]
        self.sdk.NET_DVR_PTZControlWithSpeed_Other.restype = c_bool
        self.sdk.NET_DVR_GetDVRConfig.argtypes = [c_int, c_uint, c_int, c_void_p, c_uint, c_void_p]
        self.sdk.NET_DVR_GetDVRConfig.restype = c_bool

        local_path = LocalSdkPath()
        local_path.sPath = str(SDK_DIR).encode()
        crypto_path = create_string_buffer(str(SDK_DIR / "libcrypto.so.1.1").encode())
        ssl_path = create_string_buffer(str(SDK_DIR / "libssl.so.1.1").encode())
        for config_type, value in ((2, byref(local_path)), (3, crypto_path), (4, ssl_path)):
            if not self.sdk.NET_DVR_SetSDKInitCfg(config_type, value):
                raise RuntimeError(f"SDK 初始化路径配置失败，错误码: {self._error()}")
        if not self.sdk.NET_DVR_Init():
            raise RuntimeError(f"SDK 初始化失败，错误码: {self._error()}")

        device_info = create_string_buffer(1024)
        self.user_id = self.sdk.NET_DVR_Login_V30(self.host.encode(), self.port, self.username.encode(), self.password.encode(), byref(device_info))
        if self.user_id < 0:
            error = self._error()
            self.sdk.NET_DVR_Cleanup()
            raise RuntimeError(f"设备登录失败，错误码: {error}")

    def get_tilt_position(self, channel: int) -> tuple[int, float] | None:
        """读取通道的俯仰原始坐标和角度。

        参数：
            channel: 设备视频/云台通道号，默认由 ``--channel`` 指定为 1。

        返回：
            ``(原始坐标, 映射角度)``；SDK 查询失败时返回 ``None``，调用方据此
            拒绝上下运动，避免无法判断限位时继续驱动电机。
        """
        position, returned = PtzPosition(), c_uint(0)
        if not self.sdk.NET_DVR_GetDVRConfig(self.user_id, NET_DVR_GET_PTZPOS, channel, byref(position), sizeof(position), byref(returned)):
            return None
        raw_tilt = int(position.wTiltPos)
        return raw_tilt, raw_tilt / self.tilt_coordinate_scale - self.tilt_zero_offset

    def start(self, channel: int, command: int, speed: int) -> None:
        """通过网络 SDK 开始单个方向动作。

        ``command`` 为 HCNetSDK 云台命令，``speed`` 必须为 1 到 7。
        """
        if not self.sdk.NET_DVR_PTZControlWithSpeed_Other(self.user_id, channel, command, 0, speed):
            raise RuntimeError(f"云台启动失败，错误码: {self._error()}")

    def stop(self, channel: int, command: int, speed: int) -> None:
        """通过网络 SDK 停止指定的云台方向动作。"""
        if not self.sdk.NET_DVR_PTZControlWithSpeed_Other(self.user_id, channel, command, 1, speed):
            raise RuntimeError(f"云台停止失败，错误码: {self._error()}")

    def stop_all(self, channel: int, speed: int) -> None:
        """停止四个方向；退出和异常路径均调用，防止云台持续运动。"""
        if self.sdk is None or self.user_id < 0:
            return
        for command in (TILT_UP, TILT_DOWN, PAN_LEFT, PAN_RIGHT):
            self.sdk.NET_DVR_PTZControlWithSpeed_Other(self.user_id, channel, command, 1, speed)

    def close(self) -> None:
        """登出设备并释放 HCNetSDK 全局资源。"""
        if self.sdk is None:
            return
        if self.user_id >= 0:
            self.sdk.NET_DVR_Logout(self.user_id)
            self.user_id = -1
        self.sdk.NET_DVR_Cleanup()


class PtzKeyboardController:
    """用 pynput 全局监听键盘并串行处理云台动作。

    参数：
        client: 已配置但尚未登录的 :class:`HikvisionClient`。
        args: :func:`parse_args` 返回的运行参数。

    ``pynput`` 仅负责本机按键事件；实际云台命令始终由 ``client`` 经网络 SDK 下发。
    """

    COMMANDS = {"w": TILT_UP, "up": TILT_UP, "s": TILT_DOWN, "down": TILT_DOWN, "a": PAN_LEFT, "left": PAN_LEFT, "d": PAN_RIGHT, "right": PAN_RIGHT}
    COMMAND_NAMES = {TILT_UP: "上转", TILT_DOWN: "下转", PAN_LEFT: "左转", PAN_RIGHT: "右转"}

    def __init__(self, client: HikvisionClient, args: argparse.Namespace):
        self.client, self.args = client, args
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.pressed_keys: set[str] = set()
        self.active_command: int | None = None
        self.running = True

    @staticmethod
    def key_name(key: keyboard.Key | keyboard.KeyCode) -> str | None:
        """将 pynput 的字符键和特殊键统一为命令表使用的字符串。"""
        if key in (keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right, keyboard.Key.esc):
            return key.name
        if isinstance(key, keyboard.KeyCode) and key.char:
            return key.char.lower()
        return None

    def on_press(self, key: keyboard.Key | keyboard.KeyCode) -> None:
        """监听线程回调：只入队，不在回调线程直接访问 SDK。"""
        name = self.key_name(key)
        if name:
            self.events.put(("press", name))

    def on_release(self, key: keyboard.Key | keyboard.KeyCode) -> None:
        """监听线程回调：记录真实松键事件，供主循环停止动作。"""
        name = self.key_name(key)
        if name:
            self.events.put(("release", name))

    def run(self) -> None:
        """登录设备并运行按键事件循环，退出时无条件停止云台。"""
        self.client.connect()
        print("已连接。W/A/S/D 或方向键按住转动，松开立即停止；+/- 调速；Q/Esc 退出。")
        print("俯仰限位: +90.0° / -90.0°，位置检查周期: %d ms。" % self.args.poll_ms)
        print("控制期间全局捕获键盘，退出后自动恢复。")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=True) as listener:
            try:
                while self.running:
                    timeout = self.args.poll_ms / 1000 if self.active_command in (TILT_UP, TILT_DOWN) else 0.5
                    try:
                        event_type, key_name = self.events.get(timeout=timeout)
                        self.handle_event(event_type, key_name)
                    except queue.Empty:
                        self.poll_tilt_limit()
            except KeyboardInterrupt:
                print("\n收到中断信号，正在停止云台。")
            finally:
                listener.stop()
                self.stop_current("退出程序")
                self.client.stop_all(self.args.channel, self.args.speed)
                self.client.close()

    def handle_event(self, event_type: str, key_name: str) -> None:
        """处理按下或松开事件，保证一次仅有一个方向处于运行状态。"""
        command = self.COMMANDS.get(key_name)
        if event_type == "press":
            if key_name in self.pressed_keys:
                return
            self.pressed_keys.add(key_name)
            if key_name in ("q", "esc"):
                self.running = False
                return
            if key_name in ("+", "="):
                self.stop_current("调整速度")
                self.args.speed = min(7, self.args.speed + 1)
                print(f"当前速度: {self.args.speed}")
                return
            if key_name in ("-", "_"):
                self.stop_current("调整速度")
                self.args.speed = max(1, self.args.speed - 1)
                print(f"当前速度: {self.args.speed}")
                return
            if command is None:
                self.stop_current("按下其他按键")
                return
            if command != self.active_command:
                self.stop_current("切换方向")
                self.start_command(command)
            return

        self.pressed_keys.discard(key_name)
        if command == self.active_command and not self.command_is_held(command):
            self.stop_current("按键松开")

    def command_is_held(self, command: int) -> bool:
        """判断同一方向是否仍被 W/A/S/D 或对应方向键保持。"""
        return any(self.COMMANDS.get(key) == command for key in self.pressed_keys)

    def start_command(self, command: int) -> None:
        """检查俯仰限位后开始动作；水平动作不需要位置查询。"""
        if command in (TILT_UP, TILT_DOWN):
            position = self.client.get_tilt_position(self.args.channel)
            if position is None:
                print("无法读取云台俯仰位置，已拒绝上下转动；水平控制可用。")
                return
            raw_tilt, angle = position
            if self.at_tilt_limit(command, angle):
                print(f"俯仰原始坐标 {raw_tilt}，角度 {angle:.2f}° 已到限位，未启动电机。")
                return
        try:
            self.client.start(self.args.channel, command, self.args.speed)
        except RuntimeError as error:
            print(f"错误: {error}")
            return
        self.active_command = command
        print(f"{self.COMMAND_NAMES[command]}中。")

    def poll_tilt_limit(self) -> None:
        """上下运动时周期查询坐标，到达正负限位立即停止。"""
        if self.active_command not in (TILT_UP, TILT_DOWN):
            return
        position = self.client.get_tilt_position(self.args.channel)
        if position is None:
            self.stop_current("俯仰位置读取失败，安全停止")
            print("俯仰位置读取失败，已安全停止。")
            return
        raw_tilt, angle = position
        if self.at_tilt_limit(self.active_command, angle):
            self.stop_current(f"达到俯仰限位 {angle:.2f}°")
            print(f"俯仰原始坐标 {raw_tilt}，已到 {angle:.2f}° 限位，电机停止。")

    def at_tilt_limit(self, command: int, angle: float) -> bool:
        """按运行参数和安全余量判断指定方向是否已到机械限位。"""
        if command == TILT_UP:
            return angle >= self.args.tilt_up_limit - self.args.limit_margin
        return angle <= self.args.tilt_down_limit + self.args.limit_margin

    def stop_current(self, reason: str) -> None:
        """停止当前动作并输出简洁的停止原因。"""
        command = self.active_command
        self.active_command = None
        if command is None:
            return
        try:
            self.client.stop(self.args.channel, command, self.args.speed)
            print(f"已停止: {reason}。")
        except RuntimeError as error:
            print(f"停止命令失败: {error}")


def parse_args() -> argparse.Namespace:
    """解析并校验命令行参数，密码只接受参数或环境变量，不写入文件。"""
    parser = argparse.ArgumentParser(
        description="海康网络 SDK 云台全局键盘控制（Linux X11，无 GUI、无串口）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default="192.168.1.19", help="设备 IP 地址或主机名")
    parser.add_argument("--username", default="admin", help="设备登录用户名")
    parser.add_argument("--password", default=os.environ.get("HIKVISION_PASSWORD"), help="设备密码，也可设置 HIKVISION_PASSWORD")
    parser.add_argument("--port", type=int, default=8000, help="HCNetSDK 网络服务端口")
    parser.add_argument("--channel", type=int, default=1, help="设备云台通道号")
    parser.add_argument("--speed", type=int, default=4, choices=range(1, 8), help="SDK 云台速度，范围 1 到 7")
    parser.add_argument("--tilt-up-limit", type=float, default=90.0, help="上转停止角度")
    parser.add_argument("--tilt-down-limit", type=float, default=-90.0, help="下转停止角度")
    parser.add_argument("--tilt-coordinate-scale", type=float, default=100.0, help="俯仰原始坐标每一度对应的单位数")
    parser.add_argument("--tilt-zero-offset", type=float, default=90.0, help="俯仰原始坐标的角度偏移")
    parser.add_argument("--limit-margin", type=float, default=0.5, help="接近限位时预留的停止余量（度）")
    parser.add_argument("--poll-ms", type=int, default=50, help="上下运动时读取俯仰坐标的间隔（毫秒）")
    args = parser.parse_args()
    if not args.password:
        parser.error("请通过 --password 或 HIKVISION_PASSWORD 提供设备密码")
    if args.tilt_down_limit >= args.tilt_up_limit:
        parser.error("--tilt-down-limit 必须小于 --tilt-up-limit")
    if args.tilt_coordinate_scale <= 0 or args.limit_margin < 0 or args.poll_ms < 20:
        parser.error("坐标比例必须大于 0，安全余量不能为负，检查周期不能小于 20 ms")
    return args


def main() -> int:
    """创建网络 SDK 客户端并启动终端键盘控制器。"""
    args = parse_args()
    client = HikvisionClient(args.host, args.port, args.username, args.password, args.tilt_coordinate_scale, args.tilt_zero_offset)
    controller = PtzKeyboardController(client, args)
    try:
        controller.run()
    except RuntimeError as error:
        client.stop_all(args.channel, args.speed)
        client.close()
        print(f"错误: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
