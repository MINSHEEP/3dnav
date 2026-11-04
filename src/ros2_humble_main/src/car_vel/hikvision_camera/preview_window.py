#!/usr/bin/env python3
"""海康 V40 回调预览窗口，只负责显示实时画面。

数据路径与 SDK Python 示例一致：
``NET_DVR_RealPlay_V40 -> 实时数据回调 -> PlayM4_OpenStream/InputData -> X11 窗口``。
画面解码和渲染由 SDK 的 PlayM4 完成；本脚本不含云台控制和全局键盘钩子。
"""

import argparse
import os
import socket
import ssl
import sys
import threading
import tkinter as tk
import urllib.error
import urllib.request
import xml.etree.ElementTree as ElementTree
from ctypes import CDLL, RTLD_GLOBAL, CFUNCTYPE, POINTER, Structure, byref, c_byte, c_char, c_char_p, c_int, c_uint, c_ubyte, c_ushort, c_void_p, create_string_buffer, sizeof
from pathlib import Path
from tkinter import font as tkfont


# SDK 随迁移包放在 sdk/，与当前工作目录无关。
SDK_DIR = Path(__file__).resolve().parent / "sdk"
NET_DVR_SYSHEAD, NET_DVR_STREAMDATA = 1, 2
SDK_ERRORS = {
    1: "用户名或密码错误",
    2: "当前用户没有权限",
    5: "设备连接数已满",
    7: "连接设备服务失败，请检查 IP、8000 端口和本机网络",
    10: "接收设备响应超时",
    23: "设备或当前调用方式不支持",
    64: "Player SDK 加载失败，请检查动态库路径",
}


class LocalSdkPath(Structure):
    """HCNetSDK 初始化路径结构。"""

    _fields_ = [("sPath", c_char * 256), ("byRes", c_byte * 128)]


class DeviceInfoV30(Structure):
    """SDK 头文件中的 NET_DVR_DEVICEINFO_V30，避免依赖固定字节偏移。"""

    _fields_ = [
        ("sSerialNumber", c_byte * 48), ("byAlarmInPortNum", c_byte), ("byAlarmOutPortNum", c_byte),
        ("byDiskNum", c_byte), ("byDVRType", c_byte), ("byChanNum", c_byte), ("byStartChan", c_byte),
        ("byAudioChanNum", c_byte), ("byIPChanNum", c_byte), ("byZeroChanNum", c_byte),
        ("byMainProto", c_byte), ("bySubProto", c_byte), ("bySupport", c_byte), ("bySupport1", c_byte),
        ("bySupport2", c_byte), ("wDevType", c_ushort), ("bySupport3", c_byte), ("byMultiStreamProto", c_byte),
        ("byStartDChan", c_byte), ("byStartDTalkChan", c_byte), ("byHighDChanNum", c_byte),
        ("bySupport4", c_byte), ("byLanguageType", c_byte), ("byVoiceInChanNum", c_byte),
        ("byStartVoiceInChanNo", c_byte), ("bySupport5", c_byte), ("bySupport6", c_byte),
        ("byMirrorChanNum", c_byte), ("wStartMirrorChanNo", c_ushort),
        ("bySupport7", c_byte), ("byRes2", c_byte),
    ]


if sizeof(DeviceInfoV30) != 80:
    raise RuntimeError(f"NET_DVR_DEVICEINFO_V30 ABI 异常: {sizeof(DeviceInfoV30)} 字节，预期 80")


class PreviewInfo(Structure):
    """Linux ``NET_DVR_PREVIEWINFO``；使用 V40 回调取流时 hPlayWnd 为 0。"""

    _fields_ = [
        # HCNetSDK.h 在 Linux 中将 HWND 定义为 unsigned int，结构必须保持 280 字节。
        ("lChannel", c_int), ("dwStreamType", c_uint), ("dwLinkMode", c_uint), ("hPlayWnd", c_uint),
        ("bBlocked", c_uint), ("bPassbackRecord", c_uint), ("byPreviewMode", c_ubyte),
        ("byStreamID", c_ubyte * 32), ("byProtoType", c_ubyte), ("byRes1", c_ubyte),
        ("byVideoCodingType", c_ubyte), ("dwDisplayBufNum", c_uint), ("byNPQMode", c_ubyte),
        ("byRecvMetaData", c_ubyte), ("byDataType", c_ubyte), ("byRes", c_ubyte * 213),
    ]


RealDataCallback = CFUNCTYPE(None, c_int, c_uint, POINTER(c_ubyte), c_uint, c_void_p)


def _xml_text(payload: bytes, name: str) -> str | None:
    """按 XML 本地名称取值，兼容 ISAPI 默认命名空间。"""
    try:
        root = ElementTree.fromstring(payload)
    except ElementTree.ParseError:
        return None
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] == name:
            value = (element.text or "").strip()
            return value or None
    return None


def _isapi_request(args: argparse.Namespace, path: str, *, method: str = "GET", payload: bytes | None = None) -> bytes:
    """直连相机的标准 Digest 请求，避免系统 HTTP 代理改写认证 request-uri。"""
    url = f"{args.http_scheme}://{args.host}:{args.http_port}{path}"
    manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    manager.add_password(None, url, args.username, args.password)
    handlers = [urllib.request.ProxyHandler({}), urllib.request.HTTPDigestAuthHandler(manager)]
    if args.http_scheme == "https":
        context = ssl._create_unverified_context() if args.insecure_tls else ssl.create_default_context()
        handlers.append(urllib.request.HTTPSHandler(context=context))
    headers = {"Accept": "application/xml"}
    if payload is not None:
        headers["Content-Type"] = "application/xml; charset=UTF-8"
    request = urllib.request.Request(url, data=payload, method=method, headers=headers)
    try:
        with urllib.request.build_opener(*handlers).open(request, timeout=6) as response:
            return response.read(64 * 1024)
    except urllib.error.HTTPError as error:
        body = error.read(1024).decode("utf-8", errors="replace").strip().replace("\n", " ")
        raise RuntimeError(f"ISAPI {method} {path} 失败: HTTP {error.code}" + (f"（{body[:240]}）" if body else "")) from error
    except urllib.error.URLError as error:
        raise RuntimeError(f"日夜模式请求失败: {error.reason}") from error


def set_ir_mode(args: argparse.Namespace) -> None:
    """设置当前预览通道日夜模式，保留设备已经配置的夜转日阈值并复读确认。"""
    path = f"/ISAPI/Image/channels/{args.channel}/IrcutFilter"
    current = _isapi_request(args, path)
    level = _xml_text(current, "nightToDayFilterLevel")
    try:
        threshold = int(level) if level is not None else 2
    except ValueError:
        threshold = 2
    threshold = threshold if 0 <= threshold <= 255 else 2
    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<IrcutFilter version="2.0" xmlns="http://www.std-cgi.com/ver20/XMLSchema">'
        f"<IrcutFilterType>{args.ir_mode}</IrcutFilterType><nightToDayFilterLevel>{threshold}</nightToDayFilterLevel>"
        "</IrcutFilter>"
    ).encode("utf-8")
    _isapi_request(args, path, method="PUT", payload=xml)
    configured = _isapi_request(args, path)
    if _xml_text(configured, "IrcutFilterType") != args.ir_mode:
        raise RuntimeError("设备未确认日夜模式")


class V40Preview:
    """独立的 V40 取流与 PlayM4 渲染会话。

    V40 回调只进行 SDK 缓冲转发，避免 Python 在回调中做耗时图像处理。
    """

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.sdk = self.player = None
        self.user_id = self.real_handle = -1
        self.player_port = c_int(-1)
        self.callback = RealDataCallback(self._on_real_data)
        self.window_id = 0
        self.stream_opened = False
        self.stream_error = ""
        self.device_info = DeviceInfoV30()
        self.player_dependencies = []
        self._closing = threading.Event()
        self._closed = threading.Event()
        self._close_lock = threading.Lock()
        self._player_lock = threading.RLock()

    def _error(self) -> int:
        return int(self.sdk.NET_DVR_GetLastError())

    def _sdk_error(self, prefix: str) -> str:
        """将 SDK 错误码转换为终端可直接处理的说明。"""
        code = self._error()
        return f"{prefix}: {code}" + (f"（{SDK_ERRORS[code]}）" if code in SDK_ERRORS else "")

    def _player_error(self) -> int:
        return int(self.player.PlayM4_GetLastError(self.player_port.value))

    def connect(self) -> None:
        """加载同一 SDK 发行包中的主库、组件库和播放库后登录设备。"""
        if not SDK_DIR.is_dir():
            raise RuntimeError(f"未找到 SDK: {SDK_DIR}")
        # 在登录前验证 HCNetSDK 的 TCP 服务，避免错误码 7 被误判为密码问题。
        try:
            with socket.create_connection((self.args.host, self.args.port), timeout=3):
                pass
        except OSError as error:
            raise RuntimeError(
                f"设备网络不可达: {self.args.host}:{self.args.port}（{error}）；"
                "请确认本机与相机位于可互通网段且 8000/TCP 未被拦截"
            ) from error
        self.sdk = CDLL(str(SDK_DIR / "libhcnetsdk.so"))
        self.sdk.NET_DVR_SetSDKInitCfg.argtypes = [c_int, c_void_p]
        self.sdk.NET_DVR_SetSDKInitCfg.restype = c_int
        self.sdk.NET_DVR_Init.restype = c_int
        self.sdk.NET_DVR_Login_V30.argtypes = [c_char_p, c_ushort, c_char_p, c_char_p, POINTER(DeviceInfoV30)]
        self.sdk.NET_DVR_Login_V30.restype = c_int
        self.sdk.NET_DVR_RealPlay_V40.argtypes = [c_int, POINTER(PreviewInfo), RealDataCallback, c_void_p]
        self.sdk.NET_DVR_RealPlay_V40.restype = c_int
        self.sdk.NET_DVR_StopRealPlay.argtypes = [c_int]
        self.sdk.NET_DVR_StopRealPlay.restype = c_int
        self.sdk.NET_DVR_Logout.argtypes = [c_int]
        self.sdk.NET_DVR_Logout.restype = c_int
        self.sdk.NET_DVR_Cleanup.restype = c_int
        self.sdk.NET_DVR_GetLastError.restype = c_uint
        local_path = LocalSdkPath(); local_path.sPath = str(SDK_DIR).encode()
        for kind, value in ((2, byref(local_path)), (3, create_string_buffer(str(SDK_DIR / "libcrypto.so.1.1").encode())), (4, create_string_buffer(str(SDK_DIR / "libssl.so.1.1").encode()))):
            if not self.sdk.NET_DVR_SetSDKInitCfg(kind, value):
                raise RuntimeError(self._sdk_error("SDK 路径配置失败"))
        if not self.sdk.NET_DVR_Init():
            raise RuntimeError(self._sdk_error("SDK 初始化失败"))
        self.user_id = self.sdk.NET_DVR_Login_V30(self.args.host.encode(), self.args.port, self.args.username.encode(), self.args.password.encode(), byref(self.device_info))
        if self.user_id < 0:
            error = self._error(); self.sdk.NET_DVR_Cleanup()
            message = f"设备登录失败: {error}" + (f"（{SDK_ERRORS[error]}）" if error in SDK_ERRORS else "")
            raise RuntimeError(message)
        # SDK 示例通过启动时 LD_LIBRARY_PATH 解析这些依赖；main() 已保证该环境。
        self.player = CDLL(str(SDK_DIR / "libPlayCtrl.so"), mode=RTLD_GLOBAL)
        self.player.PlayM4_GetPort.argtypes = [POINTER(c_int)]; self.player.PlayM4_GetPort.restype = c_int
        self.player.PlayM4_SetStreamOpenMode.argtypes = [c_int, c_uint]; self.player.PlayM4_SetStreamOpenMode.restype = c_int
        self.player.PlayM4_OpenStream.argtypes = [c_int, c_void_p, c_uint, c_uint]; self.player.PlayM4_OpenStream.restype = c_int
        self.player.PlayM4_Play.argtypes = [c_int, c_void_p]; self.player.PlayM4_Play.restype = c_int
        self.player.PlayM4_InputData.argtypes = [c_int, c_void_p, c_uint]; self.player.PlayM4_InputData.restype = c_int
        self.player.PlayM4_Stop.argtypes = [c_int]; self.player.PlayM4_Stop.restype = c_int
        self.player.PlayM4_CloseStream.argtypes = [c_int]; self.player.PlayM4_CloseStream.restype = c_int
        self.player.PlayM4_FreePort.argtypes = [c_int]; self.player.PlayM4_FreePort.restype = c_int
        self.player.PlayM4_GetLastError.argtypes = [c_int]; self.player.PlayM4_GetLastError.restype = c_uint
        if not self.player.PlayM4_GetPort(byref(self.player_port)):
            raise RuntimeError("PlayM4 无可用播放端口")

    def start(self, window_id: int) -> None:
        """启动 V40 取流；系统头和码流随后由 ``_on_real_data`` 处理。"""
        self.window_id = window_id
        info = PreviewInfo()
        info.lChannel, info.dwStreamType, info.dwLinkMode = self.args.channel, self.args.stream, self.args.link_mode
        info.hPlayWnd, info.bBlocked, info.dwDisplayBufNum = 0, 0, 1
        self.real_handle = self.sdk.NET_DVR_RealPlay_V40(self.user_id, byref(info), self.callback, None)
        if self.real_handle < 0:
            raise RuntimeError(self._sdk_error("V40 预览失败"))

    def _on_real_data(self, _handle: int, data_type: int, buffer, size: int, _user) -> None:
        """SDK 回调：系统头打开解码器，视频数据仅转发给 PlayM4。"""
        if self._closing.is_set():
            return
        try:
            with self._player_lock:
                # close() 等待播放器锁时不再进入 PlayM4，避免释放后回调继续访问端口。
                if self._closing.is_set():
                    return
                if data_type == NET_DVR_SYSHEAD and not self.stream_opened:
                    if not self.player.PlayM4_SetStreamOpenMode(self.player_port.value, 0):
                        self.stream_error = f"播放模式失败: {self._player_error()}"; return
                    if not self.player.PlayM4_OpenStream(self.player_port.value, buffer, size, 8 * 1024 * 1024):
                        self.stream_error = f"打开码流失败: {self._player_error()}"; return
                    if not self.player.PlayM4_Play(self.player_port.value, c_void_p(self.window_id)):
                        self.stream_error = f"启动渲染失败: {self._player_error()}"; return
                    self.stream_opened = True
                elif data_type == NET_DVR_STREAMDATA and self.stream_opened:
                    if not self.player.PlayM4_InputData(self.player_port.value, buffer, size):
                        # 缓冲瞬时满只丢当前包，避免在 SDK 回调内无限阻塞并拖垮取流线程。
                        self.stream_error = f"输入码流失败: {self._player_error()}"
        except BaseException as error:
            self.stream_error = f"播放回调异常: {error}"

    def runtime_info(self) -> str:
        return f"V40 回调预览；模拟通道 {self.device_info.byChanNum}（起始 {self.device_info.byStartChan}）；IP 通道 {self.device_info.byIPChanNum}"

    def close(self) -> None:
        """按取流、解码、登录、SDK 顺序释放，任一失败不阻断后续释放。"""
        if self.sdk is None:
            return
        with self._close_lock:
            if self._closed.is_set():
                return
            self._closing.set()
            handle = self.real_handle
            self.real_handle = -1
            # StopRealPlay 可能等待回调返回，因此不能持有 _player_lock 调用它。
            if handle >= 0:
                self.sdk.NET_DVR_StopRealPlay(handle)
            with self._player_lock:
                if self.player_port.value >= 0 and self.player is not None:
                    self.player.PlayM4_Stop(self.player_port.value)
                    self.player.PlayM4_CloseStream(self.player_port.value)
                    self.player.PlayM4_FreePort(self.player_port.value)
                    self.player_port = c_int(-1)
            if self.user_id >= 0:
                self.sdk.NET_DVR_Logout(self.user_id)
                self.user_id = -1
            self.sdk.NET_DVR_Cleanup()
            self._closed.set()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="海康 V40 回调实时预览", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--host", default="192.168.1.19", help="设备 IP 或主机名")
    parser.add_argument("--username", default="admin", help="设备用户名")
    parser.add_argument("--password", default=os.environ.get("HIKVISION_PASSWORD"), help="设备密码，也可设置 HIKVISION_PASSWORD")
    parser.add_argument("--port", type=int, default=8000, help="HCNetSDK 端口")
    parser.add_argument("--channel", type=int, default=1, help="预览与日夜模式通道")
    parser.add_argument("--stream", type=int, choices=(0, 1), default=0, help="0 主码流，1 子码流")
    parser.add_argument("--link-mode", type=int, choices=range(8), default=0, help="V40 传输方式：0 TCP，1 UDP，2 组播，3 RTP，4 RTSP/RTP，5 RTSP/HTTP，6 HRUDP，7 RTSP/HTTPS")
    parser.add_argument("--ir-mode", choices=("keep", "day", "night"), default="keep", help="启动时设置日夜模式")
    parser.add_argument("--http-scheme", choices=("http", "https"), default="http", help="ISAPI 协议；设备支持时应选择 https")
    parser.add_argument("--http-port", type=int, default=80, help="ISAPI 端口")
    parser.add_argument("--insecure-tls", action="store_true", help="仅自签名 HTTPS 证书调试时跳过证书校验")
    args = parser.parse_args()
    if not args.password: parser.error("请通过 --password 或 HIKVISION_PASSWORD 提供设备密码")
    if args.channel <= 0 or not 1 <= args.port <= 65535 or not 1 <= args.http_port <= 65535: parser.error("通道或端口参数无效")
    return args


def ensure_player_library_path() -> None:
    """在加载 PlayM4 前配置动态链接器路径，等同 SDK 示例的启动脚本。

    Linux 动态链接器只在进程启动时读取 ``LD_LIBRARY_PATH``，因此路径缺失时
    必须 re-exec 一次；这正是此前 V30 返回错误码 64 的根因。
    """
    marker = "HIKVISION_PREVIEW_LOADER_READY"
    paths = os.environ.get("LD_LIBRARY_PATH", "").split(":")
    if os.environ.get(marker) == "1" or str(SDK_DIR) in paths:
        return
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = f"{SDK_DIR}:{SDK_DIR / 'HCNetSDKCom'}" + (f":{environment['LD_LIBRARY_PATH']}" if environment.get("LD_LIBRARY_PATH") else "")
    environment[marker] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], environment)


def main() -> int:
    args = parse_args()
    ensure_player_library_path()
    print(f"预览: {args.host}:{args.port}，通道 {args.channel}，{'主' if args.stream == 0 else '子'}码流，V40 回调，链路模式 {args.link_mode}")
    if args.ir_mode != "keep":
        try: set_ir_mode(args); print(f"日夜模式已设为 {args.ir_mode}")
        except RuntimeError as error: print(f"错误: {error}"); return 1
    app = tk.Tk(); app.title(f"海康实时预览 - {args.host}"); app.geometry("960x540"); app.minsize(480, 270)
    ui_font = tkfont.Font(family="Noto Sans CJK SC", size=11)
    video = tk.Frame(app, background="black"); video.pack(fill=tk.BOTH, expand=True)
    bar = tk.Frame(app); bar.pack(fill=tk.X, pady=7)
    status = tk.StringVar(value="正在连接..."); tk.Label(bar, textvariable=status, font=ui_font).pack(side=tk.LEFT, padx=12)
    session = V40Preview(args); closing = False; done = threading.Event()

    def close() -> None:
        nonlocal closing
        if closing: return
        closing = True; app.withdraw()
        def release():
            try: session.close()
            finally: done.set()
        threading.Thread(target=release, daemon=True).start()
        def finish(count=0):
            if done.is_set() or count >= 30: app.destroy()
            else: app.after(50, lambda: finish(count + 1))
        app.after(50, finish)
    tk.Button(bar, text="退出 (Esc)", command=close, font=ui_font).pack(side=tk.RIGHT, padx=12)
    app.protocol("WM_DELETE_WINDOW", close); app.bind("<Escape>", lambda _event: close())

    def start() -> None:
        try:
            session.connect(); session.start(video.winfo_id()); status.set("已连接，等待码流..."); print(session.runtime_info())
        except RuntimeError as error:
            status.set(str(error)); print(f"错误: {error}"); close()
    def check_stream() -> None:
        if not closing:
            if session.stream_error: status.set(session.stream_error)
            app.after(200, check_stream)
    app.after(100, start); app.after(200, check_stream); app.mainloop()
    return 0


if __name__ == "__main__": raise SystemExit(main())
