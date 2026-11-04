#!/usr/bin/env python3
"""Shared USB-camera web service with YOLO and inspection evidence APIs.

The configured V4L2 source is the only owner of the ordinary USB camera.
Browsers and the YOLO child process consume its shared MJPEG output. The older
HCNetSDK sources and control APIs remain only as an explicit compatibility mode.
"""

from __future__ import annotations

import argparse
import base64
import copy
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import faulthandler
from functools import wraps
import importlib
import json
import os
import queue
import signal
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ElementTree
from ctypes import CDLL, RTLD_GLOBAL, CFUNCTYPE, POINTER, Structure, Union, byref, c_byte, c_char, c_char_p, c_int, c_uint, c_ubyte, c_ushort, c_void_p, create_string_buffer, sizeof
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse

from recognition_core import RecognitionDatabase
from recognition_runtime import (
    DEFAULT_EVIDENCE_DIR, DEFAULT_SFACE_MODEL, SFaceFeatureExtractor)
from thermal_client import ThermalHttpClient
from c12_thermal import C12ThermalClient
from web_preview import PreviewCache, PROFILES as PREVIEW_PROFILES
from ordinary_camera import (CameraSourceStore, HttpCameraSource, DEFAULT_BODY_URL,
                             validate_config, discover_usb_cameras, check_usb_device,
                             source_kind, source_label)
from behavior_project import BehaviorSupervisor
from behavior_speech import BehaviorSpeechService, PATROL_SAFETY_KEY
from c12_control import C12Control, COMMANDS as C12_COMMANDS
from c12_recording import C12Recorder
from c12_stream import C12Streams, C12OrdinarySource, VISIBLE_URL, INFRARED_URL


ROOT = Path(__file__).resolve().parent
PAGE_FILE = ROOT / "camera_control.html"
# 迁移包内所有资源均以当前脚本为基准，不依赖原主机目录。
DEFAULT_SDK_DIR = ROOT / "sdk"
DEFAULT_USB_CAMERA_DEVICE = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
DEFAULT_YOLO_SCRIPT = ROOT / "yolo_hk_voice.py"
DEFAULT_YOLO_MODEL = ROOT / "models" / "cigarette_multiclass_best.pt"
DEFAULT_YOLO_FACE_MODEL = ROOT / "models" / "face_detection_yunet_2023mar.onnx"
DEFAULT_YOLO_PLATE_MODEL = ROOT / "models" / "license_plate_best.pt"
DEFAULT_RECOGNITION_DB = Path.home() / ".ros" / "robot_navigation" / "recognition.db"
DEFAULT_YOLO_HOST = "127.0.0.1"
DEFAULT_YOLO_PORT = 8877
DEFAULT_YOLO_CONDA_ENV = "current"
# USB mode feeds YOLO from this process's loopback MJPEG endpoint.
DEFAULT_YOLO_CAPTURE_BACKEND = "mjpeg"
DEFAULT_YOLO_STREAM = 0
DEFAULT_YOLO_WEB_QUALITY = 95
DEFAULT_YOLO_STREAM_START_TIMEOUT = 30.0
DEFAULT_YOLO_VOICE_OUTPUT = "local"
DEFAULT_YOLO_SPEECH_AUDIO_DIR = Path(__file__).resolve().parent
NET_DVR_SYSHEAD = 1
NET_DVR_STREAMDATA = 2
NET_DVR_GET_PTZPOS = 293
NET_DVR_SET_PTZPOS = 292
NET_DVR_GET_ISP_CAMERAPARAMCFG = 3255
NET_DVR_SET_ISP_CAMERAPARAMCFG = 3256
NET_DVR_GET_WIPERINFO_CFG = 3259
NET_DVR_SET_WIPERINFO_CFG = 3260

# PTZ 常量来自 HCNetSDK.h；雨刮电源改走设备官方网页使用的 ISAPI PTZAux。
ZOOM_IN, ZOOM_OUT, FOCUS_NEAR, FOCUS_FAR, IRIS_OPEN, IRIS_CLOSE = 11, 12, 13, 14, 15, 16
TILT_UP, TILT_DOWN, PAN_LEFT, PAN_RIGHT = 21, 22, 23, 24
PTZ_ACTIONS = {
    "up": TILT_UP,
    "down": TILT_DOWN,
    "left": PAN_LEFT,
    "right": PAN_RIGHT,
    "zoom_in": ZOOM_IN,
    "zoom_out": ZOOM_OUT,
    "focus_near": FOCUS_NEAR,
    "focus_far": FOCUS_FAR,
    "iris_open": IRIS_OPEN,
    "iris_close": IRIS_CLOSE,
}
SDK_ERRORS = {
    1: "用户名或密码错误",
    2: "当前用户没有权限",
    5: "设备连接数已满",
    7: "连接设备服务失败，请检查 IP、8000 端口和本机网络",
    10: "接收设备响应超时",
    23: "设备或当前调用方式不支持",
    64: "Player SDK 加载失败，请检查动态库路径",
}
WIPER_MODE_NAMES = {0: "single", 1: "continuous", 2: "smart", 255: "off"}
WIPER_MODE_VALUES = {name: value for value, name in WIPER_MODE_NAMES.items()}


class LocalSdkPath(Structure):
    """HCNetSDK 的组件库搜索路径。"""

    _fields_ = [("sPath", c_char * 256), ("byRes", c_byte * 128)]


class DeviceInfoV30(Structure):
    """NET_DVR_DEVICEINFO_V30，避免旧版固定偏移解析。"""

    _fields_ = [
        ("sSerialNumber", c_byte * 48), ("byAlarmInPortNum", c_byte), ("byAlarmOutPortNum", c_byte),
        ("byDiskNum", c_byte), ("byDVRType", c_byte), ("byChanNum", c_byte), ("byStartChan", c_byte),
        ("byAudioChanNum", c_byte), ("byIPChanNum", c_byte), ("byZeroChanNum", c_byte),
        ("byMainProto", c_byte), ("bySubProto", c_byte), ("bySupport", c_byte), ("bySupport1", c_byte),
        ("bySupport2", c_byte), ("wDevType", c_ushort), ("bySupport3", c_byte), ("byMultiStreamProto", c_byte),
        ("byStartDChan", c_byte), ("byStartDTalkChan", c_byte), ("byHighDChanNum", c_byte),
        ("bySupport4", c_byte), ("byLanguageType", c_byte), ("byVoiceInChanNum", c_byte),
        ("byStartVoiceInChanNo", c_byte), ("bySupport5", c_byte), ("bySupport6", c_byte),
        # 镜像通道字段属于 ABI，不能只保留到 bySupport6；SDK 会写满 80 字节。
        ("byMirrorChanNum", c_byte), ("wStartMirrorChanNo", c_ushort),
        ("bySupport7", c_byte), ("byRes2", c_byte),
    ]


if sizeof(DeviceInfoV30) != 80:  # pragma: no cover - ABI 变更应立即暴露
    raise RuntimeError(f"NET_DVR_DEVICEINFO_V30 ABI 异常: {sizeof(DeviceInfoV30)} 字节，预期 80")


class PreviewInfo(Structure):
    """Linux ``NET_DVR_PREVIEWINFO``；hPlayWnd 是 32 位窗口句柄。"""

    _fields_ = [
        # HCNetSDK.h 在 Linux 中把 HWND 定义为 unsigned int，不能使用 c_void_p。
        ("lChannel", c_int), ("dwStreamType", c_uint), ("dwLinkMode", c_uint), ("hPlayWnd", c_uint),
        ("bBlocked", c_uint), ("bPassbackRecord", c_uint), ("byPreviewMode", c_ubyte),
        ("byStreamID", c_ubyte * 32), ("byProtoType", c_ubyte), ("byRes1", c_ubyte),
        ("byVideoCodingType", c_ubyte), ("dwDisplayBufNum", c_uint), ("byNPQMode", c_ubyte),
        ("byRecvMetaData", c_ubyte), ("byDataType", c_ubyte), ("byRes", c_ubyte * 213),
    ]


class PtzPosition(Structure):
    """NET_DVR_PTZPOS，用于上下运动限位保护。"""

    _fields_ = [("wAction", c_ushort), ("wPanPos", c_ushort), ("wTiltPos", c_ushort), ("wZoomPos", c_ushort)]


class VideoEffect(Structure):
    """NET_DVR_VIDEOEFFECT 的可调图像参数。"""

    _fields_ = [
        ("byBrightnessLevel", c_ubyte), ("byContrastLevel", c_ubyte), ("bySharpnessLevel", c_ubyte),
        ("bySaturationLevel", c_ubyte), ("byHueLevel", c_ubyte), ("byEnableFunc", c_ubyte),
        ("byLightInhibitLevel", c_ubyte), ("byGrayLevel", c_ubyte),
    ]


class Gain(Structure):
    """NET_DVR_GAIN。"""

    _fields_ = [("byGainLevel", c_ubyte), ("byGainUserSet", c_ubyte), ("byRes", c_ubyte * 2), ("dwMaxGainValue", c_uint)]


class WhiteBalance(Structure):
    """NET_DVR_WHITEBALANCE。"""

    _fields_ = [("byWhiteBalanceMode", c_ubyte), ("byWhiteBalanceModeRGain", c_ubyte), ("byWhiteBalanceModeBGain", c_ubyte), ("byRes", c_ubyte * 5)]


class Exposure(Structure):
    """NET_DVR_EXPOSURE；曝光时间单位为微秒。"""

    _fields_ = [("byExposureMode", c_ubyte), ("byAutoApertureLevel", c_ubyte), ("byRes", c_ubyte * 2), ("dwVideoExposureSet", c_uint), ("dwExposureUserSet", c_uint), ("dwRes", c_uint)]


class GammaCorrect(Structure):
    """NET_DVR_GAMMACORRECT。"""

    _fields_ = [("byGammaCorrectionEnabled", c_ubyte), ("byGammaCorrectionLevel", c_ubyte), ("byRes", c_ubyte * 6)]


class Wdr(Structure):
    """NET_DVR_WDR。"""

    _fields_ = [("byWDREnabled", c_ubyte), ("byWDRLevel1", c_ubyte), ("byWDRLevel2", c_ubyte), ("byWDRContrastLevel", c_ubyte), ("byRes", c_ubyte * 16)]


class DayNight(Structure):
    """NET_DVR_DAYNIGHT。网页日夜按钮优先走 ISAPI，保留该字段以正确读写完整相机结构。"""

    _fields_ = [
        ("byDayNightFilterType", c_ubyte), ("bySwitchScheduleEnabled", c_ubyte), ("byBeginTime", c_ubyte),
        ("byEndTime", c_ubyte), ("byDayToNightFilterLevel", c_ubyte), ("byNightToDayFilterLevel", c_ubyte),
        ("byDayNightFilterTime", c_ubyte), ("byBeginTimeMin", c_ubyte), ("byBeginTimeSec", c_ubyte),
        ("byEndTimeMin", c_ubyte), ("byEndTimeSec", c_ubyte), ("byAlarmTrigState", c_ubyte),
    ]


class Backlight(Structure):
    """NET_DVR_BACKLIGHT。"""

    _fields_ = [
        ("byBacklightMode", c_ubyte), ("byBacklightLevel", c_ubyte), ("byRes1", c_ubyte * 2),
        ("dwPositionX1", c_uint), ("dwPositionY1", c_uint), ("dwPositionX2", c_uint), ("dwPositionY2", c_uint),
        ("byRes2", c_ubyte * 4),
    ]


class NoiseRemove(Structure):
    """NET_DVR_NOISEREMOVE。"""

    _fields_ = [
        ("byDigitalNoiseRemoveEnable", c_ubyte), ("byDigitalNoiseRemoveLevel", c_ubyte),
        ("bySpectralLevel", c_ubyte), ("byTemporalLevel", c_ubyte),
        ("byDigitalNoiseRemove2DEnable", c_ubyte), ("byDigitalNoiseRemove2DLevel", c_ubyte), ("byRes", c_ubyte * 2),
    ]


class CmosModeCfg(Structure):
    """NET_DVR_CMOSMODECFG。"""

    _fields_ = [
        ("byCaptureMod", c_ubyte), ("byBrightnessGate", c_ubyte), ("byCaptureGain1", c_ubyte), ("byCaptureGain2", c_ubyte),
        ("dwCaptureShutterSpeed1", c_uint), ("dwCaptureShutterSpeed2", c_uint), ("byRes", c_ubyte * 4),
    ]


class CameraParamCfg(Structure):
    """NET_DVR_CAMERAPARAMCFG，完整保留未知字段以避免写配置时破坏设备参数。"""

    _fields_ = [
        ("dwSize", c_uint), ("struVideoEffect", VideoEffect), ("struGain", Gain),
        ("struWhiteBalance", WhiteBalance), ("struExposure", Exposure), ("struGammaCorrect", GammaCorrect),
        ("struWdr", Wdr), ("struDayNight", DayNight), ("struBackLight", Backlight),
        ("struNoiseRemove", NoiseRemove), ("byPowerLineFrequencyMode", c_ubyte), ("byIrisMode", c_ubyte),
        ("byMirror", c_ubyte), ("byDigitalZoom", c_ubyte), ("byDeadPixelDetect", c_ubyte),
        ("byBlackPwl", c_ubyte), ("byEptzGate", c_ubyte), ("byLocalOutputGate", c_ubyte),
        ("byCoderOutputMode", c_ubyte), ("byLineCoding", c_ubyte), ("byDimmerMode", c_ubyte),
        ("byPaletteMode", c_ubyte), ("byEnhancedMode", c_ubyte), ("byDynamicContrastEN", c_ubyte),
        ("byDynamicContrast", c_ubyte), ("byJPEGQuality", c_ubyte), ("struCmosModeCfg", CmosModeCfg),
        ("byFilterSwitch", c_ubyte), ("byFocusSpeed", c_ubyte), ("byAutoCompensationInterval", c_ubyte),
        ("bySceneMode", c_ubyte),
    ]


class WiperContinueWorkParam(Structure):
    """NET_DVR_WIPER_CONTINUEWORK_PARAM；持续时长单位为秒。"""

    _fields_ = [("byWorkTimeInterval", c_ubyte), ("byRes", c_ubyte * 3), ("dwContinueWorkTime", c_uint), ("byRes1", c_ubyte * 8)]


class WiperModeParam(Union):
    """NET_DVR_WIPERMODE_PARAM_UNION。"""

    _fields_ = [("byRaw", c_ubyte * 16), ("struContinue", WiperContinueWorkParam)]


class WiperInfoCfg(Structure):
    """NET_DVR_WIPERINFO_CFG；默认 ABI 大小必须为 44 字节。"""

    _fields_ = [
        ("dwSize", c_uint), ("byWiperWorkMode", c_ubyte), ("byRes1", c_ubyte),
        ("wSensitivity", c_ushort), ("byRes", c_ubyte * 20), ("ustruWiperModeParam", WiperModeParam),
    ]


class JpegPara(Structure):
    """JPEG 回退模式的 NET_DVR_JPEGPARA。"""

    _fields_ = [("wPicSize", c_ushort), ("wPicQuality", c_ushort)]


RealDataCallback = CFUNCTYPE(None, c_int, c_uint, POINTER(c_ubyte), c_uint, c_void_p)


class IsapiRequestError(RuntimeError):
    """携带 HTTP 状态的 ISAPI 错误，供网页给出可操作的设备反馈。"""

    def __init__(self, message: str, *, status: int | None = None, response: str = "") -> None:
        super().__init__(message)
        self.status = status
        self.response = response


class WiperConfigUnsupportedError(RuntimeError):
    """设备明确不支持 NET_DVR_WIPERINFO_CFG 时使用的可恢复错误。"""


class LatestJpegFrame:
    """向全部 MJPEG 客户端发布最新图像，不保留过期画面。"""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._version = 0
        self._image: bytes | None = None
        self._timestamp = 0.0
        self._stamp_ns = 0
        self._raw_frame = None
        self._error = ""

    def publish(self, image: bytes, *, stamp_ns=None, raw_frame=None) -> None:
        """原子替换上一帧并唤醒所有等待客户端。"""
        with self._condition:
            self._version += 1
            self._image = image
            self._timestamp = time.monotonic()
            self._stamp_ns = stamp_ns if stamp_ns is not None else time.monotonic_ns()
            self._raw_frame = raw_frame
            self._error = ""
            self._condition.notify_all()

    def set_error(self, message: str) -> None:
        """记录视频错误，使等待中的 HTTP 流能够尽快退出。"""
        with self._condition:
            self._error = message
            self._condition.notify_all()

    def invalidate(self, message: str) -> None:
        """Drop old-source frames on a switch, stop, or loss of the body stream."""
        with self._condition:
            self._image = None
            self._raw_frame = None
            self._version += 1
            self._error = message
            self._condition.notify_all()

    def next_after(self, version: int, timeout: float) -> tuple[int, bytes]:
        """等待比指定版本更新的帧，超时给出明确视频状态。"""
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._version == version:
                if self._error:
                    raise RuntimeError(self._error)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("等待视频帧超时")
                self._condition.wait(remaining)
            if self._image is None:
                raise RuntimeError(self._error or "视频帧为空")
            return self._version, self._image

    def latest(self, timeout: float) -> bytes:
        """取得当前最新帧；首次视频启动时会等待第一帧。"""
        _, image = self.next_after(0, timeout)
        return image

    def behavior_packet(self, version, timeout=.5):
        """Read one unique frame with its capture/read timestamp atomically."""
        self.next_after(version, timeout)
        with self._condition:
            if self._error or self._image is None:
                raise RuntimeError(self._error or '视频帧为空')
            return self._version, self._image, self._stamp_ns, self._raw_frame


class _SdkTask:
    """命令线程中的一个可取消同步任务。"""

    def __init__(self, function: Callable[..., Any], args: tuple[Any, ...], background: bool) -> None:
        self.function = function
        self.args = args
        self.background = background
        self.done = threading.Event()
        self.cancelled = threading.Event()
        self.result: Any = None
        self.error: BaseException | None = None


class SdkCommandExecutor:
    """让同一登录会话的控制和配置调用严格串行，避免 SDK 重入卡死。

    普通配置请求有固定排队上限；紧急停止使用独立队列，始终排在普通请求前。
    这样滑块、抓图等请求拥塞时，松键和看门狗仍可在当前 SDK 调用结束后立即执行。
    """

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._normal_tasks: deque[_SdkTask] = deque()
        self._urgent_tasks: deque[_SdkTask] = deque()
        self._max_normal_tasks = 64
        self._accepting = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="hik-sdk-command")
        self._thread.start()

    def _take_task(self) -> _SdkTask | None:
        """优先取得安全停止任务；关闭时取消未执行的普通任务。"""
        with self._condition:
            while self._accepting and not self._urgent_tasks and not self._normal_tasks:
                self._condition.wait()
            if self._urgent_tasks:
                return self._urgent_tasks.popleft()
            if self._normal_tasks:
                return self._normal_tasks.popleft()
            return None

    def _run(self) -> None:
        while True:
            task = self._take_task()
            if task is None:
                return
            if task.cancelled.is_set():
                task.done.set()
                continue
            try:
                task.result = task.function(*task.args)
            except BaseException as error:  # 向 HTTP 层保留 SDK 原始错误说明。
                task.error = error
                if task.background:
                    print(f"[SDK] 后台命令失败: {error}")
            finally:
                task.done.set()

    def _enqueue(self, task: _SdkTask, *, urgent: bool) -> bool:
        """将任务放入对应队列；紧急任务不受普通请求容量限制。"""
        with self._condition:
            if not self._accepting:
                return False
            if urgent:
                self._urgent_tasks.append(task)
            elif len(self._normal_tasks) >= self._max_normal_tasks:
                return False
            else:
                self._normal_tasks.append(task)
            self._condition.notify()
            return True

    def call(self, function: Callable[..., Any], *args: Any, timeout: float = 8.0, urgent: bool = False) -> Any:
        """排队并等待命令完成；排队超时的任务不会在稍后意外启动电机。"""
        if threading.current_thread() is self._thread:
            return function(*args)
        task = _SdkTask(function, args, False)
        if not self._enqueue(task, urgent=urgent):
            if self._accepting:
                raise RuntimeError("相机控制队列繁忙，请稍后重试")
            raise RuntimeError("SDK 命令服务正在关闭")
        if not task.done.wait(timeout):
            task.cancelled.set()
            raise RuntimeError(f"相机命令超时（{timeout:.1f} 秒）")
        if task.error is not None:
            raise task.error
        return task.result

    def submit(self, function: Callable[..., Any], *args: Any) -> bool:
        """提交普通后台任务。"""
        return self._enqueue(_SdkTask(function, args, True), urgent=False)

    def submit_urgent(self, function: Callable[..., Any], *args: Any) -> bool:
        """提交紧急后台任务，供无 HTTP 等待方的安全停止使用。"""
        return self._enqueue(_SdkTask(function, args, True), urgent=True)

    def in_worker_thread(self) -> bool:
        """供回退视频源避免在 SDK 命令线程中等待自身的抓图线程。"""
        return threading.current_thread() is self._thread

    def close(self) -> None:
        """停止接受新任务；SDK 卡住或队列满时也不得阻塞退出。"""
        with self._condition:
            if not self._accepting:
                return
            self._accepting = False
            cancelled = [*self._normal_tasks, *self._urgent_tasks]
            self._normal_tasks.clear()
            self._urgent_tasks.clear()
            for task in cancelled:
                task.cancelled.set()
                task.error = RuntimeError("SDK 命令服务正在关闭")
                task.done.set()
            self._condition.notify_all()
        self._thread.join(timeout=3)


class V40MjpegSource:
    """V40 码流经 PlayM4 解码，由后台轮询最新 JPEG 的实验性视频源。"""

    def __init__(self, sdk: Any, user_id: int, args: argparse.Namespace) -> None:
        self.sdk = sdk
        self.user_id = user_id
        self.args = args
        self.player: Any = None
        self.player_port = c_int(-1)
        self.real_handle = -1
        self.frames = LatestJpegFrame()
        # 4K JPEG 通常远小于 16 MiB；固定缓冲区避免在取帧线程反复分配大对象。
        self._jpeg_buffer = create_string_buffer(16 * 1024 * 1024)
        self.jpeg_misses = 0
        self.encoded_frames = 0
        self._closing = threading.Event()
        self._closed = threading.Event()
        self._close_lock = threading.Lock()
        self._player_lock = threading.RLock()
        self._callback_condition = threading.Condition()
        self._callbacks_in_flight = 0
        self._encoder: threading.Thread | None = None
        # 实时数据回调对象必须保存为实例属性，避免 ctypes 垃圾回收后崩溃。
        self.real_callback = RealDataCallback(self._on_real_data)
        self._stream_opened = False
        self._stream_ready = threading.Event()
        self._stream_error = ""

    def _player_error(self, prefix: str) -> str:
        code = int(self.player.PlayM4_GetLastError(self.player_port.value))
        return f"{prefix}: {code}"

    def _configure_player(self) -> None:
        self.player = CDLL(str(self.args.sdk_dir / "libPlayCtrl.so"), mode=RTLD_GLOBAL)
        self.player.PlayM4_GetPort.argtypes = [POINTER(c_int)]
        self.player.PlayM4_GetPort.restype = c_int
        self.player.PlayM4_SetStreamOpenMode.argtypes = [c_int, c_uint]
        self.player.PlayM4_SetStreamOpenMode.restype = c_int
        self.player.PlayM4_OpenStream.argtypes = [c_int, c_void_p, c_uint, c_uint]
        self.player.PlayM4_OpenStream.restype = c_int
        self.player.PlayM4_Play.argtypes = [c_int, c_void_p]
        self.player.PlayM4_Play.restype = c_int
        self.player.PlayM4_InputData.argtypes = [c_int, c_void_p, c_uint]
        self.player.PlayM4_InputData.restype = c_int
        # 与 SDK Java Demo 一致：由主动调用方取得最新 JPEG，避免解码线程回调 Python。
        self.player.PlayM4_GetJPEG.argtypes = [c_int, c_void_p, c_uint, POINTER(c_uint)]
        self.player.PlayM4_GetJPEG.restype = c_int
        self.player.PlayM4_Stop.argtypes = [c_int]
        self.player.PlayM4_Stop.restype = c_int
        self.player.PlayM4_CloseStream.argtypes = [c_int]
        self.player.PlayM4_CloseStream.restype = c_int
        self.player.PlayM4_FreePort.argtypes = [c_int]
        self.player.PlayM4_FreePort.restype = c_int
        self.player.PlayM4_GetLastError.argtypes = [c_int]
        self.player.PlayM4_GetLastError.restype = c_uint
        if not self.player.PlayM4_GetPort(byref(self.player_port)):
            raise RuntimeError("PlayM4 无可用播放端口")

    def _enter_callback(self) -> bool:
        """登记在途 SDK/PlayM4 回调；关闭开始后拒绝访问可能已失效的缓冲区。"""
        with self._callback_condition:
            if self._closing.is_set():
                return False
            self._callbacks_in_flight += 1
            return True

    def _leave_callback(self) -> None:
        """释放回调登记并唤醒等待安全释放播放器端口的关闭线程。"""
        with self._callback_condition:
            self._callbacks_in_flight -= 1
            self._callback_condition.notify_all()

    def _wait_for_callbacks(self, timeout: float) -> bool:
        """等待所有已经进入 Python 的回调退出；超时后由调用方保留端口避免 UAF。"""
        deadline = time.monotonic() + timeout
        with self._callback_condition:
            while self._callbacks_in_flight:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._callback_condition.wait(remaining)
        return True

    def start(self) -> None:
        """启动 V40 取流；第一帧由 ``wait_for_first_frame`` 在主线程确认。"""
        self._configure_player()
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="hik-v40-jpeg")
        self._encoder.start()
        info = PreviewInfo()
        info.lChannel = self.args.channel
        info.dwStreamType = self.args.stream
        info.dwLinkMode = self.args.link_mode
        info.hPlayWnd = 0
        info.bBlocked = 0
        info.dwDisplayBufNum = 1
        self.real_handle = self.sdk.NET_DVR_RealPlay_V40(self.user_id, byref(info), self.real_callback, None)
        if self.real_handle < 0:
            raise RuntimeError(f"V40 预览失败: {int(self.sdk.NET_DVR_GetLastError())}")

    def _on_real_data(self, _handle: int, data_type: int, buffer, size: int, _user: object) -> None:
        """实时回调只做播放器缓冲转发，不做 JPEG 编码和 HTTP 操作。"""
        if not self._enter_callback():
            return
        try:
            with self._player_lock:
                # close() 开始后不再向播放器输入数据；在途回调由关闭流程等待其退出。
                if self._closing.is_set():
                    return
                if data_type == NET_DVR_SYSHEAD and not self._stream_opened:
                    if not self.player.PlayM4_SetStreamOpenMode(self.player_port.value, 0):
                        self._stream_error = self._player_error("设置播放模式失败")
                    elif not self.player.PlayM4_OpenStream(self.player_port.value, buffer, size, 8 * 1024 * 1024):
                        self._stream_error = self._player_error("打开码流失败")
                    elif not self.player.PlayM4_Play(self.player_port.value, None):
                        self._stream_error = self._player_error("启动解码失败")
                    else:
                        self._stream_opened = True
                        self._stream_ready.set()
                elif data_type == NET_DVR_STREAMDATA and self._stream_opened:
                    if not self.player.PlayM4_InputData(self.player_port.value, buffer, size):
                        # 码流缓冲临时满时丢弃当前包，等待后续关键帧恢复，而非阻塞 SDK 回调。
                        self.jpeg_misses += 1
        except BaseException as error:
            self._stream_error = f"PlayM4 回调异常: {error}"
        finally:
            self._leave_callback()
        if self._stream_error:
            self.frames.set_error(self._stream_error)

    def _encode_loop(self) -> None:
        """轮询 PlayM4 最新 JPEG，避免解码线程以 ctypes 回调方式进入 Python。"""
        interval = 1.0 / self.args.max_fps if self.args.max_fps > 0 else 0.01
        while not self._closing.is_set():
            if not self._stream_ready.wait(timeout=0.1):
                continue
            captured_size = c_uint(0)
            image: bytes | None = None
            try:
                with self._player_lock:
                    if self._closing.is_set() or self.player_port.value < 0:
                        break
                    if self.player.PlayM4_GetJPEG(
                        self.player_port.value,
                        self._jpeg_buffer,
                        sizeof(self._jpeg_buffer),
                        byref(captured_size),
                    ):
                        if 0 < captured_size.value <= sizeof(self._jpeg_buffer):
                            image = self._jpeg_buffer.raw[: captured_size.value]
                        else:
                            self.jpeg_misses += 1
                    else:
                        # 解码器尚未产生完整帧时会返回失败；这不是致命错误，等待下一轮即可。
                        self.jpeg_misses += 1
            except BaseException as error:
                self._stream_error = f"PlayM4 JPEG 取帧异常: {error}"
                self.frames.set_error(self._stream_error)
                return
            if image is not None:
                self.frames.publish(image)
                self.encoded_frames += 1
            self._closing.wait(interval)

    def wait_for_first_frame(self, timeout: float) -> None:
        """确认 V40 已实际输出浏览器可显示的 JPEG，而不仅是成功返回句柄。"""
        self.frames.latest(timeout)

    def next_frame(self, version: int, timeout: float = 15.0) -> tuple[int, bytes]:
        """供 HTTP MJPEG 客户端等待下一张共享画面。"""
        return self.frames.next_after(version, timeout)

    def latest_frame(self, timeout: float = 10.0) -> bytes:
        """供快照接口读取已有画面。"""
        return self.frames.latest(timeout)

    def info(self) -> dict[str, Any]:
        """返回不触发 SDK 调用的视频健康指标。"""
        return {"backend": "v40", "encoded_frames": self.encoded_frames, "dropped_frames": self.jpeg_misses, "error": self._stream_error or None}

    def close(self) -> None:
        """停止取流、播放器与编码线程；由 SDK 命令线程调用。"""
        with self._close_lock:
            if self._closed.is_set():
                return
            # 与回调登记使用同一条件锁，保证关闭标记与“是否允许进入回调”是原子的。
            with self._callback_condition:
                self._closing.set()
            # 唤醒持有旧视频源的 MJPEG 客户端，避免重连后还等待旧帧 15 秒。
            self.frames.set_error("视频源已停止")
            handle = self.real_handle
            self.real_handle = -1
            try:
                # StopRealPlay 可能等待实时回调返回，因此不能持有播放器锁调用它。
                if handle >= 0:
                    self.sdk.NET_DVR_StopRealPlay(handle)
            finally:
                # StopRealPlay 不能保证已排队的实时数据回调已经离开 Python。先等待回调，
                # 再等待主动调用 PlayM4_GetJPEG 的取帧线程退出，二者都完成后才可释放端口。
                callbacks_stopped = self._wait_for_callbacks(timeout=5.0)
                if self._encoder is not None:
                    self._encoder.join(timeout=3.0)
                encoder_stopped = self._encoder is None or not self._encoder.is_alive()
                if callbacks_stopped and encoder_stopped:
                    with self._player_lock:
                        if self.player is not None and self.player_port.value >= 0:
                            self.player.PlayM4_Stop(self.player_port.value)
                            self.player.PlayM4_CloseStream(self.player_port.value)
                            self.player.PlayM4_FreePort(self.player_port.value)
                            self.player_port = c_int(-1)
                else:
                    pending = "实时回调" if not callbacks_stopped else "JPEG 取帧线程"
                    self._stream_error = f"等待 {pending} 退出超时，已保留旧播放端口以避免崩溃"
                    self.frames.set_error(self._stream_error)
                self._closed.set()


class V4L2MjpegSource:
    """Read one Linux V4L2 camera and publish a shared latest-JPEG stream.

    The source is the only owner of ``/dev/video*``. Browser clients and the
    YOLO child consume the HTTP MJPEG stream, so they never compete for the
    physical camera handle.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.frames = LatestJpegFrame()
        self._stop = threading.Event()
        self._capture_lock = threading.Lock()
        self._capture: Any = None
        self._worker: threading.Thread | None = None
        self.encoded_frames = 0
        self.dropped_frames = 0
        self.reconnects = 0
        self.error = ""
        self.actual_width = 0
        self.actual_height = 0
        self.actual_fps = 0.0
        self.actual_fourcc = ""

    @staticmethod
    def _fourcc_text(value: float) -> str:
        encoded = int(value)
        return "".join(chr((encoded >> (8 * index)) & 0xFF) for index in range(4)).strip("\x00")

    def _open_capture(self) -> Any:
        cv2 = importlib.import_module("cv2")
        device = str(self.args.usb_device)
        check_usb_device(device)
        suffix = device.removeprefix("/dev/video")
        capture_input: str | int = int(suffix) if suffix.isdigit() else device
        capture = cv2.VideoCapture(capture_input, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"无法打开 USB 摄像头 {self.args.usb_device}")
        fourcc = str(self.args.usb_fourcc).upper()[:4].ljust(4)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.args.usb_width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.args.usb_height))
        capture.set(cv2.CAP_PROP_FPS, float(self.args.usb_fps))
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        self.actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        self.actual_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
        self.actual_fourcc = self._fourcc_text(capture.get(cv2.CAP_PROP_FOURCC))
        return capture

    def _replace_capture(self, capture: Any | None) -> None:
        with self._capture_lock:
            previous = self._capture
            self._capture = capture
        if previous is not None and previous is not capture:
            try:
                previous.release()
            except BaseException:
                pass

    def start(self) -> None:
        self._worker = threading.Thread(target=self._loop, daemon=True, name="usb-v4l2-camera")
        self._worker.start()

    def _loop(self) -> None:
        cv2 = importlib.import_module("cv2")
        failure_count = 0
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                with self._capture_lock:
                    capture = self._capture
                try:
                    if capture is None:
                        capture = self._open_capture()
                        self._replace_capture(capture)
                        self.reconnects += 1
                    ok, frame = capture.read()
                    capture_stamp_ns = time.monotonic_ns()
                    if self._stop.is_set(): break
                    if not ok or frame is None:
                        raise RuntimeError("USB 摄像头读取失败")
                    ok, encoded = cv2.imencode(
                        ".jpg", frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)],
                    )
                    if not ok:
                        raise RuntimeError("USB 摄像头 JPEG 编码失败")
                    self.frames.publish(encoded.tobytes(), stamp_ns=capture_stamp_ns, raw_frame=frame)
                    self.encoded_frames += 1
                    self.error = ""
                    failure_count = 0
                except BaseException as error:
                    failure_count += 1
                    self.dropped_frames += 1
                    self.error = str(error)
                    self._replace_capture(None)
                    self.frames.invalidate(f"USB 摄像头暂时不可用: {error}")
                    self._stop.wait(min(5.0, max(0.5, float(failure_count))))
                    continue
                if self.args.max_fps > 0:
                    wait_seconds = max(0.0, 1.0 / self.args.max_fps - (time.monotonic() - started))
                    self._stop.wait(wait_seconds)
        finally:
            self._replace_capture(None)

    def wait_for_first_frame(self, timeout: float) -> None:
        self.frames.latest(timeout)

    def next_frame(self, version: int, timeout: float = 15.0) -> tuple[int, bytes]:
        return self.frames.next_after(version, timeout)

    def latest_frame(self, timeout: float = 10.0) -> bytes:
        return self.frames.latest(timeout)

    def info(self) -> dict[str, Any]:
        return {
            "backend": "v4l2",
            "device": str(self.args.usb_device),
            "width": self.actual_width,
            "height": self.actual_height,
            "fps": round(self.actual_fps, 2),
            "fourcc": self.actual_fourcc or str(self.args.usb_fourcc).upper(),
            "encoded_frames": self.encoded_frames,
            "dropped_frames": self.dropped_frames,
            "reconnects": self.reconnects,
            "error": self.error or None,
        }

    def close(self) -> None:
        self._stop.set()
        self.frames.invalidate("USB 摄像头视频源已停止")
        if self._worker is not None:
            self._worker.join(timeout=2.0)
            if self._worker.is_alive():
                raise RuntimeError("USB 读取线程尚未退出，未启动第二个来源；请稍后重试")
        self._replace_capture(None)


class JpegFallbackSource:
    """通过 HCNetSDK 抓图获取稳定 JPEG 的默认视频源。"""

    def __init__(self, gateway: "CameraGateway") -> None:
        self.gateway = gateway
        self.frames = LatestJpegFrame()
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._loop, daemon=True, name="hik-jpeg-fallback")
        self.captured_frames = 0
        self.error = ""

    def start(self) -> None:
        """启动唯一抓图线程；多浏览器连接不会增加 SDK 抓图频率。"""
        self._worker.start()

    def _loop(self) -> None:
        failures = 0
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                image = self.gateway.executor.call(self.gateway._capture_jpeg_locked, timeout=6)
                self.frames.publish(image)
                self.captured_frames += 1
                self.error = ""
                failures = 0
            except BaseException as error:
                failures += 1
                self.error = str(error)
                # 首帧阶段允许短暂网络或设备忙重试，避免一次抓图失败导致网页直接退出。
                if failures >= 3 and self.captured_frames > 0:
                    self.frames.set_error(f"JPEG 回退抓图失败: {error}")
                self._stop.wait(0.5)
                continue
            if self.gateway.args.max_fps > 0:
                interval = max(0.0, 1.0 / self.gateway.args.max_fps - (time.monotonic() - started))
                self._stop.wait(interval)

    def wait_for_first_frame(self, timeout: float) -> None:
        """确认抓图回退也取得了一帧实际 JPEG。"""
        self.frames.latest(timeout)

    def next_frame(self, version: int, timeout: float = 15.0) -> tuple[int, bytes]:
        return self.frames.next_after(version, timeout)

    def latest_frame(self, timeout: float = 10.0) -> bytes:
        return self.frames.latest(timeout)

    def info(self) -> dict[str, Any]:
        return {"backend": "jpeg", "encoded_frames": self.captured_frames, "dropped_frames": 0, "error": self.error or None}

    def close(self) -> None:
        self._stop.set()
        self.frames.set_error("JPEG 回退视频源已停止")
        # 当前方法由 SDK 命令线程调用时，抓图线程可能正在等待该命令线程，不能互相 join。
        if not self.gateway.executor.in_worker_thread():
            self._worker.join(timeout=1.5)


def _xml_text(payload: bytes, element_name: str) -> str | None:
    """按 XML 本地名称取值，兼容设备响应中的默认命名空间。"""
    try:
        root = ElementTree.fromstring(payload)
    except ElementTree.ParseError:
        return None
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] == element_name:
            value = (element.text or "").strip()
            return value or None
    return None


def _isapi_response_detail(payload: bytes) -> str:
    """从 ResponseStatus 提取简短诊断，避免将整段设备 XML 回显到网页。"""
    fields = ("statusCode", "subStatusCode", "statusString", "errorCode")
    details = [f"{field}={value}" for field in fields if (value := _xml_text(payload, field))]
    if details:
        return "; ".join(details)
    text = payload.decode("utf-8", errors="replace").strip().replace("\n", " ")
    return text[:300]


def _isapi_request(args: argparse.Namespace, path: str, *, method: str = "GET", payload: bytes | None = None) -> bytes:
    """使用标准 Digest 客户端直连设备，显式绕过桌面环境的 HTTP 代理。"""
    url = f"{args.http_scheme}://{args.host}:{args.http_port}{path}"
    manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    manager.add_password(None, url, args.username, args.password)
    # urllib 不识别 NO_PROXY 中的 CIDR。相机是局域网设备，必须避免代理改变 Digest 的 request-uri。
    handlers: list[Any] = [urllib.request.ProxyHandler({}), urllib.request.HTTPDigestAuthHandler(manager)]
    if args.http_scheme == "https":
        context = ssl._create_unverified_context() if args.insecure_tls else ssl.create_default_context()
        handlers.append(urllib.request.HTTPSHandler(context=context))
    headers = {"Accept": "application/xml"}
    if payload is not None:
        headers["Content-Type"] = "application/xml; charset=UTF-8"
    request = urllib.request.Request(url, data=payload, method=method, headers=headers)
    opener = urllib.request.build_opener(*handlers)
    try:
        with opener.open(request, timeout=6) as response:
            return response.read(64 * 1024)
    except urllib.error.HTTPError as error:
        body = error.read(4 * 1024)
        detail = _isapi_response_detail(body)
        suffix = f"（{detail}）" if detail else ""
        raise IsapiRequestError(f"ISAPI {method} {path} 失败: HTTP {error.code}{suffix}", status=error.code, response=detail) from error
    except urllib.error.URLError as error:
        raise IsapiRequestError(f"ISAPI {method} {path} 请求失败: {error.reason}") from error


def set_wiper_aux_power(args: argparse.Namespace, enabled: bool) -> None:
    """按设备官方网页的 PTZAux 协议开关雨刮，不读取没有 GET 的资源。"""
    path = f"/ISAPI/PTZCtrl/channels/{args.channel}/auxcontrols/{args.wiper_id}"
    status = "on" if enabled else "off"
    payload = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f"<PTZAux><id>{args.wiper_id}</id><type>WIPER</type><status>{status}</status></PTZAux>"
    ).encode("utf-8")
    _isapi_request(args, path, method="PUT", payload=payload)


def set_ir_mode(args: argparse.Namespace, mode: str) -> None:
    """用直连 Digest 设置当前通道日夜模式，保留设备已有切换阈值并复读验证。"""
    path = f"/ISAPI/Image/channels/{args.channel}/IrcutFilter"
    current = _isapi_request(args, path)
    level = _xml_text(current, "nightToDayFilterLevel")
    try:
        threshold = int(level) if level is not None else 2
    except ValueError:
        threshold = 2
    # 配置范围由设备决定；只保留一个合理的已读值，非法响应退回协议示例值 2。
    threshold = threshold if 0 <= threshold <= 255 else 2
    payload = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<IrcutFilter version="2.0" xmlns="http://www.std-cgi.com/ver20/XMLSchema">'
        f"<IrcutFilterType>{mode}</IrcutFilterType><nightToDayFilterLevel>{threshold}</nightToDayFilterLevel>"
        "</IrcutFilter>"
    ).encode("utf-8")
    _isapi_request(args, path, method="PUT", payload=payload)
    confirmed = _isapi_request(args, path)
    if _xml_text(confirmed, "IrcutFilterType") != mode:
        raise RuntimeError("设备未确认日夜模式设置")


def ordinary_camera_operation(method):
    """Serialize visual evidence/recognition changes with source transactions."""
    @wraps(method)
    def guarded(self, *args, **kwargs):
        with self._camera_change_lock:
            return method(self, *args, **kwargs)
    return guarded


class CameraGateway:
    """Manage a shared camera stream and optional legacy PTZ controls."""

    def __init__(self, args: argparse.Namespace, *, ordinary_source_factory=None) -> None:
        self.args = args
        self.c12_control = C12Control()
        self.c12 = C12Streams(LatestJpegFrame,
            getattr(args, 'c12_visible_url', VISIBLE_URL),
            getattr(args, 'c12_infrared_url', INFRARED_URL))
        self.c12_recorder = C12Recorder(self.c12.sources['visible'].url, self.c12.sources['infrared'].url)
        self._camera_change_lock = threading.RLock()
        self._ordinary_source_factory = ordinary_source_factory
        self._camera_source_generation = 0
        self._camera_source_error = ''
        self._ordinary_pending_close = None
        self._camera_sources = CameraSourceStore(
            getattr(args, 'camera_source_config', None),
            defaults={key:getattr(args,key,default) for key,default in (
                ('usb_device',DEFAULT_USB_CAMERA_DEVICE), ('usb_width',1280),
                ('usb_height',720), ('usb_fps',30.0),
                ('body_url',getattr(args,'body_camera_url',DEFAULT_BODY_URL)))},
            forbidden_ports=tuple({8767, getattr(args,'web_port',8766),
                                   getattr(args,'yolo_port',8877)}))
        self.sdk: Any = None
        self.user_id = -1
        self.device_info = DeviceInfoV30()
        self.executor = SdkCommandExecutor()
        self.video: V4L2MjpegSource | V40MjpegSource | JpegFallbackSource | None = None
        self._yolo = YoloProcessManager(args)
        self._thermal = ThermalHttpClient(
            getattr(args, "thermal_url", "http://127.0.0.1:8767"),
            getattr(args, "thermal_timeout", 3.0))
        self._c12_thermal = C12ThermalClient(self.c12.sources['infrared'],
            (self._camera_sources.path or DEFAULT_RECOGNITION_DB).with_name('c12_thermal.json'))
        self._thermal_event_sequences = {'legacy': 0, 'c12': 0}
        self._recognition_lock = threading.RLock()
        self._recognition_database: RecognitionDatabase | None = None
        self._recognition_face_extractor: SFaceFeatureExtractor | None = None
        self._recognition_face_detector: Any | None = None
        # 读视频源只持有短锁；重连使用单独互斥锁，避免两个浏览器交叉关闭对方刚启动的源。
        self._video_lock = threading.RLock()
        # 切换日夜模式时视频源会短暂为空。让新 MJPEG 请求等待重建完成，不能直接返回
        # “视频服务未启动”，否则浏览器可能在首帧回来前放弃本次重连。
        self._video_ready = threading.Condition(self._video_lock)
        self._video_restarting = threading.Event()
        self._video_restart_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._http_lock = threading.Lock()
        # ISAPI 日夜切换与 SDK ISP 配置都会写同一相机参数域，必须互斥。
        self._camera_config_lock = threading.Lock()
        self._active_action: str | None = None
        self._active_command: int | None = None
        self._active_speed = args.speed
        self._active_token = ""
        self._motion_deadline = 0.0
        self._watchdog_pending = False
        # 松键可能比启动请求更早到达命令线程；缓存已取消 token 防止稍后的启动误转。
        self._cancelled_motion_tokens: dict[str, float] = {}
        self._wiper_on = False
        self._last_tilt: float | None = None
        self._last_pan: float | None = None
        self._last_zoom_ratio: float | None = None
        self._front_pan_raw: int | None = None
        self._inspection_direction = "current"
        self._inspection_angle = 0.0
        self._inspection_position_estimated = False
        self._inspection_mode = "manual"
        self._inspection_task_id = ""
        self._last_ir_mode = "keep"
        self._image_config: CameraParamCfg | None = None
        self._wiper_config: WiperInfoCfg | None = None
        self._wiper_capability_cache: dict[str, Any] | None = None
        self._wiper_config_supported: bool | None = None
        self._wiper_config_reason = ""
        # 部分播放器在 StopRealPlay 返回后仍可能完成一个已入队回调。保留回调闭包直到
        # NET_DVR_Cleanup，避免 ctypes 回调对象被回收后跳入无效地址。
        self._retired_v40_sources: list[V40MjpegSource] = []
        self._closed = threading.Event()
        self._position_worker: threading.Thread | None = None
        # JPEG 回退模式只被命令线程使用，因而固定缓冲区不会发生并发写入。
        self._jpeg_buffer = create_string_buffer(16 * 1024 * 1024)
        self._behavior_speech = BehaviorSpeechService()
        self._behavior = BehaviorSupervisor(
            runtime_dir=getattr(args, 'behavior_runtime_dir', ROOT.parent / 'person-behavior-recognition'),
            python=getattr(args, 'behavior_python', None),
            config_path=getattr(args, 'behavior_config', Path.home() / '.ros/robot_navigation/behavior.json'),
            speech=self._behavior_speech.announce,
            log=lambda message: print('[行为识别] ' + message, flush=True))
        self._behavior_feeder = threading.Thread(
            target=self._behavior_feed_loop, name='behavior-shared-camera', daemon=True)
        self._behavior_feeder.start()

    def _behavior_feed_loop(self):
        source, version = None, 0
        while not self._closed.is_set():
            current = self._video_source()
            if current is not source:
                source, version = current, 0
                self._behavior.reset_source()
                self._behavior_speech.reset()
            if source is None or not self._behavior.status()['running']:
                self._closed.wait(.15)
                continue
            try:
                # All production camera sources share LatestJpegFrame. Never
                # open another device or re-stamp a cached frame at consumption.
                version, jpeg, stamp, frame = source.frames.behavior_packet(version, .5)
                if source is not self._video_source(): continue
                if frame is None:
                    cv2, np = importlib.import_module('cv2'), importlib.import_module('numpy')
                    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    self._behavior.submit(frame, stamp_ns=stamp,
                        source_key=(id(source), self._camera_source_generation), raw_jpeg=jpeg)
            except TimeoutError:
                pass
            except (ValueError, RuntimeError, OSError, AttributeError):
                self._behavior.reset_source()
                self._behavior_speech.reset()
                self._closed.wait(.2)

    def behavior_status(self):
        behavior = getattr(self, '_behavior', None)
        if behavior is None:
            return dict(success=True, running=False, ready=False, enabled={'fall': False, 'fight': False},
                        branches={}, partial=False, fresh=False, error='行为识别未初始化')
        result = behavior.status()
        speaker = getattr(self, '_behavior_speech', None)
        result['speech'] = speaker.status() if speaker else {}
        return result

    def _behavior_reset(self):
        behavior = getattr(self, '_behavior', None)
        if behavior is not None: behavior.reset_source()
        speaker = getattr(self, '_behavior_speech', None)
        if speaker is not None: speaker.reset()

    @ordinary_camera_operation
    def configure_behavior(self, payload):
        enabled = BehaviorSupervisor._validate_config(payload)
        previous = self._behavior.status()
        yolo_running = self._yolo.status().get('running', False)
        active = bool(yolo_running or previous.get('manual_detection_enabled')
                      or self._inspection_mode in ('transit', 'inspection'))
        self._behavior_speech.reset()
        if any(enabled.values()) and active:
            self._yolo.stop()
        result = self._behavior.configure(payload)
        if active and any(enabled.values()) and yolo_running and self._inspection_mode == 'manual':
            result = self._behavior.start()
        elif active and not any(enabled.values()):
            self._yolo.start()
            if self._inspection_mode in ('transit', 'inspection'):
                self._yolo.inspection_request('mode', {'mode': self._inspection_mode,
                    'task_id': self._inspection_task_id}, timeout=30.0)
        return result

    def _recognition_store(self) -> RecognitionDatabase:
        with self._recognition_lock:
            if self._recognition_database is None:
                self._recognition_database = RecognitionDatabase(
                    getattr(self.args, 'recognition_db', DEFAULT_RECOGNITION_DB))
            return self._recognition_database

    def _registry_face_extractor(self) -> SFaceFeatureExtractor:
        with self._recognition_lock:
            if self._recognition_face_extractor is None:
                self._recognition_face_extractor = SFaceFeatureExtractor(
                    getattr(self.args, 'recognition_sface_model', DEFAULT_SFACE_MODEL))
            return self._recognition_face_extractor

    def _registry_face_embedding(self, image: Any) -> list[float]:
        import cv2

        with self._recognition_lock:
            extractor = self._registry_face_extractor()
            if not extractor.available:
                raise RuntimeError(extractor.error or 'SFace模型不可用')
            if self._recognition_face_detector is None:
                model = Path(getattr(
                    self.args, 'yolo_face_model',
                    DEFAULT_YOLO_FACE_MODEL)).expanduser().resolve()
                if not model.is_file():
                    raise RuntimeError(f'YuNet人脸模型不存在: {model}')
                self._recognition_face_detector = cv2.FaceDetectorYN.create(
                    str(model), '', (320, 320), 0.70, 0.3, 5000)
            height, width = image.shape[:2]
            self._recognition_face_detector.setInputSize((width, height))
            ok, faces = self._recognition_face_detector.detect(image)
            if not ok or faces is None or len(faces) == 0:
                raise ValueError('人脸样本中没有检测到清晰正脸')
            face = max(faces, key=lambda item: float(item[-1]))
            x, y, box_width, box_height = [float(item) for item in face[:4]]
            landmarks = [
                [float(face[offset]), float(face[offset + 1])]
                for offset in range(4, min(14, len(face) - 1), 2)
            ]
            return extractor.extract(
                image, [x, y, x + box_width, y + box_height], landmarks)

    @staticmethod
    def _decode_image(value: Any) -> Any:
        import cv2
        import numpy as np

        text = str(value or '')
        if ',' in text and text.lstrip().startswith('data:'):
            text = text.split(',', 1)[1]
        try:
            payload = base64.b64decode(text, validate=True)
        except (ValueError, TypeError) as error:
            raise ValueError('人脸样本不是有效Base64图像') from error
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.size == 0:
            raise ValueError('无法解码人脸样本图像')
        return image

    def recognition_status(self) -> dict[str, Any]:
        store = self._recognition_store()
        extractor = self._registry_face_extractor()
        child = {}
        try:
            child = self.inspection_state().get('recognition') or {}
        except (RuntimeError, OSError, TimeoutError):
            child = {}
        return {
            'success': True, 'database': str(store.path),
            'stats': store.stats(),
            'models': {
                'sface': {
                    'available': extractor.available,
                    'error': extractor.error,
                },
                'runtime': child,
            },
        }

    def recognition_people(self) -> dict[str, Any]:
        return {'success': True, 'people': self._recognition_store().list_people()}

    def recognition_vehicles(self) -> dict[str, Any]:
        return {'success': True, 'vehicles': self._recognition_store().list_vehicles()}

    def recognition_events(self, query: dict[str, list[str]]) -> dict[str, Any]:
        def value(name: str, default: str = '') -> str:
            return str(query.get(name, [default])[0] or default)
        events = self._recognition_store().list_events(
            task_id=value('task_id'), category=value('category'),
            status=value('status'), limit=int(value('limit', '100')),
            offset=int(value('offset', '0')))
        return {'success': True, 'events': events}

    def recognition_audit(self, query: dict[str, list[str]]) -> dict[str, Any]:
        limit = int(query.get('limit', ['100'])[0])
        offset = int(query.get('offset', ['0'])[0])
        return {
            'success': True,
            'audit': self._recognition_store().list_audit(
                limit=limit, offset=offset),
        }

    def recognition_summary(self, task_id: str) -> dict[str, Any]:
        return {
            'success': True, 'task_id': str(task_id or ''),
            'summary': self._recognition_store().task_summary(task_id),
        }

    def recognition_evidence(self, value: str) -> bytes:
        root = Path(getattr(
            self.args, 'recognition_evidence_root', DEFAULT_EVIDENCE_DIR)).resolve()
        target = Path(str(value or '')).expanduser().resolve()
        try:
            target.relative_to(root)
        except ValueError as error:
            raise ValueError('识别证据路径越界') from error
        if not target.is_file():
            raise ValueError('识别证据不存在或已按保留策略清理')
        return target.read_bytes()

    def recognition_write(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        store = self._recognition_store()
        operator = str(payload.get('operator') or '')
        if action == 'person/upsert':
            embeddings = list(payload.get('embeddings') or [])
            images = payload.get('sample_images_base64') or []
            if images:
                for encoded in images[:10]:
                    image = self._decode_image(encoded)
                    embeddings.append(self._registry_face_embedding(image))
            person = store.upsert_person(
                payload.get('person_id', ''), payload.get('name', ''),
                embeddings, notes=payload.get('notes', ''),
                enabled=payload.get('enabled', True), operator=operator)
            return {'success': True, 'person': person}
        if action == 'person/delete':
            return {
                'success': store.delete_person(
                    payload.get('person_id', ''), operator=operator)}
        if action == 'vehicle/upsert':
            vehicle = store.upsert_vehicle(
                payload.get('vehicle_id', ''), payload.get('plate_number', ''),
                owner=payload.get('owner', ''),
                plate_color=payload.get('plate_color', 'unknown'),
                notes=payload.get('notes', ''),
                enabled=payload.get('enabled', True), operator=operator)
            return {'success': True, 'vehicle': vehicle}
        if action == 'vehicle/delete':
            return {
                'success': store.delete_vehicle(
                    payload.get('vehicle_id', ''), operator=operator)}
        if action == 'event/review':
            return {
                'success': store.review_event(
                    int(payload.get('event_id') or 0),
                    str(payload.get('status') or ''), reviewer=operator)}
        if action == 'cleanup':
            return {
                'success': True,
                'removed': store.cleanup_expired(
                    evidence_root=getattr(
                        self.args, 'recognition_evidence_root', DEFAULT_EVIDENCE_DIR),
                    operator=operator),
            }
        raise ValueError('不支持的识别管理操作')

    def _error(self, prefix: str) -> RuntimeError:
        code = int(self.sdk.NET_DVR_GetLastError())
        detail = SDK_ERRORS.get(code)
        return RuntimeError(f"{prefix}: {code}" + (f"（{detail}）" if detail else ""))

    def _configure_sdk_functions(self) -> None:
        """按 Linux HCNetSDK 头文件声明参数宽度，防止 64 位 ABI 错位。"""
        self.sdk.NET_DVR_SetSDKInitCfg.argtypes = [c_int, c_void_p]
        self.sdk.NET_DVR_SetSDKInitCfg.restype = c_int
        self.sdk.NET_DVR_Init.restype = c_int
        self.sdk.NET_DVR_SetConnectTime.argtypes = [c_uint, c_uint]
        self.sdk.NET_DVR_SetConnectTime.restype = c_int
        self.sdk.NET_DVR_Login_V30.argtypes = [c_char_p, c_ushort, c_char_p, c_char_p, POINTER(DeviceInfoV30)]
        self.sdk.NET_DVR_Login_V30.restype = c_int
        self.sdk.NET_DVR_GetLastError.restype = c_uint
        self.sdk.NET_DVR_RealPlay_V40.argtypes = [c_int, POINTER(PreviewInfo), RealDataCallback, c_void_p]
        self.sdk.NET_DVR_RealPlay_V40.restype = c_int
        self.sdk.NET_DVR_StopRealPlay.argtypes = [c_int]
        self.sdk.NET_DVR_StopRealPlay.restype = c_int
        self.sdk.NET_DVR_PTZControlWithSpeed_Other.argtypes = [c_int, c_int, c_uint, c_uint, c_uint]
        self.sdk.NET_DVR_PTZControlWithSpeed_Other.restype = c_int
        self.sdk.NET_DVR_GetDVRConfig.argtypes = [c_int, c_uint, c_int, c_void_p, c_uint, POINTER(c_uint)]
        self.sdk.NET_DVR_GetDVRConfig.restype = c_int
        self.sdk.NET_DVR_SetDVRConfig.argtypes = [c_int, c_uint, c_int, c_void_p, c_uint]
        self.sdk.NET_DVR_SetDVRConfig.restype = c_int
        self.sdk.NET_DVR_CaptureJPEGPicture_NEW.argtypes = [c_int, c_int, c_void_p, c_void_p, c_uint, POINTER(c_uint)]
        self.sdk.NET_DVR_CaptureJPEGPicture_NEW.restype = c_int
        self.sdk.NET_DVR_Logout.argtypes = [c_int]
        self.sdk.NET_DVR_Logout.restype = c_int
        self.sdk.NET_DVR_Cleanup.restype = c_int

    def _connect_locked(self) -> None:
        """在命令线程加载 SDK、配置依赖库并登录设备。"""
        if not (self.args.sdk_dir / "libhcnetsdk.so").is_file():
            raise RuntimeError(f"未找到 HCNetSDK: {self.args.sdk_dir / 'libhcnetsdk.so'}")
        self.sdk = CDLL(str(self.args.sdk_dir / "libhcnetsdk.so"))
        self._configure_sdk_functions()
        component_path = LocalSdkPath()
        component_path.sPath = os.fsencode(self.args.sdk_dir)
        for kind, value in (
            (2, byref(component_path)),
            (3, create_string_buffer(os.fsencode(self.args.sdk_dir / "libcrypto.so.1.1"))),
            (4, create_string_buffer(os.fsencode(self.args.sdk_dir / "libssl.so.1.1"))),
        ):
            if not self.sdk.NET_DVR_SetSDKInitCfg(kind, value):
                raise self._error("SDK 路径配置失败")
        if not self.sdk.NET_DVR_Init():
            raise self._error("SDK 初始化失败")
        self.sdk.NET_DVR_SetConnectTime(self.args.connect_timeout_ms, self.args.connect_retries)
        self.user_id = self.sdk.NET_DVR_Login_V30(
            self.args.host.encode(), self.args.port, self.args.username.encode(), self.args.password.encode(), byref(self.device_info)
        )
        if self.user_id < 0:
            raise self._error("设备登录失败")

    def _start_v40_locked(self) -> None:
        source = V40MjpegSource(self.sdk, self.user_id, self.args)
        try:
            source.start()
        except BaseException:
            # RealPlay 或播放器初始化失败时，编码线程和播放端口不能遗留在后台。
            source.close()
            self._retired_v40_sources.append(source)
            raise
        with self._video_ready:
            self.video = source
            self._video_ready.notify_all()

    def _start_jpeg_locked(self) -> None:
        source = JpegFallbackSource(self)
        with self._video_ready:
            self.video = source
            self._video_ready.notify_all()
        source.start()

    def _start_usb_source(self) -> V4L2MjpegSource:
        source = self._create_ordinary_source(self._camera_sources.active())
        with self._video_ready:
            self.video = source
            self._video_ready.notify_all()
        try:
            source.start()
        except BaseException:
            with self._video_ready:
                if self.video is source:
                    self.video = None
                    self._video_ready.notify_all()
            source.close()
            raise
        return source

    def _create_ordinary_source(self, config):
        if self._ordinary_pending_close is not None:
            self._ordinary_pending_close.close()
            self._ordinary_pending_close = None
        if self._ordinary_source_factory is not None:
            return self._ordinary_source_factory(config)
        if config.get('pipeline') == 'c12':
            return C12OrdinarySource(self.c12.sources['visible'], LatestJpegFrame)
        if config['source'] == 'body':
            return HttpCameraSource(config['body_url'], LatestJpegFrame())
        args = copy.copy(self.args)
        for key in ('usb_device','usb_width','usb_height','usb_fps'):
            setattr(args, key, config[key])
        return V4L2MjpegSource(args)

    def _camera_switch_lock_reason(self):
        if getattr(self.args, 'camera_type', 'usb') != 'usb':
            return '旧海康兼容模式不支持此来源切换'
        if self._inspection_mode != 'manual' or self._inspection_task_id:
            return '请先结束巡检任务，再切换普通摄像头'
        if self._yolo.status().get('running'):
            return '请先停止识别，再切换普通摄像头'
        if self.behavior_status().get('running'):
            return '请先停止行为识别，再切换普通摄像头'
        if self._closed.is_set(): return '相机服务正在关闭'
        return ''

    def camera_source_status(self):
        source = self._video_source()
        video = source.info() if source is not None else {'error':'视频未启动'}
        return dict(success=True, config=copy.deepcopy(self._camera_sources.current),
            config_error=self._camera_sources.error,
            error=self._camera_source_error,
            video=video,
            config_file=str(self._camera_sources.path or ''),
            devices=discover_usb_cameras(),
            generation=self._camera_source_generation,
            switching=self.is_video_restarting(),
            locked_reason=self._camera_switch_lock_reason())

    def _camera_selection(self):
        config = self._camera_sources.current
        return dict(source=source_kind(config), pipeline=config['pipeline'],
                    label=source_label(config), version=config['version'],
                    generation=self._camera_source_generation,
                    error=self._camera_sources.error or self._camera_source_error)

    @ordinary_camera_operation
    def configure_camera_source(self, payload):
        if not isinstance(payload, dict) or set(payload) != {'config','expected_version'}:
            raise ValueError('普通摄像头配置请求无效')
        draft = validate_config(payload['config'], forbidden_ports=self._camera_sources.forbidden_ports)
        self._camera_sources.check_version(payload['expected_version'])
        reason = self._camera_switch_lock_reason()
        if reason: raise RuntimeError(reason)
        with self._video_restart_lock:
            previous = copy.deepcopy(self._camera_sources.current)
            had_source = self._video_source() is not None
            candidate = None
            self._set_video_restarting(True)
            try:
                self._stop_video_locked()
                candidate = self._create_ordinary_source(draft)
                candidate.start()
                self._wait_for_first_frame(candidate)
                self._camera_sources.save(draft, expected_version=payload['expected_version'])
                with self._video_ready:
                    self.video = candidate
                    self._camera_source_generation += 1
                    self._video_ready.notify_all()
                self._camera_source_error = ''
            except Exception as error:
                rollback_error = ''
                if candidate is not None:
                    try: candidate.close()
                    except Exception as caught:
                        self._ordinary_pending_close = candidate
                        rollback_error = str(caught)
                if had_source and not rollback_error:
                    try:
                        restored = self._create_ordinary_source(previous)
                        restored.start()
                        with self._video_ready:
                            self.video = restored
                            self._video_ready.notify_all()
                        self._wait_for_first_frame(restored)
                    except Exception as caught: rollback_error = str(caught)
                self._camera_source_error = str(error)
                suffix = f'；恢复原来源失败：{rollback_error}' if rollback_error else '；原配置未改变，已恢复原来源' if had_source else '；原配置未改变'
                raise RuntimeError(f'摄像头切换失败：{error}{suffix}') from error
            finally:
                self._set_video_restarting(False)
            return {**self.camera_source_status(), 'message':'普通摄像头已保存并切换'}

    def _stop_video_locked(self) -> None:
        self._behavior_reset()
        with self._video_ready:
            source = self.video
            self.video = None
            self._video_ready.notify_all()
        if source is not None:
            # 即使 SDK 释放返回异常，也不能让网关继续保留已关闭的视频源。
            try:
                source.close()
            except Exception:
                if getattr(self.args, 'camera_type', 'usb') == 'usb':
                    self._ordinary_pending_close = source
                raise
            if isinstance(source, V40MjpegSource):
                self._retired_v40_sources.append(source)

    def _video_source(self) -> V4L2MjpegSource | V40MjpegSource | JpegFallbackSource | None:
        """安全取得当前源的本地引用；重连期间旧源会自行唤醒等待者。"""
        with self._video_lock:
            return self.video

    def _set_video_restarting(self, restarting: bool) -> None:
        """发布视频重建状态并唤醒等待新视频源的网页连接。"""
        if restarting:
            self._video_restarting.set()
        else:
            self._video_restarting.clear()
        with self._video_ready:
            self._video_ready.notify_all()

    def is_video_restarting(self) -> bool:
        """供 HTTP 流处理器区分计划内切换和真实视频故障。"""
        return self._video_restarting.is_set()

    def _wait_for_first_frame(self, source: V4L2MjpegSource | V40MjpegSource | JpegFallbackSource) -> None:
        """以短等待轮询首帧，使 Ctrl+C 能打断正在进行的视频重连。"""
        deadline = time.monotonic() + self.args.video_start_timeout
        last_error = None
        while True:
            if self._closed.is_set():
                raise RuntimeError("网页控制台正在关闭")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                detail = f'：{last_error}' if last_error else ''
                raise TimeoutError(f"等待视频首帧超时{detail}")
            try:
                source.wait_for_first_frame(min(0.5, remaining))
                return
            except (TimeoutError, RuntimeError) as error:
                if isinstance(error, RuntimeError) and getattr(self.args,'camera_type','usb') != 'usb':
                    raise
                last_error = error
                self._closed.wait(min(0.05, remaining))

    def connect(self) -> None:
        """检查网络、登录并启动经过首帧确认的视频源。"""
        removed = self._recognition_store().cleanup_expired(
            evidence_root=getattr(
                self.args, 'recognition_evidence_root', DEFAULT_EVIDENCE_DIR),
            operator='startup-retention')
        if any(removed.values()):
            print(f"[识别] 已按保留策略清理: {removed}")
        if self.args.camera_type == "usb":
            if self._camera_sources.error:
                print(self._camera_sources.error)
                return
            try:
                source = self._start_usb_source()
                try:
                    self._wait_for_first_frame(source)
                    info = source.info()
                    print(
                        f"[普通相机] 已打开 {source_label(self._camera_sources.current)}: "
                        f"{info.get('width', 0)}x{info.get('height', 0)} "
                        f"{info.get('fps', 0)} FPS"
                    )
                except (RuntimeError, OSError, TimeoutError) as error:
                    # Keep the web/API process alive when the camera is late or
                    # temporarily unplugged.  The V4L2 worker continues trying
                    # and will publish automatically as soon as it returns.
                    print(f"[USB] 首帧尚未就绪，后台继续重连: {error}")
                return
            except BaseException:
                self.close()
                raise
        try:
            with socket.create_connection((self.args.host, self.args.port), timeout=3):
                pass
        except OSError as error:
            self.executor.close()
            raise RuntimeError(f"设备网络不可达: {self.args.host}:{self.args.port}（{error}）") from error
        try:
            self.executor.call(self._connect_locked, timeout=12)
            v40_error: BaseException | None = None
            if self.args.video_backend != "jpeg":
                try:
                    self.executor.call(self._start_v40_locked, timeout=8)
                    source = self._video_source()
                    if source is None:
                        raise RuntimeError("V40 视频源未建立")
                    self._wait_for_first_frame(source)
                except BaseException as error:
                    v40_error = error
                    self.executor.call(self._stop_video_locked, timeout=5)
            if self._video_source() is None:
                if self.args.video_backend == "v40":
                    raise RuntimeError(f"V40 视频启动失败: {v40_error}")
                self.executor.call(self._start_jpeg_locked, timeout=3)
                source = self._video_source()
                if source is None:
                    raise RuntimeError("JPEG 回退视频源未建立")
                self._wait_for_first_frame(source)
                if v40_error is not None:
                    print(f"警告: V40 视频不可用，已切换 JPEG 回退: {v40_error}")
            try:
                self._cache_ptz_state(
                    self.executor.call(self._read_ptz_locked, timeout=3))
            except BaseException as error:
                print(f"警告: 初始云台位置读取失败: {error}")
            self._position_worker = threading.Thread(target=self._position_loop, daemon=True, name="hik-position")
            self._position_worker.start()
        except BaseException:
            self.close()
            raise

    def _read_ptz_locked(self) -> dict[str, Any] | None:
        position = PtzPosition()
        returned = c_uint(0)
        if not self.sdk.NET_DVR_GetDVRConfig(self.user_id, NET_DVR_GET_PTZPOS, self.args.channel, byref(position), sizeof(position), byref(returned)):
            return None
        pan_scale = float(self.args.inspection_pan_units_per_degree)
        zoom_scale = float(self.args.inspection_zoom_units_per_x)
        return {
            "pan_raw": int(position.wPanPos),
            "tilt_raw": int(position.wTiltPos),
            "zoom_raw": int(position.wZoomPos),
            "pan": float(position.wPanPos) / pan_scale,
            # 本设备俯仰坐标使用加 90 度偏移，与键盘控制脚本保持一致。
            "tilt": float(position.wTiltPos) / pan_scale - 90.0,
            "zoom_ratio": max(
                1.0 + float(position.wZoomPos) / zoom_scale, 1.0),
        }

    def _read_tilt_locked(self) -> float | None:
        state = self._read_ptz_locked()
        return None if state is None else float(state["tilt"])

    def _cache_ptz_state(self, state: dict[str, Any] | None) -> None:
        if state is None:
            return
        with self._state_lock:
            self._last_pan = float(state["pan"])
            self._last_tilt = float(state["tilt"])
            self._last_zoom_ratio = float(state["zoom_ratio"])
            if self._front_pan_raw is None:
                self._front_pan_raw = int(state["pan_raw"])

    def _position_loop(self) -> None:
        """低频刷新俯仰缓存；HTTP 状态接口不直接争用 SDK。"""
        while not self._closed.wait(self.args.position_interval):
            try:
                state = self.executor.call(self._read_ptz_locked, timeout=3)
                self._cache_ptz_state(state)
            except BaseException:
                continue

    def _prune_cancelled_motion_tokens_locked(self) -> None:
        """控制 token 只需覆盖启动/松键竞态，过期后立即释放内存。"""
        expiry = time.monotonic() - max(5.0, self.args.motion_timeout * 4)
        for token, created in tuple(self._cancelled_motion_tokens.items()):
            if created < expiry:
                del self._cancelled_motion_tokens[token]

    def _stop_motion_locked(self, token: str | None = None) -> dict[str, Any]:
        """发送一次停止，且只有 SDK 停止成功后才清除活动状态。"""
        with self._state_lock:
            self._prune_cancelled_motion_tokens_locked()
            if self._active_command is None:
                # stop 可能排在尚未执行的 start 前；标记 token 可阻止那个延迟启动。
                if token:
                    self._cancelled_motion_tokens[token] = time.monotonic()
                self._watchdog_pending = False
                return {"active": None}
            if token and token != self._active_token:
                # 旧页面的异步 stop 不得停止新动作，但仍记录其 token 的取消状态。
                self._cancelled_motion_tokens[token] = time.monotonic()
                return {"active": self._active_action}
            command = self._active_command
            active_token = self._active_token
            active_speed = self._active_speed
        if not self.sdk.NET_DVR_PTZControlWithSpeed_Other(self.user_id, self.args.channel, command, 1, active_speed):
            with self._state_lock:
                if self._active_token == active_token and self._active_command == command:
                    # 立刻让看门狗重试，不能因一次设备繁忙而遗留持续运动。
                    self._motion_deadline = time.monotonic()
                    self._watchdog_pending = False
            raise self._error("云台停止失败")
        with self._state_lock:
            if self._active_token == active_token and self._active_command == command:
                self._active_action = None
                self._active_command = None
                self._active_speed = self.args.speed
                self._active_token = ""
                self._motion_deadline = 0.0
                self._watchdog_pending = False
        return {"active": None}

    def _start_motion_locked(self, action: str, speed: int, token: str) -> dict[str, Any]:
        """停止旧动作、校验俯仰限位后启动新的云台或镜头命令。"""
        command = PTZ_ACTIONS.get(action)
        if command is None:
            raise ValueError("无效云台或镜头动作")
        with self._state_lock:
            self._prune_cancelled_motion_tokens_locked()
            if self._cancelled_motion_tokens.pop(token, None) is not None:
                return {"active": None, "token": token, "message": "动作已在启动前取消"}
        self._stop_motion_locked()
        if command in (TILT_UP, TILT_DOWN):
            angle = self._read_tilt_locked()
            if angle is None:
                raise RuntimeError("无法读取俯仰位置，已拒绝上下运动")
            with self._state_lock:
                self._last_tilt = angle
            if command == TILT_UP and angle >= 89.5:
                return {"active": None, "message": f"已在上限附近 ({angle:.2f} 度)"}
            if command == TILT_DOWN and angle <= -89.5:
                return {"active": None, "message": f"已在下限附近 ({angle:.2f} 度)"}
        if not self.sdk.NET_DVR_PTZControlWithSpeed_Other(self.user_id, self.args.channel, command, 0, speed):
            raise self._error("云台启动失败")
        with self._state_lock:
            self._active_action = action
            self._active_command = command
            self._active_speed = speed
            self._active_token = token
            self._motion_deadline = time.monotonic() + self.args.motion_timeout
            self._watchdog_pending = False
        return {"active": action, "token": token, "speed": speed}

    def start_motion(self, action: str, speed: Any, token: str) -> dict[str, Any]:
        """网页按下操作的入口，优先于耗时配置请求并由服务端超时停止。"""
        if self.args.camera_type == "usb":
            raise RuntimeError("USB 固定摄像头不支持云台控制")
        if not token or len(token) > 64:
            raise ValueError("控制令牌无效")
        try:
            actual_speed = max(1, min(7, int(speed)))
        except (TypeError, ValueError) as error:
            raise ValueError("云台速度必须为 1 到 7") from error
        return self.executor.call(self._start_motion_locked, action, actual_speed, token, timeout=5, urgent=True)

    def keepalive(self, action: str, token: str) -> dict[str, Any]:
        """保活只刷新内存截止时间，不向 SDK 叠加多余控制请求。"""
        with self._state_lock:
            if action != self._active_action or token != self._active_token:
                raise ValueError("当前没有对应动作")
            self._motion_deadline = time.monotonic() + self.args.motion_timeout
            return {"active": action, "token": token}

    def stop_motion(self, token: str | None = None) -> dict[str, Any]:
        """松键、失焦和页面离开时停止当前动作。"""
        return self.executor.call(self._stop_motion_locked, token, timeout=4, urgent=True)

    @staticmethod
    def _circular_distance(value: int, target: int, modulus: int) -> int:
        difference = abs(int(value) - int(target)) % modulus
        return min(difference, modulus - difference)

    def _set_ptz_position_locked(
            self, *, pan_raw: int | None = None,
            zoom_raw: int | None = None) -> dict[str, Any]:
        current = self._read_ptz_locked()
        if current is None:
            raise RuntimeError("无法读取云台绝对位置")
        position = PtzPosition()
        position.wAction = 1
        position.wPanPos = int(
            current["pan_raw"] if pan_raw is None else pan_raw)
        position.wTiltPos = int(current["tilt_raw"])
        position.wZoomPos = int(
            current["zoom_raw"] if zoom_raw is None else zoom_raw)
        if not self.sdk.NET_DVR_SetDVRConfig(
                self.user_id, NET_DVR_SET_PTZPOS, self.args.channel,
                byref(position), sizeof(position)):
            raise self._error("设备不支持云台绝对位置设置")
        deadline = time.monotonic() + self.args.inspection_position_timeout
        pan_modulus = max(
            1, int(round(360.0 * self.args.inspection_pan_units_per_degree)))
        last = current
        while time.monotonic() < deadline:
            time.sleep(0.2)
            observed = self._read_ptz_locked()
            if observed is None:
                continue
            last = observed
            pan_ok = (
                pan_raw is None
                or self._circular_distance(
                    int(observed["pan_raw"]), int(pan_raw), pan_modulus)
                <= self.args.inspection_pan_tolerance_deg
                * self.args.inspection_pan_units_per_degree
            )
            zoom_ok = (
                zoom_raw is None
                or abs(int(observed["zoom_raw"]) - int(zoom_raw))
                <= max(1, int(self.args.inspection_zoom_units_per_x * 0.1))
            )
            if pan_ok and zoom_ok:
                self._cache_ptz_state(observed)
                return observed
        self._cache_ptz_state(last)
        raise TimeoutError("云台在限定时间内未到达目标位置")

    def _estimated_pan_move_locked(self, target_angle: float) -> dict[str, Any]:
        delta = (float(target_angle) - self._inspection_angle + 180.0) % 360.0 - 180.0
        if abs(delta) < 1.0:
            return {"arrival_estimated": True}
        command = PAN_RIGHT if delta > 0.0 else PAN_LEFT
        duration = min(
            self.args.inspection_position_timeout,
            abs(delta) / 90.0 * self.args.inspection_pan_90_seconds)
        self._stop_motion_locked()
        if not self.sdk.NET_DVR_PTZControlWithSpeed_Other(
                self.user_id, self.args.channel, command, 0,
                self.args.inspection_pan_speed):
            raise self._error("云台估算转向启动失败")
        try:
            time.sleep(duration)
        finally:
            if not self.sdk.NET_DVR_PTZControlWithSpeed_Other(
                    self.user_id, self.args.channel, command, 1,
                    self.args.inspection_pan_speed):
                raise self._error("云台估算转向停止失败")
        self._inspection_angle = float(target_angle) % 360.0
        return {"arrival_estimated": True}

    def _move_inspection_direction_locked(
            self, direction: str) -> dict[str, Any]:
        direction = str(direction).strip().lower()
        angles = {"front": 0.0, "left": -90.0, "right": 90.0, "back": 180.0}
        if direction == "current":
            return {"direction": direction, "arrival_estimated": False}
        if direction not in angles:
            raise ValueError("巡检方向必须是 current/front/left/right/back")
        target_angle = angles[direction]
        absolute_error: BaseException | None = None
        try:
            current = self._read_ptz_locked()
            if current is None:
                raise RuntimeError("无法读取云台水平位置")
            if self._front_pan_raw is None:
                self._front_pan_raw = int(current["pan_raw"])
            modulus = max(
                1, int(round(
                    360.0 * self.args.inspection_pan_units_per_degree)))
            target_raw = int(round(
                self._front_pan_raw
                + target_angle * self.args.inspection_pan_units_per_degree
            )) % modulus
            observed = self._set_ptz_position_locked(pan_raw=target_raw)
            result = {
                "pan": observed["pan"], "zoom_ratio": observed["zoom_ratio"],
                "arrival_estimated": False,
            }
            self._inspection_angle = target_angle % 360.0
        except BaseException as error:
            absolute_error = error
            result = self._estimated_pan_move_locked(target_angle)
            result["absolute_error"] = str(absolute_error)
        self._inspection_direction = direction
        self._inspection_position_estimated = bool(
            result.get("arrival_estimated"))
        return {"success": True, "direction": direction, **result}

    def _set_inspection_zoom_locked(self, zoom_ratio: float) -> dict[str, Any]:
        ratio = float(zoom_ratio)
        if ratio < 1.0 or ratio > self.args.inspection_zoom_max_ratio:
            raise ValueError(
                f"变倍倍率必须在 1 到 {self.args.inspection_zoom_max_ratio:g} 之间")
        target_raw = int(round(
            (ratio - 1.0) * self.args.inspection_zoom_units_per_x))
        observed = self._set_ptz_position_locked(zoom_raw=target_raw)
        self._inspection_position_estimated = False
        return {
            "success": True, "zoom_ratio": observed["zoom_ratio"],
            "arrival_estimated": False,
        }

    def inspection_ptz(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Move to a body-relative cardinal direction or set zoom ratio."""
        if self.args.camera_type == "usb":
            return {
                "success": True,
                "supported": False,
                "fixed_view": True,
                "direction": "current",
                "zoom_ratio": None,
            }
        if str(payload.get("action", "")) == "set_zoom":
            return self.executor.call(
                self._set_inspection_zoom_locked,
                float(payload.get("zoom_ratio")),
                timeout=self.args.inspection_position_timeout + 3.0)
        return self.executor.call(
            self._move_inspection_direction_locked,
            str(payload.get("direction", "")),
            timeout=self.args.inspection_position_timeout + 3.0)

    def watchdog(self) -> None:
        """由服务器周期调用；保活停止后自动提交一次防御性停止。"""
        with self._state_lock:
            expired = self._active_command is not None and time.monotonic() >= self._motion_deadline
            if not expired or self._watchdog_pending:
                return
            token = self._active_token
            self._watchdog_pending = True
        if not self.executor.submit_urgent(self._stop_motion_locked, token):
            # 关闭阶段以外应继续尝试提交，不能因一次队列状态丢失安全停止。
            with self._state_lock:
                if self._active_token == token:
                    self._watchdog_pending = False

    def get_image_settings(self) -> dict[str, Any]:
        """图像参数接口已禁用，避免触发不稳定的 SDK 图像配置调用。"""
        raise RuntimeError("图像参数接口已禁用")

    def update_image_settings(self, values: Any) -> dict[str, Any]:
        """图像参数接口已禁用，避免触发不稳定的 SDK 图像配置调用。"""
        raise RuntimeError("图像参数接口已禁用")

    def _read_wiper_locked(self) -> dict[str, Any]:
        config = WiperInfoCfg()
        config.dwSize = sizeof(config)
        returned = c_uint(0)
        if not self.sdk.NET_DVR_GetDVRConfig(self.user_id, NET_DVR_GET_WIPERINFO_CFG, self.args.channel, byref(config), sizeof(config), byref(returned)):
            if int(self.sdk.NET_DVR_GetLastError()) == 23:
                raise WiperConfigUnsupportedError("设备不支持 SDK 雨刮工作模式配置（错误码 23）")
            raise self._error("读取雨刮工作模式失败")
        self._wiper_config = config
        return self._wiper_to_dict(config)

    @staticmethod
    def _wiper_to_dict(config: WiperInfoCfg) -> dict[str, Any]:
        mode = int(config.byWiperWorkMode)
        return {
            "mode": WIPER_MODE_NAMES.get(mode, f"unknown:{mode}"),
            "interval_seconds": int(config.ustruWiperModeParam.struContinue.byWorkTimeInterval),
            "duration_seconds": int(config.ustruWiperModeParam.struContinue.dwContinueWorkTime),
            "sensitivity": int(config.wSensitivity),
        }

    def _get_wiper_capability(self) -> dict[str, Any]:
        """返回 PTZAux 电源能力；该资源不支持 GET，状态只能记录最后一次请求。"""
        with self._state_lock:
            if self._wiper_capability_cache is not None:
                return {**self._wiper_capability_cache, "power": self._wiper_on}
            # 官方网页同样仅 PUT /auxcontrols/1，不提供可用的状态查询；避免页面加载时
            # 访问 SDK 3259 或对 auxcontrols 发非法 GET。
            capability = {"supported": True, "reason": "", "power_known": False}
            self._wiper_capability_cache = capability
            return {**capability, "power": self._wiper_on}

    def _set_wiper_power_http(self, enabled: bool, *, allow_closing: bool = False) -> dict[str, Any]:
        """使用设备网页同类的 PTZAux 请求控制雨刮，不进入 HCNetSDK。"""
        with self._http_lock:
            if self._closed.is_set() and not allow_closing:
                raise RuntimeError("网页控制台正在关闭")
            set_wiper_aux_power(self.args, enabled)
        with self._state_lock:
            self._wiper_on = enabled
        return {"power": enabled, "supported": True}

    def _update_wiper_locked(self, mode_name: str, interval_seconds: Any, duration_minutes: Any) -> dict[str, Any]:
        if mode_name not in WIPER_MODE_VALUES:
            raise ValueError("雨刮模式必须是 single、continuous、smart 或 off")
        if self._wiper_config is None:
            self._read_wiper_locked()
        assert self._wiper_config is not None
        config = self._wiper_config
        mode = WIPER_MODE_VALUES[mode_name]
        config.byWiperWorkMode = mode
        if mode == 1:
            try:
                interval = int(interval_seconds)
                minutes = int(duration_minutes)
            except (TypeError, ValueError) as error:
                raise ValueError("持续模式的间隔和时长必须是整数") from error
            if not 2 <= interval <= 60:
                raise ValueError("雨刮间隔必须在 2 到 60 秒之间")
            if not 2 <= minutes <= 1440:
                raise ValueError("雨刮持续时长必须在 2 到 1440 分钟之间")
            config.ustruWiperModeParam.struContinue.byWorkTimeInterval = interval
            config.ustruWiperModeParam.struContinue.dwContinueWorkTime = minutes * 60
        config.dwSize = sizeof(config)
        if not self.sdk.NET_DVR_SetDVRConfig(self.user_id, NET_DVR_SET_WIPERINFO_CFG, self.args.channel, byref(config), sizeof(config)):
            if int(self.sdk.NET_DVR_GetLastError()) == 23:
                raise WiperConfigUnsupportedError("设备不支持 SDK 雨刮工作模式配置（错误码 23）")
            raise self._error("设置雨刮工作模式失败")
        return self._wiper_to_dict(config)

    def get_wiper_settings(self) -> dict[str, Any]:
        """返回 PTZAux 电源能力；工作模式配置必须由用户显式启用。"""
        capability = self._get_wiper_capability()
        if not capability["supported"]:
            return {
                **capability,
                "config_supported": False,
                "config_reason": capability["reason"],
            }
        if not getattr(self.args, "enable_wiper_config", False):
            return {
                **capability,
                "config_supported": False,
                "config_reason": "当前未启用 SDK 雨刮工作模式配置；电源开关可正常使用",
            }
        with self._state_lock:
            if self._wiper_config_supported is False:
                return {
                    **capability,
                    "config_supported": False,
                    "config_reason": self._wiper_config_reason,
                }
        try:
            settings = self.executor.call(self._read_wiper_locked, timeout=6)
        except WiperConfigUnsupportedError as error:
            with self._state_lock:
                self._wiper_config_supported = False
                self._wiper_config_reason = str(error)
            return {
                **capability,
                "config_supported": False,
                "config_reason": str(error),
            }
        with self._state_lock:
            self._wiper_config_supported = True
            self._wiper_config_reason = ""
        return {**capability, **settings, "config_supported": True}

    def set_wiper_power(self, enabled: Any) -> dict[str, Any]:
        """即时打开或关闭雨刮，使用 PTZAux 避免与 V40 的 SDK 会话争用。"""
        if self.args.camera_type == "usb":
            raise RuntimeError("USB 固定摄像头不支持雨刮控制")
        if not isinstance(enabled, bool):
            raise ValueError("雨刮电源状态必须是布尔值")
        capability = self._get_wiper_capability()
        if not capability["supported"]:
            raise RuntimeError(capability["reason"])
        return self._set_wiper_power_http(enabled)

    def update_wiper_settings(self, mode: Any, interval_seconds: Any, duration_minutes: Any) -> dict[str, Any]:
        """配置单次、持续、智能或关闭模式；默认禁用以保护不支持的设备。"""
        if not isinstance(mode, str):
            raise ValueError("雨刮模式必须是字符串")
        capability = self._get_wiper_capability()
        if not capability["supported"]:
            raise RuntimeError(capability["reason"])
        if not getattr(self.args, "enable_wiper_config", False):
            raise RuntimeError("雨刮工作模式配置未启用；仅允许 PTZAux 电源开关")
        with self._state_lock:
            if self._wiper_config_supported is False:
                raise RuntimeError(self._wiper_config_reason)
        try:
            settings = self.executor.call(self._update_wiper_locked, mode, interval_seconds, duration_minutes, timeout=6)
        except WiperConfigUnsupportedError as error:
            with self._state_lock:
                self._wiper_config_supported = False
                self._wiper_config_reason = str(error)
            raise RuntimeError(str(error)) from error
        with self._state_lock:
            self._wiper_config_supported = True
            self._wiper_config_reason = ""
        return {**capability, **settings, "config_supported": True}

    def set_ir_mode(self, mode: Any) -> dict[str, Any]:
        """日夜切换期间停止旧 V40 会话，待滤光片切换后建立全新视频源。"""
        if self.args.camera_type == "usb":
            raise RuntimeError("USB 固定摄像头不支持日夜/红外控制")
        if mode not in ("day", "night"):
            raise ValueError("日夜模式必须为 day 或 night")
        with self._camera_config_lock:
            with self._video_restart_lock:
                if self._closed.is_set():
                    raise RuntimeError("网页控制台正在关闭")
                # 日夜切换会重置设备视频编码链路。发布“正在重建”后再停止旧回调，使并发
                # 的 MJPEG 请求等待新首帧，不会误判为服务已经退出。
                self._set_video_restarting(True)
                request_error: Exception | None = None
                restart_error: Exception | None = None
                try:
                    self.executor.call(self._stop_video_locked, timeout=5)
                    with self._http_lock:
                        set_ir_mode(self.args, mode)
                    with self._state_lock:
                        self._last_ir_mode = mode
                    # 给 IR-cut 机械切换和设备码流重建留出可配置的稳定时间。
                    self._closed.wait(getattr(self.args, "ir_settle_seconds", 1.0))
                except Exception as error:
                    request_error = error
                finally:
                    if not self._closed.is_set():
                        try:
                            self._restart_video_once()
                        except Exception as error:
                            restart_error = error
                    self._set_video_restarting(False)
                if request_error is not None:
                    if restart_error is not None:
                        raise RuntimeError(f"{request_error}；视频恢复失败: {restart_error}") from request_error
                    raise request_error
                if restart_error is not None:
                    raise RuntimeError(f"日夜模式已设置，但视频恢复失败: {restart_error}") from restart_error
        return {"mode": mode}

    def get_yolo_status(self) -> dict[str, Any]:
        """返回 YOLO 子进程与结果流状态，不阻塞相机主链路。"""
        return self._yolo.status()

    def get_yolo_frame(self, since: int = 0) -> tuple[int, bytes | None]:
        """通过本地服务端代理读取 YOLO 最新标注帧，浏览器不直接访问子进程端口。"""
        return self._yolo.frame(since)

    def open_yolo_stream(self):
        """打开 YOLO 子进程的本地 MJPEG 流，仅供同源 HTTP 代理使用。"""
        return self._yolo.open_stream()

    @property
    def _thermal(self):
        if (getattr(self, '_c12_thermal', None) is not None
                and self._camera_sources.current['pipeline'] == 'c12'):
            return self._c12_thermal
        return self._legacy_thermal

    @_thermal.setter
    def _thermal(self, client):
        self._legacy_thermal = client

    def open_thermal_stream(self):
        """Open the loopback thermal MJPEG stream for the same-origin proxy."""
        return self._thermal.open_stream()

    def thermal_frame(self, *, raw: bool = False) -> bytes:
        """Return one cached thermal JPEG without touching the USB device here."""
        return self._thermal.frame(raw=raw)

    def thermal_status(self) -> dict[str, Any]:
        return self._thermal.status()

    def display_thermal_frame(self):
        return self.thermal_frame()

    def display_generation(self):
        return self._camera_source_generation

    def display_status(self, measurement=None):
        selection = self._camera_selection()
        c12 = selection['pipeline'] == 'c12'
        if c12:
            thermal = {**self.c12.sources['infrared'].status(),
                       'max_temperature_c': None, 'alarm_active': False,
                       'detection_enabled': False, 'supports_temperature': False,
                       'temperature_error': self._c12_thermal.status()['error']}
        else:
            thermal = dict(self._thermal.status() if measurement is None else measurement)
            thermal['supports_temperature'] = True
        return {**selection, 'label': 'C12 双光相机' if c12 else '原有相机链路',
                'visible_label': 'C12 可见光' if c12 else selection['label'],
                'infrared_label': 'C12 红外' if c12 else '海康热成像',
                'thermal': thermal}

    def thermal_config(self, task_id: str = "") -> dict[str, Any]:
        return self._thermal.config(task_id)

    def update_thermal_config(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._thermal.update_config(payload)

    def detection_status(self) -> dict[str, Any]:
        yolo = self._yolo.status()
        thermal = self._thermal.status()
        behavior = self.behavior_status()
        behavior_required = any(behavior.get('enabled', {}).values())
        suspended = bool(behavior_required and (behavior.get('running') or behavior.get('manual_detection_enabled')
                         or self._inspection_mode in ('transit', 'inspection')))
        yolo = {**yolo, 'suspended_by_behavior': suspended and not yolo.get('running', False)}
        visual_running = bool(yolo.get("running"))
        thermal_running = bool(thermal.get("manual_detection_enabled"))
        active = visual_running or thermal_running or bool(behavior.get('running'))
        ready_count = int(bool(yolo.get("ready"))) + int(bool(thermal.get("ready"))) + int(bool(behavior.get('ready')) and behavior_required)
        required_count = 2 + int(behavior_required)
        return {
            "success": True,
            "running": active,
            "ready": ready_count == required_count,
            "partial": active and ready_count < required_count,
            "yolo": yolo,
            "thermal": thermal,
            "behavior": behavior,
        }

    @ordinary_camera_operation
    def start_detection(self) -> dict[str, Any]:
        errors: dict[str, str] = {}
        behavior = getattr(self, '_behavior', None)
        use_behavior = behavior is not None and any(behavior.status().get('enabled', {}).values())
        if use_behavior:
            try:
                self._yolo.stop()
                behavior.start()
            except (ValueError, RuntimeError, OSError) as error:
                errors['behavior'] = str(error)
        else:
            try:
                if behavior is not None: behavior.stop()
                self._yolo.start()
            except (ValueError, RuntimeError, OSError) as error:
                errors['yolo'] = str(error)
        try:
            self._thermal.set_detection(True)
        except (ValueError, RuntimeError, OSError) as error:
            errors["thermal"] = str(error)
        result = self.detection_status()
        result["errors"] = errors
        result["success"] = not errors
        result["partial"] = bool(result['partial']) or bool(errors) and bool(result["running"])
        return result

    @ordinary_camera_operation
    def stop_detection(self) -> dict[str, Any]:
        errors: dict[str, str] = {}
        if getattr(self, '_behavior', None) is not None:
            try:
                self._behavior.stop()
                self._behavior_speech.reset()
            except (ValueError, RuntimeError, OSError) as error:
                errors['behavior'] = str(error)
        try:
            self._yolo.stop()
        except (ValueError, RuntimeError, OSError) as error:
            errors["yolo"] = str(error)
        try:
            self._thermal.set_detection(False)
        except (ValueError, RuntimeError, OSError) as error:
            errors["thermal"] = str(error)
        result = self.detection_status()
        result["errors"] = errors
        result["success"] = not errors
        result["partial"] = bool(errors)
        return result

    @ordinary_camera_operation
    def toggle_detection(self) -> dict[str, Any]:
        return self.stop_detection() if self.detection_status()["running"] else self.start_detection()

    @ordinary_camera_operation
    def start_yolo(self) -> dict[str, Any]:
        """按需启动 YOLO 子进程。"""
        behavior = getattr(self, '_behavior', None)
        if behavior is not None:
            behavior.configure({'fall_enabled': False, 'fight_enabled': False, 'smoking_enabled': False})
            behavior.stop()
            self._behavior_speech.reset()
        result = self._yolo.start()
        if self._inspection_mode in ('transit', 'inspection'):
            self._yolo.inspection_request('mode', {'mode': self._inspection_mode,
                'task_id': self._inspection_task_id}, timeout=30.0)
        return result

    @ordinary_camera_operation
    def stop_yolo(self) -> dict[str, Any]:
        """停止 YOLO 子进程。"""
        return self._yolo.stop()

    @ordinary_camera_operation
    def toggle_yolo(self) -> dict[str, Any]:
        """网页按钮使用的单入口：运行则停止，否则启动。"""
        if self._yolo.status()["running"]:
            return self.stop_yolo()
        return self.start_yolo()

    @staticmethod
    def _safe_component(value: Any, name: str) -> str:
        text = str(value or "").strip()
        if (
            not text or len(text) > 80
            or any(character not in (
                "abcdefghijklmnopqrstuvwxyz"
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
            ) for character in text)
        ):
            raise ValueError(f"{name} 格式无效")
        return text

    def _evidence_path(self, *parts: str) -> Path:
        root = self.args.inspection_evidence_root.resolve()
        target = root.joinpath(*parts).resolve()
        if root not in target.parents:
            raise ValueError("现场图片路径越界")
        return target

    @staticmethod
    def _decode_jpeg(value: Any, name: str) -> bytes:
        try:
            image = base64.b64decode(str(value or ""), validate=True)
        except (ValueError, TypeError) as error:
            raise RuntimeError(f"{name} Base64 无效") from error
        if not image.startswith(b"\xff\xd8") or not image.endswith(b"\xff\xd9"):
            raise RuntimeError(f"{name} 不是有效 JPEG")
        return image

    def _store_inspection_pair(
            self, result: dict[str, Any], directory: Path,
            prefix: str) -> dict[str, Any]:
        raw = self._decode_jpeg(result.pop("raw_jpeg_base64", ""), "原图")
        annotated = self._decode_jpeg(
            result.pop("annotated_jpeg_base64", ""), "标注图")
        directory.mkdir(parents=True, exist_ok=True)
        sequence = int(result.get("sequence", 0))
        stamp = int(float(result.get("timestamp_epoch", time.time())) * 1000)
        base = f"{prefix}_{stamp}_{sequence}"
        raw_path = (directory / f"{base}_raw.jpg").resolve()
        annotated_path = (directory / f"{base}_yolo.jpg").resolve()
        if directory.resolve() not in raw_path.parents:
            raise ValueError("现场图片文件名越界")
        raw_path.write_bytes(raw)
        annotated_path.write_bytes(annotated)
        result["raw_image"] = str(raw_path)
        result["annotated_image"] = str(annotated_path)
        return result

    def _store_behavior_pair(self, result, directory, prefix):
        result = self._store_inspection_pair(result, directory, prefix + '_behavior')
        for detection in result.get('detections', []):
            detection['evidence'] = dict(raw_image=result['raw_image'],
                                         annotated_image=result['annotated_image'])
        return result

    def _store_thermal_pair(
            self, result: dict[str, Any], directory: Path,
            prefix: str) -> dict[str, Any]:
        raw = self._decode_jpeg(
            result.pop("raw_jpeg_base64", ""), "热成像原图")
        annotated = self._decode_jpeg(
            result.pop("annotated_jpeg_base64", ""), "热成像标注图")
        directory.mkdir(parents=True, exist_ok=True)
        sequence = int(result.get("sequence", 0))
        stamp = int(float(result.get("timestamp_epoch", time.time())) * 1000)
        base = f"{prefix}_{stamp}_{sequence}"
        raw_path = (directory / f"{base}_thermal_raw.jpg").resolve()
        annotated_path = (directory / f"{base}_thermal_alarm.jpg").resolve()
        if directory.resolve() not in raw_path.parents:
            raise ValueError("热成像现场图片文件名越界")
        raw_path.write_bytes(raw)
        annotated_path.write_bytes(annotated)
        result["thermal_raw_image"] = str(raw_path)
        result["thermal_annotated_image"] = str(annotated_path)
        return result

    @ordinary_camera_operation
    def set_inspection_mode(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode", "")).strip().lower()
        if mode not in {"manual", "paused", "transit", "inspection"}:
            raise ValueError("YOLO 巡检模式无效")
        task_id = str(payload.get("task_id", "")).strip()
        if task_id:
            task_id = self._safe_component(task_id, "任务 ID")
        effective_task_id = (
            "" if mode == "manual" and not task_id
            else task_id or self._inspection_task_id)
        thermal: dict[str, Any]
        try:
            thermal = self._thermal.set_mode(mode, effective_task_id)
        except (ValueError, RuntimeError, OSError) as error:
            thermal = {
                "available": False, "ready": False, "error": str(error),
                "mode": mode, "task_id": task_id,
            }
        if bool(payload.get("thermal_required")) and not thermal.get("ready"):
            raise RuntimeError(
                f"本次巡检要求热成像可用：{thermal.get('error') or '尚未获得有效热成像帧'}")
        errors = {}
        behavior = getattr(self, '_behavior', None)
        if behavior is not None:
            self._behavior_speech.reset()
            try:
                if mode in ('transit', 'inspection') and any(behavior.status().get('enabled', {}).values()):
                    self._yolo.stop()
                behavior.set_mode(mode, effective_task_id)
            except (ValueError, RuntimeError, OSError) as error:
                errors['behavior'] = str(error)
        result = {'ready': False, 'running': False}
        behavior_selected = behavior is not None and any(behavior.status().get('enabled', {}).values())
        try:
            if not behavior_selected and not self._yolo.status()["running"] and mode not in ('paused', 'manual'):
                self._yolo.start()
            if self._yolo.status()['running']:
                result = self._yolo.inspection_request(
                    "mode", {"mode": mode, "task_id": effective_task_id}, timeout=30.0)
        except (ValueError, RuntimeError, OSError) as error:
            errors['yolo'] = str(error)
        self._inspection_mode = mode
        self._inspection_task_id = effective_task_id
        behavior_status = self.behavior_status()
        return {
            **result, "success": True, "mode": mode, "task_id": self._inspection_task_id,
            "thermal": thermal, 'behavior': behavior_status, 'errors': errors,
            'partial': bool(errors) or behavior_status.get('partial', False) or (behavior_selected and not self._yolo.status()['running']),
            'running': bool(result.get('running') or behavior_status.get('running')),
            'ready': bool(result.get('ready')) and (not any(behavior_status.get('enabled', {}).values()) or behavior_status.get('ready', False)),
        }

    def inspection_state(self) -> dict[str, Any]:
        yolo_status = self._yolo.status()
        inner = {}
        if yolo_status["running"]:
            try:
                inner = self._yolo.inspection_request("state", timeout=2.0)
            except RuntimeError as error:
                inner = {"ready": False, "error": str(error)}
        if self.args.camera_type == "usb":
            source = self._video_source()
            camera = {
                "camera_type": source_kind(self._camera_sources.current),
                "device": source.info().get('device', '') if source else '',
                "camera_source": self._camera_selection(),
                "fixed_view": True,
                "direction": "current",
                "pan": None,
                "tilt": None,
                "zoom_ratio": None,
                "arrival_estimated": False,
            }
        else:
            with self._state_lock:
                camera = {
                    "camera_type": "hikvision",
                    "fixed_view": False,
                    "direction": self._inspection_direction,
                    "pan": self._last_pan,
                    "tilt": self._last_tilt,
                    "zoom_ratio": self._last_zoom_ratio,
                    "arrival_estimated": self._inspection_position_estimated,
                }
        thermal = self._thermal.status()
        behavior = self.behavior_status()
        return {
            "success": True, "mode": self._inspection_mode,
            "task_id": self._inspection_task_id,
            "running": bool(yolo_status["running"] or behavior.get('running')),
            "ready": bool(inner.get("ready", False)) and (not any(behavior.get('enabled', {}).values()) or behavior.get('ready', False)),
            "partial": bool(behavior.get('partial')) or not bool(inner.get('ready')),
            "sequence": int(inner.get("sequence", 0) or 0),
            "frame_age_seconds": inner.get("frame_age_seconds"),
            "recognition": inner.get("recognition") or {},
            "thermal": thermal,
            "display": self.display_status(thermal),
            "behavior": behavior,
            "thermal_ready": bool(thermal.get("ready")),
            "thermal_coverage_complete": not bool(thermal.get("error")),
            **camera,
        }

    def inspection_speech(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Queue fixed inspection phrases through the local offline speaker."""
        key = str(payload.get("key", "")).strip()
        if key == PATROL_SAFETY_KEY:
            if (not self._inspection_task_id or payload.get('task_id') != self._inspection_task_id
                    or self._inspection_mode not in ('transit', 'inspection')):
                return dict(success=True, key=key, queued=False)
            return dict(success=True, key=key, voice_backend='local',
                        queued=self._behavior_speech.announce_patrol_safety())
        if key not in {"thermal_high", "co2_high"}:
            raise ValueError("环境语音告警类型无效")
        speaker = getattr(self, '_behavior_speech', None)
        if speaker is None:
            raise RuntimeError("本地环境告警语音不可用")
        return dict(success=True, key=key, queued=bool(speaker.announce(key)), voice_backend='local')

    def inspection_capture(self, payload: dict[str, Any]) -> dict[str, Any]:
        task_id = self._safe_component(
            payload.get("task_id") or self._inspection_task_id, "任务 ID")
        loop_number = max(1, int(payload.get("loop_number", 1)))
        point_id = max(1, int(payload.get("route_point_id", 0)))
        direction = self._safe_component(
            payload.get("direction", "current"), "云台方向")
        duration = float(payload.get("duration_seconds", 2.0))
        behavior = getattr(self, '_behavior', None)
        behavior_result = {'detections': [], 'behavior': self.behavior_status()}
        with ThreadPoolExecutor(max_workers=3, thread_name_prefix="inspection-capture") as executor:
            behavior_status = self.behavior_status()
            suspended = bool(behavior_status.get('running') and any(behavior_status.get('enabled', {}).values()))
            yolo_future = None if suspended else executor.submit(
                self._yolo.inspection_request,
                "capture", {"duration_seconds": duration},
                timeout=max(5.0, duration + 3.0))
            thermal_future = executor.submit(self._thermal.capture, duration)
            behavior_future = executor.submit(behavior.capture, duration, task_id) if behavior is not None else None
            if yolo_future is None:
                result = {'detections': []}
                visual_error = '普通 YOLO 因行为识别互斥暂停，烟火、人脸和车牌未检测'
            else:
                try:
                    result = yolo_future.result()
                    visual_error = ''
                except (ValueError, RuntimeError, OSError) as error:
                    result = {'detections': []}
                    visual_error = str(error)
            try:
                thermal_result = thermal_future.result()
                thermal_error = ""
            except (ValueError, RuntimeError, OSError) as error:
                thermal_result = {"detections": []}
                thermal_error = str(error)
            if behavior_future is not None:
                try: behavior_result = behavior_future.result()
                except (ValueError, RuntimeError, OSError) as error:
                    behavior_result = dict(detections=[], error=str(error), partial=True, behavior=self.behavior_status())
        # A completed request cannot transfer an old task's result to a new one.
        if task_id != self._inspection_task_id:
            behavior_result = dict(detections=[], error='行为巡检任务已变化', partial=True, behavior=self.behavior_status())
        visual_detections = list(result.get("detections") or [])
        thermal_detections = list(thermal_result.get("detections") or [])
        should_save = bool(payload.get("save_images")) or bool(visual_detections)
        if should_save and result.get('raw_jpeg_base64') and result.get('annotated_jpeg_base64'):
            directory = self._evidence_path(
                task_id, f"loop_{loop_number:04d}", f"point_{point_id}")
            result = self._store_inspection_pair(
                result, directory, direction)
        else:
            result.pop("raw_jpeg_base64", None)
            result.pop("annotated_jpeg_base64", None)
            result["raw_image"] = ""
            result["annotated_image"] = ""
        should_save_thermal = bool(payload.get("save_images")) or bool(thermal_detections)
        if thermal_result.get("raw_jpeg_base64") and should_save_thermal:
            directory = self._evidence_path(
                task_id, f"loop_{loop_number:04d}", f"point_{point_id}")
            thermal_result = self._store_thermal_pair(
                thermal_result, directory, direction)
        else:
            thermal_result.pop("raw_jpeg_base64", None)
            thermal_result.pop("annotated_jpeg_base64", None)
            thermal_result["thermal_raw_image"] = ""
            thermal_result["thermal_annotated_image"] = ""
        behavior_detections = list(behavior_result.get('detections') or [])
        if (behavior_result.get('raw_jpeg_base64') and behavior_result.get('annotated_jpeg_base64')
                and (payload.get('save_images') or behavior_detections)):
            directory = self._evidence_path(task_id, f'loop_{loop_number:04d}', f'point_{point_id}')
            behavior_result = self._store_behavior_pair(behavior_result, directory, direction)
        else:
            behavior_result.pop('raw_jpeg_base64', None)
            behavior_result.pop('annotated_jpeg_base64', None)
        result["detections"] = visual_detections + thermal_detections + behavior_detections
        result['behavior'] = {**(behavior_result.get('behavior') or self.behavior_status()),
            'raw_image': behavior_result.get('raw_image', ''),
            'annotated_image': behavior_result.get('annotated_image', ''),
            'source_epoch': behavior_result.get('source_epoch'), 'stamp_ns': behavior_result.get('stamp_ns'),
            'fresh': bool(behavior_result.get('fresh')), 'error': behavior_result.get('error')}
        result['behavior_error'] = behavior_result.get('error', '')
        result['visual_error'] = visual_error
        result['partial'] = bool(visual_error or thermal_error or behavior_result.get('partial'))
        result["thermal_raw_image"] = thermal_result.get("thermal_raw_image", "")
        result["thermal_annotated_image"] = thermal_result.get("thermal_annotated_image", "")
        result["thermal"] = {
            key: value for key, value in thermal_result.items()
            if key not in {"detections", "raw_jpeg_base64", "annotated_jpeg_base64"}
        }
        result["thermal_error"] = thermal_error
        return {"success": True, **result}

    def inspection_events(self, since: int) -> dict[str, Any]:
        result = {"mode": self._inspection_mode, "sequence": int(since), "events": []}
        try:
            if self._yolo.status()["running"]:
                result = self._yolo.inspection_request(
                    f"events?since={max(0, int(since))}", timeout=3.0)
        except (ValueError, RuntimeError, OSError) as error:
            result['visual_error'] = str(error)
        task_id = self._inspection_task_id
        events = []
        for event in result.get("events", []):
            event = dict(event)
            if event.get("detections") and task_id:
                directory = self._evidence_path(task_id, "transit")
                event = self._store_inspection_pair(event, directory, "transit")
            else:
                event.pop("raw_jpeg_base64", None)
                event.pop("annotated_jpeg_base64", None)
                event["raw_image"] = ""
                event["annotated_image"] = ""
            events.append(event)
        behavior = getattr(self, '_behavior', None)
        if behavior is not None and task_id and self._inspection_mode in ('transit', 'inspection'):
            for event in behavior.events(task_id):
                if task_id != self._inspection_task_id or event.get('task_id') != task_id:
                    continue
                directory = self._evidence_path(task_id, 'transit')
                event = self._store_behavior_pair(event, directory, 'transit')
                event['behavior'] = dict(source_epoch=event['source_epoch'], stamp_ns=event['stamp_ns'], fresh=True)
                events.append(event)
        try:
            pipeline = self._camera_sources.current['pipeline']
            thermal_sequences = self._thermal_event_sequences
            thermal_result = self._thermal.events(thermal_sequences[pipeline])
            thermal_sequences[pipeline] = int(
                thermal_result.get("sequence", thermal_sequences[pipeline]))
        except (ValueError, RuntimeError, OSError):
            thermal_result = {"events": []}
        for thermal_event in thermal_result.get("events", []):
            thermal_event = dict(thermal_event)
            detection = {
                key: value for key, value in thermal_event.items()
                if key not in {"raw_jpeg_base64", "annotated_jpeg_base64", "sequence"}
            }
            event = {
                "mode": thermal_event.get("mode", self._inspection_mode),
                "timestamp_epoch": thermal_event.get("timestamp_epoch", time.time()),
                "sequence": thermal_event.get("sequence", 0),
                "detections": [detection],
                "raw_jpeg_base64": thermal_event.get("raw_jpeg_base64", ""),
                "annotated_jpeg_base64": thermal_event.get("annotated_jpeg_base64", ""),
                "raw_image": "", "annotated_image": "",
            }
            if task_id and event["raw_jpeg_base64"]:
                directory = self._evidence_path(task_id, "transit")
                event = self._store_thermal_pair(event, directory, "transit")
            else:
                event.pop("raw_jpeg_base64", None)
                event.pop("annotated_jpeg_base64", None)
                event["thermal_raw_image"] = ""
                event["thermal_annotated_image"] = ""
            events.append(event)
        return {
            "success": True, "mode": result.get("mode", self._inspection_mode),
            "sequence": int(result.get("sequence", since)), "events": events,
            'behavior': self.behavior_status(), 'visual_error': result.get('visual_error', ''),
        }

    def inspection_evidence(self, requested_path: str) -> bytes:
        path = Path(str(requested_path)).expanduser().resolve()
        root = self.args.inspection_evidence_root.resolve()
        if root not in path.parents or path.suffix.lower() not in {".jpg", ".jpeg"}:
            raise ValueError("现场图片路径无效")
        if not path.is_file():
            raise ValueError("现场图片不存在")
        image = path.read_bytes()
        if not image.startswith(b"\xff\xd8") or not image.endswith(b"\xff\xd9"):
            raise ValueError("现场图片格式无效")
        return image

    def _capture_jpeg_locked(self) -> bytes:
        """V40 不可用时的唯一抓图路径，不能由 HTTP 请求直接并发调用。"""
        jpeg_para = JpegPara(255, 0)
        returned = c_uint(0)
        if not self.sdk.NET_DVR_CaptureJPEGPicture_NEW(self.user_id, self.args.channel, byref(jpeg_para), self._jpeg_buffer, sizeof(self._jpeg_buffer), byref(returned)):
            raise self._error("抓图失败")
        if returned.value == 0:
            raise RuntimeError("设备返回空 JPEG")
        return self._jpeg_buffer.raw[: returned.value]

    def _restart_video_once(self) -> dict[str, Any]:
        """执行一次完整重连；调用方负责保证同一时刻只有一个重连事务。"""
        if self.args.camera_type == "usb":
            self._stop_video_locked()
            source = self._start_usb_source()
            self._wait_for_first_frame(source)
            return source.info()
        self.executor.call(self._stop_video_locked, timeout=5)
        error: BaseException | None = None
        if self.args.video_backend != "jpeg":
            try:
                self.executor.call(self._start_v40_locked, timeout=8)
                source = self._video_source()
                if source is None:
                    raise RuntimeError("V40 视频源未建立")
                self._wait_for_first_frame(source)
            except BaseException as caught:
                error = caught
                self.executor.call(self._stop_video_locked, timeout=5)
        if self._video_source() is None:
            if self.args.video_backend == "v40":
                raise RuntimeError(f"V40 视频重连失败: {error}")
            self.executor.call(self._start_jpeg_locked, timeout=3)
            source = self._video_source()
            if source is None:
                raise RuntimeError("JPEG 回退视频源未建立")
            self._wait_for_first_frame(source)
        source = self._video_source()
        if source is None:
            raise RuntimeError("视频重连后未获得视频源")
        return source.info()

    def restart_video(self) -> dict[str, Any]:
        """供网页恢复视频；重复请求复用正在进行的同一次重连。"""
        if not self._video_restart_lock.acquire(blocking=False):
            source = self._video_source()
            if source is None:
                return {"backend": "none", "error": "视频正在重连", "restarting": True}
            result = source.info()
            result["restarting"] = True
            return result
        try:
            self._set_video_restarting(True)
            return self._restart_video_once()
        finally:
            self._set_video_restarting(False)
            self._video_restart_lock.release()

    def next_frame(self, version: int) -> tuple[int, bytes]:
        """向 MJPEG 客户端提供下一张共享帧，切换期间等待新源接管。"""
        deadline = time.monotonic() + getattr(self.args, "video_start_timeout", 10.0)
        with self._video_ready:
            source = self.video
            while source is None and self._video_restarting.is_set() and not self._closed.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._video_ready.wait(remaining)
                source = self.video
        if source is None:
            if self._video_restarting.is_set():
                raise TimeoutError("等待视频重建首帧超时")
            raise RuntimeError("视频服务未启动")
        result = source.next_frame(version)
        if source is not self._video_source():
            raise RuntimeError('普通摄像头来源已切换，请重新连接画面')
        return result

    def snapshot(self) -> bytes:
        """直接返回视频源缓存帧，不额外占用 SDK。"""
        source = self._video_source()
        if source is None:
            raise RuntimeError("视频服务未启动")
        image = source.latest_frame()
        if source is not self._video_source():
            raise RuntimeError('普通摄像头来源已切换，请重新连接画面')
        return image

    def status(self) -> dict[str, Any]:
        """状态读取只访问缓存，不会和实时解码或控制请求争用 SDK。"""
        with self._state_lock:
            active = self._active_action
            capabilities = (
                {
                    "ptz": False,
                    "optical_zoom": False,
                    "ir": False,
                    "wiper": False,
                    "yolo": True,
                    "snapshot": True,
                    "thermal": True,
                }
                if self.args.camera_type == "usb"
                else {
                    "ptz": True,
                    "optical_zoom": True,
                    "ir": True,
                    "wiper": True,
                    "yolo": True,
                    "snapshot": True,
                    "thermal": True,
                }
            )
            result = {
                "connected": self._video_source() is not None if self.args.camera_type == "usb" else self.user_id >= 0,
                "camera_type": source_kind(self._camera_sources.current) if self.args.camera_type=='usb' else self.args.camera_type,
                "active": active,
                "tilt": self._last_tilt,
                "speed": self.args.speed,
                "channel": self.args.channel,
                "ir_mode": self._last_ir_mode,
                "wiper_power": self._wiper_on,
                "capabilities": capabilities,
            }
        result["thermal"] = self._thermal.status()
        result["detection"] = self.detection_status()
        result["yolo"] = result["detection"]["yolo"]
        result['camera_source'] = self._camera_selection()
        result['display'] = self.display_status(result['thermal'])
        source = self._video_source()
        restarting = self._video_restarting.is_set()
        if source is None:
            result["video"] = {
                "backend": "none",
                "error": None if restarting else "视频未启动",
                "restarting": restarting,
            }
        else:
            result["video"] = {**source.info(), "restarting": restarting}
        if self.args.camera_type == 'usb':
            video = result['video']
            result['connected'] = bool(source is not None and not restarting
                and not video.get('error') and video.get('encoded_frames', 0) > 0)
        return result

    def _close_locked(self) -> None:
        """按安全顺序停止云台、视频并释放 SDK 登录会话。"""
        try:
            self._stop_motion_locked()
        except BaseException:
            pass
        try:
            self._stop_video_locked()
        except BaseException:
            pass
        if self.sdk is not None:
            if self.user_id >= 0:
                self.sdk.NET_DVR_Logout(self.user_id)
                self.user_id = -1
            self.sdk.NET_DVR_Cleanup()
        # 保留旧 V40 源到进程退出。即使极少数 SDK 线程在 Cleanup 后才返回，也不能让
        # ctypes 回调闭包提前被垃圾回收并跳入无效地址。

    def close(self) -> None:
        """停止后台任务与设备资源；重复调用安全。"""
        if self._closed.is_set():
            return
        self._closed.set()
        with self._camera_change_lock:
            self._close_resources()

    def _close_resources(self) -> None:
        self.c12_recorder.close()
        self.c12_control.close()
        self.c12.close()
        if getattr(self, '_behavior', None) is not None:
            self._behavior.close()
            self._behavior_speech.close()
            self._behavior_feeder.join(timeout=1)
        with self._recognition_lock:
            if self._recognition_database is not None:
                self._recognition_database.close()
                self._recognition_database = None
        # 仅关闭本网页会话已经开启的雨刮。该路径不用 HCNetSDK，不会和视频回调并发。
        try:
            with self._state_lock:
                wiper_on = self._wiper_on
            if wiper_on:
                self._set_wiper_power_http(False, allow_closing=True)
        except BaseException:
            pass
        try:
            self._yolo.stop()
        except BaseException:
            pass
        with self._video_restart_lock:
            try:
                self.executor.call(self._close_locked, timeout=8, urgent=True)
            except BaseException:
                pass
        self.executor.close()
        if self._position_worker is not None:
            self._position_worker.join(timeout=1)


class CameraRequestHandler(BaseHTTPRequestHandler):
    """静态页面、MJPEG 与本地控制 API 的 HTTP 处理器。"""

    server_version = "HikvisionWeb/2.0"
    protocol_version = "HTTP/1.1"

    def end_headers(self) -> None:
        """Allow the robot UI, served on another local port, to use this API."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header('Access-Control-Expose-Headers', 'X-Behavior-Source-Epoch, X-Behavior-Stamp-Ns')
        super().end_headers()

    def do_OPTIONS(self) -> None:
        """Answer browser preflight requests for JSON PTZ controls."""
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Content-Length", "0")
        self.end_headers()

    @property
    def gateway(self) -> CameraGateway:
        return self.server.gateway  # type: ignore[attr-defined]

    def log_message(self, format: str, *args: object) -> None:
        """抑制高频正常轮询日志；304/YOLO 首帧 503 都是轮询状态。"""
        if len(args) > 1:
            status = str(args[1])
            path = str(args[0]) if args else ""
            if status in {"200", "304"} or (status == "503" and "/api/yolo/frame.jpg" in path):
                return
            print("[Web] " + format % args)

    def send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        """向 fetch 请求写入 JSON，并吞掉浏览器主动断开产生的 BrokenPipe。"""
        body = json.dumps(payload, ensure_ascii=False).encode()
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def send_image(self, status: HTTPStatus, image: bytes, *, sequence: int | None = None,
                   behavior_epoch=None, behavior_stamp=None) -> None:
        """发送不缓存的 JPEG；YOLO 单帧代理和普通快照共用该响应格式。"""
        try:
            self.send_response(status)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            if sequence is not None:
                self.send_header("X-YOLO-Sequence", str(sequence))
            if behavior_epoch is not None:
                self.send_header('X-Behavior-Source-Epoch', str(behavior_epoch))
                self.send_header('X-Behavior-Stamp-Ns', str(behavior_stamp))
            self.send_header("Content-Length", str(len(image)))
            self.end_headers()
            self.wfile.write(image)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def send_not_modified(self, sequence: int) -> None:
        """返回单帧轮询的 304，保留序号但不发送旧图像。"""
        try:
            self.send_response(HTTPStatus.NOT_MODIFIED)
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("X-YOLO-Sequence", str(sequence))
            self.send_header("Content-Length", "0")
            self.end_headers()
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def _read_json(self, max_bytes: int = 16 * 1024) -> dict[str, Any]:
        """限制控制请求体大小，防止错误客户端占用 HTTP 工作线程。"""
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("Content-Length 无效") from error
        if not 0 <= length <= max(0, int(max_bytes)):
            raise ValueError("请求体过大")
        payload = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(payload, dict):
            raise ValueError("请求 JSON 必须是对象")
        return payload

    def _send_page(self) -> None:
        content = PAGE_FILE.read_bytes()
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def stream_mjpeg(self) -> None:
        """一条浏览器连接只等待共享最新帧，慢客户端不影响解码线程。"""
        slots = self.server.stream_slots  # type: ignore[attr-defined]
        if not slots.acquire(blocking=False):
            self.send_json(HTTPStatus.TOO_MANY_REQUESTS, {"error": "实时视频客户端数量已达上限"})
            return
        try:
            # 单个浏览器长期不读取时，写入超时后释放 HTTP 线程和视频连接槽。
            self.connection.settimeout(self.server.stream_write_timeout)  # type: ignore[attr-defined]
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            version = 0
            while True:
                version, image = self.gateway.next_frame(version)
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(image)}\r\n\r\n".encode())
                self.wfile.write(image)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return
        except (RuntimeError, TimeoutError) as error:
            # 切换日夜模式时旧 MJPEG 连接必然被新源替换，这不是需要提示用户的故障。
            if not self.gateway.is_video_restarting():
                print(f"[Web] 视频流结束: {error}")
        finally:
            slots.release()

    def stream_yolo_mjpeg(self) -> None:
        """原样转发 YOLO 子进程的 MJPEG，避免逐帧 fetch/Blob 开销。"""
        slots = self.server.stream_slots  # type: ignore[attr-defined]
        if not slots.acquire(blocking=False):
            self.send_json(HTTPStatus.TOO_MANY_REQUESTS, {"error": "实时视频客户端数量已达上限"})
            return
        upstream = None
        try:
            upstream = self.gateway.open_yolo_stream()
            self.connection.settimeout(self.server.stream_write_timeout)  # type: ignore[attr-defined]
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Connection", "close")
            self.end_headers()
            # read1 返回当前已到达的数据；read(64KiB) 可能等待多帧填满缓冲区才返回。
            read_chunk = getattr(upstream, "read1", upstream.read)
            while True:
                chunk = read_chunk(64 * 1024)
                if not chunk:
                    return
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, socket.timeout, TimeoutError):
            return
        except (RuntimeError, OSError, urllib.error.URLError) as error:
            print(f"[Web] YOLO 结果流结束: {error}")
        finally:
            if upstream is not None:
                try:
                    upstream.close()
                except OSError:
                    pass
            slots.release()

    def stream_thermal_mjpeg(self) -> None:
        """Proxy the loopback thermal MJPEG stream without owning HCUSBSDK."""
        slots = self.server.stream_slots  # type: ignore[attr-defined]
        if not slots.acquire(blocking=False):
            self.send_json(HTTPStatus.TOO_MANY_REQUESTS, {"error": "实时视频客户端数量已达上限"})
            return
        upstream = None
        try:
            upstream = self.gateway.open_thermal_stream()
            self.connection.settimeout(self.server.stream_write_timeout)  # type: ignore[attr-defined]
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                upstream.headers.get(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame"))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Connection", "close")
            self.end_headers()
            read_chunk = getattr(upstream, "read1", upstream.read)
            while True:
                chunk = read_chunk(64 * 1024)
                if not chunk:
                    return
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, socket.timeout, TimeoutError):
            return
        except (RuntimeError, OSError, urllib.error.URLError) as error:
            print(f"[Web] 热成像流结束: {error}")
            if upstream is None:
                # The browser can only run its image-error/retry path when the
                # startup race has a real HTTP response. Returning silently on
                # this HTTP/1.1 connection leaves <img> waiting indefinitely.
                self.close_connection = True
                self.send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"error": str(error)})
        finally:
            if upstream is not None:
                try:
                    upstream.close()
                except OSError:
                    pass
            slots.release()

    def stream_browser_preview(self, source, profile, *, still=False):
        if profile not in PREVIEW_PROFILES or source not in ('raw', 'yolo', 'thermal'):
            raise ValueError('无效的网页预览档位或来源')
        cache = self.server.browser_previews
        if still:
            sequence, image = cache.frame(source, profile)
            self.send_image(HTTPStatus.OK, image, sequence=sequence)
            return
        if not self.server.stream_slots.acquire(blocking=False):
            self.send_json(HTTPStatus.TOO_MANY_REQUESTS, {'error': '视频连接数已满'})
            return
        started = False
        try:
            # Fetch before headers, so unavailable sources return proper errors.
            sequence, image = cache.frame(source, profile)
            self.connection.settimeout(min(2.0, self.server.stream_write_timeout))
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 32768)
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            started = True
            while True:
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(image)).encode('ascii') + b'\r\n\r\n' + image + b'\r\n')
                self.wfile.flush()
                time.sleep(1.0 / PREVIEW_PROFILES[profile].fps)
                sequence, image = cache.frame(source, profile)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return
        except (RuntimeError, OSError, TimeoutError) as error:
            # After streaming starts, end the response rather than write JSON
            # into an MJPEG body. The browser may retry the same bounded stream.
            if not started:
                self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(error)})
            self.close_connection = True
        finally:
            self.server.stream_slots.release()

    def stream_c12(self, channel, *, still=False):
        source = self.gateway.c12.sources[channel]
        if still:
            self.send_image(HTTPStatus.OK, source.next_frame()[1])
            return
        # Separate slots keep C12 viewers from consuming the original cameras' budget.
        if not self.server.c12_stream_slots.acquire(blocking=False):
            self.send_json(HTTPStatus.TOO_MANY_REQUESTS, {'error': 'C12 视频连接数已满'})
            return
        started = False
        try:
            version, image = source.next_frame()
            self.connection.settimeout(self.server.stream_write_timeout)
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            started = True
            while True:
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(image)).encode('ascii') + b'\r\n\r\n' + image + b'\r\n')
                self.wfile.flush()
                version, image = source.next_frame(version)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass
        except (RuntimeError, OSError, TimeoutError) as error:
            if not started:
                self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(error)})
        finally:
            self.close_connection = True
            self.server.c12_stream_slots.release()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == '/api/display/status':
                self.send_json(HTTPStatus.OK, self.gateway.display_status())
            elif path == '/api/c12/commands':
                self.send_json(HTTPStatus.OK, C12_COMMANDS)
            elif path == '/api/c12/recording':
                self.send_json(HTTPStatus.OK, self.gateway.c12_recorder.status())
            elif path == '/api/c12/status':
                self.send_json(HTTPStatus.OK, self.gateway.c12.status())
            elif path in ('/api/c12/visible/stream.mjpg', '/api/c12/infrared/stream.mjpg',
                          '/api/c12/visible/frame.jpg', '/api/c12/infrared/frame.jpg'):
                self.stream_c12(path.split('/')[3], still=path.endswith('frame.jpg'))
            elif path in ('/api/preview/stream', '/api/preview/frame.jpg',
                          '/api/display/stream', '/api/display/frame.jpg'):
                query = parse_qs(urlparse(self.path).query)
                self.stream_browser_preview(query.get('source', ['raw'])[0],
                    query.get('profile', ['remote'])[0], still=path.endswith('frame.jpg'))
            elif path == "/":
                self._send_page()
            elif path == "/api/status":
                self.send_json(HTTPStatus.OK, self.gateway.status())
            elif path == "/api/camera/source":
                self.send_json(HTTPStatus.OK, self.gateway.camera_source_status())
            elif path == "/api/stream":
                self.stream_mjpeg()
            elif path == "/api/yolo/stream.mjpg":
                self.stream_yolo_mjpeg()
            elif path == "/api/thermal/stream.mjpg":
                self.stream_thermal_mjpeg()
            elif path == "/api/snapshot":
                image = self.gateway.snapshot()
                self.send_image(HTTPStatus.OK, image)
            elif path == "/api/yolo/frame.jpg":
                query = parse_qs(urlparse(self.path).query)
                try:
                    since = max(0, int(query.get("since", ["0"])[0]))
                except (TypeError, ValueError):
                    self.send_json(HTTPStatus.BAD_REQUEST, {"error": "since 必须是非负整数"})
                    return
                sequence, image = self.gateway.get_yolo_frame(since)
                if image is None:
                    self.send_not_modified(sequence)
                else:
                    self.send_image(HTTPStatus.OK, image, sequence=sequence)
            elif path == "/api/thermal/frame.jpg":
                self.send_image(HTTPStatus.OK, self.gateway.thermal_frame())
            elif path == "/api/thermal/raw.jpg":
                self.send_image(HTTPStatus.OK, self.gateway.thermal_frame(raw=True))
            elif path == "/api/image":
                self.send_json(HTTPStatus.GONE, {"error": "图像参数接口已禁用"})
            elif path == "/api/wiper":
                self.send_json(HTTPStatus.OK, self.gateway.get_wiper_settings())
            elif path == "/api/yolo":
                self.send_json(HTTPStatus.OK, self.gateway.get_yolo_status())
            elif path == "/api/detection":
                self.send_json(HTTPStatus.OK, self.gateway.detection_status())
            elif path == '/api/behavior/status':
                self.send_json(HTTPStatus.OK, self.gateway.behavior_status())
            elif path == '/api/behavior/frame.jpg':
                query = parse_qs(urlparse(self.path).query)
                image, epoch, stamp = self.gateway._behavior.frame(query.get('epoch', [None])[0])
                self.send_image(HTTPStatus.OK, image, behavior_epoch=epoch, behavior_stamp=stamp)
            elif path == "/api/thermal/status":
                self.send_json(HTTPStatus.OK, self.gateway.thermal_status())
            elif path == "/api/thermal/config":
                query = parse_qs(urlparse(self.path).query)
                self.send_json(
                    HTTPStatus.OK,
                    self.gateway.thermal_config(query.get("task_id", [""])[0]))
            elif path == "/api/inspection/state":
                self.send_json(HTTPStatus.OK, self.gateway.inspection_state())
            elif path == "/api/inspection/events":
                query = parse_qs(urlparse(self.path).query)
                try:
                    since = max(0, int(query.get("since", ["0"])[0]))
                except (TypeError, ValueError):
                    raise ValueError("since 必须是非负整数")
                self.send_json(
                    HTTPStatus.OK, self.gateway.inspection_events(since))
            elif path == "/api/inspection/evidence":
                query = parse_qs(urlparse(self.path).query)
                image = self.gateway.inspection_evidence(
                    query.get("path", [""])[0])
                self.send_image(HTTPStatus.OK, image)
            elif path == "/api/recognition/status":
                self.send_json(HTTPStatus.OK, self.gateway.recognition_status())
            elif path == "/api/recognition/people":
                self.send_json(HTTPStatus.OK, self.gateway.recognition_people())
            elif path == "/api/recognition/vehicles":
                self.send_json(HTTPStatus.OK, self.gateway.recognition_vehicles())
            elif path == "/api/recognition/events":
                self.send_json(HTTPStatus.OK, self.gateway.recognition_events(
                    parse_qs(urlparse(self.path).query)))
            elif path == "/api/recognition/audit":
                self.send_json(HTTPStatus.OK, self.gateway.recognition_audit(
                    parse_qs(urlparse(self.path).query)))
            elif path == "/api/recognition/summary":
                query = parse_qs(urlparse(self.path).query)
                self.send_json(HTTPStatus.OK, self.gateway.recognition_summary(
                    query.get('task_id', [''])[0]))
            elif path == "/api/recognition/evidence":
                query = parse_qs(urlparse(self.path).query)
                self.send_image(HTTPStatus.OK, self.gateway.recognition_evidence(
                    query.get('path', [''])[0]))
            else:
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
        except IsapiRequestError as error:
            self.send_json(HTTPStatus.BAD_GATEWAY, {"error": str(error), "upstream_status": error.status})
        except (ValueError, RuntimeError, TimeoutError, OSError) as error:
            self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(error)})
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            max_bytes = (
                32 * 1024 * 1024
                if path == "/api/recognition/person/upsert"
                else 16 * 1024)
            payload = self._read_json(max_bytes)
            if path == '/api/c12/recording':
                operation = payload.get('operation')
                if operation == 'start':
                    if self.gateway.camera_source_status()['config']['pipeline'] != 'c12':
                        raise ValueError('请先切换到 C12 相机链路')
                    result = self.gateway.c12_recorder.start()
                elif operation == 'stop':
                    result = self.gateway.c12_recorder.stop()
                else:
                    raise ValueError('无效录像操作')
            elif path == '/api/c12/motion':
                if payload.get('operation') != 'stop' and self.gateway.camera_source_status()['config']['pipeline'] != 'c12':
                    raise ValueError('请先切换到 C12 相机链路')
                result = self.gateway.c12_control.motion(payload)
            elif path == '/api/c12/control':
                if self.gateway.camera_source_status()['config']['pipeline'] != 'c12':
                    raise ValueError('请先切换到 C12 相机链路')
                result = self.gateway.c12_control.execute(payload)
            elif path == '/api/behavior/config':
                result = self.gateway.configure_behavior(payload)
            elif path == "/api/camera/source":
                result = self.gateway.configure_camera_source(payload)
            elif path == "/api/ptz":
                operation = payload.get("operation")
                token = payload.get("token") or ""
                if operation == "start":
                    result = self.gateway.start_motion(str(payload.get("action", "")), payload.get("speed"), str(token))
                elif operation == "keepalive":
                    result = self.gateway.keepalive(str(payload.get("action", "")), str(token))
                elif operation == "stop":
                    result = self.gateway.stop_motion(str(token) or None)
                else:
                    raise ValueError("无效云台操作")
            elif path == "/api/ir":
                result = self.gateway.set_ir_mode(payload.get("mode"))
            elif path == "/api/image":
                self.send_json(HTTPStatus.GONE, {"error": "图像参数接口已禁用"})
                return
            elif path == "/api/wiper":
                operation = payload.get("operation")
                if operation == "power":
                    result = self.gateway.set_wiper_power(payload.get("enabled"))
                elif operation == "config":
                    result = self.gateway.update_wiper_settings(payload.get("mode"), payload.get("interval_seconds"), payload.get("duration_minutes"))
                else:
                    raise ValueError("无效雨刮操作")
            elif path == "/api/video":
                if payload.get("operation") != "restart":
                    raise ValueError("无效视频操作")
                result = self.gateway.restart_video()
            elif path == "/api/yolo":
                operation = str(payload.get("operation", ""))
                if operation == "toggle":
                    result = self.gateway.toggle_yolo()
                elif operation == "start":
                    result = self.gateway.start_yolo()
                elif operation == "stop":
                    result = self.gateway.stop_yolo()
                else:
                    raise ValueError("无效 YOLO 操作")
            elif path == "/api/detection":
                operation = str(payload.get("operation", ""))
                if operation == "toggle":
                    result = self.gateway.toggle_detection()
                elif operation == "start":
                    result = self.gateway.start_detection()
                elif operation == "stop":
                    result = self.gateway.stop_detection()
                else:
                    raise ValueError("无效识别操作")
            elif path == "/api/thermal/config":
                result = self.gateway.update_thermal_config(payload)
            elif path == "/api/inspection/mode":
                result = self.gateway.set_inspection_mode(payload)
            elif path == "/api/inspection/ptz":
                result = self.gateway.inspection_ptz(payload)
            elif path == "/api/inspection/capture":
                result = self.gateway.inspection_capture(payload)
            elif path == "/api/inspection/speech":
                result = self.gateway.inspection_speech(payload)
            elif path.startswith("/api/recognition/"):
                result = self.gateway.recognition_write(
                    path.removeprefix('/api/recognition/'), payload)
            else:
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
                return
            self.send_json(HTTPStatus.OK, result)
        except IsapiRequestError as error:
            self.send_json(HTTPStatus.BAD_GATEWAY, {"error": str(error), "upstream_status": error.status})
        except TimeoutError as error:
            self.send_json(HTTPStatus.GATEWAY_TIMEOUT, {"error": str(error)})
        except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as error:
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})


class CameraWebServer(ThreadingHTTPServer):
    """HTTP 工作线程为守护线程，断开浏览器不会阻塞服务关闭。"""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], handler: type[BaseHTTPRequestHandler], max_stream_clients: int = 4, stream_write_timeout: float = 10.0) -> None:
        super().__init__(address, handler)
        # 限制 MJPEG 长连接数量，防止异常重连耗尽 ThreadingHTTPServer 的工作线程。
        self.stream_slots = threading.BoundedSemaphore(max_stream_clients)
        self.c12_stream_slots = threading.BoundedSemaphore(8)
        self.stream_write_timeout = stream_write_timeout
        self.browser_previews = PreviewCache({
            'raw': lambda: self.gateway.next_frame(-1)[1],
            'yolo': lambda: self.gateway.get_yolo_frame(0)[1],
            'thermal': lambda: self.gateway.display_thermal_frame(),
        }, generation=lambda: self.gateway.display_generation())


def ensure_player_library_path(sdk_dir: Path) -> None:
    """在进程启动前配置 PlayM4 依赖路径，等效于官方 run_linux_preview.sh。"""
    marker = "HIKVISION_WEB_LOADER_READY"
    paths = os.environ.get("LD_LIBRARY_PATH", "").split(":")
    if os.environ.get(marker) == "1" or str(sdk_dir) in paths:
        return
    environment = os.environ.copy()
    prefix = f"{sdk_dir}:{sdk_dir / 'HCNetSDKCom'}"
    environment["LD_LIBRARY_PATH"] = prefix + (f":{environment['LD_LIBRARY_PATH']}" if environment.get("LD_LIBRARY_PATH") else "")
    environment[marker] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], environment)


class YoloProcessManager:
    """按需启动独立 YOLO 子进程，并提供其本地 MJPEG 结果流状态。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self._lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None
        self._stdout_thread: threading.Thread | None = None
        self._logs: deque[str] = deque(maxlen=20)
        self._log_seq = 0
        self._session_id = 0
        self._started_at = 0.0
        self._last_exit_code: int | None = None
        self._last_error = ""
        # 由最近一次 healthz 轮询填充；网页只在 frames > 0 时切换到 YOLO。
        self._last_health: dict[str, Any] = {}

    def _arg(self, name: str, default: Any) -> Any:
        """从参数对象读取值；测试环境缺省时回退默认值。"""
        return getattr(self.args, name, default)

    def _resolve_yolo_python(self) -> Path:
        """默认复用当前解释器，使整个迁移包只需一个 Conda 环境。"""
        env_name = self._arg("yolo_conda_env", DEFAULT_YOLO_CONDA_ENV)
        current_python = Path(sys.executable).resolve()
        conda_prefix = os.environ.get("CONDA_PREFIX")
        if env_name in {"", "current"} or (conda_prefix and Path(conda_prefix).name == env_name):
            return current_python
        candidates = []
        if conda_prefix:
            candidates.append(Path(conda_prefix).resolve().parent / env_name / "bin" / "python")
        conda_exe = os.environ.get("CONDA_EXE")
        if conda_exe:
            candidates.append(Path(conda_exe).resolve().parent.parent / "envs" / env_name / "bin" / "python")
        for candidate in candidates:
            if candidate.is_file():
                return candidate
        raise RuntimeError(f"未找到 YOLO 解释器: {env_name}")

    def _stream_url(self) -> str:
        return f"http://{self._arg('yolo_host', DEFAULT_YOLO_HOST)}:{self._arg('yolo_port', DEFAULT_YOLO_PORT)}/stream.mjpg"

    def _stream_source_url(self) -> str:
        """返回服务端连接用的子进程流地址，通配符监听地址改用回环地址。"""
        return f"http://{self._yolo_connect_host()}:{self._arg('yolo_port', DEFAULT_YOLO_PORT)}/stream.mjpg"

    def _yolo_connect_host(self) -> str:
        """监听地址为通配符时，代理改用本机回环地址连接子进程。"""
        host = str(self._arg("yolo_host", DEFAULT_YOLO_HOST))
        return "127.0.0.1" if host in {"0.0.0.0", "::", "[::]"} else host

    def _frame_url(self, since: int = 0) -> str:
        """返回子进程单帧地址；该地址只供服务端代理使用。"""
        return f"http://{self._yolo_connect_host()}:{self._arg('yolo_port', DEFAULT_YOLO_PORT)}/frame.jpg?since={max(0, int(since))}"

    def _health_url(self) -> str:
        return f"http://{self._yolo_connect_host()}:{self._arg('yolo_port', DEFAULT_YOLO_PORT)}/healthz"

    def _inspection_url(self, path: str) -> str:
        return (
            f"http://{self._yolo_connect_host()}:"
            f"{self._arg('yolo_port', DEFAULT_YOLO_PORT)}"
            f"/inspection/{path.lstrip('/')}"
        )

    def inspection_request(
            self, path: str, payload: dict[str, Any] | None = None,
            timeout: float = 5.0) -> dict[str, Any]:
        """Call the child process inspection API through loopback only."""
        deadline = time.monotonic() + max(0.5, timeout)
        last_error: BaseException | None = None
        while time.monotonic() < deadline:
            with self._lock:
                process = self._process
            if process is None or process.poll() is not None:
                raise RuntimeError("YOLO 进程未运行") from last_error
            data = None
            headers = {"Accept": "application/json"}
            method = "GET"
            if payload is not None:
                data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                headers["Content-Type"] = "application/json; charset=utf-8"
                method = "POST"
            request = urllib.request.Request(
                self._inspection_url(path), data=data, headers=headers,
                method=method)
            try:
                opener = urllib.request.build_opener(
                    urllib.request.ProxyHandler({}))
                remaining = max(0.2, deadline - time.monotonic())
                # capture_window() intentionally blocks for the requested
                # sampling duration.  Giving it the generic two-second probe
                # timeout races the default two-second capture and caused the
                # client to disconnect just as the result became available.
                # A capture therefore owns the remaining operation budget;
                # short probes keep the old cap so startup retries stay fast.
                request_timeout = (
                    remaining if path == "capture" else min(2.0, remaining)
                )
                with opener.open(
                        request,
                        timeout=request_timeout,
                ) as response:
                    body = response.read(64 * 1024 * 1024 + 1)
                if len(body) > 64 * 1024 * 1024:
                    raise RuntimeError("YOLO 巡检响应过大")
                result = json.loads(body or b"{}")
                if not isinstance(result, dict):
                    raise RuntimeError("YOLO 巡检响应格式错误")
                if result.get("success") is False:
                    raise RuntimeError(str(result.get("error") or "YOLO 巡检失败"))
                return result
            except urllib.error.HTTPError as error:
                try:
                    detail = json.loads(error.read() or b"{}").get("error")
                except (OSError, ValueError, AttributeError):
                    detail = None
                raise RuntimeError(
                    f"YOLO 巡检接口失败: {detail or error.code}") from error
            except (OSError, urllib.error.URLError) as error:
                last_error = error
                # A timed-out capture may still be running in the YOLO child.
                # Retrying would start a duplicate evidence window, so surface
                # the timeout immediately.  Connection-refused startup errors
                # remain retryable because the request was never accepted.
                if path == "capture" and isinstance(
                        error, (TimeoutError, socket.timeout)):
                    raise TimeoutError(
                        f"YOLO 精检采集在 {timeout:.1f} 秒内未完成") from error
                time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        raise RuntimeError(f"YOLO 巡检接口未就绪: {last_error}") from last_error

    def _available(self) -> bool:
        required = [
            self._arg("yolo_script", DEFAULT_YOLO_SCRIPT),
            self._arg("yolo_model", DEFAULT_YOLO_MODEL),
        ]
        if not self._arg("yolo_disable_face", False):
            required.append(self._arg(
                "yolo_face_model", DEFAULT_YOLO_FACE_MODEL))
        if not self._arg("yolo_disable_plate", False):
            required.append(self._arg(
                "yolo_plate_model", DEFAULT_YOLO_PLATE_MODEL))
        return all(Path(path).is_file() for path in required)

    @staticmethod
    def _open_local(url: str, timeout: float):
        """访问本机 YOLO 子进程时显式绕过系统代理，避免 localhost 被转发。"""
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        return opener.open(url, timeout=timeout)

    def _build_command(self) -> list[str]:
        command = [
            str(self._resolve_yolo_python()),
            str(self._arg("yolo_script", DEFAULT_YOLO_SCRIPT)),
            "--host",
            self._arg("host", "127.0.0.1"),
            "--sdk-port",
            str(self._arg("port", 8000)),
            "--username",
            self._arg("username", "admin"),
            "--sdk-dir",
            str(self._arg("sdk_dir", DEFAULT_SDK_DIR)),
            "--model",
            str(self._arg("yolo_model", DEFAULT_YOLO_MODEL)),
            "--face-model",
            str(self._arg("yolo_face_model", DEFAULT_YOLO_FACE_MODEL)),
            "--plate-model",
            str(self._arg("yolo_plate_model", DEFAULT_YOLO_PLATE_MODEL)),
            "--sface-model",
            str(self._arg("recognition_sface_model", DEFAULT_SFACE_MODEL)),
            "--recognition-db",
            str(self._arg("recognition_db", DEFAULT_RECOGNITION_DB)),
            "--recognition-evidence-dir",
            str(self._arg("recognition_evidence_root", DEFAULT_EVIDENCE_DIR)),
            "--channel",
            str(self._arg("channel", 1)),
            "--stream",
            str(self._arg("yolo_stream", self._arg("stream", 0))),
            "--link-mode",
            str(self._arg("link_mode", 0)),
            "--capture-backend",
            self._arg("yolo_capture_backend", DEFAULT_YOLO_CAPTURE_BACKEND),
            "--mjpeg-url",
            self._arg(
                "yolo_mjpeg_url",
                f"http://127.0.0.1:{self._arg('web_port', 8766)}/api/stream",
            ),
            "--rtsp-port",
            str(self._arg("yolo_rtsp_port", 554)),
            "--rtsp-transport",
            self._arg("yolo_rtsp_transport", "tcp"),
            "--rtsp-open-timeout-ms",
            str(self._arg("yolo_rtsp_open_timeout_ms", 5000)),
            "--rtsp-read-timeout-ms",
            str(self._arg("yolo_rtsp_read_timeout_ms", 5000)),
            "--rtsp-reconnect-delay",
            str(self._arg("yolo_rtsp_reconnect_delay", 1.0)),
            "--rtsp-reconnect-max-delay",
            str(self._arg("yolo_rtsp_reconnect_max_delay", 8.0)),
            "--connect-timeout-ms",
            str(self._arg("connect_timeout_ms", 3000)),
            "--connect-retries",
            str(self._arg("connect_retries", 1)),
            "--frame-timeout",
            str(self._arg("yolo_frame_timeout", 10.0)),
            "--conf",
            str(self._arg("yolo_conf", 0.50)),
            "--imgsz",
            str(self._arg("yolo_imgsz", 640)),
            "--inspection-imgsz",
            str(self._arg("yolo_inspection_imgsz", 1280)),
            "--face-conf",
            str(self._arg("yolo_face_conf", 0.60)),
            "--face-max-width",
            str(self._arg("yolo_face_max_width", 640)),
            "--plate-conf",
            str(self._arg("yolo_plate_conf", 0.25)),
            "--plate-imgsz",
            str(self._arg("yolo_plate_imgsz", 640)),
            "--aux-transit-fps",
            str(self._arg("yolo_aux_transit_fps", 3.0)),
            "--aux-manual-fps",
            str(self._arg("yolo_aux_manual_fps", 5.0)),
            "--aux-display-ttl",
            str(self._arg("yolo_aux_display_ttl", 0.60)),
            "--device",
            self._arg("yolo_device", "cpu"),
            "--window-name",
            self._arg("yolo_window_name", "Hikvision YOLO Detection"),
            "--speech-audio-dir",
            str(self._arg("yolo_speech_audio_dir", DEFAULT_YOLO_SPEECH_AUDIO_DIR)),
            "--web-host",
            self._arg("yolo_host", DEFAULT_YOLO_HOST),
            "--web-port",
            str(self._arg("yolo_port", DEFAULT_YOLO_PORT)),
            "--web-quality",
            str(self._arg("yolo_web_quality", DEFAULT_YOLO_WEB_QUALITY)),
            "--voice-output",
            self._arg("yolo_voice_output", DEFAULT_YOLO_VOICE_OUTPUT),
            "--audio-player",
            self._arg("yolo_audio_player", "ffplay"),
            "--no-window",
        ]
        if self._arg("yolo_voice_enable_output", False):
            command.append("--voice-enable-output")
        if self._arg("yolo_disable_face", False):
            command.append("--disable-face")
        if self._arg("yolo_disable_plate", False):
            command.append("--disable-plate")
        return command

    def _reader_loop(self, process: subprocess.Popen[str]) -> None:
        """读取子进程输出，保留最后几行便于网页状态判断。"""
        stream = process.stdout
        if stream is None:
            return
        try:
            for raw_line in iter(stream.readline, ""):
                line = raw_line.rstrip()
                if not line:
                    continue
                with self._lock:
                    self._logs.append(line)
                    self._log_seq += 1
                print(f"[YOLO] {line}", flush=True)
        finally:
            try:
                stream.close()
            except BaseException:
                pass

    def _probe_health(self) -> dict[str, Any]:
        """读取子进程健康状态；HTTP 503 的启动响应也尝试解析为 JSON。"""
        if self._arg("yolo_port", DEFAULT_YOLO_PORT) <= 0:
            return {}
        try:
            with self._open_local(self._health_url(), timeout=0.4) as response:
                payload = json.loads(response.read(64 * 1024) or b"{}")
                return payload if isinstance(payload, dict) else {}
        except urllib.error.HTTPError as error:
            try:
                payload = json.loads(error.read(64 * 1024) or b"{}")
                return payload if isinstance(payload, dict) else {}
            except (OSError, ValueError):
                return {}
        except Exception:
            return {}

    def _probe_ready(self) -> bool:
        """只有子进程已发布至少一帧标注图像时才报告 ready。"""
        health = self._probe_health()
        with self._lock:
            self._last_health = health
        try:
            frames = int(health.get("frames", 0))
        except (TypeError, ValueError):
            frames = 0
        # frames 是跨版本都存在的硬条件；缺少 ready 字段的旧子进程也可兼容。
        return frames > 0 and health.get("ready", True) is not False

    def frame(self, since: int = 0) -> tuple[int, bytes | None]:
        """从 YOLO 子进程取得单帧 JPEG；未变化时返回 ``(序号, None)``。"""
        with self._lock:
            process = self._process
        if process is None or process.poll() is not None:
            raise RuntimeError("YOLO 进程未运行")
        try:
            with self._open_local(self._frame_url(since), timeout=1.5) as response:
                sequence = int(response.headers.get("X-YOLO-Sequence", "0"))
                payload = response.read(16 * 1024 * 1024 + 1)
        except urllib.error.HTTPError as error:
            # 304 是正常的轮询结果，不能被网页显示成故障。
            if error.code == HTTPStatus.NOT_MODIFIED:
                try:
                    return int(error.headers.get("X-YOLO-Sequence", since)), None
                except (TypeError, ValueError):
                    return since, None
            if error.code == HTTPStatus.SERVICE_UNAVAILABLE:
                raise RuntimeError("YOLO 首帧尚未就绪") from error
            raise RuntimeError(f"YOLO 结果帧请求失败: HTTP {error.code}") from error
        except (OSError, ValueError) as error:
            raise RuntimeError(f"YOLO 结果帧请求失败: {error}") from error
        if len(payload) > 16 * 1024 * 1024:
            raise RuntimeError("YOLO 结果帧过大")
        if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
            raise RuntimeError("YOLO 返回的不是有效 JPEG")
        return sequence, payload

    def open_stream(self):
        """等待子进程监听端口后打开 MJPEG 响应；调用方负责关闭。"""
        startup_timeout = float(
            self._arg("yolo_stream_start_timeout", DEFAULT_YOLO_STREAM_START_TIMEOUT)
        )
        deadline = time.monotonic() + startup_timeout
        last_error: BaseException | None = None
        while True:
            with self._lock:
                process = self._process
            if process is None or process.poll() is not None:
                raise RuntimeError("YOLO 进程未运行") from last_error
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"YOLO 结果流在 {startup_timeout:g} 秒内未就绪: {last_error}"
                ) from last_error
            try:
                response = self._open_local(
                    self._stream_source_url(), timeout=min(1.0, remaining)
                )
                break
            except urllib.error.HTTPError as error:
                raise RuntimeError(f"YOLO 结果流请求失败: HTTP {error.code}") from error
            except (OSError, urllib.error.URLError) as error:
                # 子进程要先导入 PyTorch、加载模型并预热，HTTP 服务会稍后才监听。
                # 浏览器通常在启动后几十毫秒内请求结果流，因此这里必须跨过启动窗口。
                last_error = error
                time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
        content_type = response.headers.get("Content-Type", "").lower()
        if not content_type.startswith("multipart/x-mixed-replace"):
            response.close()
            raise RuntimeError("YOLO 子进程未返回 MJPEG")
        return response

    def start(self) -> dict[str, Any]:
        """启动 YOLO 子进程；重复启动时直接返回当前状态。"""
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                if not self._available():
                    raise RuntimeError(f"YOLO 脚本或模型不存在: {self._arg('yolo_script', DEFAULT_YOLO_SCRIPT)} / {self._arg('yolo_model', DEFAULT_YOLO_MODEL)}")
                environment = os.environ.copy()
                password = self._arg("password", "")
                if password:
                    environment["HIKVISION_PASSWORD"] = password
                # 自定义 RTSP 地址可能含认证信息，放环境变量而不是命令行，
                # 避免被 ps/top 或异常日志看到；空值会覆盖继承的旧配置。
                environment["HIKVISION_RTSP_URL"] = str(self._arg("yolo_rtsp_url", "") or "")
                environment["PYTHONUNBUFFERED"] = "1"
                self._logs.clear()
                self._log_seq = 0
                self._last_error = ""
                self._last_exit_code = None
                self._last_health = {}
                self._started_at = time.monotonic()
                self._session_id += 1
                process = subprocess.Popen(
                    self._build_command(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                    env=environment,
                )
                self._process = process
                self._stdout_thread = threading.Thread(target=self._reader_loop, args=(process,), daemon=True, name="yolo-stdout")
                self._stdout_thread.start()
        return self.status()

    def stop(self) -> dict[str, Any]:
        """停止 YOLO 子进程；超时后升级为强制终止。"""
        with self._lock:
            process = self._process
            thread = self._stdout_thread
        if process is None or process.poll() is not None:
            return self.status()
        try:
            os.killpg(process.pid, signal.SIGINT)
        except OSError:
            process.terminate()
        try:
            process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except OSError:
                process.kill()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        if thread is not None:
            thread.join(timeout=2)
        return self.status()

    def status(self) -> dict[str, Any]:
        """返回 YOLO 子进程、结果流和最近日志的轻量状态。"""
        with self._lock:
            process = self._process
            logs = list(self._logs)
            started_at = self._started_at
            last_error = self._last_error
            last_exit = self._last_exit_code
            log_seq = self._log_seq
            session_id = self._session_id
        available = self._available()
        running = process is not None and process.poll() is None
        ready = running and self._probe_ready()
        with self._lock:
            health = dict(self._last_health) if running else {}
        try:
            frame_count = max(0, int(health.get("frames", 0) or 0))
        except (TypeError, ValueError):
            frame_count = 0
        exit_code = process.poll() if process is not None else last_exit
        if exit_code is not None:
            # 子进程可能在 healthz 请求期间退出，不能把旧会话继续报告为 ready。
            running = False
            ready = False
            health = {}
            frame_count = 0
        if process is not None and not running:
            last_exit = process.poll()
            if last_exit is not None and last_exit != 0 and not last_error:
                last_error = logs[-1] if logs else f"YOLO 进程退出码 {last_exit}"
            with self._lock:
                self._last_exit_code = last_exit
                if last_exit is not None and last_exit != 0 and not self._last_error:
                    self._last_error = last_error
        return {
            "available": available,
            "running": running,
            "ready": ready,
            "host": self._arg("yolo_host", DEFAULT_YOLO_HOST),
            "port": self._arg("yolo_port", DEFAULT_YOLO_PORT),
            "capture_backend": self._arg("yolo_capture_backend", DEFAULT_YOLO_CAPTURE_BACKEND),
            "source_url": self._arg(
                "yolo_mjpeg_url",
                f"http://127.0.0.1:{self._arg('web_port', 8766)}/api/stream",
            ) if self._arg("yolo_capture_backend", DEFAULT_YOLO_CAPTURE_BACKEND) == "mjpeg" else None,
            "stream": self._arg("yolo_stream", self._arg("stream", DEFAULT_YOLO_STREAM)),
            "rtsp_transport": self._arg("yolo_rtsp_transport", "tcp"),
            "rtsp_port": self._arg("yolo_rtsp_port", 554),
            "stream_url": self._stream_url(),
            # 浏览器只使用同源代理；stream_url 保留给调试和旧客户端。
            "stream_proxy_url": "/api/yolo/stream.mjpg",
            "frame_url": "/api/yolo/frame.jpg",
            "health_url": self._health_url(),
            "frames": frame_count,
            "capture_fps": health.get("capture_fps", 0.0),
            "inference_fps": health.get("inference_fps", 0.0),
            "publish_fps": health.get("publish_fps", 0.0),
            "frame_age_seconds": health.get("frame_age_seconds"),
            "frame_width": health.get("frame_width", 0),
            "frame_height": health.get("frame_height", 0),
            "face_enabled": not self._arg("yolo_disable_face", False),
            "plate_enabled": not self._arg("yolo_disable_plate", False),
            "aux_transit_fps": self._arg("yolo_aux_transit_fps", 3.0),
            "voice_output": self._arg("yolo_voice_output", DEFAULT_YOLO_VOICE_OUTPUT),
            "uptime_seconds": 0.0 if not running else max(time.monotonic() - started_at, 0.0),
            "exit_code": exit_code,
            "error": last_error,
            "session_id": session_id,
            "log_seq": log_seq,
            "logs": logs[-6:],
        }


def parse_args() -> argparse.Namespace:
    """解析本地网页、V40 解码、ISAPI 与控制安全参数。"""
    parser = argparse.ArgumentParser(description="国创具身相机网页控制台", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--camera-type", choices=("usb", "hikvision"), default="usb", help="相机类型；当前产品默认固定 USB 摄像头")
    parser.add_argument("--usb-device", default=DEFAULT_USB_CAMERA_DEVICE, help="Linux V4L2 普通USB摄像头采集设备；不得选择海康热成像UVC节点")
    parser.add_argument('--camera-source-config', type=Path,
                        default=Path.home()/'.ros'/'robot_navigation'/'ordinary_camera.json',
                        help='设置页普通摄像头来源配置，保存后优先于USB启动默认值')
    parser.add_argument('--body-camera-url', default=DEFAULT_BODY_URL,
                        help='原本体HTTP JPEG/MJPEG图传代理，不启用机器人运动')
    parser.add_argument('--c12-visible-url', default=VISIBLE_URL, help='C12 可见光 RTSP 地址')
    parser.add_argument('--c12-infrared-url', default=INFRARED_URL, help='C12 红外 RTSP 地址')
    parser.add_argument('--behavior-python', type=Path, default=None,
                        help='行为识别独立虚拟环境 Python；默认使用 BEHAVIOR_HOME/venv/bin/python')
    parser.add_argument('--behavior-runtime-dir', type=Path, default=ROOT.parent / 'person-behavior-recognition')
    parser.add_argument('--behavior-config', type=Path,
                        default=Path.home() / '.ros/robot_navigation/behavior.json')
    parser.add_argument("--usb-width", type=int, default=1280, help="USB 采集宽度")
    parser.add_argument("--usb-height", type=int, default=720, help="USB 采集高度")
    parser.add_argument("--usb-fps", type=float, default=30.0, help="USB 采集帧率")
    parser.add_argument("--usb-fourcc", default="MJPG", help="USB 采集 FOURCC，推荐 MJPG")
    parser.add_argument("--host", default="192.168.1.19", help="相机 IP 或主机名")
    parser.add_argument("--username", default="admin", help="设备用户名")
    parser.add_argument("--password", default=os.environ.get("HIKVISION_PASSWORD"), help="设备密码，也可设置 HIKVISION_PASSWORD")
    parser.add_argument("--port", type=int, default=8000, help="HCNetSDK 服务端口")
    parser.add_argument("--channel", type=int, default=1, help="可见光视频与控制通道")
    parser.add_argument("--stream", type=int, choices=(0, 1), default=0, help="0 主码流，1 子码流")
    parser.add_argument("--link-mode", type=int, choices=range(8), default=0, help="V40 链路：0 TCP，1 UDP，2 组播，3 RTP，4 RTSP/RTP，5 RTSP/HTTP，6 HRUDP，7 RTSP/HTTPS")
    parser.add_argument(
        "--video-backend",
        choices=("v4l2", "auto", "v40", "jpeg"),
        default="v4l2",
        help="USB 使用 v4l2；其余选项仅用于旧海康兼容模式",
    )
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR, help="HCNetSDK 库文件目录")
    parser.add_argument("--max-fps", type=float, default=15.0, help="网页编码或回退抓图最高帧率，0 不限速")
    parser.add_argument("--jpeg-quality", type=int, default=82, help="V40 画面 JPEG 质量 1-100")
    parser.add_argument("--video-start-timeout", type=float, default=10.0, help="等待首个浏览器可显示帧的秒数")
    parser.add_argument("--speed", type=int, default=4, choices=range(1, 8), help="默认云台速度 1-7")
    parser.add_argument("--motion-timeout", type=float, default=0.75, help="无保活时自动停止运动的秒数")
    parser.add_argument("--position-interval", type=float, default=2.0, help="俯仰位置低频刷新周期秒数")
    parser.add_argument("--inspection-pan-units-per-degree", type=float, default=100.0, help="设备 PTZ 水平坐标每度单位数")
    parser.add_argument("--inspection-zoom-units-per-x", type=float, default=100.0, help="设备 PTZ 变倍坐标每 1x 单位数")
    parser.add_argument("--inspection-zoom-max-ratio", type=float, default=30.0, help="网页巡检允许的最大光学倍率")
    parser.add_argument("--inspection-position-timeout", type=float, default=12.0, help="巡检云台/变倍到位超时秒数")
    parser.add_argument("--inspection-pan-tolerance-deg", type=float, default=2.0, help="绝对云台到位允许误差度数")
    parser.add_argument("--inspection-pan-90-seconds", type=float, default=2.5, help="设备不支持绝对位置时估算转动 90 度秒数")
    parser.add_argument("--inspection-pan-speed", type=int, choices=range(1, 8), default=2, help="巡检云台缓慢转向速度 1-7")
    parser.add_argument("--inspection-evidence-root", type=Path, default=Path.home() / ".ros" / "robot_navigation" / "inspection_data", help="巡检原图和 YOLO 标注图根目录")
    parser.add_argument("--connect-timeout-ms", type=int, default=3000, help="SDK 单次连接超时毫秒")
    parser.add_argument("--connect-retries", type=int, default=1, help="SDK 登录连接次数")
    parser.add_argument("--ir-mode", choices=("keep", "day", "night"), default="keep", help="启动时强制日夜模式；keep 不改")
    parser.add_argument("--ir-settle-seconds", type=float, default=1.0, help="日夜切换后等待滤光片与码流稳定的秒数")
    parser.add_argument("--http-scheme", choices=("http", "https"), default="http", help="ISAPI 协议")
    parser.add_argument("--http-port", type=int, default=80, help="ISAPI 端口")
    parser.add_argument("--insecure-tls", action="store_true", help="仅调试自签名 HTTPS 时跳过证书校验")
    parser.add_argument("--wiper-id", type=int, default=1, help="PTZAux 雨刮辅助控制编号")
    parser.add_argument("--enable-wiper-config", action="store_true", help="显式启用 SDK 雨刮工作模式配置；当前设备不支持时请保持关闭")
    parser.add_argument("--listen", default="127.0.0.1", help="网页监听地址，默认仅本机")
    parser.add_argument("--web-port", type=int, default=8766, help="网页监听端口")
    parser.add_argument("--thermal-url", default="http://127.0.0.1:8767", help="独立热成像服务地址")
    parser.add_argument("--thermal-timeout", type=float, default=3.0, help="热成像状态和配置请求超时秒数")
    parser.add_argument("--max-stream-clients", type=int, default=6, help="同时允许的 MJPEG 连接数；YOLO 本身会占用一个连接")
    parser.add_argument("--stream-write-timeout", type=float, default=10.0, help="慢速 MJPEG 客户端写入超时秒数")
    parser.add_argument("--yolo-script", type=Path, default=DEFAULT_YOLO_SCRIPT, help="YOLO 独立脚本路径")
    parser.add_argument("--yolo-model", type=Path, default=DEFAULT_YOLO_MODEL, help="YOLO 模型路径")
    parser.add_argument("--yolo-face-model", type=Path, default=DEFAULT_YOLO_FACE_MODEL, help="YuNet 人脸模型路径")
    parser.add_argument("--yolo-plate-model", type=Path, default=DEFAULT_YOLO_PLATE_MODEL, help="车牌 YOLO 模型路径")
    parser.add_argument("--recognition-sface-model", type=Path, default=DEFAULT_SFACE_MODEL, help="人员登记和实时匹配使用的SFace模型")
    parser.add_argument("--recognition-db", type=Path, default=DEFAULT_RECOGNITION_DB, help="独立人车识别SQLite数据库")
    parser.add_argument("--recognition-evidence-root", type=Path, default=DEFAULT_EVIDENCE_DIR, help="人车识别证据目录")
    parser.add_argument("--yolo-disable-face", action="store_true", help="禁用人脸检测")
    parser.add_argument("--yolo-disable-plate", action="store_true", help="禁用车牌检测")
    parser.add_argument("--yolo-host", default=DEFAULT_YOLO_HOST, help="YOLO 结果流监听地址")
    parser.add_argument("--yolo-port", type=int, default=DEFAULT_YOLO_PORT, help="YOLO 结果流端口")
    parser.add_argument("--yolo-conda-env", default=DEFAULT_YOLO_CONDA_ENV, help="YOLO 解释器环境；current 复用当前环境")
    parser.add_argument("--yolo-capture-backend", choices=("mjpeg", "jpeg", "rtsp", "v40", "auto"), default=DEFAULT_YOLO_CAPTURE_BACKEND, help="YOLO 采集路线；USB 默认复用本服务 MJPEG 流")
    parser.add_argument("--yolo-mjpeg-url", default="", help="YOLO 输入 MJPEG 地址；为空时自动使用本服务 /api/stream")
    parser.add_argument("--yolo-stream", type=int, choices=(0, 1), default=DEFAULT_YOLO_STREAM, help="YOLO 使用的码流；默认主码流保留原始画面尺寸，1 为低延迟子码流")
    parser.add_argument("--yolo-rtsp-url", default=os.environ.get("HIKVISION_RTSP_URL", ""), help="YOLO 自定义 RTSP 地址；为空时按通道和码流自动生成")
    parser.add_argument("--yolo-rtsp-port", type=int, default=554, help="YOLO RTSP 服务端口")
    parser.add_argument("--yolo-rtsp-transport", choices=("tcp", "udp"), default="tcp", help="YOLO RTSP 传输方式")
    parser.add_argument("--yolo-rtsp-open-timeout-ms", type=int, default=5000, help="YOLO RTSP 建连超时毫秒")
    parser.add_argument("--yolo-rtsp-read-timeout-ms", type=int, default=5000, help="YOLO RTSP 读取超时毫秒")
    parser.add_argument("--yolo-rtsp-reconnect-delay", type=float, default=1.0, help="YOLO RTSP 首次重连等待秒数")
    parser.add_argument("--yolo-rtsp-reconnect-max-delay", type=float, default=8.0, help="YOLO RTSP 最大重连等待秒数")
    parser.add_argument("--yolo-device", default="cpu", help="YOLO 推理设备，例如 cpu 或 0")
    parser.add_argument("--yolo-conf", type=float, default=0.50, help="YOLO 置信度阈值；达到 50% 才确认")
    parser.add_argument("--yolo-imgsz", type=int, default=640, help="主 YOLO 推理尺寸")
    parser.add_argument("--yolo-inspection-imgsz", type=int, default=1280, help="到点精检主 YOLO 输入尺寸")
    parser.add_argument("--yolo-face-conf", type=float, default=0.60, help="人脸检测阈值")
    parser.add_argument("--yolo-face-max-width", type=int, default=640, help="YuNet 输入最大宽度")
    parser.add_argument("--yolo-plate-conf", type=float, default=0.25, help="车牌检测阈值")
    parser.add_argument("--yolo-plate-imgsz", type=int, default=640, help="车牌 YOLO 推理尺寸")
    parser.add_argument("--yolo-aux-transit-fps", type=float, default=3.0, help="行进中人脸/车牌检测频率")
    parser.add_argument("--yolo-aux-manual-fps", type=float, default=5.0, help="手动预览时人脸/车牌检测频率")
    parser.add_argument("--yolo-aux-display-ttl", type=float, default=0.60, help="辅助检测框显示保留秒数")
    parser.add_argument("--yolo-frame-timeout", type=float, default=10.0, help="YOLO 等待首帧超时秒数")
    parser.add_argument("--yolo-web-quality", type=int, default=DEFAULT_YOLO_WEB_QUALITY, help="YOLO 结果流 JPEG 质量 1-100")
    parser.add_argument("--yolo-stream-start-timeout", type=float, default=DEFAULT_YOLO_STREAM_START_TIMEOUT, help="网页结果流等待 YOLO 子进程监听的秒数")
    parser.add_argument("--yolo-window-name", default="Hikvision YOLO Detection", help="YOLO 窗口标题")
    parser.add_argument("--yolo-speech-audio-dir", type=Path, default=DEFAULT_YOLO_SPEECH_AUDIO_DIR, help="YOLO 固定语音素材目录")
    parser.add_argument("--yolo-voice-output", choices=("local", "device", "both", "none"), default=DEFAULT_YOLO_VOICE_OUTPUT, help="YOLO 语音输出位置；默认本机电脑")
    parser.add_argument("--yolo-audio-player", default="ffplay", help="YOLO 本地音频播放器，默认 ffplay")
    parser.add_argument("--yolo-voice-enable-output", action="store_true", help="YOLO 启动时显式启用设备 ISAPI 双向音频输出")
    args = parser.parse_args()
    args.sdk_dir = args.sdk_dir.expanduser().resolve()
    args.yolo_script = args.yolo_script.expanduser().resolve()
    args.yolo_model = args.yolo_model.expanduser().resolve()
    args.yolo_face_model = args.yolo_face_model.expanduser().resolve()
    args.yolo_plate_model = args.yolo_plate_model.expanduser().resolve()
    args.recognition_sface_model = args.recognition_sface_model.expanduser().resolve()
    args.recognition_db = args.recognition_db.expanduser().resolve()
    args.recognition_evidence_root = args.recognition_evidence_root.expanduser().resolve()
    args.recognition_evidence_root.mkdir(parents=True, exist_ok=True)
    args.yolo_speech_audio_dir = args.yolo_speech_audio_dir.expanduser().resolve()
    args.inspection_evidence_root = args.inspection_evidence_root.expanduser().resolve()
    args.inspection_evidence_root.mkdir(parents=True, exist_ok=True)
    if not args.yolo_mjpeg_url:
        args.yolo_mjpeg_url = f"http://127.0.0.1:{args.web_port}/api/stream"
    if args.camera_type == "hikvision" and not args.password:
        parser.error("请通过 --password 或 HIKVISION_PASSWORD 提供设备密码")
    if args.camera_type == "usb" and args.video_backend != "v4l2":
        parser.error("USB 摄像头必须使用 --video-backend v4l2")
    if args.camera_type == "hikvision" and args.video_backend == "v4l2":
        parser.error("海康兼容模式不能使用 --video-backend v4l2")
    if args.usb_width <= 0 or args.usb_height <= 0 or args.usb_fps <= 0 or len(args.usb_fourcc.strip()) != 4:
        parser.error("USB 分辨率、帧率或 FOURCC 参数无效")
    if args.channel <= 0 or not 1 <= args.port <= 65535 or not 1 <= args.http_port <= 65535 or not 1 <= args.web_port <= 65535:
        parser.error("通道或端口参数无效")
    if args.max_fps < 0 or not 1 <= args.jpeg_quality <= 100 or args.video_start_timeout <= 0 or args.motion_timeout <= 0 or args.position_interval <= 0 or args.stream_write_timeout <= 0 or args.ir_settle_seconds < 0 or args.thermal_timeout <= 0:
        parser.error("视频、JPEG、运动超时或位置刷新参数无效")
    if args.connect_timeout_ms <= 0 or args.connect_retries <= 0 or args.max_stream_clients <= 0 or args.wiper_id <= 0:
        parser.error("连接超时、连接次数、视频客户端上限和雨刮编号必须大于 0")
    if (
        args.inspection_pan_units_per_degree <= 0
        or args.inspection_zoom_units_per_x <= 0
        or args.inspection_zoom_max_ratio < 1
        or args.inspection_position_timeout <= 0
        or args.inspection_pan_tolerance_deg <= 0
        or args.inspection_pan_90_seconds <= 0
    ):
        parser.error("巡检云台坐标、倍率或到位参数无效")
    if not 1 <= args.yolo_port <= 65535:
        parser.error("YOLO 结果流端口必须大于 0")
    if not 1 <= args.yolo_rtsp_port <= 65535 or args.yolo_rtsp_open_timeout_ms <= 0 or args.yolo_rtsp_read_timeout_ms <= 0:
        parser.error("YOLO RTSP 端口和超时参数无效")
    if args.yolo_rtsp_reconnect_delay <= 0 or args.yolo_rtsp_reconnect_max_delay < args.yolo_rtsp_reconnect_delay:
        parser.error("YOLO RTSP 重连等待参数无效")
    if not 1 <= args.yolo_web_quality <= 100 or not 0.0 <= args.yolo_conf <= 1.0 or args.yolo_imgsz <= 0 or args.yolo_imgsz % 32 != 0 or args.yolo_inspection_imgsz <= 0 or args.yolo_inspection_imgsz % 32 != 0 or args.yolo_frame_timeout <= 0 or args.yolo_stream_start_timeout <= 0:
        parser.error("YOLO 质量、置信度、尺寸或等待时间参数无效")
    if (
        not 0.0 <= args.yolo_face_conf <= 1.0
        or not 0.0 <= args.yolo_plate_conf <= 1.0
        or args.yolo_face_max_width <= 0
        or args.yolo_plate_imgsz <= 0
        or args.yolo_plate_imgsz % 32 != 0
        or args.yolo_aux_transit_fps < 0
        or args.yolo_aux_manual_fps < 0
        or args.yolo_aux_display_ttl < 0
    ):
        parser.error("YOLO 人脸/车牌阈值、尺寸或辅助检测频率无效")
    return args


def main() -> int:
    """启动网页服务并在退出时完整回收视频、云台、雨刮和 SDK 资源。"""
    args = parse_args()
    if args.camera_type == "hikvision":
        ensure_player_library_path(args.sdk_dir)
    # 原生 SDK/PlayM4 崩溃时将 C 层线程栈写入终端，便于后续定位而不是静默退出。
    try:
        faulthandler.enable(all_threads=True)
    except (OSError, RuntimeError):
        pass
    if args.camera_type == "hikvision" and args.ir_mode != "keep":
        try:
            set_ir_mode(args, args.ir_mode)
            print(f"已强制设置日夜模式: {args.ir_mode}")
        except RuntimeError as error:
            print(f"错误: {error}")
            return 1
    gateway = CameraGateway(args)
    try:
        gateway.connect()
    except (RuntimeError, OSError) as error:
        print(f"错误: {error}")
        return 1
    server = CameraWebServer((args.listen, args.web_port), CameraRequestHandler, args.max_stream_clients, args.stream_write_timeout)
    server.gateway = gateway
    print(f"相机网页服务: http://{args.listen}:{args.web_port}")
    video_status = gateway.status()["video"]
    if args.camera_type == "usb":
        print(
            f"视频后端: {video_status['backend']}，设备 {video_status.get('device', args.usb_device)}，"
            f"{video_status.get('width', 0)}x{video_status.get('height', 0)}，"
            f"最多 {args.max_stream_clients} 个网页视频连接"
        )
    else:
        print(f"视频后端: {video_status['backend']}，通道 {args.channel}，{'主' if args.stream == 0 else '子'}码流，最多 {args.max_stream_clients} 个网页视频连接")
    print(
        "YOLO 按钮: "
        f"{getattr(args, 'yolo_script', DEFAULT_YOLO_SCRIPT)} -> {getattr(args, 'yolo_model', DEFAULT_YOLO_MODEL)}，"
        f"结果流端口 {getattr(args, 'yolo_host', DEFAULT_YOLO_HOST)}:{getattr(args, 'yolo_port', DEFAULT_YOLO_PORT)}，"
        f"环境 {getattr(args, 'yolo_conda_env', DEFAULT_YOLO_CONDA_ENV)}，"
        f"采集路线 {getattr(args, 'yolo_capture_backend', DEFAULT_YOLO_CAPTURE_BACKEND)}，"
        f"输入 {getattr(args, 'yolo_mjpeg_url', '') if args.yolo_capture_backend == 'mjpeg' else args.yolo_capture_backend}，"
        f"语音输出 {getattr(args, 'yolo_voice_output', DEFAULT_YOLO_VOICE_OUTPUT)}，"
        f"语音素材 {getattr(args, 'yolo_speech_audio_dir', DEFAULT_YOLO_SPEECH_AUDIO_DIR)}"
    )
    watchdog_stop = threading.Event()

    def run_watchdog() -> None:
        # 看门狗独立于 HTTP 请求；浏览器失焦或断线后也能按时停止电机。
        while not watchdog_stop.wait(0.1):
            gateway.watchdog()

    watchdog: threading.Thread | None = None
    if args.camera_type == "hikvision":
        watchdog = threading.Thread(target=run_watchdog, daemon=True, name="hik-motion-watchdog")
        watchdog.start()
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n正在关闭网页控制台。")
    finally:
        watchdog_stop.set()
        # serve_forever 运行在当前主线程；这里调用 shutdown 会等待当前线程自行退出，
        # 导致 Ctrl+C 后死锁。KeyboardInterrupt 已经中断事件循环，直接关闭监听 socket 即可。
        server.server_close()
        gateway.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
