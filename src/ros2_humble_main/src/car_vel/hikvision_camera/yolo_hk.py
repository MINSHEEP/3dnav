#!/usr/bin/env python3
"""使用海康视频源执行独立的 YOLO 实时检测。

本脚本不导入网页控制器或通用推理模块。默认数据链路为：
``RTSP -> OpenCV BGR -> YOLO``；``jpeg`` 使用 ``HCNetSDK CaptureJPEGPicture_NEW``，
``rtsp`` 使用 OpenCV-FFmpeg 读取海康标准 RTSP，断线自动重连；显式选择 ``v40``
才使用 ``HCNetSDK V40 -> PlayM4 解码回调(YV12)``。所有采集线程只保留最新帧，
推理在主线程执行，避免模型耗时阻塞原生回调或积压旧画面。
网页服务同时提供 MJPEG 兼容入口和 ``/frame.jpg`` 单帧入口；后者由上层控制台代理轮询。
"""

from __future__ import annotations

import argparse
import base64
from collections import deque
import json
import os
import queue
import socket
import sys
import threading
import time
from ctypes import CDLL, RTLD_GLOBAL, CFUNCTYPE, POINTER, Structure, byref, c_bool, c_byte, c_char, c_char_p, c_int, c_uint, c_ubyte, c_ushort, c_void_p, create_string_buffer, sizeof, string_at
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, quote, urlsplit, urlunsplit, urlparse

# 禁止 Ultralytics 在运行时修改当前环境；模型依赖应在环境创建时安装。
os.environ.setdefault("YOLO_AUTOINSTALL", "False")

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT / "models" / "cigarette_multiclass_best.pt"
DEFAULT_HOST = "192.168.1.19"
# SDK 与脚本同包发布，迁移后无需修改绝对路径。
DEFAULT_SDK_DIR = ROOT / "sdk"
NET_DVR_SYSHEAD = 1
NET_DVR_STREAMDATA = 2
YV12_FRAME_TYPE = 3
BOX_COLORS = ((0, 255, 255), (0, 165, 255), (255, 128, 0), (0, 255, 0))
SDK_ERRORS = {
    1: "用户名或密码错误",
    2: "当前用户没有权限",
    5: "设备连接数已满",
    7: "连接设备服务失败，请检查 IP、8000 端口和本机网络",
    10: "接收设备响应超时",
    23: "设备或当前调用方式不支持",
    64: "Player SDK 加载失败，请检查 SDK 动态库路径",
}
RATE_WINDOW_SECONDS = 3.0


def _rolling_rate(timestamps: deque[float], now: float | None = None) -> float:
    """计算最近时间窗内的实际速率；调用方必须已持有对应锁。"""
    current = time.monotonic() if now is None else now
    cutoff = current - RATE_WINDOW_SECONDS
    while timestamps and timestamps[0] < cutoff:
        timestamps.popleft()
    if len(timestamps) < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    return 0.0 if span <= 0 else (len(timestamps) - 1) / span


class LocalSdkPath(Structure):
    """HCNetSDK 本地组件库搜索路径结构。"""

    _fields_ = [("sPath", c_char * 256), ("byRes", c_byte * 128)]


class DeviceInfoV30(Structure):
    """SDK 的 NET_DVR_DEVICEINFO_V30，保留 ABI 定义而不使用字节偏移。"""

    _fields_ = [
        ("sSerialNumber", c_byte * 48), ("byAlarmInPortNum", c_byte), ("byAlarmOutPortNum", c_byte),
        ("byDiskNum", c_byte), ("byDVRType", c_byte), ("byChanNum", c_byte), ("byStartChan", c_byte),
        ("byAudioChanNum", c_byte), ("byIPChanNum", c_byte), ("byZeroChanNum", c_byte),
        ("byMainProto", c_byte), ("bySubProto", c_byte), ("bySupport", c_byte), ("bySupport1", c_byte),
        ("bySupport2", c_byte), ("wDevType", c_ushort), ("bySupport3", c_byte), ("byMultiStreamProto", c_byte),
        ("byStartDChan", c_byte), ("byStartDTalkChan", c_byte), ("byHighDChanNum", c_byte),
        ("bySupport4", c_byte), ("byLanguageType", c_byte), ("byVoiceInChanNum", c_byte),
        ("byStartVoiceInChanNo", c_byte), ("bySupport5", c_byte), ("bySupport6", c_byte),
        # 这两个镜像通道字段不能省略；官方结构体总长度为 80 字节。
        ("byMirrorChanNum", c_byte), ("wStartMirrorChanNo", c_ushort),
        ("bySupport7", c_byte), ("byRes2", c_byte),
    ]


if sizeof(DeviceInfoV30) != 80:  # pragma: no cover - ABI 变更应立即暴露
    raise RuntimeError(f"NET_DVR_DEVICEINFO_V30 ABI 异常: {sizeof(DeviceInfoV30)} 字节，预期 80")


class PreviewInfo(Structure):
    """Linux SDK 的 NET_DVR_PREVIEWINFO；窗口句柄必须是 32 位整数。

    HCNetSDK.h 将 ``HWND`` 定义为 ``unsigned int``。在 64 位 Python 中误用
    ``c_void_p`` 会把结构体从 280 字节扩成 288 字节，导致 V40 读取后续字段
    错位，表现为没有首帧、反复断流甚至原生库崩溃。
    """

    _fields_ = [
        ("lChannel", c_int), ("dwStreamType", c_uint), ("dwLinkMode", c_uint), ("hPlayWnd", c_uint),
        ("bBlocked", c_uint), ("bPassbackRecord", c_uint), ("byPreviewMode", c_ubyte),
        ("byStreamID", c_ubyte * 32), ("byProtoType", c_ubyte), ("byRes1", c_ubyte),
        ("byVideoCodingType", c_ubyte), ("dwDisplayBufNum", c_uint), ("byNPQMode", c_ubyte),
        ("byRecvMetaData", c_ubyte), ("byDataType", c_ubyte), ("byRes", c_ubyte * 213),
    ]


if sizeof(PreviewInfo) != 280:  # pragma: no cover - 仅在 ABI 被意外改动时触发
    raise RuntimeError(f"NET_DVR_PREVIEWINFO ABI 异常: {sizeof(PreviewInfo)} 字节，预期 280")


class FrameInfo(Structure):
    """PlayM4 的 FRAME_INFO；Linux Demo 使用全部 32 位无符号字段。"""

    _fields_ = [
        ("nWidth", c_uint), ("nHeight", c_uint), ("nStamp", c_uint),
        ("nType", c_uint), ("nFrameRate", c_uint), ("dwFrameNum", c_uint),
    ]


RealDataCallback = CFUNCTYPE(None, c_int, c_uint, POINTER(c_ubyte), c_uint, c_void_p)
DecodeDataCallback = CFUNCTYPE(None, c_int, c_void_p, c_int, POINTER(FrameInfo), c_void_p, c_void_p)


class JpegPara(Structure):
    """NET_DVR_JPEGPARA；255 表示使用设备当前抓图分辨率。"""

    _fields_ = [("wPicSize", c_ushort), ("wPicQuality", c_ushort)]


class LatestFrameQueue:
    """仅保留一帧的线程安全缓存，防止 YOLO 推理积压实时视频。"""

    def __init__(self) -> None:
        self._frames: queue.Queue[tuple[bytes, int, int]] = queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self.dropped_frames = 0
        self._capture_times: deque[float] = deque(maxlen=120)

    def put(self, payload: bytes, width: int, height: int) -> None:
        """写入最新 YV12 帧；已有未消费帧时先丢弃它。"""
        try:
            self._frames.get_nowait()
        except queue.Empty:
            pass
        else:
            with self._lock:
                self.dropped_frames += 1
        with self._lock:
            self._capture_times.append(time.monotonic())
        self._frames.put_nowait((payload, width, height))

    def get(self, timeout: float) -> tuple[bytes, int, int]:
        """在给定时间内取得一帧，超时由调用方转换为清晰错误。"""
        return self._frames.get(timeout=timeout)

    def dropped_count(self) -> int:
        """返回已被最新帧覆盖的累计帧数。"""
        with self._lock:
            return self.dropped_frames

    def capture_fps(self) -> float:
        """返回 PlayM4 解码回调最近 3 秒的实际帧率。"""
        with self._lock:
            return _rolling_rate(self._capture_times)


class LatestJpegFrame:
    """仅保留最新 JPEG 的线程安全槽位，用于网页结果流。"""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._sequence = 0
        self._value: bytes | None = None
        self._published_at = 0.0
        self._publish_times: deque[float] = deque(maxlen=120)
        self._inference_fps = 0.0
        self._capture_fps = 0.0
        self._frame_width = 0
        self._frame_height = 0
        self._closed = False

    @property
    def sequence(self) -> int:
        """返回已发布 JPEG 的序号。"""
        with self._condition:
            return self._sequence

    def publish(
        self,
        payload: bytes,
        *,
        inference_fps: float = 0.0,
        capture_fps: float = 0.0,
        frame_width: int = 0,
        frame_height: int = 0,
    ) -> None:
        """覆盖旧 JPEG 并唤醒等待中的浏览器连接。"""
        with self._condition:
            now = time.monotonic()
            self._value = payload
            self._sequence += 1
            self._published_at = now
            self._publish_times.append(now)
            self._inference_fps = max(float(inference_fps), 0.0)
            self._capture_fps = max(float(capture_fps), 0.0)
            self._frame_width = max(int(frame_width), 0)
            self._frame_height = max(int(frame_height), 0)
            self._condition.notify_all()

    def snapshot(self) -> tuple[int, bytes | None, float | None]:
        """无阻塞读取最新帧、序号和帧龄，供单帧 HTTP 与健康检查共用。"""
        with self._condition:
            if self._value is None:
                return self._sequence, None, None
            return self._sequence, self._value, max(time.monotonic() - self._published_at, 0.0)

    def wait_next(self, last_sequence: int, timeout: float = 2.0) -> tuple[int, bytes | None]:
        """等待下一个 JPEG，超时或关闭时返回当前序号。"""
        with self._condition:
            self._condition.wait_for(lambda: self._sequence > last_sequence or self._closed, timeout=timeout)
            return self._sequence, self._value

    def metrics(self) -> dict[str, float | int | None]:
        """返回采集、推理、发布和帧龄指标，用于区分瓶颈所在层。"""
        with self._condition:
            age = None if self._value is None else max(time.monotonic() - self._published_at, 0.0)
            return {
                "frames": self._sequence,
                "capture_fps": round(self._capture_fps, 2),
                "inference_fps": round(self._inference_fps, 2),
                "publish_fps": round(_rolling_rate(self._publish_times), 2),
                "frame_age_seconds": None if age is None else round(age, 3),
                "frame_width": self._frame_width,
                "frame_height": self._frame_height,
            }

    def close(self) -> None:
        """关闭槽位并唤醒所有等待者。"""
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def is_closed(self) -> bool:
        """返回槽位是否已关闭，供 MJPEG 长连接在停止时立即退出。"""
        with self._condition:
            return self._closed


class InspectionYoloState:
    """Coordinate paused/transit/inspection inference and paired evidence."""

    MODES = {'manual', 'paused', 'transit', 'inspection'}

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._mode = 'manual'
        self._task_id = ''
        self._sequence = 0
        self._frames: deque[dict[str, Any]] = deque(maxlen=240)
        self._events: deque[dict[str, Any]] = deque(maxlen=100)
        self._last_inference_started = 0.0
        self._recognition_status: dict[str, Any] = {}

    def set_mode(self, mode: str, task_id: str = '') -> dict[str, Any]:
        mode = str(mode).strip().lower()
        if mode not in self.MODES:
            raise ValueError(f'不支持的 YOLO 模式: {mode}')
        with self._condition:
            self._mode = mode
            self._task_id = str(task_id or '').strip()
            self._condition.notify_all()
        return self.state()

    def mode(self) -> str:
        with self._condition:
            return self._mode

    def context(self) -> tuple[str, str]:
        """Return one atomic mode/task snapshot for recognition scoping."""
        with self._condition:
            return self._mode, self._task_id

    def set_recognition_status(self, status: dict[str, Any]) -> None:
        with self._condition:
            self._recognition_status = dict(status or {})

    def wait_for_inference_turn(self) -> bool:
        """Return False while paused and cap transit inference at 10 FPS."""
        with self._condition:
            if self._mode == 'paused':
                self._condition.wait(timeout=0.1)
                return False
            if self._mode == 'transit':
                due = self._last_inference_started + 0.1
                remaining = due - time.monotonic()
                if remaining > 0.0:
                    self._condition.wait(timeout=remaining)
                if self._mode == 'paused':
                    return False
            self._last_inference_started = time.monotonic()
            return True

    def publish(
            self, raw_jpeg: bytes, annotated_jpeg: bytes,
            detections: list[dict[str, Any]]) -> int:
        with self._condition:
            self._sequence += 1
            frame = {
                'sequence': self._sequence,
                'timestamp_epoch': time.time(),
                'published_monotonic': time.monotonic(),
                'mode': self._mode,
                'task_id': self._task_id,
                'recognition': dict(self._recognition_status),
                'detections': [dict(item) for item in detections],
                'raw_jpeg': raw_jpeg,
                'annotated_jpeg': annotated_jpeg,
            }
            self._frames.append(frame)
            # Auxiliary face/plate detections remain available to live
            # rendering and inspection captures, but only safety detections
            # create asynchronous transit events.
            if any(bool(item.get('alert', True)) for item in detections):
                self._events.append(dict(frame))
            self._condition.notify_all()
            return self._sequence

    @staticmethod
    def _public_frame(frame: dict[str, Any]) -> dict[str, Any]:
        return {
            key: value for key, value in frame.items()
            if key not in ('raw_jpeg', 'annotated_jpeg', 'published_monotonic')
        }

    @classmethod
    def _encoded_frame(cls, frame: dict[str, Any]) -> dict[str, Any]:
        return {
            **cls._public_frame(frame),
            'raw_jpeg_base64': base64.b64encode(
                frame['raw_jpeg']).decode('ascii'),
            'annotated_jpeg_base64': base64.b64encode(
                frame['annotated_jpeg']).decode('ascii'),
        }

    def state(self) -> dict[str, Any]:
        with self._condition:
            latest = self._frames[-1] if self._frames else None
            return {
                'success': True,
                'mode': self._mode,
                'task_id': self._task_id,
                'sequence': self._sequence,
                'ready': latest is not None,
                'frame_age_seconds': (
                    None if latest is None else round(max(
                        0.0, time.monotonic()
                        - float(latest['published_monotonic'])), 3)),
            }

    def events_since(self, sequence: int) -> dict[str, Any]:
        with self._condition:
            events = [
                self._encoded_frame(frame) for frame in self._events
                if int(frame['sequence']) > int(sequence)
            ]
            return {
                'success': True,
                'mode': self._mode,
                'sequence': self._sequence,
                'events': events,
            }

    def capture_window(self, duration_seconds: float) -> dict[str, Any]:
        duration = min(10.0, max(0.1, float(duration_seconds)))
        deadline = time.monotonic() + duration
        with self._condition:
            start_sequence = self._sequence
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                self._condition.wait(timeout=min(remaining, 0.25))
            candidates = [
                frame for frame in self._frames
                if int(frame['sequence']) > start_sequence
            ]
            if not candidates and self._frames:
                candidates = [self._frames[-1]]
            if not candidates:
                raise RuntimeError('YOLO 尚未产生可用帧')

            def score(frame: dict[str, Any]) -> tuple[int, float, float, int]:
                # Safety detections outrank higher-confidence auxiliary
                # face/plate boxes.  If no alert exists, auxiliary confidence
                # still selects the clearest inspection evidence frame.
                alert_confidence = max(
                    (float(item.get('confidence', 0.0))
                     for item in frame['detections']
                     if bool(item.get('alert', True))),
                    default=-1.0,
                )
                any_confidence = max(
                    (float(item.get('confidence', 0.0))
                     for item in frame['detections']),
                    default=-1.0,
                )
                return (
                    int(alert_confidence >= 0.0), alert_confidence,
                    any_confidence, int(frame['sequence']))

            best = max(candidates, key=score)
            return {'success': True, **self._encoded_frame(best)}


def yv12_to_bgr(payload: bytes, width: int, height: int) -> np.ndarray:
    """将 PlayM4 的连续 YV12 数据转换为 OpenCV BGR 图像。"""
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise ValueError(f"YV12 分辨率无效: {width}x{height}")
    expected = width * height * 3 // 2
    if len(payload) != expected:
        raise ValueError(f"YV12 数据长度异常: 实际 {len(payload)}，预期 {expected}")
    yuv = np.frombuffer(payload, dtype=np.uint8).reshape(height * 3 // 2, width)
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YV12)


class YoloMJPEGHandler(BaseHTTPRequestHandler):
    """把 YOLO 推理结果发布成浏览器可直接打开的 MJPEG 页面。"""

    frames: LatestJpegFrame
    inspection: InspectionYoloState
    environment_speech: Any

    def log_message(self, format: str, *args: Any) -> None:
        """关闭逐请求访问日志，保留主流程关键输出。"""
        return

    def do_GET(self) -> None:  # noqa: N802 - HTTPServer 约定的方法名
        """根据请求路径返回首页、健康检查或结果流。"""
        path = urlparse(self.path).path
        if path == "/":
            self._serve_index()
        elif path == "/healthz":
            self._serve_health()
        elif path == "/frame.jpg":
            self._serve_frame()
        elif path == "/stream.mjpg":
            self._serve_stream()
        elif path == "/inspection/state":
            self._send_json(self.inspection.state())
        elif path == "/inspection/events":
            query = parse_qs(urlparse(self.path).query)
            try:
                since = max(0, int(query.get("since", ["0"])[0]))
            except (TypeError, ValueError):
                self.send_error(400, "since 必须是非负整数")
                return
            self._send_json(self.inspection.events_since(since))
        else:
            self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802 - HTTPServer 约定的方法名
        path = urlparse(self.path).path
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length < 0 or length > 1024 * 1024:
                raise ValueError('请求体过大')
            payload = json.loads(self.rfile.read(length) or b'{}')
            if path == '/inspection/mode':
                result = self.inspection.set_mode(
                    payload.get('mode', ''), payload.get('task_id', ''))
            elif path == '/inspection/capture':
                result = self.inspection.capture_window(
                    payload.get('duration_seconds', 2.0))
            elif path == '/inspection/speech':
                key = str(payload.get('key', '')).strip()
                if key not in {'thermal_high', 'co2_high'}:
                    raise ValueError('环境语音告警类型无效')
                queued = bool(self.environment_speech(key))
                result = {'success': True, 'key': key, 'queued': queued}
            else:
                self.send_error(404)
                return
            self._send_json(result)
        except (ValueError, RuntimeError, json.JSONDecodeError) as error:
            self._send_json({'success': False, 'error': str(error)}, 400)

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (
                BrokenPipeError, ConnectionAbortedError,
                ConnectionResetError, socket.timeout):
            return

    def _serve_index(self) -> None:
        body = (
            "<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>YOLO 推理结果</title></head>"
            "<body style='margin:0;background:#111;display:grid;place-items:center;min-height:100vh'>"
            "<img src='/stream.mjpg' alt='YOLO 推理结果' style='max-width:100%;height:auto'>"
            "</body></html>"
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def _serve_health(self) -> None:
        metrics = self.frames.metrics()
        sequence = int(metrics["frames"])
        body = json.dumps(
            {
                "status": "ok",
                # 端口可访问不等于已经有画面；调用方必须同时检查 frames > 0。
                "ready": sequence > 0,
                **metrics,
            },
            ensure_ascii=False,
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def _serve_frame(self) -> None:
        """返回最新一张标注 JPEG；``since`` 未变化时用 304 降低轮询带宽。"""
        query = parse_qs(urlparse(self.path).query)
        try:
            since = max(0, int(query.get("since", ["0"])[0]))
        except (TypeError, ValueError):
            self.send_error(400, "since 必须是非负整数")
            return
        sequence, jpeg, age = self.frames.snapshot()
        if jpeg is None:
            body = json.dumps({"status": "starting", "ready": False, "frames": sequence}, ensure_ascii=False).encode("utf-8")
            self.send_response(503)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                return
            return
        common_headers = {
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "X-YOLO-Sequence": str(sequence),
        }
        if age is not None:
            common_headers["X-YOLO-Frame-Age"] = f"{age:.3f}"
        # since 大于当前序号通常意味着子进程刚重启；返回当前帧帮助客户端自行校准。
        if since == sequence:
            self.send_response(304)
            for name, value in common_headers.items():
                self.send_header(name, value)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        for name, value in common_headers.items():
            self.send_header(name, value)
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        try:
            self.wfile.write(jpeg)
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def _serve_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        sequence = 0
        try:
            while True:
                next_sequence, jpeg = self.frames.wait_next(sequence, timeout=5.0)
                # close() 会唤醒 wait_next；如果不显式退出，这个守护线程会在
                # 子进程停止后继续空转，浏览器重连和程序退出都会变慢。
                if self.frames.is_closed():
                    return
                if next_sequence <= sequence or jpeg is None:
                    continue
                sequence = next_sequence
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return


class YoloMJPEGServer:
    """独立的本地 MJPEG 服务，供网页展示 YOLO 结果。"""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.frames = LatestJpegFrame()
        self.inspection = InspectionYoloState()
        self._speech_lock = threading.RLock()
        self._speech: Any | None = None
        self._speech_initialized = False
        self._pending_environment_speech: deque[str] = deque(maxlen=8)
        self._server = ThreadingHTTPServer((host, port), YoloMJPEGHandler)
        self._server.daemon_threads = True
        YoloMJPEGHandler.frames = self.frames
        YoloMJPEGHandler.inspection = self.inspection
        YoloMJPEGHandler.environment_speech = self.announce_environment
        self._thread = threading.Thread(target=self._server.serve_forever, name="yolo-mjpeg", daemon=True)

    def set_speech_service(self, speech: Any | None) -> None:
        """Attach the optional serialized speech queue used by inspections."""
        with self._speech_lock:
            self._speech = speech
            self._speech_initialized = True
            pending = tuple(self._pending_environment_speech)
            self._pending_environment_speech.clear()
        if speech is not None:
            for key in pending:
                speech.announce_environment(key)

    def announce_environment(self, key: str) -> bool:
        """Speak now, or buffer briefly while the voice output initializes."""
        with self._speech_lock:
            speech = self._speech
            initialized = self._speech_initialized
            if speech is None and not initialized:
                if key not in self._pending_environment_speech:
                    self._pending_environment_speech.append(key)
                return True
        if speech is None:
            raise RuntimeError('语音输出不可用')
        speech.announce_environment(key)
        return False

    def start(self) -> None:
        """启动 HTTP 服务线程。"""
        self._thread.start()

    def publish(
        self,
        jpeg: bytes,
        *,
        inference_fps: float = 0.0,
        capture_fps: float = 0.0,
        frame_width: int = 0,
        frame_height: int = 0,
    ) -> None:
        """发布最新 YOLO 结果帧。"""
        self.frames.publish(
            jpeg,
            inference_fps=inference_fps,
            capture_fps=capture_fps,
            frame_width=frame_width,
            frame_height=frame_height,
        )

    def publish_inspection_frame(
            self, raw_jpeg: bytes, annotated_jpeg: bytes,
            detections: list[dict[str, Any]]) -> int:
        """Publish a raw/annotated pair with structured detections."""
        return self.inspection.publish(raw_jpeg, annotated_jpeg, detections)

    def wait_for_inference_turn(self) -> bool:
        return self.inspection.wait_for_inference_turn()

    def close(self) -> None:
        """停止 HTTP 服务并唤醒等待中的客户端。"""
        self.frames.close()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


class HikvisionV40Source:
    """独立封装 HCNetSDK V40 取流、PlayM4 解码和最新帧读取。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.sdk: Any = None
        self.player: Any = None
        self.user_id = -1
        self.real_handle = -1
        self.player_port = c_int(-1)
        self.device_info = DeviceInfoV30()
        self.frames = LatestFrameQueue()
        # 必须持有回调对象，防止 ctypes 回收后导致 SDK 调用悬空函数指针。
        self.real_callback = RealDataCallback(self._on_real_data)
        self.decode_callback = DecodeDataCallback(self._on_decoded_frame)
        self.stream_opened = False
        self.stream_error = ""

    def _sdk_error(self, prefix: str) -> str:
        """组合 SDK 错误码及可执行的中文说明。"""
        code = int(self.sdk.NET_DVR_GetLastError())
        return f"{prefix}: {code}" + (f"（{SDK_ERRORS[code]}）" if code in SDK_ERRORS else "")

    def _player_error(self, prefix: str) -> str:
        """读取 PlayM4 的错误码，避免吞掉解码失败原因。"""
        code = int(self.player.PlayM4_GetLastError(self.player_port.value))
        return f"{prefix}: {code}"

    def _configure_libraries(self) -> None:
        """加载同一 SDK 发行包的 HCNetSDK 与 PlayM4，并声明 ABI。"""
        sdk_dir = self.args.sdk_dir
        if not (sdk_dir / "libhcnetsdk.so").is_file() or not (sdk_dir / "libPlayCtrl.so").is_file():
            raise FileNotFoundError(f"SDK 目录缺少 libhcnetsdk.so 或 libPlayCtrl.so: {sdk_dir}")
        self.sdk = CDLL(str(sdk_dir / "libhcnetsdk.so"))
        self.sdk.NET_DVR_SetSDKInitCfg.argtypes = [c_int, c_void_p]
        self.sdk.NET_DVR_SetSDKInitCfg.restype = c_bool
        self.sdk.NET_DVR_Init.restype = c_bool
        self.sdk.NET_DVR_SetConnectTime.argtypes = [c_uint, c_uint]
        self.sdk.NET_DVR_SetConnectTime.restype = c_bool
        self.sdk.NET_DVR_Login_V30.argtypes = [c_char_p, c_ushort, c_char_p, c_char_p, POINTER(DeviceInfoV30)]
        self.sdk.NET_DVR_Login_V30.restype = c_int
        self.sdk.NET_DVR_RealPlay_V40.argtypes = [c_int, POINTER(PreviewInfo), RealDataCallback, c_void_p]
        self.sdk.NET_DVR_RealPlay_V40.restype = c_int
        self.sdk.NET_DVR_StopRealPlay.argtypes = [c_int]
        self.sdk.NET_DVR_StopRealPlay.restype = c_bool
        self.sdk.NET_DVR_Logout.argtypes = [c_int]
        self.sdk.NET_DVR_Logout.restype = c_bool
        self.sdk.NET_DVR_Cleanup.restype = c_bool
        self.sdk.NET_DVR_GetLastError.restype = c_uint
        local_path = LocalSdkPath()
        local_path.sPath = os.fsencode(sdk_dir)
        for kind, value in (
            (2, byref(local_path)),
            (3, create_string_buffer(os.fsencode(sdk_dir / "libcrypto.so.1.1"))),
            (4, create_string_buffer(os.fsencode(sdk_dir / "libssl.so.1.1"))),
        ):
            if not self.sdk.NET_DVR_SetSDKInitCfg(kind, value):
                raise RuntimeError(self._sdk_error("SDK 路径配置失败"))
        if not self.sdk.NET_DVR_Init():
            raise RuntimeError(self._sdk_error("SDK 初始化失败"))
        self.sdk.NET_DVR_SetConnectTime(self.args.connect_timeout_ms, self.args.connect_retries)
        self.player = CDLL(str(sdk_dir / "libPlayCtrl.so"), mode=RTLD_GLOBAL)
        self.player.PlayM4_GetPort.argtypes = [POINTER(c_int)]
        self.player.PlayM4_GetPort.restype = c_bool
        self.player.PlayM4_SetStreamOpenMode.argtypes = [c_int, c_uint]
        self.player.PlayM4_SetStreamOpenMode.restype = c_bool
        self.player.PlayM4_OpenStream.argtypes = [c_int, c_void_p, c_uint, c_uint]
        self.player.PlayM4_OpenStream.restype = c_bool
        self.player.PlayM4_SetDecCallBackMend.argtypes = [c_int, DecodeDataCallback, c_int]
        self.player.PlayM4_SetDecCallBackMend.restype = c_bool
        self.player.PlayM4_Play.argtypes = [c_int, c_void_p]
        self.player.PlayM4_Play.restype = c_bool
        self.player.PlayM4_InputData.argtypes = [c_int, c_void_p, c_uint]
        self.player.PlayM4_InputData.restype = c_bool
        self.player.PlayM4_Stop.argtypes = [c_int]
        self.player.PlayM4_Stop.restype = c_bool
        self.player.PlayM4_CloseStream.argtypes = [c_int]
        self.player.PlayM4_CloseStream.restype = c_bool
        self.player.PlayM4_FreePort.argtypes = [c_int]
        self.player.PlayM4_FreePort.restype = c_bool
        self.player.PlayM4_GetLastError.argtypes = [c_int]
        self.player.PlayM4_GetLastError.restype = c_uint
        if not self.player.PlayM4_GetPort(byref(self.player_port)):
            raise RuntimeError("PlayM4 无可用播放端口")

    def connect(self) -> None:
        """初始化 SDK、加载播放器并登录相机。"""
        try:
            self._configure_libraries()
            self.user_id = self.sdk.NET_DVR_Login_V30(
                self.args.host.encode(), self.args.sdk_port, self.args.username.encode(), self.args.password.encode(), byref(self.device_info)
            )
            if self.user_id < 0:
                raise RuntimeError(self._sdk_error("设备登录失败"))
        except Exception:
            self.release()
            raise

    def start(self) -> None:
        """启动 V40 实时流；系统头由回调打开 PlayM4 解码器。"""
        info = PreviewInfo()
        info.lChannel = self.args.channel
        info.dwStreamType = self.args.stream
        info.dwLinkMode = self.args.link_mode
        info.hPlayWnd = 0
        # 官方 Linux 示例使用阻塞取流；这里只在独立采集线程中启动预览。
        info.bBlocked = 1
        info.dwDisplayBufNum = 1
        self.real_handle = self.sdk.NET_DVR_RealPlay_V40(self.user_id, byref(info), self.real_callback, None)
        if self.real_handle < 0:
            raise RuntimeError(self._sdk_error("V40 预览失败"))

    def _on_real_data(self, _handle: int, data_type: int, buffer, size: int, _user: object) -> None:
        """SDK 实时回调只向 PlayM4 转发系统头和码流，不能执行 YOLO。"""
        if data_type == NET_DVR_SYSHEAD and not self.stream_opened:
            if not self.player.PlayM4_SetStreamOpenMode(self.player_port.value, 0):
                self.stream_error = self._player_error("设置播放模式失败")
                return
            if not self.player.PlayM4_OpenStream(self.player_port.value, buffer, size, 2 * 1024 * 1024):
                self.stream_error = self._player_error("打开码流失败")
                return
            if not self.player.PlayM4_SetDecCallBackMend(self.player_port.value, self.decode_callback, 0):
                self.stream_error = self._player_error("注册解码回调失败")
                return
            if not self.player.PlayM4_Play(self.player_port.value, None):
                self.stream_error = self._player_error("启动解码失败")
                return
            self.stream_opened = True
        elif data_type == NET_DVR_STREAMDATA and self.stream_opened:
            if not self.player.PlayM4_InputData(self.player_port.value, buffer, size):
                self.stream_error = self._player_error("输入码流失败")

    def _on_decoded_frame(self, _port: int, buffer: int, size: int, info, _user: object, _reserved: object) -> None:
        """仅复制完整 YV12 帧到单槽队列，颜色转换与推理均在回调外执行。"""
        if not info or info.contents.nType != YV12_FRAME_TYPE:
            return
        width, height = int(info.contents.nWidth), int(info.contents.nHeight)
        expected = width * height * 3 // 2
        if width <= 0 or height <= 0 or size != expected:
            return
        self.frames.put(string_at(buffer, size), width, height)

    def read(self, timeout: float) -> np.ndarray:
        """读取最新一帧并在非回调线程转换为 BGR。"""
        if self.stream_error:
            raise RuntimeError(self.stream_error)
        try:
            payload, width, height = self.frames.get(timeout)
        except queue.Empty as error:
            raise TimeoutError(f"等待解码帧超时 {timeout:.1f} 秒") from error
        return yv12_to_bgr(payload, width, height)

    def release(self) -> None:
        """以取流、解码、登录、SDK 的逆序释放全部本地资源。"""
        if self.sdk is None:
            return
        if self.real_handle >= 0:
            self.sdk.NET_DVR_StopRealPlay(self.real_handle)
            self.real_handle = -1
        if self.player is not None and self.player_port.value >= 0:
            self.player.PlayM4_Stop(self.player_port.value)
            self.player.PlayM4_CloseStream(self.player_port.value)
            self.player.PlayM4_FreePort(self.player_port.value)
            self.player_port = c_int(-1)
        if self.user_id >= 0:
            self.sdk.NET_DVR_Logout(self.user_id)
            self.user_id = -1
        self.sdk.NET_DVR_Cleanup()


class JpegFrameStats:
    """SDK 抓图源的帧统计适配器，保持与 V40 源相同的状态接口。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._capture_times: deque[float] = deque(maxlen=120)

    def mark(self) -> None:
        """记录一次成功的同步 JPEG 抓图。"""
        with self._lock:
            self._capture_times.append(time.monotonic())

    def dropped_count(self) -> int:
        """抓图是按需同步获取，不存在排队丢帧。"""
        return 0

    def capture_fps(self) -> float:
        """返回最近 3 秒的设备 JPEG 抓图速率。"""
        with self._lock:
            return _rolling_rate(self._capture_times)


class LatestBgrFrame:
    """只保留 RTSP 最新 BGR 帧的线程安全槽位。

    RTSP 解码线程的速度通常高于 YOLO 推理速度。如果使用普通无限队列，
    推理会逐渐落后于实时画面；这里覆盖旧帧并记录丢弃数量，保证网页显示的
    是当前画面而不是几秒前的缓存。
    """

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._sequence = 0
        self._consumed_sequence = 0
        self._value: np.ndarray | None = None
        self._dropped = 0
        self._capture_times: deque[float] = deque(maxlen=120)
        self._closed = False

    def publish(self, frame: np.ndarray) -> None:
        """发布新帧；尚未消费的旧帧会被覆盖。"""
        with self._condition:
            if self._value is not None and self._sequence > self._consumed_sequence:
                self._dropped += 1
            self._value = frame
            self._sequence += 1
            self._capture_times.append(time.monotonic())
            self._condition.notify_all()

    def wait_next(self, last_sequence: int, timeout: float) -> tuple[int, np.ndarray | None]:
        """等待一帧序号更新；超时或关闭时返回当前状态。"""
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > last_sequence or self._closed,
                timeout=max(float(timeout), 0.0),
            )
            if self._sequence <= last_sequence or self._value is None:
                return self._sequence, None
            self._consumed_sequence = self._sequence
            return self._sequence, self._value

    def dropped_count(self) -> int:
        """返回被新帧覆盖的累计帧数。"""
        with self._condition:
            return self._dropped

    def capture_fps(self) -> float:
        """返回 OpenCV/FFmpeg 解码线程最近 3 秒的实际帧率。"""
        with self._condition:
            return _rolling_rate(self._capture_times)

    def close(self) -> None:
        """关闭槽位并唤醒等待中的读取线程。"""
        with self._condition:
            self._closed = True
            self._condition.notify_all()


def build_rtsp_url(args: argparse.Namespace) -> str:
    """生成海康标准 RTSP 地址。

    海康通道使用 ``通道号 * 100 + 码流号``：通道 1 主/子码流分别是
    ``101/102``，红外通道 2 则是 ``201/202``。显式传入 ``--rtsp-url`` 时
    原样使用，便于兼容 NVR 或厂商定制路径。
    """
    explicit = str(getattr(args, "rtsp_url", "") or "").strip()
    if explicit:
        return explicit
    username = quote(str(getattr(args, "username", "admin")), safe="")
    password = quote(str(getattr(args, "password", "")), safe="")
    host = str(getattr(args, "host", DEFAULT_HOST)).strip()
    port = int(getattr(args, "rtsp_port", 554))
    channel = int(getattr(args, "channel", 1))
    stream = int(getattr(args, "stream", 0))
    channel_id = channel * 100 + stream + 1
    return f"rtsp://{username}:{password}@{host}:{port}/Streaming/Channels/{channel_id}"


def redact_rtsp_url(url: str) -> str:
    """隐藏日志中的 RTSP 用户名和密码。"""
    try:
        parsed = urlsplit(url)
        if not parsed.hostname:
            return url
        host = parsed.hostname
        if parsed.port:
            host = f"{host}:{parsed.port}"
        return urlunsplit((parsed.scheme, host, parsed.path, parsed.query, parsed.fragment))
    except (TypeError, ValueError):
        return "rtsp://<redacted>"


class RtspSource:
    """通过 OpenCV-FFmpeg 读取海康 RTSP，并在断流后自动重连。

    采集线程只操作 ``VideoCapture``，主线程只消费最新 BGR 帧，避免 OpenCV
    原生对象被多个线程同时调用。FFmpeg 使用 TCP、低延迟和单帧缓存，适合
    网页 YOLO；密码只存在于进程内，日志会删除认证信息。
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.url = build_rtsp_url(args)
        self.frames = LatestBgrFrame()
        self._capture: Any = None
        self._capture_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_sequence = 0
        self._last_error = ""
        self._error_lock = threading.Lock()
        self._last_connect_log = 0.0

    def _set_error(self, message: str) -> None:
        """保存最近一次采集错误，避免重复刷屏。"""
        with self._error_lock:
            if message == self._last_error:
                return
            self._last_error = message
        if message:
            print(f"[RTSP] {message}", file=sys.stderr, flush=True)

    def _open_capture(self) -> Any:
        """使用 FFmpeg 打开 RTSP，并设置超时与单帧缓冲。"""
        transport = str(getattr(self.args, "rtsp_transport", "tcp"))
        open_timeout = int(getattr(self.args, "rtsp_open_timeout_ms", 5000))
        read_timeout = int(getattr(self.args, "rtsp_read_timeout_ms", 5000))
        # OpenCV-FFmpeg 通过环境变量接收 RTSP 传输选项；设置仅在构造期间生效。
        # H.265 的参考帧需要重排缓冲，强制 nobuffer/low_delay 会在每次重连后丢失 RPS，出现 POC 错误和卡帧。
        # 应用层 LatestBgrFrame 只保留最新帧，已能控制延迟，这里优先保证解码完整性。
        options = f"rtsp_transport;{transport}|fflags;discardcorrupt"
        previous_options = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS")
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = options
        try:
            backend_preference = getattr(cv2, "CAP_FFMPEG", getattr(cv2, "CAP_ANY", 0))
            params = []
            open_prop = getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None)
            read_prop = getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None)
            if open_prop is not None:
                params.extend([open_prop, open_timeout])
            if read_prop is not None:
                params.extend([read_prop, read_timeout])
            try:
                capture = cv2.VideoCapture(self.url, backend_preference, params)
            except (TypeError, cv2.error):
                # 旧版 OpenCV 不支持带参数的构造函数，退回兼容形式。
                try:
                    capture = cv2.VideoCapture(self.url, backend_preference)
                except (TypeError, cv2.error):
                    capture = cv2.VideoCapture(self.url)
        finally:
            if previous_options is None:
                os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)
            else:
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = previous_options

        if capture is None or not capture.isOpened():
            if capture is not None:
                capture.release()
            raise ConnectionError(f"OpenCV 无法打开 RTSP: {redact_rtsp_url(self.url)}")
        # 这些属性不一定被当前 OpenCV 后端接受，失败不影响已打开的连接。
        try:
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except (AttributeError, cv2.error):
            pass
        for property_id, value in (
            (getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None), open_timeout),
            (getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None), read_timeout),
        ):
            if property_id is None:
                continue
            try:
                capture.set(property_id, value)
            except (AttributeError, cv2.error):
                pass
        try:
            backend = capture.getBackendName()
        except (AttributeError, cv2.error):
            backend = "unknown"
        try:
            width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = float(capture.get(cv2.CAP_PROP_FPS))
        except (AttributeError, cv2.error, TypeError, ValueError):
            width, height, fps = 0, 0, 0.0
        # 网络设备反复重启时可能每秒建立多次连接；限制成功日志频率，避免
        # 终端被重连信息淹没，同时保留足够的现场诊断信息。
        now = time.monotonic()
        if now - self._last_connect_log >= 2.0:
            print(
                f"[RTSP] 已连接: {redact_rtsp_url(self.url)}，后端 {backend}，"
                f"{width}x{height}，{fps:.1f} FPS，传输 {transport.upper()}",
                flush=True,
            )
            self._last_connect_log = now
        return capture

    def connect(self) -> None:
        """先打开一次 RTSP，确保错误在启动阶段可见。"""
        self._stop.clear()
        capture = self._open_capture()
        with self._capture_lock:
            self._capture = capture

    def start(self) -> None:
        """启动后台读取线程。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="hikvision-rtsp", daemon=True)
        self._thread.start()

    def _take_capture(self) -> Any:
        with self._capture_lock:
            return self._capture

    def _drop_capture(self, capture: Any) -> None:
        """释放当前句柄；只由采集线程调用，避免与 read 并发。"""
        try:
            capture.release()
        except Exception:
            pass
        with self._capture_lock:
            if self._capture is capture:
                self._capture = None

    def _capture_loop(self) -> None:
        """持续读取最新帧，断流后按指数退避重新连接。"""
        capture = self._take_capture()
        delay = max(float(getattr(self.args, "rtsp_reconnect_delay", 1.0)), 0.05)
        max_delay = max(float(getattr(self.args, "rtsp_reconnect_max_delay", 8.0)), delay)
        while not self._stop.is_set():
            if capture is None:
                try:
                    capture = self._open_capture()
                    with self._capture_lock:
                        self._capture = capture
                    delay = max(float(getattr(self.args, "rtsp_reconnect_delay", 1.0)), 0.05)
                except Exception as error:
                    self._set_error(str(error))
                    self._stop.wait(delay)
                    delay = min(delay * 2.0, max_delay)
                    continue
            try:
                ok, frame = capture.read()
            except Exception as error:
                ok, frame = False, None
                self._set_error(f"读取异常: {error}")
            if ok and isinstance(frame, np.ndarray) and frame.size:
                self.frames.publish(frame)
                self._set_error("")
                delay = max(float(getattr(self.args, "rtsp_reconnect_delay", 1.0)), 0.05)
                continue
            self._set_error("RTSP 读取失败，准备重连")
            self._drop_capture(capture)
            capture = None
            self._stop.wait(delay)
            delay = min(delay * 2.0, max_delay)
        if capture is not None:
            self._drop_capture(capture)

    def read(self, timeout: float) -> np.ndarray:
        """返回下一张最新 BGR 帧；超时会给出 RTSP 错误上下文。"""
        sequence, frame = self.frames.wait_next(self._last_sequence, timeout)
        if sequence <= self._last_sequence or frame is None:
            with self._error_lock:
                error = self._last_error
            detail = f"（{error}）" if error else ""
            raise TimeoutError(f"等待 RTSP 视频帧超时 {float(timeout):.1f} 秒{detail}")
        self._last_sequence = sequence
        return frame

    def release(self) -> None:
        """停止采集线程并释放 OpenCV 句柄。"""
        self._stop.set()
        self.frames.close()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=3.0)
            if thread.is_alive():
                # 某些 FFmpeg 构建在网络读超时属性失效时会卡在 read()；
                # 释放句柄可唤醒底层调用，避免自动回退时遗留 RTSP 线程。
                capture = self._take_capture()
                if capture is not None:
                    try:
                        capture.release()
                    except Exception:
                        pass
                thread.join(timeout=1.0)
            if thread.is_alive():
                self._set_error("RTSP 采集线程未能在退出超时内结束")
            self._thread = None
        else:
            capture = self._take_capture()
            if capture is not None:
                self._drop_capture(capture)
        with self._capture_lock:
            self._capture = None


class MjpegSource(RtspSource):
    """Consume the main camera service MJPEG stream without opening V4L2 again."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args)
        self.url = str(getattr(args, "mjpeg_url", "")).strip()
        if not self.url:
            raise ValueError("MJPEG 采集地址不能为空")

    def _set_error(self, message: str) -> None:
        with self._error_lock:
            if message == self._last_error:
                return
            self._last_error = message
        if message:
            print(f"[MJPEG] {message}", file=sys.stderr, flush=True)

    def _open_capture(self) -> Any:
        backend = getattr(cv2, "CAP_FFMPEG", getattr(cv2, "CAP_ANY", 0))
        try:
            capture = cv2.VideoCapture(self.url, backend)
        except (TypeError, cv2.error):
            capture = cv2.VideoCapture(self.url)
        if capture is None or not capture.isOpened():
            if capture is not None:
                capture.release()
            raise ConnectionError(f"OpenCV 无法打开 MJPEG: {self.url}")
        try:
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except (AttributeError, cv2.error):
            pass
        try:
            width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = float(capture.get(cv2.CAP_PROP_FPS))
        except (AttributeError, cv2.error, TypeError, ValueError):
            width, height, fps = 0, 0, 0.0
        now = time.monotonic()
        if now - self._last_connect_log >= 2.0:
            print(f"[MJPEG] 已连接: {self.url}，{width}x{height}，{fps:.1f} FPS", flush=True)
            self._last_connect_log = now
        return capture

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="usb-mjpeg-yolo", daemon=True)
        self._thread.start()

    def _capture_loop(self) -> None:
        capture = self._take_capture()
        delay = max(float(getattr(self.args, "rtsp_reconnect_delay", 1.0)), 0.05)
        max_delay = max(float(getattr(self.args, "rtsp_reconnect_max_delay", 8.0)), delay)
        while not self._stop.is_set():
            if capture is None:
                try:
                    capture = self._open_capture()
                    with self._capture_lock:
                        self._capture = capture
                    delay = max(float(getattr(self.args, "rtsp_reconnect_delay", 1.0)), 0.05)
                except Exception as error:
                    self._set_error(str(error))
                    self._stop.wait(delay)
                    delay = min(delay * 2.0, max_delay)
                    continue
            try:
                ok, frame = capture.read()
            except Exception as error:
                ok, frame = False, None
                self._set_error(f"读取异常: {error}")
            if ok and isinstance(frame, np.ndarray) and frame.size:
                self.frames.publish(frame)
                self._set_error("")
                continue
            self._set_error("MJPEG 读取失败，准备重连")
            self._drop_capture(capture)
            capture = None
            self._stop.wait(delay)
            delay = min(delay * 2.0, max_delay)
        if capture is not None:
            self._drop_capture(capture)

    def read(self, timeout: float) -> np.ndarray:
        sequence, frame = self.frames.wait_next(self._last_sequence, timeout)
        if sequence <= self._last_sequence or frame is None:
            with self._error_lock:
                error = self._last_error
            detail = f"（{error}）" if error else ""
            raise TimeoutError(f"等待 MJPEG 视频帧超时 {float(timeout):.1f} 秒{detail}")
        self._last_sequence = sequence
        return frame


# 兼容调用方使用的短名称。
HikvisionRtspSource = RtspSource


class HikvisionJpegSource:
    """使用 NET_DVR_CaptureJPEGPicture_NEW 的稳定单帧采集源。

    该路线完全绕过 ``NET_DVR_RealPlay_V40`` 和 PlayM4，适合网页 YOLO：每次
    只向设备请求一张 JPEG，再在 Python 进程中解码和推理。它的延迟和帧率不如
    连续码流，但不会受到原生解码回调线程的 ABI/UAF 问题影响；默认网页 YOLO
    需要 SDK 抓图时可通过 ``--capture-backend jpeg`` 显式启用；V40 仍可通过
    ``--capture-backend v40`` 显式启用。
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.sdk: Any = None
        self.user_id = -1
        self.device_info = DeviceInfoV30()
        self.frames = JpegFrameStats()
        self._jpeg_buffer = create_string_buffer(16 * 1024 * 1024)
        self._jpeg_para = JpegPara(255, 0)

    def _sdk_error(self, prefix: str) -> str:
        """组合 SDK 错误码，便于区分登录和抓图失败。"""
        code = int(self.sdk.NET_DVR_GetLastError())
        return f"{prefix}: {code}" + (f"（{SDK_ERRORS[code]}）" if code in SDK_ERRORS else "")

    def _configure_libraries(self) -> None:
        """只加载 HCNetSDK，不加载 PlayM4，减少原生组件和线程数量。"""
        sdk_dir = self.args.sdk_dir
        library = sdk_dir / "libhcnetsdk.so"
        if not library.is_file():
            raise FileNotFoundError(f"缺少 HCNetSDK 动态库: {library}")
        self.sdk = CDLL(str(library))
        self.sdk.NET_DVR_SetSDKInitCfg.argtypes = [c_int, c_void_p]
        self.sdk.NET_DVR_SetSDKInitCfg.restype = c_bool
        self.sdk.NET_DVR_Init.restype = c_bool
        self.sdk.NET_DVR_SetConnectTime.argtypes = [c_uint, c_uint]
        self.sdk.NET_DVR_SetConnectTime.restype = c_bool
        self.sdk.NET_DVR_Login_V30.argtypes = [c_char_p, c_ushort, c_char_p, c_char_p, POINTER(DeviceInfoV30)]
        self.sdk.NET_DVR_Login_V30.restype = c_int
        self.sdk.NET_DVR_GetLastError.restype = c_uint
        self.sdk.NET_DVR_CaptureJPEGPicture_NEW.argtypes = [c_int, c_int, POINTER(JpegPara), c_void_p, c_uint, POINTER(c_uint)]
        self.sdk.NET_DVR_CaptureJPEGPicture_NEW.restype = c_bool
        self.sdk.NET_DVR_Logout.argtypes = [c_int]
        self.sdk.NET_DVR_Logout.restype = c_bool
        self.sdk.NET_DVR_Cleanup.restype = c_bool

        local_path = LocalSdkPath()
        local_path.sPath = os.fsencode(sdk_dir)
        for kind, value in (
            (2, byref(local_path)),
            (3, create_string_buffer(os.fsencode(sdk_dir / "libcrypto.so.1.1"))),
            (4, create_string_buffer(os.fsencode(sdk_dir / "libssl.so.1.1"))),
        ):
            if not self.sdk.NET_DVR_SetSDKInitCfg(kind, value):
                raise RuntimeError(self._sdk_error("SDK 路径配置失败"))
        if not self.sdk.NET_DVR_Init():
            raise RuntimeError(self._sdk_error("SDK 初始化失败"))
        self.sdk.NET_DVR_SetConnectTime(self.args.connect_timeout_ms, self.args.connect_retries)

    def connect(self) -> None:
        """初始化 SDK 并登录设备。"""
        try:
            self._configure_libraries()
            self.user_id = self.sdk.NET_DVR_Login_V30(
                self.args.host.encode(), self.args.sdk_port, self.args.username.encode(), self.args.password.encode(), byref(self.device_info)
            )
            if self.user_id < 0:
                raise RuntimeError(self._sdk_error("设备登录失败"))
        except Exception:
            self.release()
            raise

    def start(self) -> None:
        """抓图源无需建立持续预览句柄，保留统一生命周期接口。"""
        return

    def read(self, _timeout: float) -> np.ndarray:
        """从设备取得一张 JPEG，并在 Python 侧解码为 BGR。"""
        if self.sdk is None or self.user_id < 0:
            raise RuntimeError("JPEG 抓图源尚未登录")
        returned = c_uint(0)
        if not self.sdk.NET_DVR_CaptureJPEGPicture_NEW(
            self.user_id,
            self.args.channel,
            byref(self._jpeg_para),
            self._jpeg_buffer,
            sizeof(self._jpeg_buffer),
            byref(returned),
        ):
            raise RuntimeError(self._sdk_error("JPEG 抓图失败"))
        size = int(returned.value)
        if size <= 4 or size > sizeof(self._jpeg_buffer):
            raise RuntimeError(f"设备返回 JPEG 长度异常: {size}")
        # 只复制设备实际返回的长度，避免每帧先复制完整 16 MiB 缓冲区。
        payload = string_at(self._jpeg_buffer, size)
        # 某些设备会在 JPEG EOI 后附加私有元数据；只校验 SOI，交给 OpenCV
        # 判断实际图像是否可解码，避免把合法抓图误判成无效帧。
        if not payload.startswith(b"\xff\xd8"):
            raise RuntimeError("设备返回的数据不是有效 JPEG")
        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            raise RuntimeError("OpenCV 无法解码设备 JPEG")
        self.frames.mark()
        return frame

    def release(self) -> None:
        """停止抓图会话并释放 SDK。"""
        if self.sdk is None:
            return
        if self.user_id >= 0:
            try:
                self.sdk.NET_DVR_Logout(self.user_id)
            finally:
                self.user_id = -1
        try:
            self.sdk.NET_DVR_Cleanup()
        finally:
            self.sdk = None


def open_capture_source(args: argparse.Namespace) -> tuple[Any, np.ndarray]:
    """按参数打开采集源并返回首帧；``auto`` 依次尝试 RTSP、JPEG、V40。

    每个候选源都在失败后完整释放，避免重试时遗留登录句柄或播放器端口。
    """
    classes: dict[str, type[Any]] = {
        "mjpeg": MjpegSource,
        "jpeg": HikvisionJpegSource,
        "rtsp": RtspSource,
        "v40": HikvisionV40Source,
    }
    requested = getattr(args, "capture_backend", "mjpeg")
    # RTSP 不进入 SDK/PlayM4 原生线程，优先用于网页 YOLO；JPEG/V40 仍保留为
    # 设备关闭 RTSP 或旧型号不提供 RTSP 时的兼容回退。
    candidates = [requested] if requested != "auto" else ["rtsp", "jpeg", "v40"]
    errors: list[str] = []
    for name in candidates:
        source = classes[name](args)
        try:
            source.connect()
            source.start()
            frame = source.read(args.frame_timeout)
            print(f"相机采集源: {name}，首帧 {frame.shape[1]}x{frame.shape[0]}", flush=True)
            return source, frame
        except Exception as error:
            errors.append(f"{name}: {error}")
            try:
                source.release()
            except Exception as release_error:
                errors.append(f"{name} 释放失败: {release_error}")
            if requested != "auto":
                break
            print(f"警告: {name} 采集失败，尝试下一路线: {error}", file=sys.stderr, flush=True)
    raise RuntimeError("所有海康采集路线均失败: " + "；".join(errors))


def draw_detections(frame: np.ndarray, result: Any) -> int:
    """在 BGR 图像直接绘制 YOLO 检测框，避免依赖其他 YOLO 脚本。"""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return 0
    coordinates = boxes.xyxy.cpu().numpy().astype(int)
    confidences = boxes.conf.cpu().numpy()
    class_ids = boxes.cls.cpu().numpy().astype(int)
    drawn_count = 0
    for (x1, y1, x2, y2), score, class_id in zip(coordinates, confidences, class_ids):
        color = BOX_COLORS[class_id % len(BOX_COLORS)]
        name = result.names[class_id]
        if any(token in str(name).lower() for token in ('cigarette', 'butt')):
            continue  # Retired object detection, including this standalone demo.
        drawn_count += 1
        label = f"{name} {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        text_top = max(y1 - text_height - baseline - 4, 0)
        cv2.rectangle(frame, (x1, text_top), (x1 + text_width + 6, text_top + text_height + baseline + 4), color, cv2.FILLED)
        cv2.putText(frame, label, (x1 + 3, text_top + text_height + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
    return drawn_count


def draw_status(frame: np.ndarray, inference_fps: float, object_count: int, dropped_frames: int) -> None:
    """在检测画面左上角绘制实时性指标。"""
    text = f"Inference FPS: {inference_fps:.1f} | Objects: {object_count} | Dropped: {dropped_frames}"
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.rectangle(frame, (8, 8), (18 + width, 18 + height + baseline), (0, 0, 0), cv2.FILLED)
    cv2.putText(frame, text, (13, 13 + height), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)


def ensure_player_library_path(sdk_dir: Path) -> None:
    """按官方启动脚本配置播放器依赖，并在必要时重新执行当前进程。"""
    marker = "HIKVISION_YOLO_LOADER_READY"
    existing = os.environ.get("LD_LIBRARY_PATH", "").split(":")
    if os.environ.get(marker) == "1" or str(sdk_dir) in existing:
        return
    environment = os.environ.copy()
    prefix = f"{sdk_dir}:{sdk_dir / 'HCNetSDKCom'}"
    environment["LD_LIBRARY_PATH"] = prefix + (f":{environment['LD_LIBRARY_PATH']}" if environment.get("LD_LIBRARY_PATH") else "")
    environment[marker] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], environment)


def run(args: argparse.Namespace) -> None:
    """Connect to the selected stream and continuously run YOLO inference."""
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"模型不存在: {model_path}")
    if model_path.suffix.lower() == ".rknn":
        raise ValueError("当前脚本支持 PT/ONNX；RKNN 模型请使用 yolo26_rknn_infer.py")
    from ultralytics import YOLO

    model = YOLO(str(model_path), task="detect")
    warmup = np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8)
    started = time.perf_counter()
    model.predict(warmup, conf=args.conf, imgsz=args.imgsz, device=args.device, verbose=False)
    print(f"模型预热完成: {time.perf_counter() - started:.2f} 秒")
    source: Any | None = None
    web_server: YoloMJPEGServer | None = None
    processed = 0
    smoothed_fps = 0.0
    if args.web_port > 0:
        web_server = YoloMJPEGServer(args.web_host, args.web_port)
        web_server.start()
        print(f"网页结果流: http://{args.web_host}:{args.web_port}/", flush=True)
    if not args.no_window:
        cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    source_description = args.mjpeg_url if args.capture_backend == "mjpeg" else f"{args.host}:{args.sdk_port}"
    print(f"相机来源: {source_description}，采集路线 {args.capture_backend}")
    if args.no_window:
        print("OpenCV 窗口已禁用；按 Ctrl+C 退出")
    else:
        print("按 q、Esc 或关闭窗口退出")
    try:
        source, frame = open_capture_source(args)
        print(
            f"原始输出画面尺寸: {frame.shape[1]}x{frame.shape[0]}；"
            f"YOLO 内部输入尺寸 imgsz={args.imgsz}，不会改变输出帧尺寸",
            flush=True,
        )
        while args.max_frames == 0 or processed < args.max_frames:
            inference_started = time.perf_counter()
            result = model.predict(frame, conf=args.conf, imgsz=args.imgsz, device=args.device, verbose=False)[0]
            seconds = max(time.perf_counter() - inference_started, 1e-9)
            fps = 1.0 / seconds
            smoothed_fps = fps if smoothed_fps == 0.0 else 0.9 * smoothed_fps + 0.1 * fps
            count = draw_detections(frame, result)
            draw_status(frame, smoothed_fps, count, source.frames.dropped_count())
            processed += 1
            if web_server is not None:
                ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.web_quality])
                if ok:
                    web_server.publish(
                        jpeg.tobytes(),
                        inference_fps=smoothed_fps,
                        capture_fps=source.frames.capture_fps(),
                        frame_width=frame.shape[1],
                        frame_height=frame.shape[0],
                    )
            if not args.no_window:
                cv2.imshow(args.window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(args.window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            frame = source.read(args.frame_timeout)
    except KeyboardInterrupt:
        print("收到退出信号")
    finally:
        if web_server is not None:
            web_server.close()
        if source is not None:
            source.release()
        if not args.no_window:
            cv2.destroyAllWindows()
        dropped = source.frames.dropped_count() if source is not None else 0
        print(f"推理结束，共处理 {processed} 帧，跳过旧帧 {dropped} 帧")


def parse_args() -> argparse.Namespace:
    """解析并校验模型、相机来源与显示参数。"""
    parser = argparse.ArgumentParser(description="USB 相机 YOLO 实时检测", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="YOLO PT/ONNX 模型路径")
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR, help="HCNetSDK 库文件目录")
    parser.add_argument("--host", default=DEFAULT_HOST, help="海康相机 IP 或主机名")
    parser.add_argument("--sdk-port", type=int, default=8000, help="HCNetSDK 服务端口")
    parser.add_argument("--username", default="admin", help="海康设备用户名")
    parser.add_argument("--password", default=os.environ.get("HIKVISION_PASSWORD"), help="设备密码，也可设置 HIKVISION_PASSWORD")
    parser.add_argument("--channel", type=int, default=1, help="预览通道")
    parser.add_argument("--stream", type=int, choices=(0, 1), default=0, help="0 主码流，1 子码流")
    parser.add_argument("--link-mode", type=int, choices=range(8), default=0, help="V40 链路：0 TCP，1 UDP，2 组播，3 RTP，4 RTSP/RTP，5 RTSP/HTTP，6 HRUDP，7 RTSP/HTTPS")
    parser.add_argument("--capture-backend", choices=("mjpeg", "jpeg", "rtsp", "v40", "auto"), default="mjpeg", help="采集路线；USB 默认复用主相机服务的 MJPEG 流")
    parser.add_argument("--mjpeg-url", default="http://127.0.0.1:8766/api/stream", help="主相机服务的 MJPEG 输入地址")
    parser.add_argument("--rtsp-url", default=os.environ.get("HIKVISION_RTSP_URL", ""), help="自定义 RTSP 地址；为空时按通道和码流自动生成")
    parser.add_argument("--rtsp-port", type=int, default=554, help="RTSP 服务端口")
    parser.add_argument("--rtsp-transport", choices=("tcp", "udp"), default="tcp", help="RTSP 传输方式；TCP 更稳定")
    parser.add_argument("--rtsp-open-timeout-ms", type=int, default=5000, help="RTSP 建连超时毫秒")
    parser.add_argument("--rtsp-read-timeout-ms", type=int, default=5000, help="RTSP 读取超时毫秒")
    parser.add_argument("--rtsp-reconnect-delay", type=float, default=1.0, help="RTSP 断线首次重连等待秒数")
    parser.add_argument("--rtsp-reconnect-max-delay", type=float, default=8.0, help="RTSP 断线重连最大等待秒数")
    parser.add_argument("--connect-timeout-ms", type=int, default=3000, help="SDK 单次登录连接超时毫秒")
    parser.add_argument("--connect-retries", type=int, default=1, help="SDK 登录连接尝试次数")
    parser.add_argument("--frame-timeout", type=float, default=10.0, help="等待首帧或最新帧的超时秒数")
    parser.add_argument("--conf", type=float, default=0.50, help="检测置信度阈值；达到 50% 才确认")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO 推理输入尺寸")
    parser.add_argument("--device", default="cpu", help="推理设备，例如 cpu 或 CUDA 编号 0")
    parser.add_argument("--window-name", default="USB Camera YOLO Detection", help="OpenCV 窗口标题")
    parser.add_argument("--no-window", action="store_true", help="不打开 OpenCV 窗口，适合网页后台启动")
    parser.add_argument("--web-host", default="127.0.0.1", help="YOLO MJPEG 监听地址")
    parser.add_argument("--web-port", type=int, default=0, help="YOLO MJPEG 监听端口，0 表示不启用网页流")
    parser.add_argument("--web-quality", type=int, default=95, help="YOLO 网页结果流 JPEG 质量 1-100")
    parser.add_argument("--max-frames", type=int, default=0, help="最多处理帧数；0 表示持续运行")
    args = parser.parse_args()
    args.sdk_dir = args.sdk_dir.expanduser().resolve()
    if args.capture_backend != "mjpeg" and not args.password:
        parser.error("请通过 HIKVISION_PASSWORD 或 --password 提供设备密码")
    if args.capture_backend == "mjpeg" and not str(args.mjpeg_url).strip():
        parser.error("MJPEG 采集地址不能为空")
    if not 1 <= args.sdk_port <= 65535 or args.channel <= 0:
        parser.error("SDK 端口必须在 1 到 65535，通道必须大于 0")
    if args.connect_timeout_ms <= 0 or args.connect_retries <= 0 or args.frame_timeout <= 0:
        parser.error("连接超时、连接次数和帧超时必须大于 0")
    if not 1 <= args.rtsp_port <= 65535 or args.rtsp_open_timeout_ms <= 0 or args.rtsp_read_timeout_ms <= 0:
        parser.error("RTSP 端口和超时参数无效")
    if args.rtsp_reconnect_delay <= 0 or args.rtsp_reconnect_max_delay < args.rtsp_reconnect_delay:
        parser.error("RTSP 重连等待参数无效")
    if not 0.0 <= args.conf <= 1.0 or args.imgsz <= 0 or args.imgsz % 32 != 0 or args.max_frames < 0:
        parser.error("置信度、图像尺寸或最大帧数参数无效")
    if args.web_port < 0 or args.web_port > 65535:
        parser.error("网页端口必须在 0 到 65535 之间")
    if not 1 <= args.web_quality <= 100:
        parser.error("网页 JPEG 质量必须在 1 到 100 之间")
    return args


if __name__ == "__main__":
    arguments = parse_args()
    # RTSP 路线不加载 SDK/PlayM4，减少原生库对网页进程的影响；其它路线仍按
    # 官方启动脚本配置动态库搜索路径。
    if arguments.capture_backend in {"jpeg", "v40", "auto"}:
        ensure_player_library_path(arguments.sdk_dir)
    run(arguments)
