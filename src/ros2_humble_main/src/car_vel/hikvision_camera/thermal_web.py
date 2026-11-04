#!/usr/bin/env python3
"""Loopback thermal camera process backed by the supplied HCUSBSDK demo.

The process is deliberately separate from ``camera_web.py``.  It is the only
owner of HCUSBSDK and survives browser churn; the existing camera process uses
HTTP as a production adapter and exposes same-origin proxy routes to the UI.
"""

from __future__ import annotations

import argparse
import faulthandler
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

from thermal_monitor import (
    DEFAULT_CLEAR_DELTA_C,
    DEFAULT_HOLD_SECONDS,
    DEFAULT_STALE_SECONDS,
    DEFAULT_THRESHOLD_C,
    MAX_THRESHOLD_C,
    MIN_THRESHOLD_C,
    ThermalConfigConflict,
    ThermalConfigStore,
    ThermalMonitor,
    ThermalSample,
)


DEFAULT_BUNDLE_DIR = Path("/opt/hik_thermal_linux64_bundle")


def enable_native_diagnostics() -> None:
    """Let SIGABRT include Python call stacks in the existing launch log.

    This is evidence collection, not a handler that suppresses native faults
    or automatically restarts a broken SDK instance.
    """
    try:
        faulthandler.enable(all_threads=True)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"[ThermalSDK] fault traceback unavailable: {error}",
              file=sys.stderr, flush=True)


def sdk_phase(name: str) -> None:
    print(f"[ThermalSDK] pid={os.getpid()} epoch={time.time():.3f} phase={name}",
          file=sys.stderr, flush=True)


def load_vendor_demo(bundle_dir: Path) -> Any:
    """Load the audited Python demo without copying or executing its CLI."""

    module_path = bundle_dir.expanduser().resolve() / "PythonDemo" / "thermal_max_cv.py"
    if not module_path.is_file():
        raise FileNotFoundError(f"找不到附件热成像适配模块：{module_path}")
    name = "hik_thermal_vendor_demo"
    spec = importlib.util.spec_from_file_location(name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载热成像适配模块：{module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class HCUsbProducer:
    """Production adapter that feeds ``ThermalMonitor`` from one SDK login."""

    def __init__(self, monitor: ThermalMonitor, args: argparse.Namespace) -> None:
        self.monitor = monitor
        self.args = args
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="hcusb-thermal-producer")

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=max(3.0, self.args.frame_timeout + 2.0))

    def _run(self) -> None:
        # A producer instance never reinitializes a failed native SDK. The
        # HTTP-side supervisor owns retries in fresh processes instead.
        try:
            self._run_session()
        except Exception as error:
            self.monitor.set_backend_state(running=False, error=str(error))
            return
        self.monitor.set_backend_state(running=False, error="热成像采集已停止")

    def _run_session(self) -> None:
        sdk_phase("loading_adapter")
        demo = load_vendor_demo(self.args.bundle_dir)
        sdk_dir = self.args.bundle_dir / "library" / "linux64"
        sdk_phase("initializing_sdk")
        with demo.HCUsbSDK(sdk_dir) as sdk:
            if self.args.log_dir is not None:
                sdk.set_log_directory(self.args.log_dir)
            sdk_phase("enumerating_devices")
            devices = sdk.enumerate_devices()
            if not devices:
                raise RuntimeError(
                    "未发现HCUSBSDK热成像设备；请检查USB连接、udev权限和设备占用")
            device = demo.choose_device(devices, self.args.device_index, self.args.serial)
            username = self.args.username or None
            password = os.environ.get("HIK_THERMAL_PASSWORD") if username else None
            if username and password is None:
                raise RuntimeError("设置热成像用户名时必须通过HIK_THERMAL_PASSWORD提供密码")
            sdk_phase("logging_in")
            user_id, _registration = sdk.login(
                device, self.args.login_timeout, username, password)
            original_stream = None
            original_video = None
            stream = None
            restore_stream = False
            restore_video = False
            try:
                sdk_phase("configuring_stream")
                original_stream = demo.get_thermal_stream_param(
                    sdk, user_id, self.args.channel)
                original_video = demo.get_video_param(sdk, user_id)
                capabilities = demo.get_video_capabilities(sdk, user_id)
                stream_type = int(original_stream.byVideoCodingType)
                if stream_type == demo.THERMAL_STREAM_TEMP_YUY2:
                    target_stream_type = stream_type
                    video_stream_type = demo.USB_STREAM_YUY2
                elif stream_type == demo.THERMAL_STREAM_HOT_JPEG:
                    target_stream_type = stream_type
                    video_stream_type = demo.USB_STREAM_MJPEG
                elif demo.video_modes_for_format(capabilities, demo.USB_STREAM_YUY2):
                    target = demo.make_yuy2_stream_param()
                    target_stream_type = demo.THERMAL_STREAM_TEMP_YUY2
                    video_stream_type = demo.USB_STREAM_YUY2
                    restore_stream = True
                    demo.set_thermal_stream_param(
                        sdk, user_id, self.args.channel, target,
                        self.args.command_timeout)
                elif demo.video_modes_for_format(capabilities, demo.USB_STREAM_MJPEG):
                    target = demo.make_hot_stream_param()
                    target_stream_type = demo.THERMAL_STREAM_HOT_JPEG
                    video_stream_type = demo.USB_STREAM_MJPEG
                    restore_stream = True
                    demo.set_thermal_stream_param(
                        sdk, user_id, self.args.channel, target,
                        self.args.command_timeout)
                else:
                    raise RuntimeError("设备能力中没有可用于测温画面的YUY2或MJPEG模式")

                current_video = demo.get_video_param(sdk, user_id)
                if target_stream_type == demo.THERMAL_STREAM_TEMP_YUY2:
                    target_video = demo.choose_yuy2_video_param(capabilities, current_video)
                else:
                    target_video = demo.choose_mjpeg_video_param(capabilities, current_video)
                restore_video = (
                    demo.valid_video_mode(original_video)
                    and not demo.same_video_config(original_video, target_video))
                demo.set_video_param(
                    sdk, user_id, target_video, self.args.command_timeout)

                stream = demo.ThermalStream(
                    sdk, user_id, target_stream_type, video_stream_type)
                sdk_phase("starting_stream")
                stream.start()
                sdk_phase("stream_started_waiting_frames")
                descriptor = (
                    f"{device.manufacturer or 'Hikvision'} {device.name or 'thermal'} "
                    f"serial={device.serial or '-'} "
                    f"VID:PID={device.native.dwVID:04x}:{device.native.dwPID:04x}")
                self.monitor.set_backend_state(
                    running=True, error="", sdk_version=sdk.version_string(),
                    device=descriptor)
                while not self._stop.is_set():
                    frame = stream.latest(self.args.frame_timeout)
                    image = demo.decode_frame_image(frame)
                    annotated = demo.annotate_frame(image, frame, self.args.box_size)
                    threshold = self.monitor.status()["config"]["threshold_c"]
                    label = f"ALARM >= {float(threshold):.1f} C"
                    demo.cv2.putText(
                        annotated, label, (8, max(20, annotated.shape[0] - 10)),
                        demo.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
                        demo.cv2.LINE_AA)
                    ok_raw, raw_buffer = demo.cv2.imencode(
                        ".jpg", image,
                        [demo.cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])
                    ok_annotated, annotated_buffer = demo.cv2.imencode(
                        ".jpg", annotated,
                        [demo.cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])
                    if not ok_raw or not ok_annotated:
                        raise RuntimeError("OpenCV无法编码热成像JPEG")
                    self.monitor.ingest(ThermalSample(
                        raw_jpeg=raw_buffer.tobytes(),
                        annotated_jpeg=annotated_buffer.tobytes(),
                        max_temperature_c=float(frame.max_temperature_c),
                        x_normalized=int(frame.x_normalized),
                        y_normalized=int(frame.y_normalized),
                        frame_number=int(frame.frame_number),
                        captured_at_epoch=time.time(),
                        width=int(image.shape[1]),
                        height=int(image.shape[0]),
                        frozen=bool(frame.frozen),
                        temperature_source=str(frame.temperature_source),
                    ))
            finally:
                sdk_phase("closing_stream")
                errors: list[str] = []
                if stream is not None:
                    try:
                        stream.close()
                    except BaseException as error:
                        errors.append(f"停止回调失败：{error}")
                if restore_stream and original_stream is not None and not self.args.keep_stream_type:
                    try:
                        original_stream.dwSize = demo.ctypes.sizeof(original_stream)
                        demo.set_thermal_stream_param(
                            sdk, user_id, self.args.channel, original_stream,
                            self.args.command_timeout)
                    except BaseException as error:
                        errors.append(f"恢复热流类型失败：{error}")
                if restore_video and original_video is not None and not self.args.keep_stream_type:
                    try:
                        demo.set_video_param(
                            sdk, user_id, original_video, self.args.command_timeout)
                    except BaseException as error:
                        errors.append(f"恢复视频参数失败：{error}")
                try:
                    sdk.logout(user_id)
                except BaseException as error:
                    errors.append(f"注销设备失败：{error}")
                if errors and not self._stop.is_set():
                    raise RuntimeError("；".join(errors))


class ThermalRequestHandler(BaseHTTPRequestHandler):
    server_version = "HikThermal/1.0"
    protocol_version = "HTTP/1.1"

    @property
    def monitor(self) -> ThermalMonitor:
        return self.server.monitor  # type: ignore[attr-defined]

    def log_message(self, format: str, *args: object) -> None:
        if args and str(args[1] if len(args) > 1 else "") in {"200", "304"}:
            return
        super().log_message(format, *args)

    def _json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _image(self, image: bytes, sequence: int | None = None) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        if sequence is not None:
            self.send_header("X-Frame-Sequence", str(sequence))
        self.send_header("Content-Length", str(len(image)))
        self.end_headers()
        self.wfile.write(image)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("Content-Length无效") from error
        if length < 0 or length > 16 * 1024:
            raise ValueError("请求正文过大")
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        try:
            if parsed.path in {"/healthz", "/api/status"}:
                self._json(HTTPStatus.OK, self.monitor.status())
            elif parsed.path == "/api/config":
                self._json(HTTPStatus.OK, self.monitor.config_store.get(
                    query.get("task_id", [""])[0]))
            elif parsed.path == "/api/events":
                since = max(0, int(query.get("since", ["0"])[0]))
                self._json(HTTPStatus.OK, self.monitor.events(since))
            elif parsed.path == "/api/frame.jpg":
                self._image(self.monitor.snapshot(annotated=True))
            elif parsed.path == "/api/raw.jpg":
                self._image(self.monitor.snapshot(annotated=False))
            elif parsed.path == "/api/stream.mjpg":
                self._stream_mjpeg()
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
        except (ValueError, RuntimeError, TimeoutError, OSError) as error:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(error)})
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/detection":
                result = self.monitor.set_detection(bool(payload.get("enabled")))
            elif path == "/api/mode":
                result = self.monitor.set_mode(
                    str(payload.get("mode", "")), str(payload.get("task_id", "")))
            elif path == "/api/config":
                if payload.get("operation") == "clear_task":
                    result = self.monitor.config_store.clear_task(
                        str(payload.get("task_id", "")),
                        expected_version=payload.get("expected_version"))
                else:
                    result = self.monitor.config_store.update(
                        payload.get("threshold_c"),
                        scope=str(payload.get("scope", "global")),
                        task_id=str(payload.get("task_id", "")),
                        expected_version=payload.get("expected_version"),
                        actor=str(payload.get("actor", "web-ui")))
            elif path == "/api/capture":
                result = self.monitor.capture(float(payload.get("duration_seconds", 2.0)))
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
                return
            self._json(HTTPStatus.OK, {"success": True, **result})
        except ThermalConfigConflict as error:
            self._json(HTTPStatus.CONFLICT, {"error": str(error), "conflict": True})
        except (ValueError, RuntimeError, TimeoutError, OSError, json.JSONDecodeError) as error:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(error)})

    def _stream_mjpeg(self) -> None:
        slots = self.server.stream_slots  # type: ignore[attr-defined]
        if not slots.acquire(blocking=False):
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "热成像视频连接数已满"})
            return
        sequence = 0
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.connection.settimeout(self.server.stream_write_timeout)  # type: ignore[attr-defined]
            while True:
                sequence, image = self.monitor.next_frame(sequence, timeout=5.0)
                header = (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(image)).encode("ascii")
                    + b"\r\n\r\n")
                self.wfile.write(header)
                self.wfile.write(image)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        finally:
            slots.release()


class ThermalHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        monitor: ThermalMonitor,
        *,
        max_stream_clients: int = 6,
        stream_write_timeout: float = 10.0,
    ) -> None:
        super().__init__(address, ThermalRequestHandler)
        self.monitor = monitor
        self.stream_slots = threading.BoundedSemaphore(max_stream_clients)
        self.stream_write_timeout = float(stream_write_timeout)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="海康USB热成像共享取流与高温检测",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--bundle-dir", type=Path, default=DEFAULT_BUNDLE_DIR)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--serial", default="")
    parser.add_argument("--channel", type=int, default=1)
    parser.add_argument("--username", default=os.environ.get("HIK_THERMAL_USERNAME", ""))
    parser.add_argument("--login-timeout", type=int, default=5000)
    parser.add_argument("--command-timeout", type=float, default=5.0)
    parser.add_argument("--frame-timeout", type=float, default=5.0)
    parser.add_argument("--reconnect-delay", type=float, default=3.0)
    parser.add_argument("--box-size", type=int, default=48)
    parser.add_argument("--jpeg-quality", type=int, default=88)
    parser.add_argument("--keep-stream-type", action="store_true")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument(
        "--config", type=Path,
        default=Path.home() / ".ros" / "robot_navigation" / "thermal_config.json")
    parser.add_argument("--default-threshold-c", type=float, default=DEFAULT_THRESHOLD_C)
    parser.add_argument("--minimum-threshold-c", type=float, default=MIN_THRESHOLD_C)
    parser.add_argument("--maximum-threshold-c", type=float, default=MAX_THRESHOLD_C)
    parser.add_argument("--hold-seconds", type=float, default=DEFAULT_HOLD_SECONDS)
    parser.add_argument("--clear-delta-c", type=float, default=DEFAULT_CLEAR_DELTA_C)
    parser.add_argument("--stale-seconds", type=float, default=DEFAULT_STALE_SECONDS)
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--max-stream-clients", type=int, default=6)
    parser.add_argument("--stream-write-timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    args.bundle_dir = args.bundle_dir.expanduser().resolve()
    args.config = args.config.expanduser().resolve()
    if args.log_dir is not None:
        args.log_dir = args.log_dir.expanduser().resolve()
    finite_positive = (
        args.command_timeout, args.frame_timeout, args.reconnect_delay,
        args.stale_seconds, args.stream_write_timeout)
    if not all(math.isfinite(value) and value > 0 for value in finite_positive):
        parser.error("超时和重连参数必须是大于0的有限数")
    if args.device_index < 0 or not 1 <= args.channel <= 255:
        parser.error("设备序号或通道无效")
    if not 1000 <= args.login_timeout <= 5000:
        parser.error("登录超时必须在1000..5000毫秒")
    if args.hold_seconds < 0 or args.clear_delta_c <= 0:
        parser.error("持续时间不能为负且解除回差必须大于0")
    if not 1 <= args.jpeg_quality <= 100 or args.box_size < 4:
        parser.error("JPEG质量或最高温框尺寸无效")
    if not 1 <= args.port <= 65535 or args.max_stream_clients <= 0:
        parser.error("端口或视频客户端上限无效")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    enable_native_diagnostics()
    store = ThermalConfigStore(
        args.config,
        default_threshold_c=args.default_threshold_c,
        minimum_c=args.minimum_threshold_c,
        maximum_c=args.maximum_threshold_c)
    monitor = ThermalMonitor(
        store,
        hold_seconds=args.hold_seconds,
        clear_delta_c=args.clear_delta_c,
        stale_seconds=args.stale_seconds)
    from thermal_recovery import IsolatedThermalProducer
    producer = IsolatedThermalProducer(monitor, args)
    server = ThermalHttpServer(
        (args.listen, args.port), monitor,
        max_stream_clients=args.max_stream_clients,
        stream_write_timeout=args.stream_write_timeout)
    stop_requested = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        if stop_requested.is_set():
            return
        stop_requested.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    producer.start()
    print(f"热成像服务: http://{args.listen}:{args.port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        server.server_close()
        producer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

