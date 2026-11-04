"""Shared, on-demand C12 RTSP previews. No gimbal or camera-setting commands."""
import os
from contextlib import suppress
import select
import shutil
import subprocess
import tempfile
import threading
import time


VISIBLE_URL = 'rtsp://192.168.144.108:554/stream=1'
INFRARED_URL = 'rtsp://192.168.144.108:555/stream=2'
C12_VIDEO_FILTER = "fps=10,scale='min(960,iw)':-2,hflip,vflip"


def jpeg_frames(buffer):
    """Extract complete JPEGs from FFmpeg's pipe, retaining a split next frame."""
    while True:
        start = buffer.find(b'\xff\xd8')
        if start < 0:
            del buffer[:-1]
            return
        if start:
            del buffer[:start]
        end = buffer.find(b'\xff\xd9', 2)
        if end < 0:
            return
        image = bytes(buffer[:end + 2])
        del buffer[:end + 2]
        yield image


class C12Source:
    """One decoder per channel, with shared latest frames and offline retry."""
    IDLE_SECONDS = 20
    FRAME_TIMEOUT = 8

    def __init__(self, url, frame_factory):
        self.url = url
        self.frames = frame_factory()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._worker = None
        self._process = None
        self._requested = 0.0
        self._frame_at = 0.0
        self._error = ''

    def _ensure_started(self):
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError('C12 视频服务已关闭')
            self._requested = time.monotonic()
            if self._worker is None or not self._worker.is_alive():
                # Preserve the sequence across decoder restarts.
                self._frame_at, self._error = 0.0, ''
                self._worker = threading.Thread(target=self._run, daemon=True,
                                                name='c12-rtsp')
                self._worker.start()

    def next_frame(self, version=0):
        sequence, image, _, _ = self.frame_packet(version)
        return sequence, image

    def frame_packet(self, version=0, timeout=None):
        self._ensure_started()
        return self.frames.behavior_packet(version, self.FRAME_TIMEOUT if timeout is None else timeout)

    def status(self):
        with self._lock:
            age = time.monotonic() - self._frame_at if self._frame_at else None
            return dict(ready=age is not None and age < 3 and not self._error,
                        running=self._worker is not None and self._worker.is_alive(),
                        frame_age_seconds=age, error=self._error)

    def _run(self):
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            self._failed('未安装 FFmpeg，请安装 ffmpeg 后重新打开预览')
            return
        while not self._stop.is_set():
            if time.monotonic() - self._requested > self.IDLE_SECONDS:
                self._failed('C12 预览空闲已释放，打开后自动重连')
                return
            process = None
            try:
                # Ubuntu 22.04 FFmpeg 4.4 uses stimeout for RTSP socket timeout.
                command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
                           '-rtsp_transport', 'tcp', '-stimeout', '5000000',
                           '-threads', '1', '-i', self.url, '-an',
                           '-vf', C12_VIDEO_FILTER, '-threads', '1',
                           '-q:v', '6', '-f', 'image2pipe', '-vcodec', 'mjpeg', 'pipe:1']
                with tempfile.TemporaryFile() as errors:
                    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors)
                    with self._lock:
                        self._process = process
                    buffer = bytearray()
                    received = time.monotonic()
                    while not self._stop.is_set():
                        now = time.monotonic()
                        if now - self._requested > self.IDLE_SECONDS:
                            self._failed('C12 预览空闲已释放，打开后自动重连')
                            return
                        if now - received > self.FRAME_TIMEOUT:
                            raise TimeoutError('C12 等待视频帧超时，正在重连')
                        if not select.select([process.stdout], [], [], .5)[0]:
                            continue
                        chunk = os.read(process.stdout.fileno(), 65536)
                        if not chunk:
                            errors.seek(0)
                            detail = errors.read(4096).decode('utf-8', errors='replace').strip()
                            raise RuntimeError(detail or 'C12 RTSP 连接已断开，正在重连')
                        buffer.extend(chunk)
                        for image in jpeg_frames(buffer):
                            received = time.monotonic()
                            self.frames.publish(image)
                            with self._lock:
                                self._frame_at, self._error = received, ''
                        if len(buffer) > 8 * 1024 * 1024:
                            raise RuntimeError('C12 视频帧不完整，正在重连')
            except (OSError, RuntimeError, TimeoutError) as error:
                self._failed(str(error))
            finally:
                if process is not None:
                    if process.poll() is None:
                        with suppress(ProcessLookupError):
                            process.terminate()
                        try:
                            process.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                    process.stdout.close()
                with self._lock:
                    self._process = None
            self._stop.wait(2)

    def _failed(self, error):
        with self._lock:
            self._error = error
            self._frame_at = 0.0
        self.frames.invalidate(error)

    def close(self):
        self._stop.set()
        with self._lock:
            process, worker = self._process, self._worker
        if process is not None and process.poll() is None:
            with suppress(ProcessLookupError):
                process.terminate()
        if worker is not None:
            worker.join(timeout=self.FRAME_TIMEOUT + 3)
        self._failed('C12 视频服务已关闭')


class C12OrdinarySource:
    """Feed the existing preview/inference pipeline from the shared C12 decoder."""
    def __init__(self, source, frame_factory):
        self.source, self.frames = source, frame_factory()
        self._stop = threading.Event()
        self._worker = None
        self.width = self.height = self.encoded_frames = 0
        self._frame_at = 0.0
        self.error = ''

    def start(self):
        self._worker = threading.Thread(target=self._run, daemon=True, name='c12-ordinary')
        self._worker.start()

    def _run(self):
        version = 0
        while not self._stop.is_set():
            try:
                version, image, stamp, _ = self.source.frame_packet(version, timeout=.5)
                if self._stop.is_set():
                    break
                if not self.width:
                    import cv2
                    import numpy as np
                    decoded = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if decoded is None:
                        raise RuntimeError('C12 可见光帧无法解码')
                    self.height, self.width = decoded.shape[:2]
                self.frames.publish(image, stamp_ns=stamp)
                self.encoded_frames += 1
                self._frame_at, self.error = time.monotonic(), ''
            except TimeoutError:
                continue
            except (OSError, RuntimeError, ValueError) as error:
                self.error = str(error)
                self.frames.invalidate(self.error)
                self._stop.wait(.2)

    def wait_for_first_frame(self, timeout):
        self.frames.behavior_packet(0, timeout)

    def next_frame(self, version, timeout=15.0):
        return self.frames.behavior_packet(version, timeout)[:2]

    def latest_frame(self, timeout=10.0):
        return self.frames.behavior_packet(0, timeout)[1]

    def info(self):
        return dict(backend='rtsp', source='c12', device=self.source.url,
                    width=self.width, height=self.height, fps=10, fourcc='JPEG',
                    encoded_frames=self.encoded_frames, dropped_frames=0,
                    frame_age_seconds=time.monotonic() - self._frame_at if self._frame_at else None,
                    error=self.error or None)

    def close(self):
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2)
        self.source.close()
        self.frames.invalidate('C12 普通画面来源已释放')


class C12InfraredPreview:
    MESSAGE = 'C12 仅提供红外画面，暂未接入测温与高温检测'

    def __init__(self, source):
        self.source = source

    def status(self):
        return {**self.source.status(), 'pipeline': 'c12', 'supports_temperature': False,
                'detection_enabled': False, 'manual_detection_enabled': False,
                'alarm_active': False, 'max_temperature_c': None,
                'temperature_error': self.MESSAGE}

    def frame(self, *, raw=False):
        return self.source.next_frame()[1]

    def open_stream(self):
        return C12MjpegStream(self.source)

    def config(self, task_id=''):
        return {'success': True, 'supports_temperature': False, 'error': self.MESSAGE}

    def update_config(self, payload):
        raise ValueError(self.MESSAGE)

    def set_detection(self, enabled):
        return self.status()

    def set_mode(self, mode, task_id=''):
        return {**self.status(), 'mode': mode, 'task_id': task_id}

    def capture(self, duration_seconds):
        raise RuntimeError(self.MESSAGE)

    def events(self, since):
        return {'events': [], 'sequence': since}


class C12MjpegStream:
    headers = {'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}

    def __init__(self, source):
        self.source = source
        self.sequence, self.pending = source.next_frame()
        self.closed = False

    def read(self, _size):
        if self.closed:
            return b''
        if self.pending is None:
            self.sequence, self.pending = self.source.next_frame(self.sequence)
        image, self.pending = self.pending, None
        return (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                + str(len(image)).encode() + b'\r\n\r\n' + image + b'\r\n')

    def close(self):
        self.closed = True
