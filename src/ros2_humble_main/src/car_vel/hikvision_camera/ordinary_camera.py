"""Ordinary-camera configuration and the original robot HTTP image source.

This module never opens the thermal camera or enables a robot motion session.
"""
from __future__ import annotations

import copy
import importlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading
import time
from urllib.parse import urlsplit, urlunsplit
from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler, Request
import uuid

DEFAULT_USB_DEVICE = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
DEFAULT_BODY_URL = 'http://127.0.0.1:8081/camera'
DEFAULTS = dict(version=0, pipeline='legacy', source='usb', usb_device=DEFAULT_USB_DEVICE,
                usb_width=1280, usb_height=720, usb_fps=30.0, body_url=DEFAULT_BODY_URL)


def validate_config(value, *, forbidden_ports=(8766, 8767, 8877)):
    # Existing saved USB/body configurations keep their original selection.
    if isinstance(value, dict) and 'pipeline' not in value:
        value = {'pipeline': 'legacy', **value}
    if not isinstance(value, dict) or set(value) != set(DEFAULTS):
        raise ValueError('普通摄像头配置字段不完整或包含未知字段，请刷新设置页')
    result = copy.deepcopy(value)
    if result['pipeline'] not in ('legacy', 'c12'):
        raise ValueError('请选择原有相机链路或 C12 相机')
    if result['source'] not in ('usb', 'body'):
        raise ValueError('请选择 USB 摄像头或狗本体摄像头')
    if type(result['version']) is not int or result['version'] < 0:
        raise ValueError('摄像头配置版本无效')
    path = result['usb_device']
    if not isinstance(path, str) or len(path) > 512 or not re.fullmatch(
            r'/dev/(?:video\d+|v4l/(?:by-id|by-path)/[A-Za-z0-9_.:+-]+)', path):
        raise ValueError('USB 设备必须是 /dev/videoN 或 /dev/v4l 下的稳定设备路径')
    if any(word in path.lower() for word in ('hikcamera', 'thermal')):
        raise ValueError('普通摄像头不能选择海康热成像设备')
    for field, low, high in (('usb_width',160,3840),('usb_height',120,2160)):
        if type(result[field]) is not int or not low <= result[field] <= high:
            raise ValueError('USB 分辨率超出允许范围')
    fps = result['usb_fps']
    if type(fps) not in (int, float) or not math.isfinite(fps) or not 1 <= fps <= 60:
        raise ValueError('USB 帧率必须在 1–60 之间')
    url = result['body_url']
    if not isinstance(url, str) or not url or len(url) > 2048 or any(ord(c)<33 for c in url):
        raise ValueError('本体图传地址无效')
    try:
        parsed = urlsplit(url)
        port = parsed.port or (443 if parsed.scheme == 'https' else 80)
        if parsed.scheme not in ('http','https') or not parsed.hostname or not 1 <= port <= 65535:
            raise ValueError()
        if parsed.username is not None or parsed.password is not None or parsed.fragment:
            raise ValueError()
    except ValueError as error:
        raise ValueError('本体图传只支持无内嵌账号密码的 HTTP/HTTPS JPEG 或 MJPEG 地址') from error
    if port in forbidden_ports:
        raise ValueError('本体图传不能指向普通/热成像/识别网关自身，避免循环取流')
    result['usb_fps'] = float(fps)
    return result


def source_kind(config):
    return 'c12' if config.get('pipeline') == 'c12' else config['source']


def source_label(config):
    return {'usb': 'USB摄像头', 'body': '狗本体摄像头',
            'c12': 'C12可见光相机'}[source_kind(config)]


class CameraSourceStore:
    def __init__(self, path, defaults=None, *, forbidden_ports=(8766,8767,8877)):
        self.path = Path(path).expanduser() if path is not None else None
        self.forbidden_ports = forbidden_ports
        self.defaults = validate_config({**DEFAULTS, **(defaults or {})}, forbidden_ports=forbidden_ports)
        self.current = copy.deepcopy(self.defaults)
        self.error = ''
        if self.path is not None and self.path.exists():
            try:
                self.current = validate_config(json.loads(self.path.read_text(encoding='utf-8')),
                                               forbidden_ports=forbidden_ports)
            except (ValueError, OSError) as error:
                self.error = f'普通摄像头配置读取失败，未自动切换来源：{error}'

    def active(self):
        if self.error: raise RuntimeError(self.error)
        return copy.deepcopy(self.current)

    def check_version(self, expected_version):
        if type(expected_version) is not int or expected_version != self.current['version']:
            raise ValueError('摄像头配置已更新，请刷新后重试')
        if self.path is not None:
            disk = CameraSourceStore(self.path, self.defaults, forbidden_ports=self.forbidden_ports)
            if disk.current['version'] != expected_version:
                raise ValueError('磁盘摄像头配置已更新，请刷新或重新启动后台后重试')

    def save(self, value, *, expected_version):
        candidate = validate_config(value, forbidden_ports=self.forbidden_ports)
        self.check_version(expected_version)
        candidate['version'] = expected_version + 1
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.error and self.path.is_file():
                backup = self.path.with_name(self.path.name + '.invalid-' + uuid.uuid4().hex)
                with self.path.open('rb') as source, backup.open('xb') as target:
                    shutil.copyfileobj(source, target)
            fd, temporary = tempfile.mkstemp(prefix='.'+self.path.name+'.', dir=self.path.parent)
            try:
                with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                    json.dump(candidate, stream, ensure_ascii=False, indent=2)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary): os.unlink(temporary)
        self.current = candidate
        self.error = ''
        return copy.deepcopy(candidate)


def _text(path):
    try: return Path(path).read_text(encoding='utf-8').strip()
    except OSError: return ''


def _usb_details(entry):
    device = (entry/'device').resolve()
    for parent in (device, *device.parents):
        vid, pid = _text(parent/'idVendor'), _text(parent/'idProduct')
        if vid and pid: return vid.lower(), pid.lower()
    return '', ''


def discover_usb_cameras(sys_root=Path('/sys/class/video4linux'), dev_root=Path('/dev')):
    sys_root, dev_root = Path(sys_root), Path(dev_root)
    result = []
    for entry in sorted(sys_root.glob('video*')):
        if not re.fullmatch(r'video\d+', entry.name): continue
        name, index = _text(entry/'name'), _text(entry/'index')
        vid, pid = _usb_details(entry)
        if index != '0' or (vid,pid)==('2bdf','0102') or 'hikcamera' in name.lower(): continue
        node = '/dev/' + entry.name
        aliases = []
        for folder in ('by-id','by-path'):
            for link in sorted((dev_root/'v4l'/folder).glob('*')):
                if link.is_symlink() and link.resolve()==(dev_root/entry.name).resolve():
                    aliases.append('/dev/v4l/'+folder+'/'+link.name)
        result.append(dict(node=node, device=aliases[0] if aliases else node,
                           aliases=aliases, name=name or entry.name, vid=vid, pid=pid))
    return result


def check_usb_device(path):
    """Check sysfs identity BEFORE opening a V4L2 handle, including symlink aliases."""
    node = Path(os.path.realpath(path)).name
    if not re.fullmatch(r'video\d+', node):
        raise RuntimeError('USB 稳定链接不存在或没有指向视频采集设备')
    entry = Path('/sys/class/video4linux')/node
    vid, pid = _usb_details(entry)
    if (vid,pid)==('2bdf','0102') or 'hikcamera' in _text(entry/'name').lower():
        raise RuntimeError('拒绝将热成像 UVC 节点作为普通摄像头打开')
    if _text(entry/'index') not in ('','0'):
        raise RuntimeError('所选节点是元数据接口，请选择 video-index0 采集接口')


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise RuntimeError('本体图传发生重定向，请填写直接的 JPEG/MJPEG 地址')


class HttpCameraSource:
    """One bounded HTTP JPEG/MJPEG reader, shared by preview and inference.

    The known local dog-camera proxy exposes a camera-only lease. No remote
    enable, standing, velocity, or navigation command is ever issued here.
    """
    MAX_FRAME = 8 * 1024 * 1024

    def __init__(self, url, frames, *, opener=None, decoder=None):
        self.url, self.frames = url, frames
        self._opener = opener or build_opener(ProxyHandler({}), _NoRedirect())
        self._decoder = decoder or self._decode
        self._stop = threading.Event()
        self._worker = None
        self._response = None
        self._response_lock = threading.Lock()
        self.error = ''
        self.encoded_frames = 0
        self.dropped_frames = 0
        self.width = self.height = 0
        self._last_frame = 0.0
        self._token = uuid.uuid4().hex
        parsed = urlsplit(url)
        self._lease_url = (urlunsplit((parsed.scheme, parsed.netloc, '/camera/session','',''))
            if parsed.scheme=='http' and parsed.hostname in ('localhost','127.0.0.1','::1')
            and parsed.path=='/camera' else '')
        self._lease_acquired = False
        self._renew_at = 0.0

    @staticmethod
    def _decode(image):
        cv2, np = importlib.import_module('cv2'), importlib.import_module('numpy')
        frame = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None or not frame.size: raise RuntimeError('本体返回的 JPEG 无法解码')
        return int(frame.shape[1]), int(frame.shape[0])

    def _lease(self, operation):
        if not self._lease_url: return
        payload = json.dumps(dict(operation=operation, token=self._token)).encode()
        req = Request(self._lease_url, data=payload, headers={'Content-Type':'application/json'})
        with self._opener.open(req, timeout=1.5) as response:
            value = json.loads(response.read(4096))
            if not value.get('success'): raise RuntimeError('本体图传启流请求被拒绝')
        self._lease_acquired = operation != 'release'
        self._renew_at = time.monotonic() + 2.0

    def _publish(self, image):
        if len(image) > self.MAX_FRAME: raise RuntimeError('本体 JPEG 帧过大')
        width, height = self._decoder(image)
        if self._stop.is_set(): return
        self.width, self.height = width, height
        self.frames.publish(image)
        self.encoded_frames += 1
        self._last_frame = time.monotonic()
        self.error = ''

    def _read(self, response):
        content_type = response.headers.get('Content-Type','').lower()
        if not (content_type.startswith('image/jpeg') or
                content_type.startswith('multipart/x-mixed-replace')):
            raise RuntimeError('本体地址未返回 JPEG/MJPEG 图像（可能是网页或登录页）')
        if content_type.startswith('image/jpeg'):
            data = response.read(self.MAX_FRAME+1)
            if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
                raise RuntimeError('本体 JPEG 帧不完整')
            self._publish(data)
            return
        buffer = bytearray()
        read = getattr(response, 'read1', response.read)
        while not self._stop.is_set():
            if self._lease_url and time.monotonic() >= self._renew_at: self._lease('acquire')
            chunk = read(16384)
            if not chunk: raise RuntimeError('本体图传连接已断开')
            buffer.extend(chunk)
            while True:
                start = buffer.find(b'\xff\xd8')
                if start < 0:
                    if len(buffer)>65536: raise RuntimeError('本体 MJPEG 未发现图像帧')
                    break
                if start: del buffer[:start]
                end = buffer.find(b'\xff\xd9', 2)
                if end < 0:
                    if len(buffer)>self.MAX_FRAME: raise RuntimeError('本体 MJPEG 帧过大或损坏')
                    break
                image = bytes(buffer[:end+2])
                del buffer[:end+2]
                self._publish(image)

    def start(self):
        self._worker = threading.Thread(target=self._loop, daemon=True, name='body-http-camera')
        self._worker.start()

    def _loop(self):
        try:
            while not self._stop.is_set():
                try:
                    self._lease('acquire')
                    req = Request(self.url, headers={'Accept':'multipart/x-mixed-replace,image/jpeg'})
                    with self._opener.open(req, timeout=2.0) as response:
                        with self._response_lock: self._response = response
                        self._read(response)
                    if not self._stop.is_set(): self._stop.wait(0.05)
                except Exception as error:
                    self.error = f'本体摄像头不可用：{error}'
                    self.dropped_frames += 1
                    self.frames.invalidate(self.error)
                    self._stop.wait(1.0)
                finally:
                    with self._response_lock: self._response = None
        finally:
            if self._lease_url:
                try: self._lease('release')
                except Exception: pass  # The bounded proxy lease expires after a crash as well.

    def wait_for_first_frame(self, timeout): self.frames.latest(timeout)
    def next_frame(self, version, timeout=15.0): return self.frames.next_after(version,timeout)
    def latest_frame(self, timeout=10.0): return self.frames.latest(timeout)
    def info(self):
        return dict(backend='http', source='body', device=self.url,
                    width=self.width, height=self.height, fps=0, fourcc='JPEG',
                    encoded_frames=self.encoded_frames, dropped_frames=self.dropped_frames,
                    frame_age_seconds=round(time.monotonic()-self._last_frame,3) if self._last_frame else None,
                    error=self.error or None)
    def close(self):
        self._stop.set()
        self.frames.invalidate('本体摄像头视频源已停止')
        if self._worker is not None:
            self._worker.join(timeout=4.0)
            if self._worker.is_alive():
                raise RuntimeError('本体取流线程尚未退出，未启动第二个来源；请稍后重试')
