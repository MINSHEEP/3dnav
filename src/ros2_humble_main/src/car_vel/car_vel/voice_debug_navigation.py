"""Cancellable debug-only navigation; no ROS imports in the voice virtualenv."""
from __future__ import annotations

import base64
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import threading
import time
import uuid


MAP_NAME = 'shiyanshi1'
ROUTE_NAME = 'luxian1'
NAVIGATION_MODE = 'map_2d'


class NavigationCancelled(Exception):
    pass


def check_cancel(cancel):
    if cancel.is_set():
        raise NavigationCancelled()


class RosbridgeClient:
    """One receive thread; service waits never block the motion watchdog."""
    def __init__(self, url, on_message):
        self.url, self.on_message = url, on_message
        self.socket = None
        self.send_lock = threading.Lock()
        self.condition = threading.Condition()
        self.messages = {}
        self.pending = {}
        self.closed = threading.Event()

    def connect(self, cancel, timeout=60):
        import websocket
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check_cancel(cancel)
            try:
                self.socket = websocket.create_connection(
                    self.url, timeout=0.35, enable_multithread=True,
                    http_no_proxy=['127.0.0.1', 'localhost'])
                threading.Thread(target=self._receive, daemon=True,
                                 name='debug-nav-receiver').start()
                return
            except (OSError, websocket.WebSocketException):
                cancel.wait(0.2)
        raise RuntimeError('导航连接启动超时，请查看终端中的 ROS 启动信息。')

    def send(self, data):
        if self.closed.is_set() or not self.socket:
            raise RuntimeError('导航连接已中断。')
        with self.send_lock:
            self.socket.send(json.dumps(data, ensure_ascii=False, allow_nan=False))

    def subscribe(self, topic, kind, *, identity=None, throttle=0):
        self.send({'op': 'subscribe', 'id': identity or topic, 'topic': topic,
                   'type': kind, 'queue_length': 1, 'throttle_rate': throttle})

    def publish(self, topic, kind, message, *, latch=False):
        self.send({'op': 'advertise', 'topic': topic, 'type': kind, 'latch': latch})
        self.send({'op': 'publish', 'topic': topic, 'msg': message})

    def call(self, service, kind, args, cancel, timeout=15):
        identity = uuid.uuid4().hex
        with self.condition:
            self.pending[identity] = None
        try:
            check_cancel(cancel)
            self.send({'op': 'call_service', 'id': identity, 'service': service,
                       'type': kind, 'args': args})
            deadline = time.monotonic() + timeout
            with self.condition:
                while self.pending[identity] is None:
                    check_cancel(cancel)
                    if self.closed.is_set():
                        raise RuntimeError('导航连接已中断。')
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'导航服务等待超时：{service}')
                    self.condition.wait(0.05)
                response = self.pending[identity]
            if response.get('result') is False:
                raise RuntimeError(str(response.get('values') or service + ' 调用失败'))
            return response.get('values', {})
        finally:
            with self.condition:
                self.pending.pop(identity, None)

    def latest(self, topic):
        with self.condition:
            return self.messages.get(topic, (0.0, {}))

    def wait(self, topic, predicate, cancel, timeout=60, after=0.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                check_cancel(cancel)
                if self.closed.is_set():
                    raise RuntimeError('导航连接已中断。')
                received, message = self.messages.get(topic, (0.0, {}))
                if received >= after and predicate(message):
                    return message
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'等待导航状态超时：{topic}')
                self.condition.wait(0.05)

    def _receive(self):
        import websocket
        try:
            while not self.closed.is_set():
                try:
                    raw = self.socket.recv()
                except websocket.WebSocketTimeoutException:
                    continue
                if not raw:
                    break
                data = json.loads(raw)
                with self.condition:
                    if data.get('op') == 'service_response' and data.get('id') in self.pending:
                        self.pending[data['id']] = data
                    elif data.get('op') == 'publish':
                        self.messages[data['topic']] = (time.monotonic(), data.get('msg', {}))
                    self.condition.notify_all()
                self.on_message(data)
        except Exception as error:
            if not self.closed.is_set():
                print('[调试导航] 连接中断：', error)
        finally:
            self.closed.set()
            with self.condition:
                self.condition.notify_all()

    def close(self):
        self.closed.set()
        if self.socket:
            self.socket.close()


class NavigationRuntime:
    """Launch only navigation nodes in a separate ROS domain, never a UDP owner."""
    def __init__(self, workspace, launch_file, port=9091, domain=73):
        self.workspace, self.launch_file = Path(workspace), Path(launch_file)
        self.port, self.domain = port, domain
        self.process = None

    def start(self):
        if self.process and self.process.poll() is None:
            return
        if os.name != 'posix':
            raise RuntimeError('导航需要在机器人小主机的 ROS 2 环境中运行。')
        setup = self.workspace / 'install/setup.bash'
        if not setup.is_file():
            raise RuntimeError(f'找不到导航工作区环境：{setup}')
        # Do not connect to an unrelated rosbridge already occupying this port.
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', self.port))
        command = (
            'source /opt/ros/humble/setup.bash && source ' + shlex.quote(str(setup))
            + ' && exec ros2 launch ' + shlex.quote(str(self.launch_file))
            + ' bridge_port:=' + str(self.port))
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain),
                   RMW_FASTRTPS_USE_SHM='0', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
        self.process = subprocess.Popen(['bash', '-c', command], env=env,
                                        stdin=subprocess.DEVNULL, start_new_session=True)

    def close(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=2)


def map_image(message):
    """Compact occupancy grid for the debug page's coarse initial-pose picker."""
    info = message.get('info', {})
    width, height = int(info.get('width', 0)), int(info.get('height', 0))
    data = message.get('data', [])
    if width < 1 or height < 1 or len(data) != width * height:
        raise RuntimeError('地图栅格尚未接收完整。')
    stride = max(1, math.ceil(max(width, height) / 900))
    pixels = bytearray()
    for y in range(0, height, stride):
        for x in range(0, width, stride):
            value = data[y * width + x]
            pixels.append(110 if value < 0 else max(0, min(255, round(255 * (1 - value / 100)))))
    return {'width': math.ceil(width / stride), 'height': math.ceil(height / stride),
            'resolution': float(info['resolution']) * stride,
            'origin': info['origin'], 'pixels': base64.b64encode(pixels).decode('ascii')}


def saved_map_yaml(workspace=None):
    """Find the fixed map without starting ROS or depending on its Python env."""
    name = MAP_NAME + '.yaml'
    candidates = [Path(prefix) / 'share/robot_navigation2/maps' / name
                  for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
                  if prefix]
    if workspace:
        root = Path(workspace)
        candidates.extend([
            root / 'install/robot_navigation2/share/robot_navigation2/maps' / name,
            root / 'install/share/robot_navigation2/maps' / name,
            root / 'src/ros2_humble_main/src/navigation/robot_navigation2/maps' / name,
        ])
    candidates.append(Path(__file__).resolve().parents[2] / 'navigation/robot_navigation2/maps' / name)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise ValueError(f'未找到 {name}，请确认地图已保存在 robot_navigation2/maps 中。')


def read_saved_map(filename):
    """Read the flat Nav2 map YAML and PGM emitted by this project's map saver.

    Only known map fields and numeric origin sequences are accepted. This is
    deliberately not a general YAML loader: the voice venv does not need PyYAML.
    """
    filename = Path(filename)
    fields, origin_parts = {}, []
    key = ''
    for line in filename.read_text(encoding='utf-8-sig').splitlines():
        match = re.match(r'^([a-z_]+):\s*(.*?)\s*$', line)
        if match:
            key, value = match.groups()
            fields[key] = value
        elif key == 'origin':
            item = re.match(r'^\s*-\s+([^#]+)', line)
            if item:
                origin_parts.append(float(item.group(1)))
    origin = origin_parts or json.loads(fields.get('origin', '[]').split('#', 1)[0])
    resolution = float(fields['resolution'].split('#', 1)[0])
    if (not isinstance(origin, list) or len(origin) != 3 or resolution <= 0 or not math.isfinite(resolution)
            or not all(type(value) in (int, float) and math.isfinite(value) for value in origin)):
        raise ValueError('地图分辨率或原点无效。')
    image_names = shlex.split(fields['image'], comments=True)
    if len(image_names) != 1:
        raise ValueError('地图图像路径无效。')
    raw = (filename.parent / image_names[0]).read_bytes()
    offset = 0

    def token():
        nonlocal offset
        while offset < len(raw):
            if raw[offset] in b' \t\r\n':
                offset += 1
            elif raw[offset] == 35:
                newline = raw.find(b'\n', offset)
                offset = len(raw) if newline < 0 else newline + 1
            else:
                break
        start = offset
        while offset < len(raw) and raw[offset] not in b' \t\r\n#':
            offset += 1
        if start == offset:
            raise ValueError('地图 PGM 文件头不完整。')
        return raw[start:offset]

    magic = token()
    width, height, maximum = int(token()), int(token()), int(token())
    if magic != b'P5' or width <= 0 or height <= 0 or width * height > 64000000 or maximum != 255:
        raise ValueError('选点预览需要8位 P5 PGM 地图，请使用项目保存的地图。')
    if raw[offset:offset + 2] == b'\r\n':
        offset += 2
    elif raw[offset:offset + 1] in (b' ', b'\t', b'\r', b'\n'):
        offset += 1
    else:
        raise ValueError('地图 PGM 文件头分隔符无效。')
    if len(raw) - offset < width * height:
        raise ValueError('地图 PGM 像素不完整。')
    stride = max(1, math.ceil(max(width, height) / 900))
    occupied = float(fields.get('occupied_thresh', '0.65').split('#', 1)[0])
    free = float(fields.get('free_thresh', '0.196').split('#', 1)[0])
    negate = int(fields.get('negate', '0').split('#', 1)[0])
    pixels = bytearray()
    # PGM rows run top down; the existing canvas consumes bottom-up map pixels.
    for y in range(0, height, stride):
        for x in range(0, width, stride):
            gray = raw[offset + (height - 1 - y) * width + x]
            probability = (gray if negate else 255 - gray) / 255.0
            pixels.append(0 if probability > occupied else 255 if probability < free else 110)
    yaw = origin[2]
    return {'width': math.ceil(width / stride), 'height': math.ceil(height / stride),
            'resolution': resolution * stride,
            'origin': {'position': {'x': origin[0], 'y': origin[1], 'z': 0.0},
                       'orientation': {'x': 0.0, 'y': 0.0,
                                       'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}},
            'pixels': base64.b64encode(pixels).decode('ascii')}


class DebugNavigation:
    """Own one cancellable preparation/route task and a generation-bound velocity gate."""
    def __init__(self, controller, runtime, *, anchor_path=None, client_factory=RosbridgeClient,
                 clock=time.monotonic):
        self.controller, self.app, self.runtime = controller, controller.app, runtime
        self.client_factory, self.client = client_factory, None
        self.clock = clock
        self.anchor_path = Path(anchor_path or Path.home() / '.ros/robot_navigation/voice_debug.json')
        self.anchor = None
        if self.anchor_path.is_file():
            try:
                saved = json.loads(self.anchor_path.read_text(encoding='utf-8'))
                if saved.get('map_name') == MAP_NAME:
                    self.anchor = self.validate_anchor(saved)
            except (ValueError, OSError, TypeError):
                pass
        self.ready = False
        self.map_identity = None
        self.state, self.message = 'idle', '点击“开始导航”，加载地图和路线。'
        self.route = None
        self.cancel_event = threading.Event()
        self.generation = 0
        self.worker = None
        self.velocity_id = ''
        self.velocity = (0.0, 0.0, 0.0)
        self.velocity_at = 0.0
        self.sender_generation = None
        self.driving = False
        self.closed = threading.Event()
        self.map_cache = None
        self.map_error = ''
        self.heartbeat_thread = None
        self.velocity_client = None
        self.cancel_thread = None
        try:
            self._set_map(read_saved_map(saved_map_yaml(getattr(runtime, 'workspace', None))))
        except (OSError, ValueError, KeyError) as error:
            self.map_error = '选点地图读取失败：' + str(error)

    def _set_map(self, data):
        self.map_cache = dict(data, revision=uuid.uuid4().hex)
        self.map_error = ''

    @staticmethod
    def validate_anchor(data):
        result = {key: data[key] for key in ('x', 'y', 'yaw')}
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for value in result.values()):
            raise ValueError('区域a的位置和朝向无效。')
        return result

    def snapshot(self):
        busy = bool(self.worker and self.worker.is_alive())
        return {'state': self.state, 'message': self.message, 'ready': self.ready,
                'busy': busy,
                'active': self.driving, 'map_name': MAP_NAME, 'route_name': ROUTE_NAME,
                'navigation_mode': NAVIGATION_MODE,
                'anchor': self.anchor, 'map_available': self.map_cache is not None,
                'map_error': self.map_error,
                'map_revision': self.map_cache.get('revision', '') if self.map_cache else '',
                'can_set_anchor': self.map_cache is not None and not busy and not self.driving}

    def _set(self, generation, state, message):
        with self.controller.lock:
            self._check(generation)
            self.state, self.message = state, message
            self.controller.log(message)

    def _check(self, generation):
        if generation != self.generation or self.closed.is_set():
            raise NavigationCancelled()
        check_cancel(self.cancel_event)

    def start(self, task):
        if self.worker and self.worker.is_alive():
            raise RuntimeError('导航正在处理，请先停止当前任务。')
        if self.cancel_thread and self.cancel_thread.is_alive():
            raise RuntimeError('正在确认旧导航已停止，请稍后再按。')
        if task != 'prepare' and not self.ready:
            raise RuntimeError('请先点击“开始导航”并完成区域a定位。')
        self.generation += 1
        generation = self.generation
        self.cancel_event = threading.Event()
        cancel = self.cancel_event
        self.state, self.message = 'preparing', '正在启动导航…'
        self.app._web_navigation = True
        self.worker = threading.Thread(target=self._run, args=(task, generation, cancel),
                                       name='debug-navigation', daemon=True)
        self.worker.start()

    def save_anchor(self, data):
        if not self.snapshot()['can_set_anchor']:
            raise RuntimeError('请等待地图显示，并先停止当前导航任务。')
        anchor = self.validate_anchor(data)
        # Only this debug configuration is written; routes/maps remain untouched.
        self.anchor_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.anchor_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(anchor, map_name=MAP_NAME)), encoding='utf-8')
        os.replace(temporary, self.anchor_path)
        self.anchor = anchor
        self.ready = False
        self.start('prepare')

    def reset_anchor(self):
        if self.worker and self.worker.is_alive():
            raise RuntimeError('请先停止当前导航任务。')
        self.ready = False
        self.state, self.message = 'awaiting_anchor', '在地图区域a附近按住拖出朝向，再保存。'

    def _run(self, task, generation, cancel):
        try:
            if task == 'prepare':
                self._prepare(generation, cancel)
            else:
                self._route(task, generation, cancel)
        except NavigationCancelled:
            pass
        except Exception as error:
            with self.controller.lock:
                if generation == self.generation:
                    self.driving = False
                    self.app._web_navigation = False
                    self.app.action_sender.stop_motion()
                    self.state, self.message = 'error', str(error)
                    if task == 'prepare':
                        self.ready = False
                    self.controller.log('导航未完成：' + str(error))
            self._cancel_backend()
        finally:
            with self.controller.lock:
                if generation == self.generation:
                    self.app._web_navigation = False
            if self.velocity_client:
                self.velocity_client.close()
                self.velocity_client = None

    def _call(self, name, kind, args, cancel, timeout=15):
        result = self.client.call(name, 'lzp_interface/srv/' + kind, args, cancel, timeout)
        if result.get('success') is False:
            raise RuntimeError(result.get('message') or '导航服务拒绝请求。')
        return result

    def _prepare(self, generation, cancel):
        self.ready = False
        with self.controller.lock:
            self._check(generation)
            sender = self.app.action_sender
            standing = (sender._motion_mode == sender.MODE_STANDING
                        and time.monotonic() - sender._motion_mode_time < 2)
            sender_generation = sender.capture_generation()
            self.controller.posture_busy = not standing
        try:
            if not standing:
                self._set(generation, 'preparing', '先确认站立，再启动二维导航…')
                sender._work_context.generation = sender_generation
                sender._work_context.web_navigation_stand = True
                if not sender.stand_up():
                    raise RuntimeError('站立未确认，未启动导航。')
            with self.controller.lock:
                self._check(generation)
                self.controller._motion_guard()
                sender._send({'msg_type': 3, 'cmd_code': 2, 'para': 0})
        finally:
            if hasattr(sender._work_context, 'web_navigation_stand'):
                del sender._work_context.web_navigation_stand
            if hasattr(sender._work_context, 'generation'):
                del sender._work_context.generation
            with self.controller.lock:
                self.controller.posture_busy = False
        self.runtime.start()
        if self.client is None or self.client.closed.is_set():
            self.client = self.client_factory(f'ws://127.0.0.1:{self.runtime.port}', self._on_message)
            self.client.connect(cancel)
            for topic, kind in (
                ('/nav2_ready', 'std_msgs/msg/Bool'),
                ('/current_map_info', 'lzp_interface/msg/CurrentMap'),
                ('/route_execution_status', 'lzp_interface/msg/RouteExecutionStatus'),
                ('/localization/navigation_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
                ('/scan', 'sensor_msgs/msg/LaserScan'),
                ('/lio_health', 'std_msgs/msg/String'),
                ('/Odometry', 'nav_msgs/msg/Odometry'),
                ('/map', 'nav_msgs/msg/OccupancyGrid'),
            ):
                self.client.subscribe(topic, kind, throttle=200 if topic == '/Odometry' else 0)
            if not self.heartbeat_thread:
                self.heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True,
                                                         name='debug-nav-body-status')
                self.heartbeat_thread.start()
        self._set(generation, 'preparing', '等待二维导航和雷达就绪…')
        self.client.wait('/nav2_ready', lambda data: data.get('data') is True, cancel, 100)
        mode = self._call('/navigation/set_mode', 'SetNavigationMode', {'mode': NAVIGATION_MODE}, cancel)
        if mode.get('mode') != NAVIGATION_MODE:
            raise RuntimeError('未能确认二维导航模式，未加载路线。')
        self._set(generation, 'preparing', f'正在加载地图 {MAP_NAME}…')
        requested_at = self.clock()
        yaml_path = saved_map_yaml(getattr(self.runtime, 'workspace', None))
        loaded = self.client.call('/map_server/load_map', 'nav2_msgs/srv/LoadMap',
                                  {'map_url': str(yaml_path)}, cancel, 75)
        if loaded.get('result') != 0:
            raise RuntimeError(f'二维地图 {MAP_NAME} 加载失败：{loaded.get("result")}')
        with self.controller.lock:
            self._check(generation)
            self.map_identity = {'success': True, 'message': '二维地图已加载',
                                 'map_name': MAP_NAME, 'load_id': uuid.uuid4().hex}
        self.client.publish('/current_map_info', 'lzp_interface/msg/CurrentMap',
                            self.map_identity, latch=True)
        self.client.wait('/current_map_info', lambda data: data.get('success') is True
                         and data.get('map_name') == MAP_NAME and data.get('load_id'),
                         cancel, 60, after=requested_at)
        grid = self.client.wait('/map', lambda data: bool(data.get('data')), cancel, 60,
                                after=requested_at)
        with self.controller.lock:
            self._check(generation)
            self._set_map(map_image(grid))
        self.client.wait('/route_execution_status', lambda data: data.get('map_name') == MAP_NAME
                         and data.get('mode') == 'idle', cancel, 10, after=requested_at)
        listed = self._call('/list_routes', 'ListRoutes', {'map_name': MAP_NAME}, cancel)
        routes = [route for route in listed.get('routes', []) if route.get('name') == ROUTE_NAME]
        if len(routes) != 1:
            raise RuntimeError(f'地图 {MAP_NAME} 中找不到唯一的路线 {ROUTE_NAME}，请先在原导航页保存。')
        self.route = self._call('/get_route', 'GetRoute', {'route_id': routes[0]['id']}, cancel)['route']
        if not self.anchor:
            self._set(generation, 'awaiting_anchor', '地图和路线已加载；请在区域a附近按住拖出朝向，再保存。')
            return
        stamp = self._wait_odometry_stamp(generation, cancel)
        self._set(generation, 'localizing', '正在使用二维地图和 AMCL 校正区域a初始点…')
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 1.0
        covariance[35] = math.radians(45) ** 2
        x, y, yaw = (self.anchor[key] for key in ('x', 'y', 'yaw'))
        request = {'header': {'frame_id': 'map', 'stamp': stamp},
                   'pose': {'pose': {'position': {'x': x, 'y': y, 'z': 0.0},
                            'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2),
                                            'w': math.cos(yaw / 2)}}, 'covariance': covariance}}
        self.client.wait('/scan', lambda data: self._valid_odometry_stamp(data), cancel, 30,
                         after=time.monotonic() - 2)
        requested_at = self.clock()
        self.client.publish('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', request)
        self._confirm_amcl(generation, cancel, requested_at, stamp)
        with self.controller.lock:
            self._check(generation)
            self.ready = True
        self._set(generation, 'ready', '区域a二维定位已确认，可以点击“导航1”或“导航2”。')

    def _confirm_amcl(self, generation, cancel, requested_at, request_stamp, timeout=30):
        deadline = time.monotonic() + timeout
        minimum_stamp = request_stamp['sec'] * 1000000000 + request_stamp['nanosec']
        last_stamp = None
        stable = []
        while time.monotonic() < deadline:
            self._check(generation)
            self.client.call('/request_nomotion_update', 'std_srvs/srv/Empty', {}, cancel, 3)
            try:
                sample = self.client.wait('/localization/navigation_pose', lambda data: self._valid_odometry_stamp(data)
                                          and data['header']['stamp'] != last_stamp
                                          and data['header']['stamp']['sec'] * 1000000000
                                          + data['header']['stamp']['nanosec'] >= minimum_stamp,
                                          cancel, min(5, max(0.1, deadline - time.monotonic())), after=requested_at)
            except RuntimeError as error:
                if self.client.closed.is_set() or '超时' not in str(error):
                    raise
                continue
            last_stamp = sample['header']['stamp']
            if not self._amcl_sample_valid(sample):
                stable.clear()
            else:
                pose = sample['pose']['pose']
                position, orientation = pose['position'], pose['orientation']
                yaw = math.atan2(2 * orientation['w'] * orientation['z'],
                                 1 - 2 * orientation['z'] ** 2)
                current = (self.clock(), position['x'], position['y'], yaw)
                if stable and any(math.hypot(current[1] - item[1], current[2] - item[2]) > 0.15
                                  or abs(math.atan2(math.sin(yaw - item[3]), math.cos(yaw - item[3])))
                                  > math.radians(15) for item in stable):
                    stable.clear()
                stable.append(current)
                if len(stable) >= 5 and current[0] - stable[0][0] >= 1.0:
                    return
            cancel.wait(0.1)
        raise RuntimeError('区域a二维定位尚未收敛，请调整地图上的初始位置和朝向后重试。')

    def _amcl_sample_valid(self, sample):
        try:
            pose = sample['pose']['pose']
            covariance = sample['pose']['covariance']
            x, y = float(pose['position']['x']), float(pose['position']['y'])
            q = pose['orientation']
            quaternion = [float(q[key]) for key in ('x', 'y', 'z', 'w')]
            variances = [float(covariance[index]) for index in (0, 7, 35)]
            return (sample['header']['frame_id'].lstrip('/') == 'map'
                    and all(math.isfinite(value) for value in [x, y] + quaternion + variances)
                    and 0.95 <= sum(value * value for value in quaternion) <= 1.05
                    and all(value >= 0 for value in variances)
                    and variances[0] <= 0.55 ** 2 and variances[1] <= 0.55 ** 2
                    and variances[2] <= math.radians(30) ** 2
                    and math.hypot(x - self.anchor['x'], y - self.anchor['y']) <= 2.0)
        except (KeyError, TypeError, ValueError, IndexError):
            return False

    def _wait_odometry_stamp(self, generation, cancel, timeout=60):
        # The LiDAR publisher starts later than rosbridge. Recreate this
        # subscription once it is online so rosbridge can match its live QoS.
        self.client.send({'op': 'unsubscribe', 'id': '/Odometry', 'topic': '/Odometry'})
        self.client.subscribe('/Odometry', 'nav_msgs/msg/Odometry', throttle=100)
        deadline = time.monotonic() + timeout
        last_note = ''
        while True:
            self._check(generation)
            try:
                health = json.loads(self.client.latest('/lio_health')[1].get('data') or '{}')
            except (TypeError, ValueError):
                health = {}
            reason = str(health.get('reason') or health.get('code') or '')
            if health.get('state') == 'fault':
                raise RuntimeError('雷达定位故障，未启动导航：' + reason
                                   + '。请查看终端 small_point_lio 的首条故障信息。')
            note = ('正在等待雷达定位初始化，请保持机械狗静止。'
                    if health.get('state') in ('initializing', 'resetting', 'recovering')
                    else '正在等待实时里程计和激光数据；地图已显示，导航尚未就绪。')
            if reason:
                note += ' 定位状态：' + reason
            if note != last_note:
                self._set(generation, 'waiting_odometry', note)
                last_note = note
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError('未收到实时里程计 /Odometry，导航未启动。'
                                   '请检查 MID360 驱动、IMU 和雷达网口；若终端同时提示 /scan 未更新，'
                                   '需先恢复雷达/LIO数据。' + (' 定位状态：' + reason if reason else ''))
            try:
                odometry = self.client.wait('/Odometry', self._valid_odometry_stamp, cancel,
                                            min(2, remaining), after=time.monotonic() - 2)
                stamp = odometry['header']['stamp']
                # Same TF margin as the terrain navigation page: avoid asking
                # AMCL for a transform newer than its latest received TF.
                nanoseconds = int(stamp['sec']) * 1000000000 + int(stamp['nanosec'])
                nanoseconds = max(1, nanoseconds - 200000000)
                sec, nanosec = divmod(nanoseconds, 1000000000)
                return {'sec': sec, 'nanosec': nanosec}
            except RuntimeError:
                if self.client.closed.is_set():
                    raise
                cancel.wait(0.05)

    @staticmethod
    def _valid_odometry_stamp(data):
        try:
            stamp = data['header']['stamp']
            sec, nanosec = stamp['sec'], stamp['nanosec']
            return (type(sec) is int and type(nanosec) is int and sec >= 0
                    and 0 <= nanosec < 1000000000 and (sec > 0 or nanosec > 0))
        except (KeyError, TypeError):
            return False

    def _route(self, task, generation, cancel):
        first, count = 2, 9
        phrase = 'first' if task == 'route1' else 'second'
        points = sorted(self.route.get('points', []), key=lambda point: point['sequence'])
        selected = points[first:first + count]
        if len(selected) != count:
            raise RuntimeError(f'路线 {ROUTE_NAME} 没有完整的点{first + 1}至点{first + count}。')
        with self.controller.lock:
            self._check(generation)
            self.controller._motion_guard()
        mode = self._call('/navigation/set_mode', 'SetNavigationMode', {'mode': NAVIGATION_MODE}, cancel)
        if mode.get('mode') != NAVIGATION_MODE:
            raise RuntimeError('当前未确认二维导航模式，路线未启动。')
        with self.controller.lock:
            self._check(generation)
            self.app._web_navigation = True
            self.sender_generation = self.app.action_sender.capture_generation()
            self.velocity_id = 'debug-velocity-' + str(generation)
        # rosbridge publish messages do not echo subscription IDs. A separate
        # socket and captured generation prevent late old velocities from rearming.
        self.velocity_client = self.client_factory(
            f'ws://127.0.0.1:{self.runtime.port}',
            lambda data: self._on_velocity(data, generation))
        self.velocity_client.connect(cancel, timeout=5)
        self.velocity_client.subscribe('/cmd_vel_safe', 'geometry_msgs/msg/Twist')
        requested_at = self.clock()
        try:
            self._call('/start_route', 'StartRoute', {
                'route_id': self.route['id'], 'loop_count': 1, 'continuous': False,
                'reverse': False, 'hold_at_each_point': True, 'owner_token': '',
                'start_point_id': selected[0]['id'], 'point_limit': count,
                'navigation_mode': NAVIGATION_MODE, 'preview_id': '',
                'authorization_generation': 0, 'request_deadline_unix_ns': 0}, cancel)
        finally:
            if cancel.is_set():
                self._cancel_backend()
        with self.controller.lock:
            self._check(generation)
            self.driving = True
            self.velocity_at = 0.0
        self._set(generation, 'navigating', f'正在前往点{first + 1}；讲解等待当前语音播放结束后发送。')
        phrase_sent = False
        released = set()
        reached_at = {}
        saw_route = False
        deadline = self.clock() + 1800
        while self.clock() < deadline:
            check_cancel(cancel)
            if not phrase_sent and not self.app.web_voice_busy():
                # The microphone gate does not participate in TEXT input.
                self.app.web_text(self.controller.phrases[phrase])
                phrase_sent = True
                with self.controller.lock:
                    self._check(generation)
                    self.controller.log('导航已发送：' + self.controller.phrases[phrase])
            received, status = self.client.latest('/route_execution_status')
            if received < requested_at:
                if self.clock() - requested_at > 5 or self.client.closed.is_set():
                    raise RuntimeError('未收到当前路线的执行状态，已停止移动。')
                cancel.wait(0.05)
                continue
            if self.clock() - received > 2 or self.client.closed.is_set():
                raise RuntimeError('导航状态连接超时，已停止移动。')
            if status.get('mode') == 'route' and status.get('route_id') != self.route['id']:
                raise RuntimeError('导航路线已被替换。')
            if status.get('mode') == 'route':
                saw_route = True
            if not saw_route:
                if self.clock() - requested_at > 5:
                    raise RuntimeError('未收到当前路线的执行确认。')
                cancel.wait(0.05)
                continue
            if status.get('state') == 'awaiting_release':
                point_id = status.get('current_point', {}).get('id')
                expected = [point['id'] for point in selected]
                if point_id not in expected:
                    raise RuntimeError('到达点与所选路线不一致。')
                now = self.clock()
                if point_id not in reached_at:
                    reached_at[point_id] = now
                    index = first + expected.index(point_id) + 1
                    self._set(generation, 'navigating', f'已到达点{index}。')
                if point_id not in released:
                    self._call('/control_route', 'ControlRoute', {'command': 'release',
                               'owner_token': '', 'authorization_generation': 0}, cancel)
                    released.add(point_id)
                    if len(released) < count:
                        self._set(generation, 'navigating', '正在前往下一个点…')
            elif status.get('mode') == 'idle':
                if len(released) != count or len(reached_at) != count:
                    raise RuntimeError(status.get('message') or '导航提前结束，未到达全部目标点。')
                with self.controller.lock:
                    self._check(generation)
                    self.driving = False
                    self.app._web_navigation = False
                    self.app.action_sender.stop_motion()
                # Preserve a pending narration even when the route finishes quickly.
                while not phrase_sent:
                    check_cancel(cancel)
                    if not self.app.web_voice_busy():
                        self.app.web_text(self.controller.phrases[phrase])
                        phrase_sent = True
                    else:
                        cancel.wait(0.1)
                self._set(generation, 'completed', f'已到达点{first + count}，保持原地。')
                return
            cancel.wait(0.05)
        raise RuntimeError('导航执行超时，已停止。')

    def _on_message(self, data):
        if data.get('op') == 'call_service':
            mode = data.get('args', {}).get('mode')
            with self.controller.lock:
                sender = self.app.action_sender
                ok = (mode in ('normal', 'rough_ground', 'obstacle', 'ramp')
                      and (self.driving or mode == 'normal')
                      and not self.controller.posture_busy and sender._can_send_motion()
                      and sender._motion_mode == sender.MODE_STANDING
                      and time.monotonic() - sender._motion_mode_time < 2)
                if ok:
                    # Keep this debug session in the explicitly requested obstacle gait.
                    if self.driving:
                        sender._send_velocity(0.0, 0.0, 0.0)
                        self.velocity_at = 0.0
                    sender._send({'msg_type': 3, 'cmd_code': 2, 'para': 0})
            self.client.send({'op': 'service_response', 'id': data['id'],
                              'service': data['service'], 'result': True,
                              'values': {'success': bool(ok), 'message': '越障模式' if ok else '当前不可切换地形模式'}})

    def _on_velocity(self, data, generation):
        if data.get('topic') == '/cmd_vel_safe':
            message = data.get('msg', {})
            try:
                velocity = (float(message['linear']['x']), float(message['linear']['y']),
                            float(message['angular']['z']))
                if not all(math.isfinite(value) for value in velocity):
                    return
            except (KeyError, TypeError, ValueError):
                return
            with self.controller.lock:
                if self.driving and generation == self.generation:
                    self.velocity = (max(-0.5, min(0.5, velocity[0])),
                                     max(-0.2, min(0.2, velocity[1])),
                                     max(-0.6, min(0.6, velocity[2])))
                    self.velocity_at = time.monotonic()

    def _heartbeat(self):
        config_file = Path.home() / '.ros/robot_navigation/robot_body.json'
        try:
            if config_file.is_file():
                config = json.loads(config_file.read_text(encoding='utf-8'))
            else:
                config = {'selected': 'xingji', 'profiles': {'xingji': {
                    'navigation_verified': True, 'max_vx': 0.5, 'max_vy': 0.2, 'max_yaw': 0.6}}}
        except (ValueError, OSError):
            config = {}
        while not self.closed.wait(0.2):
            if not self.client or self.client.closed.is_set():
                continue
            try:
                sender = self.app.action_sender
                online = time.monotonic() - sender._motion_mode_time < 2
                fault = bool(getattr(sender, '_hardware_fault_latched', False)
                             or getattr(sender, '_hardware_error', 0))
                status = {'config': config, 'active_body': 'xingji', 'generation': 1,
                          'connected': online, 'switching': False, 'error': '',
                          'status': {'stale': not online, 'hardware_error': fault,
                                     'emergency_latched': fault,
                                     'motion_mode': sender._motion_mode,
                                     'standing': online and sender._motion_mode == sender.MODE_STANDING}}
                self.client.publish('/robot_body/status', 'std_msgs/msg/String',
                                    {'data': json.dumps(status)}, latch=True)
                if self.map_identity:
                    self.client.publish('/current_map_info', 'lzp_interface/msg/CurrentMap',
                                        self.map_identity, latch=True)
            except Exception:
                continue

    def tick(self):
        """Called under controller.lock; no network/cloud waits in this path."""
        if not self.driving:
            return
        try:
            self.controller._motion_guard()
            if self.app.action_sender.capture_generation() != self.sender_generation:
                raise RuntimeError('机器人动作已被其他操作停止，导航已取消。')
            if self.client.closed.is_set():
                raise RuntimeError('导航连接中断，已停止移动。')
            try:
                health = json.loads(self.client.latest('/lio_health')[1].get('data') or '{}')
            except (TypeError, ValueError):
                health = {}
            if isinstance(health, dict) and health.get('state') in ('fault', 'resetting', 'recovering'):
                raise RuntimeError('雷达里程计异常，二维导航已停止，请重新确认区域a定位。')
            if self.velocity_client is None or self.velocity_client.closed.is_set():
                raise RuntimeError('导航速度连接中断，已停止移动。')
            velocity = self.velocity if time.monotonic() - self.velocity_at < 0.3 else (0, 0, 0)
            self.app.action_sender._send_velocity(*velocity)
        except Exception as error:
            self.cancel(str(error))
            self.app.action_sender.stop_motion()

    def cancel(self, reason='已停止导航，可手动遥控。'):
        was_busy = self.driving or (self.worker and self.worker.is_alive())
        self.cancel_event.set()
        self.generation += 1
        self.driving = False
        self.app._web_navigation = False
        self.velocity_at = 0.0
        if was_busy:
            self.state, self.message = 'interrupted', reason
            self.controller.log(reason)
            self.cancel_thread = threading.Thread(target=self._cancel_backend, daemon=True,
                                                  name='debug-nav-cancel')
            self.cancel_thread.start()

    def _cancel_backend(self):
        try:
            if self.client and not self.client.closed.is_set():
                self.client.publish('/cancel_navigation', 'std_msgs/msg/Empty', {})
                self.client.publish('/relocalization/cancel', 'std_msgs/msg/Empty', {})
        except Exception:
            pass

    def close(self):
        with self.controller.lock:
            self.cancel()
            self.closed.set()
        if self.client:
            self.client.close()
        if self.worker:
            self.worker.join(timeout=2)
        self.runtime.close()
