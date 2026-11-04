#!/usr/bin/env python3
"""Pure UDP/JSON bridge for Cyber1 status and stand-alone remote control.

This module intentionally uses only Python's standard networking/JSON modules
and ROS 2 messages.  It does not import or link the vendor SDK.
"""

import ipaddress
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from lzp_interface.srv import SetTraversalMode
from car_vel.camera_stream_lease import CameraStreamLease


MSG_QUERY_REQUEST = 1
MSG_COMMAND_REQUEST = 3

QUERY_REALTIME_STATUS = 1

CONNECTION_PING = b'PING'
CONNECTION_PONG = b'PONG'

CMD_SET_MOTION_MODEL = 2
CMD_PERFORM_ACTION = 3
CMD_RESET = 4
CMD_OBSTACLE_STOP = 5
CMD_EMERGENCY = 6
CMD_CAMERA_STREAM = 9
CMD_STAND = 10
CMD_LIE_DOWN = 11

ACTION_IDLE_MODE = 18
MODE_STANDING = 2
CUSTOM_ACTION_STEP_SEC = 0.10
CUSTOM_ACTION_SETTLE_SEC = 0.50
IDLE_MODE_SWITCH_SEC = 0.30

ZERO_VELOCITY = {
    'vel_move_frontback': 0.0,
    'vel_move_leftright': 0.0,
    'vel_turn_leftright': 0.0,
}

POSTURE_LABELS = {
    0: '趴下（未复位）',
    1: '趴下',
    2: '站立',
    3: '低姿态',
    4: '翻倒',
}

OFFICIAL_ACTIONS = {
    2: '向前跳',
    5: '撒尿',
    11: '打招呼',
    13: '扭身体',
    14: '比心',
    15: '拜年',
}

CUSTOM_ACTIONS = {
    1: '点头',
    2: '摇头',
    3: '向左探头 5 秒',
    4: '向右探头 5 秒',
    5: '来回侧身',
    6: '向上抬头 5 秒',
    7: '向下低头 5 秒',
    8: '向左歪头 2 秒',
}


def clamp_velocity(value):
    """Return a finite protocol velocity in the supported [-1, 1] range."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if number != number or number in (float('inf'), float('-inf')):
        return 0.0
    if abs(number) < 0.01:
        return 0.0
    return max(-1.0, min(1.0, number))


def command_payload(command_code, parameter=None):
    """Build the command shape used by the robot's UDP JSON protocol."""
    payload = {
        'msg_type': MSG_COMMAND_REQUEST,
        'cmd_code': int(command_code),
    }
    if parameter is not None:
        payload['para'] = int(parameter)
    return payload


class CameraProxyHandler(BaseHTTPRequestHandler):
    """Pass the robot's HTTP JPEG/MJPEG stream through the ROS host."""

    upstream_url = 'http://192.168.96.2:8080'
    error_callback = None
    session_callback = None
    protocol_version = 'HTTP/1.0'

    def _session_reply(self, status, value):
        body = json.dumps(value, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        """Allow only the local gateway to acquire a camera-only bounded lease."""
        if urlsplit(self.path).path != '/camera/session':
            self.send_error(404); return
        if not ipaddress.ip_address(self.client_address[0]).is_loopback:
            self.send_error(403); return
        try:
            if self.headers.get_content_type() != 'application/json':
                raise ValueError('图传请求必须使用 JSON')
            length = int(self.headers.get('Content-Length','0'))
            if not 0 < length <= 4096: raise ValueError('图传请求长度无效')
            payload = json.loads(self.rfile.read(length))
            callback = type(self).session_callback
            if callback is None: raise RuntimeError('本体图传控制不可用')
            result = callback(payload)
            self._session_reply(200, result)
        except (ValueError, TypeError) as error:
            self._session_reply(400, dict(success=False, error=str(error)))
        except RuntimeError as error:
            self._session_reply(503, dict(success=False, error=str(error)))
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def do_GET(self):
        """Serve a health check or proxy the camera byte stream."""
        path = urlsplit(self.path).path
        if path == '/health':
            body = b'{"ok":true}'
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return
        if path != '/camera':
            self.send_error(404)
            return

        request = Request(
            type(self).upstream_url,
            headers={
                'Accept': 'multipart/x-mixed-replace,image/jpeg,image/*,*/*',
                'Cache-Control': 'no-cache',
                'User-Agent': 'quadruped-camera-proxy/1.0',
            },
        )
        response_started = False
        try:
            with urlopen(request, timeout=5.0) as upstream:
                self.send_response(200)
                self.send_header(
                    'Content-Type',
                    upstream.headers.get('Content-Type', 'image/jpeg'),
                )
                content_length = upstream.headers.get('Content-Length')
                if content_length:
                    self.send_header('Content-Length', content_length)
                self.send_header('Cache-Control', 'no-store, no-cache')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                response_started = True

                read_chunk = getattr(upstream, 'read1', upstream.read)
                while True:
                    chunk = read_chunk(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        except (OSError, URLError) as error:
            callback = type(self).error_callback
            if callback is not None:
                callback(str(error))
            if not response_started and not self.wfile.closed:
                try:
                    self.send_error(502, 'Camera upstream unavailable')
                except (BrokenPipeError, ConnectionResetError):
                    pass

    def log_message(self, _format, *_args):
        """Keep per-frame proxy traffic out of the ROS launch log."""


class DogRemoteBridge(Node):
    """Publish robot status and guard remote commands with a short lease."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('dog_remote_bridge', parameter_overrides=parameter_overrides)

        self.declare_parameter('dog_ip', '192.168.96.2')
        self.declare_parameter('dog_port', 8080)
        # The robot associates one control/status session with the client's
        # ephemeral UDP source port.  This mirrors the vendor client: binding
        # the client to UDP 8080 prevents this firmware from accepting the
        # session or returning status.
        self.declare_parameter('local_udp_port', 0)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('query_rate_hz', 1.0)
        self.declare_parameter('connection_keepalive_sec', 30.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('session_timeout_sec', 0.45)
        self.declare_parameter('status_timeout_sec', 3.0)
        self.declare_parameter('stand_confirmation_timeout_sec', 8.0)
        self.declare_parameter('navigation_gait_confirmation_timeout_sec', 2.0)
        self.declare_parameter('high_speed_model_value', 1)
        self.declare_parameter('obstacle_model_value', 0)
        self.declare_parameter('camera_proxy_bind', '0.0.0.0')
        self.declare_parameter('camera_proxy_port', 8081)
        self.declare_parameter(
            'camera_upstream_url', 'http://192.168.96.2:8080')

        dog_ip = str(self.get_parameter('dog_ip').value)
        dog_port = int(self.get_parameter('dog_port').value)
        local_udp_port = max(
            0, int(self.get_parameter('local_udp_port').value))
        send_rate = max(5.0, float(self.get_parameter('send_rate_hz').value))
        query_rate = max(0.2, float(self.get_parameter('query_rate_hz').value))
        self.connection_keepalive = max(
            10.0,
            float(self.get_parameter('connection_keepalive_sec').value),
        )
        self.command_timeout = max(
            0.10, float(self.get_parameter('command_timeout_sec').value))
        self.session_timeout = max(
            0.40, float(self.get_parameter('session_timeout_sec').value))
        self.status_timeout = max(
            1.0, float(self.get_parameter('status_timeout_sec').value))
        self.stand_confirmation_timeout = max(
            1.0,
            float(self.get_parameter(
                'stand_confirmation_timeout_sec').value),
        )
        self.high_speed_model = int(
            self.get_parameter('high_speed_model_value').value)
        self.navigation_gait_confirmation_timeout = max(0.1, min(3.0, float(
            self.get_parameter('navigation_gait_confirmation_timeout_sec').value)))
        self.obstacle_model = int(
            self.get_parameter('obstacle_model_value').value)
        resolved_dog_ip = socket.gethostbyname(dog_ip)
        self.dog_address = (resolved_dog_ip, dog_port)
        self.dog_network = ipaddress.ip_network(
            f'{resolved_dog_ip}/24', strict=False)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(('0.0.0.0', local_udp_port))
        except OSError as error:
            self.get_logger().warn(
                f'Cannot bind UDP port {local_udp_port}: {error}; '
                'falling back to an ephemeral port.')
            self.socket.bind(('0.0.0.0', 0))
        self.local_udp_port = int(self.socket.getsockname()[1])
        self.socket.settimeout(0.20)
        self.socket_closed = False
        self.send_lock = threading.Lock()
        self.receive_buffer = ''
        self.receive_buffer_lock = threading.Lock()
        self.queries_sent = 0
        self.connection_pings_sent = 0
        self.udp_reconnect_count = 0
        self.datagrams_received = 0
        self.parsed_packets = 0
        self.non_json_datagrams = 0
        self.non_status_packets = 0

        self.state_lock = threading.RLock()
        self.remote_active = False
        self.emergency_latched = False
        # Compatibility gate: emergency_latched means any latched motion stop.
        # Only an explicit remote emergency command sets the hardware flag.
        self.hardware_emergency_latched = False
        self.last_heartbeat = 0.0
        self.last_velocity_command = 0.0
        self.requested_velocity = (0.0, 0.0, 0.0)
        self.action_velocity = None
        self.last_status_time = 0.0
        self.last_model_status_time = 0.0
        self.last_packet_time = 0.0
        self.last_connection_ping_time = 0.0
        self.last_pong_time = 0.0
        self.connection_ready = False
        self.robot_status = {}
        self.current_model = None
        self.obstacle_stop_enabled = True
        self.navigation_collision_configured = False
        # Match keyboard_test.cpp's two local toggle variables exactly.
        self.keyboard_high_speed_mode = True
        self.keyboard_obstacle_open = True
        self.idle_mode_enabled = False
        self.active_action = ''
        self.camera_stream_enabled = False
        self._camera_stream_lease = CameraStreamLease(self._set_camera_stream)
        self.action_cancel = threading.Event()
        self.action_thread = None
        self.camera_server = None
        self.camera_thread = None
        self.camera_error_last_logged = 0.0

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_publisher = self.create_publisher(
            String, '/dog_status', latched)
        self.event_publisher = self.create_publisher(
            String, '/dog_remote/events', reliable)
        self.create_subscription(
            Twist, '/dog_remote/cmd_vel', self._velocity_callback, reliable)
        self.create_subscription(
            String, '/dog_remote/command', self._command_callback, reliable)
        self.create_subscription(
            Bool, '/dog_remote/enable', self._enable_callback, reliable)
        self.create_subscription(
            Empty, '/dog_remote/heartbeat', self._heartbeat_callback, reliable)
        self.create_service(
            Trigger,
            '/dog_remote/ensure_standing',
            self._ensure_standing_callback,
        )
        self.create_service(
            SetTraversalMode,
            '/dog_remote/set_traversal_mode',
            self._set_traversal_mode_callback,
        )

        self.control_timer = self.create_timer(
            1.0 / send_rate, self._control_tick)
        self.query_timer = self.create_timer(
            1.0 / query_rate, self._query_status)
        self.status_timer = self.create_timer(0.25, self._publish_status)
        self.camera_lease_timer = self.create_timer(0.5, self._camera_stream_lease.tick)

        self.receive_stop = threading.Event()
        self.receive_thread = threading.Thread(
            target=self._receive_loop,
            name='dog_udp_receive',
            daemon=True,
        )
        self.receive_thread.start()

        self._start_camera_proxy()
        self.get_logger().info(
            f'Pure UDP/JSON dog bridge ready for {dog_ip}:{dog_port}; '
            f'listening for replies on UDP {self.local_udp_port}; '
            'no vendor SDK is loaded.')
        self._send_connection_ping()

    def _send_datagram(self, encoded):
        if self.socket_closed:
            return False
        try:
            with self.send_lock:
                self.socket.sendto(encoded, self.dog_address)
            return True
        except OSError as error:
            if not self.socket_closed:
                self.get_logger().error(f'Dog UDP send failed: {error}')
            return False

    def _send_json(self, payload):
        encoded = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        return self._send_datagram(encoded)

    def _send_connection_ping(self):
        if self._send_datagram(CONNECTION_PING):
            now = time.monotonic()
            with self.state_lock:
                self.connection_pings_sent += 1
                self.last_connection_ping_time = now
                pings_sent = self.connection_pings_sent
                handshake_ready = self.connection_ready
            if not handshake_ready and pings_sent in (5, 30):
                self.get_logger().warn(
                    'UDP handshake is not ready; '
                    f'{pings_sent} total PING packets sent since startup '
                    f'from local port {self.local_udp_port}. '
                    'This is a cumulative counter, not consecutive failed attempts.')

    def _send_command(self, code, parameter=None):
        return self._send_json(command_payload(code, parameter))

    def _set_camera_stream(self, enabled):
        """Only command 9; never activates remote control or robot movement."""
        sent = self._send_command(CMD_CAMERA_STREAM, int(enabled))
        if sent:
            with self.state_lock:
                self.camera_stream_enabled = bool(enabled)
        return sent

    def _send_velocity(self, velocity):
        vx, vy, yaw = velocity
        return self._send_json({
            'vel_move_frontback': clamp_velocity(vx),
            'vel_move_leftright': clamp_velocity(vy),
            'vel_turn_leftright': clamp_velocity(yaw),
        })

    def _send_zero_burst(self):
        for _ in range(3):
            self._send_json(dict(ZERO_VELOCITY))
            time.sleep(0.015)

    def _query_status(self):
        now = time.monotonic()
        with self.state_lock:
            connection_ready = self.connection_ready
            last_status_time = self.last_status_time
            last_packet_time = self.last_packet_time
            last_ping_time = self.last_connection_ping_time
        if not connection_ready:
            self._send_connection_ping()
            return

        status_reference = max(last_status_time, last_packet_time)
        reconnect_after = max(1.0, self.status_timeout - 0.5)
        if status_reference and now - status_reference > reconnect_after:
            with self.state_lock:
                if self.connection_ready:
                    self.connection_ready = False
                    self.udp_reconnect_count += 1
                    reconnect_count = self.udp_reconnect_count
                else:
                    reconnect_count = self.udp_reconnect_count
            self.get_logger().warn(
                'Dog status response timed out; restarting the UDP '
                f'PING/PONG handshake (reconnect {reconnect_count}).')
            self._send_connection_ping()
            return

        if now - last_ping_time >= self.connection_keepalive:
            self._send_connection_ping()
            return

        if self._send_json({
            'msg_type': MSG_QUERY_REQUEST,
            'query_code': QUERY_REALTIME_STATUS,
        }):
            with self.state_lock:
                self.queries_sent += 1
                queries_sent = self.queries_sent
                status_received = self.last_status_time > 0
            if not status_received and queries_sent in (5, 30):
                self.get_logger().warn(
                    'No dog status response after '
                    f'{queries_sent} queries from local UDP port '
                    f'{self.local_udp_port}.')

    def _start_camera_proxy(self):
        bind_address = str(
            self.get_parameter('camera_proxy_bind').value).strip()
        proxy_port = int(self.get_parameter('camera_proxy_port').value)
        upstream_url = str(
            self.get_parameter('camera_upstream_url').value).strip()

        handler = type(
            'BoundCameraProxyHandler',
            (CameraProxyHandler,),
            {
                'upstream_url': upstream_url,
                'error_callback': staticmethod(self._camera_proxy_error),
                'session_callback': staticmethod(self._camera_stream_lease.request),
            },
        )
        try:
            self.camera_server = ThreadingHTTPServer(
                (bind_address, proxy_port), handler)
        except OSError as error:
            self.get_logger().error(
                f'Camera proxy could not listen on {bind_address}:'
                f'{proxy_port}: {error}')
            return
        self.camera_server.daemon_threads = True
        self.camera_thread = threading.Thread(
            target=self.camera_server.serve_forever,
            name='dog_camera_proxy',
            daemon=True,
        )
        self.camera_thread.start()
        self.get_logger().info(
            f'Camera proxy ready on http://{bind_address}:{proxy_port}/camera '
            f'-> {upstream_url}')

    def _camera_proxy_error(self, message):
        now = time.monotonic()
        if now - self.camera_error_last_logged < 10.0:
            return
        self.camera_error_last_logged = now
        self.get_logger().warn(f'Camera proxy upstream failed: {message}')

    def _receive_loop(self):
        decoder = json.JSONDecoder()
        while not self.receive_stop.is_set():
            try:
                datagram, address = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if not self.socket_closed:
                    self.get_logger().warn('Dog UDP receive socket closed unexpectedly.')
                break
            if ipaddress.ip_address(address[0]) not in self.dog_network:
                continue
            with self.state_lock:
                self.datagrams_received += 1

            text = datagram.decode('utf-8', errors='ignore').strip('\x00\r\n ')
            if not text:
                continue
            if text == CONNECTION_PONG.decode('ascii'):
                now = time.monotonic()
                with self.state_lock:
                    first_pong = not self.connection_ready
                    self.connection_ready = True
                    self.last_packet_time = now
                    self.last_pong_time = now
                if first_pong:
                    self.get_logger().info(
                        'Dog UDP handshake completed: PING/PONG accepted on '
                        f'local port {self.local_udp_port}.')
                self._query_status()
                continue
            with self.receive_buffer_lock:
                self.receive_buffer += text
                if len(self.receive_buffer) > 131072:
                    self.receive_buffer = self.receive_buffer[-65536:]

                while self.receive_buffer:
                    start = self.receive_buffer.find('{')
                    if start < 0:
                        with self.state_lock:
                            self.non_json_datagrams += 1
                            non_json_count = self.non_json_datagrams
                        if non_json_count <= 3:
                            preview = self.receive_buffer[:120]
                            self.get_logger().info(
                                'Received non-JSON dog UDP data: '
                                f'{preview!r}')
                        self.receive_buffer = ''
                        break
                    if start:
                        self.receive_buffer = self.receive_buffer[start:]
                    try:
                        packet, end = decoder.raw_decode(self.receive_buffer)
                    except json.JSONDecodeError:
                        break
                    self.receive_buffer = self.receive_buffer[end:].lstrip()
                    if isinstance(packet, dict):
                        self._handle_packet(packet)

    @staticmethod
    def _find_status_object(value):
        if isinstance(value, str):
            stripped = value.strip()
            if stripped.startswith(('{', '[')):
                try:
                    return DogRemoteBridge._find_status_object(
                        json.loads(stripped))
                except json.JSONDecodeError:
                    return None
            return None
        if isinstance(value, list):
            for child in value:
                match = DogRemoteBridge._find_status_object(child)
                if match is not None:
                    return match
            return None
        if not isinstance(value, dict):
            return None
        status_keys = {
            'battery', 'hardware_error', 'motion_mode', 'velocity_level',
            'model', 'hw_failure',
        }
        if status_keys.intersection(value):
            return value
        for child in value.values():
            match = DogRemoteBridge._find_status_object(child)
            if match is not None:
                return match
        return None

    @staticmethod
    def _normalise_failures(value):
        if value is None:
            return []
        if isinstance(value, list):
            failures = value
        elif isinstance(value, dict):
            failures = [
                {'part': key, 'value': item}
                for key, item in value.items()
                if item not in (0, False, None, '', [])
            ]
        elif value in (0, False, '', '0'):
            return []
        else:
            failures = [value]
        return failures

    def _handle_packet(self, packet):
        now = time.monotonic()
        with self.state_lock:
            self.last_packet_time = now
            self.parsed_packets += 1

        status = self._find_status_object(packet)
        if status is None:
            with self.state_lock:
                self.non_status_packets += 1
                non_status_count = self.non_status_packets
            if non_status_count <= 3:
                keys = ', '.join(sorted(str(key) for key in packet.keys()))
                self.get_logger().info(
                    'Received dog UDP JSON without realtime status fields '
                    f'(keys: {keys or "none"}; '
                    f'msg_type={packet.get("msg_type")!r}; '
                    f'query_code={packet.get("query_code")!r}).')
            return

        update = dict(status)
        if 'battery' in update:
            try:
                update['battery'] = max(0.0, min(100.0, float(update['battery'])))
            except (TypeError, ValueError):
                update.pop('battery', None)
        if 'motion_mode' in update:
            try:
                update['motion_mode'] = int(update['motion_mode'])
            except (TypeError, ValueError):
                update.pop('motion_mode', None)
        if 'model' in update:
            try:
                update['model'] = int(update['model'])
            except (TypeError, ValueError):
                update.pop('model', None)

        with self.state_lock:
            self.robot_status.update(update)
            self.last_status_time = now
            self.connection_ready = True
            if 'model' in update:
                self.current_model = update['model']
                self.last_model_status_time = now

    def _enable_callback(self, message):
        now = time.monotonic()
        if message.data:
            with self.state_lock:
                was_active = self.remote_active
                self.remote_active = True
                self.last_heartbeat = now
                self.requested_velocity = (0.0, 0.0, 0.0)
                self.last_velocity_command = now
            self._send_zero_burst()
            self._camera_stream_lease.remote(True)
            if not was_active:
                self.get_logger().info(
                    'Remote session enabled; requested camera stream '
                    'with command code 9.')
                self._event(
                    'info',
                    '独立遥控会话已启用，相机流启动命令已发送。',
                )
        else:
            self._deactivate_remote('页面已退出遥控模式。')

    def _heartbeat_callback(self, _message):
        with self.state_lock:
            if self.remote_active:
                self.last_heartbeat = time.monotonic()

    def _velocity_callback(self, message):
        with self.state_lock:
            if not self.remote_active or self.emergency_latched:
                return
            if self.active_action:
                return
            self.requested_velocity = (
                clamp_velocity(message.linear.x),
                clamp_velocity(message.linear.y),
                clamp_velocity(message.angular.z),
            )
            self.last_velocity_command = time.monotonic()

    def _command_callback(self, message):
        try:
            request = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self._event('error', '收到无法解析的遥控命令。')
            return
        if not isinstance(request, dict):
            self._event('error', '遥控命令必须是 JSON 对象。')
            return

        command = str(request.get('command', '')).strip().lower()
        if command == 'stop':
            self._stop_motion('已停止全部遥控运动。')
            return
        if command == 'emergency':
            self._emergency_stop()
            return

        with self.state_lock:
            active = self.remote_active
        if not active:
            self._event('error', '遥控会话未启用，命令已拒绝。')
            return

        with self.state_lock:
            stopped = self.emergency_latched
        if stopped and command != 'reset':
            self._event('warning', '停车锁定中；请先确认安全并按 R 复位，不会自动站立或恢复旧指令。')
            return

        handlers = {
            'reset': self._reset_robot,
            'stand': self._stand_robot,
            'lie_down': self._lie_down_robot,
            'toggle_model': self._toggle_model,
            'toggle_obstacle_stop': self._toggle_obstacle_stop,
            'toggle_idle': self._toggle_idle_mode,
        }
        if command in handlers:
            handlers[command]()
            return
        if command == 'custom_action':
            self._start_custom_action(request.get('id'))
            return
        if command == 'official_action':
            self._start_official_action(request.get('code'))
            return
        self._event('error', f'不支持的遥控命令：{command or "空命令"}。')

    def _reset_robot(self):
        self._cancel_action()
        with self.state_lock:
            soft_only = self.emergency_latched and not getattr(self, 'hardware_emergency_latched', False)
            self.emergency_latched = False
            self.hardware_emergency_latched = False
            self.requested_velocity = (0.0, 0.0, 0.0)
            self.last_velocity_command = 0.0
        self._send_zero_burst()
        if not soft_only:
            self._send_command(CMD_RESET)
        self._event('info', '停车锁定已由人工解除；未站立、未恢复旧速度。')

    def _stand_robot(self):
        self._cancel_action()
        self._send_zero_burst()
        self._send_command(CMD_STAND)
        self._event('info', '已发送站立命令。')

    def _lie_down_robot(self):
        self._cancel_action()
        self._send_zero_burst()
        self._send_command(CMD_LIE_DOWN)
        self._event('info', '已发送趴下命令。')

    def _toggle_model(self):
        self._cancel_action()
        self._send_zero_burst()
        with self.state_lock:
            self.navigation_collision_configured = False
            if self.keyboard_high_speed_mode:
                target = self.obstacle_model
                self.keyboard_high_speed_mode = False
            else:
                target = self.high_speed_model
                self.keyboard_high_speed_mode = True
            self.current_model = target
        self._send_command(CMD_SET_MOTION_MODEL, target)
        self._event('info', f'已切换为{self._model_label(target)}。')

    def _toggle_obstacle_stop(self):
        self._cancel_action()
        self._send_zero_burst()
        with self.state_lock:
            if not self.keyboard_high_speed_mode:
                self._event('warning', '越障模式下按 O 不执行停障切换。')
                return
            target = not self.keyboard_obstacle_open
            self.navigation_collision_configured = False
            self.keyboard_obstacle_open = target
            self.obstacle_stop_enabled = target
        self._send_command(CMD_OBSTACLE_STOP, 1 if target else 0)
        self._event('info', f'停障开关已{"开启" if target else "关闭"}。')

    def _toggle_idle_mode(self):
        self._cancel_action()
        self._send_zero_burst()
        with self.state_lock:
            self.action_cancel = threading.Event()
            self.active_action = '切换原地模式'
            self.action_thread = threading.Thread(
                target=self._run_idle_toggle,
                args=(self.action_cancel,),
                name='dog_idle_mode_toggle',
                daemon=True,
            )
            thread = self.action_thread
        thread.start()

    def _run_idle_toggle(self, cancel_event):
        try:
            with self.state_lock:
                enabled = self.idle_mode_enabled
            if enabled:
                self._send_command(CMD_PERFORM_ACTION, ACTION_IDLE_MODE)
                if cancel_event.wait(IDLE_MODE_SWITCH_SEC):
                    return
                with self.state_lock:
                    self.idle_mode_enabled = False
                self._event('info', '原地模式已关闭。')
                return

            if not self._ensure_standing(cancel_event):
                return
            self._send_command(CMD_PERFORM_ACTION, ACTION_IDLE_MODE)
            if cancel_event.wait(IDLE_MODE_SWITCH_SEC):
                return
            with self.state_lock:
                self.idle_mode_enabled = True
            self._event('info', '原地模式已开启。')
        finally:
            with self.state_lock:
                self.active_action = ''
                self.action_velocity = None
                self.action_thread = None

    def _start_official_action(self, code_value):
        try:
            code = int(code_value)
        except (TypeError, ValueError):
            self._event('error', '官方动作代码无效。')
            return
        if code not in OFFICIAL_ACTIONS:
            self._event('error', f'不允许的官方动作代码：{code}。')
            return
        self._cancel_action()
        self._send_zero_burst()
        self._send_command(CMD_PERFORM_ACTION, code)
        self._event('info', f'已发送官方动作：{OFFICIAL_ACTIONS[code]}。')

    def _start_custom_action(self, action_value):
        try:
            action_id = int(action_value)
        except (TypeError, ValueError):
            self._event('error', '自定义动作编号无效。')
            return
        if action_id not in CUSTOM_ACTIONS:
            self._event('error', f'不允许的自定义动作编号：{action_id}。')
            return
        with self.state_lock:
            if self.action_thread is not None and self.action_thread.is_alive():
                self._event('warning', '已有动作正在执行；请先按 X 停止。')
                return
            self.action_cancel = threading.Event()
            self.active_action = CUSTOM_ACTIONS[action_id]
            self.requested_velocity = (0.0, 0.0, 0.0)
            self.action_thread = threading.Thread(
                target=self._run_custom_action,
                args=(action_id, self.action_cancel),
                name=f'dog_custom_action_{action_id}',
                daemon=True,
            )
            thread = self.action_thread
        self._event('info', f'开始自定义动作：{CUSTOM_ACTIONS[action_id]}。')
        thread.start()

    def _run_custom_action(self, action_id, cancel_event):
        temporary_idle = False
        completed = False
        try:
            if not self._ensure_standing(cancel_event):
                return
            with self.state_lock:
                already_idle = self.idle_mode_enabled
            if not already_idle:
                self._send_command(CMD_PERFORM_ACTION, ACTION_IDLE_MODE)
                with self.state_lock:
                    self.idle_mode_enabled = True
                temporary_idle = True
                self._event('info', '自定义动作已进入原地模式。')
                if cancel_event.wait(IDLE_MODE_SWITCH_SEC):
                    return

            sequence = self._custom_sequence(action_id)
            for velocity, duration in sequence:
                with self.state_lock:
                    self.action_velocity = velocity
                self._send_velocity(velocity)
                if cancel_event.wait(duration):
                    return
            self._send_velocity((0.0, 0.0, 0.0))
            if cancel_event.wait(CUSTOM_ACTION_SETTLE_SEC):
                return
            completed = True
        finally:
            with self.state_lock:
                self.action_velocity = (0.0, 0.0, 0.0)
            self._send_velocity((0.0, 0.0, 0.0))
            if temporary_idle:
                self._send_command(CMD_PERFORM_ACTION, ACTION_IDLE_MODE)
                cancel_event.wait(IDLE_MODE_SWITCH_SEC)
                with self.state_lock:
                    self.idle_mode_enabled = False
                self._event('info', '自定义动作已退出原地模式。')
            with self.state_lock:
                action_name = self.active_action
                self.active_action = ''
                self.action_velocity = None
                self.action_thread = None
            if completed:
                self._event('success', f'自定义动作已完成：{action_name}。')
            else:
                self._event('warning', f'自定义动作已中止：{action_name}。')

    def _ensure_standing_callback(self, _request, response):
        with self.state_lock:
            emergency = self.emergency_latched
            connected = self.connection_ready
        if emergency:
            response.success = False
            response.message = '机器人处于停车锁定状态，无法自动站立；请先人工确认复位'
            return response
        if not connected:
            self._send_connection_ping()
            response.success = False
            response.message = '机器人通信尚未就绪，稍后自动重试'
            return response

        completed = self._ensure_standing(
            threading.Event(), timeout_sec=self.stand_confirmation_timeout)
        if not completed:
            response.success = False
            response.message = '等待机器人站立确认超时，稍后自动重试'
            return response

        response.success = self._configure_navigation_safety_mode()
        response.message = ('机器人已站立，星玑自主导航固定越障模式并保持碰撞停止开启'
                            if response.success else '越障模式发送失败或反馈未确认，禁止开始导航')
        return response

    def _set_traversal_mode_callback(self, request, response):
        """Select an explicit autonomous traversal mode and confirm safety."""
        requested = str(request.mode).strip().lower() or 'normal'
        supported = {
            'normal', 'ramp', 'stairs_up', 'stairs_down',
            'obstacle', 'rough_ground',
        }
        if requested not in supported:
            response.success = False
            response.message = f'不支持的地形运动模式：{requested}'
            return response
        with self.state_lock:
            emergency = self.emergency_latched
            connected = self.connection_ready
            status_fresh = (
                self.last_status_time > 0.0 and
                time.monotonic() - self.last_status_time <= self.status_timeout)
            hardware_error = self.robot_status.get('hardware_error', 0)
            failures = self._normalise_failures(
                self.robot_status.get('hw_failure'))
        if emergency:
            response.success = False
            response.message = '机器人处于停车锁定状态，请先人工确认复位'
            return response
        if not connected or not status_fresh:
            response.success = False
            response.message = '机器人状态反馈不可用或已过期'
            return response
        if hardware_error not in (0, False, None, '', '0') or failures:
            response.success = False
            response.message = '机器人报告硬件故障，禁止切换地形模式'
            return response
        if not self._ensure_standing(
                threading.Event(), timeout_sec=self.stand_confirmation_timeout):
            response.success = False
            response.message = '等待机器人站立确认超时'
            return response

        # These names describe terrain stages, not Xingji's native model.
        # In particular flat recovery / campus completion sends "normal";
        # it must NEVER select the vendor high-speed gait. Manual remote
        # toggling remains a separate interface and is intentionally unchanged.
        response.success = self._configure_navigation_safety_mode()
        response.message = (f'地形阶段 {requested}；星玑保持越障模式'
                            if response.success else '越障模式发送失败或反馈未确认，禁止开始导航')
        return response

    def _navigation_gait_confirmed(self, *, after=None):
        """Fresh, explicit body feedback, never the optimistic UI model cache."""
        now = time.monotonic()
        with self.state_lock:
            return bool(
                self.connection_ready and not self.emergency_latched
                and self.last_status_time > 0.0
                and 0.0 <= now - self.last_status_time <= self.status_timeout
                and self.last_model_status_time > 0.0
                and 0.0 <= now - self.last_model_status_time <= self.status_timeout
                and (after is None or self.last_model_status_time > after)
                and self.robot_status.get('motion_mode') == MODE_STANDING
                and self.robot_status.get('model') == self.obstacle_model
                and self.robot_status.get('hardware_error', 0) in (0, False, None, '', '0')
                and not self._normalise_failures(self.robot_status.get('hw_failure'))
                and (after is not None or self.navigation_collision_configured)
                and self.obstacle_stop_enabled)

    def _configure_navigation_safety_mode(self):
        """Idempotently select obstacle mode with collision stopping enabled.

        Page entry must set explicit target values rather than invoke the
        keyboard-style toggles, otherwise every refresh could reverse the
        previous state. Autonomous page entry must not weaken the vendor
        collision-stop guard.
        """
        if self._navigation_gait_confirmed():
            return True
        self._cancel_action()
        self._send_zero_burst()
        with self.state_lock:
            self.navigation_collision_configured = False
        sent_at = time.monotonic()
        if not self._send_command(CMD_OBSTACLE_STOP, 1):
            self._event('warning', '星玑碰撞停止配置发送失败，导航保持零速。')
            return False
        with self.state_lock:
            # This command has no separate status field in the vendor protocol.
            # Remember successful dispatch; gait itself still requires feedback.
            self.obstacle_stop_enabled = True
        if not self._send_command(CMD_SET_MOTION_MODEL, self.obstacle_model):
            self._event('warning', '星玑越障模式指令发送失败，导航保持零速。')
            return False
        deadline = sent_at + self.navigation_gait_confirmation_timeout
        while time.monotonic() <= deadline:
            if self._navigation_gait_confirmed(after=sent_at):
                with self.state_lock:
                    self.navigation_collision_configured = True
                    self.keyboard_obstacle_open = True
                    self.obstacle_stop_enabled = True
                    self.keyboard_high_speed_mode = False
                self._event('info', '星玑自主导航：越障模式反馈已确认，保持碰撞停止开启。')
                return True
            with self.state_lock:
                if self.emergency_latched or not self.connection_ready:
                    break
            self._query_status()
            time.sleep(0.05)
        self._event('warning', '星玑越障模式反馈未确认，导航保持零速；请检查本体状态。')
        return False

    def _ensure_standing(self, cancel_event, timeout_sec=None):
        deadline = (
            None if timeout_sec is None
            else time.monotonic() + max(0.1, float(timeout_sec))
        )
        with self.state_lock:
            standing = (
                self.robot_status.get('motion_mode') == MODE_STANDING
                and time.monotonic() - self.last_status_time
                <= self.status_timeout
            )
        if standing:
            return True

        self._send_zero_burst()
        self._send_command(CMD_STAND)
        self._event('info', '机器人未站立，正在先执行站立。')
        while not cancel_event.wait(0.10):
            if deadline is not None and time.monotonic() >= deadline:
                return False
            self._query_status()
            with self.state_lock:
                if (
                    self.robot_status.get('motion_mode') == MODE_STANDING
                    and time.monotonic() - self.last_status_time
                    <= self.status_timeout
                ):
                    return True
        return False

    @staticmethod
    def _custom_sequence(action_id):
        """Return the exact velocity values and timings from keyboard_test.cpp."""
        step = CUSTOM_ACTION_STEP_SEC
        if action_id == 1:
            values = (
                [-1.0] * 3 + [0.0] + [1.0] * 3 + [0.0]
            ) * 3
            return [((value, 0.0, 0.0), step) for value in values]
        if action_id == 2:
            values = (
                [1.0] * 3 + [0.0] + [-1.0] * 3 + [0.0]
            ) * 3
            return [((0.0, 0.0, value), step) for value in values]
        if action_id == 3:
            return [((0.0, 0.0, 1.0), 5.0)]
        if action_id == 4:
            return [((0.0, 0.0, -1.0), 5.0)]
        if action_id == 5:
            values = (
                [1.0] * 3 + [0.0] + [-1.0] * 3 + [0.0]
            ) * 2
            return [((0.0, value, 0.0), step) for value in values]
        if action_id == 6:
            return [((-1.0, 0.0, 0.0), 5.0)]
        if action_id == 7:
            return [((1.0, 0.0, 0.0), 5.0)]
        return [((0.0, -1.0, 0.0), 2.0)]

    def _cancel_action(self):
        with self.state_lock:
            cancel_event = self.action_cancel
            self.action_velocity = (0.0, 0.0, 0.0)
            thread = self.action_thread
        cancel_event.set()
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=0.50)

    def _stop_motion(self, message):
        self._cancel_action()
        with self.state_lock:
            self.requested_velocity = (0.0, 0.0, 0.0)
            self.last_velocity_command = time.monotonic()
        self._send_zero_burst()
        self._event('warning', message)

    def _safety_stop(self):
        """Latched zero velocity; never sends a native posture/disable command."""
        self._cancel_action()
        with self.state_lock:
            self.emergency_latched = True
            self.requested_velocity = (0.0, 0.0, 0.0)
            self.last_velocity_command = 0.0
        self._send_zero_burst()
        self._event('warning', '已请求零速停车并锁定运动；确认安全后人工复位，不会发送本体硬急停。')

    def _emergency_stop(self):
        """Native CMD6 is reserved for the remote button/T command path."""
        self._cancel_action()
        with self.state_lock:
            self.emergency_latched = True
            self.hardware_emergency_latched = True
            self.requested_velocity = (0.0, 0.0, 0.0)
        self._send_zero_burst()
        self._send_command(CMD_EMERGENCY)
        self._event('error', '已发送机械狗急停命令；按 R 复位后才能继续遥控。')

    def _deactivate_remote(self, reason):
        with self.state_lock:
            was_active = self.remote_active
            self.remote_active = False
            self.requested_velocity = (0.0, 0.0, 0.0)
            self.last_velocity_command = 0.0
        self._cancel_action()
        self._camera_stream_lease.remote(False)
        if was_active:
            self._send_zero_burst()
            self._event('warning', reason)

    def _control_tick(self):
        now = time.monotonic()
        with self.state_lock:
            if self.remote_active and now - self.last_heartbeat > self.session_timeout:
                lease_expired = True
            else:
                lease_expired = False
            active = self.remote_active
            emergency = self.emergency_latched
            active_action = self.active_action
            requested = self.requested_velocity
            command_fresh = now - self.last_velocity_command <= self.command_timeout

        if lease_expired:
            self._deactivate_remote('遥控页面心跳超时，机械狗已自动停止。')
            return
        if not active:
            return
        if emergency:
            self._send_velocity((0.0, 0.0, 0.0))
        elif active_action:
            return
        elif command_fresh:
            self._send_velocity(requested)
        else:
            self._send_velocity((0.0, 0.0, 0.0))

    def _model_label(self, value):
        if value == self.high_speed_model:
            return '高速模式'
        if value == self.obstacle_model:
            return '越障模式'
        return f'未知模式（{value}）'

    def _publish_status(self):
        now = time.monotonic()
        with self.state_lock:
            status = dict(self.robot_status)
            last_status = self.last_status_time
            remote_active = self.remote_active
            emergency = self.emergency_latched
            model = self.current_model
            obstacle = self.obstacle_stop_enabled
            idle = self.idle_mode_enabled
            active_action = self.active_action
            camera_stream_enabled = self.camera_stream_enabled
            connection_ready = self.connection_ready

        age = now - last_status if last_status else None
        connected = age is not None and age <= self.status_timeout
        motion_mode = status.get('motion_mode')
        failures = self._normalise_failures(status.get('hw_failure'))
        hardware_value = status.get('hardware_error', 0)
        hardware_value_set = (
            hardware_value not in (0, False, None, '', '0')
            and hardware_value != []
            and hardware_value != {}
        )
        hardware_error = bool(hardware_value_set or failures)

        message = String()
        message.data = json.dumps({
            'connected': connected,
            'stale': not connected,
            'last_seen_age_sec': round(age, 3) if age is not None else None,
            'battery': status.get('battery'),
            'motion_mode': motion_mode,
            'posture': POSTURE_LABELS.get(
                motion_mode,
                f'未知（{motion_mode}）' if motion_mode is not None else '未知',
            ),
            'velocity_level': status.get('velocity_level'),
            'model': model,
            'model_label': self._model_label(model) if model is not None else '未知',
            'navigation_gait_policy': 'obstacle',
            'navigation_gait_confirmed': self._navigation_gait_confirmed(),
            'obstacle_stop_enabled': obstacle,
            'hardware_error': hardware_error,
            'hardware_error_value': hardware_value,
            'hw_failures': failures,
            'remote_active': remote_active,
            'emergency_latched': emergency,
            'hardware_emergency_latched': getattr(self, 'hardware_emergency_latched', False),
            'stop_kind': ('manual_emergency' if getattr(self, 'hardware_emergency_latched', False)
                          else 'zero_speed_latched' if emergency else ''),
            'idle_mode_enabled': idle,
            'active_action': active_action,
            'camera_stream_enabled': camera_stream_enabled,
            'udp_handshake_complete': connection_ready,
            'local_udp_port': self.local_udp_port,
            'connection_pings_sent': self.connection_pings_sent,
            'udp_reconnect_count': self.udp_reconnect_count,
            'queries_sent': self.queries_sent,
            'datagrams_received': self.datagrams_received,
            'parsed_packets': self.parsed_packets,
            'non_json_datagrams': self.non_json_datagrams,
            'non_status_packets': self.non_status_packets,
            'robot_type': status.get('robot_type'),
            'robot_number': status.get('robot_number'),
            'software_version': status.get('software_version'),
            'protocol_version': status.get('protocol_version'),
        }, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(message)

    def _event(self, level, message):
        event = String()
        event.data = json.dumps({
            'level': level,
            'message': message,
            'time': time.time(),
        }, ensure_ascii=False, separators=(',', ':'))
        self.event_publisher.publish(event)

    def close(self):
        """Stop all motion and close the UDP receiver cleanly."""
        if self.socket_closed:
            return
        self._deactivate_remote('遥控桥接节点正在停止。')
        self._camera_stream_lease.close()
        if self.camera_server is not None:
            self.camera_server.shutdown()
            self.camera_server.server_close()
        if self.camera_thread is not None and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        self.receive_stop.set()
        self.socket_closed = True
        try:
            self.socket.close()
        except OSError:
            pass
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)


def main(args=None):
    """Run the pure UDP/JSON bridge as a ROS 2 node."""
    from car_vel.body_lock import acquire_body_lock
    ownership = acquire_body_lock(expected_body='xingji')
    rclpy.init(args=args)
    node = DogRemoteBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        ownership.close()


if __name__ == '__main__':
    main()
