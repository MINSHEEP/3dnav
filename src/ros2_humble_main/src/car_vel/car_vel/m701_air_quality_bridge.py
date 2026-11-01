"""Poll an M701 RS-485 sensor and publish validated, read-only JSON status."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lzp_interface.srv import ConfigureM701
from .m701_connection import ConfigStore, ConnectionError, diagnose, draft, identity, inventory, observation, resolve

from .m701_protocol import (
    build_read_all_request,
    extract_read_all_response,
    parse_read_all_response,
)

try:
    import serial
except ImportError:  # Keep the node alive so the UI shows an actionable error.
    serial = None


DEFAULT_M701_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


class M701AirQualityBridge(Node):
    """Own the serial port and expose only validated M701 telemetry."""

    def __init__(self):
        super().__init__('m701_air_quality_bridge')
        self.declare_parameter('port', DEFAULT_M701_PORT)
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('address', 1)
        self.declare_parameter('poll_interval_sec', 1.0)
        self.declare_parameter('response_timeout_sec', 0.35)
        self.declare_parameter('warmup_sec', 300.0)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.address = int(self.get_parameter('address').value)
        self.poll_interval = max(
            0.5, float(self.get_parameter('poll_interval_sec').value))
        self.response_timeout = max(
            0.05, float(self.get_parameter('response_timeout_sec').value))
        self.warmup_sec = max(
            0.0, float(self.get_parameter('warmup_sec').value))

        # Validate configuration before the first hardware access.
        build_read_all_request(self.address)
        self.publisher = self.create_publisher(String, '/air_quality/m701', 1)
        self._serial = None
        self._first_opened_at = None
        self._failures = 0
        self._warning_key = None
        self._warning_at = 0.0
        self._store = ConfigStore()
        fallback = {'version': 0, 'mode': 'manual', 'port': self.port,
                    'baudrate': self.baudrate, 'address': self.address, 'binding': None}
        self._config_error = ''
        try:
            self._config = self._store.load(fallback)
        except Exception as error:
            # Keep settings usable, but never silently operate on corrupt config.
            self._config = fallback
            self._config_error = f'连接配置读取失败：{error}；请在设置页重新确认并保存'
        self.port = self._config['port']
        self.baudrate = self._config['baudrate']
        self.address = self._config['address']
        self._devices, self._missing_usb = [], []
        self._connection_error = {'code': 'waiting', 'message': '等待传感器数据'}
        self._connection_publisher = self.create_publisher(String, '/air_quality/m701/connection', 1)
        self.create_service(ConfigureM701, '/air_quality/m701/configure', self._configure)
        self.create_timer(2.0, self._publish_connection)
        self._timer = self.create_timer(self.poll_interval, self._poll)
        self.get_logger().info(
            f'M701 bridge ready: {self.port}, {self.baudrate} 8N1, '
            f'address={self.address}, poll={self.poll_interval:.2f}s')

    def _base_payload(self):
        return {
            'sensor': 'M701',
            'port': self.port,
            'address': self.address,
            'sample_time': datetime.now(timezone.utc).isoformat(),
            'sample_time_unix': time.time(),
        }

    def _publish_invalid(self, error):
        if isinstance(error, ConnectionError):
            code = error.code
        elif isinstance(error, TimeoutError):
            code = 'response_timeout'
        elif (isinstance(error, PermissionError) or getattr(error, 'errno', None) == 13
              or re.search(r'\[Errno 13\]|Permission denied', str(error), re.I)):
            code = 'permission_denied'
        elif (getattr(error, 'errno', None) in (11, 16)
              or re.search(r'\[Errno (11|16)\]|exclusively lock|resource busy', str(error), re.I)):
            code = 'port_busy'
        else:
            code = 'communication_error'
        prefix = {'permission_denied': '串口权限不足，请核对 dialout 组和设备权限：',
                  'port_busy': '串口被其他程序占用，请关闭串口调试工具或重复采集程序：',
                  'response_timeout': '串口已打开但传感器无响应，请检查供电、接线、波特率和地址：'}.get(code, '')
        self._connection_error = {'code': code, 'message': prefix + str(error)}
        payload = self._base_payload()
        payload.update({
            'online': self._serial is not None,
            'valid': False,
            'warming_up': False,
            'values': None,
            'error': prefix + str(error),
            'error_code': code,
            'consecutive_failures': self._failures,
        })
        self.publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))))

    def _open_serial(self):
        if self._config_error:
            raise ConnectionError('config_error', self._config_error)
        if serial is None:
            raise ConnectionError('dependency_missing',
                'pyserial is not installed; install the python3-serial package')
        self._devices, self._missing_usb = inventory()
        try:
            device = resolve(self._config, self._devices)
        except ConnectionError as error:
            detail = diagnose(self._config, self._devices, self._missing_usb)
            raise ConnectionError(error.code, detail['message']) from error
        if self._config['binding'] is None:
            self._config['binding'] = identity(device)
        self.port = device['port']
        connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        )
        try:
            # USB indices can be reused between discovery and open. Never send
            # the first Modbus request without verifying the binding again.
            fresh, _ = inventory()
            if resolve(self._config, fresh)['port'] != self.port:
                raise ConnectionError('device_changed', '打开串口期间设备发生变化，请重新连接')
            connection.reset_input_buffer()
        except Exception:
            connection.close()
            raise
        self._serial = connection
        if self._first_opened_at is None:
            self._first_opened_at = time.monotonic()
        self.get_logger().info(f'M701 serial port opened: {self.port}')

    def _close_serial(self):
        connection, self._serial = self._serial, None
        self._first_opened_at = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def _read_response(self):
        request = build_read_all_request(self.address)
        self._serial.reset_input_buffer()
        self._serial.write(request)
        self._serial.flush()
        # CH340 automatic direction switching may need a short turnaround.
        time.sleep(0.002)

        received = bytearray()
        deadline = time.monotonic() + self.response_timeout
        while time.monotonic() < deadline:
            waiting = int(getattr(self._serial, 'in_waiting', 0) or 0)
            chunk = self._serial.read(min(64, max(1, waiting)))
            if chunk:
                received.extend(chunk)
                frame = extract_read_all_response(received, self.address)
                if frame is not None:
                    return frame
        if received:
            raise ConnectionError('invalid_response',
                                  f'收到 {len(received)} 字节，但未通过 M701 帧长度、地址、功能码和 CRC 校验')
        raise TimeoutError(
            f'M701 response timeout after {self.response_timeout:.2f}s '
            f'({len(received)} bytes received)')

    def _poll(self):
        try:
            if self._serial is None:
                self._open_serial()
            frame = self._read_response()
            values = parse_read_all_response(frame, self.address)
            self._connection_error = {'code': 'ok', 'message': 'M701 数据校验通过'}
            self._failures = 0
            self._warning_key = None
            elapsed = (
                time.monotonic() - self._first_opened_at
                if self._first_opened_at is not None else 0.0)
            payload = self._base_payload()
            payload.update({
                'online': True,
                'valid': True,
                'warming_up': elapsed < self.warmup_sec,
                'warmup_remaining_sec': max(0.0, self.warmup_sec - elapsed),
                'values': values,
                'error': '',
                'consecutive_failures': 0,
            })
            self.publisher.publish(String(data=json.dumps(
                payload, ensure_ascii=False, separators=(',', ':'))))
        except Exception as error:
            self._failures += 1
            warning_key = (type(error).__name__, str(error))
            now = time.monotonic()
            if warning_key != self._warning_key or now - self._warning_at >= 30.0:
                self.get_logger().warning(f'M701 poll failed: {error}')
                self._warning_key, self._warning_at = warning_key, now
            self._publish_invalid(error)
            self._close_serial()

    def _connection_status(self):
        diagnostic = diagnose(self._config, self._devices, self._missing_usb)
        if self._config_error:
            diagnostic = {'code': 'config_error', 'message': self._config_error,
                          'suggested_port': '', 'confirmation_required': False}
        return {'config': self._config, 'config_error': self._config_error,
                'devices': [dict(d, identity=observation(d)) for d in self._devices],
                'usb_without_serial': self._missing_usb,
                'active_port': self.port if self._serial is not None else '',
                'diagnostic': diagnostic,
                'error': self._connection_error}

    def _publish_connection(self):
        try:
            self._devices, self._missing_usb = inventory()
            payload = self._connection_status()
        except Exception as error:
            payload = {'config': self._config, 'devices': [], 'config_error': self._config_error,
                       'error': {'code': 'inventory_error', 'message': str(error)}}
        self._connection_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _configure(self, request, response):
        # Timer and service share the default mutually-exclusive callback group:
        # test and polling cannot concurrently own/read/write the same port.
        try:
            self._devices, self._missing_usb = inventory()
            if request.operation not in ('list', 'test', 'save'):
                raise ValueError('未知气体传感器操作')
            if request.operation != 'list':
                if request.expected_version != self._config['version']:
                    raise ValueError('配置已被其他页面更新，请刷新后重试')
                proposed = draft(self._config, json.loads(request.config_json), self._devices)
                if request.operation == 'save':
                    proposed['version'] += 1
                    self._store.save(proposed)  # Failure leaves active config intact.
                    self._close_serial()
                    self._config = proposed
                    self._config_error = ''
                    self.port, self.baudrate, self.address = (
                        proposed['port'], proposed['baudrate'], proposed['address'])
                    self._connection_error = {'code': 'waiting', 'message': '配置已保存，等待首次有效数据'}
                    self._publish_invalid(ConnectionError('waiting', '连接配置已更新，等待新数据'))
                    response.message = '已保存独立传感器配置，两个本体共用；后台将重新连接'
                else:
                    previous = self._config, self.port, self.baudrate, self.address, self._config_error
                    self._close_serial()
                    try:
                        self._config, self._config_error = proposed, ''
                        self.baudrate, self.address = proposed['baudrate'], proposed['address']
                        self._open_serial()
                        values = parse_read_all_response(self._read_response(), self.address)
                        response.message = f'只读查询校验通过（CO₂ {values["co2_ppm"]} ppm）；尚未保存配置'
                    finally:
                        self._close_serial()
                        self._config, self.port, self.baudrate, self.address, self._config_error = previous
                        self._publish_invalid(ConnectionError('waiting', '连接测试结束，原配置等待重新采样'))
            else:
                response.message = '设备列表已刷新；未向串口发送任何命令'
            response.success = True
        except Exception as error:
            response.success, response.message = False, str(error)
        response.status_json = json.dumps(self._connection_status(), ensure_ascii=False)
        return response

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = M701AirQualityBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
