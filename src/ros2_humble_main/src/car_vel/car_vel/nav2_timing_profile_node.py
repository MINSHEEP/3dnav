"""Bounded idle-only controller computation profile; no velocity publishers."""
from __future__ import annotations

import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_msgs.msg import Bool, String

from .nav2_timing_profile import BaselineStore, PARAMETER, TimingProfilePolicy, valid_batch


def configured_batch(path):
    """Read the same original parameter file passed to Nav2 by the launch."""
    import yaml
    value = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return valid_batch(value['controller_server']['ros__parameters']['FollowPath']['batch_size'])


class Nav2TimingProfileNode(Node):
    def __init__(self):
        super().__init__('nav2_timing_profile')
        self.declare_parameter('enabled', True)
        self.declare_parameter('optimized_batch_size', 400)
        baseline_file = str(self.declare_parameter('baseline_params_file', '').value)
        path = str(self.declare_parameter('ownership_path', str(
            Path.home() / '.ros/robot_navigation/nav2_timing_profile.json')).value)
        configured = configured_batch(baseline_file)
        optimized = int(self.get_parameter('optimized_batch_size').value)
        self._store = BaselineStore(path, configured, optimized)
        self._record_error = ''
        try:
            record = self._store.load_record()
            baseline = record['baseline'] if record else None
        except (OSError, ValueError, TypeError) as error:
            record = None
            baseline = None
            self._record_error = f'原控制器配置记录不可用，未修改参数；请在停止整个导航后台后检查记录：{error}'
        self._policy = TimingProfilePolicy(optimized_batch=optimized,
            configured_baseline=configured, recovered_baseline=baseline)
        self._owned_write_pending = bool(record and record.get('pending_write'))
        self._write_gid = str(record.get('controller_gid') or '') if record else ''
        self._controller_gid = ''
        self._observed_controller_gid = self._write_gid
        self._record_invalid = bool(self._record_error)
        if self._owned_write_pending:
            self._policy.uncertain_write('检测到上次未决参数写入，等待确认控制器已重启；请重启整个导航后台')
        elif self._record_invalid:
            self._policy.uncertain_write(self._record_error)
        self._mode = self._body = ''
        self._active = True
        self._mode_at = self._body_at = self._active_at = 0.0
        self._future = None
        self._operation = None
        self._next_retry = self._last_read = 0.0
        self._read_client = self.create_client(GetParameters, '/controller_server/get_parameters')
        self._write_client = self.create_client(SetParametersAtomically,
            '/controller_server/set_parameters_atomically')
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(String, '/navigation/controller_timing', qos)
        self.create_subscription(String, '/navigation/mode', self._mode_callback, qos)
        self.create_subscription(String, '/robot_body/status', self._body_callback, qos)
        self.create_subscription(Bool, '/navigation_active', self._active_callback, qos)
        self._last_log = None
        self.create_timer(0.25, self._tick)

    def _mode_callback(self, message):
        try:
            value = json.loads(message.data)
            mode = value.get('mode')
            if mode not in ('map_2d', 'terrain_3d'):
                raise ValueError('unknown navigation mode')
            self._mode = mode
        except (ValueError, TypeError, AttributeError):
            self._mode = ''
        self._mode_at = time.monotonic()

    def _body_callback(self, message):
        try:
            value = json.loads(message.data)
            selected = value.get('config', {}).get('selected', '')
            valid = (selected in ('xingji', 'gangben_zsl1', 'xuanji')
                     and value.get('active_body') == selected and value.get('connected') is True
                     and not value.get('switching') and not value.get('error'))
            self._body = selected if valid else ''
        except (ValueError, TypeError, AttributeError):
            self._body = ''
        self._body_at = time.monotonic()

    def _active_callback(self, message):
        self._active = getattr(message, 'data', None) is not False
        self._active_at = time.monotonic()

    def _finish(self, now):
        future, self._future = self._future, None
        operation, self._operation = self._operation, None
        try:
            result = future.result()
            if operation == 'read':
                if len(result.values) != 1 or result.values[0].type != ParameterType.PARAMETER_INTEGER:
                    raise ValueError('controller batch_size readback is not an integer')
                self._policy.observe_actual(int(result.values[0].integer_value))
                self._last_read = now
                if self._owned_write_pending:
                    self._store.save(self._policy.baseline, controller_gid=self._controller_gid)
                    self._owned_write_pending = False
            else:
                self._policy.finish_set(bool(result.result.successful), result.result.reason)
        except Exception as error:
            if operation == 'set':
                self._policy.uncertain_write(
                    f'控制器参数写入结果不确定：{error}；请重启整个导航后台')
            else:
                self._policy.fail(f'控制器配置同步失败：{error}')
            self._next_retry = now + 1.0

    def _observe_controller_instance(self):
        try:
            endpoints = self.get_publishers_info_by_topic('/parameter_events')
            gids = {bytes(endpoint.endpoint_gid).hex() for endpoint in endpoints
                    if endpoint.node_name == 'controller_server' and endpoint.node_namespace == '/'
                    and any(endpoint.endpoint_gid)}
            gid = next(iter(gids)) if len(gids) == 1 else ''
        except (AttributeError, TypeError, ValueError, RuntimeError):
            gid = ''
        self._controller_gid = gid
        previous = self._observed_controller_gid
        if (gid and previous and gid != previous and not self._record_invalid):
            # The parameter publisher belongs to the lifecycle node itself;
            # a new unique endpoint proves the old controller node is gone.
            if self._future is not None:
                self._future.cancel()
                self._future = self._operation = None
            self._policy.controller_restarted()
        if gid:
            self._observed_controller_gid = gid

    def _tick(self):
        now = time.monotonic()
        self._observe_controller_instance()
        # A navigation mode is a latched configuration event, not a sensor
        # heartbeat. Silence cannot revoke an already confirmed parameter or
        # reset its fallback. Explicit changes/invalid messages still update it.
        mode = self._mode
        body = self._body if now - self._body_at <= 2.0 else ''
        active = self._active or now - self._active_at > 1.0
        self._policy.set_context(mode, body, active=active,
            enabled=self.get_parameter('enabled').value is True)
        if self._future is not None:
            if self._future.done():
                self._finish(now)
            elif self._policy.expire(now):
                if self._operation != 'set':
                    self._future.cancel()
                    self._future = self._operation = None
                    self._next_retry = now + 1.0
        if self._future is None and now >= self._next_retry:
            operation = self._policy.next_operation()
            # Read-only reconciliation detects a restarted controller without
            # resetting its optimizer or extending any command's timestamp.
            if (operation is None and self._policy.pending is None
                    and not self._policy.waiting_late_write
                    and self._policy.profile != 'unknown' and now - self._last_read >= 2.0):
                operation = ('read', None)
            if operation is not None:
                kind, value = operation
                client = self._read_client if kind == 'read' else self._write_client
                if client.service_is_ready():
                    try:
                        if kind == 'set':
                            if not self._controller_gid:
                                raise RuntimeError('无法确认唯一controller_server代际，未修改参数')
                            # Persist ownership before the RPC may change Nav2.
                            self._store.save(self._policy.baseline, pending_write=True,
                                             controller_gid=self._controller_gid)
                            self._owned_write_pending = True
                            self._write_gid = self._controller_gid
                            request = SetParametersAtomically.Request()
                            request.parameters = [Parameter(name=PARAMETER,
                                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                                     integer_value=value))]
                        else:
                            request = GetParameters.Request(names=[PARAMETER])
                        self._policy.begin(kind, value, now=now)
                        self._operation = kind
                        self._future = client.call_async(request)
                    except Exception as error:
                        if kind == 'set' and self._owned_write_pending:
                            self._policy.uncertain_write(
                                f'控制器参数请求结果不确定：{error}；请重启整个导航后台')
                        else:
                            self._policy.fail(f'控制器配置请求失败：{error}')
                        self._next_retry = now + 1.0
        status = self._policy.status()
        status.update(stamp_ns=self.get_clock().now().nanoseconds,
            navigation_mode=mode, body_id=body, navigation_active=active,
            mode_message_age_sec=(round(now-self._mode_at, 3) if self._mode else None),
            mode_context_source='latched_configuration',
            activity_confirmed=0.0 <= now-self._active_at <= 1.0,
            controller_gid=self._controller_gid,
            actual_verified_age_sec=(round(now - self._last_read, 3) if self._last_read else None),
            baseline_params_file_note='基线来自实际回读；持久记录用于节点重启恢复',
            record_warning=self._record_error)
        message = ('控制器候选轨迹计算量已降低，速度和指令有效期保持原设置'
                   if status['ready'] and status['actual_batch_size'] != status['baseline_batch_size']
                   else '已使用原控制器参数' if status['ready'] else
                   '等待有效的导航模式和本体配置，未修改控制器参数' if status['profile'] == 'unknown' else
                   '等待导航空闲后同步控制器计算配置' if active else
                   '正在同步控制器计算配置')
        if status['error']:
            message += '；' + status['error']
        status['message'] = message
        key = (status['profile'], status['state'], status['actual_batch_size'], status['error'])
        if key != self._last_log:
            self._last_log = key
            self.get_logger().info(f'NAV2_TIMING {json.dumps(status, ensure_ascii=False)}')
        self._publisher.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = Nav2TimingProfileNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
