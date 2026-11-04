#!/usr/bin/env python3
"""ROS adapter for fail-closed automatic terrain-mode supervision."""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

from lzp_interface.msg import TerrainObservation as TerrainObservationMessage
from lzp_interface.srv import SetTraversalMode

from .automatic_terrain_navigation import AutomaticTerrainController
from .navigation_mode import MAP_2D, TERRAIN_3D, normalize_navigation_mode
from .terrain_navigation import TerrainCapabilityProfile, TerrainObservation


class AutomaticTerrainNavigationNode(Node):
    """Gate Nav2 and switch gait only after stable live observations."""

    def __init__(self):
        super().__init__('automatic_terrain_navigation_node')
        self._group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._observation_timeout = max(0.3, float(self.declare_parameter(
            'observation_timeout_sec', 0.75).value))
        self._controller = AutomaticTerrainController(
            enter_confirmations=int(self.declare_parameter(
                'enter_confirmations', 3).value),
            exit_confirmations=int(self.declare_parameter(
                'exit_confirmations', 5).value))
        self._controller.configure(
            enabled=bool(self.declare_parameter('enabled', True).value),
            enabled_modes=self.declare_parameter(
                'enabled_modes', ['rough_ground', 'obstacle', 'ramp']).value)
        self._observation = None
        self._observation_at = 0.0
        self._observation_sequence = 0
        self._observation_stamp = 0.0
        self._evidence_source = 'current'
        self._navigation_active = False
        self._navigation_known = False
        self._campus_active = False
        self._navigation_mode = TERRAIN_3D
        # Only a fresh, validated Xingji identity may use the requested
        # score-threshold exception. Unknown bodies retain the old policy.
        self._body_id = ''
        self._body_at = 0.0
        self._body_generation = None
        self._fixed_body_identity = None
        self._pending_fixed_identity = None
        self.declare_parameter('xingji_3d_fixed_obstacle_enabled', True)
        self.declare_parameter(
            'xingji_3d_confidence_check_enabled', False)
        self._pending_future = None
        self._pending_mode = ''
        self._pending_at = 0.0
        self._pending_timed_out = False
        self._switch_fault = ''
        self._switch_timeout = max(1.0, float(self.declare_parameter(
            'mode_switch_timeout_sec', 12.0).value))
        self._last_status = ''

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensors = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._safe_publisher = self.create_publisher(
            Bool, '/automatic_terrain_navigation/safe', latched)
        self._speed_publisher = self.create_publisher(
            Float32, '/automatic_terrain_navigation/speed_limit', latched)
        self._status_publisher = self.create_publisher(
            String, '/automatic_terrain_navigation/status', latched)
        self.create_subscription(
            TerrainObservationMessage, '/terrain_observation',
            self._observation_callback, sensors, callback_group=self._group)
        self.create_subscription(
            Bool, '/navigation_active', self._navigation_callback,
            latched, callback_group=self._group)
        self.create_subscription(
            Bool, '/campus_mission/active', self._campus_callback,
            latched, callback_group=self._group)
        self.create_subscription(
            String, '/automatic_terrain_navigation/config',
            self._config_callback, latched, callback_group=self._group)
        self.create_subscription(
            String, '/navigation/mode', self._mode_callback,
            latched, callback_group=self._group)
        self.create_subscription(
            String, '/robot_body/status', self._body_callback,
            latched, callback_group=self._group)
        self._mode_client = self.create_client(
            SetTraversalMode, '/dog_remote/set_traversal_mode',
            callback_group=self._group)
        self._timer = self.create_timer(
            0.10, self._tick, callback_group=self._group)
        self.get_logger().info(
            'Automatic same-map terrain navigation supervisor is ready; '
            'stairs remain excluded')

    def _observation_callback(self, message):
        try:
            stamp = message.header.stamp
            source_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
            age = self.get_clock().now().nanoseconds / 1e9 - source_time
            if (source_time <= 0 or not -0.05 <= age <= self._observation_timeout
                    or str(message.header.frame_id).lstrip('/') != 'odom'):
                raise ValueError('地形观测时间戳或坐标系无效')
            value = TerrainObservation.from_mapping({
                'terrain_type': message.terrain_type,
                'slope_deg': message.slope_deg,
                'cross_slope_deg': message.cross_slope_deg,
                'step_height_m': message.step_height_m,
                'gap_width_m': message.gap_width_m,
                'roughness_m': message.roughness_m,
                'clearance_width_m': message.clearance_width_m,
                'confidence': message.confidence,
                'negative_obstacle': message.negative_obstacle,
                'slippery': message.slippery,
            })
        except (AttributeError, TypeError, ValueError) as error:
            with self._lock:
                self._observation = None
                self._observation_at = 0.0
            self.get_logger().error(
                f'Invalid live terrain observation rejected: {error}')
            return
        with self._lock:
            if source_time <= self._observation_stamp:
                return
            self._observation = value
            self._observation_stamp = source_time
            self._evidence_source = str(message.evidence_source or 'current')
            self._observation_at = time.monotonic() - max(0., age)
            self._observation_sequence += 1

    def _navigation_callback(self, message):
        with self._lock:
            self._navigation_active = bool(message.data)
            self._navigation_known = True

    def _body_callback(self, message):
        """Keep identity only; this callback never commands or switches a body."""
        try:
            data = json.loads(message.data)
            config = data.get('config', {})
            selected = config.get('selected', '')
            profile = config.get('profiles', {}).get(selected, {})
            state = data.get('status')
            generation = data.get('generation')
            valid = (selected in ('xingji', 'gangben_zsl1')
                     and data.get('active_body') == selected
                     and data.get('connected') is True
                     and type(generation) is int and generation >= 1
                     and not data.get('switching') and not data.get('error')
                     and isinstance(state, dict)
                     and not state.get('stale') and not state.get('emergency_latched')
                     and not state.get('hardware_emergency_latched')
                     and not state.get('hardware_error') and not state.get('error')
                     and profile.get('navigation_verified') is True)
            body = selected if valid else ''
        except (AttributeError, TypeError, ValueError):
            body = ''
        with self._lock:
            self._body_id = body
            self._body_at = time.monotonic()
            self._body_generation = generation if body else None

    def _fixed_obstacle_enabled(self, now=None):
        now = time.monotonic() if now is None else now
        return bool(
            self.get_parameter('xingji_3d_fixed_obstacle_enabled').value is True
            and self._navigation_mode == TERRAIN_3D
            and self._body_id == 'xingji'
            and type(self._body_generation) is int and self._body_generation >= 1
            and 0.0 <= now - self._body_at <= 2.0)

    def _confidence_required(self, now=None):
        """Skip only the composite score for confirmed, current Xingji 3-D."""
        now = time.monotonic() if now is None else now
        return not (
            self.get_parameter('xingji_3d_confidence_check_enabled').value is False
            and self._navigation_mode == TERRAIN_3D
            and getattr(self, '_body_id', '') == 'xingji'
            and 0.0 <= now - getattr(self, '_body_at', 0.0) <= 2.0)

    def _campus_callback(self, message):
        with self._lock:
            previous = self._campus_active
            self._campus_active = bool(message.data)
            if previous and not self._campus_active and not self._fixed_obstacle_enabled():
                self._controller.assume_mode('normal')

    def _config_callback(self, message):
        try:
            value = json.loads(message.data)
            profile = TerrainCapabilityProfile.from_mapping(
                value.get('capability_profile'))
            automatic = value.get('automatic_navigation', value)
            with self._lock:
                self._controller.configure(
                    enabled=bool(automatic.get('enabled', True)),
                    enabled_modes=automatic.get(
                        'enabled_modes', ['rough_ground', 'obstacle', 'ramp']),
                    profile=profile)
                if (self._switch_fault and self._pending_future is None
                        and not self._navigation_active and not self._campus_active):
                    self._switch_fault = ''
                    self._controller.assume_mode('unconfirmed')
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(
                f'Automatic terrain config rejected: {error}')

    def _mode_callback(self, message):
        try:
            payload = json.loads(str(message.data))
            value = payload.get('mode') if isinstance(payload, dict) else payload
            mode = normalize_navigation_mode(value)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(
                f'Navigation mode rejected by terrain supervisor: {error}')
            mode = TERRAIN_3D
        with self._lock:
            self._navigation_mode = mode

    def _publish(self, decision):
        confidence_required = self._confidence_required()
        confidence_policy = ('suspended_2d' if self._navigation_mode == MAP_2D
                             else 'required' if confidence_required
                             else 'bypassed_xingji_3d')
        fixed = self._fixed_obstacle_enabled()
        speed_policy = ('nav2_body_limits' if fixed
                        else 'suspended_2d' if self._navigation_mode == MAP_2D
                        else 'terrain_adaptive')
        if not decision.safe:
            speed_limit_source = 'safety_stop'
        elif decision.state == 'suspended':
            speed_limit_source = 'campus_connector'
        elif decision.state == 'restricted_flat_site':
            speed_limit_source = 'restricted_map'
        elif fixed and self._evidence_source not in (
                'current_anchored', 'recent_measured_anchored'):
            speed_limit_source = 'unanchored_observation'
        elif fixed or self._navigation_mode == MAP_2D:
            speed_limit_source = 'nav2_body_limits'
        else:
            speed_limit_source = 'terrain_assessment'
        self._safe_publisher.publish(Bool(data=decision.safe))
        self._speed_publisher.publish(Float32(data=decision.speed_limit_mps))
        payload = json.dumps({
            'stamp_ns': self.get_clock().now().nanoseconds,
            'state': decision.state,
            'safe': decision.safe,
            'current_mode': decision.current_mode,
            'requested_mode': decision.requested_mode,
            'speed_limit_mps': decision.speed_limit_mps,
            'speed_policy': speed_policy,
            'terrain_speed_limit_enabled': (
                self._navigation_mode != MAP_2D and not fixed),
            'speed_limit_source': speed_limit_source,
            'reason': decision.reason,
            'navigation_active': self._navigation_active,
            'campus_active': self._campus_active,
            'navigation_mode': self._navigation_mode,
            'evidence_source': self._evidence_source,
            'confidence_check_enabled': (
                self._navigation_mode != MAP_2D and confidence_required),
            'confidence_policy': confidence_policy,
            'execution_policy': ('fixed_obstacle' if self._fixed_obstacle_enabled()
                                 else 'terrain_classification'),
            'observed_terrain_type': (self._observation.terrain_type
                                      if self._observation is not None else 'unknown'),
            'observation_age_sec': (
                time.monotonic() - self._observation_at
                if self._observation_at > 0.0 else -1.0),
        }, ensure_ascii=False, separators=(',', ':'))
        self._status_publisher.publish(String(data=payload))
        # Observation age changes every tick. Log decisions, not that ticking
        # field, so a stopped robot leaves a useful reason in the launch log.
        status_key = (
            decision.state, decision.safe, decision.current_mode,
            decision.requested_mode, decision.reason, confidence_policy)
        if status_key != self._last_status:
            self._last_status = status_key
            detail = (
                f'地形导航状态={decision.state}, safe={decision.safe}, '
                f'限速={decision.speed_limit_mps:.2f} m/s: {decision.reason}')
            if confidence_policy == 'bypassed_xingji_3d':
                detail += '；星玑3D综合置信度阈值已取消，其他地形检查保留'
            if decision.safe:
                self.get_logger().info(detail)
            else:
                self.get_logger().warning(detail)

    def _consume_future(self):
        future = self._pending_future
        if future is None or not future.done():
            return
        mode = self._pending_mode
        fixed_identity = self._pending_fixed_identity
        self._pending_fixed_identity = None
        self._pending_future = None
        self._pending_mode = ''
        self._pending_at = 0.0
        if fixed_identity is not None and (
                not self._fixed_obstacle_enabled()
                or fixed_identity != (self._body_id, self._body_generation)):
            self._controller.assume_mode('unconfirmed')
            self._pending_timed_out = False
            self.get_logger().warning('固定越障确认所属本体或导航策略已改变，忽略迟到结果')
            return
        try:
            result = future.result()
            success = bool(result and result.success)
            message = getattr(result, 'message', '')
        except Exception as error:
            success = False
            message = str(error)
        if self._pending_timed_out:
            success = False
            message = '模式切换超时后的迟到结果已忽略；停止任务后重新保存地形设置'
        self._pending_timed_out = False
        if not success:
            self._switch_fault = str(message or '地形运动模式切换失败')
        self._controller.confirm_mode(
            mode, success=success, message=message)
        if success:
            self.get_logger().info(message or f'Switched terrain mode to {mode}')
        else:
            self.get_logger().error(
                f'Automatic terrain mode switch to {mode} failed: {message}')

    def _tick(self):
        with self._lock:
            self._consume_future()
            now = time.monotonic()
            if (self._pending_future is not None and self._pending_at > 0.0
                    and now - self._pending_at >= self._switch_timeout):
                self._pending_timed_out = True
                self._switch_fault = (
                    '地形模式切换超时，已保持停车；停止任务并等待服务返回后重新保存地形设置')
            if self._switch_fault:
                self._publish(self._controller._decision(
                    False, 'switch_failed', self._switch_fault))
                return
            fixed = self._fixed_obstacle_enabled(now)
            identity = (self._body_id, self._body_generation)
            if fixed and self._fixed_body_identity != identity:
                self._controller.assume_mode('unconfirmed')
                self._fixed_body_identity = identity
            elif not fixed and self._fixed_body_identity is not None:
                if (self._navigation_mode == TERRAIN_3D
                        and self.get_parameter('xingji_3d_fixed_obstacle_enabled').value is True
                        and (self._body_id != 'gangben_zsl1'
                             or not 0.0 <= now - self._body_at <= 2.0)):
                    self._controller.assume_mode('unconfirmed')
                    self._publish(self._controller._decision(
                        False, 'blocked', '星玑固定越障本体身份无效或过期，等待重新确认'))
                    return
                self._fixed_body_identity = None
                self._controller.assume_mode('normal')
            if not self._navigation_known:
                decision = self._controller._decision(
                    False, 'blocked', '等待导航任务状态')
            elif self._navigation_mode == MAP_2D:
                requested = (
                    'normal' if self._controller.current_mode != 'normal'
                    else '')
                decision = self._controller._decision(
                    True,
                    'suspended_2d',
                    '2D地图导航：地形步态监督已暂停',
                    requested=requested,
                    speed=self._controller.capability.profile.normal_speed_mps,
                )
            else:
                decision = self._controller.update(
                    self._observation,
                    observation_fresh=(
                        self._observation_at > 0.0 and
                        now - self._observation_at <= self._observation_timeout),
                    navigation_active=self._navigation_active,
                    campus_active=self._campus_active,
                    observation_sequence=self._observation_sequence,
                    check_confidence=self._confidence_required(now),
                    fixed_execution_mode='obstacle' if fixed else '')
                if self._evidence_source == 'restricted_map' and decision.safe:
                    decision = type(decision)(safe=True, state='restricted_flat_site',
                        current_mode=decision.current_mode, requested_mode=decision.requested_mode,
                        speed_limit_mps=min(.10, decision.speed_limit_mps),
                        reason='受限平地：使用已验证地图地面先验，实时近场坑洞不可见；低速通行')
                elif fixed and decision.safe and self._evidence_source not in (
                        'current_anchored', 'recent_measured_anchored'):
                    # This is an evidence restriction, not a terrain category
                    # limit; do not grant full speed to unanchored observations.
                    decision = type(decision)(safe=True, state=decision.state,
                        current_mode=decision.current_mode, requested_mode=decision.requested_mode,
                        speed_limit_mps=min(.10, decision.speed_limit_mps),
                        reason='星玑保持越障模式；地面证据尚未锚定，保留证据限速')
            if decision.requested_mode and self._pending_future is None:
                if self._mode_client.service_is_ready():
                    request = SetTraversalMode.Request()
                    request.mode = decision.requested_mode
                    self._pending_mode = decision.requested_mode
                    self._pending_fixed_identity = identity if fixed else None
                    self._pending_at = now
                    try:
                        self._pending_future = self._mode_client.call_async(request)
                    except Exception as error:
                        self._pending_mode = ''
                        self._pending_at = 0.0
                        self._switch_fault = f'模式切换请求失败：{error}'
                        self._publish(self._controller._decision(
                            False, 'switch_failed', self._switch_fault))
                        return
                    decision = type(decision)(
                        safe=False, state='switching',
                        current_mode=decision.current_mode,
                        requested_mode=self._pending_mode,
                        speed_limit_mps=0.0,
                        reason=f'正在切换到 {self._pending_mode}')
                else:
                    decision = type(decision)(
                        safe=False, state='blocked',
                        current_mode=decision.current_mode,
                        requested_mode='', speed_limit_mps=0.0,
                        reason='机器狗地形模式服务不可用')
            elif self._pending_future is not None:
                decision = type(decision)(
                    safe=False, state='switching',
                    current_mode=decision.current_mode,
                    requested_mode=self._pending_mode,
                    speed_limit_mps=0.0,
                    reason=f'正在切换到 {self._pending_mode}')
            self._publish(decision)


def main(args=None):
    rclpy.init(args=args)
    node = AutomaticTerrainNavigationNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
