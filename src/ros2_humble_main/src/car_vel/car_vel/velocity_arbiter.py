#!/usr/bin/env python3
"""Exclusive fail-safe velocity ownership for teleop, Nav2 and connectors."""

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float32, String
from lzp_interface.msg import RouteExecutionStatus
from .motion_diagnostics import (
    MotionTrace, CommandStreamTiming, age, velocity, arbiter_reason, velocity_guard_status)
from .navigation_motion_progress import NavigationMotionProgress

from .velocity_ownership import (
    constrain_navigation_velocity,
    select_velocity_source,
    should_stop_for_terrain_update,
)


class VelocityArbiter(Node):
    """Forward exactly one command source and stop on stale ownership/data."""

    def __init__(self):
        super().__init__('velocity_arbiter')
        self.timeout = max(0.1, float(self.declare_parameter(
            'command_timeout_sec', 0.3).value))
        self.terrain_timeout = max(0.2, float(self.declare_parameter(
            'terrain_guard_timeout_sec', 0.5).value))
        self.lio_pause_timeout = max(0.5, float(self.declare_parameter(
            'lio_pause_timeout_sec', 2.5).value))
        self.navigation_input_topic = str(self.declare_parameter(
            'navigation_input_topic', '/cmd_vel').value).strip()
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/cmd_vel_safe').value).strip()
        self._controller_diagnostic_topic = str(self.declare_parameter(
            'controller_diagnostic_topic', '/cmd_vel_nav').value).strip()
        self._controller_timing = CommandStreamTiming(self._controller_diagnostic_topic)
        if not self.navigation_input_topic or not self.output_topic:
            raise RuntimeError('Velocity arbiter topics must not be empty')
        if self.navigation_input_topic == self.output_topic:
            raise RuntimeError(
                'Velocity arbiter input and output topics must be different')
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(
            Twist, self.output_topic, reliable)
        self.create_subscription(
            Twist, '/cmd_vel_teleop', self._teleop_callback, reliable)
        self.create_subscription(
            Twist, self.navigation_input_topic, self._nav_callback, reliable)
        self.create_subscription(
            Twist, self._controller_diagnostic_topic, self._controller_diagnostic_callback, reliable)
        self.create_subscription(
            Twist, '/cmd_vel_campus', self._campus_callback, reliable)
        self.create_subscription(
            Bool, '/campus_mission/connector_active', self._active_callback, latched)
        self.create_subscription(
            Bool, '/automatic_terrain_navigation/safe',
            self._terrain_safe_callback, latched)
        self.create_subscription(
            Float32, '/automatic_terrain_navigation/speed_limit',
            self._terrain_speed_callback, latched)
        self.create_subscription(
            String, '/automatic_terrain_navigation/status',
            self._terrain_status_callback, latched)
        self.create_subscription(
            String, '/navigation/mode', self._navigation_mode_callback, latched)
        self.create_subscription(
            String, '/robot_body/status', self._body_callback, latched)
        # Observations only: these callbacks never set a motion gate.
        self.create_subscription(Bool, '/navigation_active', self._navigation_active_callback, latched)
        self.create_subscription(RouteExecutionStatus, '/route_execution_status', self._route_diagnostic_callback, latched)
        self.create_subscription(String, '/terrain_observation/status', self._observer_diagnostic_callback, reliable)
        self.create_subscription(Odometry, '/Odometry', self._progress_odometry_callback, sensor)
        self._guard_publisher = self.create_publisher(
            String, '/navigation/velocity_guard', latched)
        self.create_subscription(
            Empty, '/emergency_stop', self._emergency_callback, reliable)
        self.create_subscription(
            Empty, '/motion_stop', self._emergency_callback, reliable)
        self.create_subscription(
            String, '/dog_remote/command', self._manual_command_callback, reliable)
        self.create_subscription(
            Empty, '/clear_emergency_stop', self._clear_callback, reliable)
        self.create_subscription(
            Bool, '/lio/motion_paused', self._lio_pause_callback, latched)
        self.create_subscription(
            Bool, '/navigation/motion_paused', self._navigation_pause_callback, latched)
        self._lock = threading.RLock()
        self._campus_active = False
        self._emergency = False
        self._lio_paused = True
        self._lio_pause_at = None
        self._navigation_paused = True
        self._navigation_pause_at = None
        self._terrain_safe = False
        self._terrain_speed_limit = 0.0
        self._terrain_guard_at = 0.0
        self._terrain_guard_enabled = True
        self._require_atomic_terrain_status = True
        self._restricted_terrain = False
        self._terrain_evidence = ''
        self._body_id = ''; self._body_at = 0.0; self._body_generation = None
        self._global_localization = None
        self._global_localization_at = 0.
        self.create_subscription(String, '/localization/status',
                                 self._global_localization_callback, latched)
        self._guard_reason = ''
        self._guard_published_at = 0.0
        self._latest_teleop = Twist()
        self._latest_nav = Twist()
        self._latest_campus = Twist()
        self._teleop_at = 0.0
        self._nav_at = 0.0
        self._campus_at = 0.0
        self._diag_nav_received_at=0.0
        self._diag_navigation_active=False
        self._diag_route={};self._diag_route_at=0.0
        self._diag_observer={};self._diag_observer_at=0.0
        self._diag_terrain={}
        self._progress_monitor = NavigationMotionProgress()
        self._progress_epoch = 0
        self._progress_warning_code = ''
        self._progress_log_at = -math.inf
        self._progress_log_code = ''
        self._motion_trace=MotionTrace(self.get_logger(),'NAV_VELOCITY')
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            'Velocity arbiter ready: connector > teleop > terrain-gated nav; '
            f'navigation input={self.navigation_input_topic}, '
            f'hardware output={self.output_topic}, '
            f'command timeout={self.timeout:.2f} s')

    def _teleop_callback(self, message):
        with self._lock:
            self._progress_epoch += 1
            self._latest_teleop = message
            self._teleop_at = time.monotonic()

    def _nav_callback(self, message):
        with self._lock:
            if message.linear.x == 0. and message.linear.y == 0. and message.angular.z == 0.:
                self._progress_epoch += 1
            self._latest_nav = message
            self._nav_at = time.monotonic()
            self._diag_nav_received_at = self._nav_at

    def _navigation_active_callback(self, message):
        with self._lock:
            active = getattr(message,'data',None) is True
            if active != self._diag_navigation_active:
                self._progress_epoch += 1
            self._diag_navigation_active = active

    def _progress_odometry_callback(self, message):
        """Observe LIO pose only; bad telemetry cannot affect a motion gate."""
        try:
            pose = message.pose.pose
            q = pose.orientation
            quaternion = tuple(float(v) for v in (q.x, q.y, q.z, q.w))
            norm = math.sqrt(sum(v*v for v in quaternion))
            if not math.isfinite(norm) or norm < 1e-6:
                raise ValueError('invalid odometry orientation')
            qx, qy, qz, qw = (v/norm for v in quaternion)
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            with self._lock:
                self._progress_monitor.observe(now=time.monotonic(), stamp=stamp,
                    frame=message.header.frame_id, child_frame=message.child_frame_id,
                    x=float(pose.position.x), y=float(pose.position.y), yaw=yaw)
        except Exception:
            try:
                with self._lock:
                    self._progress_monitor.observe(now=time.monotonic(), stamp=0,
                        frame='', child_frame='', x=math.nan, y=math.nan, yaw=math.nan)
            except Exception:
                pass

    def _route_diagnostic_callback(self, message):
        try:
            point=int(message.point_index)
            context=dict(route_id=int(message.route_id),route_name=str(message.route_name)[:80],
                map_name=str(message.map_name)[:80],point_number=point+1 if point>=0 else None,
                point_count=int(message.point_count),loop=int(message.current_loop),
                state=str(message.state)[:60],mode=str(message.mode)[:40])
            with self._lock:
                self._diag_route=context;self._diag_route_at=time.monotonic()
        except (AttributeError,TypeError,ValueError,OverflowError):
            return

    def _observer_diagnostic_callback(self, message):
        try:
            raw=json.loads(str(message.data)[:8192])
            if not isinstance(raw,dict):return
            context={k:raw[k] for k in ('state','message','terrain_type','confidence',
                'slope_deg','step_height_m','recent_ground_cells','evidence_source',
                'restricted_flat_site_enabled','map_pose_source_age_sec',
                'localization_refresh_state','corridor_return_count',
                'near_field_return_count','nearest_forward_return_m') if k in raw}
            with self._lock:
                self._diag_observer=context;self._diag_observer_at=time.monotonic()
        except (AttributeError,TypeError,ValueError):
            return

    def _campus_callback(self, message):
        with self._lock:
            self._latest_campus = message
            self._campus_at = time.monotonic()

    def _active_callback(self, message):
        with self._lock:
            active = bool(message.data)
            if active == self._campus_active:
                return
            self._campus_active = active
            self._progress_epoch += 1
            self._teleop_at = 0.0
            self._nav_at = 0.0
            self._campus_at = 0.0
        self.publisher.publish(Twist())

    def _terrain_safe_callback(self, message):
        if getattr(self, '_require_atomic_terrain_status', False):
            return
        now = time.monotonic()
        with self._lock:
            self._terrain_safe = bool(message.data)
            self._terrain_guard_at = now
            stop_navigation = should_stop_for_terrain_update(
                terrain_safe=self._terrain_safe,
                connector_active=self._campus_active,
                teleop_fresh=now - self._teleop_at <= self.timeout,
                terrain_guard_enabled=self._terrain_guard_enabled)
            if stop_navigation:
                self._progress_epoch = getattr(self, '_progress_epoch', 0)+1
        if stop_navigation:
            self.publisher.publish(Twist())

    def _terrain_speed_callback(self, message):
        if getattr(self, '_require_atomic_terrain_status', False):
            return
        with self._lock:
            self._terrain_speed_limit = max(0.0, float(message.data))

    def _controller_diagnostic_callback(self, message):
        # This callback is deliberately separate from _nav_callback: a raw
        # controller message cannot authorize or extend the smoothed command.
        with self._lock:
            self._controller_timing.observe(time.monotonic(), velocity(message))

    def _terrain_status_callback(self, message):
        now = time.monotonic()
        data={}; diagnostic_error=''
        try:
            data=json.loads(message.data)
            safe=data['safe']
            speed=float(data['speed_limit_mps'])
            age=(self.get_clock().now().nanoseconds-int(data['stamp_ns']))/1e9
            if type(safe) is not bool or not math.isfinite(speed) or not 0 <= speed <= .35:
                raise ValueError('invalid terrain status')
            if not 0 <= age <= self.terrain_timeout:
                raise ValueError('stale terrain status')
            if self._terrain_guard_enabled and (
                    data.get('navigation_mode') != 'terrain_3d'
                    or data.get('navigation_active') is not True):
                safe=False
            if data.get('evidence_source') == 'restricted_map':
                speed=min(speed,.10)
        except (KeyError, AttributeError, OverflowError, TypeError, ValueError, json.JSONDecodeError) as error:
            diagnostic_error=str(error)[:200]
            safe=False;speed=0.0
        with self._lock:
            self._diag_terrain={key:data[key] for key in ('state','reason','evidence_source',
                'observed_terrain_type','observation_age_sec','confidence_policy',
                'navigation_active','stamp_ns','speed_policy','terrain_speed_limit_enabled',
                'speed_limit_source') if isinstance(data,dict) and key in data}
            self._diag_terrain['validation_error']=diagnostic_error
            self._terrain_safe=safe
            self._terrain_speed_limit=speed
            self._restricted_terrain = safe and data.get('evidence_source') == 'restricted_map'
            self._terrain_evidence = str(data.get('evidence_source', '')) if safe else ''
            self._terrain_guard_at=now
            stop=should_stop_for_terrain_update(terrain_safe=safe,
                connector_active=self._campus_active,
                teleop_fresh=now-self._teleop_at<=self.timeout,
                terrain_guard_enabled=self._terrain_guard_enabled)
            if stop:
                self._progress_epoch = getattr(self, '_progress_epoch', 0)+1
        if stop:
            self.publisher.publish(Twist())

    def _navigation_mode_callback(self, message):
        try:
            payload = json.loads(str(message.data))
            mode = str(payload.get('mode', '')).strip().lower()
            if mode not in ('map_2d', 'terrain_3d'):
                raise ValueError(f'unsupported navigation mode {mode!r}')
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(
                f'Navigation mode rejected by velocity arbiter: {error}')
            with self._lock:
                self._terrain_guard_enabled = True
                self._progress_epoch += 1
                self._nav_at = 0.0
            self.publisher.publish(Twist())
            return
        with self._lock:
            guard_enabled = mode != 'map_2d'
            # The route manager republishes this state at 5 Hz. Only a real
            # mode transition invalidates commands; a heartbeat must neither
            # stop a fresh command nor extend an old command's lifetime.
            if guard_enabled == self._terrain_guard_enabled:
                return
            self._terrain_guard_enabled = guard_enabled
            self._progress_epoch += 1
            self._nav_at = 0.0
            if guard_enabled:
                self._terrain_safe=False
                self._terrain_guard_at=0.0
        self.publisher.publish(Twist())

    def _body_callback(self, message):
        try:
            data = json.loads(message.data)
            selected = data.get('config', {}).get('selected', '')
            state = data.get('status') or {}
            profile = data.get('config', {}).get('profiles', {}).get(selected, {})
            valid = (selected in ('xingji', 'gangben_zsl1')
                     and data.get('active_body') == selected and data.get('connected') is True
                     and not data.get('switching') and not data.get('error')
                     and not state.get('stale') and not state.get('emergency_latched')
                     and not state.get('hardware_error') and not state.get('error')
                     and profile.get('navigation_verified') is True)
            body = selected if valid else ''
            generation = data.get('generation')
        except (ValueError, TypeError, AttributeError):
            body = ''; generation = None
        with self._lock:
            changed = body != self._body_id or generation != self._body_generation
            self._body_id = body; self._body_generation = generation
            self._body_at = time.monotonic()
            if changed:
                self._progress_epoch += 1
            if changed and self._terrain_guard_enabled:
                self._nav_at = 0.0
                self._terrain_safe = False
        if changed and self._terrain_guard_enabled:
            self.publisher.publish(Twist())

    def _emergency_callback(self, _message):
        with self._lock:
            self._progress_epoch += 1
            self._emergency = True
            self._teleop_at = self._nav_at = self._campus_at = 0.0
            self.publisher.publish(Twist())

    def _manual_command_callback(self, message):
        try:
            value = json.loads(message.data)
        except (AttributeError, TypeError, ValueError):
            return
        if isinstance(value, dict):
            command = str(value.get('command', '')).strip().lower()
            if command == 'emergency':
                self._emergency_callback(message)
            elif command == 'reset':
                # Explicit human R/reset only. The body independently validates
                # its remote session and still requires new standing authority.
                self._clear_callback(message)

    def _clear_callback(self, _message):
        with self._lock:
            self._progress_epoch += 1
            self._emergency = False
            self._teleop_at = 0.0
            self._nav_at = 0.0
            self._campus_at = 0.0

    def _lio_pause_callback(self, message):
        value = getattr(message, 'data', None)
        paused = value if type(value) is bool else True
        with self._lock:
            changed = self._lio_pause_at is None or paused != self._lio_paused
            self._lio_paused = paused
            self._lio_pause_at = time.monotonic()
            if changed:
                # Neither held input nor pre-pause commands may be replayed.
                self._progress_epoch += 1
                self._teleop_at = self._nav_at = self._campus_at = 0.0
                self.publisher.publish(Twist())

    def _navigation_pause_callback(self, message):
        value = getattr(message, 'data', None)
        paused = value if type(value) is bool else True
        with self._lock:
            changed = (self._navigation_pause_at is None
                       or paused != self._navigation_paused)
            self._navigation_paused = paused
            self._navigation_pause_at = time.monotonic()
            if changed:
                self._progress_epoch += 1
                self._teleop_at = self._nav_at = self._campus_at = 0.0
                self.publisher.publish(Twist())

    def _global_localization_callback(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError("localization status must be an object")
        except (ValueError, TypeError):
            status = {}
        with self._lock:
            previous = self._global_localization or {}
            previous_age = time.monotonic()-self._global_localization_at
            self._global_localization = status
            self._global_localization_at = time.monotonic()
            if (self._body_id == 'xingji' and self._terrain_guard_enabled
                    and previous and ((previous.get('ready') is True)
                    != (status.get('ready') is True) or previous_age > .3)):
                if status.get('ready') is True:
                    self._nav_at = self._campus_at = 0.
                    self._progress_epoch += 1
                self.publisher.publish(Twist())

    def _global_localization_ready(self, now):
        data = getattr(self, '_global_localization', None) or {}
        return (now-getattr(self, '_global_localization_at', 0.) <= .3
                and data.get('ready') is True and data.get('scope') == 'xingji_3d'
                and abs(self.get_clock().now().nanoseconds-int(data.get('stamp_ns') or 0)) <= 300_000_000
                and data.get('body_generation') == self._body_generation)

    def _tick(self):
        diagnostic=None
        with self._lock:
            now = time.monotonic()
            lio_ready = (self._lio_pause_at is not None
                         and now - self._lio_pause_at <= self.lio_pause_timeout
                         and not self._lio_paused)
            navigation_ready = (self._navigation_pause_at is not None
                                and now - self._navigation_pause_at <= self.lio_pause_timeout
                                and not self._navigation_paused)
            source = select_velocity_source(
                connector_active=self._campus_active,
                emergency=self._emergency or not lio_ready or not navigation_ready,
                teleop_fresh=now - self._teleop_at <= self.timeout,
                nav_fresh=now - self._nav_at <= self.timeout,
                campus_fresh=now - self._campus_at <= self.timeout)
            reason = ('LIO 初始化、临时暂停或状态过期，已清零速度并保持本体姿态'
                      if not lio_ready else '')
            if lio_ready and not navigation_ready:
                reason = '导航恢复尚未完成、等待人工处理或状态过期，已保持零速停车'
            if self._emergency:
                reason = '停车锁已锁定，保持零速；需要人工确认复位，定位恢复不会自动解除'
            if source == 'teleop':
                output = self._latest_teleop
            elif source == 'campus':
                output = self._latest_campus
            elif source == 'nav':
                guard_fresh = (
                    now - self._terrain_guard_at <= self.terrain_timeout)
                body_known = self._body_id in ('xingji', 'gangben_zsl1') and now-self._body_at <= 2.
                forward_only = self._terrain_guard_enabled and self._body_id == 'xingji'
                reason = ''
                if self._terrain_guard_enabled and not body_known:
                    guard_fresh = False
                    reason = '3D导航本体身份无效或过期，已保持停车'
                elif forward_only and self._terrain_evidence not in (
                        'current_anchored', 'recent_measured_anchored', 'restricted_map'):
                    guard_fresh = False
                    reason = '星玑3D等待通过高度校验的地面证据，已保持停车'
                elif forward_only and (self._latest_nav.linear.x < -1e-6
                                       or abs(self._latest_nav.linear.y) > 1e-6):
                    reason = '星玑3D仅有前方地形证据，禁止自动倒退或横移；请停止任务后安全遥控调整'
                x_value, y_value, yaw_value = constrain_navigation_velocity(
                    self._latest_nav.linear.x,
                    self._latest_nav.linear.y,
                    self._latest_nav.angular.z,
                    terrain_safe=self._terrain_safe and guard_fresh,
                    speed_limit_mps=self._terrain_speed_limit,
                    terrain_guard_enabled=self._terrain_guard_enabled,
                    restricted_flat_site=getattr(self, '_restricted_terrain', False),
                    forward_only=forward_only)
                output = Twist()
                output.linear.x = x_value
                output.linear.y = y_value
                output.angular.z = yaw_value
            else:
                output = Twist()
            if (source in ('nav', 'campus') and self._terrain_guard_enabled
                    and self._body_id == 'xingji' and not self._global_localization_ready(now)):
                output = Twist()
                reason = ('三维定位正在限时复核，保持零速等待连续可信匹配'
                          if (self._global_localization or {}).get('recovering') is True else
                          '统一三维定位未确认、过期或失去有效匹配，已零速停车；请检查定位状态')
            # Serialize with pause callbacks: a pre-pause nonzero command
            # must never be enqueued after the callback's zero command.
            self.publisher.publish(output)
            # Diagnostics run only after publishing the chosen output. Neither
            # a telemetry exception nor a logger/publisher failure changes it.
            # Capture the values actually selected above, not another control
            # decision. Diagnostic serialization/logging occurs after publish.
            try:
                diagnostic=dict(stage='arbiter',schema=1,stamp_ns=self.get_clock().now().nanoseconds,
                    global_localization_blocked=(source in ('nav', 'campus')
                        and self._terrain_guard_enabled and self._body_id == 'xingji'
                        and not self._global_localization_ready(now)),
                    global_localization=dict(self._global_localization or {}),
                    global_localization_age_sec=age(now,self._global_localization_at),
                    source=source,navigation_active=self._diag_navigation_active,
                    connector_active=self._campus_active,
                    route=dict(self._diag_route),route_age_sec=age(now,self._diag_route_at),
                    input_topic=self.navigation_input_topic,output_topic=self.output_topic,
                    input=velocity(self._latest_nav),output=velocity(output),
                    input_seen=self._diag_nav_received_at>0,
                    input_age_sec=age(now,self._diag_nav_received_at),
                    command_age_sec=age(now,self._nav_at),
                    input_discarded=self._diag_nav_received_at>0 and self._nav_at<=0,
                    command_timeout_sec=self.timeout,
                    lio_ready=lio_ready,navigation_ready=navigation_ready,emergency=self._emergency,
                    lio_age_sec=age(now,self._lio_pause_at),navigation_pause_age_sec=age(now,self._navigation_pause_at),
                    terrain_enabled=self._terrain_guard_enabled,body_id=self._body_id,
                    body_generation=self._body_generation,body_age_sec=age(now,self._body_at),
                    body_known=self._body_id in ('xingji','gangben_zsl1') and now-self._body_at<=2.,
                    terrain_fresh=now-self._terrain_guard_at<=self.terrain_timeout,
                    terrain_status_age_sec=age(now,self._terrain_guard_at),
                    terrain_safe=self._terrain_safe,evidence=self._terrain_evidence,
                    speed_limit_mps=self._terrain_speed_limit,terrain=dict(self._diag_terrain),
                    controller_stream=self._controller_timing.snapshot(now, self.timeout),
                    observer=dict(self._diag_observer),observer_age_sec=age(now,self._diag_observer_at))
                diagnostic['reason_code']=arbiter_reason(diagnostic)
            except Exception:
                diagnostic=None
            try:
                self._publish_motion_guard(now, source, output, reason, diagnostic)
            except Exception:
                pass
        if diagnostic is not None:
            try:
                self._motion_trace.emit(diagnostic,now=now,active=diagnostic['navigation_active'])
            except Exception:
                pass  # A diagnostic sink can never suppress a command.

    def _publish_motion_guard(self, now, source, output, reason, diagnostic=None):
        status = velocity_guard_status(diagnostic) if diagnostic is not None else {}
        # This expands display reasons only; the command was already published.
        if status.get('blocked'):
            reason = status.get('reason') or reason
        route = self._diag_route
        context = (self._progress_epoch, self._body_generation,
            route.get('route_id'), route.get('point_number'), route.get('loop'),
            route.get('map_name'), route.get('mode'), route.get('state'))
        progress = self._progress_monitor.evaluate(now=now,
            ros_now=self.get_clock().now().nanoseconds/1e9,
            enabled=(self._terrain_guard_enabled and self._body_id == 'xingji'
                and now-self._body_at <= 2. and self._diag_navigation_active
                and source == 'nav' and not reason),
            command=velocity(output), context=context)
        code = progress['warning_code']
        changed = reason != self._guard_reason
        warning_changed = code != self._progress_warning_code
        if changed or warning_changed or now-self._guard_published_at >= .5:
            self._guard_reason = reason
            self._progress_warning_code = code
            self._guard_published_at = now
            self._guard_publisher.publish(String(data=json.dumps({
                **status,
                'stamp_ns': self.get_clock().now().nanoseconds,
                'blocked': bool(reason), 'reason': reason,
                'warning': progress['warning'], 'warning_code': code,
                'warning_reason': progress['reason'] if progress['warning'] else '',
                'motion_progress': progress,
            }, ensure_ascii=False)))
            if changed and reason:
                self.get_logger().warning(reason)
        if code != self._progress_log_code and now-self._progress_log_at >= 1.:
            previous_code = self._progress_log_code
            self._progress_log_code = code
            self._progress_log_at = now
            if code or previous_code:
                record = dict(progress, stamp_ns=self.get_clock().now().nanoseconds,
                    input_topic=self.navigation_input_topic, output_topic=self.output_topic,
                    output=velocity(output), route=dict(route),
                    route_age_sec=age(now,self._diag_route_at))
                if not code:
                    record['reason'] = '运动进展提示已解除；可能已移动、任务结束、保护停车或证据已重置'
                message='NAV_PROGRESS '+json.dumps(record,ensure_ascii=False,allow_nan=False)
                if code:self.get_logger().warning(message)
                else:self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
