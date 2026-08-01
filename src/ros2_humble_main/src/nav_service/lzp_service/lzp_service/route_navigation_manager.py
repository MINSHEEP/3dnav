#!/usr/bin/env python3
"""Persistent single-goal and ordered-route navigation manager."""

from __future__ import annotations

import copy
import json
import math
import threading
import time
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from lzp_interface.msg import (
    CurrentMap,
    RelocalizationRequest,
    Route as RouteMsg,
    RouteExecutionStatus,
    RoutePoint as RoutePointMsg,
)
from lzp_interface.srv import (
    ControlRoute,
    DeleteRoute,
    DeleteRoutesByMap,
    GetRoute,
    GetRouteExecutionState,
    ListRoutes,
    SaveRoute,
    StartRoute,
)

from .route_database import RouteDatabase, RouteNameConflict


class RouteNavigationManager(Node):
    """Own the only web-facing NavigateToPose client and route state machine."""

    VALID_CONTROLS = {'pause', 'resume', 'skip', 'stop'}

    def __init__(self):
        super().__init__('route_navigation_manager')
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        db_path = self.declare_parameter('route_db_path', '').value
        self._max_point_retries = max(
            0, int(self.declare_parameter(
                'route_point_retry_count', 1).value))
        self._settled_xy_tolerance = max(
            0.05, float(self.declare_parameter(
                'settled_xy_tolerance', 0.25).value))
        self._settled_yaw_tolerance = math.radians(max(
            1.0, float(self.declare_parameter(
                'settled_yaw_tolerance_deg', 20.0).value)))
        self._settled_hold_seconds = max(
            0.5, float(self.declare_parameter(
                'settled_hold_seconds', 2.0).value))
        self._lio_auto_reset_delay = max(
            1.0, float(self.declare_parameter(
                'lio_auto_reset_delay_sec', 2.0).value))
        self._lio_reset_service_wait = max(
            0.5, float(self.declare_parameter(
                'lio_reset_service_wait_sec', 3.0).value))
        self._lio_reset_settle_time = max(
            0.5, float(self.declare_parameter(
                'lio_reset_settle_before_reset_sec', 1.0).value))
        self._lio_reinitialize_timeout = max(
            3.0, float(self.declare_parameter(
                'lio_reinitialize_timeout_sec', 10.0).value))
        self._lio_reset_max_attempts = max(
            1, int(self.declare_parameter(
                'lio_reset_max_attempts', 2).value))
        self._auto_relocalization_max_attempts = max(
            1, int(self.declare_parameter(
                'auto_relocalization_max_attempts', 2).value))
        self._auto_relocalization_pose_max_age = max(
            0.5, float(self.declare_parameter(
                'auto_relocalization_pose_max_age_sec', 3.0).value))
        self._auto_relocalization_retry_delay = max(
            0.5, float(self.declare_parameter(
                'auto_relocalization_retry_delay_sec', 1.0).value))
        self._costmap_clear_timeout = max(
            0.5, float(self.declare_parameter(
                'recovery_costmap_clear_timeout_sec', 2.0).value))
        self._database = RouteDatabase(db_path or None)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseStamped,
            '/web_goal_pose',
            self._single_goal_callback,
            volatile_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Empty,
            '/cancel_navigation',
            self._cancel_navigation_callback,
            volatile_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            CurrentMap,
            '/current_map_info',
            self._current_map_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/lio_health',
            self._lio_health_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            amcl_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/relocalization/status',
            self._relocalization_status_callback,
            transient_qos,
            callback_group=self._callback_group,
        )

        self._goal_status_publisher = self.create_publisher(
            String, '/goal_pose_status', transient_qos)
        self._nav2_ready_publisher = self.create_publisher(
            Bool, '/nav2_ready', transient_qos)
        self._navigation_active_publisher = self.create_publisher(
            Bool, '/navigation_active', transient_qos)
        self._route_status_publisher = self.create_publisher(
            RouteExecutionStatus, '/route_execution_status', transient_qos)
        self._route_log_publisher = self.create_publisher(
            String, '/route_navigation_log', volatile_qos)
        self._zero_velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel_nav', 10)
        self._emergency_stop_publisher = self.create_publisher(
            Empty, '/emergency_stop', volatile_qos)
        self._clear_emergency_stop_publisher = self.create_publisher(
            Empty, '/clear_emergency_stop', volatile_qos)
        self._relocalization_request_publisher = self.create_publisher(
            RelocalizationRequest,
            '/relocalization/request',
            volatile_qos,
        )
        self._lio_reset_client = self.create_client(
            Trigger,
            '/lio/reset_estimator',
            callback_group=self._callback_group,
        )
        self._costmap_clear_clients = {
            '全局': self.create_client(
                ClearEntireCostmap,
                '/global_costmap/clear_entirely_global_costmap',
                callback_group=self._callback_group,
            ),
            '局部': self.create_client(
                ClearEntireCostmap,
                '/local_costmap/clear_entirely_local_costmap',
                callback_group=self._callback_group,
            ),
        }

        self.create_service(
            SaveRoute,
            '/save_route',
            self._save_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            ListRoutes,
            '/list_routes',
            self._list_routes_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetRoute,
            '/get_route',
            self._get_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            DeleteRoute,
            '/delete_route',
            self._delete_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            StartRoute,
            '/start_route',
            self._start_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            ControlRoute,
            '/control_route',
            self._control_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetRouteExecutionState,
            '/get_route_execution_state',
            self._get_execution_state_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            DeleteRoutesByMap,
            '/delete_routes_by_map',
            self._delete_routes_by_map_callback,
            callback_group=self._callback_group,
        )

        self._current_map_name = ''
        self._mode = 'idle'
        self._state = 'stopped'
        self._status_message = 'Execution state reset after manager start'
        self._route: Optional[Dict] = None
        self._point_index = -1
        self._current_loop = 0
        self._loop_count = 1
        self._continuous = False
        self._dwell_deadline: Optional[float] = None
        self._paused_dwell_remaining = 0.0

        self._sending = False
        self._goal_token = 0
        self._goal_context: Optional[str] = None
        self._active_goal_handle = None
        self._active_pose: Optional[PoseStamped] = None
        self._cancel_reason: Optional[str] = None
        self._cancel_in_progress = False
        self._pending_single_pose: Optional[PoseStamped] = None
        self._nav2_ready = False
        self._last_goal_status = ''
        self._lio_recovering = False
        self._retry_current_after_recovery = False
        self._point_failure_retries = 0
        self._settled_since: Optional[float] = None
        self._recovery_pose: Optional[PoseStamped] = None
        self._latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self._latest_amcl_received_at: Optional[float] = None
        self._recovery_rough_pose: Optional[
            PoseWithCovarianceStamped] = None
        self._lio_recovery_started_at: Optional[float] = None
        self._auto_recovery_stage = 'idle'
        self._lio_reset_future = None
        self._lio_reset_wait_started_at: Optional[float] = None
        self._lio_reset_not_before: Optional[float] = None
        self._lio_reset_attempts = 0
        self._auto_relocalization_request_id = ''
        self._auto_relocalization_attempts = 0
        self._auto_relocalization_retry_at: Optional[float] = None
        self._costmap_clear_futures = {}
        self._costmap_clear_deadline: Optional[float] = None

        self.create_timer(
            0.2, self._timer_callback, callback_group=self._callback_group)
        self._publish_goal_status('ready')
        self._publish_all_status()
        self.get_logger().info(
            f'Route navigation manager started; database={self._database.db_path}')

    @staticmethod
    def _finite(value, name: str) -> float:
        converted = float(value)
        if not math.isfinite(converted):
            raise ValueError(f'{name} must be finite')
        return converted

    @classmethod
    def _validate_route(cls, route: Dict) -> Dict:
        map_name = str(route.get('map_name', '')).strip()
        name = str(route.get('name', '')).strip()
        if not map_name:
            raise ValueError('No current map is available')
        if not name or len(name) > 128:
            raise ValueError('Route name must contain 1 to 128 characters')
        points = list(route.get('points') or [])
        if not points:
            raise ValueError('A route must contain at least one target point')
        if len(points) > 1000:
            raise ValueError('A route cannot contain more than 1000 points')

        validated_points = []
        for index, point in enumerate(points):
            point_name = str(point.get('name', '')).strip()
            if not point_name:
                point_name = f'目标点 {index + 1}'
            if len(point_name) > 128:
                raise ValueError(
                    f'Point {index + 1} name exceeds 128 characters')
            dwell = cls._finite(
                point.get('dwell_seconds', 0.0),
                f'Point {index + 1} dwell time',
            )
            if dwell < 0.0 or dwell > 86400.0:
                raise ValueError(
                    f'Point {index + 1} dwell time must be in [0, 86400]')
            validated_points.append({
                'id': int(point.get('id', 0)),
                'sequence': index,
                'name': point_name,
                'x': cls._finite(point.get('x'), f'Point {index + 1} x'),
                'y': cls._finite(point.get('y'), f'Point {index + 1} y'),
                'yaw': cls._finite(
                    point.get('yaw'), f'Point {index + 1} yaw'),
                'dwell_seconds': dwell,
            })
        return {
            'id': int(route.get('id', 0)),
            'map_name': map_name,
            'name': name,
            'points': validated_points,
        }

    @staticmethod
    def _route_point_to_message(point: Dict) -> RoutePointMsg:
        message = RoutePointMsg()
        message.id = int(point.get('id', 0))
        message.sequence = int(point.get('sequence', 0))
        message.name = str(point.get('name', ''))
        message.x = float(point.get('x', 0.0))
        message.y = float(point.get('y', 0.0))
        message.yaw = float(point.get('yaw', 0.0))
        message.dwell_seconds = float(point.get('dwell_seconds', 0.0))
        return message

    @classmethod
    def _route_to_message(cls, route: Dict) -> RouteMsg:
        message = RouteMsg()
        message.id = int(route.get('id', 0))
        message.map_name = str(route.get('map_name', ''))
        message.name = str(route.get('name', ''))
        message.points = [
            cls._route_point_to_message(point)
            for point in route.get('points', [])
        ]
        return message

    @staticmethod
    def _message_to_route(message: RouteMsg) -> Dict:
        return {
            'id': int(message.id),
            'map_name': str(message.map_name),
            'name': str(message.name),
            'points': [
                {
                    'id': int(point.id),
                    'sequence': int(point.sequence),
                    'name': str(point.name),
                    'x': float(point.x),
                    'y': float(point.y),
                    'yaw': float(point.yaw),
                    'dwell_seconds': float(point.dwell_seconds),
                }
                for point in message.points
            ],
        }

    def _current_point(self) -> Optional[Dict]:
        if (
            self._route is None
            or self._point_index < 0
            or self._point_index >= len(self._route['points'])
        ):
            return None
        return self._route['points'][self._point_index]

    def _dwell_remaining(self) -> float:
        if self._state == 'paused_dwell':
            return max(0.0, self._paused_dwell_remaining)
        if self._state == 'dwelling' and self._dwell_deadline is not None:
            return max(0.0, self._dwell_deadline - time.monotonic())
        return 0.0

    def _status_message_object(self) -> RouteExecutionStatus:
        message = RouteExecutionStatus()
        message.mode = self._mode
        message.state = self._state
        message.active = self._mode != 'idle'
        message.paused = self._state in (
            'paused_navigation', 'paused_dwell')
        message.route_id = int(self._route['id']) if self._route else 0
        message.route_name = str(self._route['name']) if self._route else ''
        message.map_name = (
            str(self._route['map_name'])
            if self._route else self._current_map_name
        )
        message.point_index = int(self._point_index)
        message.point_count = (
            len(self._route['points']) if self._route else 0)
        message.current_loop = int(self._current_loop)
        message.loop_count = int(self._loop_count)
        message.continuous = bool(self._continuous)
        message.dwell_remaining = self._dwell_remaining()
        point = self._current_point()
        if point is not None:
            message.current_point = self._route_point_to_message(point)
        message.message = self._status_message
        return message

    def _publish_goal_status(self, status: str) -> None:
        if status == self._last_goal_status:
            return
        self._last_goal_status = status
        self._goal_status_publisher.publish(String(data=status))

    def _publish_route_log(self, text: str) -> None:
        self._route_log_publisher.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_all_status(self) -> None:
        self._nav2_ready_publisher.publish(Bool(data=self._nav2_ready))
        self._navigation_active_publisher.publish(
            Bool(data=self._mode != 'idle'))
        self._route_status_publisher.publish(self._status_message_object())

    def _timer_callback(self) -> None:
        with self._lock:
            ready = self._action_client.server_is_ready()
            if ready != self._nav2_ready:
                self._nav2_ready = ready
                self._publish_goal_status(
                    'nav2_ready' if ready else 'waiting_for_nav2')

            if (
                self._mode == 'route'
                and self._state == 'dwelling'
                and self._dwell_deadline is not None
                and time.monotonic() >= self._dwell_deadline
            ):
                self._dwell_deadline = None
                self._advance_route()
            elif (
                self._mode == 'route'
                and self._state == 'waiting_nav2'
                and ready
                and not self._sending
                and self._active_goal_handle is None
            ):
                self._dispatch_current_route_point()
            elif (
                self._mode == 'single'
                and self._state == 'waiting_nav2'
                and ready
                and self._pending_single_pose is not None
            ):
                pose = self._pending_single_pose
                self._pending_single_pose = None
                self._send_pose(pose, 'single')

            self._drive_auto_recovery()
            self._publish_all_status()

    def _amcl_pose_callback(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        with self._lock:
            if self._lio_recovering:
                return
            try:
                values = (
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    message.pose.pose.orientation.x,
                    message.pose.pose.orientation.y,
                    message.pose.pose.orientation.z,
                    message.pose.pose.orientation.w,
                )
                if not all(math.isfinite(float(value)) for value in values):
                    return
            except (AttributeError, TypeError, ValueError):
                return
            self._latest_amcl_pose = copy.deepcopy(message)
            self._latest_amcl_received_at = time.monotonic()

    def _capture_recovery_rough_pose(self) -> None:
        if (
            self._latest_amcl_pose is None
            or self._latest_amcl_received_at is None
            or time.monotonic() - self._latest_amcl_received_at
            > self._auto_relocalization_pose_max_age
        ):
            self._recovery_rough_pose = None
            return
        self._recovery_rough_pose = copy.deepcopy(self._latest_amcl_pose)
        self._recovery_rough_pose.header.frame_id = 'map'

    def _drive_auto_recovery(self) -> None:
        if not self._lio_recovering:
            return
        if (
            self._mode not in ('route', 'single')
            and self._auto_recovery_stage in ('idle', 'monitoring')
        ):
            return
        now = time.monotonic()
        if self._auto_recovery_stage == 'clearing_costmaps':
            self._drive_recovery_costmap_clear(now)
            return
        if self._auto_recovery_stage == 'retrying_relocalization':
            if (
                self._auto_relocalization_retry_at is not None
                and now >= self._auto_relocalization_retry_at
            ):
                self._trigger_auto_relocalization()
            return
        if (
            self._auto_recovery_stage == 'monitoring'
            and self._lio_recovery_started_at is not None
            and now - self._lio_recovery_started_at
            >= self._lio_auto_reset_delay
        ):
            self._begin_lio_reset()
            return
        if self._auto_recovery_stage == 'waiting_lio_reinitialize':
            if (
                self._lio_reset_wait_started_at is not None
                and now - self._lio_reset_wait_started_at
                >= self._lio_reinitialize_timeout
            ):
                if self._lio_reset_attempts < self._lio_reset_max_attempts:
                    if self._mode == 'route':
                        self._publish_route_log(
                            'LIO 重置后未在限定时间内恢复健康输出，'
                            '正在执行第二次进程内重置；路径与当前目标点继续保留。')
                    self._begin_lio_reset(force=True)
                else:
                    self._fail_auto_recovery(
                        'LIO 连续两次进程内重置后仍未恢复健康输出，'
                        '已保持安全停车。')
            return
        if self._auto_recovery_stage != 'resetting':
            return
        if (
            self._lio_reset_not_before is not None
            and now < self._lio_reset_not_before
        ):
            return
        if self._lio_reset_wait_started_at is None:
            self._lio_reset_wait_started_at = now
        if self._lio_reset_future is not None:
            if (
                self._lio_reset_wait_started_at is not None
                and now - self._lio_reset_wait_started_at
                > self._lio_reset_service_wait
            ):
                self._fail_auto_recovery(
                    'LIO 重置服务响应超时，已保持安全停车。')
            return
        if self._lio_reset_client.service_is_ready():
            self._lio_reset_wait_started_at = now
            future = self._lio_reset_client.call_async(Trigger.Request())
            self._lio_reset_future = future
            future.add_done_callback(self._lio_reset_response_callback)
            return
        if (
            self._lio_reset_wait_started_at is not None
            and now - self._lio_reset_wait_started_at
            > self._lio_reset_service_wait
        ):
            self._fail_auto_recovery(
                '未找到 /lio/reset_estimator 服务，已保持安全停车。')

    def _begin_lio_reset(self, force: bool = False) -> None:
        if (
            self._auto_recovery_stage != 'monitoring'
            and not (
                force
                and self._auto_recovery_stage
                == 'waiting_lio_reinitialize'
            )
        ):
            return
        self._auto_recovery_stage = 'resetting'
        self._lio_reset_attempts += 1
        now = time.monotonic()
        self._lio_reset_wait_started_at = None
        self._lio_reset_not_before = now + self._lio_reset_settle_time
        self._emergency_stop_publisher.publish(Empty())
        self._send_zero_velocity()
        self._status_message = (
            'Persistent LIO mismatch; resetting estimator in-process')
        if self._mode == 'route':
            self._publish_route_log(
                'LIO 连续恢复超过安全时间窗，正在保持路径任务并执行'
                '进程内估计器重置；无需刷新页面。')
        if self._active_goal_handle is not None or self._sending:
            self._cancel_reason = 'lio_recovery'
            self._cancel_active_goal()

    def _lio_reset_response_callback(self, future) -> None:
        with self._lock:
            if future is not self._lio_reset_future:
                return
            self._lio_reset_future = None
            try:
                response = future.result()
            except Exception as error:
                self._fail_auto_recovery(
                    f'LIO 估计器自动重置调用失败：{error}')
                return
            if not response.success:
                self._fail_auto_recovery(
                    f'LIO 估计器拒绝自动重置：{response.message}')
                return
            self._auto_recovery_stage = 'waiting_lio_reinitialize'
            self._lio_reset_wait_started_at = time.monotonic()
            self._lio_reset_not_before = None
            self._status_message = (
                'LIO estimator reset; waiting for fresh initialization')
            if self._mode == 'route':
                self._publish_route_log(
                    'LIO 估计器已在当前进程内重置，正在利用持续输入的'
                    '雷达和IMU重新初始化。')
            self._publish_all_status()

    def _trigger_auto_relocalization(self) -> None:
        if self._auto_recovery_stage == 'relocalizing':
            return
        if not self._current_map_name:
            self._fail_auto_recovery(
                '当前地图名称不可用，无法执行自动三维重定位。')
            return
        if self._recovery_rough_pose is None:
            self._fail_auto_recovery(
                '异常前没有足够新的AMCL可信位姿，无法安全自动重定位。')
            return
        self._auto_relocalization_attempts += 1
        self._auto_relocalization_retry_at = None
        request = RelocalizationRequest()
        request.rough_pose = copy.deepcopy(self._recovery_rough_pose)
        request.rough_pose.header.frame_id = 'map'
        request.rough_pose.header.stamp = self.get_clock().now().to_msg()
        request.map_name = self._current_map_name
        request.request_id = (
            f'auto_lio_{int(time.time() * 1000)}_'
            f'{self._auto_relocalization_attempts}')
        self._auto_relocalization_request_id = request.request_id
        self._auto_recovery_stage = 'relocalizing'
        self._status_message = 'Running automatic 3-D relocalization'
        self._relocalization_request_publisher.publish(request)
        if self._mode == 'route':
            self._publish_route_log(
                f'开始第 {self._auto_relocalization_attempts} 次自动三维'
                '重定位；成功并经AMCL连续验证后，将重试当前目标点。')

    def _relocalization_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            if (
                self._auto_recovery_stage != 'relocalizing'
                or str(status.get('request_id', ''))
                != self._auto_relocalization_request_id
                or not bool(status.get('terminal', False))
            ):
                return
            if bool(status.get('success', False)):
                self._complete_auto_recovery()
                return
            message_text = str(
                status.get('message', '自动三维重定位失败'))
            if (
                self._auto_relocalization_attempts
                < self._auto_relocalization_max_attempts
            ):
                self._auto_recovery_stage = 'retrying_relocalization'
                self._auto_relocalization_retry_at = (
                    time.monotonic()
                    + self._auto_relocalization_retry_delay
                )
                if self._mode == 'route':
                    self._publish_route_log(
                        f'自动三维重定位未通过（{message_text}），'
                        '将等待新采集点云后再尝试一次。')
                return
            self._fail_auto_recovery(
                f'自动三维重定位连续失败：{message_text}')

    def _complete_auto_recovery(self) -> None:
        self._auto_recovery_stage = 'clearing_costmaps'
        self._costmap_clear_deadline = (
            time.monotonic() + self._costmap_clear_timeout
        )
        self._costmap_clear_futures = {
            name: None for name in self._costmap_clear_clients
        }
        self._status_message = (
            'Relocalization passed; clearing stale Nav2 costmaps')
        if self._mode == 'route':
            self._publish_route_log(
                '三维重定位与AMCL连续验证均已通过，正在清理 LIO 重置前的'
                '局部与全局代价地图。')
        self._drive_recovery_costmap_clear(time.monotonic())
        self._publish_all_status()

    def _drive_recovery_costmap_clear(self, now: float) -> None:
        if self._auto_recovery_stage != 'clearing_costmaps':
            return
        for name, client in self._costmap_clear_clients.items():
            if (
                self._costmap_clear_futures.get(name) is None
                and client.service_is_ready()
            ):
                self._costmap_clear_futures[name] = client.call_async(
                    ClearEntireCostmap.Request())

        all_finished = bool(self._costmap_clear_futures) and all(
            future is not None and future.done()
            for future in self._costmap_clear_futures.values()
        )
        timed_out = (
            self._costmap_clear_deadline is not None
            and now >= self._costmap_clear_deadline
        )
        if not all_finished and not timed_out:
            return

        failures = []
        for name, future in self._costmap_clear_futures.items():
            if future is None or not future.done():
                failures.append(f'{name}代价地图服务不可用')
                continue
            try:
                future.result()
            except Exception as error:
                failures.append(f'{name}代价地图清理失败：{error}')
        if failures and self._mode == 'route':
            self._publish_route_log(
                f'代价地图自动清理未全部确认（{"；".join(failures)}），'
                '将依靠 Nav2 的实时更新继续恢复。')
        self._resume_after_auto_recovery()

    def _resume_after_auto_recovery(self) -> None:
        self._lio_recovering = False
        self._auto_recovery_stage = 'idle'
        self._lio_recovery_started_at = None
        self._lio_reset_future = None
        self._lio_reset_wait_started_at = None
        self._lio_reset_not_before = None
        self._lio_reset_attempts = 0
        self._auto_relocalization_request_id = ''
        self._auto_relocalization_attempts = 0
        self._auto_relocalization_retry_at = None
        self._recovery_rough_pose = None
        self._costmap_clear_futures = {}
        self._costmap_clear_deadline = None
        self._clear_emergency_stop_publisher.publish(Empty())
        if self._mode == 'route':
            self._publish_route_log(
                '自动恢复已完成；路径仍保持用户设置的暂停状态。'
                if self._state in ('paused_navigation', 'paused_dwell')
                else '自动恢复闭环已完成，正在重试当前目标点。')
        if self._state in ('paused_navigation', 'paused_dwell'):
            self._status_message = (
                'Automatic recovery completed; route remains paused')
        elif self._active_goal_handle is not None:
            self._state = 'navigating'
            self._status_message = 'Navigation resumed after relocalization'
        elif self._sending:
            self._state = 'sending'
        elif self._retry_current_after_recovery:
            self._retry_current_after_recovery = False
            if self._mode == 'route':
                self._dispatch_current_route_point()
            elif self._recovery_pose is not None:
                pose = self._recovery_pose
                self._recovery_pose = None
                self._send_pose(copy.deepcopy(pose), 'single')
        elif self._mode == 'route':
            self._dispatch_current_route_point()
        self._publish_all_status()

    def _fail_auto_recovery(self, message: str) -> None:
        if self._auto_recovery_stage == 'failed':
            return
        self._auto_recovery_stage = 'failed'
        self._lio_reset_future = None
        self._lio_reset_wait_started_at = None
        self._lio_reset_not_before = None
        self._auto_relocalization_retry_at = None
        self._costmap_clear_futures = {}
        self._costmap_clear_deadline = None
        self._emergency_stop_publisher.publish(Empty())
        self._send_zero_velocity()
        self._state = 'recovery_failed'
        self._status_message = message
        if self._mode == 'route':
            self._publish_route_log(
                f'{message} 路径与当前目标点仍保留，禁止使用不可信位姿继续运动。')
        self._publish_all_status()

    def _lio_health_callback(self, message: String) -> None:
        """Keep the active task while LIO rolls back and validates weak scans."""
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            status = {'state': str(message.data or '')}
        state = str(status.get('state', '')).lower()
        temporary_stop = bool(status.get('temporary_stop', False))

        with self._lock:
            active_navigation = self._mode in ('route', 'single')
            recovering = (
                state in ('recovering', 'resetting')
                or (state == 'warning' and temporary_stop)
                or state == 'fault'
            )
            if recovering:
                first_notice = not self._lio_recovering
                self._lio_recovering = True
                if first_notice:
                    now = time.monotonic()
                    self._lio_recovery_started_at = now
                    externally_resetting = state == 'resetting'
                    self._auto_recovery_stage = (
                        'waiting_lio_reinitialize'
                        if externally_resetting else 'monitoring'
                    )
                    self._lio_reset_attempts = (
                        1 if externally_resetting else 0
                    )
                    self._lio_reset_wait_started_at = (
                        now if externally_resetting else None
                    )
                    self._auto_relocalization_attempts = 0
                    self._auto_relocalization_request_id = ''
                    self._auto_relocalization_retry_at = None
                    self._capture_recovery_rough_pose()
                if active_navigation and self._state not in (
                    'stopping', 'paused_navigation', 'paused_dwell'
                ):
                    self._state = (
                        'waiting_lio_recovery'
                        if self._active_goal_handle is None and not self._sending
                        else 'recovering_lio'
                    )
                    self._status_message = (
                        'LIO is validating healthy scans; current task retained')
                    if first_notice and self._mode == 'route':
                        self._publish_route_log(
                            'LIO 正在自动恢复：当前路径和目标点已保留，'
                            '恢复后将继续当前目标点。')
                self._publish_all_status()
                return

            if state != 'healthy' or not self._lio_recovering:
                return

            if self._auto_recovery_stage in (
                'resetting', 'waiting_lio_reinitialize'
            ):
                if self._auto_recovery_stage == 'waiting_lio_reinitialize':
                    self._trigger_auto_relocalization()
                    self._publish_all_status()
                return
            if self._auto_recovery_stage in (
                'relocalizing', 'retrying_relocalization',
                'clearing_costmaps', 'failed'
            ):
                return

            self._lio_recovering = False
            self._auto_recovery_stage = 'idle'
            self._lio_recovery_started_at = None
            self._lio_reset_wait_started_at = None
            self._lio_reset_not_before = None
            self._lio_reset_attempts = 0
            self._auto_relocalization_attempts = 0
            self._auto_relocalization_request_id = ''
            self._auto_relocalization_retry_at = None
            self._recovery_rough_pose = None
            if not active_navigation:
                self._retry_current_after_recovery = False
                return
            if self._state in (
                'paused_navigation', 'paused_dwell', 'pausing', 'dwelling'
            ):
                self._publish_all_status()
                return
            if self._mode == 'route':
                self._publish_route_log(
                    'LIO 连续健康扫描验证通过，路径任务自动继续。')
            if self._active_goal_handle is not None:
                self._state = 'navigating'
                self._status_message = 'Navigation resumed after LIO recovery'
            elif self._sending:
                self._state = 'sending'
            elif self._retry_current_after_recovery:
                self._retry_current_after_recovery = False
                if self._mode == 'route':
                    self._dispatch_current_route_point()
                elif self._recovery_pose is not None:
                    pose = self._recovery_pose
                    self._recovery_pose = None
                    self._send_pose(copy.deepcopy(pose), 'single')
            elif self._mode == 'route':
                self._dispatch_current_route_point()
            self._publish_all_status()

    def _current_map_callback(self, message: CurrentMap) -> None:
        with self._lock:
            previous = self._current_map_name
            if message.success and message.map_name:
                self._current_map_name = str(message.map_name)
            if (
                previous
                and self._current_map_name != previous
                and self._mode == 'route'
            ):
                self._publish_route_log(
                    '当前地图在路径执行期间发生变化，已停止路径导航')
                self._request_stop('map_changed')

    def _save_route_callback(self, request, response):
        with self._lock:
            try:
                route = self._message_to_route(request.route)
                route['map_name'] = self._current_map_name
                route = self._validate_route(route)
                if route['id'] > 0:
                    existing = self._database.get_route(route['id'])
                    if (
                        existing is None
                        or existing['map_name'] != self._current_map_name
                    ):
                        raise ValueError(
                            'Route does not belong to the current map')
                if (
                    self._mode == 'route'
                    and self._route is not None
                    and route['id'] == self._route['id']
                ):
                    raise ValueError('The active route cannot be modified')
                route_id = self._database.save_route(route)
                response.success = True
                response.route_id = route_id
                response.message = f'Route saved: {route["name"]}'
            except RouteNameConflict:
                response.success = False
                response.route_id = 0
                response.message = '当前地图中已经存在同名路径，请修改路径名称'
            except Exception as error:
                response.success = False
                response.route_id = 0
                response.message = str(error)
        return response

    def _list_routes_callback(self, request, response):
        with self._lock:
            requested_map = str(request.map_name).strip()
            map_name = requested_map or self._current_map_name
            if not self._current_map_name:
                response.success = False
                response.message = 'No current map is available'
                response.routes = []
            elif map_name != self._current_map_name:
                response.success = False
                response.message = 'Routes may only be listed for the current map'
                response.routes = []
            else:
                routes = self._database.list_routes(map_name)
                response.success = True
                response.message = f'Loaded {len(routes)} routes'
                response.routes = [
                    self._route_to_message(route) for route in routes]
        return response

    def _get_route_callback(self, request, response):
        with self._lock:
            route = self._database.get_route(request.route_id)
            if route is None:
                response.success = False
                response.message = 'Route not found'
            elif route['map_name'] != self._current_map_name:
                response.success = False
                response.message = 'Route does not belong to the current map'
            else:
                response.success = True
                response.message = 'Route loaded'
                response.route = self._route_to_message(route)
        return response

    def _delete_route_callback(self, request, response):
        with self._lock:
            route_id = int(request.route_id)
            if (
                self._mode == 'route'
                and self._route is not None
                and route_id == self._route['id']
            ):
                response.success = False
                response.message = 'The active route cannot be deleted'
                return response
            route = self._database.get_route(route_id)
            if route is None or route['map_name'] != self._current_map_name:
                response.success = False
                response.message = 'Route not found on the current map'
                return response
            response.success = self._database.delete_route(route_id)
            response.message = (
                'Route deleted' if response.success else 'Route not found')
        return response

    def _delete_routes_by_map_callback(self, request, response):
        with self._lock:
            map_name = str(request.map_name).strip()
            if (
                self._mode == 'route'
                and self._route is not None
                and self._route['map_name'] == map_name
            ):
                response.success = False
                response.deleted_count = 0
                response.message = 'Cannot delete routes for the active map task'
                return response
            count = self._database.delete_routes_by_map(map_name)
            response.success = True
            response.deleted_count = count
            response.message = f'Deleted {count} routes for map {map_name}'
        return response

    def _start_route_callback(self, request, response):
        with self._lock:
            if self._mode != 'idle':
                response.success = False
                response.message = 'Another navigation task is already active'
                return response
            if self._lio_recovering:
                response.success = False
                response.message = (
                    'LIO is recovering; wait for healthy scan validation')
                return response
            route = self._database.get_route(request.route_id)
            if route is None:
                response.success = False
                response.message = 'Route not found'
                return response
            if route['map_name'] != self._current_map_name:
                response.success = False
                response.message = 'Route does not belong to the current map'
                return response
            if not route['points']:
                response.success = False
                response.message = 'Route has no target points'
                return response
            loop_count = int(request.loop_count)
            if not request.continuous and loop_count < 1:
                response.success = False
                response.message = 'Loop count must be at least 1'
                return response
            if not self._action_client.server_is_ready():
                response.success = False
                response.message = 'Nav2 is not ready'
                return response

            self._mode = 'route'
            self._state = 'starting'
            self._status_message = 'Starting route'
            self._route = route
            self._point_index = 0
            self._current_loop = 1
            self._loop_count = max(1, loop_count)
            self._continuous = bool(request.continuous)
            self._dwell_deadline = None
            self._paused_dwell_remaining = 0.0
            self._cancel_reason = None
            self._pending_single_pose = None
            self._lio_recovering = False
            self._retry_current_after_recovery = False
            self._point_failure_retries = 0
            self._settled_since = None
            self._recovery_pose = None
            loop_text = (
                '持续循环'
                if self._continuous else f'循环 {self._loop_count} 次')
            self._route_log_publisher.publish(
                String(data='__ROUTE_LOG_CLEAR__'))
            self._publish_route_log(
                f'开始路径“{route["name"]}”，{loop_text}')
            self._dispatch_current_route_point()
            response.success = True
            response.message = 'Route navigation started'
            self._publish_all_status()
        return response

    def _control_route_callback(self, request, response):
        with self._lock:
            command = str(request.command).strip().lower()
            if command not in self.VALID_CONTROLS:
                response.success = False
                response.message = f'Unsupported command: {command}'
                return response
            if self._mode != 'route':
                response.success = False
                response.message = 'No route navigation task is active'
                return response

            handlers = {
                'pause': self._pause_route,
                'resume': self._resume_route,
                'skip': self._skip_route_point,
                'stop': lambda: self._request_stop('user_stop'),
            }
            success, message = handlers[command]()
            response.success = success
            response.message = message
            self._publish_all_status()
        return response

    def _get_execution_state_callback(self, _request, response):
        with self._lock:
            response.status = self._status_message_object()
        return response

    def _single_goal_callback(self, pose: PoseStamped) -> None:
        with self._lock:
            if self._lio_recovering:
                self._publish_goal_status('waiting_for_lio_recovery')
                return
            if self._mode == 'route':
                self._publish_goal_status('goal_rejected_route_active')
                self._publish_route_log(
                    '路径执行期间拒绝了新的单目标导航请求')
                return
            if not pose.header.frame_id:
                pose.header.frame_id = 'map'
            pose = copy.deepcopy(pose)

            if self._mode == 'single':
                self._pending_single_pose = pose
                if self._active_goal_handle is not None or self._sending:
                    self._cancel_reason = 'replace_single'
                    self._state = 'replacing'
                    self._cancel_active_goal()
                else:
                    self._send_pose(pose, 'single')
                return

            self._mode = 'single'
            self._state = 'waiting_nav2'
            self._status_message = 'Single navigation goal queued'
            self._pending_single_pose = pose
            self._publish_goal_status('goal_queued')
            if self._action_client.server_is_ready():
                self._pending_single_pose = None
                self._send_pose(pose, 'single')
            self._publish_all_status()

    def _cancel_navigation_callback(self, _message: Empty) -> None:
        with self._lock:
            if self._mode == 'idle':
                self._publish_goal_status('goal_canceled')
                return
            if self._mode == 'route':
                self._publish_route_log('收到导航取消/急停请求，正在停止路径')
            self._request_stop('external_cancel')

    def _route_pose(self, point: Dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        pose.pose.position.z = 0.0
        half_yaw = float(point['yaw']) * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _dispatch_current_route_point(self) -> None:
        point = self._current_point()
        if self._mode != 'route' or point is None:
            return
        if self._lio_recovering:
            self._state = 'waiting_lio_recovery'
            self._status_message = (
                f'Current point retained during LIO recovery: {point["name"]}')
            return
        if not self._action_client.server_is_ready():
            self._state = 'waiting_nav2'
            self._status_message = 'Waiting for Nav2'
            return
        self._state = 'sending'
        self._status_message = (
            f'Navigating to point {self._point_index + 1}: {point["name"]}')
        self._publish_route_log(
            f'第 {self._current_loop} 轮：前往目标点 '
            f'{self._point_index + 1}/{len(self._route["points"])}'
            f'“{point["name"]}”')
        self._send_pose(self._route_pose(point), 'route')

    def _send_pose(self, pose: PoseStamped, context: str) -> None:
        if self._sending or self._active_goal_handle is not None:
            return
        if not self._action_client.server_is_ready():
            self._state = 'waiting_nav2'
            if context == 'single':
                self._pending_single_pose = pose
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._sending = True
        self._goal_context = context
        self._active_pose = copy.deepcopy(pose)
        self._settled_since = None
        self._goal_token += 1
        token = self._goal_token
        self._state = 'sending'
        if context == 'single':
            self._publish_goal_status('sending_goal')
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, current_token=token:
            self._goal_feedback_callback(feedback, current_token),
        )
        future.add_done_callback(
            lambda result, current_token=token:
            self._goal_response_callback(result, current_token))

    def _goal_response_callback(self, future, token: int) -> None:
        with self._lock:
            if token != self._goal_token:
                try:
                    late_handle = future.result()
                    if late_handle.accepted:
                        late_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self._sending = False
            try:
                goal_handle = future.result()
            except Exception as error:
                self._handle_goal_terminal(
                    False, f'发送目标失败：{error}')
                return
            if not goal_handle.accepted:
                self._handle_goal_terminal(False, 'Nav2拒绝目标')
                return

            self._active_goal_handle = goal_handle
            self._state = (
                'recovering_lio' if self._lio_recovering else 'navigating')
            if self._goal_context == 'single':
                self._publish_goal_status('navigating')
            self._status_message = 'Navigation goal accepted'
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result, handle=goal_handle:
                self._goal_result_callback(result, handle))
            if self._cancel_reason is not None:
                self._cancel_active_goal()
            self._publish_all_status()

    @staticmethod
    def _pose_yaw(pose) -> float:
        orientation = pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z),
        )

    def _goal_feedback_callback(self, feedback_message, token: int) -> None:
        """Accept a stable, safe near-goal pose after precise Nav2 settling."""
        with self._lock:
            if (
                token != self._goal_token
                or self._active_goal_handle is None
                or self._active_pose is None
                or self._cancel_reason is not None
                or self._lio_recovering
            ):
                self._settled_since = None
                return
            try:
                current = feedback_message.feedback.current_pose.pose
                target = self._active_pose.pose
                xy_error = math.hypot(
                    target.position.x - current.position.x,
                    target.position.y - current.position.y,
                )
                yaw_error = abs(math.atan2(
                    math.sin(self._pose_yaw(target) - self._pose_yaw(current)),
                    math.cos(self._pose_yaw(target) - self._pose_yaw(current)),
                ))
            except (AttributeError, TypeError, ValueError):
                self._settled_since = None
                return

            if (
                xy_error > self._settled_xy_tolerance
                or yaw_error > self._settled_yaw_tolerance
            ):
                self._settled_since = None
                return

            now = time.monotonic()
            if self._settled_since is None:
                self._settled_since = now
                return
            if now - self._settled_since < self._settled_hold_seconds:
                return

            self._cancel_reason = 'settled_goal'
            self._state = 'settling'
            self._status_message = (
                f'Goal stable within {xy_error:.2f} m and '
                f'{math.degrees(yaw_error):.1f} deg')
            self._cancel_active_goal()

    def _goal_result_callback(self, future, goal_handle) -> None:
        with self._lock:
            if goal_handle is not self._active_goal_handle:
                return
            try:
                status = future.result().status
            except Exception as error:
                status = GoalStatus.STATUS_UNKNOWN
                self.get_logger().error(
                    f'Unable to read navigation result: {error}')
            self._active_goal_handle = None
            self._cancel_in_progress = False
            succeeded = status == GoalStatus.STATUS_SUCCEEDED
            status_text = {
                GoalStatus.STATUS_SUCCEEDED: '已到达',
                GoalStatus.STATUS_ABORTED: 'Nav2已中止',
                GoalStatus.STATUS_CANCELED: '目标已取消',
                GoalStatus.STATUS_UNKNOWN: '目标结果未知',
            }.get(status, f'目标状态 {status}')
            self._handle_goal_terminal(succeeded, status_text)

    def _cancel_active_goal(self) -> None:
        if self._active_goal_handle is None or self._cancel_in_progress:
            return
        self._cancel_in_progress = True
        future = self._active_goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future) -> None:
        with self._lock:
            self._cancel_in_progress = False
            try:
                accepted = bool(future.result().goals_canceling)
            except Exception as error:
                accepted = False
                self.get_logger().warn(f'Cancel request failed: {error}')
            if not accepted and self._active_goal_handle is not None:
                self._status_message = 'Nav2 did not accept cancellation'
                if self._mode == 'route':
                    self._publish_route_log('Nav2未确认取消当前目标')
            self._publish_all_status()

    def _handle_goal_terminal(self, succeeded: bool, detail: str) -> None:
        context = self._goal_context
        cancel_reason = self._cancel_reason
        recovery_pose = (
            copy.deepcopy(self._active_pose)
            if self._active_pose is not None else None)
        self._goal_context = None
        self._active_pose = None
        self._settled_since = None
        self._cancel_reason = None

        if context == 'single':
            if cancel_reason == 'replace_single' and self._pending_single_pose:
                pose = self._pending_single_pose
                self._pending_single_pose = None
                self._send_pose(pose, 'single')
                return
            if cancel_reason in ('user_stop', 'external_cancel', 'map_changed'):
                self._publish_goal_status('goal_canceled')
                self._finish_task('idle', detail)
                return
            if (
                not succeeded
                and (
                    self._lio_recovering
                    or cancel_reason == 'lio_recovery'
                )
            ):
                self._retry_current_after_recovery = True
                self._recovery_pose = recovery_pose
                self._state = 'waiting_lio_recovery'
                self._status_message = (
                    'Single goal retained until LIO recovery completes')
                self._publish_goal_status('waiting_for_lio_recovery')
                self._publish_all_status()
                return
            if cancel_reason == 'settled_goal':
                self._publish_goal_status('goal_succeeded')
                self._finish_task('idle', 'Goal reached within stable tolerance')
                return
            if cancel_reason is not None:
                self._publish_goal_status('goal_canceled')
            elif succeeded:
                self._publish_goal_status('goal_succeeded')
            else:
                self._publish_goal_status('goal_aborted')
            self._finish_task('idle', detail)
            return

        if context != 'route' or self._mode != 'route':
            self._finish_task('idle', detail)
            return

        point = self._current_point()
        point_name = point['name'] if point else '未知目标点'
        if cancel_reason in ('user_stop', 'external_cancel', 'map_changed'):
            self._publish_route_log(f'路径已停止：{detail}')
            self._finish_task('stopped', detail)
        elif cancel_reason == 'pause':
            self._state = 'paused_navigation'
            self._status_message = f'Paused before point: {point_name}'
            self._publish_route_log(f'已暂停，当前目标点“{point_name}”尚未完成')
            self._send_zero_velocity()
        elif cancel_reason == 'skip':
            self._publish_route_log(f'已跳过目标点“{point_name}”')
            self._advance_route()
        elif (
            not succeeded
            and (
                self._lio_recovering
                or cancel_reason == 'lio_recovery'
            )
        ):
            self._retry_current_after_recovery = True
            self._state = 'waiting_lio_recovery'
            self._status_message = (
                f'Waiting for LIO recovery before retrying {point_name}')
            self._publish_route_log(
                f'目标点“{point_name}”在 LIO 恢复期间中断；'
                '目标点未跳过，恢复后将自动重试。')
        elif succeeded or cancel_reason == 'settled_goal':
            self._publish_route_log(f'已到达目标点“{point_name}”')
            self._point_failure_retries = 0
            dwell = float(point.get('dwell_seconds', 0.0)) if point else 0.0
            if dwell > 0.0:
                self._state = 'dwelling'
                self._dwell_deadline = time.monotonic() + dwell
                self._status_message = (
                    f'Dwelling at {point_name} for {dwell:g} seconds')
                self._publish_route_log(
                    f'在“{point_name}”停留 {dwell:g} 秒')
            else:
                self._advance_route()
        else:
            if self._point_failure_retries < self._max_point_retries:
                self._point_failure_retries += 1
                self._publish_route_log(
                    f'目标点“{point_name}”执行失败（{detail}），'
                    f'正在进行第 {self._point_failure_retries} 次自动重试。')
                self._dispatch_current_route_point()
            else:
                self._publish_route_log(
                    f'目标点“{point_name}”重试 '
                    f'{self._point_failure_retries} 次后仍失败，已跳过：{detail}')
                self._point_failure_retries = 0
                self._advance_route()
        self._publish_all_status()

    def _advance_route(self) -> None:
        if self._mode != 'route' or self._route is None:
            return
        self._dwell_deadline = None
        self._paused_dwell_remaining = 0.0
        self._point_failure_retries = 0
        self._point_index += 1
        if self._point_index >= len(self._route['points']):
            if self._continuous or self._current_loop < self._loop_count:
                self._current_loop += 1
                self._point_index = 0
                self._publish_route_log(
                    f'开始第 {self._current_loop} 轮路径导航')
            else:
                route_name = self._route['name']
                self._publish_route_log(f'路径“{route_name}”执行完成')
                self._finish_task('completed', 'Route completed')
                return
        self._dispatch_current_route_point()

    def _pause_route(self):
        if self._state in ('paused_navigation', 'paused_dwell'):
            return True, 'Route is already paused'
        if self._state == 'dwelling':
            self._paused_dwell_remaining = self._dwell_remaining()
            self._dwell_deadline = None
            self._state = 'paused_dwell'
            self._status_message = 'Dwell timer paused'
            self._publish_route_log('已暂停目标点停留倒计时')
            self._send_zero_velocity()
            return True, 'Route paused'
        if self._state in (
            'sending', 'navigating', 'waiting_nav2', 'starting',
            'recovering_lio', 'waiting_lio_recovery', 'settling'):
            self._cancel_reason = 'pause'
            self._state = 'pausing'
            self._status_message = 'Pausing route'
            if self._active_goal_handle is not None:
                self._cancel_active_goal()
            elif not self._sending:
                self._cancel_reason = None
                self._state = 'paused_navigation'
                self._send_zero_velocity()
            return True, 'Pause requested'
        return False, f'Route cannot be paused from state {self._state}'

    def _resume_route(self):
        if self._state == 'paused_navigation':
            self._publish_route_log('继续导航当前未完成目标点')
            self._dispatch_current_route_point()
            return True, 'Route resumed'
        if self._state == 'paused_dwell':
            self._dwell_deadline = (
                time.monotonic() + self._paused_dwell_remaining)
            self._state = 'dwelling'
            self._status_message = 'Dwell timer resumed'
            self._publish_route_log(
                f'继续停留倒计时，剩余 '
                f'{self._paused_dwell_remaining:.1f} 秒')
            self._paused_dwell_remaining = 0.0
            return True, 'Route resumed'
        return False, 'Route is not paused'

    def _skip_route_point(self):
        point = self._current_point()
        if point is None:
            return False, 'No current route point'
        if self._state in (
            'sending', 'navigating', 'pausing', 'recovering_lio',
            'waiting_lio_recovery', 'settling'):
            self._cancel_reason = 'skip'
            self._state = 'skipping'
            self._status_message = f'Skipping point: {point["name"]}'
            if self._active_goal_handle is not None:
                self._cancel_active_goal()
            elif not self._sending:
                self._cancel_reason = None
                self._publish_route_log(f'已跳过目标点“{point["name"]}”')
                self._advance_route()
            return True, 'Skip requested'
        if self._state in (
            'dwelling', 'paused_dwell', 'paused_navigation', 'waiting_nav2'):
            self._publish_route_log(f'已跳过目标点“{point["name"]}”')
            self._advance_route()
            return True, 'Point skipped'
        return False, f'Point cannot be skipped from state {self._state}'

    def _request_stop(self, reason: str):
        if self._mode == 'idle':
            return False, 'No navigation task is active'
        self._pending_single_pose = None
        self._dwell_deadline = None
        self._paused_dwell_remaining = 0.0
        self._cancel_reason = reason
        self._state = 'stopping'
        self._status_message = 'Stopping navigation'
        self._send_zero_velocity()
        if self._active_goal_handle is not None:
            self._cancel_active_goal()
        elif self._sending:
            pass
        else:
            if self._mode == 'route':
                self._publish_route_log('路径导航已停止')
            self._cancel_reason = None
            self._finish_task('stopped', 'Navigation stopped')
        return True, 'Stop requested'

    def _send_zero_velocity(self) -> None:
        self._zero_velocity_publisher.publish(Twist())

    def _finish_task(self, final_state: str, message: str) -> None:
        self._mode = 'idle'
        self._state = final_state
        self._status_message = message
        self._route = None
        self._point_index = -1
        self._current_loop = 0
        self._loop_count = 1
        self._continuous = False
        self._dwell_deadline = None
        self._paused_dwell_remaining = 0.0
        self._sending = False
        self._active_goal_handle = None
        self._active_pose = None
        self._goal_context = None
        self._cancel_reason = None
        self._cancel_in_progress = False
        self._pending_single_pose = None
        self._retry_current_after_recovery = False
        self._point_failure_retries = 0
        self._settled_since = None
        self._recovery_pose = None
        self._send_zero_velocity()
        self._publish_all_status()

    def cancel_before_shutdown(self) -> None:
        with self._lock:
            self._pending_single_pose = None
            if self._active_goal_handle is not None:
                try:
                    self._active_goal_handle.cancel_goal_async()
                except Exception as error:
                    self.get_logger().warn(
                        f'Unable to cancel goal during shutdown: {error}')
            self._send_zero_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = RouteNavigationManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_before_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
