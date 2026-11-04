#!/usr/bin/env python3
"""Coordinate one-shot 3-D relocalization and AMCL initialisation.

The browser deliberately uses only rosbridge topics.  This node is the ROS 2
side of that small API: it forwards a rough map pose to the ``/relocalize``
Action, publishes the accepted pose to AMCL, and owns the bounded fallback
sequence. It never publishes ``map -> odom``. In Xingji 3D the verified
candidate is handed to global_pose_authority; legacy scopes retain AMCL.
"""

import copy
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lzp_interface.action import Relocalize
from lzp_interface.msg import CurrentMap, RelocalizationRequest
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from car_vel.initial_pose_quality import scan_map_quality
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptyService


class RelocalizationCoordinator(Node):
    """Confirm the scoped authority, or use the original legacy AMCL sequence."""

    MAP_FAILURE_CODES = {
        Relocalize.Result.CODE_MAP_NAME_INVALID,
        Relocalize.Result.CODE_METADATA_MISSING,
        Relocalize.Result.CODE_METADATA_INVALID,
        Relocalize.Result.CODE_PCD_MISSING,
        Relocalize.Result.CODE_PCD_CHECKSUM_MISMATCH,
        Relocalize.Result.CODE_PCD_LOAD_FAILED,
    }

    def __init__(self):
        super().__init__('relocalization_coordinator')

        self.declare_parameter('search_xy', 3.0)
        self.declare_parameter('search_yaw_deg', 90.0)
        self.declare_parameter('relocalize_timeout_sec', 30.0)
        # PCL's align() calls cannot be interrupted from another thread.  The
        # Action deadline is therefore a soft server-side limit.  Keep a
        # separate client-side deadline so AMCL fallback can start even if an
        # NDT/GICP call is still unwinding in the server process.
        self.declare_parameter('action_result_grace_sec', 1.5)
        self.declare_parameter('action_server_wait_sec', 5.0)
        self.declare_parameter('cancel_confirmation_timeout_sec', 12.0)
        self.declare_parameter('amcl_confirmation_timeout_sec', 10.0)
        self.declare_parameter('rough_amcl_timeout_sec', 15.0)
        self.declare_parameter('initial_scan_match_distance', 0.20)
        self.declare_parameter('initial_scan_match_ratio', 0.65)
        self.declare_parameter('amcl_confirmation_samples', 5)
        self.declare_parameter('amcl_confirmation_hold_sec', 1.5)
        self.declare_parameter('amcl_sample_max_gap_sec', 0.75)
        self.declare_parameter('amcl_position_std_limit', 0.55)
        self.declare_parameter('amcl_yaw_std_limit_deg', 30.0)
        self.declare_parameter('amcl_pose_error_limit', 0.75)
        self.declare_parameter('amcl_yaw_error_limit_deg', 35.0)
        self.declare_parameter('refined_amcl_pose_error_limit', 0.30)
        self.declare_parameter('refined_amcl_yaw_error_limit_deg', 15.0)
        self.declare_parameter('amcl_position_spread_limit', 0.15)
        self.declare_parameter('amcl_yaw_spread_limit_deg', 15.0)
        self.declare_parameter('nomotion_update_count', 8)
        self.declare_parameter('nomotion_update_interval_sec', 0.35)

        self.search_xy = self._positive_float('search_xy', 3.0)
        self.search_yaw = math.radians(
            self._positive_float('search_yaw_deg', 90.0))
        self.relocalize_timeout = self._positive_float(
            'relocalize_timeout_sec', 30.0)
        self.action_result_grace = self._positive_float(
            'action_result_grace_sec', 1.5)
        self.action_server_wait = self._positive_float(
            'action_server_wait_sec', 5.0)
        self.cancel_confirmation_timeout = self._positive_float(
            'cancel_confirmation_timeout_sec', 12.0)
        self.amcl_timeout = self._positive_float(
            'amcl_confirmation_timeout_sec', 10.0)
        self.rough_amcl_timeout = self._positive_float('rough_amcl_timeout_sec', 15.0)
        self.scan_match_distance = self._positive_float('initial_scan_match_distance', 0.20)
        self.scan_match_ratio = self._positive_float('initial_scan_match_ratio', 0.65)
        self.amcl_sample_count = max(
            3, int(self.get_parameter('amcl_confirmation_samples').value))
        self.amcl_hold = self._positive_float(
            'amcl_confirmation_hold_sec', 1.5)
        self.amcl_max_gap = self._positive_float(
            'amcl_sample_max_gap_sec', 0.75)
        self.amcl_position_std_limit = self._positive_float(
            'amcl_position_std_limit', 0.55)
        self.amcl_yaw_std_limit = math.radians(
            self._positive_float('amcl_yaw_std_limit_deg', 30.0))
        self.amcl_pose_error_limit = self._positive_float(
            'amcl_pose_error_limit', 0.75)
        self.amcl_yaw_error_limit = math.radians(
            self._positive_float('amcl_yaw_error_limit_deg', 35.0))
        self.refined_amcl_pose_error_limit = self._positive_float(
            'refined_amcl_pose_error_limit', 0.30)
        self.refined_amcl_yaw_error_limit = math.radians(
            self._positive_float(
                'refined_amcl_yaw_error_limit_deg', 15.0))
        self.amcl_position_spread_limit = self._positive_float(
            'amcl_position_spread_limit', 0.15)
        self.amcl_yaw_spread_limit = math.radians(
            self._positive_float('amcl_yaw_spread_limit_deg', 15.0))
        self.nomotion_update_count = max(
            1, int(self.get_parameter('nomotion_update_count').value))
        self.nomotion_update_interval = self._positive_float(
            'nomotion_update_interval_sec', 0.35)

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.initial_map = None
        self.initial_scan = None
        self.initial_scan_at = 0.0
        self.initial_tf = Buffer()
        self.initial_tf_listener = TransformListener(self.initial_tf, self)
        self.create_subscription(OccupancyGrid, '/map', self._initial_map_callback, status_qos)
        self.create_subscription(LaserScan, '/scan', self._initial_scan_callback, amcl_qos)
        self.action_client = ActionClient(self, Relocalize, '/relocalize')
        self.request_subscription = self.create_subscription(
            RelocalizationRequest,
            '/relocalization/request',
            self._request_callback,
            command_qos,
        )
        self.cancel_subscription = self.create_subscription(
            Empty,
            '/relocalization/cancel',
            self._cancel_callback,
            command_qos,
        )
        self.current_map_subscription = self.create_subscription(
            CurrentMap,
            '/current_map_info',
            self._current_map_callback,
            status_qos,
        )
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            amcl_qos,
        )

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', command_qos)
        self.status_publisher = self.create_publisher(
            String, '/relocalization/status', status_qos)
        self.navigation_cancel_publisher = self.create_publisher(
            Empty, '/cancel_navigation', command_qos)
        self.zero_velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel', command_qos)
        self.nomotion_client = self.create_client(
            EmptyService, '/request_nomotion_update')

        self.current_map_load_id = ''
        self._global_mode = ''
        self._global_body = ''
        self._global_body_generation = None
        self._strict_identity = None
        self._await_authority = None
        self._global_status = {}
        self._global_status_at = 0.
        self._global_mode_at = self._global_body_at = 0.
        self.create_subscription(String, '/navigation/mode', self._global_mode_callback, status_qos)
        self.create_subscription(String, '/robot_body/status', self._global_body_callback, status_qos)
        self.create_subscription(String, '/localization/status', self._authority_callback, status_qos)
        self.current_map_name = ''
        self.active_request_map_name = ''
        self.active_request_id = ''
        self.active_request_is_automatic = False
        self.generation = 0
        self.pending_request = None
        self.server_wait_deadline = None
        self.send_generation = None
        self.active_goal_handle = None
        self.active_goal_generation = None
        self.action_deadline_generation = None
        self.action_result_deadline = None
        self.abandoned_action_generations = set()
        self.cancel_requested = False
        self.cancel_wait_deadline = None
        self.last_rough_pose = None
        self.last_rough_generation = None
        self.current_request_stamp = None
        self.amcl_wait = None
        self.latest_amcl_stamp = None
        self.last_status_signature = None
        self.timer = self.create_timer(0.1, self._timer_callback)

        self._publish_status(
            'idle', 'idle', 0.0,
            '三维初始位姿精修协调节点已就绪。', terminal=False)
        self.get_logger().info(
            'Relocalization coordinator ready: '
            '/relocalization/request -> /relocalize -> /initialpose')

    def _positive_float(self, name, default):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            value = default
        if not math.isfinite(value) or value <= 0.0:
            self.get_logger().warn(
                f'Invalid {name}; using {default} instead.')
            return default
        return value

    @staticmethod
    def _yaw(pose):
        quaternion = pose.orientation
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z),
        )

    @staticmethod
    def _angle_distance(first, second):
        delta = first - second
        return abs(math.atan2(math.sin(delta), math.cos(delta)))

    @staticmethod
    def _stamp_seconds(stamp):
        if stamp is None:
            return None
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return value if math.isfinite(value) and value > 0.0 else None

    @staticmethod
    def _pose_dictionary(pose):
        return {
            'x': round(float(pose.pose.pose.position.x), 4),
            'y': round(float(pose.pose.pose.position.y), 4),
            'yaw': round(RelocalizationCoordinator._yaw(
                pose.pose.pose), 5),
        }

    def _publish_status(
        self,
        state,
        stage,
        progress,
        message,
        *,
        terminal=False,
        success=False,
        level='info',
        source_pose='',
        code=0,
        pose=None,
        metrics=None,
        map_name=None,
        request_stamp=None,
        request_id=None,
        force=False,
    ):
        payload = {
            'state': state,
            'stage': stage,
            'progress': max(0.0, min(1.0, float(progress))),
            'message': str(message),
            'terminal': bool(terminal),
            'success': bool(success),
            'level': level,
            'source_pose': source_pose,
            'code': int(code),
            'map_name': (
                str(map_name)
                if map_name is not None
                else self.active_request_map_name or self.current_map_name),
            'generation': self.generation,
            'request_stamp': (
                request_stamp
                if request_stamp is not None
                else self.current_request_stamp),
            'request_id': (
                str(request_id)
                if request_id is not None
                else self.active_request_id),
        }
        if pose is not None:
            try:
                payload['pose'] = self._pose_dictionary(pose)
            except (AttributeError, TypeError, ValueError) as error:
                # Status reporting must never terminate the localization
                # coordinator.  Keep the state transition visible even if a
                # future caller supplies an unexpected pose message type.
                self.get_logger().error(
                    'Failed to encode pose in relocalization status: '
                    f'{type(error).__name__}: {error}')
        if pose is not None:
            payload['pose_stamp'] = dict(sec=int(pose.header.stamp.sec),
                                         nanosec=int(pose.header.stamp.nanosec))
        payload['map_load_id'] = getattr(self, 'current_map_load_id', '')
        payload['candidate_verified'] = bool(getattr(self, '_publishing_verified', False))
        payload['body_generation'] = getattr(self, '_global_body_generation', None)
        if metrics:
            payload['metrics'] = metrics
        encoded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        # Feedback text may be repeated for several clouds.  Avoid flooding
        # rosbridge and the browser log with an identical status.
        signature = (
            state, stage, round(payload['progress'], 2), payload['message'],
            terminal, success, source_pose, int(code))
        if not force and signature == self.last_status_signature:
            return
        self.last_status_signature = signature
        self.status_publisher.publish(String(data=encoded))
        if level == 'error':
            self.get_logger().error(message)
        elif level == 'warning':
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _current_map_callback(self, message):
        if message.success and message.map_name:
            self.current_map_name = str(message.map_name).strip()
            self.current_map_load_id = str(message.load_id).strip()

    def _global_mode_callback(self, message):
        try:
            self._global_mode = json.loads(message.data).get('mode', '')
        except (ValueError, TypeError):
            self._global_mode = ''
        self._global_mode_at = time.monotonic()

    def _global_body_callback(self, message):
        try:
            data = json.loads(message.data)
            self._global_body = data.get('config', {}).get('selected', '')
            self._global_body_generation = data.get('generation')
        except (ValueError, TypeError):
            self._global_body = ''
        self._global_body_at = time.monotonic()

    def _global_identity(self):
        return (self.current_map_name, self.current_map_load_id,
                self._global_body, self._global_body_generation, self._global_mode)

    def _authority_callback(self, message):
        waiting = getattr(self, '_await_authority', None)
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
            self._global_status = data
            self._global_status_at = time.monotonic()
            if not waiting:
                return
            if (waiting['generation'] != self.generation
                    or self._strict_identity != self._global_identity()):
                self._await_authority = None
                return
            if (data.get('scope') == 'xingji_3d' and data.get('ready') is True
                    and data.get('seed_request_id') == self.active_request_id
                    and data.get('map_load_id') == self.current_map_load_id):
                self._await_authority = None
                self._publish_status('ready', 'complete', 1.,
                    '三维精修和统一定位输出已确认；请预览路线后单独启动导航。',
                    terminal=True, success=True, source_pose='refined',
                    pose=waiting['pose'], metrics=waiting['metrics'], force=True)
        except (ValueError, TypeError):
            return

    def _request_callback(self, request):
        message = request.rough_pose
        requested_map_name = str(request.map_name).strip()
        requested_id = str(request.request_id).strip()
        request_stamp = {
            'sec': int(message.header.stamp.sec),
            'nanosec': int(message.header.stamp.nanosec),
        }
        if not requested_map_name:
            self._publish_status(
                'failed', 'validate', 0.0,
                '重定位请求没有携带已加载的地图名称，不能执行三维初始位姿精修。',
                terminal=True, level='error',
                code=Relocalize.Result.CODE_INVALID_GOAL,
                map_name=requested_map_name,
                request_stamp=request_stamp,
                request_id=requested_id, force=True)
            return
        if not requested_id:
            self._publish_status(
                'failed', 'validate', 0.0,
                '重定位请求缺少唯一 request_id，已拒绝本次精修。',
                terminal=True, level='error',
                code=Relocalize.Result.CODE_INVALID_GOAL,
                map_name=requested_map_name,
                request_stamp=request_stamp,
                request_id=requested_id, force=True)
            return
        if not self.current_map_name:
            self._publish_status(
                'failed', 'validate', 0.0,
                f'后端尚未确认二维地图“{requested_map_name}”已加载，'
                '未执行三维精修；请等待地图加载确认后重试。',
                terminal=True, level='error',
                code=Relocalize.Result.CODE_INVALID_GOAL,
                map_name=requested_map_name,
                request_stamp=request_stamp,
                request_id=requested_id, force=True)
            return
        if self.current_map_name != requested_map_name:
            self._publish_status(
                'failed', 'validate', 0.0,
                f'前端请求地图“{requested_map_name}”，但后端当前二维地图为'
                f'“{self.current_map_name}”；为避免二维地图与定位 PCD 混用，'
                '已拒绝本次精修。请重新加载所选地图。',
                terminal=True, level='error',
                code=Relocalize.Result.CODE_INVALID_GOAL,
                map_name=requested_map_name,
                request_stamp=request_stamp,
                request_id=requested_id, force=True)
            return
        if message.header.frame_id not in ('', 'map'):
            self._publish_status(
                'failed', 'validate', 0.0,
                f'粗初始位姿必须位于 map 坐标系，收到 '
                f'{message.header.frame_id!r}。',
                terminal=True, level='error',
                code=Relocalize.Result.CODE_INVALID_GOAL,
                map_name=requested_map_name,
                request_stamp=request_stamp,
                request_id=requested_id, force=True)
            return

        strict = (getattr(self, '_global_mode', '') == 'terrain_3d'
                  and getattr(self, '_global_body', '') == 'xingji')
        self._strict_identity = self._global_identity() if strict else None
        self._await_authority = None
        self.active_request_map_name = requested_map_name
        self.active_request_id = requested_id
        self.active_request_is_automatic = requested_id.startswith(
            'auto_lio_')
        self.current_request_stamp = request_stamp
        self.generation += 1
        generation = self.generation
        # A new request supersedes an earlier explicit cancel.  If an older
        # Action goal is still unwinding, keep this request pending until its
        # result callback confirms that the single registration worker is free.
        self.cancel_requested = False
        self.cancel_wait_deadline = None
        rough_pose = copy.deepcopy(message)
        rough_pose.header.frame_id = 'map'
        self.pending_request = {
            'generation': generation,
            'map_name': requested_map_name,
            'rough_pose': rough_pose,
        }
        self.last_rough_pose = copy.deepcopy(rough_pose)
        self.last_rough_generation = generation
        self.server_wait_deadline = None
        self.amcl_wait = None

        # Manual initial-pose refinement owns navigation cancellation.  An
        # automatic LIO recovery request is issued only after the route manager
        # has retained and canceled the current Nav2 goal itself; publishing
        # /cancel_navigation again here would stop the retained route task.
        if not self.active_request_is_automatic:
            self.navigation_cancel_publisher.publish(Empty())
        self.zero_velocity_publisher.publish(Twist())
        preparation_message = (
            'LIO 已恢复新鲜点云，正在自动执行三维重定位；'
            '当前导航任务和目标点保持不变。'
            if self.active_request_is_automatic
            else '已停止导航输出，正在准备三维初始位姿精修。'
        )
        self._publish_status(
            'starting', 'stop_robot', 0.02,
            preparation_message,
            source_pose='rough', pose=rough_pose, force=True)

        busy_generation = None
        if (
            self.active_goal_handle is not None
            and self.active_goal_generation in
            self.abandoned_action_generations
        ):
            busy_generation = self.active_goal_generation
        elif (
            self.send_generation is not None
            and self.send_generation in self.abandoned_action_generations
        ):
            busy_generation = self.send_generation
        if busy_generation is not None:
            self._fallback_or_fail(
                Relocalize.Result.CODE_TIMEOUT,
                '上一次不可中断的 NDT/GICP 工作线程仍在退出；'
                '为避免两个配准任务重叠，本次直接使用手动粗初始点交给 AMCL。')
            return

        if self.active_goal_handle is not None:
            self._publish_status(
                'canceling', 'cancel_previous', 0.03,
                '正在取消上一次三维精修请求。',
                source_pose='rough')
            self._request_action_cancel(
                self.active_goal_handle, 'replacing the previous request')
            return
        if self.send_generation is not None:
            # The old request has not received a goal handle yet, so it cannot
            # be canceled at this instant.  Its response callback will cancel
            # it (if accepted) and start this newest pending request.
            self._publish_status(
                'canceling', 'cancel_previous', 0.03,
                '正在等待上一次三维精修请求返回，以便安全替换。',
                source_pose='rough')
            return
        self._start_pending_request(generation)

    def _cancel_callback(self, _message):
        self.generation += 1
        self.pending_request = None
        self.server_wait_deadline = None
        self._clear_action_deadline()
        self.amcl_wait = None
        self.cancel_requested = True
        self.cancel_wait_deadline = None
        if self.active_goal_handle is not None:
            self._request_action_cancel(
                self.active_goal_handle, 'explicit browser cancellation')
            self._publish_status(
                'canceling', 'cancel_previous', 0.0,
                '已请求取消三维初始位姿精修，正在等待配准工作线程安全退出。',
                level='warning', code=Relocalize.Result.CODE_CANCELLED,
                force=True)
            self.cancel_wait_deadline = (
                time.monotonic() + self.cancel_confirmation_timeout)
            return
        if self.send_generation is not None:
            # send_goal_async cannot be canceled until it yields a goal
            # handle.  Its response callback will cancel an accepted goal and
            # publish the terminal state only after the Action result arrives.
            self._publish_status(
                'canceling', 'cancel_previous', 0.0,
                '已请求取消三维初始位姿精修，正在等待 Action 接收结果。',
                level='warning', code=Relocalize.Result.CODE_CANCELLED,
                force=True)
            self.cancel_wait_deadline = (
                time.monotonic() + self.cancel_confirmation_timeout)
            return
        self._finish_cancel()

    def _finish_cancel(self):
        self.cancel_requested = False
        self.cancel_wait_deadline = None
        self.active_goal_handle = None
        self.active_goal_generation = None
        self._clear_action_deadline()
        self._publish_status(
            'canceled', 'canceled', 0.0,
            '已取消三维初始位姿精修。',
            terminal=True, level='warning',
            code=Relocalize.Result.CODE_CANCELLED, force=True)

    def _request_action_cancel(self, goal_handle, context):
        if goal_handle is None:
            return False
        try:
            goal_handle.cancel_goal_async()
            return True
        except Exception as error:
            self.get_logger().warn(
                f'Unable to request Action cancellation during {context}: '
                f'{error}')
            return False

    def _clear_action_deadline(self, generation=None):
        if (
            generation is not None
            and self.action_deadline_generation != generation
        ):
            return
        self.action_deadline_generation = None
        self.action_result_deadline = None

    def _start_pending_request(self, generation):
        if (
            self.send_generation is not None
            or self.active_goal_handle is not None
        ):
            return
        pending = self.pending_request
        if (
            pending is None
            or pending['generation'] != generation
            or generation != self.generation
        ):
            return
        if getattr(self, '_strict_identity', None) is not None:
            status = getattr(self, '_global_status', {})
            idle_ack = (status.get('initial_request_id') == self.active_request_id
                        and status.get('matching') is False
                        and status.get('scope') == 'xingji_3d'
                        and status.get('map_load_id') == self.current_map_load_id
                        and time.monotonic()-getattr(self, '_global_status_at', 0.) < .5)
            if not idle_ack:
                now = time.monotonic()
                if self.server_wait_deadline is None:
                    self.server_wait_deadline = now+self.action_server_wait
                if now >= self.server_wait_deadline:
                    self._fallback_or_fail(Relocalize.Result.CODE_TIMEOUT,
                        '局部匹配未确认让出配准线程，未启动重叠精修；请检查统一定位状态')
                else:
                    self._publish_status('waiting', 'wait_local_tracker', .04,
                        '正在等待局部三维匹配退出，再执行初始定位精修。', source_pose='rough')
                return
        if not self.action_client.server_is_ready():
            if self.server_wait_deadline is None:
                self.server_wait_deadline = (
                    time.monotonic() + self.action_server_wait)
                self._publish_status(
                    'waiting', 'wait_action_server', 0.04,
                    '正在等待 /relocalize Action 服务启动。',
                    source_pose='rough')
            return

        self.server_wait_deadline = None
        goal = Relocalize.Goal()
        goal.rough_pose = copy.deepcopy(pending['rough_pose'])
        goal.map_name = pending['map_name']
        goal.search_xy = float(self.search_xy)
        goal.search_yaw = float(self.search_yaw)
        goal.timeout_sec = float(self.relocalize_timeout)
        strict = (getattr(self, '_global_mode', '') == 'terrain_3d'
                  and getattr(self, '_global_body', '') == 'xingji')
        self._strict_identity = self._global_identity() if strict else None
        self._await_authority = None
        goal.robust_validation = strict
        goal.map_load_id = self.current_map_load_id if strict else '' 
        # Consume this request before dispatch.  A replacement request may be
        # queued while send_goal_async is in flight, but this generation must
        # never be sent twice.
        self.pending_request = None
        self.send_generation = generation
        self.action_deadline_generation = generation
        self.action_result_deadline = (
            time.monotonic()
            + self.relocalize_timeout
            + self.action_result_grace
        )
        self._publish_status(
            'refining', 'send_goal', 0.05,
            f'正在对地图“{goal.map_name}”执行 NDT/GICP 自动精修'
            f'（位置 {self.search_xy:g} 米，朝向 ±{math.degrees(self.search_yaw):g}°，'
            f'计算预算 {self.relocalize_timeout:g} 秒）。',
            source_pose='rough')
        try:
            future = self.action_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback, token=generation:
                self._feedback_callback(feedback, token),
            )
        except Exception as error:
            if self.send_generation == generation:
                self.send_generation = None
            self._clear_action_deadline(generation)
            if generation == self.generation:
                self._fallback_or_fail(
                    Relocalize.Result.CODE_INTERNAL_ERROR,
                    f'无法发送三维精修 Action 目标：{error}')
            elif self.pending_request is not None:
                self._start_pending_request(
                    self.pending_request['generation'])
            elif self.cancel_requested:
                self._finish_cancel()
            return
        future.add_done_callback(
            lambda response, token=generation:
            self._goal_response_callback(response, token))

    def _feedback_callback(self, feedback_message, generation):
        if (
            generation != self.generation
            or generation in self.abandoned_action_generations
        ):
            return
        feedback = feedback_message.feedback
        self._publish_status(
            'refining',
            feedback.stage or 'registration',
            feedback.progress,
            feedback.message or '三维初始位姿精修正在运行。',
            source_pose='rough')

    def _goal_response_callback(self, future, generation):
        if self.send_generation == generation:
            self.send_generation = None
        try:
            goal_handle = future.result()
        except Exception as error:
            if generation in self.abandoned_action_generations:
                self.abandoned_action_generations.discard(generation)
                self._clear_action_deadline(generation)
                if self.pending_request is not None:
                    self._start_pending_request(
                        self.pending_request['generation'])
                elif self.cancel_requested:
                    self._finish_cancel()
                return
            if generation == self.generation:
                self._fallback_or_fail(
                    Relocalize.Result.CODE_INTERNAL_ERROR,
                    f'发送三维精修目标失败：{error}')
            elif self.pending_request is not None:
                self._start_pending_request(
                    self.pending_request['generation'])
            elif self.cancel_requested:
                self._finish_cancel()
            return

        if generation in self.abandoned_action_generations:
            if goal_handle.accepted:
                # AMCL fallback has already started.  Keep the handle only so
                # a later request cannot overlap the still-unwinding PCL
                # worker, and ignore its eventual result.
                self.active_goal_handle = goal_handle
                self.active_goal_generation = generation
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(
                    lambda result, token=generation, handle=goal_handle:
                    self._result_callback(result, token, handle))
                self._request_action_cancel(
                    goal_handle, 'abandoned goal response')
            else:
                self.abandoned_action_generations.discard(generation)
                self._clear_action_deadline(generation)
                if self.pending_request is not None:
                    self._start_pending_request(
                        self.pending_request['generation'])
                elif self.cancel_requested:
                    self._finish_cancel()
            return

        if generation != self.generation:
            if goal_handle.accepted:
                # The Action server has one worker.  A cancel response only
                # acknowledges the request; it does not mean a PCL align call
                # has returned.  Retain the handle and wait for the terminal
                # result before starting the replacement request.
                self.active_goal_handle = goal_handle
                self.active_goal_generation = generation
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(
                    lambda result, token=generation, handle=goal_handle:
                    self._result_callback(result, token, handle))
                self._request_action_cancel(
                    goal_handle, 'superseded goal response')
            elif self.pending_request is not None:
                self._start_pending_request(
                    self.pending_request['generation'])
            elif self.cancel_requested:
                self._finish_cancel()
            return
        if not goal_handle.accepted:
            self._fallback_or_fail(
                Relocalize.Result.CODE_INTERNAL_ERROR,
                '三维精修 Action 拒绝了请求。')
            return

        self.pending_request = None
        self.active_goal_handle = goal_handle
        self.active_goal_generation = generation
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, token=generation, handle=goal_handle:
            self._result_callback(result, token, handle))

    def _result_callback(self, future, generation, goal_handle):
        if goal_handle is self.active_goal_handle:
            self.active_goal_handle = None
            self.active_goal_generation = None
        self._clear_action_deadline(generation)
        if generation in self.abandoned_action_generations:
            self.abandoned_action_generations.discard(generation)
            if self.pending_request is not None:
                self._start_pending_request(
                    self.pending_request['generation'])
            elif self.cancel_requested:
                self._finish_cancel()
            return
        if generation != self.generation:
            if self.pending_request is not None:
                self._start_pending_request(
                    self.pending_request['generation'])
            elif self.cancel_requested:
                self._finish_cancel()
            return
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
        except Exception as error:
            self._fallback_or_fail(
                Relocalize.Result.CODE_INTERNAL_ERROR,
                f'读取三维精修结果失败：{error}')
            return

        try:
            self._process_action_result(result, status)
        except Exception as error:
            self._fallback_or_fail(
                Relocalize.Result.CODE_INTERNAL_ERROR,
                f'处理三维精修结果时发生异常：{error}')

    def _process_action_result(self, result, status):
        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and result.success
            and result.code == Relocalize.Result.CODE_OK
        ):
            metrics = {
                'fitness_m': round(float(result.fitness), 4),
                'overlap': round(float(result.overlap), 4),
                'correction_xy_m': round(float(result.correction_xy), 4),
                'correction_yaw_deg': round(
                    math.degrees(float(result.correction_yaw)), 2),
            }
            if self._strict_identity is not None:
                if (self._strict_identity != self._global_identity()
                        or not result.candidate_verified
                        or time.monotonic()-self._global_mode_at > 2.
                        or time.monotonic()-self._global_body_at > 2.):
                    self._fallback_or_fail(Relocalize.Result.CODE_VALIDATION_FAILED,
                        '三维定位候选未确认或地图/本体身份已变化')
                    return
                self._await_authority = dict(generation=self.generation,
                    pose=copy.deepcopy(result.refined_pose), metrics=metrics,
                    deadline=time.monotonic()+3.)
                self._publishing_verified = True
                self._publish_status('refined', 'validation_passed', .9,
                    '多个三维候选已验证，正在确认唯一全局定位输出。',
                    source_pose='refined', pose=result.refined_pose, metrics=metrics, force=True)
                self._publishing_verified = False
                return
            rough_pose = self._active_rough_pose()
            if rough_pose is None:
                rough_pose = copy.deepcopy(result.refined_pose)
            self._publish_status(
                'refined', 'validation_passed', 0.82,
                'NDT/GICP 精修及质量验证通过，正在交给 AMCL 确认。',
                source_pose='refined', pose=result.refined_pose,
                metrics=metrics, force=True)
            self._begin_amcl_confirmation(
                result.refined_pose, rough_pose, 'refined', metrics=metrics)
            return

        code = int(result.code)
        message = result.message or (
            f'三维精修未成功，Action 状态码 {status}。')
        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status(
                'canceled', 'canceled', 0.0, message,
                terminal=True, level='warning',
                code=Relocalize.Result.CODE_CANCELLED, force=True)
            return
        self._fallback_or_fail(code, message)

    def _active_rough_pose(self):
        # ``send_goal_async`` copies the goal, but rclpy does not expose that
        # goal through the returned handle.  Keep a generation-local copy.
        if self.last_rough_generation != self.generation:
            return None
        return copy.deepcopy(self.last_rough_pose)

    def _fallback_or_fail(self, code, message):
        self.pending_request = None
        self.server_wait_deadline = None
        self._clear_action_deadline(self.generation)
        rough_pose = self._active_rough_pose()
        if self.active_request_is_automatic or getattr(self, '_strict_identity', None) is not None:
            self._publish_status(
                'failed', 'automatic_refinement_failed', 1.0,
                f'三维重定位未通过，未采用粗定位降级启动导航：{message}',
                terminal=True, level='error', code=code, force=True)
            return
        if code in self.MAP_FAILURE_CODES or rough_pose is None:
            self._publish_status(
                'failed', 'map_validation_failed', 1.0,
                f'三维定位地图不可用，已禁止导航：{message}',
                terminal=True, level='error', code=code, force=True)
            return
        warning = (
            f'三维精修失败，已回退到手动粗初始点并交给 AMCL：{message}')
        self._publish_status(
            'fallback', 'fallback_to_rough', 0.82, warning,
            level='warning', source_pose='rough',
            code=code, pose=rough_pose, force=True)
        self._begin_amcl_confirmation(
            rough_pose, rough_pose, 'rough',
            refinement_error=warning)

    def _begin_amcl_confirmation(
        self,
        candidate,
        rough_pose,
        source,
        *,
        metrics=None,
        refinement_error='',
        retried=False,
    ):
        pose = copy.deepcopy(candidate)
        pose.header.frame_id = 'map'
        now = self.get_clock().now()
        backdated = now - Duration(seconds=0.2)
        pose.header.stamp = backdated.to_msg()
        self._ensure_initial_covariance(pose)
        if source == 'rough':
            # Search limits describe a roughly two-sigma neighbourhood, not an
            # already precise pose. Never reuse the browser's refined covariance.
            pose.pose.covariance[0] = pose.pose.covariance[7] = (self.search_xy / 2.0) ** 2
            pose.pose.covariance[35] = (self.search_yaw / 2.0) ** 2

        self.amcl_wait = {
            'generation': self.generation,
            'candidate': copy.deepcopy(pose),
            'rough_pose': copy.deepcopy(rough_pose),
            'source': source,
            'retried': bool(retried),
            'started_at': time.monotonic(),
            'deadline': time.monotonic() + (self.rough_amcl_timeout if source == 'rough' else self.amcl_timeout),
            'amcl_stamp_before': self.latest_amcl_stamp,
            'window': None,
            'metrics': metrics or {},
            'refinement_error': refinement_error,
            'nomotion_sent': 0,
            'next_nomotion_at': time.monotonic(),
        }
        self.initial_pose_publisher.publish(pose)
        label = '精修位姿' if source == 'refined' else '手动粗初始点'
        self._publish_status(
            'confirming_amcl', 'amcl_confirmation', 0.86,
            f'已发布{label}，正在等待 AMCL 连续稳定确认。',
            source_pose=source, pose=pose,
            metrics=metrics, force=True)

    @staticmethod
    def _ensure_initial_covariance(message):
        covariance = message.pose.covariance
        if len(covariance) != 36:
            covariance = [0.0] * 36
        if covariance[0] <= 0.0:
            covariance[0] = 0.04
        if covariance[7] <= 0.0:
            covariance[7] = 0.04
        if covariance[35] <= 0.0:
            covariance[35] = 0.03
        message.pose.covariance = covariance

    def _initial_map_callback(self, message):
        self.initial_map = message

    def _initial_scan_callback(self, message):
        self.initial_scan = message
        self.initial_scan_at = time.monotonic()

    def _initial_scan_quality(self, pose):
        scan, grid = self.initial_scan, self.initial_map
        if scan is None or grid is None:
            return None, '缺少当前激光或二维地图'
        if grid.header.frame_id != pose.header.frame_id:
            return None, '地图与定位坐标系不一致'
        scan_stamp = self._stamp_seconds(scan.header.stamp)
        pose_stamp = self._stamp_seconds(pose.header.stamp)
        if (time.monotonic() - self.initial_scan_at > 0.75
                or scan_stamp is None or pose_stamp is None
                or abs(scan_stamp - pose_stamp) > 0.75):
            return None, '激光与定位时间不同步'
        try:
            # Compensate the scan's acquisition time through odom. Using map TF
            # here would accidentally score the previous AMCL pose.
            transform = self.initial_tf.lookup_transform_full(
                'base_footprint', Time.from_msg(pose.header.stamp),
                scan.header.frame_id, Time.from_msg(scan.header.stamp), 'odom')
        except TransformException as error:
            return None, '激光 TF 不可用：' + str(error)
        quality = scan_map_quality(grid, scan, pose.pose.pose,
                                   transform.transform, self.scan_match_distance)
        if quality['valid_beams'] < 30:
            return None, '有效激光不足 30 束'
        return quality, f"匹配率 {quality['match_ratio']:.0%}，要求 {self.scan_match_ratio:.0%}"

    def _amcl_callback(self, message):
        stamp = self._stamp_seconds(message.header.stamp)
        if stamp is not None:
            if (
                self.latest_amcl_stamp is None
                or stamp > self.latest_amcl_stamp
            ):
                self.latest_amcl_stamp = stamp
        waiting = self.amcl_wait
        if (
            waiting is None
            or waiting['generation'] != self.generation
            or stamp is None
        ):
            return
        previous_stamp = waiting['amcl_stamp_before']
        if previous_stamp is not None and stamp <= previous_stamp + 1.0e-6:
            return

        covariance = message.pose.covariance
        if len(covariance) < 36:
            return
        variances = (covariance[0], covariance[7], covariance[35])
        if any(not math.isfinite(value) or value < 0.0 for value in variances):
            return
        position_std = math.sqrt(max(variances[0], variances[1]))
        yaw_std = math.sqrt(variances[2])
        pose = message.pose.pose
        candidate = waiting['candidate'].pose.pose
        sample_yaw = self._yaw(pose)
        candidate_yaw = self._yaw(candidate)
        position_error = math.hypot(
            pose.position.x - candidate.position.x,
            pose.position.y - candidate.position.y)
        yaw_error = self._angle_distance(sample_yaw, candidate_yaw)
        if waiting['source'] == 'refined':
            pose_error_limit = self.refined_amcl_pose_error_limit
            yaw_error_limit = self.refined_amcl_yaw_error_limit
        else:
            pose_error_limit = self.search_xy
            yaw_error_limit = self.search_yaw
        sample_is_valid = (
            position_std <= self.amcl_position_std_limit
            and yaw_std <= self.amcl_yaw_std_limit
            and position_error <= pose_error_limit
            and yaw_error <= yaw_error_limit
        )
        if not sample_is_valid:
            waiting['window'] = None
            reason = (f'位置不确定度 {position_std:.2f}/{self.amcl_position_std_limit:.2f} 米，'
                      f'朝向不确定度 {math.degrees(yaw_std):.0f}/{math.degrees(self.amcl_yaw_std_limit):.0f}°；'
                      f'修正距离 {position_error:.2f}/{pose_error_limit:.2f} 米，'
                      f'修正角度 {math.degrees(yaw_error):.0f}/{math.degrees(yaw_error_limit):.0f}°')
            waiting['quality_error'] = reason
            if time.monotonic() >= waiting.get('next_diagnostic_at', 0.0):
                waiting['next_diagnostic_at'] = time.monotonic() + 1.0
                elapsed = time.monotonic() - waiting['started_at']
                self._publish_status('confirming_amcl', 'amcl_confirmation', 0.86,
                    f'AMCL 正在收敛（已等待 {elapsed:.0f} 秒）：' + reason,
                    source_pose=waiting['source'])
            return

        quality, reason = self._initial_scan_quality(message)
        if quality is None or quality['match_ratio'] < self.scan_match_ratio:
            waiting['window'] = None
            waiting['quality_error'] = reason
            self._publish_status('confirming_amcl', 'scan_map_validation', 0.86,
                '等待激光与地图匹配确认：' + reason, source_pose=waiting['source'])
            return
        waiting['metrics'].update(quality)
        window = waiting['window']
        if (
            window is None
            or stamp <= window['last_stamp'] + 1.0e-6
            or stamp - window['last_stamp'] > self.amcl_max_gap
            or math.hypot(
                pose.position.x - window['anchor_x'],
                pose.position.y - window['anchor_y'])
            > self.amcl_position_spread_limit
            or self._angle_distance(sample_yaw, window['anchor_yaw'])
            > self.amcl_yaw_spread_limit
        ):
            window = {
                'first_stamp': stamp,
                'last_stamp': stamp,
                'count': 1,
                'anchor_x': pose.position.x,
                'anchor_y': pose.position.y,
                'anchor_yaw': sample_yaw,
                'last_pose': copy.deepcopy(message),
            }
            waiting['window'] = window
        else:
            window['last_stamp'] = stamp
            window['count'] += 1
            window['last_pose'] = copy.deepcopy(message)

        duration = window['last_stamp'] - window['first_stamp']
        progress = min(
            0.99,
            0.86 + 0.13 * min(
                window['count'] / self.amcl_sample_count,
                duration / self.amcl_hold))
        self._publish_status(
            'confirming_amcl', 'amcl_confirmation', progress,
            f'AMCL 稳定确认：{window["count"]}/{self.amcl_sample_count} '
            f'个新时间戳样本，连续 {duration:.1f}/{self.amcl_hold:.1f} 秒。',
            source_pose=waiting['source'])

        if (
            window['count'] >= self.amcl_sample_count
            and duration >= self.amcl_hold
        ):
            source = waiting['source']
            metrics = waiting['metrics']
            refinement_error = waiting['refinement_error']
            confirmed_pose = window['last_pose']
            self.amcl_wait = None
            message_text = (
                '三维精修位姿已经 AMCL 稳定确认，现在可以发送导航目标。'
                if source == 'refined'
                else '已使用手动粗初始点完成 AMCL 稳定定位，'
                '现在可以发送导航目标。')
            if refinement_error:
                message_text += ' 三维精修此前未通过，当前为降级定位。'
            self._publish_status(
                'ready', 'complete', 1.0, message_text,
                terminal=True, success=True,
                level='info' if source == 'refined' else 'warning',
                source_pose=source, pose=confirmed_pose,
                metrics=metrics, force=True)

    def _request_initial_amcl_update(self, waiting, now):
        if now >= waiting['deadline'] or now < waiting['next_nomotion_at']:
            return
        if waiting['source'] != 'rough' and waiting['nomotion_sent'] >= self.nomotion_update_count:
            return
        future = waiting.get('nomotion_future')
        if future is not None and not future.done():
            return
        scan = self.initial_scan
        if scan is None:
            return
        stamp = self._stamp_seconds(scan.header.stamp)
        if stamp is None or stamp == waiting.get('nomotion_scan_stamp'):
            return
        if self.nomotion_client.service_is_ready():
            waiting['nomotion_future'] = self.nomotion_client.call_async(EmptyService.Request())
            waiting['nomotion_scan_stamp'] = stamp
            waiting['nomotion_sent'] += 1
            interval = min(self.nomotion_update_interval, 0.2) if waiting['source'] == 'rough' else self.nomotion_update_interval
            waiting['next_nomotion_at'] = now + interval

    def _timer_callback(self):
        now = time.monotonic()
        awaiting = getattr(self, '_await_authority', None)
        if awaiting and now > awaiting['deadline']:
            self._await_authority = None
            self._fallback_or_fail(Relocalize.Result.CODE_VALIDATION_FAILED,
                '统一三维定位输出未确认，请检查/localization/status，重新确认初始点')
        if (
            self.cancel_requested
            and self.cancel_wait_deadline is not None
            and now >= self.cancel_wait_deadline
        ):
            busy_generation = (
                self.active_goal_generation
                if self.active_goal_handle is not None
                else self.send_generation
            )
            if busy_generation is not None:
                self.abandoned_action_generations.add(busy_generation)
            self.cancel_requested = False
            self.cancel_wait_deadline = None
            self._publish_status(
                'canceled', 'cancel_timeout', 0.0,
                '取消请求已等待 12 秒；网页可以继续操作，但不可中断的 '
                'NDT/GICP 工作线程可能仍在后台退出。下一次初始点会先使用 '
                'AMCL 粗定位，避免启动重叠配准。',
                terminal=True, level='warning',
                code=Relocalize.Result.CODE_CANCELLED, force=True)

        pending = self.pending_request
        if pending is not None and self.server_wait_deadline is not None:
            if self.action_client.server_is_ready():
                self._start_pending_request(pending['generation'])
            elif now >= self.server_wait_deadline:
                self.server_wait_deadline = None
                self._fallback_or_fail(
                    Relocalize.Result.CODE_INTERNAL_ERROR,
                    '等待 /relocalize Action 服务启动超时。')

        action_generation = self.action_deadline_generation
        if (
            action_generation is not None
            and self.action_result_deadline is not None
            and now >= self.action_result_deadline
        ):
            self._clear_action_deadline(action_generation)
            if (
                action_generation == self.generation
                and not self.cancel_requested
                and self.amcl_wait is None
            ):
                # The server checks the same 10 s deadline between stages, but
                # a blocking PCL align() cannot observe it.  Abandon only the
                # late Action result; keep its handle until the worker exits so
                # another expensive registration cannot overlap it.
                self.abandoned_action_generations.add(action_generation)
                if (
                    self.active_goal_handle is not None
                    and self.active_goal_generation == action_generation
                ):
                    self._request_action_cancel(
                        self.active_goal_handle,
                        'registration hard-fallback deadline')
                self._fallback_or_fail(
                    Relocalize.Result.CODE_TIMEOUT,
                    'NDT/GICP 在后端硬回退期限内没有返回结果；'
                    '已请求取消仍在运行的配准，并立即改用手动粗初始点交给 AMCL。')

        waiting = self.amcl_wait
        if waiting is None or waiting['generation'] != self.generation:
            return
        self._request_initial_amcl_update(waiting, now)

        if now < waiting['deadline']:
            return
        if (
            waiting['source'] == 'refined'
            and not waiting['retried']
            and not self.active_request_is_automatic
        ):
            rough_pose = waiting['rough_pose']
            metrics = waiting['metrics']
            self._publish_status(
                'fallback', 'amcl_retry_rough', 0.84,
                'AMCL 未能在 10 秒内确认精修位姿，正在自动使用原始手动初始点重试一次。',
                level='warning', source_pose='rough',
                pose=rough_pose, metrics=metrics, force=True)
            self._begin_amcl_confirmation(
                rough_pose,
                rough_pose,
                'rough',
                metrics=metrics,
                refinement_error='AMCL 未确认精修位姿',
                retried=True,
            )
            return

        source = waiting['source']
        self.amcl_wait = None
        self._publish_status(
            'failed', 'amcl_confirmation_timeout', 1.0,
            'AMCL 在自动回退后仍未形成稳定定位，已禁止导航；'
            '请检查点云、TF、地图匹配后重新设置初始点。' + waiting.get('quality_error', ''),
            terminal=True, level='error',
            source_pose=source,
            code=Relocalize.Result.CODE_VALIDATION_FAILED,
            force=True)

    def destroy_node(self):
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
