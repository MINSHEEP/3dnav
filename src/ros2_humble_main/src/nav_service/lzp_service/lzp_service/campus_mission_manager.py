#!/usr/bin/env python3
"""Fail-closed multi-map campus navigation mission manager."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from lzp_interface.msg import (
    CurrentMap,
    RelocalizationRequest,
    RouteExecutionStatus,
    TerrainObservation as TerrainObservationMessage,
)
from lzp_interface.srv import (
    ControlCampusMission,
    NavigateManagedGoal,
    SetTraversalMode,
    StartCampusMission,
    SwitchMap,
)

from .connector_traversal import ConnectorTraversal, DogHealth, OdomSample
from .campus_topology_store import CampusTopologyStore
from .terrain_navigation import (
    CampusTopology,
    TerrainObservation,
    TransitionPose,
)


def _yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class CampusMissionManager(Node):
    """Execute topology transitions using Nav2, map switching and guarded connectors."""

    SPECIAL_MODES = frozenset({
        'ramp', 'stairs_up', 'stairs_down', 'obstacle', 'rough_ground'})

    def __init__(self):
        super().__init__('campus_mission_manager')
        self._group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        topology_file = str(self.declare_parameter(
            'campus_topology_file',
            '~/.ros/robot_navigation/campus_topology.json').value).strip()
        if not topology_file:
            topology_file = '~/.ros/robot_navigation/campus_topology.json'
        self._topology_store = CampusTopologyStore(topology_file)
        self._topology_revision = 0
        self._topology = None
        try:
            self._reload_topology()
            self.get_logger().info(
                f'Loaded executable campus topology: '
                f'{self._topology_store.path}')
        except Exception as error:
            self.get_logger().error(
                f'Campus mission execution disabled: {error}')

        self._stage_timeout = max(10.0, float(self.declare_parameter(
            'stage_timeout_sec', 90.0).value))
        self._connector_lateral_limit = max(0.05, float(self.declare_parameter(
            'connector_lateral_limit_m', 0.25).value))
        self._connector_yaw_limit = max(2.0, float(self.declare_parameter(
            'connector_yaw_limit_deg', 10.0).value))

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        commands = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        sensors = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(
            CurrentMap, '/current_map_info', self._map_callback, latched,
            callback_group=self._group)
        self.create_subscription(
            RouteExecutionStatus, '/route_execution_status',
            self._navigation_callback, latched, callback_group=self._group)
        self.create_subscription(
            String, '/dog_status', self._dog_callback, latched,
            callback_group=self._group)
        self.create_subscription(
            String, '/relocalization/status', self._relocalization_callback,
            latched, callback_group=self._group)
        self.create_subscription(
            String, '/terrain_costmap/status', self._terrain_callback,
            latched, callback_group=self._group)
        self.create_subscription(
            Odometry, '/Odometry', self._odometry_callback, sensors,
            callback_group=self._group)
        self.create_subscription(
            TerrainObservationMessage, '/terrain_observation',
            self._live_terrain_callback, sensors,
            callback_group=self._group)

        self._status_publisher = self.create_publisher(
            String, '/campus_mission/status', latched)
        self._active_publisher = self.create_publisher(
            Bool, '/campus_mission/active', latched)
        self._connector_active_publisher = self.create_publisher(
            Bool, '/campus_mission/connector_active', latched)
        self._velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel_campus', commands)
        self._cancel_navigation_publisher = self.create_publisher(
            Empty, '/cancel_navigation', commands)
        self._stop_publisher = self.create_publisher(
            Empty, '/motion_stop', commands)
        self._relocalization_publisher = self.create_publisher(
            RelocalizationRequest, '/relocalization/request', commands)

        self._goal_client = self.create_client(
            NavigateManagedGoal, '/navigate_managed_goal',
            callback_group=self._group)
        self._mode_client = self.create_client(
            SetTraversalMode, '/dog_remote/set_traversal_mode',
            callback_group=self._group)
        self._map_client = self.create_client(
            SwitchMap, '/switch_map', callback_group=self._group)
        self.create_service(
            StartCampusMission, '/start_campus_mission', self._start,
            callback_group=self._group)
        self.create_service(
            ControlCampusMission, '/control_campus_mission', self._control,
            callback_group=self._group)

        self._current_map = ''
        self._route_status = None
        self._last_goal_sequence = 0
        self._dog_health = None
        self._odom = None
        self._relocalization_status = {}
        self._terrain_status = {}
        self._terrain_received_at = 0.0
        self._live_terrain = None
        self._live_terrain_received_at = 0.0
        self._map_switch_started_at = 0.0
        self._active = False
        self._state = 'idle'
        self._stage = 'idle'
        self._message = 'Campus mission manager is ready'
        self._plan = None
        self._segment_index = 0
        self._available_modes = set()
        self._mission_id = ''
        self._owner = ''
        self._expected_goal_sequence = 0
        self._stage_after_goal = ''
        self._goal_request_started_at = 0.0
        self._pending_future = None
        self._pending_kind = ''
        self._deadline = None
        self._connector = None
        self._relocalization_id = ''
        self._current_mode = 'normal'
        self._has_final_pose = False
        self._final_pose = None
        self._timer = self.create_timer(0.05, self._tick)
        self._publish_status(force=True)
        self._connector_active_publisher.publish(Bool(data=False))

    def _map_callback(self, message):
        with self._lock:
            if message.success and message.map_name:
                self._current_map = str(message.map_name).strip()

    def _navigation_callback(self, message):
        with self._lock:
            self._route_status = message
            self._last_goal_sequence = max(
                self._last_goal_sequence, int(message.goal_event_sequence))

    def _dog_callback(self, message):
        with self._lock:
            now = time.monotonic()
            try:
                value = json.loads(message.data)
                self._dog_health = DogHealth(
                    connected=bool(value.get('connected', False)),
                    standing=int(value.get('motion_mode', -1)) == 2,
                    hardware_error=bool(value.get('hardware_error', True)),
                    received_at=now)
            except (TypeError, ValueError, json.JSONDecodeError):
                self._dog_health = DogHealth(False, False, True, now)
            if self._active and self._stage != 'traverse_connector':
                health = self._dog_health
                if health is not None and (
                        not health.connected or not health.standing
                        or health.hardware_error):
                    self._fail(
                        'Robot health became unsafe during campus mission')

    def _odometry_callback(self, message):
        with self._lock:
            position = message.pose.pose.position
            self._odom = OdomSample(
                float(position.x), float(position.y), float(position.z),
                _yaw(message.pose.pose.orientation), time.monotonic())

    def _health_ready(self):
        health = self._dog_health
        return bool(
            health is not None
            and time.monotonic() - health.received_at <= 1.0
            and health.connected and health.standing
            and not health.hardware_error)

    def _relocalization_callback(self, message):
        with self._lock:
            try:
                self._relocalization_status = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                self._relocalization_status = {}

    def _terrain_callback(self, message):
        with self._lock:
            try:
                self._terrain_status = json.loads(message.data)
                self._terrain_received_at = time.monotonic()
            except (TypeError, ValueError, json.JSONDecodeError):
                self._terrain_status = {}

    def _live_terrain_callback(self, message):
        with self._lock:
            try:
                self._live_terrain = TerrainObservation.from_mapping({
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
                self._live_terrain_received_at = time.monotonic()
            except (TypeError, ValueError):
                self._live_terrain = None
                self._live_terrain_received_at = 0.0

    def _reload_topology(self):
        content = self._topology_store.load()
        self._topology = CampusTopology.from_mapping(content)
        self._topology_revision = int(content.get('revision', 0))

    def _live_terrain_ready(self, segment):
        if (self._live_terrain is None
                or time.monotonic() - self._live_terrain_received_at > 0.75):
            return False, '实时地形观测不可用或已过期'
        assessment = self._topology.capability.assess(
            self._live_terrain, self._available_modes)
        if not assessment.traversable:
            return False, f'实时地形禁止通行：{assessment.reason}'
        expected = segment.assessment.required_mode or 'normal'
        observed = assessment.required_mode or 'normal'
        if observed not in {'normal', expected}:
            return False, (
                f'实时地形需要 {observed}，与计划模式 {expected} 不一致')
        return True, assessment.reason

    def _start(self, request, response):
        with self._lock:
            if self._active:
                response.success = False
                response.message = 'A campus mission is already active'
                return response
            try:
                self._reload_topology()
            except Exception as error:
                response.success = False
                response.message = f'园区拓扑刷新失败：{error}'
                return response
            if self._topology is None:
                response.success = False
                response.message = 'Campus topology is not configured'
                return response
            source = str(request.source_area).strip()
            target = str(request.target_area).strip()
            modes = {
                str(value).strip().lower() for value in request.available_modes
                if str(value).strip()}
            try:
                plan = self._topology.plan(source, target, modes)
            except Exception as error:
                response.success = False
                response.message = str(error)
                return response
            if not plan.segments and not request.has_final_pose:
                response.success = False
                response.message = 'Mission has no transition or final goal'
                return response
            if any(not value.transition.executable for value in plan.segments):
                response.success = False
                response.message = 'Every transition needs calibrated source_pose and target_pose'
                return response
            source_map = self._topology.areas[source].map_name
            if not self._current_map or self._current_map != source_map:
                response.success = False
                response.message = (
                    f'Active map is {self._current_map or "unknown"}; '
                    f'source area requires {source_map}')
                return response
            if self._route_status is not None and self._route_status.active:
                response.success = False
                response.message = 'Another Nav2 task is active'
                return response
            for segment in plan.segments:
                if segment.assessment.required_mode in self.SPECIAL_MODES:
                    if segment.transition.elevation_change_m is None and (
                            segment.assessment.required_mode in
                            {'stairs_up', 'stairs_down'}):
                        response.success = False
                        response.message = (
                            f'{segment.transition.transition_id} needs '
                            'elevation_change_m for stair verification')
                        return response
            self._plan = plan
            self._segment_index = 0
            self._available_modes = modes
            self._mission_id = uuid.uuid4().hex
            self._owner = f'campus:{self._mission_id}'
            self._has_final_pose = bool(request.has_final_pose)
            self._final_pose = request.final_pose if request.has_final_pose else None
            self._active = True
            self._state = 'running'
            self._set_stage('prepare_segment', 'Mission accepted')
            self._active_publisher.publish(Bool(data=True))
            self._publish_status(force=True)
            response.success = True
            response.message = f'Campus mission {self._mission_id} accepted'
            return response

    def _control(self, request, response):
        command = str(request.command).strip().lower()
        with self._lock:
            if command != 'stop':
                response.success = False
                response.message = 'Only the fail-safe stop command is supported'
                return response
            if not self._active:
                response.success = True
                response.message = 'No campus mission is active'
                return response
            self._cancel_navigation_publisher.publish(Empty())
            self._publish_zero()
            self._finish('stopped', 'Campus mission stopped by operator')
            response.success = True
            response.message = 'Stop requested'
            return response

    def _set_stage(self, stage, message, timeout=None):
        self._stage = stage
        self._message = str(message)
        self._deadline = (
            time.monotonic() + (self._stage_timeout if timeout is None else timeout))
        self._publish_status(force=True)

    def _current_segment(self):
        if self._plan is None or self._segment_index >= len(self._plan.segments):
            return None
        return self._plan.segments[self._segment_index]

    @staticmethod
    def _pose_stamped(value: TransitionPose):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.pose.position.x = value.x
        message.pose.position.y = value.y
        message.pose.position.z = value.z
        message.pose.orientation.z = math.sin(value.yaw * 0.5)
        message.pose.orientation.w = math.cos(value.yaw * 0.5)
        return message

    def _call_goal(self, pose, *, require_yaw, next_stage):
        if not self._goal_client.service_is_ready():
            self._fail('Managed Nav2 goal service is unavailable')
            return
        request = NavigateManagedGoal.Request()
        request.goal = pose
        request.owner_token = self._owner
        request.require_yaw = bool(require_yaw)
        self._expected_goal_sequence = self._last_goal_sequence + 1
        self._goal_request_started_at = time.monotonic()
        self._pending_future = self._goal_client.call_async(request)
        self._pending_kind = f'goal:{next_stage}'
        self._set_stage('request_goal', 'Submitting a mission-owned Nav2 goal')

    def _call_mode(self, mode, next_stage):
        if not self._mode_client.service_is_ready():
            self._fail('Robot traversal-mode service is unavailable')
            return
        request = SetTraversalMode.Request()
        request.mode = str(mode)
        self._pending_future = self._mode_client.call_async(request)
        self._pending_kind = f'mode:{next_stage}:{mode}'
        self._set_stage('request_mode', f'Requesting traversal mode {mode}')

    def _call_map(self, map_name):
        if not self._map_client.service_is_ready():
            self._fail('Map switch service is unavailable')
            return
        request = SwitchMap.Request()
        request.map_name = str(map_name)
        self._map_switch_started_at = time.monotonic()
        self._pending_future = self._map_client.call_async(request)
        self._pending_kind = 'map:wait_terrain'
        self._set_stage('switch_map', f'Switching to map {map_name}', timeout=120.0)

    def _consume_future(self):
        future = self._pending_future
        if future is None or not future.done():
            return False
        kind = self._pending_kind
        self._pending_future = None
        self._pending_kind = ''
        try:
            result = future.result()
        except Exception as error:
            self._fail(f'{kind} request failed: {error}')
            return True
        if result is None or not result.success:
            self._fail(getattr(result, 'message', f'{kind} request rejected'))
            return True
        parts = kind.split(':')
        category = parts[0]
        next_stage = parts[1]
        if category == 'goal':
            self._set_stage('wait_goal', 'Waiting for Nav2 goal completion', timeout=300.0)
            self._stage_after_goal = next_stage
        elif category == 'mode':
            self._current_mode = parts[2]
            self._set_stage(next_stage, result.message)
        elif category == 'map':
            if self._current_map != self._current_segment().target.map_name:
                self._fail(
                    'Map switch acknowledged but active map does not match target')
                return True
            self._terrain_status = {}
            self._terrain_received_at = 0.0
            self._set_stage(
                'wait_terrain',
                'Waiting for the complete target-map terrain costmap',
                timeout=120.0)
        return True

    def _goal_terminal(self):
        status = self._route_status
        if status is None:
            return None
        if int(status.goal_event_sequence) < self._expected_goal_sequence:
            return None
        if str(status.last_goal_owner_token) != self._owner:
            return None
        if status.active and str(status.owner_token) == self._owner:
            return None
        if time.monotonic() - self._goal_request_started_at < 0.05:
            return None
        return str(status.last_goal_result), str(status.last_goal_detail)

    def _start_connector(self, segment):
        if self._odom is None or self._dog_health is None:
            self._fail('Fresh 3D odometry and robot status are required')
            return
        terrain_ready, terrain_message = self._live_terrain_ready(segment)
        if not terrain_ready:
            self._fail(terrain_message, latch_stop=True)
            return
        distance = segment.transition.distance_m
        expected_height = segment.transition.elevation_change_m
        if segment.transition.source_pose is not None and (
                segment.transition.target_pose is not None):
            source = segment.transition.source_pose
            target = segment.transition.target_pose
            pose_distance = math.hypot(
                target.x - source.x, target.y - source.y)
            if self._topology.areas[
                    segment.transition.source_area].map_name == segment.target.map_name:
                if abs(pose_distance - distance) > 0.5:
                    self._fail(
                        'Calibrated connector distance conflicts with endpoint poses')
                    return
                if expected_height is None:
                    expected_height = target.z - source.z
        self._connector = ConnectorTraversal(
            distance_m=distance,
            speed_mps=segment.assessment.speed_limit_mps,
            expected_height_delta_m=float(expected_height or 0.0),
            lateral_limit_m=self._connector_lateral_limit,
            yaw_limit_deg=self._connector_yaw_limit)
        result = self._connector.begin(
            time.monotonic(), self._odom, self._dog_health)
        if result.state == 'failed':
            self._fail(result.message, latch_stop=True)
            return
        self._publish_zero()
        self._connector_active_publisher.publish(Bool(data=True))
        self._set_stage('traverse_connector', result.message, timeout=600.0)

    def _publish_zero(self):
        self._velocity_publisher.publish(Twist())

    def _request_relocalization(self, segment):
        self._relocalization_id = f'{self._mission_id}:{self._segment_index}'
        message = RelocalizationRequest()
        message.map_name = segment.target.map_name
        message.request_id = self._relocalization_id
        message.rough_pose = PoseWithCovarianceStamped()
        message.rough_pose.header.frame_id = 'map'
        message.rough_pose.header.stamp = self.get_clock().now().to_msg()
        pose = segment.transition.target_pose
        message.rough_pose.pose.pose.position.x = pose.x
        message.rough_pose.pose.pose.position.y = pose.y
        message.rough_pose.pose.pose.position.z = pose.z
        message.rough_pose.pose.pose.orientation.z = math.sin(pose.yaw * 0.5)
        message.rough_pose.pose.pose.orientation.w = math.cos(pose.yaw * 0.5)
        self._relocalization_publisher.publish(message)
        self._set_stage(
            'wait_relocalization',
            'Refining pose against target 3D map', timeout=180.0)

    def _advance_segment(self):
        self._segment_index += 1
        self._set_stage('prepare_segment', 'Preparing next campus transition')

    def _tick(self):
        with self._lock:
            if not self._active:
                return
            now = time.monotonic()
            if self._deadline is not None and now > self._deadline:
                self._fail(
                    f'Mission stage timed out: {self._stage}',
                    latch_stop=(self._stage == 'traverse_connector'))
                return
            if self._consume_future() or not self._active:
                return
            segment = self._current_segment()

            if self._stage == 'prepare_segment':
                if not self._health_ready():
                    self._fail('Fresh healthy standing robot status is required')
                    return
                if segment is None:
                    if self._has_final_pose:
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.pose = self._final_pose
                        self._call_goal(pose, require_yaw=False, next_stage='complete')
                    else:
                        self._finish('completed', 'Campus mission completed')
                    return
                expected_map = self._topology.areas[
                    segment.transition.source_area].map_name
                if self._current_map != expected_map:
                    self._fail(f'Active map {self._current_map} does not match {expected_map}')
                    return
                self._call_goal(
                    self._pose_stamped(segment.transition.source_pose),
                    require_yaw=True, next_stage='at_connector')
                return

            if self._stage == 'wait_goal':
                terminal = self._goal_terminal()
                if terminal is None:
                    return
                result, detail = terminal
                if result != 'succeeded':
                    self._fail(f'Nav2 mission goal {result}: {detail}')
                    return
                next_stage = self._stage_after_goal
                if next_stage == 'complete':
                    self._finish('completed', 'Campus mission and final goal completed')
                else:
                    self._set_stage(next_stage, detail)
                return

            if self._stage == 'at_connector':
                mode = segment.assessment.required_mode or 'normal'
                source_map = self._topology.areas[
                    segment.transition.source_area].map_name
                if mode in self.SPECIAL_MODES or source_map != segment.target.map_name:
                    self._call_mode(mode, 'begin_connector')
                else:
                    self._call_goal(
                        self._pose_stamped(segment.transition.target_pose),
                        require_yaw=False, next_stage='connector_complete')
                return

            if self._stage == 'begin_connector':
                self._start_connector(segment)
                return

            if self._stage == 'traverse_connector':
                terrain_ready, terrain_message = self._live_terrain_ready(
                    segment)
                if not terrain_ready:
                    self._fail(terrain_message, latch_stop=True)
                    return
                result = self._connector.update(now, self._odom, self._dog_health)
                command = Twist()
                command.linear.x = result.linear_x
                command.angular.z = result.angular_z
                self._velocity_publisher.publish(command)
                self._message = result.message
                self._publish_status()
                if result.state == 'failed':
                    self._fail(result.message, latch_stop=True)
                elif result.state == 'completed':
                    self._publish_zero()
                    self._connector_active_publisher.publish(Bool(data=False))
                    self._call_mode('normal', 'connector_complete')
                return

            if self._stage == 'connector_complete':
                source_map = self._topology.areas[
                    segment.transition.source_area].map_name
                target_map = segment.target.map_name
                if source_map != target_map:
                    self._call_map(target_map)
                else:
                    self._advance_segment()
                return

            if self._stage == 'wait_terrain':
                if self._current_map != segment.target.map_name:
                    return
                if self._terrain_status.get('state') != 'ready':
                    return
                if self._terrain_received_at <= self._map_switch_started_at:
                    return
                if str(self._terrain_status.get('map_name', '')) != segment.target.map_name:
                    return
                self._request_relocalization(segment)
                return

            if self._stage == 'wait_relocalization':
                status = self._relocalization_status
                if str(status.get('request_id', '')) != self._relocalization_id:
                    return
                if not bool(status.get('terminal', False)):
                    self._message = str(status.get('message', self._message))
                    self._publish_status()
                    return
                if not bool(status.get('success', False)):
                    self._fail(f'3D relocalization failed: {status.get("message", "unknown") }')
                    return
                self._advance_segment()

    def _fail(self, message, *, latch_stop=False):
        self._publish_zero()
        self._connector_active_publisher.publish(Bool(data=False))
        self._cancel_navigation_publisher.publish(Empty())
        if self._current_mode != 'normal' and self._mode_client.service_is_ready():
            request = SetTraversalMode.Request()
            request.mode = 'normal'
            self._mode_client.call_async(request)
            self._current_mode = 'normal'
        if latch_stop:
            self._stop_publisher.publish(Empty())
        self._finish('failed', message)

    def _finish(self, state, message):
        self._active = False
        self._state = str(state)
        self._stage = 'idle'
        self._message = str(message)
        self._deadline = None
        self._pending_future = None
        self._pending_kind = ''
        self._connector = None
        if self._current_mode != 'normal' and self._mode_client.service_is_ready():
            request = SetTraversalMode.Request()
            request.mode = 'normal'
            self._mode_client.call_async(request)
            self._current_mode = 'normal'
        self._publish_zero()
        self._connector_active_publisher.publish(Bool(data=False))
        self._active_publisher.publish(Bool(data=False))
        self._publish_status(force=True)

    def _publish_status(self, force=False):
        segment = self._current_segment()
        total = len(self._plan.segments) if self._plan is not None else 0
        payload = {
            'active': self._active,
            'state': self._state,
            'stage': self._stage,
            'mission_id': self._mission_id,
            'segment_index': self._segment_index,
            'segment_count': total,
            'transition_id': (
                segment.transition.transition_id if segment is not None else ''),
            'current_map': self._current_map,
            'topology_revision': self._topology_revision,
            'live_terrain_type': (
                self._live_terrain.terrain_type
                if self._live_terrain is not None else 'unknown'),
            'live_terrain_age_sec': (
                max(0.0, time.monotonic() - self._live_terrain_received_at)
                if self._live_terrain_received_at > 0.0 else -1.0),
            'message': self._message,
        }
        self._status_publisher.publish(String(
            data=json.dumps(payload, ensure_ascii=False, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = CampusMissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
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
