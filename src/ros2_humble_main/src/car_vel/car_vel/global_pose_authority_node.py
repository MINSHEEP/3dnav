"""Xingji 3D global anchor. Other scopes forward the original AMCL message.

Only this node owns map->odom while AMCL's broadcast parameter is confirmed
off. A failed/late parameter RPC never authorizes a second TF publisher.
"""
import json
import math
from collections import deque
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lzp_interface.action import Relocalize
from lzp_interface.msg import CurrentMap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty as EmptyService
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from .global_pose_authority import (
    GlobalPoseAuthority, LioAnchorContinuity, LioStreamGuard, Pose2, compose, angle,
    RUNTIME_TIMING_MODEL)
from .rigid_pose import Pose3, TRANSFORM_MODEL, quaternion


def pose2(pose):
    q = pose.orientation
    norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if not math.isfinite(norm) or abs(norm-1.) > .02:
        raise ValueError('invalid orientation')
    return Pose3.from_quaternion(float(pose.position.x), float(pose.position.y),
        float(getattr(pose.position, 'z', 0.)), q.x, q.y, q.z, q.w)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class GlobalPoseAuthorityNode(Node):
    def __init__(self):
        super().__init__('global_pose_authority')
        coast = float(self.declare_parameter('matching_coast_sec', 3.0).value)
        recovery = float(self.declare_parameter('matching_recovery_sec', 2.0).value)
        tracking_rmse = float(self.declare_parameter('tracking_maximum_rmse', .22).value)
        self.tracking_timeout = float(self.declare_parameter('tracking_timeout_sec', 2.0).value)
        if (not all(math.isfinite(v) for v in (coast, recovery, tracking_rmse))
                or not 1. <= coast <= 4. or not 0. < recovery <= 2.
                or not 0. < tracking_rmse <= .25
                or not math.isfinite(self.tracking_timeout) or not .5 <= self.tracking_timeout <= 2.5):
            raise ValueError('三维跟踪期限或质量参数无效')
        self.policy = GlobalPoseAuthority(coast=coast,recovery=recovery,tracking_rmse=tracking_rmse)
        self.mode = self.body = ''
        self.body_generation = None
        self.mode_at = self.body_at = self.lio_at = 0.
        self.lio_ok = False
        self.lio_continuity = LioAnchorContinuity()
        self.lio_epoch = 0
        self.map_name = self.load_id = ''
        self.active = True
        self.scope = False
        self.scope_known = False
        self.was_ready = False
        self.was_recovering = False
        self.motion_latched = False
        self.failure = None
        self.raw_pose = None
        self.last_raw_stamp = None
        self.last_pose_stamp = None
        # Used only in the Xingji3D scope; legacy pose forwarding is unchanged.
        self.lio_stream = LioStreamGuard(freshness_sec=.45)
        self.stream_identity = None
        self.seed_request = ''
        self.initial_request_id = ''
        self.legacy_required = False
        self.legacy_wait = None
        self.registration_busy = False
        self.tracking = None
        self.tracking_handle = None
        self.next_match = 0.
        self.error = ''
        self.tf_fault = False
        self.own_tf_gid = None
        self.own_tf_messages = deque(maxlen=64)
        self.external_tf_at = 0.
        self.handoff_at = 0.
        self.actual_tf = None
        self.baseline = None
        self.amcl_gid = ''
        self.param_future = None
        self.param_operation = None
        self.param_started = self.last_read = self.last_owned_tf = 0.
        self.record = None
        self.record_bad = False
        path = str(self.declare_parameter('ownership_path', str(
            Path.home()/'.ros/robot_navigation/global_pose_authority.json')).value)
        self.record_path = Path(path)
        try:
            if self.record_path.exists():
                self.record = json.loads(self.record_path.read_text(encoding='utf-8'))
                if (self.record.get('version') != 1 or type(self.record.get('baseline')) is not bool
                        or not self.record.get('gid')):
                    raise ValueError('invalid ownership record')
        except (OSError, ValueError, TypeError) as error:
            self.record_bad = True
            self.error = f'定位TF权限记录不可用，请停止后台后检查：{error}'
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
            '/localization/navigation_pose', 10)
        self.status_pub = self.create_publisher(String, '/localization/status', latched)
        self.interval_policy_pub = self.create_publisher(
            Bool, '/localization/interval_compensation', latched)
        self.interval_policy_pub.publish(Bool(data=False))
        self.create_subscription(String, '/navigation/mode', self._mode, latched)
        self.create_subscription(String, '/robot_body/status', self._body, latched)
        self.create_subscription(String, '/lio_health', self._lio, latched)
        self.create_subscription(CurrentMap, '/current_map_info', self._map, latched)
        self.create_subscription(Bool, '/navigation_active', self._active, latched)
        amcl_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._raw, amcl_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._legacy_initial, 10)
        self.nomotion = self.create_client(EmptyService, '/request_nomotion_update')
        self.create_subscription(String, '/relocalization/status', self._seed, latched)
        self.create_subscription(TFMessage, '/tf', self._tf_observation, 100)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.read = self.create_client(GetParameters, '/amcl/get_parameters')
        self.write = self.create_client(SetParametersAtomically, '/amcl/set_parameters_atomically')
        self.action = ActionClient(self, Relocalize, '/relocalize')
        self.last_status = None
        self.create_timer(.05, self._tick)

    def _mode(self, message):
        try:
            mode = json.loads(message.data)['mode']
            if mode not in ('map_2d', 'terrain_3d'):
                raise ValueError('unknown mode')
            self.mode = mode
        except (ValueError, KeyError, TypeError, AttributeError):
            self.mode = ''
        self.mode_at = time.monotonic()

    def _body(self, message):
        try:
            data = json.loads(message.data)
            selected = data['config']['selected']
            state = data.get('status') or {}
            profile = data.get('config', {}).get('profiles', {}).get(selected, {})
            valid = (selected in ('xingji', 'gangben_zsl1') and data.get('active_body') == selected
                     and data.get('connected') is True and not data.get('switching')
                     and not data.get('error') and not state.get('stale')
                     and not state.get('emergency_latched') and not state.get('hardware_error')
                     and not state.get('error') and profile.get('navigation_verified') is True)
            generation = data.get('generation')
            self.body = selected if valid else ''
            if generation != self.body_generation:
                self._capture_failure('body_generation_changed',
                    f'本体配置批次变化：{self.body_generation} -> {generation}')
                self.policy.reset()
            self.body_generation = generation
        except (ValueError, KeyError, TypeError, AttributeError):
            self.body = ''
        self.body_at = time.monotonic()

    def _lio(self, message):
        data = None
        # Five healthy scans can take1.48s in the recorded delayed stream.
        # This keeps the anchor only; native motion_pause still stops motion.
        self.lio_continuity.max_pause_sec = (
            3.0 if self.mode == 'terrain_3d' and self.body == 'xingji' else 1.0)
        try:
            data = json.loads(message.data)
            ok = self.lio_continuity.observe(data, time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.lio_continuity.observe(None, time.monotonic())
            ok = False
        if not ok and self.lio_ok:
            reason = (data.get('reason') or data.get('code') or data.get('state')) if isinstance(data, dict) else 'invalid_health_message'
            self._capture_failure('lio_health_rejected', f'LIO健康检查未通过：{reason}',
                                  lio_health=data)
            self.lio_epoch += 1
            self.policy.reset()
        self.lio_ok = ok
        self.last_lio_health = data
        self.lio_at = time.monotonic()

    def _map(self, message):
        identity = (message.map_name, message.load_id) if message.success else ('', '')
        reload = str(message.message).startswith((
            'Successfully loaded and confirmed map:', 'Successfully loaded map:'))
        if identity != (self.map_name, self.load_id) or reload:
            self._capture_failure('map_identity_changed',
                f'地图加载批次变化：{self.map_name}/{self.load_id} -> {identity}')
            self.policy.reset()
            self.seed_request = ''
            self.initial_request_id = ''
            self.legacy_wait = None
        self.map_name, self.load_id = identity

    def _active(self, message):
        self.active = message.data is not False

    def _raw(self, message):
        self.raw_pose = message
        if self.legacy_required and not self.scope:
            self._confirm_legacy(message)
        # Pass through unchanged in legacy scopes; never fabricate a new stamp.
        if self.scope_known and not self.scope and self._legacy_ready():
            self.pose_pub.publish(message)
            self.last_raw_stamp = stamp_seconds(message.header.stamp)

    def _legacy_ready(self):
        return (self.baseline is not None and self.actual_tf == self.baseline
                and (self.param_future is None or self.param_operation == 'read')
                and not self.record_bad
                and not self.legacy_required
                and not (self.record or {}).get('pending'))

    def _legacy_initial(self, message):
        if not self.legacy_required or self.scope:
            return
        try:
            odom, stamp = self._odom()
            if (message.header.frame_id.lstrip('/') != 'map'
                    or abs(self.get_clock().now().nanoseconds*1e-9-stamp_seconds(stamp)) > .3):
                return
            self.legacy_wait = dict(expected=pose2(message.pose.pose), odom=odom,
                identity=(self.map_name, self.load_id, self.body_generation),
                reference_stamp=stamp_seconds(message.header.stamp), last_stamp=0., count=0,
                deadline=time.monotonic()+5., next_update=0., updates=0)
            self.error = '正在核对原AMCL位置，尚未恢复其TF发布'
        except Exception as error:
            self.legacy_wait = None
            self.error = f'切回原定位需要重新设置初始点：{error}'

    def _confirm_legacy(self, message):
        wait = self.legacy_wait
        if not wait or self.active or time.monotonic() > wait['deadline']:
            return
        if wait['identity'] != (self.map_name, self.load_id, self.body_generation):
            self.legacy_wait = None
            return
        try:
            mapped = pose2(message.pose.pose)
            stamp = stamp_seconds(message.header.stamp)
            odom, odom_stamp = self._odom()
            expected, reference = wait['expected'], wait['odom']
            covariance = message.pose.covariance
            valid = (stamp > max(wait['reference_stamp'], wait['last_stamp'])
                and abs(self.get_clock().now().nanoseconds*1e-9-stamp) <= .5
                and abs(self.get_clock().now().nanoseconds*1e-9-stamp_seconds(odom_stamp)) <= .3
                and math.hypot(odom.x-reference.x, odom.y-reference.y) <= .03
                and abs(angle(odom.yaw-reference.yaw)) <= math.radians(3)
                and math.hypot(mapped.x-expected.x, mapped.y-expected.y) <= .3
                and abs(angle(mapped.yaw-expected.yaw)) <= math.radians(10)
                and all(math.isfinite(float(covariance[i])) and 0 <= covariance[i] <= limit
                        for i,limit in ((0,.25),(7,.25),(35,math.radians(20)**2))))
            wait['count'] = wait['count']+1 if valid else 0
            wait['last_stamp'] = max(wait['last_stamp'], stamp)
            if wait['count'] >= 3:
                self.legacy_required = False
                self.legacy_wait = None
                self.error = ''
        except Exception:
            wait['count'] = 0

    def _legacy_tick(self, wall):
        wait = self.legacy_wait
        if not self.legacy_required or self.scope or not wait:
            return
        if wall > wait['deadline']:
            self.legacy_wait = None
            self.error = 'AMCL位置同步未通过；保持停止，请在原导航模式重新设置初始点'
        elif wall >= wait['next_update'] and wait['updates'] < 8 and self.nomotion.service_is_ready():
            wait['next_update'] = wall+.35
            wait['updates'] += 1
            self.nomotion.call_async(EmptyService.Request())

    def _odom(self, stamp=None):
        transform = self.buffer.lookup_transform('odom', 'base_footprint', stamp or Time())
        t, q = transform.transform.translation, transform.transform.rotation
        if (not all(math.isfinite(v) for v in (t.x,t.y,t.z,q.x,q.y,q.z,q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .02):
            raise ValueError('LIO transform contains invalid coordinates or rotation')
        value = Pose3.from_quaternion(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        return value, transform.header.stamp

    def _seed(self, message):
        try:
            data = json.loads(message.data)
            self.registration_busy = not data.get('terminal', False) and data.get('state') != 'idle'
            if (self.scope and data.get('stage') == 'stop_robot'
                    and data.get('map_name') == self.map_name
                    and data.get('map_load_id') == self.load_id
                    and data.get('body_generation') == self.body_generation):
                self.initial_request_id = str(data.get('request_id') or '')
                self.seed_request = ''
                self.error = '正在等待本次初始精修结果'
                self.lio_epoch += 1
                self.policy.set_identity((self.body, self.body_generation, self.map_name,
                                          self.load_id, self.lio_epoch, self.mode))
                if self.tracking_handle is not None:
                    self.tracking_handle.cancel_goal_async()
                return
            if (self.scope and not self.active and self.initial_request_id
                    and data.get('request_id') == self.initial_request_id
                    and data.get('map_name') == self.map_name
                    and data.get('map_load_id') == self.load_id
                    and data.get('body_generation') == self.body_generation
                    and data.get('terminal') is True and data.get('success') is False
                    and data.get('state') in ('failed', 'canceled')):
                self.policy.reset()
                self.seed_request = ''
                self.error = '初始精修未确认：' + str(data.get('message') or '请求失败或取消')
                return
            if (not self.scope or self.active or data.get('stage') != 'validation_passed'
                    or data.get('source_pose') != 'refined'
                    or data.get('map_name') != self.map_name
                    or data.get('map_load_id') != self.load_id
                    or data.get('body_generation') != self.body_generation
                    or not self.initial_request_id
                    or data.get('request_id') != self.initial_request_id
                    or data.get('candidate_verified') is not True):
                return
            stamp = data['pose_stamp']
            seconds = float(stamp['sec'])+float(stamp['nanosec'])*1e-9
            odom, _ = self._odom(Time(seconds=int(stamp['sec']), nanoseconds=int(stamp['nanosec'])))
            p, q = data['pose'], data['metrics']
            if q.get('transform_model') != TRANSFORM_MODEL:
                self.error = '三维配准模型版本不匹配，请重建接口和icp_registration并重启后台'
                return
            rotation = p['orientation']
            mapped = Pose3.from_quaternion(float(p['x']), float(p['y']), float(p['z']),
                *(float(rotation[k]) for k in ('x', 'y', 'z', 'w')))
            if self.tracking_handle is not None:
                self.tracking_handle.cancel_goal_async()
            self.lio_epoch += 1
            self.policy.set_identity((self.body, self.body_generation, self.map_name,
                                      self.load_id, self.lio_epoch, self.mode))
            if self.policy.accept(mapped, odom, stamp=seconds,
                    now=self.get_clock().now().nanoseconds*1e-9, rmse=float(q['fitness_m']),
                    overlap=float(q['overlap']), ambiguous=False, initial=True):
                self.seed_request = data['request_id']
                self.tf_fault = False
                self.motion_latched = False
                self.failure = None
                self.was_ready = False
                self.error = ''
        except Exception as error:
            self.error = f'三维锚点未接收：{error}'

    @staticmethod
    def _tf_fingerprint(transform):
        t, q = transform.transform.translation, transform.transform.rotation
        stamp = transform.header.stamp
        return (stamp.sec, stamp.nanosec, t.x,t.y,t.z,q.x,q.y,q.z,q.w)

    def _tf_observation(self, message):
        # Humble's Python callback exposes only the message, not publisher_gid.
        # Ignore only an exact bounded echo of a verified TF we just sent,
        # including its timestamp and full transform, never just its position.
        try:
            for transform in message.transforms:
                if (transform.header.frame_id.lstrip('/') != 'map'
                        or transform.child_frame_id.lstrip('/') != 'odom'):
                    continue
                if self._tf_fingerprint(transform) in self.own_tf_messages:
                    continue
                self.external_tf_at = time.monotonic()
                if self.scope and self.last_owned_tf and self.actual_tf is False:
                    self.tf_fault = True
                    self.policy.reset()
                    self.error = '检测到未经本节点发送的map→odom，已停止三维定位输出'
        except (AttributeError, TypeError, RuntimeError):
            self.tf_fault = True

    def _ensure_own_tf_gid(self):
        if self.own_tf_gid is not None:
            return True
        own = {bytes(e.endpoint_gid) for e in self.get_publishers_info_by_topic('/tf')
               if e.node_name == self.get_name() and e.node_namespace == self.get_namespace()
               and any(e.endpoint_gid)}
        if len(own) > 1:
            self.tf_fault = True
            self.error = '发现多个统一定位实例，未取得唯一TF发布权'
        if len(own) == 1:
            self.own_tf_gid = next(iter(own))
            return True
        return False

    def _save(self, pending):
        self.record_path.parent.mkdir(parents=True, exist_ok=True)
        value = dict(version=1, gid=self.amcl_gid, baseline=self.baseline, pending=pending)
        temporary = self.record_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(value), encoding='utf-8')
        temporary.replace(self.record_path)
        self.record = value

    def _sync_tf_parameters(self, now):
        # rclpy.Node owns self._parameters as its parameter dictionary. Keep
        # the ownership RPC state machine out of that inherited namespace.
        endpoints = self.get_publishers_info_by_topic('/parameter_events')
        gids = {bytes(e.endpoint_gid).hex() for e in endpoints
                if e.node_name == 'amcl' and e.node_namespace == '/' and any(e.endpoint_gid)}
        gid = next(iter(gids)) if len(gids) == 1 else ''
        if not gid:
            self.actual_tf = None
            return  # Discovery silence does not prove an in-flight SET was canceled.
        if gid != self.amcl_gid:
            self.actual_tf = None
            self.policy.reset()
            self.amcl_gid = gid
            self.last_owned_tf = 0.
            if self.param_future:
                # A different publisher GID proves the old lifecycle node is gone.
                self.param_future.cancel()
                self.param_future = None
            if self.record and self.record.get('gid') == gid:
                self.baseline = self.record['baseline']
                if self.record.get('pending'):
                    self.record_bad = True
                    self.error = '上次定位TF参数写入结果未决，请重启整个导航后台'
            else:
                self.baseline = None
        if not gid or self.record_bad:
            return
        if self.param_future:
            if not self.param_future.done():
                if now-self.param_started > 3.:
                    self.actual_tf = None
                    self.error = '定位TF参数响应超时，等待原请求完成；未启用第二个TF发布者'
                    if self.param_operation == 'read':
                        self.param_future.cancel()
                        self.param_future = None
                return
            future, self.param_future = self.param_future, None
            try:
                response = future.result()
                if self.param_operation == 'read':
                    if len(response.values) != 1 or response.values[0].type != ParameterType.PARAMETER_BOOL:
                        raise ValueError('AMCL tf_broadcast not a boolean')
                    self.actual_tf = response.values[0].bool_value
                    if self.baseline is None:
                        self.baseline = self.actual_tf
                    self.last_read = now
                    if self.scope or self.record is not None:
                        self._save(False)
                elif not response.result.successful:
                    raise ValueError(response.result.reason)
                else:
                    self.actual_tf = None  # Successful SET still requires GET.
            except Exception as error:
                self.actual_tf = None
                if self.param_operation == 'set':
                    self.record_bad = True
                self.error = f'定位TF参数确认失败：{error}'
                return
        desired = False if self.scope else self.baseline
        if not self.scope and self.legacy_required:
            desired = self.actual_tf  # Readback is allowed; enabling AMCL TF is not.
        if self.actual_tf is None or now-self.last_read > 1.:
            if not self.read.service_is_ready():
                return
            self.param_operation = 'read'
            self.param_future = self.read.call_async(GetParameters.Request(names=['tf_broadcast']))
        elif self.scope_known and desired is not None and self.actual_tf != desired:
            # An ownership change occurs only at rest; own TF stops before restoration.
            if self.active or now-self.last_owned_tf < .7 or not self.write.service_is_ready():
                return
            self._save(True)
            request = SetParametersAtomically.Request(parameters=[Parameter(name='tf_broadcast',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=desired))])
            self.param_operation = 'set'
            self.param_future = self.write.call_async(request)
            self.handoff_at = now
        if self.param_future:
            self.param_started = now

    def _match(self, pose, stamp, now):
        if (self.tracking is not None or self.registration_busy or now < self.next_match
                or not self.action.server_is_ready() or self.policy.transform is None
                or self.policy.hard_failure or self.motion_latched):
            return
        goal = Relocalize.Goal()
        goal.map_name, goal.map_load_id = self.map_name, self.load_id
        goal.robust_validation = goal.tracking = True
        # Filter candidates against the same acceptance envelope before the
        # registration server ranks them, rather than discarding its winner.
        goal.search_xy, goal.search_yaw = self.policy.max_xy, self.policy.max_yaw
        goal.timeout_sec = self.tracking_timeout
        goal.rough_pose = self._message(compose(self.policy.transform, pose), stamp)
        identity = self.policy.identity
        future = self.action.send_goal_async(goal)
        self.tracking = future
        self.next_match = now+.75

        def accepted(response):
            try:
                handle = response.result()
                if not handle.accepted:
                    self.policy.match_failed()
                    self.tracking = None
                    return
                self.tracking_handle = handle
                result_future = handle.get_result_async()
                self.tracking = result_future
                result_future.add_done_callback(lambda f: self._match_result(f, identity))
                if identity != self.policy.identity:
                    handle.cancel_goal_async()
            except Exception as error:
                self.policy.match_failed()
                self.error = f'局部匹配请求失败：{error}'
                self.tracking = None
        future.add_done_callback(accepted)

    def _match_result(self, future, identity):
        self.tracking = self.tracking_handle = None
        try:
            wrapped = future.result()
            result = wrapped.result
            if not self.scope or identity != self.policy.identity:
                return
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success
                    or not result.candidate_verified):
                self.policy.match_failed()
                self.error = f'局部三维匹配未通过：{result.message}'
                return
            if getattr(result, 'transform_model', '') != TRANSFORM_MODEL:
                self.policy.match_failed()
                self.error = '局部三维配准模型版本不匹配，请重建接口和icp_registration并重启后台'
                return
            stamp = result.refined_pose.header.stamp
            odom, _ = self._odom(Time(seconds=stamp.sec, nanoseconds=stamp.nanosec))
            if self.policy.accept(pose2(result.refined_pose.pose.pose), odom,
                    stamp=stamp_seconds(stamp), now=self.get_clock().now().nanoseconds*1e-9,
                    rmse=float(result.fitness), overlap=float(result.overlap), ambiguous=False):
                self.error = ''
            else:
                self.error = f'局部三维修正被拒绝：{self.policy.reason}'
        except Exception as error:
            self.policy.match_failed()
            self.error = f'局部三维匹配结果无效：{error}'

    @staticmethod
    def _message(pose, stamp):
        message = PoseWithCovarianceStamped()
        message.header.frame_id, message.header.stamp = 'map', stamp
        p, q = message.pose.pose.position, message.pose.pose.orientation
        p.x, p.y, p.z = pose.x, pose.y, getattr(pose, 'z', 0.)
        q.x, q.y, q.z, q.w = quaternion(pose)
        for index in (0, 7, 14):
            message.pose.covariance[index] = .04
        for index in (21, 28, 35):
            message.pose.covariance[index] = .02
        return message

    def _capture_failure(self, code, detail, *, lio_health=None):
        """Freeze first loss before reset/generic lock can destroy evidence.

        Diagnostic only: never changes readiness, recovery budgets or motion.
        Only a newly validated manual initial pose clears this record.
        """
        if (not self.scope or (not self.active and self.policy.transform is None) or
                getattr(self, 'failure', None) is not None):
            return
        def safe(value):
            if isinstance(value, dict):
                return {str(k): safe(v) for k, v in value.items()}
            if isinstance(value, list):
                return [safe(v) for v in value]
            return None if isinstance(value, float) and not math.isfinite(value) else value
        clock = self.get_clock().now().nanoseconds * 1e-9
        self.failure = dict(code=code, detail=str(detail), stamp_ns=int(clock*1e9),
            map_name=self.map_name, map_load_id=self.load_id,
            body_generation=self.body_generation, lio_epoch=self.lio_epoch,
            policy_reason=self.policy.reason, accepted_stamp=self.policy.accepted_stamp,
            accepted_age_sec=(clock-self.policy.accepted_stamp
                              if self.policy.accepted_stamp else None),
            rmse_m=self.policy.rmse, overlap=self.policy.overlap,
            recovery_deadline=self.policy.recovery_deadline,
            recovery_matches=self.policy.recovery_samples,
            recovery_matches_required=self.policy.recovery_matches_required if self.scope else 0,
            lio_timing=dict(self.lio_stream.details()),
            lio_health=safe(lio_health if lio_health is not None
                            else getattr(self, 'last_lio_health', None)),
            lio_health_pause_limit_sec=self.lio_continuity.max_pause_sec,
            tf_broadcast_actual=self.actual_tf, tf_fault=self.tf_fault)
        self.get_logger().error('LOCALIZATION_FAILURE ' + json.dumps(
            self.failure, ensure_ascii=False))

    def _motion_failure_message(self):
        evidence = getattr(self, 'failure', None) or {}
        detail = evidence.get('detail') or self.policy.reason
        return ('运行中三维定位失效，已锁定零速；首次原因：'
                + str(evidence.get('code') or 'localization_untrusted') + '；'
                + str(detail) + '；停止任务后重新确认初始点')

    def _tick(self):
        wall = time.monotonic()
        clock = self.get_clock().now().nanoseconds*1e-9
        # Original 2D pose forwarding/restoration has no dependency on a body
        # connection. Xingji 3D TF ownership still requires every body check.
        self.scope_known = (wall-self.mode_at < 2. and (
            self.mode == 'map_2d' or (wall-self.body_at < 2.
            and bool(self.mode) and bool(self.body))))
        # Freshness gates readiness, not the selected localization owner.
        # A late heartbeat must not masquerade as a real switch to legacy.
        wanted = (self.mode == 'terrain_3d' and self.body == 'xingji'
                  and (self.scope or self.scope_known))
        legacy_seed = None
        if wanted != self.scope:
            self._capture_failure('localization_scope_changed',
                f'定位模式变化：{self.mode}，本体：{self.body}')
            if self.scope:
                self.legacy_required = True
                self.legacy_wait = None
                self.error = '切回原定位前需要核对AMCL位置，请保持停止并设置初始点'
                # A fresh same-body anchor may initialize the legacy filter at rest.
                if (self.mode == 'map_2d' and self.body == 'xingji' and not self.active
                        and self.policy.ready(clock) and not self.motion_latched
                        and self.policy.recovery_deadline is None and not self.policy.hard_failure):
                    try:
                        odom, stamp = self._odom()
                        if abs(clock-stamp_seconds(stamp)) <= .3:
                            legacy_seed = self._message(compose(self.policy.transform, odom), stamp)
                    except Exception:
                        pass
            self.policy.reset()
            self.seed_request = ''
            self.was_ready = False
            self.last_pose_stamp = None
            self.last_owned_tf = wall if self.scope else self.last_owned_tf
        self.scope = wanted
        self.lio_continuity.max_pause_sec = 3.0 if self.scope else 1.0
        if legacy_seed is not None:
            self._legacy_initial(legacy_seed)
            self.initial_pose_pub.publish(legacy_seed)
        if self.lio_ok and not self.lio_continuity.allows_anchor(wall):
            self._capture_failure('lio_health_continuity_expired',
                                  'LIO健康状态连续性复核未通过或超过允许时长')
            self.lio_ok = False
            self.lio_epoch += 1
            self.policy.reset()
        identity = (self.body,self.body_generation,self.map_name,self.load_id,self.lio_epoch,self.mode)
        if identity != self.stream_identity:
            self.lio_stream.reset()
            self.stream_identity = identity
        self.policy.set_identity(identity)
        try:
            self._sync_tf_parameters(wall)
        except Exception as error:
            self.actual_tf = None
            self.error = f'定位TF权限同步失败：{error}'
        try:
            self._legacy_tick(wall)
        except Exception as error:
            self.error = f'切回原定位等待确认：{error}'
        ready = False
        recovering = False
        sensor_fresh = False
        if self.scope:
            try:
                odom = stamp = None
                stream = self.lio_stream
                anchored = self.policy.transform is not None
                try:
                    odom,stamp = self._odom()
                    stream.observe(odom,stamp_seconds(stamp),clock,wall,anchored)
                except (ValueError,TypeError,AttributeError) as error:
                    stream.lookup_error = str(error)
                    self._capture_failure('invalid_lio_pose', str(error))
                    stream.fail('invalid_lio_pose')
                except Exception as error:
                    stream.missing(clock,wall,anchored,error)
                sensor_fresh = stream.fresh
                if stream.failed:
                    self._capture_failure(stream.reason,
                        'LIO连续性检查失败：' + json.dumps(stream.details(), ensure_ascii=False))
                    self.lio_epoch += 1
                    self.policy.set_identity((self.body,self.body_generation,self.map_name,
                                              self.load_id,self.lio_epoch,self.mode))
                    self.error = 'LIO连续性检查失败，旧锚点已作废：'+json.dumps(stream.details(),ensure_ascii=False)
                elif stream.requires_match and anchored:
                    # An actual sample gap still requires a fresh measured map
                    # match. Its deadline belongs to map evidence, not to the
                    # independently verified LIO transport recovery window.
                    deadline = self.policy.accepted_stamp+self.policy.coast+self.policy.recovery
                    self.policy.begin_recovery(clock,
                        min(stream.recovery,max(0.,deadline-clock)),stream.match_min_stamp)
                    stream.requires_match = False
                if self.lio_continuity.paused and anchored:
                    self.policy.begin_recovery(clock,3.,clock,
                        require_matches=not self.policy.ready(clock))
                if not self.scope_known and anchored:
                    self.policy.begin_recovery(clock,3.,clock)
                lio = (self.lio_ok and wall-self.lio_at < 2.5
                       and self.lio_continuity.allows_anchor(wall))
                owned = (self.actual_tf is False and self.param_operation != 'set'
                         and self._ensure_own_tf_gid()
                         and wall-max(self.external_tf_at, self.handoff_at) > .7
                         and not self.tf_fault and not self.record_bad)
                infrastructure = (owned and lio and bool(self.load_id) and not stream.failed
                                  and (sensor_fresh or stream.recovering))
                phase = self.policy.phase(clock,can_resume=(self.scope_known and stream.stream_ready
                    and not self.lio_continuity.paused)) if infrastructure else 'failed'
                if phase == 'ready' and stream.recovering:
                    # A good map anchor never substitutes for the three new
                    # LIO samples required to release a transport hold.
                    phase = 'recovering'
                ready = phase == 'ready'
                recovering = phase == 'recovering'
                if ((self.was_ready or self.was_recovering) and self.active
                        and not ready and not recovering):
                    if not owned:
                        code = 'tf_ownership_unconfirmed'
                    elif not lio:
                        code = 'lio_health_untrusted'
                    elif self.policy.reason == 'recovery_timeout':
                        code = 'matching_recovery_timeout'
                    else:
                        code = self.policy.reason or 'global_match_stale'
                    self._capture_failure(code, self.error or self.policy.reason)
                    self.motion_latched = True
                if self.motion_latched:
                    ready = False
                    recovering = False
                    self.error = self._motion_failure_message()
                if lio and stream.stream_ready and owned:
                    self._match(odom, stamp, wall)
                # A bounded recovery keeps measured relative TF available to
                # Nav2, while ready=false independently blocks final velocity.
                if ((ready or recovering) and sensor_fresh and stamp is not None
                        and stamp_seconds(stamp) != self.last_pose_stamp):
                    mapped = compose(self.policy.transform, odom)
                    message = self._message(mapped, stamp)
                    age = max(0., clock-self.policy.accepted_stamp)
                    for index in (0, 7, 14):
                        message.pose.covariance[index] = (
                            max(.05, self.policy.rmse)**2 + (.10*age)**2)
                    for index in (21, 28, 35):
                        message.pose.covariance[index] = math.radians(3+2*age)**2
                    self.pose_pub.publish(message)
                    t = TransformStamped()
                    t.header.frame_id, t.child_frame_id = 'map', 'odom'
                    t.header.stamp = Time(nanoseconds=int((stamp_seconds(stamp)+.2)*1e9)).to_msg()
                    a = self.policy.transform
                    t.transform.translation.x, t.transform.translation.y = a.x, a.y
                    t.transform.translation.z = getattr(a, 'z', 0.)
                    q = t.transform.rotation
                    q.x, q.y, q.z, q.w = quaternion(a)
                    self.own_tf_messages.append(self._tf_fingerprint(t))
                    self.broadcaster.sendTransform(t)
                    self.last_owned_tf = wall
                    self.last_pose_stamp = stamp_seconds(stamp)
            except Exception as error:
                ready = False
                recovering = False
                if (self.was_ready or self.was_recovering) and self.active:
                    self._capture_failure('localization_exception', str(error))
                    self.motion_latched = True
                self.error = (self._motion_failure_message() if self.motion_latched
                              else f'三维定位等待有效LIO/TF：{error}')
        else:
            ready = self.scope_known and self._legacy_ready()
            if ready and self.raw_pose is not None:
                stamp = stamp_seconds(self.raw_pose.header.stamp)
                if stamp != self.last_raw_stamp:
                    self.pose_pub.publish(self.raw_pose)
                    self.last_raw_stamp = stamp
        data = dict(schema=1, stamp_ns=int(clock*1e9), ready=ready,
            scope='xingji_3d' if self.scope else 'legacy', body_id=self.body,
            transform_model=TRANSFORM_MODEL if self.scope else 'legacy',
            navigation_mode=self.mode, map_name=self.map_name, map_load_id=self.load_id,
            body_generation=self.body_generation, seed_request_id=self.seed_request,
            initial_request_id=self.initial_request_id,
            accepted_stamp=self.policy.accepted_stamp,
            rmse_m=self.policy.rmse, overlap=self.policy.overlap,
            age_sec=(clock-self.policy.accepted_stamp if self.policy.accepted_stamp else None),
            reason=(('identity_recovery' if not self.scope_known else
                     'sensor_recovery' if self.lio_stream.recovering else 'matching_recovery')
                    if recovering else self.policy.reason),
            error=self.error, failure=getattr(self, 'failure', None) if self.scope else None,
            sensor_fresh=sensor_fresh,
            recovering=recovering,
            recovery_remaining_sec=(max(0., self.policy.recovery_deadline-clock)
                                    if recovering and self.policy.recovery_deadline is not None
                                    else max(0., self.lio_stream.deadline-wall)
                                    if recovering and self.lio_stream.recovering else 0.),
            recovery_matches=self.policy.recovery_samples,
            recovery_matches_required=self.policy.recovery_matches_required if self.scope else 0,
            sensor_recovering=recovering and self.lio_stream.recovering,
            sensor_stream_ready=self.lio_stream.stream_ready,
            lio_timing=self.lio_stream.details(),
            lio_quality_pause=self.lio_continuity.paused,
            tf_broadcast_actual=self.actual_tf, tf_broadcast_baseline=self.baseline,
            legacy_initialization_required=self.legacy_required,
            matching=self.tracking is not None, rejected=self.policy.rejected)
        # A bounded native lease selects the dt-aware gate. No authority
        # heartbeat, inactive task, legacy mode or fault restores the old gate.
        interval_enabled = (self.scope and self.scope_known and self.active
                            and self.lio_ok and not self.motion_latched
                            and self.policy.transform is not None)
        publisher = getattr(self, 'interval_policy_pub', None)
        if publisher is not None:
            publisher.publish(Bool(data=bool(interval_enabled)))
        data['interval_compensation'] = bool(interval_enabled)
        data['runtime_timing_model'] = RUNTIME_TIMING_MODEL if self.scope else 'legacy'
        data['lio_health_pause_limit_sec'] = self.lio_continuity.max_pause_sec
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
        self.was_ready = ready if self.scope else False
        self.was_recovering = recovering if self.scope else False
        signature = (data['scope'], ready, data['reason'], self.error,data['lio_timing']['state'])
        if signature != self.last_status:
            self.last_status = signature
            self.get_logger().info('GLOBAL_LOCALIZATION '+json.dumps(data, ensure_ascii=False))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPoseAuthorityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
