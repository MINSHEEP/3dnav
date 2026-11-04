#!/usr/bin/env python3
"""Publish a bounded live terrain observation in front of the robot."""

import json
import math
import time
from collections import deque
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty as EmptyService

from lzp_interface.msg import TerrainObservation, CurrentMap, TerrainGrid
from .rolling_terrain import RollingGroundMemory, matched_pose
from .restricted_ground import restricted_flat_ground
from .terrain_pose import propagate_localization, rigid_transform_matrix
from .terrain_navigation import TerrainCapabilityModel

from .terrain_observer import (
    corridor_return_diagnostics, estimate_local_terrain,
)


def _yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z
               + orientation.x * orientation.y),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z))


class TerrainObserverNode(Node):
    """Convert current registered returns into a fail-closed corridor model."""

    def __init__(self):
        super().__init__('terrain_observer_node')
        self.input_topic = str(self.declare_parameter(
            'input_topic', '/cloud_registered').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/terrain_observation').value)
        self.minimum_update_seconds = max(0.1, float(self.declare_parameter(
            'minimum_update_seconds', 0.20).value))
        self.maximum_points = max(1000, int(self.declare_parameter(
            'maximum_points', 60000).value))
        self.forward_min = float(self.declare_parameter(
            'forward_min', 0.10).value)
        self.forward_max = float(self.declare_parameter(
            'forward_max', 1.30).value)
        self.half_width = float(self.declare_parameter(
            'half_width', 0.45).value)
        self._odom = None
        self._odom_received_at = 0.0
        self._last_update_at = 0.0
        self._odom_samples = deque(maxlen=300)
        self._memory = RollingGroundMemory(max_age_seconds=float(self.declare_parameter(
            'ground_memory_seconds', 2.0).value))
        self._map_identity = None
        self._body = ''; self._body_at = 0.0; self._body_generation = None
        self._navigation_mode = ''; self._mode_at = 0.0
        self._last_cloud_stamp = 0.0
        self._restricted_enabled = False
        self._grid = None; self._grid_status = None
        self._map_pose = None; self._map_pose_samples = 0
        self._scan_at = 0.0; self._scan_clear = False
        self._lio_healthy = False; self._lio_at = 0.0
        self._sensor_height = float(self.declare_parameter('nominal_sensor_height_m', .52).value)
        if not .2 <= self._sensor_height <= 1.2:
            raise ValueError('nominal_sensor_height_m must be within 0.2..1.2 m')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigation_active = False
        self._refresh_future = None; self._refresh_started_at = 0.
        self._refresh_at = 0.; self._refresh_failures = 0
        self._refresh_backoff_until = 0.; self._refresh_state = 'idle'
        self._map_pose_age = None
        self._map_pose_cutoff = 0.0

        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        output_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(
            TerrainObservation, self.output_topic, output_qos)
        self.status_publisher = self.create_publisher(
            String, '/terrain_observation/status', output_qos)
        self.create_subscription(
            Odometry, '/Odometry', self._odom_callback, sensor_qos)
        self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, sensor_qos)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CurrentMap, '/current_map_info', self._map_callback, latched)
        self.create_subscription(String, '/lio_health', self._lio_callback, latched)
        self.create_subscription(Bool, '/navigation_active', self._navigation_callback, latched)
        self.create_subscription(String, '/robot_body/status', self._body_callback, latched)
        self.create_subscription(String, '/navigation/mode', self._mode_callback, latched)
        self.create_subscription(String, '/automatic_terrain_navigation/config',
                                 self._config_callback, latched)
        self.create_subscription(TerrainGrid, '/terrain_grid', self._grid_callback, latched)
        self.create_subscription(String, '/terrain_grid/status', self._grid_status_callback, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/navigation_pose',
                                 self._map_pose_callback, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 lambda _: self._clear_memory(), sensor_qos)
        self._refresh_client = self.create_client(EmptyService, '/request_nomotion_update')
        self._refresh_timer = self.create_timer(.10, self._pose_refresh_tick)
        self.get_logger().info(
            f'Live terrain observer ready: {self.input_topic} -> '
            f'{self.output_topic}')

    def _odom_callback(self, message):
        if str(message.header.frame_id).lstrip('/') != 'odom':
            self._clear_memory()
            return
        stamp = self._stamp(message.header.stamp)
        if stamp <= 0:
            return
        if self._odom_samples and stamp < self._odom_samples[-1][0]:
            self._clear_memory()
        pose = message.pose.pose
        self._odom_samples.append((stamp, float(pose.position.x),
                                   float(pose.position.y), _yaw(pose.orientation)))
        self._odom = message
        self._odom_received_at = time.monotonic()

    @staticmethod
    def _stamp(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _clear_memory(self):
        self._memory.clear()
        self._odom_samples.clear()
        self._last_cloud_stamp = 0.0
        self._map_pose = None; self._map_pose_samples = 0
        self._map_pose_age = None
        self._map_pose_cutoff = self.get_clock().now().nanoseconds/1e9

    def _map_callback(self, message):
        identity = (str(message.map_name), str(message.load_id)) if message.success else None
        if identity != self._map_identity:
            self._clear_memory()
            self._grid = None; self._grid_status = None
        self._map_identity = identity

    def _lio_callback(self, message):
        try:
            healthy = json.loads(message.data).get('state') == 'healthy'
        except (ValueError, TypeError, AttributeError):
            healthy = False
        self._lio_healthy = healthy
        self._lio_at = time.monotonic()
        if not healthy:
            self._clear_memory()

    def _body_callback(self, message):
        try:
            data = json.loads(message.data)
            selected = str(data.get('config', {}).get('selected', ''))
            state=data.get('status') or {}
            profile=data.get('config',{}).get('profiles',{}).get(selected,{})
            body = selected if (data.get('active_body') == selected and data.get('connected')
                and not data.get('switching') and not data.get('error')
                and not state.get('stale') and not state.get('emergency_latched')
                and not state.get('hardware_error') and not state.get('error')
                and profile.get('navigation_verified') is True) else ''
            generation=data.get('generation')
        except (ValueError, TypeError):
            body = ''; generation=None
        if body != self._body or generation != self._body_generation:
            self._clear_memory()
        self._body = body; self._body_at = time.monotonic()
        self._body_generation=generation

    def _mode_callback(self, message):
        try:
            mode = json.loads(message.data).get('mode', '')
        except (ValueError, TypeError):
            mode = ''
        if mode != self._navigation_mode:
            self._clear_memory()
        self._navigation_mode = mode
        self._mode_at = time.monotonic()

    def _config_callback(self, message):
        try:
            data = json.loads(message.data)
            automatic = data.get('automatic_navigation', data)
            self._restricted_enabled = (
                automatic.get('enabled', True) is True
                and automatic.get('restricted_flat_site') is True)
        except (ValueError, TypeError, AttributeError):
            self._restricted_enabled = False

    def _navigation_callback(self, message):
        self._navigation_active = bool(message.data)

    def _pose_refresh_tick(self):
        now = time.monotonic()
        if (not self._restricted_enabled or not self._navigation_active
                or self._body != 'xingji' or self._navigation_mode != 'terrain_3d'
                or now-self._body_at > 2. or now-self._mode_at > 1.
                or not self._lio_healthy or now-self._lio_at > 2.5):
            self._refresh_state = 'inactive'
            return
        future = self._refresh_future
        if future is not None:
            if not future.done():
                self._refresh_state = ('timeout' if now-self._refresh_started_at > 1.
                                       else 'waiting')
                return  # Never issue a second RPC while one is unresolved.
            self._refresh_future = None
            try:
                if future.result() is None:
                    raise ValueError('定位刷新服务返回为空')
                self._refresh_failures = 0
            except Exception:
                self._refresh_failures += 1
                if self._refresh_failures >= 3:
                    self._refresh_backoff_until = now+3.
        if now < self._refresh_backoff_until:
            self._refresh_state = 'backoff'
            return
        pose_age = (self.get_clock().now().nanoseconds/1e9
                    - self._stamp(self._map_pose.header.stamp)) if self._map_pose is not None else math.inf
        if pose_age < .20 or now-self._refresh_at < .40:
            self._refresh_state = 'fresh' if pose_age < .20 else 'rate_limited'
            return
        if not self._refresh_client.service_is_ready():
            self._refresh_state = 'waiting_service'
            return
        self._refresh_at = now; self._refresh_started_at = now
        try:
            self._refresh_future = self._refresh_client.call_async(EmptyService.Request())
            if self._refresh_future is None:
                raise ValueError('定位刷新请求未创建')
            self._refresh_state = 'waiting'
        except Exception:
            self._refresh_state = 'request_failed'
            self._refresh_backoff_until = now+3.

    def _grid_callback(self, message):
        self._grid = message

    def _grid_status_callback(self, message):
        try:
            self._grid_status = json.loads(message.data)
        except (ValueError, TypeError):
            self._grid_status = None

    def _map_pose_callback(self, message):
        stamp = self._stamp(message.header.stamp)
        if str(message.header.frame_id).lstrip('/') != 'map' or stamp <= 0:
            self._map_pose = None
            return
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if (not math.isfinite(stamp) or not 0 <= age <= 2.
                or stamp < self._map_pose_cutoff):
            return
        if self._map_pose is not None and stamp <= self._stamp(self._map_pose.header.stamp):
            return
        self._map_pose = message
        self._map_pose_samples += 1

    def _scan_callback(self, message):
        age = self.get_clock().now().nanoseconds/1e9-self._stamp(message.header.stamp)
        if (str(message.header.frame_id).lstrip('/') != 'laser_2d' or not 0 <= age <= .30
                or len(message.ranges)<16 or not math.isfinite(message.angle_increment)
                or message.angle_increment<=0 or not math.isfinite(message.angle_min)
                or not math.isfinite(message.range_min) or not math.isfinite(message.range_max)
                or not 0 <= message.range_min < message.range_max
                or (len(message.ranges)-1)*message.angle_increment < 1.5*math.pi):
            self._scan_clear = False
            return
        self._scan_at = time.monotonic()-age
        self._scan_clear = True
        for i, value in enumerate(message.ranges):
            if math.isnan(value) or value == -math.inf or value < message.range_min:
                self._scan_clear = False
                return
            angle = message.angle_min+i*message.angle_increment
            if math.isfinite(value) and message.range_min <= value <= message.range_max:
                x,y=value*math.cos(angle)+.23,value*math.sin(angle)
                if -.60 <= x <= 1.75 and abs(y) <= .60:
                    self._scan_clear = False
                    return

    def _restricted_prior(self, points, stamp, robot_x, robot_y, yaw, *, ground_anchor=None):
        self._map_pose_age = None
        if ground_anchor is None or not all(math.isfinite(v) for v in ground_anchor):
            return None
        # Even a map prior needs a live, height-anchored floor. Otherwise a
        # mislabelled roof in both the map and scan could bypass the anchor.
        live_floor = np.asarray(points)
        if (live_floor.ndim != 2 or live_floor.shape[1] != 3
                or np.count_nonzero(np.isfinite(live_floor).all(axis=1)
                    & (np.abs(live_floor[:, 2]-ground_anchor[2]) <= .15)
                    & (np.hypot(live_floor[:, 0]-robot_x, live_floor[:, 1]-robot_y) <= 2.5)) < 30):
            return None
        grid,status,pose=self._grid,self._grid_status,self._map_pose
        if (not self._restricted_enabled or grid is None or not isinstance(status,dict)
                or pose is None or self._map_pose_samples < 2
                or not self._scan_clear or time.monotonic()-self._scan_at > .30):
            return None
        identity=(str(grid.map_name),str(grid.map_load_id))
        if (identity != self._map_identity
                or status.get('state') != 'ready' or status.get('artifact_verified') is not True
                or status.get('algorithm') != 'clustered_measured_ground_v3'
                or (status.get('map_name'),status.get('map_load_id')) != identity
                or int(status.get('stamp_ns',0)) != (
                    int(grid.header.stamp.sec)*1000000000+int(grid.header.stamp.nanosec))
                or str(grid.header.frame_id).lstrip('/') != 'map'):
            return None
        covariance=pose.pose.covariance
        if any(not math.isfinite(covariance[i]) for i in (0,1,6,7,35)):
            return None
        if covariance[35] < 0 or abs(covariance[1]-covariance[6]) > 1e-8:
            return None
        eigenvalues = np.linalg.eigvalsh([[covariance[0], covariance[1]],
                                        [covariance[6], covariance[7]]])
        if eigenvalues[0] < -1e-8:
            return None
        original=pose.pose.pose
        mapped = propagate_localization((original.position.x, original.position.y,
            _yaw(original.orientation)), source_stamp=self._stamp(pose.header.stamp),
            capture_stamp=stamp, odometry=list(self._odom_samples),
            position_sigma=math.sqrt(max(0., eigenvalues[-1])),
            yaw_sigma=math.sqrt(covariance[35]))
        if mapped is None:
            return None
        self._map_pose_age = mapped.source_age
        map_yaw=mapped.yaw
        if self._body == 'xingji' and self._navigation_mode == 'terrain_3d':
            try:
                capture = Time(nanoseconds=int(round(stamp*1e9)))
                tf = self._tf_buffer.lookup_transform('map', 'odom', capture).transform
                base = self._tf_buffer.lookup_transform('map', 'base_footprint', capture).transform
                t, q = tf.translation, tf.rotation
                matrix = np.asarray(rigid_transform_matrix((t.x, t.y, t.z),
                                                          (q.x, q.y, q.z, q.w)))
                bx, by = float(base.translation.x), float(base.translation.y)
                if (not math.isfinite(bx) or not math.isfinite(by)
                        or math.hypot(bx-mapped.x, by-mapped.y) > .35):
                    return None
                live = np.asarray(points) @ matrix[:, :3].T + matrix[:, 3]
                return restricted_flat_ground(grid, robot_x=bx, robot_y=by,
                    yaw=_yaw(base.rotation), live_points_map=live,
                    uncertainty_radius_m=2*mapped.position_sigma+1.75*math.sin(2*mapped.yaw_sigma))
            except Exception:
                return None  # Missing exact-time TF cannot certify a map prior.
        angle=map_yaw-yaw; c,s=math.cos(angle),math.sin(angle)
        live=np.asarray(points).copy(); dx=live[:,0]-robot_x; dy=live[:,1]-robot_y
        live[:,0]=mapped.x+c*dx-s*dy
        live[:,1]=mapped.y+s*dx+c*dy
        return restricted_flat_ground(grid,robot_x=mapped.x,
            robot_y=mapped.y,yaw=map_yaw,live_points_map=live,
            uncertainty_radius_m=2*mapped.position_sigma+1.75*math.sin(2*mapped.yaw_sigma))

    def _xyz(self, message):
        fields = {field.name: field for field in message.fields}
        required = [fields.get(name) for name in ('x', 'y', 'z')]
        if any(field is None or field.datatype != PointField.FLOAT32
               for field in required):
            raise ValueError('cloud must contain float32 x/y/z')
        count = int(message.width) * int(message.height)
        if count <= 0 or int(message.point_step) <= 0:
            raise ValueError('cloud is empty')
        data = memoryview(message.data)
        byte_order = '>' if message.is_bigendian else '<'
        columns = [np.ndarray(
            shape=(count,), dtype=f'{byte_order}f4', buffer=data,
            offset=int(field.offset), strides=(int(message.point_step),))
            for field in required]
        stride = max(1, int(math.ceil(count / self.maximum_points)))
        return np.column_stack([column[::stride] for column in columns])

    def _ground_anchor(self, message):
        # /Odometry deliberately sets z=0. Physical sensor TF follows height.
        transform = self._tf_buffer.lookup_transform(
            'odom', 'livox_frame', Time.from_msg(message.header.stamp))
        point = transform.transform.translation
        anchor = (float(point.x), float(point.y), float(point.z)-self._sensor_height)
        if not all(math.isfinite(v) for v in anchor):
            raise ValueError('真实雷达姿态或地面高度基准无效')
        return anchor

    @staticmethod
    def _message(value):
        message = TerrainObservation()
        message.terrain_type = value.terrain_type
        # Dataclass annotations do not coerce int/NumPy values, while ROS
        # generated float64 setters require built-in floats even for zero.
        for field in ('slope_deg', 'cross_slope_deg', 'step_height_m',
                      'gap_width_m', 'roughness_m', 'clearance_width_m',
                      'confidence'):
            number = float(getattr(value, field))
            if not math.isfinite(number):
                raise ValueError(f'Invalid terrain observation field: {field}')
            setattr(message, field, number)
        message.negative_obstacle = bool(value.negative_obstacle)
        message.slippery = bool(value.slippery)
        return message

    def _cloud_callback(self, message):
        now = time.monotonic()
        if now - self._last_update_at < self.minimum_update_seconds:
            return
        self._last_update_at = now
        if self._odom is None or now - self._odom_received_at > 0.75:
            self._publish_error('三维里程计不可用或已过期')
            return
        try:
            stamp = self._stamp(message.header.stamp)
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if (str(message.header.frame_id).lstrip('/') != 'odom'
                    or not -0.05 <= age <= 0.75 or stamp <= self._last_cloud_stamp):
                raise ValueError('点云坐标系、时间或帧顺序无效')
            pose = matched_pose(list(self._odom_samples), stamp)
            if pose is None:
                raise ValueError('没有与点云采集时刻匹配的里程计')
            robot_x, robot_y, yaw = pose
            self._last_cloud_stamp = stamp
            points = self._xyz(message)
            geometry = dict(forward_min=self.forward_min,
                            forward_max=self.forward_max, half_width=self.half_width)
            evidence_source = 'current'
            if self._navigation_mode == 'terrain_3d' and self._body not in ('xingji', 'gangben_zsl1'):
                raise ValueError('3D导航本体身份尚未确认，不能授权地形通行')
            if (self._body == 'xingji' and self._navigation_mode == 'terrain_3d'
                    and now - self._body_at <= 2.0 and now-self._mode_at <= 1.0):
                if not self._lio_healthy or now - self._lio_at > 2.5:
                    raise ValueError('LIO状态异常或过期，暂停使用历史地面和地图先验')
                anchor = self._ground_anchor(message)
                geometry['ground_anchor'] = anchor
                geometry['separate_missing_returns'] = True
                evidence_source = 'current_anchored'
                self._memory.update(points, stamp_seconds=stamp,
                                    robot_x=robot_x, robot_y=robot_y, yaw=yaw,
                                    ground_anchor=anchor)
                value = self._memory.observe(points, stamp_seconds=stamp,
                    robot_x=robot_x, robot_y=robot_y, yaw=yaw, **geometry)
                if self._memory.cells:
                    evidence_source = 'recent_measured_anchored'
                if (value.terrain_type in ('unknown', 'flat')
                        and not TerrainCapabilityModel().assess(value).traversable):
                    prior = self._restricted_prior(points, stamp, robot_x, robot_y, yaw,
                                                   ground_anchor=anchor)
                    if prior is not None:
                        value = prior
                        evidence_source = 'restricted_map'
            else:
                self._memory.clear()
                value = estimate_local_terrain(points, robot_x=robot_x,
                                               robot_y=robot_y, yaw=yaw, **geometry)
            return_coverage = corridor_return_diagnostics(
                points, robot_x=robot_x, robot_y=robot_y,
                yaw=yaw, forward_min=self.forward_min,
                forward_max=self.forward_max, half_width=self.half_width)
            output = self._message(value)
            output.header = message.header
            output.evidence_source = evidence_source
            self.publisher.publish(output)
            self.status_publisher.publish(String(data=json.dumps({
                'state': 'ready',
                'terrain_type': value.terrain_type,
                'confidence': value.confidence,
                'slope_deg': value.slope_deg,
                'step_height_m': value.step_height_m,
                'sampling_gap_width_m': value.sampling_gap_width_m,
                'recent_ground_cells': len(self._memory.cells),
                'evidence_source': evidence_source,
                'restricted_flat_site_enabled': self._restricted_enabled,
                'map_pose_source_age_sec': self._map_pose_age,
                'localization_refresh_state': self._refresh_state,
                **return_coverage,
            }, ensure_ascii=False, separators=(',', ':'))))
        except Exception as error:
            self._publish_error(str(error))

    def _publish_error(self, message):
        self._memory.clear()
        from .terrain_navigation import TerrainObservation as Observation
        output = self._message(Observation(terrain_type='unknown', confidence=0.0,
                                          clearance_width_m=0.0))
        output.header.frame_id = 'odom'
        output.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(output)
        self.status_publisher.publish(String(data=json.dumps({
            'state': 'error', 'message': str(message),
        }, ensure_ascii=False, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = TerrainObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
