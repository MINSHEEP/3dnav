#!/usr/bin/env python3
"""ROS adapter that publishes the selected global map policy for Nav2."""

import copy
import json
import math
import threading
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from lzp_interface.msg import CurrentMap, TerrainGrid

from .navigation_mode import MAP_2D, TERRAIN_3D, normalize_navigation_mode
from .terrain_costmap import GridGeometry, select_navigation_costmap


def _yaw_from_quaternion(value):
    sin_yaw = 2.0 * (value.w * value.z + value.x * value.y)
    cos_yaw = 1.0 - 2.0 * (value.y * value.y + value.z * value.z)
    return math.atan2(sin_yaw, cos_yaw)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class TerrainCostmapNode(Node):
    """Merge `/map` and `/terrain_grid` without weakening either veto."""

    def __init__(self):
        super().__init__('terrain_costmap_node')
        self.base_topic = str(self.declare_parameter(
            'base_map_topic', '/map').value)
        self.terrain_topic = str(self.declare_parameter(
            'terrain_grid_topic', '/terrain_grid').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/terrain_costmap').value)
        self.occupied_threshold = int(self.declare_parameter(
            'occupied_threshold', 65).value)
        self.special_cost = int(self.declare_parameter(
            'special_cost', 100).value)
        self._lock = threading.RLock()
        self._base_map = None
        self._base_topic_map = None
        self._terrain_grid = None
        self._early_terrain_grid = None
        self._current_map_name = ''
        self._current_map_load_id = ''
        self._previous_map_load_id = ''
        self._generation_unconfirmed = False
        self._terrain_invalid_load_id = ''
        self._navigation_mode = TERRAIN_3D
        self._map_epoch = 0
        self._reload_requires_get_map = False
        self._authoritative_base_map = None
        self._get_map_future = None
        self._get_map_requested_at = 0.0
        self._get_map_client = self.create_client(
            GetMap, '/map_server/map')
        self.create_timer(1.0, self._request_authoritative_map)

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            OccupancyGrid, self.output_topic, latched)
        self.status_publisher = self.create_publisher(
            String, '/terrain_costmap/status', latched)
        self.create_subscription(
            OccupancyGrid, self.base_topic, self._base_callback, latched)
        self.create_subscription(
            TerrainGrid, self.terrain_topic, self._terrain_callback, latched)
        self.create_subscription(
            String, '/terrain_grid/status',
            self._terrain_status_callback, latched)
        self.create_subscription(
            CurrentMap, '/current_map_info', self._map_callback, latched)
        self.create_subscription(
            String, '/navigation/mode', self._mode_callback, latched)
        self.get_logger().info(
            f'Terrain costmap adapter ready: {self.base_topic} + '
            f'{self.terrain_topic} -> {self.output_topic}')

    def _base_callback(self, message):
        with self._lock:
            self._base_topic_map = message
            # A transient-local /map sample may have been queued before the
            # confirmed LoadMap event. It cannot prove this reload generation.
            if (self._navigation_mode == TERRAIN_3D
                    and self._reload_requires_get_map):
                return
            self._base_map = message
        self._publish_if_ready()

    def _terrain_callback(self, message):
        with self._lock:
            terrain_load_id = str(getattr(message, 'map_load_id', ''))
            if not terrain_load_id:
                return
            if terrain_load_id == self._terrain_invalid_load_id:
                return
            if terrain_load_id != self._current_map_load_id:
                if terrain_load_id == self._previous_map_load_id:
                    return
                # TerrainGrid can be delivered before CurrentMap because
                # these are independent DDS streams. Retain that generation
                # without replacing the currently selected terrain.
                self._early_terrain_grid = message
                if (self._navigation_mode == TERRAIN_3D
                        and self._current_map_load_id):
                    # A future generation is visible before its CurrentMap
                    # confirmation. Stop advertising the old fused map now.
                    self._generation_unconfirmed = True
                    self._base_map = None
                    self._terrain_grid = None
                    self._authoritative_base_map = None
                    self._reload_requires_get_map = True
                    self._get_map_future = None
                    self._map_epoch += 1
                else:
                    return
            else:
                self._terrain_grid = message
        self._publish_if_ready()

    def _terrain_status_callback(self, message):
        try:
            status = json.loads(str(message.data))
            state = status.get('state')
            revoked_name = str(status.get('map_name', '')).strip()
            revoked_load_id = str(status.get('map_load_id', '')).strip()
        except (TypeError, ValueError, AttributeError, json.JSONDecodeError):
            return
        with self._lock:
            if state == 'ready':
                # A delayed ready status cannot undo a same-generation
                # retraction, even if the operator temporarily chose 2D.
                return
            if state not in ('unavailable', 'error'):
                return
            if (not revoked_load_id
                    or revoked_load_id != self._current_map_load_id):
                return
            if (not revoked_name
                    or revoked_name != self._current_map_name):
                return
            if not self._current_map_load_id:
                return
            # Once the terrain producer retracts this UUID, a delayed latched
            # TerrainGrid from the same UUID must not make it ready again.
            self._terrain_invalid_load_id = self._current_map_load_id
            self._terrain_grid = None
            if (self._early_terrain_grid is not None
                    and str(self._early_terrain_grid.map_load_id)
                    == self._current_map_load_id):
                self._early_terrain_grid = None
            self._map_epoch += 1
            publish_waiting = self._navigation_mode == TERRAIN_3D
        if publish_waiting:
            self._publish_if_ready()

    def _map_callback(self, message):
        if not message.success or not message.map_name:
            publish_waiting = False
            with self._lock:
                if self._navigation_mode == TERRAIN_3D:
                    self._base_map = None
                    self._terrain_grid = None
                    self._authoritative_base_map = None
                    self._current_map_name = ''
                    self._current_map_load_id = ''
                    self._generation_unconfirmed = False
                    self._terrain_invalid_load_id = ''
                    self._reload_requires_get_map = True
                    self._map_epoch += 1
                    self._get_map_future = None
                    publish_waiting = True
                else:
                    self._current_map_load_id = ''
                    self._authoritative_base_map = None
                    self._reload_requires_get_map = True
            if publish_waiting:
                self._publish_if_ready()
            return
        map_name = str(message.map_name).strip()
        load_id = str(getattr(message, 'load_id', '')).strip()
        needs_authoritative_map = False
        keep_2d_topic_map = False
        with self._lock:
            same_name = map_name == self._current_map_name
            if same_name and load_id == self._current_map_load_id:
                # The 1 Hz heartbeat repeats the active load ID; it is not a
                # new map generation and must not invalidate a ready map.
                return
            keep_2d_topic_map = same_name and self._navigation_mode == MAP_2D
            if not keep_2d_topic_map:
                self._base_map = None
            if not same_name:
                self._base_topic_map = None
            self._terrain_grid = (
                self._early_terrain_grid
                if load_id and self._early_terrain_grid is not None
                and str(self._early_terrain_grid.map_name) == map_name
                and str(self._early_terrain_grid.map_load_id) == load_id
                else None)
            self._early_terrain_grid = None
            self._previous_map_load_id = self._current_map_load_id
            self._generation_unconfirmed = False
            self._terrain_invalid_load_id = ''
            self._authoritative_base_map = None
            self._reload_requires_get_map = True
            self._get_map_future = None
            self._current_map_name = map_name
            self._current_map_load_id = load_id
            self._map_epoch += 1
            needs_authoritative_map = (
                self._navigation_mode == TERRAIN_3D and bool(load_id))
        if not keep_2d_topic_map:
            self._publish_if_ready()
        if needs_authoritative_map:
            self._request_authoritative_map()

    def _request_authoritative_map(self):
        """Read the live MapServer only after a confirmed same-name reload."""
        with self._lock:
            if (not self._reload_requires_get_map
                    or not self._current_map_load_id
                    or self._terrain_invalid_load_id
                    == self._current_map_load_id
                    or self._generation_unconfirmed
                    or self._navigation_mode != TERRAIN_3D
                    or self._authoritative_base_map is not None):
                return
            if self._get_map_future is not None:
                if time.monotonic() - self._get_map_requested_at < 5.0:
                    return
                # A hung GetMap must never make an old map acceptable.
                try:
                    self._get_map_future.cancel()
                except Exception:
                    pass
                self._get_map_future = None
            if not self._get_map_client.service_is_ready():
                return
            epoch = self._map_epoch
            try:
                future = self._get_map_client.call_async(GetMap.Request())
            except Exception as error:
                self.get_logger().warning(
                    f'Authoritative GetMap request failed: {error}')
                return
            self._get_map_future = future
            self._get_map_requested_at = time.monotonic()
            future.add_done_callback(
                lambda completed, generation=epoch:
                self._authoritative_map_callback(completed, generation))

    def _authoritative_map_callback(self, future, epoch):
        try:
            base = future.result().map
            if (str(base.header.frame_id).lstrip('/') != 'map'
                    or int(base.info.width) <= 0
                    or int(base.info.height) <= 0
                    or not math.isfinite(float(base.info.resolution))
                    or float(base.info.resolution) <= 0.0
                    or len(base.data) != int(base.info.width) * int(base.info.height)):
                raise ValueError('GetMap returned an invalid OccupancyGrid')
        except Exception as error:
            with self._lock:
                if epoch == self._map_epoch and future is self._get_map_future:
                    self._get_map_future = None
            self.get_logger().warning(
                f'Authoritative GetMap response rejected: {error}')
            return
        with self._lock:
            if (epoch != self._map_epoch
                    or future is not self._get_map_future
                    or not self._reload_requires_get_map):
                return
            self._get_map_future = None
            self._authoritative_base_map = base
            if self._navigation_mode == TERRAIN_3D:
                self._base_map = base
        self._publish_if_ready()

    def _mode_callback(self, message):
        try:
            payload = json.loads(str(message.data))
            value = payload.get('mode') if isinstance(payload, dict) else payload
            mode = normalize_navigation_mode(value)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f'Navigation mode update rejected by costmap adapter: {error}')
            mode = TERRAIN_3D
        with self._lock:
            if mode == self._navigation_mode:
                return
            self._navigation_mode = mode
            if self._reload_requires_get_map:
                self._base_map = (
                    self._base_topic_map if mode == MAP_2D
                    else self._authoritative_base_map)
        self._publish_if_ready()
        if mode == TERRAIN_3D:
            self._request_authoritative_map()

    def _publish_status(self, state, **values):
        message = String()
        message.data = json.dumps(
            {
                'state': state,
                'navigation_mode': self._navigation_mode,
                'map_load_id': self._current_map_load_id,
                **values,
            }, ensure_ascii=False,
            separators=(',', ':'))
        self.status_publisher.publish(message)

    def _publish_if_ready(self):
        with self._lock:
            base = self._base_map
            terrain = self._terrain_grid
            current_map_name = self._current_map_name
            current_map_load_id = self._current_map_load_id
            navigation_mode = self._navigation_mode
            map_epoch = self._map_epoch
        terrain_required = navigation_mode != MAP_2D
        if base is None or (terrain_required and terrain is None):
            self._publish_status(
                'waiting', base_map=base is not None,
                terrain_grid=terrain is not None,
                source='terrain_fused' if terrain_required else 'raw_map')
            return
        if terrain_required and (
                not current_map_name
                or not current_map_load_id
                or str(terrain.map_name) != current_map_name
                or str(getattr(terrain, 'map_load_id', ''))
                != current_map_load_id):
            self._publish_status(
                'waiting', base_map=True, terrain_grid=False,
                map_name=current_map_name,
                message='waiting for terrain grid matching the active map')
            return
        base_frame = str(base.header.frame_id).lstrip('/')
        terrain_frame = (
            str(terrain.header.frame_id).lstrip('/')
            if terrain is not None else '')
        if not base_frame or (
                terrain_required and base_frame != terrain_frame):
            detail = (
                f'frame mismatch: base={base.header.frame_id!r}, '
                f'terrain={terrain.header.frame_id!r}')
            self.get_logger().error(detail)
            self._publish_status('error', message=detail)
            return
        try:
            base_geometry = GridGeometry(
                resolution=float(base.info.resolution),
                width=int(base.info.width), height=int(base.info.height),
                origin_x=float(base.info.origin.position.x),
                origin_y=float(base.info.origin.position.y),
                origin_yaw=_yaw_from_quaternion(
                    base.info.origin.orientation))
            terrain_geometry = None
            terrain_cost = None
            if terrain_required:
                terrain_cost = terrain.traversal_cost
                terrain_geometry = GridGeometry(
                    resolution=float(terrain.resolution),
                    width=int(terrain.width), height=int(terrain.height),
                    origin_x=float(terrain.origin_x),
                    origin_y=float(terrain.origin_y))
            result = select_navigation_costmap(
                navigation_mode,
                base.data,
                base_geometry,
                terrain_cost,
                terrain_geometry,
                occupied_threshold=self.occupied_threshold,
                special_cost=self.special_cost,
            )
            output = OccupancyGrid()
            output.header = copy.deepcopy(base.header)
            output.header.stamp = self.get_clock().now().to_msg()
            output.info = copy.deepcopy(base.info)
            output.info.map_load_time = output.header.stamp
            output.data = result.data.astype(int).tolist()
            with self._lock:
                # Fusion may take longer than another LoadMap callback. Never
                # publish a stale ready result after that callback invalidates
                # the generation, or Nav2 could retain the old costmap.
                if (map_epoch != self._map_epoch
                        or base is not self._base_map
                        or terrain is not self._terrain_grid
                        or navigation_mode != self._navigation_mode):
                    return
                self.publisher.publish(output)
                self._publish_status(
                    'ready', frame_id=base.header.frame_id,
                    map_name=current_map_name,
                    source=(
                        'raw_map' if navigation_mode == MAP_2D
                        else 'terrain_fused'),
                    stamp_ns=_stamp_ns(output.header.stamp),
                    width=int(output.info.width), height=int(output.info.height),
                    resolution=float(output.info.resolution),
                    terrain_covered_cells=result.terrain_covered_cells,
                    unknown_cells=result.unknown_cells,
                    caution_cells=result.caution_cells,
                    special_cells=result.special_cells,
                    blocked_cells=result.blocked_cells)
        except Exception as error:
            self.get_logger().error(f'Terrain costmap update rejected: {error}')
            self._publish_status('error', message=str(error))


def main(args=None):
    rclpy.init(args=args)
    node = TerrainCostmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
