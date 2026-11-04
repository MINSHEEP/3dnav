#!/usr/bin/env python3
"""Publish a CPU-bounded 2.5D terrain grid from the persistent global cloud."""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from lzp_interface.msg import CurrentMap, TerrainGrid
from lzp_interface.srv import Check3DGround
from .pcd_io import load_pcd_xyz
from .terrain_grid import build_terrain_grid
from .terrain_grid_io import (
    load_bound_terrain_grid,
    terrain_artifact_signature,
)


def check_ground_candidate(
        grid, x, y, *, footprint_radius_m=0.45,
        minimum_confidence=0.75, inferred_roughness_marker_m):
    """Check a *map selection candidate*, never authorize physical motion.

    New version-2 artifacts explicitly mark measured ground. Legacy version-1
    artifacts instead used roughness to flag inferred cells; that ambiguous
    range stays rejected until the user rebuilds the terrain. Neither format
    proves that the selected surface is currently safe.
    """
    invalid_z = 0.0
    try:
        x = float(x)
        y = float(y)
        radius = float(footprint_radius_m)
        minimum = float(minimum_confidence)
        marker = float(inferred_roughness_marker_m)
        resolution = float(grid.resolution)
        origin_x = float(grid.origin_x)
        origin_y = float(grid.origin_y)
        width = int(grid.width)
        height = int(grid.height)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, 'invalid_grid', '地形图数据或坐标无效', invalid_z
    if not all(map(math.isfinite, (
            x, y, radius, minimum, marker,
            resolution, origin_x, origin_y))):
        return False, 'invalid_grid', '地形图数据或坐标不是有限值', invalid_z
    if radius <= 0.0 or resolution <= 0.0 or width <= 0 or height <= 0:
        return False, 'invalid_grid', '地形图尺寸或占地半径无效', invalid_z
    if not 0.0 < marker or not 0.0 < minimum <= 1.0:
        return False, 'invalid_grid', '地面证据阈值无效', invalid_z
    length = width * height
    try:
        if any(len(layer) != length for layer in (
                grid.elevation, grid.confidence,
                grid.roughness_m, grid.traversal_cost)):
            return False, 'invalid_grid', '地形图图层长度不一致', invalid_z
    except (AttributeError, TypeError):
        return False, 'invalid_grid', '地形图缺少必要图层', invalid_z

    center_col = math.floor((x - origin_x) / resolution)
    center_row = math.floor((y - origin_y) / resolution)
    if not (0 <= center_col < width and 0 <= center_row < height):
        return False, 'outside_map', '选点位于三维地形图之外', invalid_z

    center_index = center_row * width + center_col
    center_cost = int(grid.traversal_cost[center_index])
    measured = getattr(grid, 'ground_measured', None)
    if measured is not None and len(measured) != length:
        return False, 'invalid_grid', '实测地面证据图层长度不一致', invalid_z
    if center_cost < 0:
        return False, 'unknown_center', '选点位于未知地形格，不能设为初始点或目标点', invalid_z
    if center_cost != 0 and not (center_cost == 60 and measured is not None):
        return False, 'non_free_center', '初始点或目标点只能选普通地面，不能选警戒、特殊或禁行格', invalid_z

    min_col = math.floor((x - radius - origin_x) / resolution)
    max_col = math.floor((x + radius - origin_x) / resolution)
    min_row = math.floor((y - radius - origin_y) / resolution)
    max_row = math.floor((y + radius - origin_y) / resolution)
    if min_col < 0 or min_row < 0 or max_col >= width or max_row >= height:
        return False, 'footprint_outside_map', '机器狗占地范围超出地形图', invalid_z

    center_z = float(grid.elevation[center_index])
    if not math.isfinite(center_z):
        return False, 'unknown_ground', '选点没有有限的地面高程', invalid_z

    radius_sq = radius * radius
    for row in range(min_row, max_row + 1):
        cell_y_min = origin_y + row * resolution
        cell_y_max = cell_y_min + resolution
        nearest_y = max(cell_y_min, min(y, cell_y_max))
        for col in range(min_col, max_col + 1):
            cell_x_min = origin_x + col * resolution
            cell_x_max = cell_x_min + resolution
            nearest_x = max(cell_x_min, min(x, cell_x_max))
            if (nearest_x - x) ** 2 + (nearest_y - y) ** 2 > radius_sq:
                continue
            index = row * width + col
            if measured is not None and int(measured[index]) != 1:
                return False, 'unmeasured_ground', '占地范围地面未经真实点云确认，不能用推断面选点', invalid_z
            cost = int(grid.traversal_cost[index])
            if cost < 0:
                return False, 'unknown_footprint', '占地范围存在未知地形格', invalid_z
            if cost >= 90:
                return False, 'blocked_footprint', '占地范围存在特殊或禁行地形格', invalid_z
            elevation = float(grid.elevation[index])
            confidence = float(grid.confidence[index])
            roughness = float(grid.roughness_m[index])
            if not math.isfinite(elevation):
                return False, 'unknown_ground', '占地范围缺少有限地面高程', invalid_z
            if not math.isfinite(confidence) or confidence < minimum:
                return False, 'low_confidence', '占地范围地面观测置信度不足', invalid_z
            if not math.isfinite(roughness):
                return False, 'unknown_ground', '地面粗糙度不可用', invalid_z
            if measured is not None and roughness > 0.06:
                return False, 'rough_ground_limit', '实测地面粗糙度超过能力限制', invalid_z
            if measured is None and roughness >= marker:
                return (
                    False, 'possible_reconstruction',
                    '旧地形图无法区分实测与推断地面，请在导航页点击重建地形后重新加载',
                    invalid_z)
    return (
        True, 'candidate_ground',
        '仅表示已保存三维地图中的可选地面候选；不保证实时无坑或允许机器人运动',
        center_z)


class TerrainGridNode(Node):
    """Convert `/global_mapping_cloud` into conservative terrain layers."""

    def __init__(self):
        super().__init__('terrain_grid_node')
        self.resolution = float(self.declare_parameter('resolution', 0.25).value)
        self.maximum_cells = int(self.declare_parameter(
            'maximum_cells', 400_000).value)
        self.minimum_update_seconds = float(self.declare_parameter(
            'minimum_update_seconds', 5.0).value)
        self.caution_slope_deg = float(self.declare_parameter(
            'caution_slope_deg', 12.0).value)
        self.special_slope_deg = float(self.declare_parameter(
            'special_slope_deg', 18.0).value)
        self.blocked_slope_deg = float(self.declare_parameter(
            'blocked_slope_deg', 28.0).value)
        self.caution_step_m = float(self.declare_parameter(
            'caution_step_m', 0.08).value)
        self.special_step_m = float(self.declare_parameter(
            'special_step_m', 0.12).value)
        self.blocked_step_m = float(self.declare_parameter(
            'blocked_step_m', 0.18).value)
        self.roughness_caution_m = float(self.declare_parameter(
            'roughness_caution_m', 0.05).value)
        self.minimum_confidence = float(self.declare_parameter(
            'minimum_confidence', 0.35).value)
        self.input_topic = str(self.declare_parameter(
            'input_topic', '/global_mapping_cloud').value)
        self.saved_pcd_directory = str(self.declare_parameter(
            'saved_pcd_directory', '').value).strip()
        self.last_update_ns = 0
        self.current_map_name = ''
        self.current_map_load_id = ''
        self._rejected_map_generations = set()
        self.loaded_artifact_signature = None
        self._ground_grid = None
        self._ground_map_name = ''
        self._ground_load_id = ''
        self._ground_frame_id = ''
        self._ground_artifact_verified = False
        self._ground_algorithm = ''
        self._ground_roughness_marker_m = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            TerrainGrid, '/terrain_grid', output_qos)
        self.create_service(
            Check3DGround, '/terrain/check_3d_ground',
            self._check_3d_ground)
        self.status_publisher = self.create_publisher(
            String, '/terrain_grid/status', output_qos)
        self.subscription = None
        self.map_subscription = None
        if self.saved_pcd_directory:
            self.map_subscription = self.create_subscription(
                CurrentMap, '/current_map_info', self._map_callback, output_qos)
        else:
            self.subscription = self.create_subscription(
                PointCloud2, self.input_topic, self._cloud_callback, qos)
        self.get_logger().info(
            f'2.5D terrain grid ready from {self.input_topic}: '
            f'resolution={self.resolution:.2f} m, '
            f'maximum_cells={self.maximum_cells}')

    def _map_callback(self, message):
        if not message.success or not message.map_name:
            self._invalidate_map('no active map selected')
            return
        map_name = str(message.map_name).strip()
        load_id = str(getattr(message, 'load_id', '') or '').strip()
        if not load_id:
            self._invalidate_map('active map has no load_id')
            return
        generation = (map_name, load_id)
        if generation in self._rejected_map_generations:
            # A prior failed validation or in-place file mutation must never
            # be rescued by a heartbeat carrying the old LoadMap UUID.
            if (generation == (
                    self.current_map_name, self.current_map_load_id)
                    and self._ground_grid is not None):
                self._invalidate_map('saved terrain requires a new LoadMap')
            return
        if (map_name != self.current_map_name
                or load_id != self.current_map_load_id):
            # A fresh LoadMap UUID starts a new map/AMCL generation, even
            # when the map filename and saved artifact are unchanged. A
            # heartbeat repeating the same pair does not trigger a reload.
            self._invalidate_map(
                'terrain map generation changed')
        self.current_map_load_id = load_id
        try:
            if (
                not map_name or Path(map_name).name != map_name or
                '/' in map_name or '\\' in map_name
            ):
                raise ValueError('current map name is unsafe')
            directory = Path(self.saved_pcd_directory)
            metadata_path = directory / f'{map_name}.localization.yaml'
            with metadata_path.open('r', encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream)
            if not isinstance(metadata, dict):
                raise ValueError('localization metadata must be an object')

            metadata_signature = terrain_artifact_signature(metadata_path)
            if 'terrain_grid' not in metadata:
                pcd_path = directory / f'{map_name}.pcd'
                signature = (
                    map_name, 'pcd', metadata_signature,
                    terrain_artifact_signature(pcd_path),
                )
                if signature == self.loaded_artifact_signature:
                    return
                if self.loaded_artifact_signature is not None:
                    self._rejected_map_generations.add(generation)
                    self._invalidate_map(
                        'saved terrain changed without a new LoadMap')
                    return
                if self.current_map_name or self.loaded_artifact_signature:
                    self._invalidate_map('reloading terrain from legacy PCD')
                self.get_logger().warning(
                    f'Map {map_name} has no persisted terrain artifact; '
                    'falling back to PCD reconstruction.')
                points = load_pcd_xyz(pcd_path)
                self.current_map_name = map_name
                self._publish_grid(
                    points, frame_id='map',
                    source=f'legacy complete PCD {pcd_path.name}')
            else:
                terrain_path = directory / f'{map_name}.terrain.npz'
                signature = (
                    map_name, 'terrain', metadata_signature,
                    terrain_artifact_signature(terrain_path),
                )
                if signature == self.loaded_artifact_signature:
                    return
                if self.loaded_artifact_signature is not None:
                    self._rejected_map_generations.add(generation)
                    self._invalidate_map(
                        'saved terrain changed without a new LoadMap')
                    return
                if self.current_map_name or self.loaded_artifact_signature:
                    self._invalidate_map('loading verified terrain artifact')
                loaded = load_bound_terrain_grid(
                    directory,
                    map_name=map_name,
                    localization_metadata=metadata,
                )
                if loaded is None:
                    raise ValueError(
                        'terrain metadata exists but no artifact was loaded')
                self.current_map_name = map_name
                self._publish_grid_data(
                    loaded.grid,
                    frame_id=loaded.frame_id,
                    source=f'persisted terrain {terrain_path.name}',
                    quality=(loaded.generation or {}).get('quality'),
                    algorithm=(loaded.generation or {}).get('algorithm'),
                    generation=loaded.generation,
                    artifact_verified=True,
                    source_pcd_sha256=metadata['terrain_grid'][
                        'source_pcd_sha256'],
                    terrain_sha256=metadata['terrain_grid']['sha256'],
                )
            self.loaded_artifact_signature = signature
        except Exception as error:
            self._rejected_map_generations.add(generation)
            self._publish_error(
                f'saved terrain map rejected: {error}',
                map_name=map_name, map_load_id=load_id)
            self.current_map_name = ''
            self.current_map_load_id = ''
            self.loaded_artifact_signature = None

    def _invalidate_map(self, reason):
        revoked_name = getattr(self, '_ground_map_name', '') or self.current_map_name
        revoked_load_id = getattr(self, '_ground_load_id', '') or self.current_map_load_id
        self.current_map_name = ''
        self.current_map_load_id = ''
        self.loaded_artifact_signature = None
        self._ground_grid = None
        self._ground_map_name = ''
        self._ground_load_id = ''
        self._ground_frame_id = ''
        self._ground_artifact_verified = False
        self._ground_algorithm = ''
        self._ground_roughness_marker_m = None
        status = String()
        status.data = json.dumps({
            'state': 'unavailable', 'message': str(reason),
            'map_name': revoked_name, 'map_load_id': revoked_load_id,
            'stamp_ns': 0,
            'artifact_verified': False,
            'source_pcd_sha256': '', 'terrain_sha256': '',
        }, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(status)

    @staticmethod
    def _xyz(message):
        fields = {field.name: field for field in message.fields}
        required = [fields.get(name) for name in ('x', 'y', 'z')]
        if any(field is None or field.datatype != PointField.FLOAT32
               for field in required):
            raise ValueError('global mapping cloud must contain float32 x/y/z')
        count = int(message.width) * int(message.height)
        data = memoryview(message.data)
        byte_order = '>' if message.is_bigendian else '<'
        columns = [np.ndarray(
            shape=(count,), dtype=f'{byte_order}f4', buffer=data,
            offset=int(field.offset), strides=(int(message.point_step),))
            for field in required]
        return np.column_stack(columns)

    def _cloud_callback(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_update_ns < self.minimum_update_seconds * 1e9:
            return
        self.last_update_ns = now_ns
        try:
            self._publish_grid(
                self._xyz(message), frame_id=message.header.frame_id,
                stamp=message.header.stamp, source=self.input_topic)
        except Exception as error:
            self._publish_error(str(error))

    def _publish_grid(self, points, *, frame_id, source, stamp=None):
        grid = build_terrain_grid(
            points, resolution=self.resolution,
            maximum_cells=self.maximum_cells,
            caution_slope_deg=self.caution_slope_deg,
            special_slope_deg=self.special_slope_deg,
            blocked_slope_deg=self.blocked_slope_deg,
            caution_step_m=self.caution_step_m,
            special_step_m=self.special_step_m,
            blocked_step_m=self.blocked_step_m,
            roughness_caution_m=self.roughness_caution_m,
            minimum_confidence=self.minimum_confidence)
        self._publish_grid_data(
            grid, frame_id=frame_id, source=source, stamp=stamp)

    def _publish_grid_data(
            self, grid, *, frame_id, source, stamp=None,
            quality=None, algorithm=None, generation=None,
            artifact_verified=False,
            source_pcd_sha256='', terrain_sha256=''):
        if self.saved_pcd_directory and not self.current_map_load_id:
            self._publish_error('saved terrain has no active map_load_id')
            return
        parameters = (
            generation.get('parameters', {})
            if isinstance(generation, dict) else {})
        self._ground_grid = grid
        self._ground_map_name = self.current_map_name
        self._ground_load_id = self.current_map_load_id
        self._ground_frame_id = str(frame_id)
        self._ground_artifact_verified = bool(artifact_verified)
        self._ground_algorithm = str(algorithm or '')
        self._ground_roughness_marker_m = (
            parameters.get('roughness_caution_m')
            if isinstance(parameters, dict) else None)
        output = TerrainGrid()
        output.header.frame_id = str(frame_id)
        output.header.stamp = stamp or self.get_clock().now().to_msg()
        output.map_name = self.current_map_name
        output.map_load_id = self.current_map_load_id
        output.resolution = float(grid.resolution)
        output.width = grid.width
        output.height = grid.height
        output.origin_x = grid.origin_x
        output.origin_y = grid.origin_y
        output.elevation = grid.elevation.tolist()
        output.slope_deg = grid.slope_deg.tolist()
        output.roughness_m = grid.roughness_m.tolist()
        output.step_height_m = grid.step_height_m.tolist()
        output.confidence = grid.confidence.tolist()
        output.traversal_cost = grid.traversal_cost.tolist()
        output.ground_measured = (
            grid.ground_measured.tolist() if getattr(grid, 'ground_measured', None) is not None else [])
        self.publisher.publish(output)
        known = int(np.count_nonzero(grid.traversal_cost >= 0))
        blocked = int(np.count_nonzero(grid.traversal_cost == 100))
        status = String()
        status.data = json.dumps({
            'state': 'ready', 'width': grid.width,
            'height': grid.height, 'resolution': grid.resolution,
            'known_cells': known, 'blocked_cells': blocked,
            'frame_id': output.header.frame_id, 'source': source,
            'map_name': output.map_name,
            'map_load_id': output.map_load_id,
            'stamp_ns': (
                int(output.header.stamp.sec) * 1_000_000_000
                + int(output.header.stamp.nanosec)),
            'artifact_verified': bool(artifact_verified),
            'source_pcd_sha256': str(source_pcd_sha256),
            'terrain_sha256': str(terrain_sha256),
            'algorithm': str(algorithm or ''),
            'quality': quality if isinstance(quality, dict) else None,
        }, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(status)

    def _publish_error(self, error, *, map_name='', map_load_id=''):
        self.get_logger().warning(f'Terrain grid update skipped: {error}')
        revoked_name = (
            map_name or getattr(self, '_ground_map_name', '')
            or self.current_map_name)
        revoked_load_id = (
            map_load_id or getattr(self, '_ground_load_id', '')
            or self.current_map_load_id)
        self._ground_grid = None
        self._ground_map_name = ''
        self._ground_load_id = ''
        self._ground_frame_id = ''
        self._ground_artifact_verified = False
        self._ground_algorithm = ''
        self._ground_roughness_marker_m = None
        status = String()
        status.data = json.dumps({
            'state': 'error', 'message': str(error),
            'map_name': revoked_name, 'map_load_id': revoked_load_id,
            'stamp_ns': 0,
            'artifact_verified': False,
            'source_pcd_sha256': '', 'terrain_sha256': '',
        }, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(status)

    def _check_3d_ground(self, request, response):
        """Read-only, conservative map-point check; not a navigation permit."""
        response.valid = False
        response.ground_z = 0.0
        requested_map = str(request.map_name).strip()
        requested_load_id = str(getattr(request, 'map_load_id', '') or '').strip()
        if not requested_map or not self.current_map_name:
            response.code = 'map_unavailable'
            response.message = '当前没有已加载的地图'
        elif not self.current_map_load_id:
            response.code = 'map_generation_unavailable'
            response.message = '当前地图没有有效的加载代际，无法核对三维地面'
        elif not requested_load_id:
            response.code = 'load_id_required'
            response.message = '请求缺少地图加载代际，请刷新地图后重新选点'
        elif (requested_load_id != self.current_map_load_id
              or requested_load_id != self._ground_load_id):
            response.code = 'stale_map_generation'
            response.message = '地图已重新加载，请刷新地图后重新选点'
        elif (requested_map != self.current_map_name
              or requested_map != self._ground_map_name):
            response.code = 'map_mismatch'
            response.message = '请求的地图与当前三维地形图不一致'
        elif (not self.saved_pcd_directory or self._ground_grid is None
              or not self._ground_artifact_verified):
            response.code = 'unverified_ground'
            response.message = '没有经过校验的已保存三维地形图'
        elif self._ground_frame_id.lstrip('/') != 'map':
            response.code = 'frame_mismatch'
            response.message = '地形图坐标系不是 map'
        elif self._ground_algorithm not in (
                'free_space_lower_envelope_v2', 'clustered_measured_ground_v3'):
            response.code = 'unsupported_artifact'
            response.message = '地形图生成算法缺少可核对的重建标记规则'
        else:
            valid, code, message, z = check_ground_candidate(
                self._ground_grid, request.x, request.y,
                inferred_roughness_marker_m=(
                    self._ground_roughness_marker_m))
            response.valid = valid
            response.code = code
            response.message = message
            response.ground_z = z
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TerrainGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
