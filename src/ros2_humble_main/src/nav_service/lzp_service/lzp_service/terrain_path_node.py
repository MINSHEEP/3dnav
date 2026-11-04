#!/usr/bin/env python3
"""Publish a terrain-elevated, display-only copy of the Nav2 global plan."""

from __future__ import annotations

import copy
import json

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from lzp_interface.msg import CurrentMap, TerrainGrid

from .terrain_grid import TerrainGridData
from .navigation_mode import (
    MAP_2D,
    TERRAIN_3D,
    normalize_navigation_mode,
)
from .terrain_path import (
    elevate_path_points,
    path_length_m,
    simplify_path_indices,
)


def _frame_id(value) -> str:
    return str(value or '').strip().lstrip('/')


class TerrainPathNode(Node):
    """Adapt `/plan` for visualization without feeding it back to Nav2."""

    def __init__(self):
        super().__init__('terrain_path_node')
        self._clearance_m = max(0.0, float(self.declare_parameter(
            'clearance_m', 0.08).value))
        self._minimum_spacing_m = max(0.0, float(self.declare_parameter(
            'minimum_spacing_m', 0.10).value))
        self._maximum_points = max(2, int(self.declare_parameter(
            'maximum_points', 4000).value))

        self._navigation_active = False
        self._navigation_mode = TERRAIN_3D
        self._current_map_name = ''
        self._terrain_map_name = ''
        self._terrain_frame_id = ''
        self._terrain_data_cache = None
        self._pending_plan = None

        plan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            Path, '/navigation/global_path', latched_qos)
        self.status_publisher = self.create_publisher(
            String, '/navigation/global_path/status', latched_qos)
        self.create_subscription(
            Path, '/plan', self._plan_callback, plan_qos)
        self.create_subscription(
            TerrainGrid, '/terrain_grid', self._terrain_callback, latched_qos)
        self.create_subscription(
            CurrentMap,
            '/current_map_info',
            self._map_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool, '/navigation_active', self._active_callback, latched_qos)
        self.create_subscription(
            String, '/navigation/mode', self._mode_callback, latched_qos)

        self._publish_empty('inactive', 'navigation is not active')
        self.get_logger().info(
            'Terrain path display bridge ready: /plan -> '
            '/navigation/global_path (display only)')

    def _status(self, state, message='', **values):
        source_count = int(values.pop('source_pose_count', 0))
        display_count = int(values.pop('display_pose_count', 0))
        lifted_count = int(values.pop('lifted_count', 0))
        fallback_count = int(values.pop('fallback_count', 0))
        length_m = float(values.pop('length_m', 0.0))
        payload = {
            'state': str(state),
            'message': str(message),
            'map_name': self._current_map_name,
            'frame_id': str(values.pop('frame_id', '')),
            'source_pose_count': source_count,
            'display_pose_count': display_count,
            'pose_count': display_count,
            'lifted': lifted_count > 0,
            'lifted_count': lifted_count,
            'fallback': fallback_count > 0,
            'fallback_count': fallback_count,
            'length_m': length_m,
            'length': length_m,
            'terrain_ready': self._matching_terrain() is not None,
            'navigation_mode': self._navigation_mode,
            'navigation_active': self._navigation_active,
            'display_only': True,
            **values,
        }
        output = String()
        output.data = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(output)

    def _empty_frame(self):
        terrain = self._matching_terrain()
        if terrain is not None:
            return self._terrain_frame_id
        return 'map' if self._current_map_name else ''

    def _publish_empty(self, state, message, **values):
        output = Path()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._empty_frame()
        self.path_publisher.publish(output)
        self._status(
            state,
            message,
            frame_id=output.header.frame_id,
            **values,
        )

    def _matching_terrain(self):
        if self._terrain_data_cache is None or not self._current_map_name:
            return None
        if self._terrain_map_name != self._current_map_name:
            return None
        return self._terrain_data_cache

    def _map_callback(self, message):
        map_name = (
            str(message.map_name).strip()
            if message.success and message.map_name else '')
        if map_name == self._current_map_name:
            return
        self._current_map_name = map_name
        self._pending_plan = None
        if self._terrain_map_name != map_name:
            self._terrain_map_name = ''
            self._terrain_frame_id = ''
            self._terrain_data_cache = None
        if not map_name:
            self._publish_empty('waiting', 'waiting for an active map')
            return
        waiting_for = (
            'a Nav2 plan' if self._navigation_mode == MAP_2D
            else 'terrain and a Nav2 plan')
        self._publish_empty(
            'waiting', f'waiting for {waiting_for} for {map_name}')

    def _mode_callback(self, message):
        try:
            payload = json.loads(str(message.data))
            value = payload.get('mode') if isinstance(payload, dict) else payload
            mode = normalize_navigation_mode(value)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f'Navigation mode update rejected by path adapter: {error}')
            mode = TERRAIN_3D
        if mode == self._navigation_mode:
            return
        self._navigation_mode = mode
        if not self._navigation_active:
            self._publish_empty('inactive', 'navigation is not active')
        elif self._pending_plan is not None:
            self._publish_pending_plan()
        else:
            self._publish_empty('waiting', 'waiting for the Nav2 global plan')

    def _active_callback(self, message):
        active = bool(message.data)
        if active == self._navigation_active:
            return
        self._navigation_active = active
        if not active:
            self._pending_plan = None
            self._publish_empty('inactive', 'navigation is not active')
            return
        self._publish_empty('waiting', 'waiting for the Nav2 global plan')

    def _terrain_callback(self, message):
        map_name = str(message.map_name).strip()
        if not map_name:
            return
        if self._current_map_name and map_name != self._current_map_name:
            return
        try:
            terrain_data = self._terrain_data(message)
        except (TypeError, ValueError, OverflowError) as error:
            self.get_logger().warning(
                f'Terrain path grid rejected: {error}')
            if map_name == self._terrain_map_name:
                # This also covers terrain arriving before CurrentMap: a newer
                # malformed sample must invalidate an older same-map cache.
                self._terrain_map_name = ''
                self._terrain_frame_id = ''
                self._terrain_data_cache = None
            if map_name == self._current_map_name:
                self._publish_empty('error', str(error))
            return
        # Keep only the two layers needed by the display bridge. The source
        # message also carries four large diagnostic layers, so retaining the
        # complete ROS message here would duplicate them for no benefit.
        self._terrain_map_name = map_name
        self._terrain_frame_id = _frame_id(message.header.frame_id)
        self._terrain_data_cache = terrain_data
        if not self._current_map_name:
            self._status(
                'waiting', 'terrain received before the active map',
                frame_id=_frame_id(message.header.frame_id))
            return
        if map_name != self._current_map_name:
            return
        if self._navigation_active and self._pending_plan is not None:
            self._publish_pending_plan()

    def _plan_callback(self, message):
        if not self._navigation_active:
            return
        if not message.poses:
            self._pending_plan = None
            self._publish_empty('empty', 'Nav2 published an empty plan')
            return
        self._pending_plan = message
        self._publish_pending_plan()

    @staticmethod
    def _terrain_data(message):
        expected = int(message.width) * int(message.height)
        if expected <= 0:
            raise ValueError('terrain grid is empty')
        if len(message.elevation) != expected:
            raise ValueError('terrain elevation layer size is invalid')
        if len(message.traversal_cost) != expected:
            raise ValueError('terrain cost layer size is invalid')
        zeros = np.zeros(expected, dtype=np.float32)
        return TerrainGridData(
            resolution=float(message.resolution),
            width=int(message.width),
            height=int(message.height),
            origin_x=float(message.origin_x),
            origin_y=float(message.origin_y),
            elevation=np.asarray(message.elevation, dtype=np.float32),
            slope_deg=zeros,
            roughness_m=zeros,
            step_height_m=zeros,
            confidence=zeros,
            traversal_cost=np.asarray(
                message.traversal_cost, dtype=np.int8),
        )

    def _publish_pending_plan(self):
        plan = self._pending_plan
        if plan is None or not self._navigation_active:
            return
        if self._navigation_mode == MAP_2D:
            self._publish_planar_path(plan)
            return

        terrain_data = self._matching_terrain()
        if terrain_data is None:
            self._publish_empty(
                'waiting', 'plan cached while waiting for matching terrain',
                source_pose_count=len(plan.poses))
            return

        plan_frame = _frame_id(plan.header.frame_id)
        terrain_frame = self._terrain_frame_id
        if not plan_frame or plan_frame != terrain_frame:
            self._publish_empty(
                'error',
                f'path/terrain frame mismatch: {plan_frame!r} != '
                f'{terrain_frame!r}',
                source_pose_count=len(plan.poses),
            )
            return

        try:
            points = np.asarray([
                [
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z),
                ]
                for pose in plan.poses
            ], dtype=np.float64)
            if not np.isfinite(points).all():
                raise ValueError('Nav2 plan contains a non-finite position')
            indices = simplify_path_indices(
                points,
                minimum_spacing_m=self._minimum_spacing_m,
                maximum_points=self._maximum_points,
            )
            elevated = elevate_path_points(
                points[indices],
                terrain_data,
                clearance_m=self._clearance_m,
            )
            length_m = path_length_m(elevated.points)
        except (TypeError, ValueError, OverflowError) as error:
            self.get_logger().warning(
                f'Terrain path update rejected: {error}')
            self._publish_empty(
                'error', str(error), source_pose_count=len(plan.poses))
            return

        output = Path()
        output.header = copy.deepcopy(plan.header)
        output.header.frame_id = terrain_frame
        for display_index, source_index in enumerate(indices.tolist()):
            pose = copy.deepcopy(plan.poses[source_index])
            pose.header.frame_id = terrain_frame
            pose.pose.position.z = float(elevated.points[display_index, 2])
            output.poses.append(pose)
        self.path_publisher.publish(output)
        self._status(
            'ready',
            'terrain-elevated Nav2 plan ready for display',
            frame_id=terrain_frame,
            source_pose_count=len(plan.poses),
            display_pose_count=len(output.poses),
            lifted_count=elevated.elevated_count,
            fallback_count=elevated.unknown_count,
            length_m=length_m,
        )

    def _publish_planar_path(self, plan):
        """Publish a simplified raw Nav2 plan for 2D map navigation."""
        plan_frame = _frame_id(plan.header.frame_id)
        if not plan_frame:
            self._publish_empty(
                'error', 'Nav2 plan has no frame',
                source_pose_count=len(plan.poses))
            return
        try:
            points = np.asarray([
                [
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z),
                ]
                for pose in plan.poses
            ], dtype=np.float64)
            if not np.isfinite(points).all():
                raise ValueError('Nav2 plan contains a non-finite position')
            indices = simplify_path_indices(
                points,
                minimum_spacing_m=self._minimum_spacing_m,
                maximum_points=self._maximum_points,
            )
            display_points = points[indices]
            length_m = path_length_m(display_points)
        except (TypeError, ValueError, OverflowError) as error:
            self._publish_empty(
                'error', str(error), source_pose_count=len(plan.poses))
            return

        output = Path()
        output.header = copy.deepcopy(plan.header)
        output.header.frame_id = plan_frame
        for source_index in indices.tolist():
            pose = copy.deepcopy(plan.poses[source_index])
            pose.header.frame_id = plan_frame
            output.poses.append(pose)
        self.path_publisher.publish(output)
        self._status(
            'ready',
            'raw Nav2 plan ready for 2D map display',
            frame_id=plan_frame,
            source_pose_count=len(plan.poses),
            display_pose_count=len(output.poses),
            lifted_count=0,
            fallback_count=0,
            length_m=length_m,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TerrainPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
