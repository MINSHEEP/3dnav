#!/usr/bin/env python3
"""Persistent single-goal and ordered-route navigation manager."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
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
from std_srvs.srv import Empty as EmptyService, Trigger

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
    SetInflationRadius,
    SetNavigationMode,
    StartRoute,
    NavigateManagedGoal,
    Preview3DGoal,
    Preview3DRoute,
    Reserve3DInspectionRoute,
    Revoke3DInspectionRoute,
    StartPreviewed3DGoal,
)

from .route_database import RouteDatabase, RouteNameConflict
from .goal_traversability import (
    GoalValidationResult,
    TraversabilityGrid,
    validate_navigation_goal,
)
from .terrain_navigation import (
    TerrainCapabilityModel,
)
from .navigation_mode import (
    MAP_2D,
    TERRAIN_3D,
    NavigationModeStore,
    navigation_mode_label,
    normalize_navigation_mode,
    uses_terrain,
)
from .costmap_inflation import (
    INFLATION_TARGETS,
    InflationRadiusStore,
    InflationSyncState,
    normalize_inflation_radius,
)


def _validate_preview_plan(
        path: NavPath, grid: TraversabilityGrid,
        goal: PoseStamped, *,
        start_xy: Optional[tuple] = None,
        footprint_radius_m: float = 0.0) -> GoalValidationResult:
    """Inspect every crossed terrain cell, not only Nav2's path vertices.

    Cost 90 is reserved for a special traversal that needs separate
    calibration. The production fused map normally raises it to lethal 100.
    Both forms remain a veto to an ordinary single-goal preview.
    """
    if str(path.header.frame_id).lstrip('/') != 'map' or not path.poses:
        return GoalValidationResult(
            False, 'invalid_plan', '规划器未返回 map 坐标系中的有效路径')
    if len(path.poses) > 20_000:
        return GoalValidationResult(
            False, 'invalid_plan', '规划路径点数异常，已拒绝预览')

    points = []
    for stamped in path.poses:
        frame = str(stamped.header.frame_id).lstrip('/')
        if frame and frame != 'map':
            return GoalValidationResult(
                False, 'invalid_plan', '规划路径的坐标系与地图不一致')
        x = float(stamped.pose.position.x)
        y = float(stamped.pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return GoalValidationResult(
                False, 'invalid_plan', '规划路径含有无效坐标')
        points.append((x, y))

    if start_xy is not None:
        start_x, start_y = float(start_xy[0]), float(start_xy[1])
        if (not math.isfinite(start_x) or not math.isfinite(start_y)
                or math.hypot(
                    points[0][0] - start_x,
                    points[0][1] - start_y) > 0.35):
            return GoalValidationResult(
                False, 'invalid_plan',
                '规划路径起点与机器人当前定位不一致')
        # Revalidate the actual connector too, rather than accepting its
        # length while only inspecting cells on the old previewed path.
        points.insert(0, (start_x, start_y))

    requested = goal.pose.position
    if math.hypot(
            points[-1][0] - float(requested.x),
            points[-1][1] - float(requested.y)) > 0.25:
        return GoalValidationResult(
            False, 'invalid_plan', '规划终点与所选目标不一致')

    radius = float(footprint_radius_m)
    if not math.isfinite(radius) or radius < 0.0 or radius > 2.0:
        return GoalValidationResult(
            False, 'invalid_footprint', '3D 路线占地半径配置无效')
    cosine = math.cos(float(grid.origin_yaw)) if radius else 1.0
    sine = math.sin(float(grid.origin_yaw)) if radius else 0.0
    sampled = 0
    footprint_checks = 0
    checked_footprint_tiles = set()
    for index, end in enumerate(points):
        start = points[index - 1] if index else end
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance > 100.0:
            return GoalValidationResult(
                False, 'invalid_plan', '规划路径存在异常长的跳跃段')
        steps = max(1, int(math.ceil(distance / max(
            float(grid.resolution) * 0.5, 0.01))))
        sampled += steps + 1
        if sampled > 200_000:
            return GoalValidationResult(
                False, 'invalid_plan', '规划路径过长，无法完成逐格安全检查')
        for step in range(steps + 1):
            fraction = step / steps
            x = start[0] + (end[0] - start[0]) * fraction
            y = start[1] + (end[1] - start[1]) * fraction
            cell = grid.world_to_cell(x, y)
            if cell is None:
                return GoalValidationResult(
                    False, 'path_outside_map', '规划路径越出当前三维地形图')
            cost = int(grid.data[cell])
            if cost < 0:
                return GoalValidationResult(
                    False, 'path_unknown', '规划路径经过未确认地面，禁止执行')
            if cost >= 90:
                return GoalValidationResult(
                    False, 'needs_calibration',
                    '规划路径经过禁行或特殊地形（含楼梯/台阶）；'
                    '未完成专项标定前禁止执行')
            if radius:
                local_x = (cosine * (x - grid.origin_x)
                           + sine * (y - grid.origin_y))
                local_y = (-sine * (x - grid.origin_x)
                           + cosine * (y - grid.origin_y))
                resolution = float(grid.resolution)
                # Cache a conservative quarter-cell square, not just the
                # center cell: any footprint position in that square must be
                # clear. This keeps long routes bounded without overlooking
                # hazards near a sub-cell edge.
                tile_size = resolution * 0.25
                tile_col = math.floor(local_x / tile_size)
                tile_row = math.floor(local_y / tile_size)
                if (tile_row, tile_col) in checked_footprint_tiles:
                    continue
                tile_x_min = tile_col * tile_size
                tile_x_max = tile_x_min + tile_size
                tile_y_min = tile_row * tile_size
                tile_y_max = tile_y_min + tile_size
                min_col = math.floor((tile_x_min - radius) / resolution)
                max_col = math.floor((tile_x_max + radius) / resolution)
                min_row = math.floor((tile_y_min - radius) / resolution)
                max_row = math.floor((tile_y_max + radius) / resolution)
                for row in range(min_row, max_row + 1):
                    cell_y_min = row * resolution
                    cell_y_max = cell_y_min + resolution
                    gap_y = max(0.0, cell_y_min - tile_y_max,
                                tile_y_min - cell_y_max)
                    for col in range(min_col, max_col + 1):
                        cell_x_min = col * resolution
                        cell_x_max = cell_x_min + resolution
                        gap_x = max(0.0, cell_x_min - tile_x_max,
                                    tile_x_min - cell_x_max)
                        if gap_x ** 2 + gap_y ** 2 > radius ** 2:
                            continue
                        footprint_checks += 1
                        if footprint_checks > 5_000_000:
                            return GoalValidationResult(
                                False, 'invalid_plan',
                                '3D 路线过长，无法完成机身占地安全检查')
                        if not (0 <= row < grid.height
                                and 0 <= col < grid.width):
                            return GoalValidationResult(
                                False, 'footprint_outside_map',
                                '机身占地范围越出当前三维地形图')
                        footprint_cost = int(grid.data[row, col])
                        if footprint_cost < 0:
                            return GoalValidationResult(
                                False, 'footprint_unknown',
                                '机身占地范围存在未确认地面，禁止执行')
                        if footprint_cost >= 90:
                            return GoalValidationResult(
                                False, 'footprint_blocked',
                                '机身占地范围存在禁行或特殊地形，禁止执行')
                checked_footprint_tiles.add((tile_row, tile_col))

    return GoalValidationResult(True, 'ok', '路径逐格预检通过')


class RouteNavigationManager(Node):
    """Own the only web-facing NavigateToPose client and route state machine."""

    VALID_CONTROLS = {'pause', 'resume', 'skip', 'stop', 'release'}
    TRANSIENT_PREFLIGHT_CODES = frozenset({
        'localization_stale',
        'global_localization_recovering',
        'terrain_not_ready',
        'robot_outside_map',
        'robot_not_traversable',
    })

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
        # Patrol points normally need a reliable position, not a forced final
        # heading. Requiring yaw here made near-goal turn drift restart Nav2
        # corrections indefinitely. Enable only for routes that truly need a
        # fixed camera direction.
        self._settled_requires_yaw = bool(self.declare_parameter(
            'settled_requires_yaw', False).value)
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
        self._goal_preflight_pose_max_age = max(
            0.5, float(self.declare_parameter(
                'goal_preflight_pose_max_age_sec', 3.0).value))
        self._goal_preflight_tolerance = max(
            0.0, float(self.declare_parameter(
                'goal_preflight_tolerance_m', 0.10).value))
        self._goal_preflight_start_tolerance = max(
            0.0, float(self.declare_parameter(
                'goal_preflight_start_tolerance_m', 0.35).value))
        self._goal_preflight_wait_timeout = max(
            1.0, float(self.declare_parameter(
                'goal_preflight_wait_timeout_sec', 8.0).value))
        self._preview_plan_timeout = max(
            1.0, min(10.0, float(self.declare_parameter(
                'preview_3d_plan_timeout_sec', 5.0).value)))
        # Human review has no countdown; identity/content changes revoke proof.
        self._nomotion_update_interval = max(
            0.1, float(self.declare_parameter(
                'goal_preflight_nomotion_interval_sec', 0.35).value))
        self._nomotion_update_max_requests = max(
            1, int(self.declare_parameter(
                'goal_preflight_nomotion_max_requests', 8).value))
        self._costmap_clear_timeout = max(
            0.5, float(self.declare_parameter(
                'recovery_costmap_clear_timeout_sec', 2.0).value))
        self._database = RouteDatabase(db_path or None)
        self._terrain_capability = TerrainCapabilityModel()
        mode_path = str(self.declare_parameter(
            'navigation_mode_config_path', '').value).strip()
        self._navigation_mode_store = NavigationModeStore(
            mode_path or (
                Path.home() / '.ros' / 'robot_navigation' /
                'navigation_mode.json'))
        self._navigation_mode = self._navigation_mode_store.snapshot()['mode']
        self._active_navigation_mode = ''
        inflation_path = str(self.declare_parameter(
            'inflation_radius_config_path', '').value).strip()
        self._inflation_radius_store = InflationRadiusStore(
            inflation_path or (
                Path.home() / '.ros' / 'robot_navigation' /
                'inflation_radius.json'))
        self._inflation_sync = InflationSyncState(
            self._inflation_radius_store.snapshot())
        self._inflation_sync_timeout = max(
            0.5, float(self.declare_parameter(
                'inflation_sync_timeout_sec', 3.0).value))
        self._inflation_retry_delay = max(
            0.2, float(self.declare_parameter(
                'inflation_retry_delay_sec', 1.0).value))
        self._inflation_futures = {}
        self._inflation_retry_due = 0.0

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
        self._preview_action_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose',
            callback_group=self._callback_group)
        self.create_subscription(
            PoseStamped,
            '/web_goal_pose',
            self._legacy_web_goal_callback,
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
        # A software stop must also revoke pending goals/preview generations.
        # Otherwise a later manual unlock + page-entry standing confirmation
        # could re-authorize the old task still running inside Nav2.
        self.create_subscription(
            Empty, '/motion_stop', self._cancel_navigation_callback,
            volatile_qos, callback_group=self._callback_group)
        self.create_subscription(
            Empty, '/emergency_stop', self._cancel_navigation_callback,
            volatile_qos, callback_group=self._callback_group)
        self.create_subscription(
            CurrentMap,
            '/current_map_info',
            self._current_map_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String, '/robot_body/status', self._robot_body_status_callback,
            transient_qos, callback_group=self._callback_group)
        self.create_subscription(String, '/localization/status', self._global_localization_callback,
            transient_qos, callback_group=self._callback_group)
        self.create_subscription(
            String, '/navigation/controller_timing', self._controller_timing_callback,
            transient_qos, callback_group=self._callback_group)
        self.create_subscription(
            String, '/navigation/goal_ui_event', self._goal_ui_event_callback,
            volatile_qos, callback_group=self._callback_group)
        self.create_subscription(
            String,
            '/lio_health',
            self._lio_health_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/navigation_pose',
            self._amcl_pose_callback,
            amcl_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            OccupancyGrid,
            '/terrain_costmap',
            self._terrain_costmap_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/terrain_costmap/status',
            self._terrain_costmap_status_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/relocalization/status',
            self._relocalization_status_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            '/campus_mission/active',
            self._campus_mission_active_callback,
            transient_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/inspection/status',
            self._inspection_status_callback,
            transient_qos,
            callback_group=self._callback_group,
        )

        self._goal_status_publisher = self.create_publisher(
            String, '/goal_pose_status', transient_qos)
        self._nav2_ready_publisher = self.create_publisher(
            Bool, '/nav2_ready', transient_qos)
        self._navigation_active_publisher = self.create_publisher(
            Bool, '/navigation_active', transient_qos)
        self._navigation_pause_publisher = self.create_publisher(
            Bool, '/navigation/motion_paused', transient_qos)
        self._route_status_publisher = self.create_publisher(
            RouteExecutionStatus, '/route_execution_status', transient_qos)
        self._route_log_publisher = self.create_publisher(
            String, '/route_navigation_log', volatile_qos)
        self._navigation_mode_publisher = self.create_publisher(
            String, '/navigation/mode', transient_qos)
        self._inflation_radius_publisher = self.create_publisher(
            String, '/navigation/inflation_radius', transient_qos)
        self._zero_velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel_nav', 10)
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
        self._nomotion_update_client = self.create_client(
            EmptyService,
            '/request_nomotion_update',
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
        self._inflation_parameter_clients = {
            'global': self.create_client(
                SetParameters,
                '/global_costmap/global_costmap/set_parameters',
                callback_group=self._callback_group,
            ),
            'local': self.create_client(
                SetParameters,
                '/local_costmap/local_costmap/set_parameters',
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
        self.create_service(
            NavigateManagedGoal,
            '/navigate_managed_goal',
            self._managed_goal_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Preview3DGoal,
            '/navigation/preview_3d_goal',
            self._preview_3d_goal_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Preview3DRoute,
            '/navigation/preview_3d_route',
            self._preview_3d_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Reserve3DInspectionRoute,
            '/navigation/reserve_3d_inspection_route',
            self._reserve_3d_inspection_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            Revoke3DInspectionRoute,
            '/navigation/revoke_3d_inspection_route',
            self._revoke_3d_inspection_route_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            StartPreviewed3DGoal,
            '/navigation/start_previewed_3d_goal',
            self._start_previewed_3d_goal_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetNavigationMode,
            '/navigation/set_mode',
            self._set_navigation_mode_callback,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetInflationRadius,
            '/navigation/set_inflation_radius',
            self._set_inflation_radius_callback,
            callback_group=self._callback_group,
        )

        self._current_map_name = ''
        self._current_map_load_id = ''
        self._map_generation = 0
        self._robot_body_status = None
        self._global_localization_status = None
        self._global_localization_at = 0.
        self._controller_timing_status = None
        self._controller_timing_received_at = None
        self._robot_body_received_at = None
        self._robot_body_status_samples = 0
        self._preview_busy = False
        self._preview_request_generation = 0
        self._preview_token = None
        self._inspection_route_authorization = None
        self._campus_mission_active = False
        self._inspection_task_active = False
        self._inspection_task_id = ''
        self._inspection_task_paused = False
        self._mode = 'idle'
        self._state = 'stopped'
        self._status_message = 'Execution state reset after manager start'
        self._route: Optional[Dict] = None
        self._point_index = -1
        self._current_loop = 0
        self._loop_count = 1
        self._continuous = False
        self._reverse = False
        self._hold_at_each_point = False
        self._owner_token = ''
        self._managed_goal_requires_yaw = False
        self._point_limit = 0
        self._points_executed = 0
        self._dwell_deadline: Optional[float] = None
        self._paused_dwell_remaining = 0.0
        self._point_event_sequence = 0
        self._last_point_id = 0
        self._last_point_index = -1
        self._last_point_name = ''
        self._last_point_result = ''
        self._last_point_detail = ''
        self._goal_event_sequence = 0
        self._last_goal_owner_token = ''
        self._last_goal_result = ''
        self._last_goal_detail = ''

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
        self._latest_amcl_stamp_ns = 0
        self._preflight_wait_context: Optional[str] = None
        self._preflight_wait_pose: Optional[PoseStamped] = None
        self._preflight_wait_code = ''
        self._preflight_wait_deadline: Optional[float] = None
        self._preflight_wait_next_nomotion_at: Optional[float] = None
        self._preflight_wait_nomotion_requests = 0
        # A web single-goal refresh deliberately stores no goal pose.  A new
        # click is required after a new AMCL sample has been confirmed.
        self._single_refresh_map_name = ''
        self._single_refresh_baseline_stamp_ns = 0
        self._single_refresh_started_at: Optional[float] = None
        self._single_refresh_deadline: Optional[float] = None
        self._single_refresh_next_nomotion_at: Optional[float] = None
        self._single_refresh_nomotion_requests = 0
        self._single_refresh_future = None
        self._traversability_grid: Optional[TraversabilityGrid] = None
        self._traversability_map_name = ''
        self._pending_traversability_grid: Optional[TraversabilityGrid] = None
        self._pending_traversability_stamp_ns = 0
        self._terrain_costmap_ready_map_name = ''
        self._terrain_costmap_ready_load_id = ''
        self._terrain_costmap_ready_stamp_ns = 0
        self._recovery_rough_pose: Optional[
            PoseWithCovarianceStamped] = None
        self._lio_recovery_started_at: Optional[float] = None
        self._auto_recovery_stage = 'idle'
        self._lio_recovery_hard_fault = False
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
                'z': cls._finite(
                    point.get('z', 0.0), f'Point {index + 1} z'),
                'yaw': cls._finite(
                    point.get('yaw'), f'Point {index + 1} yaw'),
                'dwell_seconds': dwell,
                'area_id': str(point.get('area_id', '')).strip()[:128],
                'floor_id': str(point.get('floor_id', '')).strip()[:128],
                'terrain_type': str(
                    point.get('terrain_type', 'flat')
                ).strip().lower() or 'flat',
                'traversal_mode': str(
                    point.get('traversal_mode', 'normal')
                ).strip().lower() or 'normal',
                'transition_id': str(
                    point.get('transition_id', '')).strip()[:128],
                'speed_limit_mps': cls._finite(
                    point.get('speed_limit_mps', 0.0),
                    f'Point {index + 1} speed limit'),
            })
            if validated_points[-1]['speed_limit_mps'] < 0.0:
                raise ValueError(
                    f'Point {index + 1} speed limit must be non-negative')
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
        message.z = float(point.get('z', 0.0))
        message.yaw = float(point.get('yaw', 0.0))
        message.dwell_seconds = float(point.get('dwell_seconds', 0.0))
        message.area_id = str(point.get('area_id', ''))
        message.floor_id = str(point.get('floor_id', ''))
        message.terrain_type = str(point.get('terrain_type', 'flat'))
        message.traversal_mode = str(point.get('traversal_mode', 'normal'))
        message.transition_id = str(point.get('transition_id', ''))
        message.speed_limit_mps = float(point.get('speed_limit_mps', 0.0))
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
                    'z': float(point.z),
                    'yaw': float(point.yaw),
                    'dwell_seconds': float(point.dwell_seconds),
                    'area_id': str(point.area_id),
                    'floor_id': str(point.floor_id),
                    'terrain_type': str(point.terrain_type),
                    'traversal_mode': str(point.traversal_mode),
                    'transition_id': str(point.transition_id),
                    'speed_limit_mps': float(point.speed_limit_mps),
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
        message.navigation_mode = self._effective_navigation_mode()
        message.state = self._state
        message.active = self._mode != 'idle'
        message.paused = self._state in (
            'paused_navigation', 'paused_dwell', 'paused_release',
            'paused_preflight')
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
        message.reverse = bool(self._reverse)
        message.hold_at_each_point = bool(self._hold_at_each_point)
        message.awaiting_release = self._state in (
            'awaiting_release', 'paused_release')
        message.owner_token = self._owner_token
        message.point_limit = int(self._point_limit)
        message.points_executed = int(self._points_executed)
        message.dwell_remaining = self._dwell_remaining()
        point = self._current_point()
        if point is not None:
            message.current_point = self._route_point_to_message(point)
        message.point_event_sequence = int(self._point_event_sequence)
        message.last_point_id = int(self._last_point_id)
        message.last_point_index = int(self._last_point_index)
        message.last_point_name = self._last_point_name
        message.last_point_result = self._last_point_result
        message.last_point_detail = self._last_point_detail
        message.goal_event_sequence = int(self._goal_event_sequence)
        message.last_goal_owner_token = self._last_goal_owner_token
        message.last_goal_result = self._last_goal_result
        message.last_goal_detail = self._last_goal_detail
        message.message = self._status_message
        return message

    def _record_managed_goal_event(self, result: str, detail: str) -> None:
        """Persist a token-correlated terminal event for mission managers."""
        if not self._owner_token:
            return
        self._goal_event_sequence += 1
        self._last_goal_owner_token = self._owner_token
        self._last_goal_result = str(result)
        self._last_goal_detail = str(detail)

    def _record_point_event(self, result: str, detail: str) -> None:
        point = self._current_point()
        self._point_event_sequence += 1
        self._last_point_id = int(point.get('id', 0)) if point else 0
        self._last_point_index = int(
            point.get('sequence', self._point_index)) if point else -1
        self._last_point_name = str(point.get('name', '')) if point else ''
        self._last_point_result = str(result)
        self._last_point_detail = str(detail)

    def _log_goal_event(self, event: str, pose=None, reason: str = '') -> None:
        """Log decisions, not motion; no browser-local history is required."""
        received = getattr(self, '_latest_amcl_received_at', None)
        age = None if received is None else time.monotonic() - received
        if age is not None:
            age = round(max(0.0, age), 3) if math.isfinite(age) else None
        goal = None
        if pose is not None:
            try:
                values = (float(pose.pose.position.x), float(pose.pose.position.y))
                goal = {'x': values[0], 'y': values[1],
                        'frame_id': str(pose.header.frame_id or 'map')[:80]}
                if not all(math.isfinite(value) for value in values):
                    goal = None
            except (AttributeError, TypeError, ValueError, OverflowError):
                goal = None
        record = {
            'event': str(event)[:100],
            'map_name': str(getattr(self, '_current_map_name', ''))[:80],
            'map_load_id': str(getattr(self, '_current_map_load_id', ''))[:128],
            'navigation_mode': (getattr(self, '_active_navigation_mode', '')
                                or getattr(self, '_navigation_mode', '')),
            'task_mode': getattr(self, '_mode', ''),
            'state': getattr(self, '_state', ''),
            'busy_state': {
                'campus_active': bool(getattr(self, '_campus_mission_active', False)),
                'inspection_active': bool(getattr(self, '_inspection_task_active', False)),
                'inspection_task_id': str(getattr(self, '_inspection_task_id', ''))[:128],
                'lio_recovering': bool(getattr(self, '_lio_recovering', False)),
                'recovery_stage': str(getattr(self, '_auto_recovery_stage', 'idle')),
                'single_refresh_map': str(getattr(self, '_single_refresh_map_name', ''))[:80],
                'goal_handle_present': getattr(self, '_active_goal_handle', None) is not None,
                'goal_sending': bool(getattr(self, '_sending', False)),
            },
            'amcl_age_sec': age,
            'goal': goal,
            'reason': str(reason or getattr(self, '_status_message', ''))[:600],
        }
        text = 'NAV_GOAL ' + json.dumps(record, ensure_ascii=False, allow_nan=False)
        logger = self.get_logger()
        if 'rejected' in event or 'failed' in event or 'manual_reset' in event:
            logger.warning(text)
        else:
            getattr(logger, 'info', logger.warning)(text)

    def _goal_ui_event_callback(self, message: String) -> None:
        """Accept bounded diagnostics only; this topic can never start motion."""
        try:
            raw = str(message.data)
            if len(raw) > 1600:
                return
            event = json.loads(raw)
            if not isinstance(event, dict) or event.get('event') != 'page_notice':
                return
            reason = event.get('reason')
            page_map = event.get('map_name', '')
            if (not isinstance(reason, str) or not 1 <= len(reason) <= 600
                    or not isinstance(page_map, str) or len(page_map) > 80):
                return
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            now = time.monotonic()
            key = (page_map, reason)
            previous_at = getattr(self, '_ui_goal_event_at', float('-inf'))
            if (now - previous_at < 0.1 or (
                    key == getattr(self, '_ui_goal_event_key', None)
                    and now - previous_at < 1.0)):
                return
            self._ui_goal_event_at, self._ui_goal_event_key = now, key
            self._log_goal_event('ui_page_notice', reason=f'页面地图 {page_map}: {reason}')

    def _publish_goal_status(self, status: str) -> None:
        if (
            status == self._last_goal_status
            and not status.startswith('goal_rejected')
        ):
            return
        self._last_goal_status = status
        self._log_goal_event(status)
        self._goal_status_publisher.publish(String(data=status))

    def _publish_route_log(self, text: str) -> None:
        self._route_log_publisher.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_all_status(self) -> None:
        self._navigation_pause_publisher.publish(Bool(data=(
            self._mode != 'idle' and (
                self._lio_recovering or self._auto_recovery_stage in (
                    'failed', 'manual_reset_required')))))
        self._nav2_ready_publisher.publish(Bool(data=self._nav2_ready))
        self._navigation_active_publisher.publish(
            Bool(data=self._mode != 'idle'))
        self._route_status_publisher.publish(self._status_message_object())
        self._publish_navigation_mode()
        self._publish_inflation_radius()

    def _effective_navigation_mode(self) -> str:
        return self._active_navigation_mode or self._navigation_mode

    def _navigation_mode_locked(self) -> bool:
        return (
            self._mode != 'idle'
            or self._campus_mission_active
            or self._inspection_task_active)

    def _publish_navigation_mode(self) -> None:
        snapshot = self._navigation_mode_store.snapshot()
        locked = self._navigation_mode_locked()
        payload = {
            **snapshot,
            'label': navigation_mode_label(snapshot['mode']),
            'locked': locked,
            'active_mode': (
                self._active_navigation_mode if self._mode != 'idle' else ''),
            'execution_mode': self._mode,
            'goal_preflight_pose_max_age_sec': getattr(
                self, '_goal_preflight_pose_max_age', 3.0),
            'inspection_active': self._inspection_task_active,
            'campus_mission_active': self._campus_mission_active,
            'preview_blockers': self._navigation_busy_details(),
            'message': (
                '导航或巡检正在运行，模式已锁定'
                if locked else '可在空闲状态切换导航模式'),
        }
        self._navigation_mode_publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))))

    def _set_navigation_mode_callback(self, request, response):
        with self._lock:
            try:
                requested = normalize_navigation_mode(request.mode)
            except ValueError as error:
                response.success = False
                response.message = str(error)
                response.mode = self._navigation_mode
                response.version = int(
                    self._navigation_mode_store.snapshot()['version'])
                return response
            if (
                    requested != self._navigation_mode
                    and self._navigation_mode_locked()):
                response.success = False
                response.message = (
                    '导航模式已锁定；请先停止巡检、路线、单点或园区导航')
                response.mode = self._navigation_mode
                response.version = int(
                    self._navigation_mode_store.snapshot()['version'])
                self._publish_navigation_mode()
                return response
            try:
                snapshot = self._navigation_mode_store.set(
                    requested, updated_by='settings-page')
            except OSError as error:
                response.success = False
                response.message = f'导航模式保存失败：{error}'
                response.mode = self._navigation_mode
                response.version = int(
                    self._navigation_mode_store.snapshot()['version'])
                return response
            changed = snapshot['mode'] != self._navigation_mode
            self._navigation_mode = snapshot['mode']
            if changed:
                self._preview_token = None
                # Never let a raw 2D grid satisfy the first terrain-mode goal
                # while the costmap adapter is still republishing its new
                # policy. The next matching status+stamp pair repopulates it.
                self._traversability_grid = None
                self._traversability_map_name = ''
                self._pending_traversability_grid = None
                self._pending_traversability_stamp_ns = 0
                self._terrain_costmap_ready_map_name = ''
                self._terrain_costmap_ready_stamp_ns = 0
            response.success = True
            response.message = (
                f'已切换到{navigation_mode_label(self._navigation_mode)}')
            response.mode = self._navigation_mode
            response.version = int(snapshot['version'])
            self._publish_all_status()
            return response

    def _inflation_ready_for_navigation(self) -> bool:
        return self._inflation_sync.ready

    def _publish_inflation_radius(self) -> None:
        persisted = self._inflation_radius_store.snapshot()
        sync = self._inflation_sync.snapshot()
        locked = self._navigation_mode_locked()
        if locked:
            message = '导航或巡检正在运行，膨胀半径已锁定'
        elif sync['applied']:
            message = (
                f'全局和局部代价地图已应用 {persisted["radius_m"]:.2f} 米')
        elif self._nav2_ready:
            message = '膨胀半径已保存，正在同步全局和局部代价地图'
        else:
            message = '膨胀半径已保存，等待 Nav2 启动后应用'
        payload = {
            **persisted,
            **sync,
            'locked': locked,
            'nav2_ready': self._nav2_ready,
            'message': message,
        }
        self._inflation_radius_publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))))

    def _set_inflation_radius_callback(self, request, response):
        with self._lock:
            current = self._inflation_radius_store.snapshot()
            try:
                requested = normalize_inflation_radius(request.radius_m)
            except ValueError as error:
                response.success = False
                response.applied = self._inflation_sync.ready
                response.message = str(error)
                response.radius_m = float(current['radius_m'])
                response.version = int(current['version'])
                return response
            if (
                requested != current['radius_m']
                and self._navigation_mode_locked()
            ):
                response.success = False
                response.applied = self._inflation_sync.ready
                response.message = (
                    '膨胀半径已锁定；请先停止巡检、路线、单点或园区导航')
                response.radius_m = float(current['radius_m'])
                response.version = int(current['version'])
                self._publish_inflation_radius()
                return response
            try:
                configured = self._inflation_radius_store.set(
                    requested, updated_by='settings-page')
            except OSError as error:
                response.success = False
                response.applied = self._inflation_sync.ready
                response.message = f'膨胀半径保存失败：{error}'
                response.radius_m = float(current['radius_m'])
                response.version = int(current['version'])
                return response
            if (
                configured['version'] != current['version']
                or configured['radius_m'] != current['radius_m']
            ):
                self._inflation_sync.configure(configured)
                self._inflation_retry_due = 0.0
            self._drive_inflation_radius_sync()
            response.success = True
            response.applied = self._inflation_sync.ready
            response.radius_m = float(configured['radius_m'])
            response.version = int(configured['version'])
            response.message = (
                f'膨胀半径 {configured["radius_m"]:.2f} 米已应用到两个代价地图'
                if response.applied else
                f'膨胀半径 {configured["radius_m"]:.2f} 米已保存，正在应用')
            self._publish_inflation_radius()
            return response

    def _reset_inflation_runtime(self, reason: str) -> None:
        for pending in self._inflation_futures.values():
            try:
                pending['future'].cancel()
            except Exception:
                pass
        self._inflation_futures = {}
        self._inflation_sync.reset(reason)
        self._inflation_retry_due = 0.0

    def _collect_inflation_results(self, now: float) -> None:
        for target, pending in list(self._inflation_futures.items()):
            future = pending['future']
            if not future.done() and now - pending['started_at'] <= (
                    self._inflation_sync_timeout):
                continue
            self._inflation_futures.pop(target, None)
            if not future.done():
                try:
                    future.cancel()
                except Exception:
                    pass
                self._inflation_sync.mark_result(
                    target,
                    radius_m=pending['radius_m'],
                    version=pending['version'],
                    success=False,
                    reason='parameter service timeout',
                )
                continue
            try:
                result = future.result()
                entries = list(getattr(result, 'results', []))
                successful = len(entries) == 1 and bool(entries[0].successful)
                reason = '' if successful else (
                    str(entries[0].reason) if entries else
                    'parameter service returned no result')
            except Exception as error:
                successful = False
                reason = str(error)
            self._inflation_sync.mark_result(
                target,
                radius_m=pending['radius_m'],
                version=pending['version'],
                success=successful,
                reason=reason,
            )

    def _drive_inflation_radius_sync(self) -> None:
        now = time.monotonic()
        self._collect_inflation_results(now)
        if not self._nav2_ready or self._inflation_sync.ready:
            return
        if now < self._inflation_retry_due:
            return
        configured = self._inflation_radius_store.snapshot()
        for target in INFLATION_TARGETS:
            if (
                not self._inflation_sync.needs(target)
                or target in self._inflation_futures
            ):
                continue
            client = self._inflation_parameter_clients[target]
            if not client.service_is_ready():
                self._inflation_sync.mark_unavailable(
                    target, 'parameter service unavailable')
                continue
            request = SetParameters.Request()
            request.parameters = [Parameter(
                name='inflation_layer.inflation_radius',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(configured['radius_m']),
                ),
            )]
            try:
                future = client.call_async(request)
                self._inflation_futures[target] = {
                    'future': future,
                    'radius_m': float(configured['radius_m']),
                    'version': int(configured['version']),
                    'started_at': now,
                }
            except Exception as error:
                self._inflation_sync.mark_result(
                    target,
                    radius_m=configured['radius_m'],
                    version=configured['version'],
                    success=False,
                    reason=str(error),
                )
        self._inflation_retry_due = now + self._inflation_retry_delay

    def _timer_callback(self) -> None:
        with self._lock:
            failure = self._global_localization_check()
            if failure and self._mode != 'idle' and self._state != 'stopping':
                self._preview_token = None
                if failure.code == 'global_localization_recovering':
                    self._send_zero_velocity()
                    self._publish_all_status()
                    return  # Keep task/goal; do not dispatch the next point.
                self._publish_route_log(failure.message)
                self._request_stop('global_localization_untrusted')
            ready = self._action_client.server_is_ready()
            if ready != self._nav2_ready:
                self._nav2_ready = ready
                self._reset_inflation_runtime(
                    'waiting for Nav2 costmaps' if ready else 'Nav2 unavailable')
                if not self._single_refresh_map_name:
                    self._publish_goal_status(
                        'nav2_ready' if ready else 'waiting_for_nav2')

            self._drive_inflation_radius_sync()
            self._drive_single_localization_refresh()

            if (
                self._mode == 'route'
                and self._state == 'waiting_preflight'
            ):
                self._drive_preflight_wait()
            elif (
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
                and self._inflation_ready_for_navigation()
                and not self._sending
                and self._active_goal_handle is None
            ):
                self._dispatch_current_route_point()
            elif (
                self._mode == 'single'
                and self._state == 'waiting_nav2'
                and ready
                and self._inflation_ready_for_navigation()
                and self._pending_single_pose is not None
            ):
                pose = self._pending_single_pose
                self._pending_single_pose = None
                self._dispatch_web_single_pose(pose)

            self._drive_auto_recovery()
            self._publish_all_status()

    def _global_localization_required(self):
        return (self._effective_navigation_mode() == TERRAIN_3D
                and (getattr(self, '_robot_body_status', None) or {}).get(
                    'config', {}).get('selected') == 'xingji')

    def _global_localization_check(self):
        data = getattr(self, '_global_localization_status', None) or {}
        if not self._global_localization_required():
            if data and (data.get('scope') != 'legacy' or data.get('ready') is not True
                    or data.get('navigation_mode') != self._effective_navigation_mode()
                    or time.monotonic()-getattr(self, '_global_localization_at', 0.) > .3):
                return GoalValidationResult(False, 'localization_tf_handoff_pending',
                    '正在恢复原AMCL定位输出，完成参数回读后才能启动导航')
            return None
        status_age = time.monotonic()-getattr(self, '_global_localization_at', 0.)
        identity_valid = (data.get('scope') == 'xingji_3d'
                 and data.get('map_name') == self._current_map_name
                 and data.get('map_load_id') == self._current_map_load_id
                 and data.get('body_generation') == (self._robot_body_status or {}).get('generation'))
        if identity_valid and data.get('ready') is True and 0 <= status_age <= .3:
            return None
        if (identity_valid and .3 < status_age <= 1.5
                and data.get('ready') is True and data.get('sensor_fresh') is True):
            return GoalValidationResult(False,'global_localization_recovering',
                '定位状态心跳短时延迟，已零速等待；当前任务保留，持续掉流则停止')
        remaining = data.get('recovery_remaining_sec')
        timing = data.get('lio_timing') or {}
        def bounded_age(value, lower):
            return (type(value) in (int,float) and math.isfinite(value) and lower <= value
                    and value+max(0.,status_age) <= 1.500001)
        sensor_hold = (data.get('reason') in ('sensor_recovery','identity_recovery')
            and data.get('sensor_recovering') is True and isinstance(timing,dict)
            and timing.get('state') in ('tf_stale','tf_lookup_missing','sample_gap',
                                        'verifying_stream','waiting_global_match')
            and bounded_age(timing.get('source_age_sec'),-.1)
            and bounded_age(timing.get('receipt_age_sec'),0.))
        match_hold = data.get('sensor_fresh') is True and data.get('reason') == 'matching_recovery'
        identity_hold = data.get('sensor_fresh') is True and data.get('reason') == 'identity_recovery'
        if (identity_valid and data.get('ready') is False
                and data.get('recovering') is True and (sensor_hold or match_hold or identity_hold)
                and type(remaining) in (float, int) and math.isfinite(remaining)
                and 0 <= status_age <= 1.5
                # The authority owns the one fixed recovery deadline. A
                # sensor/identity hold may become match verification without
                # starting a new window or changing this downstream budget.
                and status_age < remaining <= 3.001):
            return GoalValidationResult(False, 'global_localization_recovering',
                '定位状态、数据流或三维匹配正在限时复核，已零速停车；保留当前任务，可信观测恢复后继续')
        return GoalValidationResult(False, 'global_localization_untrusted',
            '统一三维定位未确认：'+str(data.get('error') or data.get('reason') or '等待定位状态')
            +'。请停止任务、重新确认初始点并预览后再启动。')

    def _global_localization_callback(self, message):
        with self._lock:
            try:
                self._global_localization_status = json.loads(message.data)
                if not isinstance(self._global_localization_status, dict):
                    raise ValueError("localization status must be an object")
            except (ValueError, TypeError):
                self._global_localization_status = {}
            self._global_localization_at = time.monotonic()
            failure = self._global_localization_check()
            if failure and self._mode != 'idle' and self._state != 'stopping':
                self._preview_token = None
                if failure.code == 'global_localization_recovering':
                    self._send_zero_velocity()
                    if not getattr(self, '_global_match_hold', False):
                        self._publish_route_log(failure.message)
                    self._global_match_hold = True
                    return
                self._publish_route_log(failure.message)
                self._request_stop('global_localization_untrusted')
            elif not failure and getattr(self, '_global_match_hold', False):
                self._global_match_hold = False
                if self._mode != 'idle':
                    self._publish_route_log('连续三维匹配复核通过，定位已恢复；原任务按当前暂停状态继续。')

    def _amcl_pose_callback(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        # Historical field/method names are retained for the refresh protocol;
        # the subscription now receives only the scoped navigation pose.
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
                stamp_ns = (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
                if (
                    stamp_ns > 0
                    and self._latest_amcl_stamp_ns > 0
                    and stamp_ns <= self._latest_amcl_stamp_ns
                ):
                    return
            except (AttributeError, TypeError, ValueError):
                return
            self._latest_amcl_pose = copy.deepcopy(message)
            self._latest_amcl_received_at = time.monotonic()
            if stamp_ns > 0:
                self._latest_amcl_stamp_ns = stamp_ns

    def _terrain_costmap_callback(self, message: OccupancyGrid) -> None:
        with self._lock:
            try:
                frame_id = str(message.header.frame_id).lstrip('/')
                if frame_id != 'map':
                    raise ValueError(
                        f'terrain costmap frame must be map, got {frame_id}')
                info = message.info
                grid = TraversabilityGrid.from_costs(
                    data=message.data,
                    resolution=float(info.resolution),
                    width=int(info.width),
                    height=int(info.height),
                    origin_x=float(info.origin.position.x),
                    origin_y=float(info.origin.position.y),
                    origin_yaw=self._pose_yaw(info.origin),
                )
                self._pending_traversability_grid = grid
                self._pending_traversability_stamp_ns = (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
                if (
                    self._current_map_name
                    and self._terrain_costmap_ready_map_name
                    == self._current_map_name
                    and (
                        self._effective_navigation_mode() == MAP_2D
                        or (
                            self._current_map_load_id
                            and self._terrain_costmap_ready_load_id
                            == self._current_map_load_id))
                    and self._terrain_costmap_ready_stamp_ns
                    == self._pending_traversability_stamp_ns
                ):
                    self._traversability_grid = grid
                    self._traversability_map_name = self._current_map_name
            except Exception as error:
                self._traversability_grid = None
                self._traversability_map_name = ''
                self._pending_traversability_grid = None
                self._pending_traversability_stamp_ns = 0
                self.get_logger().warning(
                    f'Terrain goal preflight map rejected: {error}')

    def _terrain_costmap_status_callback(self, message: String) -> None:
        with self._lock:
            was_ready_for_active_3d = (
                self._effective_navigation_mode() == TERRAIN_3D
                and bool(self._current_map_load_id)
                and self._terrain_costmap_ready_map_name
                == self._current_map_name
                and self._terrain_costmap_ready_load_id
                == self._current_map_load_id
                and self._terrain_costmap_ready_stamp_ns > 0)
            try:
                status = json.loads(str(message.data))
                status_mode = normalize_navigation_mode(
                    status.get('navigation_mode') or self._navigation_mode)
                if status_mode != self._navigation_mode:
                    return
                ready_map_name = (
                    str(status.get('map_name', '')).strip()
                    if status.get('state') == 'ready' else '')
                ready_stamp_ns = (
                    int(status.get('stamp_ns', 0))
                    if ready_map_name else 0)
                ready_load_id = (
                    str(status.get('map_load_id', '')).strip()
                    if ready_map_name else '')
            except (TypeError, ValueError, json.JSONDecodeError):
                ready_map_name = ''
                ready_stamp_ns = 0
                ready_load_id = ''
            previous_stamp_ns = self._terrain_costmap_ready_stamp_ns
            self._terrain_costmap_ready_map_name = ready_map_name
            self._terrain_costmap_ready_load_id = ready_load_id
            if ready_stamp_ns != self._terrain_costmap_ready_stamp_ns:
                self._preview_token = None
                self._inspection_route_authorization = None
            self._terrain_costmap_ready_stamp_ns = ready_stamp_ns
            if (
                ready_map_name
                and ready_map_name == self._current_map_name
                and (
                    self._effective_navigation_mode() == MAP_2D
                    or (
                        self._current_map_load_id
                        and ready_load_id == self._current_map_load_id))
                and ready_stamp_ns > 0
                and ready_stamp_ns == self._pending_traversability_stamp_ns
                and self._pending_traversability_grid is not None
            ):
                self._traversability_grid = self._pending_traversability_grid
                self._traversability_map_name = ready_map_name
            elif ready_map_name != self._current_map_name:
                self._traversability_grid = None
                self._traversability_map_name = ''
            still_ready_for_active_3d = (
                ready_map_name == self._current_map_name
                and bool(self._current_map_load_id)
                and ready_load_id == self._current_map_load_id
                and ready_stamp_ns > 0)
            if (
                was_ready_for_active_3d
                and (not still_ready_for_active_3d
                     or ready_stamp_ns != previous_stamp_ns)
                and self._mode in ('route', 'single')
                and self._state != 'stopping'
            ):
                # Status loss is not just a future-goal veto. Cancel the
                # currently moving 3D goal via the established stop seam.
                stopped, detail = self._request_stop('external_cancel')
                self._preview_token = None
                self._status_message = (
                    '三维地形图已失效，正在停止导航；'
                    '请确认机器人安全停止后重新加载地图'
                    if stopped else
                    f'三维地形图已失效且导航取消未确认（{detail}）；'
                    '请现场确认并必要时急停')
                self._publish_goal_status('goal_stopped_terrain_unavailable')
                self._publish_all_status()

    def _robot_body_status_callback(self, message: String) -> None:
        with self._lock:
            try:
                status = json.loads(str(message.data))
                if not isinstance(status, dict):
                    raise ValueError('robot body status must be an object')
            except (TypeError, ValueError, json.JSONDecodeError):
                self._robot_body_status = None
                self._robot_body_received_at = None
                self._robot_body_status_samples = 0
                self._preview_token = None
                self._inspection_route_authorization = None
                return
            previous = self._robot_body_status or {}
            if (
                status.get('generation') == previous.get('generation')
                and status.get('active_body') == previous.get('active_body')
            ):
                self._robot_body_status_samples += 1
            else:
                self._robot_body_status_samples = 1
            self._robot_body_status = status
            self._robot_body_received_at = time.monotonic()
            if (
                status.get('generation') != previous.get('generation')
                or status.get('active_body') != previous.get('active_body')
                or status.get('switching')
            ):
                self._preview_token = None
                self._inspection_route_authorization = None

    def _controller_timing_callback(self, message: String) -> None:
        """Observe configuration transactions; never cancel an active goal."""
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict) or status.get('schema') != 1:
                raise ValueError('invalid controller profile status')
        except (ValueError, TypeError, AttributeError) as error:
            status = {'ready': False, 'error': str(error)}
        with self._lock:
            self._controller_timing_status = status
            self._controller_timing_received_at = time.monotonic()

    def _controller_timing_preflight(self, *, navigation_mode=None):
        """Only reject a new goal while an observed configuration is unready.

        Confirmed parameter values do not expire like sensor observations.
        This optional computation profile is not an active-motion heartbeat.
        """
        status = getattr(self, '_controller_timing_status', None)
        if getattr(self, '_mode', 'idle') != 'idle':
            return None  # Never make an active route wait for an idle-only update.
        if status is None:
            return None  # Compatibility with installations without the optimizer.
        body = (getattr(self, '_robot_body_status', None) or {}).get('config', {}).get('selected')
        mode = navigation_mode if navigation_mode is not None else self._effective_navigation_mode()
        expected = 'xingji_3d' if mode == TERRAIN_3D and body == 'xingji' else 'baseline'
        matches = (status.get('navigation_mode') == mode
                   and status.get('body_id') == body
                   and status.get('profile') == expected)
        received_at = getattr(self, '_controller_timing_received_at', None)
        verified_age = status.get('actual_verified_age_sec')
        fresh = (received_at is not None and 0.0 <= time.monotonic() - received_at <= 3.0
                 and type(verified_age) in (int, float) and 0.0 <= verified_age <= 3.0)
        if status.get('ready') is True and matches and fresh:
            return None
        # No waiting loop, task cancellation, or command publication here.
        # A write has a 3 s deadline; unresolved late acknowledgements remain
        # explicitly unconfirmed rather than silently authorizing a new task.
        reason = str(status.get('error') or status.get('message') or '控制器计算参数尚未确认')
        return GoalValidationResult(False, 'controller_profile_pending',
            f'导航尚未启动：{reason}。请等待参数确认后重新预览；'
            '若远端写入持续未确认，请停止导航后台后重启。')

    def _goal_preflight(self, pose: PoseStamped) -> GoalValidationResult:
        localization_check = self._global_localization_check()
        if localization_check is not None:
            return localization_check
        timing_check = getattr(self, '_controller_timing_preflight', None)
        timing_result = timing_check() if timing_check is not None else None
        if timing_result is not None:
            return timing_result
        if not self._current_map_name:
            return GoalValidationResult(
                False, 'terrain_not_ready', '当前地图尚未完成加载')
        if uses_terrain(self._effective_navigation_mode()) and (
                self._traversability_grid is None
                or self._traversability_map_name != self._current_map_name
                or not self._current_map_load_id
                or self._terrain_costmap_ready_load_id
                != self._current_map_load_id):
            return GoalValidationResult(
                False,
                'terrain_not_ready',
                '当前地图的地形代价图尚未就绪，请等待地形状态更新',
            )
        if (
            self._latest_amcl_pose is None
            or self._latest_amcl_received_at is None
            or time.monotonic() - self._latest_amcl_received_at
            > self._goal_preflight_pose_max_age
        ):
            return GoalValidationResult(
                False,
                'localization_stale',
                '机器人实时位置已过期，请重新确认初始定位',
            )
        robot = self._latest_amcl_pose.pose.pose.position
        goal = pose.pose.position
        if not all(math.isfinite(float(value)) for value in (
                robot.x, robot.y, goal.x, goal.y)):
            return GoalValidationResult(
                False,
                'invalid_goal',
                '导航起点或目标点包含无效坐标',
            )
        if self._effective_navigation_mode() == MAP_2D:
            return GoalValidationResult(
                True,
                'allowed_2d',
                '2D地图导航预检通过（地形预检已暂停）',
            )
        return validate_navigation_goal(
            self._traversability_grid,
            start_x=float(robot.x),
            start_y=float(robot.y),
            goal_x=float(goal.x),
            goal_y=float(goal.y),
            goal_tolerance_m=self._goal_preflight_tolerance,
            start_tolerance_m=self._goal_preflight_start_tolerance,
        )

    def _navigation_busy_details(self, *, allow_inspection=False):
        """Report every owner without clearing ownership or resuming motion."""
        blockers = []
        mode = getattr(self, '_mode', 'idle')
        if mode != 'idle':
            label = {'single': '单点导航', 'route': '多点路线'}.get(mode, mode)
            blockers.append(f'{label}尚未结束，请先停止该任务')
        if getattr(self, '_campus_mission_active', False):
            blockers.append('园区任务仍占用导航，请在园区导航页停止任务')
        if getattr(self, '_inspection_task_active', False) and not allow_inspection:
            task_id = str(getattr(self, '_inspection_task_id', ''))[:128]
            blockers.append(f'巡检任务仍占用导航（{task_id or "未提供任务编号"}），'
                            '请在巡检任务页终止任务；暂停不会释放占用')
        if getattr(self, '_lio_recovering', False):
            stage = str(getattr(self, '_auto_recovery_stage', 'unknown'))
            blockers.append(f'里程计恢复尚未完成（阶段 {stage}），请检查定位恢复状态')
        if getattr(self, '_single_refresh_map_name', ''):
            blockers.append('单点定位刷新尚未结束，请等待刷新结果或取消该次导航')
        if getattr(self, '_active_goal_handle', None) is not None:
            blockers.append('Nav2 目标尚未结束，请等待停止确认')
        if getattr(self, '_sending', False):
            blockers.append('Nav2 目标发送尚未完成，请等待目标响应或停止确认')
        return blockers

    def _preview_3d_guard(
            self, pose: PoseStamped, requested_map_name: str,
            requested_map_load_id: str, *, allow_inspection: bool = False
    ) -> GoalValidationResult:
        """Check the scoped 3-D preview/confirm interface under ``_lock``."""
        if self._navigation_mode != TERRAIN_3D or (
                self._effective_navigation_mode() != TERRAIN_3D):
            return GoalValidationResult(
                False, 'wrong_mode', '仅星玑 3D 地形导航模式支持此预览')
        blockers = self._navigation_busy_details(allow_inspection=allow_inspection)
        if blockers:
            return GoalValidationResult(
                False, 'navigation_busy',
                '；'.join(blockers) + '。解除占用后重新预览，无需因此修改路线')
        if not requested_map_name or requested_map_name != self._current_map_name:
            return GoalValidationResult(
                False, 'map_changed', '所选三维地图与当前加载地图不一致')
        if (not requested_map_load_id or not self._current_map_load_id
                or requested_map_load_id != self._current_map_load_id):
            return GoalValidationResult(
                False, 'map_changed',
                '所选目标属于旧地图加载代际，请重新选点')
        if str(pose.header.frame_id).lstrip('/') != 'map':
            return GoalValidationResult(
                False, 'invalid_goal', '目标必须位于 map 坐标系')
        position = pose.pose.position
        rotation = pose.pose.orientation
        coordinates = (
            position.x, position.y, position.z,
            rotation.x, rotation.y, rotation.z, rotation.w)
        try:
            if not all(math.isfinite(float(value)) for value in coordinates):
                raise ValueError('non-finite goal pose')
            norm = math.sqrt(sum(
                float(value) ** 2 for value in (
                    rotation.x, rotation.y, rotation.z, rotation.w)))
            if not 0.95 <= norm <= 1.05:
                raise ValueError('invalid goal orientation')
        except (TypeError, ValueError, OverflowError):
            return GoalValidationResult(
                False, 'invalid_goal', '目标坐标或朝向无效')

        status = self._robot_body_status
        if (
            not isinstance(status, dict)
            or self._robot_body_received_at is None
            or time.monotonic() - self._robot_body_received_at > 2.0
            or self._robot_body_status_samples < 2
        ):
            return GoalValidationResult(
                False, 'body_unavailable',
                '星玑本体状态缺失、未确认新心跳或超过 2 秒未更新')
        config = status.get('config')
        profiles = config.get('profiles') if isinstance(config, dict) else None
        xingji_profile = (
            profiles.get('xingji') if isinstance(profiles, dict) else None)
        worker = status.get('status')
        if not all(isinstance(value, dict) for value in (
                config, xingji_profile, worker)):
            return GoalValidationResult(
                False, 'body_unavailable', '星玑本体状态结构不完整')
        if (
            config.get('selected') != 'xingji'
            or status.get('active_body') != 'xingji'
            or type(status.get('generation')) is not int
            or status['generation'] < 1
            or not status.get('connected')
            or status.get('switching')
            or status.get('error')
            or worker.get('stale') is True
            or worker.get('switching')
            or worker.get('emergency_latched')
            or worker.get('hardware_error') is True
            or worker.get('error')
            or xingji_profile.get('navigation_verified') is not True
        ):
            return GoalValidationResult(
                False, 'body_unavailable',
                '星玑本体未连接、正在切换、存在故障或导航参数未确认')
        if not self._inflation_ready_for_navigation():
            return GoalValidationResult(
                False, 'inflation_not_ready',
                '障碍膨胀半径尚未同步到全局和局部代价地图')
        return self._goal_preflight(pose)

    def _preview_3d_snapshot(self) -> dict:
        """Capture identities that must remain unchanged while planning."""
        position = self._latest_amcl_pose.pose.pose.position
        start_pose = self._latest_amcl_pose.pose.pose
        return {
            'map_name': self._current_map_name,
            'map_load_id': self._current_map_load_id,
            'map_generation': self._map_generation,
            'terrain_stamp_ns': self._terrain_costmap_ready_stamp_ns,
            'body_generation': self._robot_body_status.get('generation'),
            'mode_version': self._navigation_mode_store.snapshot()['version'],
            'inflation_version': self._inflation_sync.snapshot()['version'],
            'start_x': float(position.x),
            'start_y': float(position.y),
            'start_yaw': self._pose_yaw(start_pose),
        }

    def _preview_3d_snapshot_matches(self, snapshot: dict) -> bool:
        if (
            snapshot['map_name'] != self._current_map_name
            or snapshot['map_load_id'] != self._current_map_load_id
            or snapshot['map_generation'] != self._map_generation
            or snapshot['terrain_stamp_ns']
            != self._terrain_costmap_ready_stamp_ns
            or snapshot['body_generation']
            != (self._robot_body_status or {}).get('generation')
            or snapshot['mode_version']
            != self._navigation_mode_store.snapshot()['version']
            or snapshot['inflation_version']
            != self._inflation_sync.snapshot()['version']
        ):
            return False
        position = self._latest_amcl_pose.pose.pose.position
        yaw = self._pose_yaw(self._latest_amcl_pose.pose.pose)
        return (
            math.hypot(
                float(position.x) - snapshot['start_x'],
                float(position.y) - snapshot['start_y']) <= 0.15
            and abs(math.remainder(
                yaw - snapshot['start_yaw'], 2.0 * math.pi))
            <= math.radians(15.0)
        )

    def _preview_3d_identity_matches(self, snapshot: dict) -> bool:
        """Stable map/terrain/body/config identity; pose may move in a patrol."""
        return not (
            snapshot['map_name'] != self._current_map_name
            or snapshot['map_load_id'] != self._current_map_load_id
            or snapshot['map_generation'] != self._map_generation
            or snapshot['terrain_stamp_ns']
            != self._terrain_costmap_ready_stamp_ns
            or snapshot['body_generation']
            != (self._robot_body_status or {}).get('generation')
            or snapshot['mode_version']
            != self._navigation_mode_store.snapshot()['version']
            or snapshot['inflation_version']
            != self._inflation_sync.snapshot()['version']
        )

    def _preview_3d_change_reason(self, snapshot: dict, *, include_pose=True) -> str:
        """Explain a rejected preview without relaxing any acceptance fence."""
        current = self._preview_3d_snapshot()
        labels = {
            'map_name': '地图名称', 'map_load_id': '地图加载批次',
            'map_generation': '地图版本', 'terrain_stamp_ns': '地形地图版本',
            'body_generation': '本体配置', 'mode_version': '导航模式配置',
            'inflation_version': '障碍膨胀配置',
        }
        changes = [label+'已变化' for key, label in labels.items()
                   if snapshot[key] != current[key]]
        distance = math.hypot(current['start_x']-snapshot['start_x'],
                              current['start_y']-snapshot['start_y'])
        yaw_deg = math.degrees(abs(math.remainder(
            current['start_yaw']-snapshot['start_yaw'], 2.0*math.pi)))
        if include_pose and (distance > .15 or yaw_deg > 15.):
            changes.append(f'起点平移 {distance:.3f} m、转角 {yaw_deg:.1f}°'
                           '（允许 0.150 m / 15.0°，含定位校正）')
        return '；'.join(changes) or '预览状态已变化'

    def _recheck_3d_start(self, snapshot: dict, segments: list, *, reuse_preview=False) -> tuple:
        """One start proof for goals, routes and inspection continuations.

        Receipt timestamps are not identities. A trusted correction need not
        discard a preview: inspect the actual <=35 cm connector and footprint
        using the same path validator, keeping map/config and heading fences.
        """
        if not self._preview_3d_identity_matches(snapshot):
            return GoalValidationResult(False,'preview_stale',
                self._preview_3d_change_reason(snapshot,include_pose=False)+'；请重新预览'), None
        current = self._preview_3d_snapshot()
        yaw_deg = math.degrees(abs(math.remainder(
            current['start_yaw']-snapshot['start_yaw'],2.0*math.pi)))
        if not math.isfinite(yaw_deg) or yaw_deg > 15.:
            return GoalValidationResult(False,'preview_stale',
                f'起点朝向变化 {yaw_deg:.1f}°（允许 15.0°），请重新预览'), None
        if reuse_preview:
            # Only a server-created preview under the same terrain identity
            # reuses the full route proof. The actual connector is checked now.
            if not segments or not segments[0][1].poses:
                return GoalValidationResult(False,'invalid_plan','已预览路径缺少起点'), None
            first = segments[0][1].poses[0]
            connector = NavPath()
            connector.header.frame_id = 'map'
            connector.poses = [copy.deepcopy(first)]
            checked = _validate_preview_plan(connector,self._traversability_grid,first,
                start_xy=(current['start_x'],current['start_y']),footprint_radius_m=0.45)
        else:
            checked, _ = self._validate_3d_route_segments(
                segments,(current['start_x'],current['start_y']))
        return checked,current if checked.allowed else None

    @staticmethod
    def _route_digest(route: dict) -> str:
        canonical = json.dumps(
            route, ensure_ascii=False, sort_keys=True,
            allow_nan=False, separators=(',', ':'))
        return hashlib.sha256(canonical.encode('utf-8')).hexdigest()

    @staticmethod
    def _route_preview_points(
            route: dict, *, start_point_id: int = 0,
            point_limit: int = 0, reverse: bool = False,
            loop_count: int = 1, continuous: bool = False) -> list:
        """Actual target order, including one closure for repeated rounds."""
        if continuous:
            raise ValueError('3D 持续循环未开放；请选择有限巡检轮次')
        if loop_count < 1 or loop_count > 10000 or point_limit < 0:
            raise ValueError('3D 路线轮次或目标点限制无效')
        ordered = list(route.get('points') or [])
        if reverse:
            ordered.reverse()
        if not ordered:
            raise ValueError('路线没有目标点')
        first = 0
        if start_point_id > 0:
            matches = [index for index, point in enumerate(ordered)
                       if int(point.get('id', 0)) == start_point_id]
            if not matches:
                raise ValueError('起始点不属于当前路线')
            first = matches[0]
        points = list(ordered[first:])
        # Repeated rounds use the same geometry. One complete wrap proves the
        # additional last->first transition, never an infinite loop.
        if loop_count > 1 and (point_limit == 0 or point_limit > len(points)):
            points.extend(ordered)
        if point_limit > 0:
            points = points[:point_limit]
        if len(points) > 32:
            raise ValueError('3D 路线预览最多支持 32 个连续目标点；请拆分路线')
        return points

    def _plan_3d_route_segments(self, points: list, start_pose: PoseStamped):
        """Use Nav2 only for planning; never send NavigateToPose here."""
        deadline = time.monotonic() + 45.0
        paths = []
        previous = copy.deepcopy(start_pose)
        for index, point in enumerate(points):
            goal_pose = self._route_pose(point)
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            plan_goal = ComputePathToPose.Goal()
            plan_goal.goal = goal_pose
            plan_goal.start = previous
            plan_goal.use_start = True
            plan_goal.planner_id = 'GridBased'
            try:
                send_future = self._preview_action_client.send_goal_async(
                    plan_goal)
                remaining = min(
                    self._preview_plan_timeout,
                    deadline - time.monotonic())
                if not self._await_preview_future(send_future, remaining):
                    return GoalValidationResult(
                        False, 'planner_timeout',
                        f'第 {index + 1} 段路径预览超时'), []
                handle = send_future.result()
                if handle is None or not handle.accepted:
                    return GoalValidationResult(
                        False, 'planner_rejected',
                        f'规划器拒绝第 {index + 1} 段路径'), []
                result_future = handle.get_result_async()
                remaining = min(
                    self._preview_plan_timeout,
                    deadline - time.monotonic())
                if not self._await_preview_future(result_future, remaining):
                    handle.cancel_goal_async()
                    return GoalValidationResult(
                        False, 'planner_timeout',
                        f'第 {index + 1} 段路径计算超时'), []
                result = result_future.result()
                if result.status != GoalStatus.STATUS_SUCCEEDED:
                    return GoalValidationResult(
                        False, 'planner_failed',
                        f'规划器未找到第 {index + 1} 段可通行路径'), []
                paths.append((goal_pose, result.result.path))
                previous = goal_pose
            except Exception as error:
                return GoalValidationResult(
                    False, 'planner_error',
                    f'第 {index + 1} 段路径预览失败：{error}'), []
        return GoalValidationResult(True, 'ready', '整条路线规划完成'), paths

    def _validate_3d_route_segments(
            self, segments: list, start_xy: tuple) -> tuple:
        combined = NavPath()
        combined.header.frame_id = 'map'
        combined.header.stamp = self.get_clock().now().to_msg()
        previous_xy = start_xy
        for index, (goal_pose, path) in enumerate(segments):
            result = _validate_preview_plan(
                path, self._traversability_grid, goal_pose,
                start_xy=previous_xy, footprint_radius_m=0.45)
            if not result.allowed:
                return GoalValidationResult(
                    False, result.code,
                    f'第 {index + 1} 段不安全：{result.message}'), NavPath()
            combined.poses.extend(copy.deepcopy(path.poses))
            if len(combined.poses) > 30000:
                return GoalValidationResult(
                    False, 'invalid_plan',
                    '整条规划路线点数过多，无法安全预览'), NavPath()
            position = goal_pose.pose.position
            previous_xy = (float(position.x), float(position.y))
        return GoalValidationResult(True, 'ready', '整条路线逐段预检通过'), combined

    @staticmethod
    def _await_preview_future(future, timeout_sec: float) -> bool:
        """Wait on one action future without spinning recursively or holding lock."""
        completed = threading.Event()
        future.add_done_callback(lambda _result: completed.set())
        return completed.wait(max(0.0, float(timeout_sec)))

    def _preview_3d_route_callback(self, request, response):
        response.valid = False
        response.code = 'unavailable'
        response.message = '3D 路线预览不可用'
        response.preview_id = ''
        response.map_name = ''
        response.path = NavPath()
        with self._lock:
            self._preview_token = None
            if self._preview_busy:
                response.code = 'preview_busy'
                response.message = '已有 3D 路径预览正在计算'
                return response
            response.map_name = self._current_map_name
            route = self._database.get_route(int(request.route_id))
            if route is None or route['map_name'] != self._current_map_name:
                response.code = 'route_missing'
                response.message = '路线不存在或不属于当前地图'
                return response
            blockers = self._terrain_capability.planar_execution_blockers(
                route['points'], allow_ground_elevation=True)
            if blockers:
                response.code = 'needs_calibration'
                response.message = '路线含有未标定的特殊地形点，禁止执行'
                return response
            try:
                params = {
                    'route_id': int(request.route_id),
                    'start_point_id': int(request.start_point_id),
                    'point_limit': int(request.point_limit),
                    'reverse': bool(request.reverse),
                    'loop_count': int(request.loop_count),
                    'continuous': bool(request.continuous),
                }
                points = self._route_preview_points(route, **{
                    key: value for key, value in params.items()
                    if key != 'route_id'})
                digest = self._route_digest(route)
            except (TypeError, ValueError, OverflowError) as error:
                response.code = 'invalid_route'
                response.message = str(error)
                return response
            first_pose = self._route_pose(points[0])
            requested_map = str(request.map_name).strip()
            requested_load = str(request.map_load_id).strip()
            guard = self._preview_3d_guard(
                first_pose, requested_map, requested_load)
            if not guard.allowed and guard.code != 'localization_stale':
                response.code, response.message = guard.code, guard.message
                self._log_goal_event('3d_route_preview_rejected', first_pose,
                                     reason=f'{guard.code}: {guard.message}')
                return response
            refresh_required = not guard.allowed
            if not (self._preview_action_client.server_is_ready()
                    and self._action_client.server_is_ready()):
                response.code = 'planner_unavailable'
                response.message = 'Nav2 规划器或导航动作尚未就绪'
                return response
            request_identity = (self._preview_3d_snapshot()
                                if self._latest_amcl_pose is not None else None)
            request_generation = getattr(self, '_preview_request_generation', 0)
            self._preview_busy = True

        try:
            if refresh_required:
                refreshed = self._refresh_3d_preview_pose(
                    first_pose, requested_map, requested_load)
                if not refreshed.allowed:
                    response.code, response.message = refreshed.code, refreshed.message
                    return response
            with self._lock:
                current_route = self._database.get_route(params['route_id'])
                if (getattr(self, '_preview_request_generation', 0) != request_generation
                        or (request_identity is not None
                            and not self._preview_3d_identity_matches(request_identity))
                        or current_route is None or self._route_digest(current_route) != digest):
                    response.code = 'preview_stale'
                    response.message = '定位刷新期间路线、地图、本体或预览状态已改变，请重新预览'
                    return response
                for point in points:
                    check = self._goal_preflight(self._route_pose(point))
                    if not check.allowed:
                        response.code, response.message = check.code, check.message
                        return response
                snapshot = self._preview_3d_snapshot()
                start_pose = PoseStamped()
                start_pose.header.frame_id = 'map'
                start_pose.header.stamp = self.get_clock().now().to_msg()
                start_pose.pose = copy.deepcopy(self._latest_amcl_pose.pose.pose)
            planned, segments = self._plan_3d_route_segments(
                points, start_pose)
            if not planned.allowed:
                response.code, response.message = (
                    planned.code, planned.message)
                return response
            with self._lock:
                guard = self._preview_3d_guard(first_pose, requested_map, requested_load)
            # A long, many-point preview may outlive AMCL's stationary update.
            # Refresh real data again, then reject paths if the start changed.
            if not guard.allowed and guard.code == 'localization_stale':
                guard = self._refresh_3d_preview_pose(first_pose, requested_map, requested_load)
            with self._lock:
                if getattr(self, '_preview_request_generation', 0) != request_generation:
                    response.code, response.message = 'preview_canceled', '路线预览已取消；未启动导航'
                    return response
                if guard.allowed:
                    guard = self._preview_3d_guard(first_pose, requested_map, requested_load)
                if not guard.allowed:
                    response.code, response.message = (
                        guard.code, guard.message)
                    return response
                current_route = self._database.get_route(params['route_id'])
                if (current_route is None or
                        self._route_digest(current_route) != digest or
                        not self._preview_3d_snapshot_matches(snapshot)):
                    response.code = 'preview_stale'
                    reason = ('路线数据已变化' if current_route is None or
                              self._route_digest(current_route) != digest else
                              self._preview_3d_change_reason(snapshot))
                    response.message = f'预览已失效：{reason}；请保持静止后重新预览'
                    return response
                checked, combined = self._validate_3d_route_segments(
                    segments, (snapshot['start_x'], snapshot['start_y']))
                if not checked.allowed:
                    response.code, response.message = (
                        checked.code, checked.message)
                    return response
                preview_id = uuid.uuid4().hex
                self._preview_token = {
                    'kind': 'route', 'id': preview_id,
                    'created_at': time.monotonic(),
                    'route_digest': digest, 'params': params,
                    'snapshot': snapshot, 'segments': segments,
                }
                response.valid = True
                response.code = 'ready'
                response.message = (
                    '整条 3D 路线逐段预检通过；仅是当前静态规划证据，'
                    '实际执行仍会重新检查实时地形与定位')
                response.preview_id = preview_id
                response.path = combined
                return response
        except Exception as error:
            self.get_logger().error(
                f'3D route preview failed unexpectedly: {error}')
            response.code = 'preview_error'
            response.message = '3D 路线预览失败，请检查导航服务日志后重试'
            return response
        finally:
            with self._lock:
                self._preview_busy = False
                self._log_goal_event('3d_route_preview_result', first_pose,
                    reason=f'{response.code}: {response.message}')

    def _reserve_3d_inspection_route_callback(self, request, response):
        """Patrol Start may follow a stationary preview by several seconds.

        Refresh without moving, then repeat the original token/snapshot checks.
        This grants a lease only; it never starts navigation itself.
        """
        with self._lock:
            token = self._preview_token
            if (self._preview_busy or not token or token.get('kind') != 'route'
                    or token.get('id') != str(request.preview_id)):
                return self._reserve_3d_inspection_route_checked(request, response)
            route = self._database.get_route(token['params']['route_id'])
            if not route or not route.get('points'):
                return self._reserve_3d_inspection_route_checked(request, response)
            pose = self._route_pose(route['points'][0])
            snapshot = token['snapshot']
            guard = self._preview_3d_guard(pose, snapshot['map_name'], snapshot['map_load_id'])
            if guard.allowed or guard.code != 'localization_stale':
                return self._reserve_3d_inspection_route_checked(request, response)
            self._preview_busy = True
        try:
            refreshed = self._refresh_3d_preview_pose(pose, snapshot['map_name'], snapshot['map_load_id'])
        finally:
            with self._lock:
                self._preview_busy = False
        with self._lock:
            if not refreshed.allowed or self._preview_token is not token:
                if self._preview_token is token:
                    self._preview_token = None
                response.success = False
                response.message = (refreshed.message if not refreshed.allowed
                                    else '巡检预览已取消或更改，请重新预览')
                return response
            return self._reserve_3d_inspection_route_checked(request, response)

    def _reserve_3d_inspection_route_checked(self, request, response):
        response.success = False
        response.message = '3D 巡检路线预览无效，请重新预览'
        with self._lock:
            token = self._preview_token
            if not token or token.get('id') != str(request.preview_id):
                return response
            self._preview_token = None
            if self._preview_busy or token.get('kind') != 'route':
                response.message = '3D 路线预览已失效或正在更新，请重新预览'
                return response
            owner = str(request.owner_token).strip()
            params = token['params']
            if (not owner or len(owner) > 128
                    or bool(request.bidirectional)
                    or int(request.route_id) != params['route_id']
                    or int(request.loop_count) != params['loop_count']
                    or params['reverse'] or params['start_point_id']
                    or params['point_limit'] or params['continuous']):
                response.message = (
                    '3D 巡检暂不支持双向路线；预览参数与本次任务不一致')
                return response
            route = self._database.get_route(params['route_id'])
            if not route or not route.get('points'):
                response.message = '路线不存在或已没有路线点，请重新预览'
                return response
            guard = self._preview_3d_guard(
                self._route_pose(route['points'][0]),
                token['snapshot']['map_name'],
                token['snapshot']['map_load_id'])
            if not guard.allowed:
                response.message = guard.message
                return response
            if self._route_digest(route) != token['route_digest']:
                response.message = '路线内容已变化，请重新预览巡检路线'
                return response
            checked,current = self._recheck_3d_start(
                token['snapshot'],token['segments'],reuse_preview=True)
            if not checked.allowed:
                response.message = checked.message
                return response
            self._inspection_route_authorization = {
                'owner_token': owner,
                'route_id': params['route_id'],
                'route_digest': token['route_digest'],
                'snapshot': current,
                'state': 'pending',
                'generation': 0,
                'pause_latched': False,
            }
            response.success = True
            response.message = '3D 路线预览已绑定本次巡检任务，等待相机预热'
            return response

    def _revoke_3d_inspection_route_callback(self, request, response):
        """Synchronously fence an in-flight StartRoute before acknowledging."""
        response.success = False
        response.motion_stopped = False
        response.route_session_active = False
        response.authorization_generation = 0
        response.message = '巡检路线授权撤销失败'
        owner = str(request.owner_token).strip()
        action = str(request.action).strip().lower()
        if not owner or action not in ('pause', 'resume', 'cancel'):
            response.message = '巡检任务标识或撤销动作无效'
            return response
        with self._lock:
            authorization = self._inspection_route_authorization
            active_owner = (
                self._mode == 'route' and self._owner_token == owner
                and self._effective_navigation_mode() == TERRAIN_3D)
            if authorization and authorization['owner_token'] != owner:
                response.message = '另一巡检任务拥有 3D 路线授权'
                return response
            if action == 'resume' and not authorization:
                response.message = '3D 巡检授权已失效，请重新预览并启动任务'
                return response
            if authorization:
                authorization['generation'] = (
                    int(authorization.get('generation', 0)) + 1)
                response.authorization_generation = authorization['generation']
                authorization['pause_latched'] = action == 'pause'
                if action == 'cancel':
                    self._inspection_route_authorization = None
            if action == 'pause':
                self._inspection_task_paused = True
                if active_owner:
                    if self._state != 'pausing':
                        self._pause_route()
                    elif (self._active_goal_handle is not None
                          and not self._cancel_in_progress):
                        self._cancel_active_goal()
                    self._send_zero_velocity()
            elif action == 'resume':
                self._inspection_task_paused = False
            else:
                self._inspection_task_paused = True
                if active_owner:
                    if self._state != 'stopping':
                        self._request_stop('inspection_cancel')
                    elif (self._active_goal_handle is not None
                          and not self._cancel_in_progress):
                        self._cancel_active_goal()
                    self._send_zero_velocity()
            response.success = True
            response.route_session_active = (
                self._mode == 'route' and self._owner_token == owner)
            response.motion_stopped = (
                not response.route_session_active
                or (action == 'pause' and self._state in (
                    'paused_navigation', 'paused_dwell', 'paused_release',
                    'paused_preflight')))
            response.message = (
                '3D 巡检授权已同步撤销，等待导航确认停止'
                if action == 'cancel' else
                '3D 巡检暂停屏障已建立，等待导航确认暂停'
                if action == 'pause' else
                '3D 巡检授权已解除暂停屏障')
            return response

    def _prepare_3d_route_start(self, request) -> tuple:
        """Refresh a confirmed start and revalidate its single-use evidence.

        Patrol already has an explicit task Start plus a generation-fenced
        lease; after refresh it must revalidate that lease and replan.
        """
        result, ticket = self._prepare_3d_route_start_checked(request)
        if result.code == 'global_localization_recovering':
            owner = bool(str(request.owner_token).strip())
            token = (ticket or {}).get('refresh_token')
            generation = (ticket or {}).get('request_generation')
            # This waits only for the current explicit Start or a valid
            # inspection lease. Never restore the consumed preview token.
            started = time.monotonic()
            deadline = started+3.1
            hard_deadline = deadline+3.0
            while result.code == 'global_localization_recovering':
                with self._lock:
                    now = time.monotonic()
                    data = getattr(self, '_global_localization_status', None) or {}
                    status_age = now-getattr(self, '_global_localization_at', 0.)
                    remaining = data.get('recovery_remaining_sec')
                    # The guard has already validated this authority hold.
                    # A newer recovery may still have time remaining when
                    # the caller's original 3.1s wait ends. Follow that
                    # published deadline, subtracting transport age; never
                    # extend the authority or wait indefinitely on flapping.
                    if (data.get('ready') is False and data.get('recovering') is True
                            and type(remaining) in (int, float) and math.isfinite(remaining)
                            and 0. <= status_age <= 1.5 and status_age < remaining <= 3.001):
                        deadline = min(hard_deadline,
                            max(deadline, now+remaining-status_age))
                if now >= deadline:
                    return GoalValidationResult(False,'global_localization_recovering',
                        '3D 路线尚未启动：等待定位复核结束仍未就绪，未创建活动路线；'
                        '定位恢复后请重新预览并启动'),None
                time.sleep(min(.05, deadline-now))
                with self._lock:
                    if not owner and (token is None or self._preview_token is not None
                            or getattr(self,'_preview_request_generation',0) != generation):
                        return GoalValidationResult(False,'preview_canceled',
                            '等待定位期间路线已取消或预览被替换；未启动导航'),None
                result,ticket = self._prepare_3d_route_start_checked(
                    request,confirmed_token=token if not owner else None)
            return result,ticket
        if result.allowed or result.code != 'localization_stale':
            return result, ticket
        # The first check has consumed the manual token. Keep it local to this
        # explicit Start request: never restore it to the shared preview slot.
        confirmed_token = (ticket or {}).get('refresh_token')
        request_generation = (ticket or {}).get('request_generation')
        with self._lock:
            if self._mode != 'idle' or self._preview_busy:
                return GoalValidationResult(False, 'navigation_busy', '另有导航或预览正在进行'), None
            if confirmed_token is not None and (
                    self._preview_token is not None
                    or getattr(self, '_preview_request_generation', 0) != request_generation):
                return GoalValidationResult(False, 'preview_canceled',
                    '路线预览已取消或替换，请重新预览'), None
            route = self._database.get_route(int(request.route_id))
            if not route or not route.get('points'):
                return GoalValidationResult(False, 'route_missing', '路线不存在，请重新选择'), None
            try:
                points = self._route_preview_points(
                    route, start_point_id=int(request.start_point_id),
                    point_limit=int(request.point_limit), reverse=bool(request.reverse),
                    loop_count=int(request.loop_count), continuous=bool(request.continuous))
            except (TypeError, ValueError, OverflowError) as error:
                return GoalValidationResult(False, 'invalid_route', str(error)), None
            pose = self._route_pose(points[0])
            map_name, load_id = self._current_map_name, self._current_map_load_id
            owner = bool(str(request.owner_token).strip())
            self._preview_busy = True
        try:
            refreshed = self._refresh_3d_preview_pose(
                pose, map_name, load_id, allow_inspection=owner)
        finally:
            with self._lock:
                self._preview_busy = False
        if not refreshed.allowed:
            return refreshed, None
        if not owner:
            with self._lock:
                if (confirmed_token is None or self._preview_token is not None
                        or getattr(self, '_preview_request_generation', 0) != request_generation):
                    return GoalValidationResult(False, 'preview_canceled',
                        '刷新期间路线预览已取消或替换，请重新预览'), None
                # Recheck expiry, route digest, <=15 cm/15 deg pose change,
                # identities, every waypoint and every previewed segment.
                result, ticket = self._prepare_3d_route_start_checked(
                    request, confirmed_token=confirmed_token)
                self._log_goal_event('3d_route_start_refresh_checked', pose,
                    reason=f'{result.code}: {result.message or "定位刷新后路线复核通过"}')
                return result, ticket
        return self._prepare_3d_route_start_checked(request)

    def _prepare_3d_route_start_checked(self, request, *, confirmed_token=None) -> tuple:
        """Authorize a Xingji 3-D route before the state machine can move.

        Inspection continuations perform fresh non-motion planning here. The
        final state transition checks the returned ticket again, so changes
        during this potentially slow operation cannot slip through.
        """
        with self._lock:
            try:
                mode = normalize_navigation_mode(
                    str(request.navigation_mode or self._navigation_mode))
            except ValueError as error:
                return GoalValidationResult(
                    False, 'wrong_mode', str(error)), None
            if mode != TERRAIN_3D:
                return GoalValidationResult(True, 'unchanged_2d', ''), None
            config = (self._robot_body_status or {}).get('config') or {}
            if config.get('selected') == 'gangben_zsl1':
                # This feature is Xingji-only; preserve the existing ZSL-1
                # route path rather than guessing its autonomous capability.
                return GoalValidationResult(True, 'unchanged_gangben', ''), None
            if self._mode != 'idle' or self._preview_busy:
                return GoalValidationResult(
                    False, 'navigation_busy',
                    '导航或预览正在进行，不能启动另一条 3D 路线'), None
            route = self._database.get_route(int(request.route_id))
            if route is None or route['map_name'] != self._current_map_name:
                return GoalValidationResult(
                    False, 'route_missing', '路线不存在或不属于当前地图'), None
            if self._terrain_capability.planar_execution_blockers(
                    route['points'], allow_ground_elevation=True):
                return GoalValidationResult(
                    False, 'needs_calibration',
                    '路线包含未标定特殊地形点，不能按平面路线启动'), None
            try:
                digest = self._route_digest(route)
                params = {
                    'route_id': int(request.route_id),
                    'start_point_id': int(request.start_point_id),
                    'point_limit': int(request.point_limit),
                    'reverse': bool(request.reverse),
                    'loop_count': int(request.loop_count),
                    'continuous': bool(request.continuous),
                }
                points = self._route_preview_points(route, **{
                    key: value for key, value in params.items()
                    if key != 'route_id'})
            except (TypeError, ValueError, OverflowError) as error:
                return GoalValidationResult(
                    False, 'invalid_route', str(error)), None
            owner = str(request.owner_token).strip()
            token = None
            authorization = None
            if owner:
                authorization = self._inspection_route_authorization
                if (not authorization
                        or authorization['owner_token'] != owner
                        or authorization['route_id'] != params['route_id']
                        or authorization['route_digest'] != digest
                        or int(request.authorization_generation)
                        != authorization['generation']
                        or not self._inspection_task_active
                        or self._inspection_task_id != owner
                        or self._inspection_task_paused
                        or authorization.get('pause_latched', False)
                        or int(request.request_deadline_unix_ns) <= 0
                        or time.time_ns()
                        > int(request.request_deadline_unix_ns)):
                    return GoalValidationResult(
                        False, 'inspection_authorization_expired',
                        '3D 巡检授权已失效或任务不再活动；请重新预览并启动'), None
                if not self._preview_3d_identity_matches(
                        authorization['snapshot']):
                    return GoalValidationResult(
                        False, 'preview_stale',
                        self._preview_3d_change_reason(
                            authorization['snapshot'],include_pose=False)+'；巡检已阻止续跑'), None
            else:
                token = confirmed_token if confirmed_token is not None else self._preview_token
                if confirmed_token is None and token and str(request.preview_id) == token.get('id'):
                    # A matching confirmation is single-use even on failure.
                    self._preview_token = None
                if (not token or token.get('kind') != 'route'
                        or str(request.preview_id) != token.get('id')
                        or token['params'] != params
                        or token['route_digest'] != digest):
                    return GoalValidationResult(
                        False, 'preview_missing',
                        '3D 路线预览已失效或路线参数变化，请重新预览'), None
                if not self._preview_3d_identity_matches(token['snapshot']):
                    return GoalValidationResult(
                        False, 'preview_stale',
                        self._preview_3d_change_reason(
                            token['snapshot'],include_pose=False)+'；请重新预览'), None
            reference = (authorization or token)['snapshot']
            guard = self._preview_3d_guard(
                self._route_pose(points[0]),
                reference['map_name'], reference['map_load_id'],
                allow_inspection=bool(owner))
            if not guard.allowed:
                if (guard.code in ('localization_stale','global_localization_recovering')
                        and not owner and confirmed_token is None):
                    return guard, {
                        'refresh_token': token,
                        'request_generation': getattr(self, '_preview_request_generation', 0),
                    }
                return guard, None
            if not (self._preview_action_client.server_is_ready()
                    and self._action_client.server_is_ready()):
                return GoalValidationResult(
                    False, 'planner_unavailable',
                    'Nav2 规划器或导航动作未就绪'), None
            for point in points:
                check = self._goal_preflight(self._route_pose(point))
                if not check.allowed:
                    return check, None
            snapshot = self._preview_3d_snapshot()
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose = copy.deepcopy(self._latest_amcl_pose.pose.pose)
            if not owner:
                checked, snapshot = self._recheck_3d_start(
                    token['snapshot'],token['segments'],reuse_preview=True)
                if not checked.allowed:
                    return checked, None
                return GoalValidationResult(True, 'ready', ''), {
                    'route_digest': digest, 'snapshot': snapshot,
                    'params': params, 'owner_token': '',
                    'request_generation': getattr(self, '_preview_request_generation', 0),
                }
            self._preview_busy = True

        try:
            planned, segments = self._plan_3d_route_segments(
                points, start_pose)
            if not planned.allowed:
                return planned, None
            with self._lock:
                guard = self._preview_3d_guard(
                    self._route_pose(points[0]), reference['map_name'],
                    reference['map_load_id'], allow_inspection=True)
            if not guard.allowed and guard.code == 'localization_stale':
                guard = self._refresh_3d_preview_pose(
                    self._route_pose(points[0]), reference['map_name'],
                    reference['map_load_id'], allow_inspection=True)
            if not guard.allowed:
                return guard, None
            with self._lock:
                guard = self._preview_3d_guard(
                    self._route_pose(points[0]), reference['map_name'],
                    reference['map_load_id'], allow_inspection=True)
                if not guard.allowed:
                    return guard, None
                checked, snapshot = self._recheck_3d_start(snapshot,segments)
                if not checked.allowed:
                    return checked, None
                return GoalValidationResult(True, 'ready', ''), {
                    'route_digest': digest, 'snapshot': snapshot,
                    'params': params, 'owner_token': owner,
                    'deadline_ns': int(request.request_deadline_unix_ns),
                    # Never promote an old request to a new pause/resume
                    # generation that arrived during planning or refresh.
                    'auth_generation': int(request.authorization_generation),
                }
        finally:
            with self._lock:
                self._preview_busy = False

    def _verify_3d_route_start_ticket(self, ticket: dict, route: dict) -> bool:
        if (self._route_digest(route) != ticket['route_digest']
                or not self._preview_3d_snapshot_matches(
                    ticket['snapshot'])):
            return False
        owner = ticket['owner_token']
        if not owner:
            return (ticket.get('request_generation')
                    == getattr(self, '_preview_request_generation', 0))
        authorization = self._inspection_route_authorization
        return bool(
            authorization
            and authorization['owner_token'] == owner
            and authorization['route_digest'] == ticket['route_digest']
            and authorization['generation'] == ticket['auth_generation']
            and not authorization.get('pause_latched', False)
            and self._inspection_task_active
            and self._inspection_task_id == owner
            and not self._inspection_task_paused
            and time.time_ns() <= ticket['deadline_ns']
            and self._preview_3d_identity_matches(
                authorization['snapshot']))

    def _refresh_3d_preview_pose(
            self, pose, map_name: str, map_load_id: str, *,
            allow_inspection: bool = False) -> GoalValidationResult:
        """Bounded no-motion refresh; caller owns _preview_busy, not _lock.

        No pose is queued for execution. A successful refresh authorizes only
        another preflight; a Start request must still discard its old token.
        """
        with self._lock:
            if (self._latest_amcl_pose is None
                    or self._latest_amcl_stamp_ns <= 0):
                return GoalValidationResult(False, 'localization_missing',
                    '尚无有效初始定位，请先设置初始点并完成 AMCL 确认')
            baseline = self._latest_amcl_stamp_ns
            generation = getattr(self, '_preview_request_generation', 0)
            identity = self._preview_3d_snapshot()
            started_at = time.monotonic()
            deadline = started_at + self._goal_preflight_wait_timeout
            self._status_message = '3D定位数据过期，正在请求无运动刷新；不会启动导航'
            self._publish_goal_status('goal_refreshing_3d_localization')
            self._log_goal_event('3d_refresh_started', pose)
            self._send_zero_velocity()

        def finish(allowed, code, message):
            self._status_message = message
            self._publish_goal_status('goal_3d_localization_refreshed' if allowed
                                      else 'goal_3d_localization_refresh_failed')
            self._log_goal_event('3d_refresh_finished' if allowed
                                 else '3d_refresh_failed', pose, reason=message)
            return GoalValidationResult(allowed, code, message)

        future = None
        requests = 0
        next_request_at = started_at
        while True:
            with self._lock:
                now = time.monotonic()
                if getattr(self, '_preview_request_generation', 0) != generation:
                    return finish(False, 'preview_canceled', '3D定位刷新已取消；未发送目标')
                if not self._preview_3d_identity_matches(identity):
                    return finish(False, 'preview_stale',
                                  '刷新期间地图、地形、本体或配置已改变，请重新选点')
                guard = (self._preview_3d_guard(pose, map_name, map_load_id, allow_inspection=True)
                         if allow_inspection else self._preview_3d_guard(pose, map_name, map_load_id))
                if not guard.allowed and guard.code != 'localization_stale':
                    return finish(False, guard.code, guard.message)
                if future is not None and future.done():
                    try:
                        future.result()
                    except Exception as error:
                        return finish(False, 'localization_refresh_failed',
                                      f'AMCL无运动刷新失败：{error}')
                    future = None
                received_at = self._latest_amcl_received_at
                stamp_ns = self._latest_amcl_stamp_ns
                source_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                if (guard.allowed and stamp_ns > baseline
                        and received_at is not None and received_at >= started_at
                        and now - received_at <= self._goal_preflight_pose_max_age
                        and math.isfinite(source_age)
                        and -0.05 <= source_age <= self._goal_preflight_pose_max_age):
                    return finish(True, 'localization_refreshed',
                                  '已收到新的有效AMCL定位；仍需地图、地形及路径预检')
                if now >= deadline:
                    return finish(False, 'localization_refresh_timeout',
                                  '等待新定位超时；未启动导航，请检查AMCL、扫描和TF状态')
                if (future is None and requests < self._nomotion_update_max_requests
                        and now >= next_request_at
                        and self._nomotion_update_client.service_is_ready()):
                    try:
                        future = self._nomotion_update_client.call_async(EmptyService.Request())
                    except Exception as error:
                        return finish(False, 'localization_refresh_failed',
                                      f'无法请求AMCL无运动刷新：{error}')
                    requests += 1
                    next_request_at = now + self._nomotion_update_interval
            # AMCL, cancellation and identity callbacks must keep running.
            time.sleep(0.02)

    def _preview_3d_goal_callback(self, request, response):
        response.valid = False
        response.code = 'unavailable'
        response.message = '3D 路径预览不可用'
        response.preview_id = ''
        response.map_name = ''
        response.path = NavPath()
        with self._lock:
            # A new request supersedes the previous draft even if it fails.
            self._preview_token = None
            if self._preview_busy:
                response.code = 'preview_busy'
                response.message = '已有 3D 路径预览正在计算'
                return response
            pose = copy.deepcopy(request.goal)
            requested_map = str(request.map_name).strip()
            guard = self._preview_3d_guard(
                pose, requested_map, str(request.map_load_id).strip())
            response.map_name = self._current_map_name
            if not guard.allowed and guard.code != 'localization_stale':
                response.code, response.message = guard.code, guard.message
                return response
            if not self._preview_action_client.server_is_ready():
                response.code = 'planner_unavailable'
                response.message = 'Nav2 全局规划器尚未就绪'
                return response
            if not self._action_client.server_is_ready():
                response.code = 'navigation_unavailable'
                response.message = 'Nav2 导航动作尚未就绪'
                return response
            refresh_required = not guard.allowed
            request_generation = getattr(self, '_preview_request_generation', 0)
            request_identity = (self._preview_3d_snapshot()
                                if refresh_required and self._latest_amcl_pose is not None
                                else None)
            self._preview_busy = True

        handle = None
        try:
            if refresh_required:
                refreshed = self._refresh_3d_preview_pose(
                    pose, requested_map, str(request.map_load_id).strip())
                if not refreshed.allowed:
                    response.code, response.message = refreshed.code, refreshed.message
                    return response
            with self._lock:
                if getattr(self, '_preview_request_generation', 0) != request_generation:
                    response.code = 'preview_canceled'
                    response.message = '预览已取消，未生成可执行目标'
                    return response
                if request_identity is not None and not self._preview_3d_identity_matches(request_identity):
                    response.code = 'preview_stale'
                    response.message = '刷新期间地图、地形、本体或配置已改变，请重新选点'
                    return response
                guard = self._preview_3d_guard(
                    pose, requested_map, str(request.map_load_id).strip())
                if not guard.allowed:
                    response.code, response.message = guard.code, guard.message
                    return response
                snapshot = self._preview_3d_snapshot()
                start_pose = PoseStamped()
                start_pose.header.frame_id = 'map'
                start_pose.header.stamp = self.get_clock().now().to_msg()
                start_pose.pose = copy.deepcopy(self._latest_amcl_pose.pose.pose)
            deadline = time.monotonic() + self._preview_plan_timeout
            goal = ComputePathToPose.Goal()
            goal.goal = pose
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.start = start_pose
            goal.use_start = True
            goal.planner_id = 'GridBased'
            send_future = self._preview_action_client.send_goal_async(goal)
            if not self._await_preview_future(
                    send_future, deadline - time.monotonic()):
                response.code = 'planner_timeout'
                response.message = '等待 Nav2 规划器接收预览超时'
                return response
            handle = send_future.result()
            if handle is None or not handle.accepted:
                response.code = 'planner_rejected'
                response.message = 'Nav2 规划器拒绝了路径预览请求'
                return response
            result_future = handle.get_result_async()
            if not self._await_preview_future(
                    result_future, deadline - time.monotonic()):
                response.code = 'planner_timeout'
                response.message = 'Nav2 路径预览计算超时'
                handle.cancel_goal_async()
                return response
            result = result_future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                response.code = 'planner_failed'
                response.message = 'Nav2 未能规划到该目标的可通行路径'
                return response
            path = result.result.path
            with self._lock:
                if getattr(self, '_preview_request_generation', 0) != request_generation:
                    response.code = 'preview_canceled'
                    response.message = '预览已取消，未生成可执行目标'
                    return response
                guard = self._preview_3d_guard(
                    pose, requested_map, str(request.map_load_id).strip())
            if not guard.allowed and guard.code == 'localization_stale':
                refreshed = self._refresh_3d_preview_pose(
                    pose, requested_map, str(request.map_load_id).strip())
                if not refreshed.allowed:
                    response.code, response.message = refreshed.code, refreshed.message
                    return response
            with self._lock:
                if getattr(self, '_preview_request_generation', 0) != request_generation:
                    response.code = 'preview_canceled'
                    response.message = '预览已取消，未生成可执行目标'
                    return response
                guard = self._preview_3d_guard(
                    pose, requested_map, str(request.map_load_id).strip())
                if not guard.allowed:
                    response.code, response.message = guard.code, guard.message
                    return response
                if not self._preview_3d_snapshot_matches(snapshot):
                    response.code = 'preview_stale'
                    response.message = (
                        '预览期间地图、地形、本体或机器人位姿发生变化，请重新预览')
                    return response
                path_check = _validate_preview_plan(
                    path, self._traversability_grid, pose,
                    start_xy=(snapshot['start_x'], snapshot['start_y']),
                    footprint_radius_m=0.45)
                if not path_check.allowed:
                    response.code = path_check.code
                    response.message = path_check.message
                    return response
                preview_id = uuid.uuid4().hex
                self._preview_token = {
                    'kind': 'goal',
                    'id': preview_id,
                    'created_at': time.monotonic(),
                    'goal': copy.deepcopy(pose),
                    'snapshot': snapshot,
                    'path': copy.deepcopy(path),
                }
                response.valid = True
                response.code = 'ready'
                response.message = (
                    '3D 路径预览通过；仅表示当前规划与地形检查通过，'
                    '不代表楼梯等特殊地形已获执行许可')
                response.preview_id = preview_id
                response.path = path
                return response
        except Exception as error:
            response.code = 'planner_error'
            response.message = f'3D 路径预览失败：{error}'
            return response
        finally:
            with self._lock:
                self._preview_busy = False

    def _start_previewed_3d_goal_callback(self, request, response):
        refresh_context = None
        with self._lock:
            token = self._preview_token
            if (token and token.get('kind') == 'goal'
                    and str(request.preview_id) == token['id']
                    and not self._preview_busy):
                pose = copy.deepcopy(token['goal'])
                guard = self._preview_3d_guard(pose, token['snapshot']['map_name'],
                                             token['snapshot']['map_load_id'])
                if not guard.allowed and guard.code == 'localization_stale':
                    # A fresh pose may change the start of the planned route.
                    # Consume this token before refreshing; never auto-execute it.
                    self._preview_token = None
                    self._preview_busy = True
                    refresh_context = (pose, token['snapshot']['map_name'],
                                       token['snapshot']['map_load_id'])
                    refresh_token = token
                    refresh_generation = getattr(self,'_preview_request_generation',0)
        if refresh_context is not None:
            response.success = False
            try:
                refreshed = self._refresh_3d_preview_pose(*refresh_context)
                if not refreshed.allowed:
                    response.code,response.message = refreshed.code,refreshed.message
                    return response
                with self._lock:
                    if (self._preview_token is not None
                            or getattr(self,'_preview_request_generation',0) != refresh_generation):
                        response.code,response.message = 'preview_canceled','刷新期间预览已取消或替换，未启动'
                        return response
                    self._preview_token = refresh_token
            except Exception as error:
                response.code = 'localization_refresh_failed'
                response.message = f'3D定位刷新失败，未启动导航：{error}'
                return response
            finally:
                with self._lock:
                    self._preview_busy = False
        with self._lock:
            response.success = False
            response.code = 'preview_missing'
            response.message = '没有可启动的 3D 路径预览，请重新设置目标'
            # Tokens are single-use, including failed confirmation attempts.
            token = self._preview_token
            self._preview_token = None
            if (not token or token.get('kind') != 'goal'
                    or str(request.preview_id) != token['id']):
                return response
            if self._preview_busy:
                response.code = 'preview_stale'
                response.message = '预览正在重算，请等待本次检查完成'
                return response
            pose = copy.deepcopy(token['goal'])
            guard = self._preview_3d_guard(
                pose, token['snapshot']['map_name'],
                token['snapshot']['map_load_id'])
            if not guard.allowed:
                response.code, response.message = guard.code, guard.message
                return response
            path_check, _ = self._recheck_3d_start(
                token['snapshot'],[(pose,token['path'])],reuse_preview=True)
            if not path_check.allowed:
                response.code, response.message = (
                    path_check.code, path_check.message)
                return response
            if not self._action_client.server_is_ready():
                response.code = 'navigation_unavailable'
                response.message = 'Nav2 导航动作尚未就绪'
                return response
            # Explicit click consumes the server-side draft. The existing
            # single-goal send seam repeats its preflight before NavigateToPose.
            pose.header.stamp = self.get_clock().now().to_msg()
            try:
                self._single_goal_callback(pose)
            except Exception as error:
                if self._mode == 'single':
                    try:
                        self._request_stop('external_cancel')
                    except Exception:
                        self._send_zero_velocity()
                response.code = 'start_error'
                response.message = f'导航目标未能安全提交：{error}'
                return response
            if self._mode != 'single':
                response.code = 'start_rejected'
                response.message = '启动前实时预检未通过，目标没有发送'
                return response
            response.success = True
            response.code = 'accepted'
            response.message = '3D 导航目标已提交，Nav2 正在确认接收'
            return response

    def _clear_preflight_wait(self) -> None:
        self._preflight_wait_context = None
        self._preflight_wait_pose = None
        self._preflight_wait_code = ''
        self._preflight_wait_deadline = None
        self._preflight_wait_next_nomotion_at = None
        self._preflight_wait_nomotion_requests = 0

    def _clear_single_localization_refresh(self) -> None:
        self._single_refresh_map_name = ''
        self._single_refresh_baseline_stamp_ns = 0
        self._single_refresh_started_at = None
        self._single_refresh_deadline = None
        self._single_refresh_next_nomotion_at = None
        self._single_refresh_nomotion_requests = 0
        self._single_refresh_future = None

    def _fail_single_localization_refresh(self, reason: str) -> None:
        if not self._single_refresh_map_name:
            return
        self._clear_single_localization_refresh()
        self._pending_single_pose = None
        self._status_message = reason
        self._publish_goal_status('goal_localization_refresh_failed')
        self._publish_all_status()

    def _begin_single_localization_refresh(self) -> None:
        """Refresh AMCL only; never retain the rejected map-frame goal."""
        self._clear_single_localization_refresh()
        self._pending_single_pose = None
        if not self._current_map_name:
            self._status_message = '当前没有已加载地图，请加载地图后重新点击目标'
            self._publish_goal_status('goal_localization_refresh_failed')
            self._publish_all_status()
            return
        now = time.monotonic()
        self._single_refresh_map_name = self._current_map_name
        self._single_refresh_baseline_stamp_ns = self._latest_amcl_stamp_ns
        self._single_refresh_started_at = now
        self._single_refresh_deadline = now + self._goal_preflight_wait_timeout
        self._single_refresh_next_nomotion_at = now
        self._single_refresh_nomotion_requests = 0
        self._single_refresh_future = None
        self._status_message = (
            '定位过期，已丢弃本次目标；正在请求静止定位更新。'
            '刷新完成后请重新点击目标。')
        self._publish_goal_status('goal_refreshing_localization')
        self._send_zero_velocity()
        self._drive_single_localization_refresh(now)
        self._publish_all_status()

    def _drive_single_localization_refresh(
        self, now: Optional[float] = None
    ) -> None:
        if not self._single_refresh_map_name:
            return
        if self._current_map_name != self._single_refresh_map_name:
            self._fail_single_localization_refresh(
                '地图已变化，定位刷新已取消；请重新定位并点击目标')
            return
        now = time.monotonic() if now is None else float(now)
        if self._single_refresh_deadline is not None and (
            now >= self._single_refresh_deadline
        ):
            self._fail_single_localization_refresh(
                '旧导航取消未确认，且等待新定位采样已超时；'
                '请先确认机器人已停止，再重新定位并点击目标'
                if self._mode != 'idle' else
                '等待新定位采样超时；请确认 AMCL 与地图状态后重新点击目标')
            return
        # A stale replacement must first wait for the old Nav2 goal to end.
        # In particular, do not announce success while its cancellation is
        # still pending and do not send the new rejected pose later.
        if self._mode != 'idle':
            return
        future = self._single_refresh_future
        if future is not None and future.done():
            self._single_refresh_future = None
            try:
                future.result()
            except Exception as error:
                self._fail_single_localization_refresh(
                    f'静止定位更新服务失败：{error}；请重新确认定位')
                return
        if (
            self._latest_amcl_pose is not None
            and self._latest_amcl_received_at is not None
            and self._single_refresh_started_at is not None
            and self._latest_amcl_received_at >= self._single_refresh_started_at
            and self._latest_amcl_stamp_ns > 0
            and self._latest_amcl_stamp_ns
            > self._single_refresh_baseline_stamp_ns
        ):
            self._clear_single_localization_refresh()
            self._status_message = (
                '定位已刷新，本次目标未发送；请重新点击目标并再次预检')
            self._publish_goal_status('goal_localization_refreshed')
            self._publish_all_status()
            return
        if (
            self._single_refresh_nomotion_requests
            < self._nomotion_update_max_requests
            and (
                self._single_refresh_next_nomotion_at is None
                or now >= self._single_refresh_next_nomotion_at
            )
        ):
            self._single_refresh_next_nomotion_at = (
                now + self._nomotion_update_interval)
            if self._nomotion_update_client.service_is_ready():
                try:
                    self._single_refresh_future = (
                        self._nomotion_update_client.call_async(
                            EmptyService.Request()))
                except Exception as error:
                    self._fail_single_localization_refresh(
                        f'无法请求静止定位更新：{error}')
                    return
                self._single_refresh_nomotion_requests += 1

    def _dispatch_web_single_pose(self, pose: PoseStamped) -> None:
        """Recheck queued/replacement web goals at the actual send seam."""
        if self._mode != 'single' or self._owner_token:
            return
        result = self._goal_preflight(pose)
        if not result.allowed:
            self._pending_single_pose = None
            stop_requested, _ = self._request_stop('external_cancel')
            if not stop_requested:
                self._status_message = (
                    '旧导航取消请求失败，可能仍在运动；'
                    '本次目标已丢弃，请确认机器人安全停止')
                self._publish_goal_status(
                    'goal_localization_refresh_failed'
                    if result.code == 'localization_stale'
                    else 'goal_rejected_navigation_stopping')
                self._publish_all_status()
                return
            if result.code == 'localization_stale':
                self._begin_single_localization_refresh()
            else:
                self._status_message = result.message
                self._publish_goal_status(self._goal_rejection_status(result))
                self._publish_all_status()
            return
        self._send_pose(pose, 'single')

    def _begin_preflight_wait(
        self,
        pose: PoseStamped,
        context: str,
        result: GoalValidationResult,
    ) -> None:
        now = time.monotonic()
        first_wait = self._preflight_wait_context is None
        self._preflight_wait_context = str(context)
        self._preflight_wait_pose = copy.deepcopy(pose)
        if first_wait:
            self._preflight_wait_deadline = (
                now + self._goal_preflight_wait_timeout)
            self._preflight_wait_next_nomotion_at = now
            self._preflight_wait_nomotion_requests = 0
        changed_code = result.code != self._preflight_wait_code
        self._preflight_wait_code = result.code
        self._state = 'waiting_preflight'
        self._status_message = result.message
        if self._mode == 'route' and (first_wait or changed_code):
            point = self._current_point()
            point_name = point['name'] if point else '当前目标点'
            self._publish_route_log(
                f'目标点“{point_name}”暂未发送：{result.message}；'
                '目标点已保留，正在自动刷新后重试。')
        self._drive_preflight_wait(now)

    def _drive_preflight_wait(self, now: Optional[float] = None) -> None:
        if (
            self._preflight_wait_context != 'route'
            or self._mode != 'route'
            or self._state != 'waiting_preflight'
        ):
            return
        now = time.monotonic() if now is None else float(now)
        point = self._current_point()
        if point is None:
            self._clear_preflight_wait()
            self._finish_task('stopped', 'Current route point is unavailable')
            return
        pose = self._route_pose(point)
        result = self._goal_preflight(pose)
        if result.allowed:
            waited_code = self._preflight_wait_code
            self._clear_preflight_wait()
            self._publish_route_log(
                f'目标点“{point["name"]}”的'
                f'{"实时定位" if waited_code in ("localization_stale", "global_localization_recovering") else "地形代价图"}'
                '已刷新，正在继续发送当前目标。')
            self._state = 'sending'
            self._status_message = (
                f'Navigating to point {self._point_index + 1}: {point["name"]}')
            self._send_pose(pose, 'route')
            return
        if result.code not in self.TRANSIENT_PREFLIGHT_CODES:
            self._clear_preflight_wait()
            detail = f'{result.message}（{result.code}）'
            body_config = (self._robot_body_status or {}).get('config') or {}
            if (self._effective_navigation_mode() == TERRAIN_3D
                    and body_config.get('selected') == 'xingji'):
                self._state = 'paused_preflight'
                self._status_message = (
                    f'3D 当前目标点安全检查失败：{detail}')
                self._send_zero_velocity()
                self._publish_route_log(
                    f'目标点“{point["name"]}”刷新后仍不可通行，'
                    f'路线已安全暂停，当前点未跳过：{detail}')
                self._publish_all_status()
                return
            self._publish_route_log(
                f'目标点“{point["name"]}”未发送：{detail}')
            self._record_point_event('unreachable', detail)
            self._point_failure_retries = 0
            self._advance_route()
            return
        if result.code != self._preflight_wait_code:
            self._preflight_wait_code = result.code
            self._status_message = result.message
            self._publish_route_log(
                f'目标点“{point["name"]}”仍在等待：{result.message}')
        if (
            self._preflight_wait_deadline is not None
            and now >= self._preflight_wait_deadline
        ):
            detail = result.message
            self._clear_preflight_wait()
            self._state = 'paused_preflight'
            self._status_message = (
                f'Preflight refresh timed out: {detail}')
            self._send_zero_velocity()
            self._publish_route_log(
                f'目标点“{point["name"]}”等待刷新超时，路径已安全暂停；'
                '当前点未跳过。请确认定位/地形状态后继续任务。')
            return
        if (
            result.code == 'localization_stale'
            and self._preflight_wait_nomotion_requests
            < self._nomotion_update_max_requests
            and (
                self._preflight_wait_next_nomotion_at is None
                or now >= self._preflight_wait_next_nomotion_at
            )
        ):
            self._preflight_wait_next_nomotion_at = (
                now + self._nomotion_update_interval)
            if self._nomotion_update_client.service_is_ready():
                self._preflight_wait_nomotion_requests += 1
                self._nomotion_update_client.call_async(
                    EmptyService.Request())

    @staticmethod
    def _goal_rejection_status(result: GoalValidationResult) -> str:
        statuses = {
            'terrain_not_ready': 'goal_rejected_terrain_not_ready',
            'localization_stale': 'goal_rejected_localization_stale',
            'robot_outside_map': 'goal_rejected_robot_not_traversable',
            'robot_not_traversable': 'goal_rejected_robot_not_traversable',
            'goal_outside_map': 'goal_rejected_goal_outside_map',
            'goal_disconnected': 'goal_rejected_goal_disconnected',
            'goal_blocked': 'goal_rejected_goal_blocked',
            'invalid_coordinates': 'goal_rejected_invalid_coordinates',
        }
        return statuses.get(result.code, 'goal_rejected')

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
        self._publish_all_status()

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
        body = (getattr(self, '_robot_body_status', None) or {}).get('status') or {}
        if not isinstance(body, dict):
            body = {}
        if (getattr(self, '_lio_recovery_hard_fault', False)
                or body.get('emergency_latched') is True):
            # A hard fault or manual emergency is not authorization to reset
            # the body when localization happens to recover.
            self._lio_recovering = False
            self._auto_recovery_stage = 'manual_reset_required'
            self._state = 'recovery_failed'
            self._status_message = (
                '定位已恢复，但硬故障或人工急停需人工复位、确认站立后重新启动任务')
            self._send_zero_velocity()
            self._publish_goal_status('goal_recovery_requires_manual_reset')
            self._publish_route_log(self._status_message)
            self._publish_all_status()
            return
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
            if (active_navigation
                    and self._auto_recovery_stage == 'manual_reset_required'):
                # A later soft recovery heartbeat must not replace the
                # operator's obligation to stop this task and reset safely.
                self._publish_all_status()
                return
            recovering = (
                state in ('recovering', 'resetting')
                or (state == 'warning' and temporary_stop)
                or state == 'fault'
            )
            if recovering:
                self._preview_token = None
                first_notice = not self._lio_recovering
                self._lio_recovering = True
                if first_notice:
                    self._lio_recovery_hard_fault = state == 'fault'
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
                if state == 'fault':
                    self._lio_recovery_hard_fault = True
                if active_navigation and self._state not in (
                    'stopping', 'paused_navigation', 'paused_dwell'
                ):
                    self._clear_preflight_wait()
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

            body = (getattr(self, '_robot_body_status', None) or {}).get('status') or {}
            if (getattr(self, '_lio_recovery_hard_fault', False)
                    or (isinstance(body, dict)
                        and body.get('emergency_latched') is True)):
                # Fast healthy-scan recovery has the same manual-reset
                # boundary as a full estimator reset and relocalization.
                self._resume_after_auto_recovery()
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
            previous_load_id = self._current_map_load_id
            self._current_map_name = (
                str(message.map_name).strip()
                if message.success and message.map_name else '')
            self._current_map_load_id = (
                str(getattr(message, 'load_id', '')).strip()
                if self._current_map_name else '')
            # /current_map_info is also a periodic heartbeat.  An explicit
            # successful LoadMap result, including a reload of the same name,
            # starts a new map/AMCL generation; heartbeat repeats do not.
            confirmed_reload = (
                bool(previous)
                and self._current_map_name == previous
                and str(getattr(message, 'message', '')).startswith((
                    'Successfully loaded and confirmed map:',
                    'Successfully loaded map:')))
            map_invalidated = (
                self._current_map_name != previous
                or self._current_map_load_id != previous_load_id
                or confirmed_reload)
            if map_invalidated:
                matching_early_grid = (
                    bool(self._current_map_load_id)
                    and self._terrain_costmap_ready_map_name
                    == self._current_map_name
                    and self._terrain_costmap_ready_load_id
                    == self._current_map_load_id
                    and self._terrain_costmap_ready_stamp_ns > 0
                    and self._terrain_costmap_ready_stamp_ns
                    == self._pending_traversability_stamp_ns
                    and self._pending_traversability_grid is not None)
                early_grid = (
                    self._pending_traversability_grid
                    if matching_early_grid else None)
                early_stamp = (
                    self._pending_traversability_stamp_ns
                    if matching_early_grid else 0)
                self._map_generation = getattr(
                    self, '_map_generation', 0) + 1
                self._preview_token = None
                self._inspection_route_authorization = None
                # A map-frame pose belongs to the map generation that produced
                # it. Never reuse a fresh-looking AMCL sample across a switch.
                self._latest_amcl_pose = None
                self._latest_amcl_received_at = None
                self._latest_amcl_stamp_ns = 0
                self._clear_preflight_wait()
                self._traversability_grid = None
                self._traversability_map_name = ''
                # Invalidate this manager's cached terrain on LoadMap.  This
                # is not end-to-end terrain provenance: independently latched
                # DDS topics still need a generation identifier upstream.
                self._pending_traversability_grid = None
                self._pending_traversability_stamp_ns = 0
                self._terrain_costmap_ready_map_name = ''
                self._terrain_costmap_ready_load_id = ''
                self._terrain_costmap_ready_stamp_ns = 0
                if matching_early_grid:
                    self._pending_traversability_grid = early_grid
                    self._pending_traversability_stamp_ns = early_stamp
                    self._terrain_costmap_ready_map_name = (
                        self._current_map_name)
                    self._terrain_costmap_ready_load_id = (
                        self._current_map_load_id)
                    self._terrain_costmap_ready_stamp_ns = early_stamp
                    self._traversability_grid = early_grid
                    self._traversability_map_name = self._current_map_name
                self._pending_single_pose = None
                self._recovery_pose = None
                self._retry_current_after_recovery = False
                if self._single_refresh_map_name:
                    self._fail_single_localization_refresh(
                        '地图已切换或卸载，定位刷新已取消；请重新定位')
            if (
                previous
                and map_invalidated
                and self._mode in ('route', 'single')
            ):
                if self._mode == 'route':
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
        try:
            prepared, ticket = self._prepare_3d_route_start(request)
        except Exception as error:
            response.success = False
            response.message = f'3D 路线启动前安全校验失败：{error}'
            return response
        if not prepared.allowed:
            response.success = False
            response.message = prepared.message
            self._log_goal_event('3d_route_start_rejected',
                reason=f'{prepared.code}: {prepared.message}')
            return response
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
            requested_value = str(request.navigation_mode or '').strip()
            try:
                requested_mode = (
                    normalize_navigation_mode(requested_value)
                    if requested_value else self._navigation_mode)
            except ValueError as error:
                response.success = False
                response.message = str(error)
                return response
            if requested_value and requested_mode != self._navigation_mode:
                response.success = False
                response.message = (
                    '巡检任务冻结的导航模式与当前系统设置不一致；'
                    '请停止任务后重新选择模式')
                return response
            # Check before claiming route ownership. Later internal route goals
            # deliberately skip idle-only configuration transactions.
            timing_check = getattr(self, '_controller_timing_preflight', None)
            timing_result = (timing_check(navigation_mode=requested_mode)
                             if timing_check is not None else None)
            if timing_result is not None:
                response.success = False
                response.message = timing_result.message
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
            blockers = self._terrain_capability.planar_execution_blockers(
                route['points'], allow_ground_elevation=(
                    requested_mode == TERRAIN_3D
                    and (self._robot_body_status or {}).get('config', {}).get(
                        'selected') == 'xingji'))
            if blockers:
                names = '、'.join(blockers[:3])
                response.success = False
                response.message = (
                    '路线包含特殊地形点，当前 Nav2 平面执行器禁止直接进入：'
                    f'{names}。请先接入厂商步态、姿态和足端反馈适配器。')
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
            if not self._inflation_ready_for_navigation():
                response.success = False
                response.message = (
                    '障碍膨胀半径尚未同步到全局和局部代价地图，请稍后重试')
                return response
            if ticket is not None and not self._verify_3d_route_start_ticket(
                    ticket, route):
                response.success = False
                response.message = (
                    '3D 路线启动前地图、地形、定位、本体、路线或巡检任务状态'
                    '已变化；未发送导航目标，请重新预览')
                return response
            if ticket is not None and ticket['owner_token']:
                self._inspection_route_authorization['state'] = 'active'

            self._active_navigation_mode = requested_mode
            self._mode = 'route'
            self._state = 'starting'
            self._status_message = 'Starting route'
            route = copy.deepcopy(route)
            self._reverse = bool(request.reverse)
            if self._reverse:
                route['points'] = list(reversed(route['points']))
            self._route = route
            start_point_id = int(request.start_point_id)
            self._point_index = 0
            if start_point_id > 0:
                matching_indices = [
                    index for index, point in enumerate(route['points'])
                    if int(point.get('id', 0)) == start_point_id
                ]
                if not matching_indices:
                    response.success = False
                    response.message = 'Start point does not belong to route'
                    self._mode = 'idle'
                    self._active_navigation_mode = ''
                    self._route = None
                    return response
                self._point_index = matching_indices[0]
            self._current_loop = 1
            self._loop_count = max(1, loop_count)
            self._continuous = bool(request.continuous)
            self._hold_at_each_point = bool(request.hold_at_each_point)
            self._owner_token = str(request.owner_token).strip()
            self._point_limit = max(0, int(request.point_limit))
            self._points_executed = 0
            self._dwell_deadline = None
            self._paused_dwell_remaining = 0.0
            self._cancel_reason = None
            self._pending_single_pose = None
            self._clear_single_localization_refresh()
            self._lio_recovering = False
            self._retry_current_after_recovery = False
            self._point_failure_retries = 0
            self._settled_since = None
            self._recovery_pose = None
            self._clear_preflight_wait()
            self._last_point_id = 0
            self._last_point_index = -1
            self._last_point_name = ''
            self._last_point_result = ''
            self._last_point_detail = ''
            loop_text = (
                '持续循环'
                if self._continuous else f'循环 {self._loop_count} 次')
            self._route_log_publisher.publish(
                String(data='__ROUTE_LOG_CLEAR__'))
            self._publish_route_log(
                f'开始路径“{route["name"]}”，{loop_text}'
                f'{"，反向执行" if self._reverse else ""}'
                f'{"，逐点等待巡检释放" if self._hold_at_each_point else ""}')
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
            request_owner = str(request.owner_token).strip()
            body_config = (self._robot_body_status or {}).get('config') or {}
            if (request_owner
                    and self._effective_navigation_mode() == TERRAIN_3D
                    and body_config.get('selected') == 'xingji'
                    and request_owner != self._owner_token):
                response.success = False
                response.message = '3D 路线属于另一任务，拒绝跨任务控制'
                return response
            if (self._owner_token
                    and self._effective_navigation_mode() == TERRAIN_3D
                    and body_config.get('selected') == 'xingji'
                    and command in ('resume', 'release', 'skip')):
                authorization = self._inspection_route_authorization
                if (not authorization
                        or authorization['owner_token'] != self._owner_token
                        or authorization.get('pause_latched', False)
                        or int(request.authorization_generation)
                        != authorization['generation']
                        or not self._inspection_task_active
                        or self._inspection_task_id != self._owner_token
                        or self._inspection_task_paused):
                    response.success = False
                    response.message = (
                        '3D 巡检路线授权已暂停或撤销，旧控制请求无效；'
                        '请由当前巡检任务确认恢复')
                    return response
            if self._owner_token and request_owner != self._owner_token:
                response.success = False
                response.message = 'Route is owned by another task'
                return response

            handlers = {
                'pause': self._pause_route,
                'resume': self._resume_route,
                'skip': self._skip_route_point,
                'stop': lambda: self._request_stop('user_stop'),
                'release': self._release_route_point,
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

    def _confirmed_gangben_3d_body(self) -> bool:
        """The legacy 3D topic is permitted only for a live, selected Gangben."""
        status = self._robot_body_status
        if (
            not isinstance(status, dict)
            or self._robot_body_received_at is None
            or time.monotonic() - self._robot_body_received_at > 2.0
            or self._robot_body_status_samples < 2
        ):
            return False
        config = status.get('config')
        worker = status.get('status')
        profiles = config.get('profiles') if isinstance(config, dict) else None
        profile = (
            profiles.get('gangben_zsl1')
            if isinstance(profiles, dict) else None
        )
        return bool(
            isinstance(worker, dict)
            and isinstance(profile, dict)
            and config.get('selected') == 'gangben_zsl1'
            and status.get('active_body') == 'gangben_zsl1'
            and type(status.get('generation')) is int
            and status['generation'] >= 1
            and status.get('connected') is True
            and not status.get('switching')
            and not status.get('error')
            and worker.get('stale') is not True
            and not worker.get('switching')
            and not worker.get('emergency_latched')
            and worker.get('hardware_error') is not True
            and not worker.get('error')
            and profile.get('navigation_verified') is True
        )

    def _legacy_web_goal_callback(self, pose: PoseStamped) -> None:
        """Preserve 2D and verified Gangben goals; Xingji 3D needs preview."""
        with self._lock:
            mode = self._effective_navigation_mode()
            if mode != MAP_2D and not (
                    mode == TERRAIN_3D
                    and self._confirmed_gangben_3d_body()):
                self._status_message = (
                    '3D 星玑目标必须先预览路径并确认启动；'
                    '钢镚本体状态未确认时旧目标话题也不会触发运动')
                self._publish_goal_status(
                    'goal_rejected_3d_preview_required')
                self._publish_all_status()
                return
            self._single_goal_callback(pose)

    def _single_goal_callback(self, pose: PoseStamped) -> None:
        with self._lock:
            self._log_goal_event('received', pose)
            body_status = (getattr(self, '_robot_body_status', None) or {}).get('status')
            if isinstance(body_status, dict) and body_status.get('emergency_latched') is True:
                self._status_message = '停车仍锁定，请先停止任务并人工确认复位；目标不会自动解除停车锁'
                self._publish_goal_status('goal_rejected_emergency_latched')
                self._publish_all_status()
                return
            if self._campus_mission_active:
                self._status_message = '园区任务正在执行，未发送本次目标'
                self._publish_goal_status('goal_rejected_campus_mission_active')
                return
            if self._lio_recovering:
                self._status_message = 'LIO 尚在恢复，未发送本次目标'
                self._publish_goal_status('waiting_for_lio_recovery')
                return
            if self._mode == 'route' or (
                    self._mode == 'single' and self._owner_token):
                self._publish_goal_status('goal_rejected_route_active')
                self._publish_route_log(
                    '路径执行期间拒绝了新的单目标导航请求')
                return
            if not pose.header.frame_id:
                pose.header.frame_id = 'map'
            pose = copy.deepcopy(pose)
            if self._single_refresh_map_name:
                # An extra click during an in-flight refresh cannot extend
                # its timeout or become an implicit confirmation.  The user
                # must click again after seeing the completed refresh.
                self._pending_single_pose = None
                self._drive_single_localization_refresh()
                return
            if self._mode == 'single' and (
                self._state == 'stopping'
                or self._cancel_reason in (
                    'external_cancel', 'map_changed', 'user_stop',
                    'inspection_cancel')
            ):
                self._pending_single_pose = None
                self._status_message = (
                    '旧导航取消尚未确认，禁止接受新目标；'
                    '请先确认机器人停止')
                self._publish_goal_status('goal_rejected_navigation_stopping')
                self._publish_all_status()
                return
            # An explicit new click supersedes a previous refresh notice, but
            # it still must pass the full preflight again before any send.
            self._clear_single_localization_refresh()
            starting_execution = self._mode == 'idle'
            if starting_execution:
                self._active_navigation_mode = self._navigation_mode
            preflight = self._goal_preflight(pose)
            if not preflight.allowed:
                if starting_execution:
                    self._active_navigation_mode = ''
                if preflight.code == 'localization_stale':
                    self._pending_single_pose = None
                    if self._mode == 'single':
                        stop_requested, _ = self._request_stop(
                            'external_cancel')
                        if not stop_requested:
                            self._status_message = (
                                '旧导航取消请求失败，可能仍在运动；'
                                '本次目标已丢弃，请确认机器人安全停止')
                            self._publish_goal_status(
                                'goal_localization_refresh_failed')
                            self._publish_all_status()
                            return
                    self._begin_single_localization_refresh()
                    return
                self._status_message = preflight.message
                self._publish_goal_status(
                    self._goal_rejection_status(preflight))
                self.get_logger().warning(
                    f'Navigation goal rejected before Nav2: '
                    f'{preflight.code}: {preflight.message}')
                self._publish_all_status()
                return

            if self._mode == 'single':
                self._pending_single_pose = pose
                if self._active_goal_handle is not None or self._sending:
                    self._cancel_reason = 'replace_single'
                    self._state = 'replacing'
                    self._cancel_active_goal()
                else:
                    self._dispatch_web_single_pose(pose)
                return

            self._mode = 'single'
            self._owner_token = ''
            self._state = 'waiting_nav2'
            self._status_message = 'Single navigation goal queued'
            self._pending_single_pose = pose
            self._publish_goal_status('goal_queued')
            if self._action_client.server_is_ready():
                self._pending_single_pose = None
                self._send_pose(pose, 'single')
            self._publish_all_status()

    def _managed_goal_callback(self, request, response):
        with self._lock:
            owner = str(request.owner_token).strip()
            if not owner:
                response.success = False
                response.message = 'Managed navigation requires an owner token'
                return response
            if self._campus_mission_active and not owner.startswith('campus:'):
                response.success = False
                response.message = 'A campus mission owns navigation control'
                return response
            if self._mode != 'idle':
                response.success = False
                response.message = 'Another navigation task is already active'
                return response
            if self._lio_recovering:
                response.success = False
                response.message = 'LIO is recovering'
                return response
            if not self._action_client.server_is_ready():
                response.success = False
                response.message = 'Nav2 is not ready'
                return response
            if not self._inflation_ready_for_navigation():
                response.success = False
                response.message = (
                    '障碍膨胀半径尚未同步到全局和局部代价地图')
                return response
            pose = copy.deepcopy(request.goal)
            pose.header.frame_id = pose.header.frame_id or 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            self._active_navigation_mode = self._navigation_mode
            preflight = self._goal_preflight(pose)
            if not preflight.allowed:
                self._active_navigation_mode = ''
                response.success = False
                response.message = preflight.message
                self._publish_goal_status(
                    self._goal_rejection_status(preflight))
                return response
            self._mode = 'single'
            self._owner_token = owner
            self._managed_goal_requires_yaw = bool(request.require_yaw)
            self._state = 'waiting_nav2'
            self._status_message = f'Managed goal queued for {owner}'
            self._pending_single_pose = None
            self._send_pose(pose, 'single')
            response.success = True
            response.message = 'Managed navigation goal accepted'
            self._publish_all_status()
        return response

    def _campus_mission_active_callback(self, message: Bool) -> None:
        with self._lock:
            self._campus_mission_active = bool(message.data)
            self._publish_navigation_mode()
            self._publish_inflation_radius()

    def _inspection_status_callback(self, message: String) -> None:
        with self._lock:
            try:
                payload = json.loads(str(message.data))
                active = bool(payload.get('active', False))
                task_id = str(payload.get('task_id') or '')
                paused = bool(payload.get('paused', False))
            except (AttributeError, TypeError, json.JSONDecodeError):
                return
            authorization = self._inspection_route_authorization
            if authorization and (
                    (active and task_id != authorization['owner_token'])
                    or (not active and authorization['state'] == 'active')):
                self._inspection_route_authorization = None
            if (active == self._inspection_task_active
                    and task_id == self._inspection_task_id
                    and paused == self._inspection_task_paused):
                return
            self._inspection_task_active = active
            self._inspection_task_id = task_id
            self._inspection_task_paused = paused
            self._publish_navigation_mode()
            self._publish_inflation_radius()

    def _cancel_navigation_callback(self, _message: Empty) -> None:
        with self._lock:
            self._preview_request_generation = getattr(
                self, '_preview_request_generation', 0) + 1
            self._preview_token = None
            if self._single_refresh_map_name:
                self._clear_single_localization_refresh()
                self._pending_single_pose = None
            if self._mode == 'idle':
                self._publish_goal_status('goal_canceled')
                self._status_message = '定位刷新已取消；目标未发送'
                self._publish_all_status()
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
        # Keep z in the route/domain model, but the current Nav2 adapter is a
        # floor-local 2-D executor. Cross-floor/special transitions are gated
        # before dispatch and must never be disguised as a planar goal.
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
        if not self._inflation_ready_for_navigation():
            self._state = 'waiting_nav2'
            self._status_message = 'Waiting for costmap inflation synchronization'
            return
        pose = self._route_pose(point)
        preflight = self._goal_preflight(pose)
        if not preflight.allowed:
            if preflight.code in self.TRANSIENT_PREFLIGHT_CODES:
                self._begin_preflight_wait(pose, 'route', preflight)
                return
            detail = f'{preflight.message}（{preflight.code}）'
            body_config = (self._robot_body_status or {}).get('config') or {}
            if (self._effective_navigation_mode() == TERRAIN_3D
                    and body_config.get('selected') == 'xingji'):
                self._state = 'paused_preflight'
                self._status_message = (
                    f'3D 当前目标点安全检查失败：{detail}')
                self._send_zero_velocity()
                self._publish_route_log(
                    f'目标点“{point["name"]}”未发送，路线已安全暂停：{detail}')
                self._publish_all_status()
                return
            self._publish_route_log(
                f'目标点“{point["name"]}”未发送：{detail}')
            self._record_point_event('unreachable', detail)
            self._point_failure_retries = 0
            self._advance_route()
            return
        self._state = 'sending'
        self._status_message = (
            f'Navigating to point {self._point_index + 1}: {point["name"]}')
        self._publish_route_log(
            f'第 {self._current_loop} 轮：前往目标点 '
            f'{self._point_index + 1}/{len(self._route["points"])}'
            f'“{point["name"]}”')
        self._send_pose(pose, 'route')

    def _send_pose(self, pose: PoseStamped, context: str) -> None:
        if self._sending or self._active_goal_handle is not None:
            return
        if not self._action_client.server_is_ready():
            self._state = 'waiting_nav2'
            if context == 'single':
                self._pending_single_pose = pose
            return
        if not self._inflation_ready_for_navigation():
            self._state = 'waiting_nav2'
            self._status_message = 'Waiting for costmap inflation synchronization'
            if context == 'single':
                self._pending_single_pose = pose
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._log_goal_event('nav2_send', pose, reason=f'context={context}')
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
            if self._cancel_reason is not None:
                self._state = 'stopping'
                self._status_message = (
                    'Old navigation goal accepted late; cancellation pending')
            else:
                self._state = (
                    'recovering_lio' if self._lio_recovering else 'navigating')
                self._status_message = 'Navigation goal accepted'
                if self._goal_context == 'single':
                    self._publish_goal_status('navigating')
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

            if xy_error > self._settled_xy_tolerance or (
                (self._settled_requires_yaw or self._managed_goal_requires_yaw)
                and yaw_error > self._settled_yaw_tolerance
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
            if self._settled_requires_yaw or self._managed_goal_requires_yaw:
                self._status_message = (
                    f'Goal stable within {xy_error:.2f} m and '
                    f'{math.degrees(yaw_error):.1f} deg')
            else:
                self._status_message = (
                    f'Goal position stable within {xy_error:.2f} m')
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

    def _cancel_active_goal(self) -> bool:
        if self._active_goal_handle is None:
            return False
        if self._cancel_in_progress:
            return True
        self._cancel_in_progress = True
        try:
            future = self._active_goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_response_callback)
        except Exception as error:
            self._cancel_in_progress = False
            self._status_message = f'Nav2取消请求失败：{error}'
            self.get_logger().error(self._status_message)
            self._send_zero_velocity()
            self._publish_all_status()
            return False
        return True

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

    def _retain_goal_for_global_recovery(self, context, pose):
        failure = self._global_localization_check()
        if (pose is None or not failure
                or failure.code != 'global_localization_recovering'):
            return False
        self._send_zero_velocity()
        if context == 'route':
            self._begin_preflight_wait(pose, 'route', failure)
        else:
            self._pending_single_pose = pose
            self._state = 'waiting_nav2'
            self._status_message = failure.message
        self._publish_route_log('控制器在定位复核期间结束目标；当前目标已保留，复核通过后重新预检，不跳过当前点。')
        self._publish_all_status()
        return True

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

        if (not succeeded and cancel_reason is None and context in ('route', 'single')
                and getattr(self, '_global_match_hold', False)
                and self._retain_goal_for_global_recovery(context, recovery_pose)):
            return

        if context == 'single':
            if cancel_reason == 'replace_single' and self._pending_single_pose:
                pose = self._pending_single_pose
                self._pending_single_pose = None
                self._dispatch_web_single_pose(pose)
                return
            if cancel_reason in (
                    'user_stop', 'external_cancel', 'map_changed',
                    'inspection_cancel', 'global_localization_untrusted'):
                # A stale replacement intentionally cancels the old goal.
                # That cancellation is not a user cancellation of the AMCL
                # refresh, so it must not overwrite the refresh protocol.
                if not self._single_refresh_map_name:
                    self._publish_goal_status('goal_canceled')
                self._record_managed_goal_event('canceled', detail)
                self._finish_task('idle', detail)
                if self._single_refresh_map_name:
                    self._drive_single_localization_refresh()
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
                self._record_managed_goal_event(
                    'succeeded', 'Goal reached within stable tolerance')
                self._finish_task('idle', 'Goal reached within stable tolerance')
                return
            if cancel_reason is not None:
                self._publish_goal_status('goal_canceled')
                self._record_managed_goal_event('canceled', detail)
            elif succeeded:
                self._publish_goal_status('goal_succeeded')
                self._record_managed_goal_event('succeeded', detail)
            else:
                self._publish_goal_status('goal_aborted')
                self._record_managed_goal_event('aborted', detail)
            self._finish_task('idle', detail)
            return

        if context != 'route' or self._mode != 'route':
            self._finish_task('idle', detail)
            return

        point = self._current_point()
        point_name = point['name'] if point else '未知目标点'
        if cancel_reason in (
                'user_stop', 'external_cancel', 'map_changed',
                'inspection_cancel', 'global_localization_untrusted'):
            self._publish_route_log(f'路径已停止：{detail}')
            self._finish_task('stopped', detail)
        elif cancel_reason == 'pause':
            self._state = 'paused_navigation'
            self._status_message = f'Paused before point: {point_name}'
            self._publish_route_log(f'已暂停，当前目标点“{point_name}”尚未完成')
            self._send_zero_velocity()
        elif cancel_reason == 'skip':
            self._publish_route_log(f'已跳过目标点“{point_name}”')
            self._record_point_event('skipped', detail)
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
            self._record_point_event('arrived', detail)
            dwell = float(point.get('dwell_seconds', 0.0)) if point else 0.0
            if self._hold_at_each_point:
                self._state = 'awaiting_release'
                self._status_message = (
                    f'Point reached; waiting for owner release: {point_name}')
                self._publish_route_log(
                    f'目标点“{point_name}”已到达，等待巡检任务释放')
                self._send_zero_velocity()
            elif dwell > 0.0:
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
                body_config = (self._robot_body_status or {}).get('config') or {}
                if (self._effective_navigation_mode() == TERRAIN_3D
                        and body_config.get('selected') == 'xingji'):
                    self._state = 'paused_preflight'
                    self._status_message = (
                        f'3D 目标点“{point_name}”执行失败；路线保留在当前点')
                    self._point_failure_retries = 0
                    self._send_zero_velocity()
                    self._publish_route_log(
                        f'目标点“{point_name}”失败（{detail}）；'
                        '3D 路线已暂停，未跳过未预览的路径段')
                    self._publish_all_status()
                    return
                self._publish_route_log(
                    f'目标点“{point_name}”重试 '
                    f'{self._point_failure_retries} 次后仍失败，已跳过：{detail}')
                self._record_point_event('failed', detail)
                self._point_failure_retries = 0
                self._advance_route()
        self._publish_all_status()

    def _advance_route(self) -> None:
        if self._mode != 'route' or self._route is None:
            return
        self._dwell_deadline = None
        self._paused_dwell_remaining = 0.0
        self._point_failure_retries = 0
        self._points_executed += 1
        if self._point_limit > 0 and self._points_executed >= self._point_limit:
            route_name = self._route['name']
            self._publish_route_log(
                f'路径“{route_name}”已执行指定的 {self._point_limit} 个目标点')
            self._finish_task('completed', 'Route point limit completed')
            return
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
        if self._state in (
                'paused_navigation', 'paused_dwell', 'paused_release',
                'paused_preflight'):
            return True, 'Route is already paused'
        if self._state == 'awaiting_release':
            self._state = 'paused_release'
            self._status_message = 'Point hold paused'
            self._publish_route_log('已暂停，保持在当前巡检点')
            self._send_zero_velocity()
            return True, 'Route paused'
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
            'recovering_lio', 'waiting_lio_recovery', 'settling',
            'waiting_preflight'):
            self._cancel_reason = 'pause'
            self._state = 'pausing'
            self._status_message = 'Pausing route'
            if self._active_goal_handle is not None:
                self._cancel_active_goal()
            elif not self._sending:
                self._cancel_reason = None
                self._clear_preflight_wait()
                self._state = 'paused_navigation'
                self._send_zero_velocity()
            return True, 'Pause requested'
        return False, f'Route cannot be paused from state {self._state}'

    def _resume_route(self):
        if self._state in ('paused_navigation', 'paused_preflight'):
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
        if self._state == 'paused_release':
            self._state = 'awaiting_release'
            self._status_message = 'Waiting for point release'
            self._publish_route_log('已继续，等待巡检任务释放当前点')
            return True, 'Route resumed'
        return False, 'Route is not paused'

    def _skip_route_point(self):
        point = self._current_point()
        if point is None:
            return False, 'No current route point'
        body_config = (self._robot_body_status or {}).get('config') or {}
        if (self._effective_navigation_mode() == TERRAIN_3D
                and body_config.get('selected') == 'xingji'
                and self._state not in ('awaiting_release', 'dwelling')):
            return False, (
                '3D 路线尚未到达当前点，跳过会产生未经预览的新路径段；'
                '请停止任务、修改路线并重新预览')
        if self._state in (
                'paused_dwell', 'paused_navigation', 'paused_release',
                'paused_preflight'):
            return False, 'Resume the route before skipping the current point'
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
            'dwelling', 'waiting_nav2', 'waiting_preflight',
            'awaiting_release'):
            self._clear_preflight_wait()
            self._publish_route_log(f'已跳过目标点“{point["name"]}”')
            self._record_point_event('skipped', 'Point skipped by owner')
            self._advance_route()
            return True, 'Point skipped'
        return False, f'Point cannot be skipped from state {self._state}'

    def _release_route_point(self):
        if not self._hold_at_each_point:
            return False, 'Route is not configured for point release'
        if self._state == 'paused_release':
            return False, 'Resume the route before releasing the point'
        if self._state != 'awaiting_release':
            return False, f'Point cannot be released from state {self._state}'
        point = self._current_point()
        point_name = point['name'] if point else '未知目标点'
        self._publish_route_log(f'巡检任务已释放目标点“{point_name}”')
        self._advance_route()
        return True, 'Point released'

    def _request_stop(self, reason: str):
        if self._mode == 'idle':
            return False, 'No navigation task is active'
        self._pending_single_pose = None
        self._clear_preflight_wait()
        self._dwell_deadline = None
        self._paused_dwell_remaining = 0.0
        self._cancel_reason = reason
        self._state = 'stopping'
        self._status_message = 'Stopping navigation'
        self._send_zero_velocity()
        if self._active_goal_handle is not None:
            if not self._cancel_active_goal():
                return False, 'Nav2 cancellation request failed'
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
        self._active_navigation_mode = ''
        self._state = final_state
        self._status_message = message
        self._route = None
        self._point_index = -1
        self._current_loop = 0
        self._loop_count = 1
        self._continuous = False
        self._reverse = False
        self._hold_at_each_point = False
        self._owner_token = ''
        self._managed_goal_requires_yaw = False
        self._point_limit = 0
        self._points_executed = 0
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
        if getattr(self, '_auto_recovery_stage', 'idle') in (
                'failed', 'manual_reset_required'):
            # Reached only after task completion/cancellation is confirmed.
            # Release task ownership, never the native/body emergency latch.
            self._auto_recovery_stage = (
                'monitoring' if self._lio_recovering else 'idle')
            self._lio_recovery_hard_fault = False
        self._clear_preflight_wait()
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
