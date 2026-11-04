#!/usr/bin/env python3
"""Offline safety-contract tests for Xingji-only, two-stage 3-D goals."""

import ast
import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
import textwrap
from types import SimpleNamespace
import uuid


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / 'lzp_service' / 'route_navigation_manager.py'
SOURCE = SOURCE_PATH.read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)
MANAGER = next(item for item in TREE.body if (
    isinstance(item, ast.ClassDef)
    and item.name == 'RouteNavigationManager'))


@dataclass
class Result:
    allowed: bool
    code: str
    message: str


def _load_plan_check():
    item = next(item for item in TREE.body if (
        isinstance(item, ast.FunctionDef)
        and item.name == '_validate_preview_plan'))
    namespace = {'GoalValidationResult': Result, 'math': math}
    exec('from __future__ import annotations\n'
         + ast.get_source_segment(SOURCE, item), namespace)
    return namespace['_validate_preview_plan']


CHECK_PLAN = _load_plan_check()


def _load_manager_methods(*names):
    names = list(names)
    if '_start_previewed_3d_goal_callback' in names:
        names.extend(name for name in ('_recheck_3d_start','_validate_3d_route_segments')
                     if name not in names)
    if any(name in names for name in ('_preview_3d_guard', '_publish_navigation_mode')):
        if '_navigation_busy_details' not in names:
            names.append('_navigation_busy_details')
    for public, checked in (
            ('_prepare_3d_route_start', '_prepare_3d_route_start_checked'),
            ('_reserve_3d_inspection_route_callback', '_reserve_3d_inspection_route_checked')):
        if public in names and checked not in names:
            names.append(checked)
    functions = [next(item for item in MANAGER.body if (
        isinstance(item, ast.FunctionDef) and item.name == name))
        for name in names]
    namespace = {
        'GoalValidationResult': Result, 'math': math, 'time': time,
        'threading': threading, 'copy': copy, 'uuid': uuid, 'json': json,
        'ComputePathToPose': SimpleNamespace(Goal=lambda: SimpleNamespace()),
        'GoalStatus': SimpleNamespace(STATUS_SUCCEEDED=4),
        'NavPath': lambda: SimpleNamespace(header=SimpleNamespace(frame_id=''), poses=[]),
        'PoseStamped': lambda: _pose(0.0, 0.0),
        '_validate_preview_plan': CHECK_PLAN,
        'TERRAIN_3D': 'terrain_3d',
        'MAP_2D': 'map_2d',
        'normalize_navigation_mode': lambda value: value,
    }
    class_source = 'from __future__ import annotations\nclass Harness:\n'
    for item in functions:
        class_source += textwrap.indent(
            ast.get_source_segment(SOURCE, item), '    ') + '\n'
    exec(class_source, namespace)
    return namespace['Harness']


class _Data:
    def __init__(self, costs):
        self.costs = costs

    def __getitem__(self, cell):
        return self.costs[cell[0]][cell[1]]


class _Grid:
    resolution = 1.0
    origin_x = 0.0
    origin_y = 0.0
    origin_yaw = 0.0

    def __init__(self, costs):
        self.data = _Data(costs)
        self.height = len(costs)
        self.width = len(costs[0])

    def world_to_cell(self, x, y):
        row, column = math.floor(y), math.floor(x)
        if 0 <= row < self.height and 0 <= column < self.width:
            return row, column
        return None


def _pose(x, y, *, frame='map'):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=None),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def _path(*points):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        poses=[_pose(*point) for point in points])


def test_preview_path_checks_between_vertices_and_rejects_unknown():
    grid = _Grid([[0, -1, 0]])
    result = CHECK_PLAN(
        _path((0.2, 0.2), (2.2, 0.2)), grid, _pose(2.2, 0.2))
    assert not result.allowed
    assert result.code == 'path_unknown'


def test_3d_route_footprint_rejects_adjacent_unknown_or_blocked_cell():
    for hazard, expected in ((-1, 'footprint_unknown'),
                             (100, 'footprint_blocked')):
        rows = [[0] * 5 for _ in range(5)]
        rows[1][2] = hazard
        grid = _Grid(rows)
        path = _path((1.2, 2.2), (3.2, 2.2))
        assert CHECK_PLAN(path, grid, _pose(3.2, 2.2)).allowed
        checked = CHECK_PLAN(
            path, grid, _pose(3.2, 2.2),
            footprint_radius_m=0.45)
        assert not checked.allowed and checked.code == expected


def test_single_goal_uses_footprint_on_preview_and_confirmation():
    for method in ('_preview_3d_goal_callback',
                   '_start_previewed_3d_goal_callback'):
        source = next(item for item in MANAGER.body if (
            isinstance(item, ast.FunctionDef) and item.name == method))
        text=ast.get_source_segment(SOURCE, source)
        assert ('footprint_radius_m=0.45' in text or '_recheck_3d_start' in text)
    helper=next(item for item in MANAGER.body if isinstance(item,ast.FunctionDef)
                and item.name=='_validate_3d_route_segments')
    assert 'footprint_radius_m=0.45' in ast.get_source_segment(SOURCE,helper)


def test_preview_path_rejects_special_lethal_and_map_exit():
    for cost in (90, 100):
        grid = _Grid([[0, cost, 0]])
        result = CHECK_PLAN(
            _path((0.2, 0.2), (2.2, 0.2)), grid, _pose(2.2, 0.2))
        assert not result.allowed
        assert result.code == 'needs_calibration'
    clear = _Grid([[0, 0]])
    outside = CHECK_PLAN(
        _path((0.2, 0.2), (2.2, 0.2)), clear, _pose(2.2, 0.2))
    assert outside.code == 'path_outside_map'


def test_preview_path_accepts_only_map_frame_safe_path():
    grid = _Grid([[0, 60, 0]])
    assert CHECK_PLAN(
        _path((0.2, 0.2), (2.2, 0.2)), grid,
        _pose(2.2, 0.2)).allowed
    wrong_frame = _path((0.2, 0.2), (2.2, 0.2))
    wrong_frame.header.frame_id = 'odom'
    assert CHECK_PLAN(
        wrong_frame, grid, _pose(2.2, 0.2)).code == 'invalid_plan'


def test_preview_path_must_begin_near_confirmed_amcl_start():
    grid = _Grid([[0, 0, 0]])
    disconnected = CHECK_PLAN(
        _path((1.2, 0.2), (2.2, 0.2)), grid,
        _pose(2.2, 0.2), start_xy=(0.2, 0.2))
    assert not disconnected.allowed
    assert disconnected.code == 'invalid_plan'
    connected = CHECK_PLAN(
        _path((0.25, 0.2), (2.2, 0.2)), grid,
        _pose(2.2, 0.2), start_xy=(0.2, 0.2))
    assert connected.allowed


def _guard_harness():
    Harness = _load_manager_methods('_preview_3d_guard')
    value = Harness()
    value._navigation_mode = 'terrain_3d'
    value._effective_navigation_mode = lambda: 'terrain_3d'
    value._mode = 'idle'
    value._campus_mission_active = False
    value._inspection_task_active = False
    value._lio_recovering = False
    value._single_refresh_map_name = ''
    value._active_goal_handle = None
    value._sending = False
    value._current_map_name = 'garden'
    value._current_map_load_id = 'load-1'
    value._robot_body_status = {
        'config': {'selected': 'xingji', 'profiles': {
            'xingji': {'navigation_verified': True}}},
        'active_body': 'xingji', 'connected': True,
        'generation': 1,
        'switching': False, 'locked': True,
        'status': {'emergency_latched': False},
    }
    value._robot_body_received_at = time.monotonic()
    value._robot_body_status_samples = 2
    value._inflation_ready_for_navigation = lambda: True
    value._goal_preflight = lambda _pose: Result(True, 'ok', 'ready')
    return value


def test_preview_is_xingji_terrain_only_and_ignores_body_switch_lock():
    value = _guard_harness()
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').allowed
    value._robot_body_status['config']['selected'] = 'gangben_zsl1'
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').code == 'body_unavailable'
    value._robot_body_status['config']['selected'] = 'xingji'
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-old').code == 'map_changed'
    value._navigation_mode = 'map_2d'
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').code == 'wrong_mode'


def test_preview_rejects_stale_faulted_or_busy_state():
    value = _guard_harness()
    value._robot_body_status_samples = 1
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').code == 'body_unavailable'
    value._robot_body_status_samples = 2
    value._robot_body_received_at -= 3.0
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').code == 'body_unavailable'
    value._robot_body_received_at = time.monotonic()
    value._robot_body_status['status']['emergency_latched'] = True
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').code == 'body_unavailable'
    value._robot_body_status['status']['emergency_latched'] = False
    value._inspection_task_active = True
    assert value._preview_3d_guard(
        _pose(1.0, 0.0), 'garden', 'load-1').code == 'navigation_busy'


def test_preview_snapshot_rejects_map_body_terrain_or_pose_change():
    Harness = _load_manager_methods(
        '_preview_3d_snapshot', '_preview_3d_snapshot_matches')
    value = Harness()
    value._current_map_name = 'garden'
    value._current_map_load_id = 'load-1'
    value._map_generation = 3
    value._terrain_costmap_ready_stamp_ns = 500
    value._robot_body_status = {'generation': 7}
    mode_state = {'version': 2}
    inflation_state = {'version': 4}
    value._navigation_mode_store = SimpleNamespace(
        snapshot=lambda: mode_state)
    value._inflation_sync = SimpleNamespace(
        snapshot=lambda: inflation_state)
    value._latest_amcl_pose = SimpleNamespace(
        pose=SimpleNamespace(pose=_pose(0.0, 0.0).pose))
    value._pose_yaw = lambda pose: 0.0
    snapshot = value._preview_3d_snapshot()
    assert value._preview_3d_snapshot_matches(snapshot)
    value._current_map_load_id = 'load-2'
    assert not value._preview_3d_snapshot_matches(snapshot)
    value._current_map_load_id = 'load-1'
    value._terrain_costmap_ready_stamp_ns = 501
    assert not value._preview_3d_snapshot_matches(snapshot)
    value._terrain_costmap_ready_stamp_ns = 500
    value._robot_body_status['generation'] = 8
    assert not value._preview_3d_snapshot_matches(snapshot)
    value._robot_body_status['generation'] = 7
    mode_state['version'] = 3
    assert not value._preview_3d_snapshot_matches(snapshot)
    mode_state['version'] = 2
    inflation_state['version'] = 5
    assert not value._preview_3d_snapshot_matches(snapshot)
    inflation_state['version'] = 4
    value._map_generation = 4
    assert not value._preview_3d_snapshot_matches(snapshot)
    value._map_generation = 3
    value._latest_amcl_pose.pose.pose.position.x = 0.20
    assert not value._preview_3d_snapshot_matches(snapshot)


def test_body_status_requires_two_current_generation_heartbeats():
    Harness = _load_manager_methods('_robot_body_status_callback')
    value = Harness()
    value._lock = threading.RLock()
    value._robot_body_status = None
    value._robot_body_received_at = None
    value._robot_body_status_samples = 0
    value._preview_token = {'id': 'old'}
    message = SimpleNamespace(data=json.dumps({
        'generation': 5, 'active_body': 'xingji'}))
    value._robot_body_status_callback(message)
    assert value._robot_body_status_samples == 1
    assert value._preview_token is None
    value._robot_body_status_callback(message)
    assert value._robot_body_status_samples == 2
    value._robot_body_status_callback(SimpleNamespace(data=json.dumps({
        'generation': 6, 'active_body': 'xingji'})))
    assert value._robot_body_status_samples == 1


class _Future:
    def __init__(self, result):
        self.value = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.value


def test_preview_only_plans_then_confirm_consumes_one_token():
    Harness = _load_manager_methods(
        '_preview_3d_goal_callback', '_start_previewed_3d_goal_callback')
    value = Harness()
    value._lock = threading.RLock()
    value._preview_token = None
    value._preview_busy = False
    value._preview_plan_timeout = 5.0
    value._preview_ttl = 20.0
    value._current_map_name = 'garden'
    value._traversability_grid = _Grid([[0] * 5 for _ in range(4)])
    value._mode = 'idle'
    value._latest_amcl_pose = SimpleNamespace(
        pose=SimpleNamespace(pose=_pose(1.2, 1.2).pose))
    value._preview_3d_guard = lambda _pose, _map, _load_id: Result(
        True, 'ok', 'ready')
    value._preview_3d_snapshot = lambda: {
        'map_name': 'garden', 'map_load_id': 'load-1',
        'start_x': 1.2, 'start_y': 1.2, 'start_yaw': 0.}
    value._preview_3d_identity_matches = lambda _snapshot: True
    value._preview_3d_snapshot_matches = lambda _snapshot: True
    value._await_preview_future = lambda _future, _timeout: True
    value.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: 'stamp'))
    planned = _path((1.2, 1.2), (3.2, 1.2))
    goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: _Future(SimpleNamespace(
            status=4, result=SimpleNamespace(path=planned))))
    planner_calls = []
    value._preview_action_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal: (
            planner_calls.append(goal), _Future(goal_handle))[1])
    navigation_calls = []
    value._action_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal: navigation_calls.append(goal))
    value._single_goal_callback = lambda pose: (
        navigation_calls.append(pose), setattr(value, '_mode', 'single'))

    response = SimpleNamespace()
    value._preview_3d_goal_callback(
        SimpleNamespace(
            goal=_pose(3.2, 1.2), map_name='garden',
            map_load_id='load-1'), response)
    assert response.valid and response.preview_id
    assert planner_calls[0].use_start is True
    assert planner_calls[0].planner_id == 'GridBased'
    assert navigation_calls == [], 'preview must not dispatch NavigateToPose'

    confirmation = SimpleNamespace()
    value._start_previewed_3d_goal_callback(
        SimpleNamespace(preview_id=response.preview_id), confirmation)
    assert confirmation.success
    assert len(navigation_calls) == 1
    second = SimpleNamespace()
    value._start_previewed_3d_goal_callback(
        SimpleNamespace(preview_id=response.preview_id), second)
    assert not second.success
    assert len(navigation_calls) == 1


def test_legacy_web_goal_topic_preserves_verified_gangben_3d_and_2d():
    Harness = _load_manager_methods(
        '_confirmed_gangben_3d_body', '_legacy_web_goal_callback')
    value = Harness()
    value._lock = threading.RLock()
    mode = {'value': 'terrain_3d'}
    value._effective_navigation_mode = lambda: mode['value']
    value._robot_body_status = None
    value._robot_body_received_at = None
    value._robot_body_status_samples = 0
    sent, statuses = [], []
    value._single_goal_callback = lambda pose: sent.append(pose)
    value._publish_goal_status = lambda status: statuses.append(status)
    value._publish_all_status = lambda: None
    goal = _pose(1.0, 0.0)
    value._legacy_web_goal_callback(goal)
    assert sent == []
    assert statuses == ['goal_rejected_3d_preview_required']
    mode['value'] = 'map_2d'
    value._legacy_web_goal_callback(goal)
    assert sent == [goal]
    mode['value'] = 'terrain_3d'
    value._robot_body_status = {
        'config': {'selected': 'gangben_zsl1', 'profiles': {
            'gangben_zsl1': {'navigation_verified': True}}},
        'active_body': 'gangben_zsl1', 'connected': True,
        'generation': 3, 'switching': False, 'error': '',
        'status': {'stale': False, 'emergency_latched': False,
                   'hardware_error': False, 'error': ''},
    }
    value._robot_body_received_at = time.monotonic()
    value._robot_body_status_samples = 2
    value._legacy_web_goal_callback(goal)
    assert sent == [goal, goal], 'verified Gangben keeps the legacy 3D goal'

    # A switch, stale heartbeat, failed profile, or Xingji selection cannot
    # use the legacy topic to bypass Xingji's two-stage preview.
    for mutate in (
        lambda: value._robot_body_status.update(active_body='xingji'),
        lambda: value._robot_body_status['config'].update(selected='xingji'),
        lambda: value._robot_body_status.update(connected=False),
        lambda: setattr(value, '_robot_body_status_samples', 1),
        lambda: setattr(value, '_robot_body_received_at',
                        time.monotonic() - 3.0),
        lambda: value._robot_body_status['config']['profiles'][
            'gangben_zsl1'].update(navigation_verified=False),
        lambda: value._robot_body_status['status'].update(
            emergency_latched=True),
    ):
        value._robot_body_status['active_body'] = 'gangben_zsl1'
        value._robot_body_status['connected'] = True
        value._robot_body_status['config']['selected'] = 'gangben_zsl1'
        value._robot_body_status['config']['profiles'][
            'gangben_zsl1']['navigation_verified'] = True
        value._robot_body_status['status']['emergency_latched'] = False
        value._robot_body_received_at = time.monotonic()
        value._robot_body_status_samples = 2
        mutate()
        value._legacy_web_goal_callback(goal)
        assert sent == [goal, goal]
        assert statuses[-1] == 'goal_rejected_3d_preview_required'
    assert "'/web_goal_pose',\n            self._legacy_web_goal_callback" in SOURCE


def test_active_3d_goal_stops_when_terrain_costmap_is_retracted():
    Harness = _load_manager_methods('_terrain_costmap_status_callback')
    value = Harness()
    value._lock = threading.RLock()
    value._navigation_mode = 'terrain_3d'
    active_mode = {'value': 'terrain_3d'}
    value._effective_navigation_mode = lambda: active_mode['value']
    value._current_map_name = 'garden'
    value._current_map_load_id = 'load-1'
    value._terrain_costmap_ready_map_name = 'garden'
    value._terrain_costmap_ready_load_id = 'load-1'
    value._terrain_costmap_ready_stamp_ns = 123
    value._pending_traversability_stamp_ns = 123
    value._pending_traversability_grid = object()
    value._traversability_grid = value._pending_traversability_grid
    value._traversability_map_name = 'garden'
    value._preview_token = {'id': 'old'}
    value._mode = 'single'
    value._state = 'navigating'
    stops, statuses = [], []
    value._request_stop = lambda reason: (stops.append(reason), (True, ''))[1]
    value._publish_goal_status = lambda status: statuses.append(status)
    value._publish_all_status = lambda: None
    waiting = SimpleNamespace(data=json.dumps({
        'state': 'waiting', 'navigation_mode': 'terrain_3d',
        'map_name': 'garden', 'map_load_id': 'load-1'}))
    value._terrain_costmap_status_callback(waiting)
    assert stops == ['external_cancel']
    assert value._preview_token is None
    assert statuses == ['goal_stopped_terrain_unavailable']

    # The same status in 2D must not stop legacy navigation.
    active_mode['value'] = 'map_2d'
    value._terrain_costmap_ready_map_name = 'garden'
    value._terrain_costmap_ready_load_id = 'load-1'
    value._terrain_costmap_ready_stamp_ns = 124
    value._state = 'navigating'
    value._terrain_costmap_status_callback(waiting)
    assert stops == ['external_cancel']


if __name__ == '__main__':
    test_preview_path_checks_between_vertices_and_rejects_unknown()
    test_3d_route_footprint_rejects_adjacent_unknown_or_blocked_cell()
    test_single_goal_uses_footprint_on_preview_and_confirmation()
    test_preview_path_rejects_special_lethal_and_map_exit()
    test_preview_path_accepts_only_map_frame_safe_path()
    test_preview_path_must_begin_near_confirmed_amcl_start()
    test_preview_is_xingji_terrain_only_and_ignores_body_switch_lock()
    test_preview_rejects_stale_faulted_or_busy_state()
    test_preview_snapshot_rejects_map_body_terrain_or_pose_change()
    test_body_status_requires_two_current_generation_heartbeats()
    test_preview_only_plans_then_confirm_consumes_one_token()
    test_legacy_web_goal_topic_preserves_verified_gangben_3d_and_2d()
    test_active_3d_goal_stops_when_terrain_costmap_is_retracted()
