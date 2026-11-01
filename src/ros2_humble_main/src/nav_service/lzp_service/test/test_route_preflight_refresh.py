#!/usr/bin/env python3
"""Regression coverage for transient route-goal preflight failures."""

import ast
import copy
import math
from pathlib import Path
import textwrap
import threading
import time
from types import SimpleNamespace
from typing import Optional


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROUTE_MANAGER = PACKAGE_ROOT / 'lzp_service' / 'route_navigation_manager.py'
LAUNCH_MANAGER = (
    PACKAGE_ROOT.parents[1] / 'car_vel' / 'launch' /
    'launch_manager.launch.py')


def load_dispatch_harness():
    source = ROUTE_MANAGER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'RouteNavigationManager'
    )
    method = next(
        node for node in manager.body
        if isinstance(node, ast.FunctionDef)
        and node.name == '_dispatch_current_route_point'
    )
    namespace = {'TERRAIN_3D': 'terrain_3d'}
    body = 'class RouteHarness:\n' + textwrap.indent(
        ast.get_source_segment(source, method), '    ')
    exec(body, namespace)
    return namespace['RouteHarness']


def load_refresh_harness():
    source = ROUTE_MANAGER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'RouteNavigationManager'
    )
    methods = []
    for name in ('_clear_preflight_wait', '_drive_preflight_wait'):
        methods.append(next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and node.name == name
        ))

    class EmptyServiceStub:
        class Request:
            pass

    namespace = {
        'EmptyService': EmptyServiceStub,
        'Optional': Optional,
        'time': time,
        'TERRAIN_3D': 'terrain_3d',
    }
    body = 'class RefreshHarness:\n' + ''.join(
        textwrap.indent(ast.get_source_segment(source, method), '    ') + '\n'
        for method in methods
    )
    exec(body, namespace)
    return namespace['RefreshHarness']


def load_amcl_callback_harness():
    source = ROUTE_MANAGER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'RouteNavigationManager'
    )
    method = next(
        node for node in manager.body
        if isinstance(node, ast.FunctionDef) and node.name == '_amcl_pose_callback'
    )
    namespace = {
        'PoseWithCovarianceStamped': object,
        'copy': copy,
        'math': math,
        'time': time,
    }
    body = 'class AmclHarness:\n' + textwrap.indent(
        ast.get_source_segment(source, method), '    ')
    exec(body, namespace)
    return namespace['AmclHarness']


def make_harness(preflight_code):
    harness = load_dispatch_harness()()
    harness.TRANSIENT_PREFLIGHT_CODES = frozenset({
        'localization_stale', 'terrain_not_ready',
        'robot_outside_map', 'robot_not_traversable'})
    point = {'id': 11, 'name': '目标点 1'}
    harness._mode = 'route'
    harness._state = 'starting'
    harness._lio_recovering = False
    harness._sending = False
    harness._active_goal_handle = None
    harness._route = {'name': 'ceshi1', 'points': [point]}
    harness._point_index = 0
    harness._current_loop = 1
    harness._action_client = SimpleNamespace(server_is_ready=lambda: True)
    harness._inflation_ready_for_navigation = lambda: True
    harness._effective_navigation_mode = lambda: 'map_2d'
    harness._robot_body_status = {'config': {'selected': 'xingji'}}
    harness._send_zero_velocity = lambda: None
    harness._publish_all_status = lambda: None
    harness._current_point = lambda: point
    harness._route_pose = lambda _point: SimpleNamespace()
    harness._goal_preflight = lambda _pose: SimpleNamespace(
        allowed=False,
        code=preflight_code,
        message={
            'localization_stale': '机器人实时位置已过期',
            'terrain_not_ready': '地形代价图尚未就绪',
            'robot_not_traversable': '机器人当前位置不在已验证可通行区域',
            'goal_blocked': '目标点不可通行',
        }[preflight_code],
    )
    harness.logs = []
    harness.events = []
    harness.waits = []
    harness.sent = []
    harness.advances = 0
    harness._publish_route_log = harness.logs.append
    harness._record_point_event = (
        lambda result, detail: harness.events.append((result, detail)))
    harness._begin_preflight_wait = (
        lambda pose, context, result: harness.waits.append(
            (pose, context, result.code)))
    harness._advance_route = lambda: setattr(
        harness, 'advances', harness.advances + 1)
    harness._send_pose = lambda pose, context: harness.sent.append(
        (pose, context))
    return harness


def test_stale_localization_waits_without_skipping_current_point():
    harness = make_harness('localization_stale')

    harness._dispatch_current_route_point()

    assert harness.waits and harness.waits[0][1:] == (
        'route', 'localization_stale')
    assert harness.advances == 0
    assert harness.events == []
    assert harness.sent == []


def test_unready_terrain_waits_without_skipping_current_point():
    harness = make_harness('terrain_not_ready')

    harness._dispatch_current_route_point()

    assert harness.waits and harness.waits[0][1:] == (
        'route', 'terrain_not_ready')
    assert harness.advances == 0
    assert harness.events == []


def test_permanently_blocked_goal_keeps_existing_skip_policy():
    harness = make_harness('goal_blocked')

    harness._dispatch_current_route_point()

    assert harness.waits == []
    assert harness.advances == 1
    assert harness.events and harness.events[0][0] == 'unreachable'


def test_blocked_xingji_3d_goal_pauses_instead_of_unpreviewed_skip():
    harness = make_harness('goal_blocked')
    harness._effective_navigation_mode = lambda: 'terrain_3d'

    harness._dispatch_current_route_point()

    assert harness._state == 'paused_preflight'
    assert harness.advances == 0
    assert harness.events == []
    assert harness.sent == []


def test_robot_start_failure_waits_without_completing_route():
    harness = make_harness('robot_not_traversable')

    harness._dispatch_current_route_point()

    assert harness.waits and harness.waits[0][1:] == (
        'route', 'robot_not_traversable')
    assert harness.advances == 0
    assert harness.events == []


def make_refresh_harness(results):
    harness = load_refresh_harness()()
    harness.TRANSIENT_PREFLIGHT_CODES = frozenset({
        'localization_stale', 'terrain_not_ready',
        'robot_outside_map', 'robot_not_traversable'})
    point = {'id': 11, 'name': '目标点 1'}
    harness._preflight_wait_context = 'route'
    harness._preflight_wait_pose = SimpleNamespace()
    harness._preflight_wait_code = 'localization_stale'
    harness._preflight_wait_deadline = 20.0
    harness._preflight_wait_next_nomotion_at = 0.0
    harness._preflight_wait_nomotion_requests = 0
    harness._nomotion_update_max_requests = 8
    harness._nomotion_update_interval = 0.35
    harness._mode = 'route'
    harness._state = 'waiting_preflight'
    harness._route = {'name': 'ceshi1', 'points': [point]}
    harness._point_index = 0
    harness._point_failure_retries = 0
    harness._effective_navigation_mode = lambda: 'map_2d'
    harness._robot_body_status = {'config': {'selected': 'xingji'}}
    harness._publish_all_status = lambda: None
    harness._current_point = lambda: point
    harness._route_pose = lambda _point: SimpleNamespace()
    queue = list(results)
    harness._goal_preflight = lambda _pose: queue.pop(0)
    harness.logs = []
    harness.events = []
    harness.sent = []
    harness.advances = 0
    harness.zeros = 0
    harness._publish_route_log = harness.logs.append
    harness._record_point_event = (
        lambda result, detail: harness.events.append((result, detail)))
    harness._advance_route = lambda: setattr(
        harness, 'advances', harness.advances + 1)
    harness._send_pose = lambda pose, context: harness.sent.append(
        (pose, context))
    harness._send_zero_velocity = lambda: setattr(
        harness, 'zeros', harness.zeros + 1)
    harness._finish_task = lambda state, detail: None
    harness.nomotion_calls = 0

    def request_nomotion(_request):
        harness.nomotion_calls += 1

    harness._nomotion_update_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=request_nomotion,
    )
    return harness


def preflight(allowed, code='', message=''):
    return SimpleNamespace(allowed=allowed, code=code, message=message)


def test_wait_requests_nomotion_then_retries_the_same_point():
    harness = make_refresh_harness([
        preflight(False, 'localization_stale', '机器人实时位置已过期'),
        preflight(True),
    ])

    harness._drive_preflight_wait(10.0)
    assert harness.nomotion_calls == 1
    assert harness.sent == []
    assert harness.advances == 0

    harness._drive_preflight_wait(10.2)
    assert len(harness.sent) == 1
    assert harness.sent[0][1] == 'route'
    assert harness.advances == 0
    assert harness._point_index == 0
    assert harness._preflight_wait_context is None


def test_refresh_timeout_pauses_and_retains_the_current_point():
    harness = make_refresh_harness([
        preflight(False, 'localization_stale', '机器人实时位置已过期')])
    harness._preflight_wait_deadline = 9.0

    harness._drive_preflight_wait(10.0)

    assert harness._state == 'paused_preflight'
    assert harness._point_index == 0
    assert harness.advances == 0
    assert harness.events == []
    assert harness.zeros == 1
    assert any('当前点未跳过' in item for item in harness.logs)


def test_xingji_3d_wait_becomes_blocked_and_never_skips():
    harness = make_refresh_harness([
        preflight(False, 'goal_blocked', '目标点不可通行')])
    harness._effective_navigation_mode = lambda: 'terrain_3d'

    harness._drive_preflight_wait(10.0)

    assert harness._state == 'paused_preflight'
    assert harness._point_index == 0
    assert harness.advances == 0
    assert harness.events == []
    assert harness.sent == []
    assert harness.zeros == 1
    assert any('当前点未跳过' in item for item in harness.logs)


def test_2d_wait_becomes_blocked_and_keeps_existing_skip_policy():
    harness = make_refresh_harness([
        preflight(False, 'goal_blocked', '目标点不可通行')])

    harness._drive_preflight_wait(10.0)

    assert harness.advances == 1
    assert harness.events and harness.events[0][0] == 'unreachable'


def amcl_message(stamp_ns, x):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )),
    )


def test_duplicate_amcl_timestamp_cannot_fake_a_fresh_pose():
    harness = load_amcl_callback_harness()()
    harness._lock = threading.RLock()
    harness._lio_recovering = False
    harness._latest_amcl_pose = None
    harness._latest_amcl_received_at = None
    harness._latest_amcl_stamp_ns = 0
    first = amcl_message(12_000_000_345, 1.0)
    duplicate = amcl_message(12_000_000_345, 99.0)

    harness._amcl_pose_callback(first)
    first_received_at = harness._latest_amcl_received_at
    harness._amcl_pose_callback(duplicate)

    assert harness._latest_amcl_pose.pose.pose.position.x == 1.0
    assert harness._latest_amcl_received_at == first_received_at


def test_start_unknown_tolerance_is_explicitly_deployed():
    route = ROUTE_MANAGER.read_text(encoding='utf-8')
    launch = LAUNCH_MANAGER.read_text(encoding='utf-8')

    assert "'goal_preflight_start_tolerance_m', 0.35" in route
    assert launch.count("'goal_preflight_start_tolerance_m'") >= 2
    assert "'goal_preflight_start_tolerance_m': LaunchConfiguration(" in launch


if __name__ == '__main__':
    test_stale_localization_waits_without_skipping_current_point()
    test_unready_terrain_waits_without_skipping_current_point()
    test_permanently_blocked_goal_keeps_existing_skip_policy()
    test_blocked_xingji_3d_goal_pauses_instead_of_unpreviewed_skip()
    test_robot_start_failure_waits_without_completing_route()
    test_wait_requests_nomotion_then_retries_the_same_point()
    test_refresh_timeout_pauses_and_retains_the_current_point()
    test_xingji_3d_wait_becomes_blocked_and_never_skips()
    test_2d_wait_becomes_blocked_and_keeps_existing_skip_policy()
    test_duplicate_amcl_timestamp_cannot_fake_a_fresh_pose()
    test_start_unknown_tolerance_is_explicitly_deployed()
