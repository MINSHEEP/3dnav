#!/usr/bin/env python3
"""Regression coverage for stable near-goal route completion."""

import ast
import copy
import math
from pathlib import Path
import threading
import time
import textwrap
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROUTE_MANAGER = PACKAGE_ROOT / 'lzp_service' / 'route_navigation_manager.py'
NAV_PARAMS = (
    PACKAGE_ROOT.parents[1] / 'navigation' / 'robot_navigation2'
    / 'config' / 'nav2_params.yaml'
)


def load_goal_feedback_harness():
    source = ROUTE_MANAGER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'RouteNavigationManager'
    )
    methods = []
    for name in ('_pose_yaw', '_goal_feedback_callback', '_launch_status_callback',
                 '_skip_route_point', '_handle_goal_terminal'):
        methods.append(next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and node.name == name
        ))
    namespace = {'math': math, 'time': time, 'copy': copy,
                 'LaunchStatus': object, 'TERRAIN_3D': 'terrain_3d'}
    body = 'class RouteHarness:\n' + ''.join(
        textwrap.indent(ast.get_source_segment(source, method), '    ') + '\n'
        for method in methods
    )
    exec(body, namespace)
    harness = namespace['RouteHarness']
    harness._pose_yaw = staticmethod(harness._pose_yaw)
    return harness


def pose(x, y, yaw):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y),
        orientation=SimpleNamespace(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0),
        ),
    )


def feedback(current_pose):
    return SimpleNamespace(
        feedback=SimpleNamespace(current_pose=SimpleNamespace(pose=current_pose)))


def make_harness(require_yaw):
    harness = load_goal_feedback_harness()()
    harness._lock = threading.RLock()
    harness._speed_pending = {}
    harness._goal_token = 7
    harness._active_goal_handle = object()
    harness._active_pose = SimpleNamespace(pose=pose(0.0, 0.0, 0.0))
    harness._cancel_reason = None
    harness._lio_recovering = False
    harness._settled_since = time.monotonic() - 2.1
    harness._settled_xy_tolerance = 0.30
    harness._settled_yaw_tolerance = math.radians(20.0)
    harness._settled_requires_yaw = require_yaw
    harness._managed_goal_requires_yaw = False
    harness._settled_hold_seconds = 2.0
    harness._state = 'navigating'
    cancelled = []
    harness._cancel_active_goal = lambda: cancelled.append(True)
    return harness, cancelled


def test_stuck_route_stops_only_on_fresh_exit_and_missing_action_server():
    harness, _ = make_harness(require_yaw=False)
    harness._state, harness._mode = 'stopping', 'route'
    harness._route_stop_requested_at_ns = 9_000_000_000
    harness.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    server_ready = [False]
    harness._action_client = SimpleNamespace(server_is_ready=lambda: server_ready[0])
    harness._publish_route_log = lambda message: None
    finished = []
    harness._finish_task = lambda state, message: finished.append(state)
    status = SimpleNamespace(navigation_running=False, navigation_pid=0,
                             timestamp=SimpleNamespace(sec=8, nanosec=0))
    harness._launch_status_callback(status)
    status.timestamp.sec = 9
    server_ready[0] = True
    harness._launch_status_callback(status)
    server_ready[0] = False
    status.navigation_pid = 123
    harness._launch_status_callback(status)
    assert finished == []
    status.navigation_pid = 0
    harness._launch_status_callback(status)
    assert finished == ['stopped']
    assert harness._goal_token == 8


def test_position_stable_patrol_point_completes_despite_heading_error():
    harness, cancelled = make_harness(require_yaw=False)

    harness._goal_feedback_callback(feedback(pose(0.20, 0.0, math.pi)), 7)

    assert harness._cancel_reason == 'settled_goal'
    assert harness._state == 'settling'
    assert cancelled == [True]


def test_strict_heading_mode_keeps_waiting_for_yaw_alignment():
    harness, cancelled = make_harness(require_yaw=True)

    harness._goal_feedback_callback(feedback(pose(0.20, 0.0, math.pi)), 7)

    assert harness._cancel_reason is None
    assert harness._settled_since is None
    assert cancelled == []


def test_goal_angle_critic_only_activates_inside_xy_goal_tolerance():
    route_source = ROUTE_MANAGER.read_text(encoding='utf-8')
    params = NAV_PARAMS.read_text(encoding='utf-8')
    goal_angle_block = params.split('GoalAngleCritic:', 1)[1].split(
        'PreferForwardCritic:', 1)[0]
    assert "'settled_requires_yaw', False" in route_source
    assert 'threshold_to_consider: 0.25' in goal_angle_block


if __name__ == '__main__':
    test_position_stable_patrol_point_completes_despite_heading_error()
    test_strict_heading_mode_keeps_waiting_for_yaw_alignment()
    test_goal_angle_critic_only_activates_inside_xy_goal_tolerance()


def test_near_goal_shuffle_within_30cm_finishes_but_farther_pose_does_not():
    harness, cancelled = make_harness(require_yaw=False)
    harness._goal_feedback_callback(feedback(pose(0.28, 0.0, math.pi)), 7)
    assert harness._cancel_reason == 'settled_goal'
    assert cancelled == [True]
    harness, cancelled = make_harness(require_yaw=False)
    harness._goal_feedback_callback(feedback(pose(0.31, 0.0, 0.0)), 7)
    assert harness._settled_since is None
    assert cancelled == []


def test_near_goal_still_requires_continuous_hold_and_current_goal():
    harness, cancelled = make_harness(require_yaw=False)
    harness._settled_since = None
    harness._goal_feedback_callback(feedback(pose(0.28, 0.0, 0.0)), 7)
    assert cancelled == []
    harness._settled_since = time.monotonic() - 3.0
    harness._goal_feedback_callback(feedback(pose(0.28, 0.0, 0.0)), 6)
    assert cancelled == []


def test_skip_before_arrival_cancels_then_advances_only_after_goal_terminal():
    harness, cancelled = make_harness(require_yaw=False)
    harness._mode = 'route'
    harness._goal_context = 'route'
    harness._robot_body_status = {'config': {'selected': 'xingji'}}
    harness._effective_navigation_mode = lambda: 'map_2d'
    harness._current_point = lambda: {'name': '未到达点'}
    events, advances = [], []
    harness._publish_route_log = lambda message: None
    harness._publish_all_status = lambda: None
    harness._record_point_event = lambda *event: events.append(event)
    harness._advance_route = lambda: advances.append(True)
    ok, _ = harness._skip_route_point()
    assert ok and cancelled == [True]
    assert harness._cancel_reason == 'skip'
    assert advances == []
    harness._handle_goal_terminal(False, 'Goal canceled')
    assert events == [('skipped', 'Goal canceled')]
    assert advances == [True]


def test_xingji_3d_skip_still_requires_a_previewed_path():
    harness, cancelled = make_harness(require_yaw=False)
    harness._robot_body_status = {'config': {'selected': 'xingji'}}
    harness._effective_navigation_mode = lambda: 'terrain_3d'
    harness._current_point = lambda: {'name': '未到达点'}
    ok, message = harness._skip_route_point()
    assert not ok and '未经预览' in message
    assert cancelled == []
