#!/usr/bin/env python3
"""Regression coverage for stable near-goal route completion."""

import ast
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
    for name in ('_pose_yaw', '_goal_feedback_callback'):
        methods.append(next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and node.name == name
        ))
    namespace = {'math': math, 'time': time}
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
    harness._goal_token = 7
    harness._active_goal_handle = object()
    harness._active_pose = SimpleNamespace(pose=pose(0.0, 0.0, 0.0))
    harness._cancel_reason = None
    harness._lio_recovering = False
    harness._settled_since = time.monotonic() - 2.1
    harness._settled_xy_tolerance = 0.25
    harness._settled_yaw_tolerance = math.radians(20.0)
    harness._settled_requires_yaw = require_yaw
    harness._managed_goal_requires_yaw = False
    harness._settled_hold_seconds = 2.0
    harness._state = 'navigating'
    cancelled = []
    harness._cancel_active_goal = lambda: cancelled.append(True)
    return harness, cancelled


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
    assert 'threshold_to_consider: 0.15' in goal_angle_block


if __name__ == '__main__':
    test_position_stable_patrol_point_completes_despite_heading_error()
    test_strict_heading_mode_keeps_waiting_for_yaw_alignment()
    test_goal_angle_critic_only_activates_inside_xy_goal_tolerance()
