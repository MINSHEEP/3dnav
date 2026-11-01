#!/usr/bin/env python3
"""Policy tests for mode-aware route goal preflight."""

import ast
from pathlib import Path
from types import SimpleNamespace
import math
import textwrap
import time


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROUTE_MANAGER = PACKAGE_ROOT / 'lzp_service' / 'route_navigation_manager.py'


def _harness(mode, *, pose_age=0.0, with_grid=False):
    source = ROUTE_MANAGER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == 'RouteNavigationManager')
    methods = [
        node for node in manager.body
        if isinstance(node, ast.FunctionDef)
        and node.name in ('_goal_preflight', '_global_localization_required',
                          '_global_localization_check')]

    class Result:
        def __init__(self, allowed, code, message):
            self.allowed = allowed
            self.code = code
            self.message = message

    namespace = {
        'PoseStamped': object,
        'GoalValidationResult': Result,
        'MAP_2D': 'map_2d',
        'TERRAIN_3D': 'terrain_3d',
        'uses_terrain': lambda value: value == 'terrain_3d',
        'validate_navigation_goal': lambda *_args, **_kwargs: Result(
            True, 'allowed_3d', 'terrain validation passed'),
        'math': math,
        'time': time,
    }
    body = 'class Harness:\n' + '\n'.join(textwrap.indent(
        ast.get_source_segment(source, method), '    ') for method in methods)
    exec(body, namespace)
    value = namespace['Harness']()
    value._current_map_name = 'garden'
    value._current_map_load_id = 'load-1'
    value._traversability_grid = object() if with_grid else None
    value._traversability_map_name = 'garden' if with_grid else ''
    value._terrain_costmap_ready_load_id = 'load-1' if with_grid else ''
    value._latest_amcl_pose = SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0))))
    value._latest_amcl_received_at = time.monotonic() - pose_age
    value._goal_preflight_pose_max_age = 3.0
    value._goal_preflight_tolerance = 0.1
    value._goal_preflight_start_tolerance = 0.35
    value._effective_navigation_mode = lambda: mode
    value._robot_body_status = {'config': {'selected': 'xingji'}, 'generation': 1}
    value._global_localization_status = (dict(ready=True, scope='xingji_3d',
        map_name='garden', map_load_id='load-1', body_generation=1)
        if mode == 'terrain_3d' else {})
    value._global_localization_at = time.monotonic()
    return value


def _goal(x=1.0, y=2.0):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y)))


def test_2d_preflight_keeps_map_and_localization_but_skips_terrain_grid():
    result = _harness('map_2d')._goal_preflight(_goal())
    assert result.allowed
    assert result.code == 'allowed_2d'


def test_2d_preflight_still_rejects_stale_localization():
    result = _harness('map_2d', pose_age=10.0)._goal_preflight(_goal())
    assert not result.allowed
    assert result.code == 'localization_stale'


def test_3d_preflight_still_fails_closed_without_terrain():
    result = _harness('terrain_3d')._goal_preflight(_goal())
    assert not result.allowed
    assert result.code == 'terrain_not_ready'


def test_3d_preflight_rejects_stale_same_name_load_id():
    value = _harness('terrain_3d', with_grid=True)
    value._terrain_costmap_ready_load_id = 'older-load'
    result = value._goal_preflight(_goal())
    assert not result.allowed
    assert result.code == 'terrain_not_ready'
    value._terrain_costmap_ready_load_id = 'load-1'
    assert value._goal_preflight(_goal()).allowed


def test_both_modes_reject_non_finite_goals():
    for mode, with_grid in (('map_2d', False), ('terrain_3d', True)):
        result = _harness(mode, with_grid=with_grid)._goal_preflight(
            _goal(float('nan'), 0.0))
        assert not result.allowed
        assert result.code == 'invalid_goal'


if __name__ == '__main__':
    test_2d_preflight_keeps_map_and_localization_but_skips_terrain_grid()
    test_2d_preflight_still_rejects_stale_localization()
    test_3d_preflight_still_fails_closed_without_terrain()
    test_3d_preflight_rejects_stale_same_name_load_id()
    test_both_modes_reject_non_finite_goals()
