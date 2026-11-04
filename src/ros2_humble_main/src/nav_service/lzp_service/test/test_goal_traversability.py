#!/usr/bin/env python3
"""Pure regression tests for target cost and connectivity preflight."""

from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.goal_traversability import (  # noqa: E402
    TraversabilityGrid,
    validate_navigation_goal,
)


def grid(data, width=5, height=3, resolution=1.0):
    return TraversabilityGrid.from_costs(
        data=np.asarray(data, dtype=np.int8),
        resolution=resolution,
        width=width,
        height=height,
        origin_x=0.0,
        origin_y=0.0,
    )


def test_same_component_goal_is_allowed():
    terrain = grid([
        0, 0, 0, 100, 0,
        0, 60, 0, 100, 0,
        0, 0, 0, 100, 0,
    ])

    result = validate_navigation_goal(
        terrain, start_x=0.5, start_y=0.5,
        goal_x=2.5, goal_y=2.5, goal_tolerance_m=0.10)

    assert result.allowed
    assert result.code == 'ok'


def test_nearby_free_cell_in_another_component_never_allows_goal():
    terrain = grid([
        0, 0, 100, 0, 0,
        0, 0, 100, 100, 0,
        0, 0, 100, 0, 0,
    ])

    result = validate_navigation_goal(
        terrain, start_x=0.5, start_y=0.5,
        goal_x=3.5, goal_y=0.5, goal_tolerance_m=2.0)

    assert not result.allowed
    assert result.code == 'goal_disconnected'


def test_blocked_goal_reports_nearest_cell_in_start_component():
    terrain = grid([
        0, 0, 0, 100, 100,
        0, 0, 0, 100, 100,
        0, 0, 0, 100, 100,
    ])

    result = validate_navigation_goal(
        terrain, start_x=0.5, start_y=0.5,
        goal_x=3.5, goal_y=1.5, goal_tolerance_m=0.10)

    assert not result.allowed
    assert result.code == 'goal_blocked'
    assert 0.99 < result.nearest_start_component_distance_m < 1.01


def test_blocked_goal_within_navfn_tolerance_is_allowed():
    terrain = grid([
        0, 0, 0, 100, 100,
        0, 0, 0, 100, 100,
        0, 0, 0, 100, 100,
    ])

    result = validate_navigation_goal(
        terrain, start_x=0.5, start_y=0.5,
        goal_x=3.01, goal_y=1.5, goal_tolerance_m=0.60)

    assert result.allowed
    assert result.code == 'within_tolerance'


def test_unknown_start_hole_uses_nearby_verified_component():
    terrain = grid([
        0, 0, 0, 0, 0,
        0, -1, 0, 0, 0,
        0, 0, 0, 0, 0,
    ], resolution=0.25)

    result = validate_navigation_goal(
        terrain, start_x=0.375, start_y=0.375,
        goal_x=0.875, goal_y=0.375, goal_tolerance_m=0.10)

    assert result.allowed
    assert result.code == 'ok_start_nearby'
    assert 0.249 < result.nearest_start_traversable_distance_m < 0.251


def test_lethal_start_never_uses_nearby_component():
    terrain = grid([
        0, 0, 0, 0, 0,
        0, 100, 0, 0, 0,
        0, 0, 0, 0, 0,
    ], resolution=0.25)

    result = validate_navigation_goal(
        terrain, start_x=0.375, start_y=0.375,
        goal_x=0.875, goal_y=0.375, goal_tolerance_m=0.10)

    assert not result.allowed
    assert result.code == 'robot_not_traversable'


def test_unknown_start_beyond_limit_remains_rejected():
    terrain = grid([
        -1, -1, -1, -1, -1,
        -1, -1, -1, 0, 0,
        -1, -1, -1, 0, 0,
    ], resolution=0.25)

    result = validate_navigation_goal(
        terrain, start_x=0.125, start_y=0.375,
        goal_x=0.875, goal_y=0.375, goal_tolerance_m=0.10)

    assert not result.allowed
    assert result.code == 'robot_not_traversable'
    assert result.nearest_start_traversable_distance_m > 0.35
