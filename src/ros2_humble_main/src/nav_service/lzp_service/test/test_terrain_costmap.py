#!/usr/bin/env python3
"""Pure tests for the terrain-aware Nav2 map seam."""

from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.terrain_costmap import (  # noqa: E402
    GridGeometry,
    fuse_terrain_costmap,
    select_navigation_costmap,
)


def geometry(width=3, height=2, resolution=1.0, x=0.0, y=0.0, yaw=0.0):
    return GridGeometry(resolution, width, height, x, y, yaw)


def test_both_maps_have_veto_power_and_costs_are_preserved():
    result = fuse_terrain_costmap(
        [0, 0, 100, -1, 0, 0], geometry(),
        [0, 60, 0, 0, 90, 100], geometry(), special_cost=90)

    assert result.data.tolist() == [0, 60, 100, -1, 90, 100]
    assert result.caution_cells == 1
    assert result.special_cells == 1
    assert result.blocked_cells == 2


def test_special_terrain_is_lethal_by_default_to_force_connector_handoff():
    result = fuse_terrain_costmap(
        [0], geometry(width=1, height=1),
        [90], geometry(width=1, height=1), special_cost=100)

    assert result.data.tolist() == [100]
    assert result.special_cells == 1


def test_missing_terrain_coverage_never_becomes_free():
    result = fuse_terrain_costmap(
        [0] * 6, geometry(),
        [0], geometry(width=1, height=1))

    assert result.data.tolist() == [0, -1, -1, -1, -1, -1]
    assert result.unknown_cells == 5


def test_base_origin_yaw_is_used_when_sampling_terrain():
    base = geometry(width=2, height=1, x=2.0, y=0.0, yaw=np.pi / 2.0)
    terrain = geometry(width=1, height=3, x=1.0, y=0.0)

    result = fuse_terrain_costmap([0, 0], base, [0, 60, 90], terrain)

    assert result.data.tolist() == [0, 60]


def test_invalid_lengths_are_rejected():
    try:
        fuse_terrain_costmap([0], geometry(), [0] * 6, geometry())
    except ValueError as error:
        assert 'base grid' in str(error)
    else:
        raise AssertionError('invalid base grid length must be rejected')


def test_2d_mode_publishes_the_raw_map_without_waiting_for_terrain():
    result = select_navigation_costmap(
        'map_2d', [0, 25, 100, -1, 0, 0], geometry())

    assert result.data.tolist() == [0, 25, 100, -1, 0, 0]
    assert result.terrain_covered_cells == 0
    assert result.special_cells == 0
    assert result.blocked_cells == 1


def test_3d_mode_keeps_the_existing_terrain_fusion_policy():
    result = select_navigation_costmap(
        'terrain_3d', [0, 0], geometry(width=2, height=1),
        [0, 100], geometry(width=2, height=1))

    assert result.data.tolist() == [0, 100]
    assert result.terrain_covered_cells == 2
