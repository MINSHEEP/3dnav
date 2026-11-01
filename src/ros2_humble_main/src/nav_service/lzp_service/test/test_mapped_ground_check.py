#!/usr/bin/env python3
"""The restricted flat-site map prior must fail closed on missing ground."""

from dataclasses import replace
from pathlib import Path
import math
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lzp_service.mapped_ground_check import check_mapped_ground  # noqa: E402
from lzp_service.terrain_grid import (  # noqa: E402
    GroundSupportGrid, TerrainGridData,
)


def _fixture():
    width = height = 20
    count = width * height
    terrain = TerrainGridData(
        resolution=0.25, width=width, height=height,
        origin_x=-2.5, origin_y=-2.5,
        elevation=np.zeros(count, dtype=np.float32),
        slope_deg=np.zeros(count, dtype=np.float32),
        roughness_m=np.zeros(count, dtype=np.float32),
        step_height_m=np.zeros(count, dtype=np.float32),
        confidence=np.ones(count, dtype=np.float32),
        traversal_cost=np.zeros(count, dtype=np.int8),
    )
    support = GroundSupportGrid(
        resolution=0.05, width=200, height=200,
        origin_x=-5.0, origin_y=-5.0,
        free_mask=np.ones(200 * 200, dtype=bool),
    )
    return terrain, support


def _check(terrain, support, **kwargs):
    arguments = dict(
        expected_map_id='sha256:map-a', current_map_id='sha256:map-a',
        pose_x=0.0, pose_y=0.0, pose_yaw=0.0,
    )
    arguments.update(kwargs)
    return check_mapped_ground(terrain, support, **arguments)


def _change_cell(grid, name, *, x=0.0, y=0.0, value):
    column = int(math.floor((x - grid.origin_x) / grid.resolution))
    row = int(math.floor((y - grid.origin_y) / grid.resolution))
    values = np.asarray(getattr(grid, name)).copy()
    values[row * grid.width + column] = value
    return replace(grid, **{name: values})


def test_known_flat_ground_with_explicit_2d_support_can_pass():
    terrain, support = _fixture()
    result = _check(terrain, support)
    assert result.safe
    assert result.reason == 'known_flat_ground'
    assert result.terrain_cells_checked >= 20
    assert result.support_cells_checked >= result.terrain_cells_checked


def test_unknown_in_robot_footprint_or_lookahead_rejects():
    terrain, support = _fixture()
    for x in (-0.25, 1.1):
        unknown = _change_cell(
            terrain, 'traversal_cost', x=x, value=-1)
        result = _check(unknown, support)
        assert not result.safe
        assert result.reason == 'terrain_unknown_or_blocked'
        assert result.failing_terrain_cell is not None


def test_outside_map_and_nonfinite_pose_reject():
    terrain, support = _fixture()
    assert _check(terrain, support, pose_x=2.0).reason == 'outside_map'
    assert _check(terrain, support, pose_yaw=float('nan')).reason == (
        'invalid_input')


def test_mismatched_or_missing_map_identity_rejects():
    terrain, support = _fixture()
    assert _check(
        terrain, support, current_map_id='sha256:other').reason == (
            'map_mismatch')
    assert _check(
        terrain, support, expected_map_id='').reason == 'map_mismatch'


def test_special_and_blocked_cost_never_pass():
    terrain, support = _fixture()
    for value in (90, 100, -1):
        unsafe = _change_cell(
            terrain, 'traversal_cost', value=value)
        result = _check(unsafe, support)
        assert not result.safe
        assert result.reason == 'terrain_unknown_or_blocked'


def test_nonfree_2d_support_rejects_even_when_terrain_is_flat():
    terrain, support = _fixture()
    support = _change_cell(support, 'free_mask', x=0.0, y=0.0,
                           value=False)
    result = _check(terrain, support)
    assert not result.safe
    assert result.reason == 'not_2d_supported_free'


def test_floor_and_overhead_caution_cannot_be_overridden():
    terrain, support = _fixture()
    caution = _change_cell(terrain, 'traversal_cost', value=60)
    caution = _change_cell(caution, 'roughness_m', value=1.0)
    # Saved map roughness may be caused by overhead returns, but can also be
    # a low-clearance obstacle; the current artifact cannot distinguish them.
    assert _check(caution, support).reason == (
        'terrain_caution_requires_review')
    steep = _change_cell(caution, 'slope_deg', value=12.0)
    assert _check(steep, support).reason == (
        'terrain_caution_requires_review')
    step = _change_cell(caution, 'step_height_m', value=0.08)
    assert _check(step, support).reason == (
        'terrain_caution_requires_review')


def test_malformed_or_low_quality_layers_reject():
    terrain, support = _fixture()
    assert _check(
        _change_cell(terrain, 'confidence', value=0.34), support
    ).reason == 'low_confidence'
    assert _check(
        _change_cell(terrain, 'elevation', value=np.nan), support
    ).reason == 'nonfinite_elevation'
    assert _check(
        _change_cell(terrain, 'slope_deg', value=np.nan), support
    ).reason == 'nonfinite_slope_deg'
    assert _check(
        terrain, support, minimum_confidence=0.10
    ).reason == 'invalid_input'
    assert _check(
        terrain, replace(support, free_mask=np.full(40000, -1))
    ).reason == 'invalid_input'


def test_rotated_footprint_checks_terrain_and_support_cells():
    terrain, support = _fixture()
    result = _check(terrain, support, pose_yaw=math.pi / 4)
    assert result.safe
    bad = _change_cell(terrain, 'traversal_cost', x=0.75, y=0.75,
                       value=-1)
    assert _check(bad, support, pose_yaw=math.pi / 4).reason == (
        'terrain_unknown_or_blocked')
