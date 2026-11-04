#!/usr/bin/env python3
"""Pure tests for lifting a Nav2 global path onto saved terrain."""

from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.terrain_grid import TerrainGridData  # noqa: E402
from lzp_service.terrain_path import (  # noqa: E402
    elevate_path_points,
    path_length_m,
    simplify_path_indices,
)


def _terrain():
    return TerrainGridData(
        resolution=1.0,
        width=2,
        height=2,
        origin_x=0.0,
        origin_y=0.0,
        elevation=np.asarray([0.0, 1.0, 2.0, np.nan], dtype=np.float32),
        slope_deg=np.zeros(4, dtype=np.float32),
        roughness_m=np.zeros(4, dtype=np.float32),
        step_height_m=np.zeros(4, dtype=np.float32),
        confidence=np.asarray([1.0, 1.0, 1.0, 0.0], dtype=np.float32),
        traversal_cost=np.asarray([0, 60, 0, -1], dtype=np.int8),
    )


def test_path_uses_cell_elevation_and_keeps_unknown_pose_height():
    source = np.asarray([
        [0.2, 0.2, 0.0],
        [1.2, 0.2, 0.0],
        [0.2, 1.2, 0.0],
        [1.2, 1.2, 0.35],
        [4.0, 4.0, 0.5],
    ], dtype=np.float64)
    result = elevate_path_points(source, _terrain(), clearance_m=0.08)

    np.testing.assert_allclose(result.points[:3, 2], [0.08, 1.08, 2.08])
    np.testing.assert_allclose(result.points[3:, 2], [0.35, 0.5])
    np.testing.assert_allclose(result.points[:, :2], source[:, :2])
    assert result.elevated_count == 3
    assert result.unknown_count == 2


def test_path_rejects_non_finite_coordinates():
    try:
        elevate_path_points(
            np.asarray([[0.0, np.nan, 0.0]]), _terrain())
    except ValueError as error:
        assert 'finite' in str(error)
    else:
        raise AssertionError('non-finite path coordinate was accepted')


def test_display_simplification_preserves_endpoints_and_a_turn():
    source = np.asarray([
        [0.00, 0.00, 0.0],
        [0.04, 0.00, 0.0],
        [0.08, 0.00, 0.0],
        [0.08, 0.04, 0.0],
        [0.08, 0.08, 0.0],
    ], dtype=np.float64)

    indices = simplify_path_indices(
        source, minimum_spacing_m=0.10, maximum_points=4)

    assert indices.tolist() == [0, 2, 4]
    np.testing.assert_allclose(source[indices[1]], [0.08, 0.0, 0.0])


def test_display_limit_and_source_path_length_are_independent():
    source = np.column_stack((
        np.linspace(0.0, 9.9, 100),
        np.zeros(100),
        np.zeros(100),
    ))

    indices = simplify_path_indices(
        source, minimum_spacing_m=0.0, maximum_points=5)

    assert indices.tolist() == [0, 25, 50, 74, 99]
    assert path_length_m(source) == 9.9


def test_display_point_limit_does_not_drop_a_major_turn():
    source = np.asarray([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [4.0, 1.0, 0.0],
        [4.0, 2.0, 0.0],
        [4.0, 3.0, 0.0],
        [4.0, 4.0, 0.0],
        [4.0, 5.0, 0.0],
        [4.0, 6.0, 0.0],
    ], dtype=np.float64)

    indices = simplify_path_indices(
        source, minimum_spacing_m=0.0, maximum_points=4)

    assert len(indices) == 4
    assert indices[0] == 0
    assert indices[-1] == 10
    assert 4 in indices
