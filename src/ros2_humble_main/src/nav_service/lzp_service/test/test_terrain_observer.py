#!/usr/bin/env python3
"""Pure tests for bounded live terrain estimation in front of the dog."""

from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.terrain_observer import (  # noqa: E402
    corridor_return_diagnostics, estimate_local_terrain,
)
from lzp_service.terrain_navigation import TerrainCapabilityModel  # noqa: E402


def surface(slope=0.0):
    return np.asarray([
        (x, y, slope * x)
        for x in np.arange(0.1, 1.31, 0.04)
        for y in np.arange(-0.45, 0.46, 0.04)
    ], dtype=np.float32)


def test_flat_corridor_is_high_confidence_and_traversable():
    result = estimate_local_terrain(surface(), robot_x=0.0, robot_y=0.0, yaw=0.0)
    assert result.terrain_type == 'flat'
    assert abs(result.slope_deg) < 1.0
    assert result.confidence >= 0.70
    assert not result.negative_obstacle


def test_ramp_is_classified_from_longitudinal_plane():
    result = estimate_local_terrain(
        surface(slope=0.20), robot_x=0.0, robot_y=0.0, yaw=0.0)
    assert result.terrain_type == 'ramp'
    assert 10.0 < result.slope_deg < 13.0


def test_central_vertical_cluster_blocks_connector():
    obstacle = np.asarray([
        (x, y, z)
        for x in np.arange(0.45, 0.66, 0.03)
        for y in np.arange(-0.12, 0.13, 0.03)
        for z in np.arange(0.10, 0.31, 0.04)
    ], dtype=np.float32)
    result = estimate_local_terrain(
        np.vstack((surface(), obstacle)),
        robot_x=0.0, robot_y=0.0, yaw=0.0)
    assert result.terrain_type == 'blocked'


def test_sparse_cloud_fails_closed_as_unknown():
    result = estimate_local_terrain(
        np.asarray([(0.5, 0.0, 0.0), (0.6, 0.1, 0.0)], dtype=np.float32),
        robot_x=0.0, robot_y=0.0, yaw=0.0)
    assert result.terrain_type == 'unknown'
    assert result.confidence < 0.5


def test_flat_far_returns_do_not_unlock_unseen_near_field():
    # Recorded MID360 geometry sees a flat floor only ~1 m ahead.
    points = np.asarray([
        (x, y, 0.0)
        for x in np.arange(1.04, 1.30, 0.01)
        for y in np.arange(-0.40, 0.41, 0.03)
    ], dtype=np.float32)
    result = estimate_local_terrain(
        points, robot_x=0.0, robot_y=0.0, yaw=0.0)
    returns = corridor_return_diagnostics(
        points, robot_x=0.0, robot_y=0.0, yaw=0.0)

    assert result.terrain_type == 'flat'
    assert returns['corridor_return_count'] > 0
    assert returns['near_field_return_count'] == 0
    assert returns['nearest_forward_return_m'] >= 1.0
    assert not TerrainCapabilityModel().assess(result).traversable
