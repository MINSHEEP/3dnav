#!/usr/bin/env python3
"""Pure regression tests for the CPU-bounded 2.5D terrain grid."""

from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.terrain_grid import (  # noqa: E402
    BLOCKED_COST,
    CAUTION_COST,
    FREE_COST,
    GroundSupportGrid,
    SPECIAL_COST,
    UNKNOWN_COST,
    build_terrain_grid,
)


def test_flat_surface_is_free_and_missing_cells_stay_unknown():
    points = np.asarray([
        (x, y, 0.01 * ((x + y) % 2))
        for x in np.arange(0.0, 2.1, 0.2)
        for y in np.arange(0.0, 2.1, 0.2)
    ], dtype=np.float32)
    grid = build_terrain_grid(points, resolution=0.25)

    assert np.count_nonzero(grid.traversal_cost == FREE_COST) > 20
    assert np.count_nonzero(grid.traversal_cost == UNKNOWN_COST) > 0
    assert np.nanmax(grid.slope_deg) < 12.0


def test_large_step_is_blocked():
    points = []
    for x in np.arange(0.0, 2.1, 0.2):
        for y in np.arange(0.0, 2.1, 0.2):
            points.append((x, y, 0.0 if x < 1.0 else 0.24))
    grid = build_terrain_grid(np.asarray(points), resolution=0.25)

    assert np.count_nonzero(grid.traversal_cost == BLOCKED_COST) > 0
    assert np.nanmax(grid.step_height_m) >= 0.20


def test_resolution_adapts_to_cell_budget():
    points = np.asarray([
        (0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
        (0.0, 100.0, 0.0), (100.0, 100.0, 0.0),
    ], dtype=np.float32)
    grid = build_terrain_grid(
        points, resolution=0.1, maximum_cells=10_000)

    assert grid.resolution >= 1.0
    assert grid.width * grid.height < 11_000


def test_bidirectional_low_step_limit_is_automatic_but_excess_is_special():
    def stepped(height):
        return np.asarray([
            (x, y, 0.0 if x < 1.0 else height)
            for x in np.arange(0.0, 2.1, 0.2)
            for y in np.arange(0.0, 2.1, 0.2)
        ], dtype=np.float32)

    allowed = build_terrain_grid(
        stepped(0.12), resolution=0.25, special_step_m=0.12)
    rejected = build_terrain_grid(
        stepped(0.14), resolution=0.25, special_step_m=0.12)

    assert np.count_nonzero(allowed.traversal_cost == SPECIAL_COST) == 0
    assert np.count_nonzero(rejected.traversal_cost == SPECIAL_COST) > 0


def test_continuous_ramp_keeps_real_slope_over_long_baseline():
    def ramp(angle_deg):
        gradient = np.tan(np.radians(angle_deg))
        return np.asarray([
            (x, y, gradient * x)
            for x in np.arange(0.0, 3.01, 0.05)
            for y in np.arange(0.0, 1.01, 0.10)
        ], dtype=np.float32)

    allowed = build_terrain_grid(
        ramp(18.0), resolution=0.25, special_slope_deg=18.0,
        special_step_m=0.12)
    rejected = build_terrain_grid(
        ramp(20.0), resolution=0.25, special_slope_deg=18.0,
        special_step_m=0.12)

    assert np.count_nonzero(allowed.traversal_cost == SPECIAL_COST) == 0
    assert np.count_nonzero(rejected.traversal_cost == SPECIAL_COST) > 0


def test_free_space_support_reconstructs_ground_below_overhead_returns():
    points = []
    for x in np.arange(0.0, 3.01, 0.20):
        for y in np.arange(0.0, 3.01, 0.20):
            z = 3.0 if 1.0 <= x < 2.0 and 1.0 <= y < 2.0 else 0.0
            points.append((x, y, z))
    support = GroundSupportGrid(
        resolution=0.10,
        width=31,
        height=31,
        origin_x=0.0,
        origin_y=0.0,
        free_mask=np.ones((31, 31), dtype=bool),
    )

    uncorrected = build_terrain_grid(
        np.asarray(points), resolution=0.25)
    corrected = build_terrain_grid(
        np.asarray(points), resolution=0.25,
        ground_support=support,
        overhead_search_radius_m=1.0,
        overhead_min_clearance_m=0.75,
    )

    row = int(np.floor((1.5 - corrected.origin_y) / corrected.resolution))
    column = int(np.floor((1.5 - corrected.origin_x) / corrected.resolution))
    index = row * corrected.width + column
    assert uncorrected.elevation[index] > 2.90
    assert corrected.elevation[index] < 0.10
    assert corrected.traversal_cost[index] == UNKNOWN_COST
    assert corrected.ground_measured[index] == 0


def test_ground_reconstruction_never_fills_nonfree_or_unobserved_large_areas():
    points = np.asarray([
        (0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
        (0.0, 0.2, 0.0), (2.0, 2.0, 3.0),
    ], dtype=np.float32)
    support_mask = np.zeros((25, 25), dtype=bool)
    support_mask[:5, :5] = True
    support = GroundSupportGrid(
        resolution=0.10, width=25, height=25,
        origin_x=0.0, origin_y=0.0, free_mask=support_mask)

    grid = build_terrain_grid(points, resolution=0.25, ground_support=support)
    row = int(np.floor((2.0 - grid.origin_y) / grid.resolution))
    column = int(np.floor((2.0 - grid.origin_x) / grid.resolution))
    index = row * grid.width + column

    assert grid.elevation[index] == 3.0
    assert grid.traversal_cost[index] == UNKNOWN_COST
