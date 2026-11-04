"""Fuse a validated 2-D map with a 2.5-D terrain grid for Nav2.

The public interface is deliberately pure: callers provide two row-major
grids and receive one row-major occupancy grid.  ROS messages, subscriptions
and lifecycle timing stay in the adapter node.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .navigation_mode import MAP_2D, normalize_navigation_mode


UNKNOWN = -1
LETHAL = 100


@dataclass(frozen=True)
class GridGeometry:
    """Geometry required to locate cell centres in a shared world frame."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def validate(self) -> None:
        values = (
            self.resolution, self.origin_x, self.origin_y, self.origin_yaw)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('grid geometry must contain finite numbers')
        if self.resolution <= 0.0 or self.width <= 0 or self.height <= 0:
            raise ValueError('grid resolution, width and height must be positive')


@dataclass(frozen=True)
class TerrainCostmapResult:
    """A fused Nav2 grid and diagnostics useful to operators and tests."""

    data: np.ndarray
    terrain_covered_cells: int
    unknown_cells: int
    caution_cells: int
    special_cells: int
    blocked_cells: int


def select_navigation_costmap(
        mode,
        base_data,
        base_geometry: GridGeometry,
        terrain_cost=None,
        terrain_geometry: GridGeometry | None = None,
        *,
        occupied_threshold: int = 65,
        special_cost: int = 90) -> TerrainCostmapResult:
    """Select raw-map or terrain-fused policy behind one stable Nav2 seam."""
    canonical = normalize_navigation_mode(mode)
    if canonical != MAP_2D:
        if terrain_cost is None or terrain_geometry is None:
            raise ValueError('terrain mode requires a matching terrain grid')
        return fuse_terrain_costmap(
            base_data,
            base_geometry,
            terrain_cost,
            terrain_geometry,
            occupied_threshold=occupied_threshold,
            special_cost=special_cost,
        )

    base_geometry.validate()
    occupied_threshold = int(occupied_threshold)
    if not 1 <= occupied_threshold <= LETHAL:
        raise ValueError('occupied_threshold must be in [1, 100]')
    base = np.asarray(base_data, dtype=np.int16).reshape(-1)
    expected = base_geometry.width * base_geometry.height
    if base.size != expected:
        raise ValueError(f'base grid has {base.size} cells, expected {expected}')
    if np.any((base < UNKNOWN) | (base > LETHAL)):
        raise ValueError('base occupancy values must be in [-1, 100]')
    output = base.astype(np.int8, copy=True)
    return TerrainCostmapResult(
        data=output,
        terrain_covered_cells=0,
        unknown_cells=int(np.count_nonzero(output == UNKNOWN)),
        caution_cells=0,
        special_cells=0,
        blocked_cells=int(np.count_nonzero(output >= occupied_threshold)),
    )


def fuse_terrain_costmap(
        base_data,
        base_geometry: GridGeometry,
        terrain_cost,
        terrain_geometry: GridGeometry,
        *,
        occupied_threshold: int = 65,
        special_cost: int = 90) -> TerrainCostmapResult:
    """Return a conservative occupancy grid for Nav2's global StaticLayer.

    The result keeps the 2-D map geometry so all existing map/odom transforms
    remain valid.  The terrain grid is sampled at each base-cell centre.
    A cell is usable only when both sources know it.  The 2-D map retains veto
    power over walls, edits and unknown space; terrain retains veto power over
    cliffs, excessive steps/slopes and missing 3-D coverage.
    """

    base_geometry.validate()
    terrain_geometry.validate()
    occupied_threshold = int(occupied_threshold)
    special_cost = int(special_cost)
    if not 1 <= occupied_threshold <= LETHAL:
        raise ValueError('occupied_threshold must be in [1, 100]')
    if not 1 <= special_cost <= LETHAL:
        raise ValueError('special_cost must be in [1, 100]')

    base = np.asarray(base_data, dtype=np.int16).reshape(-1)
    terrain = np.asarray(terrain_cost, dtype=np.int16).reshape(-1)
    expected_base = base_geometry.width * base_geometry.height
    expected_terrain = terrain_geometry.width * terrain_geometry.height
    if base.size != expected_base:
        raise ValueError(
            f'base grid has {base.size} cells, expected {expected_base}')
    if terrain.size != expected_terrain:
        raise ValueError(
            f'terrain grid has {terrain.size} cells, expected {expected_terrain}')
    if np.any((base < UNKNOWN) | (base > LETHAL)):
        raise ValueError('base occupancy values must be in [-1, 100]')
    if np.any((terrain < UNKNOWN) | (terrain > LETHAL)):
        raise ValueError('terrain costs must be in [-1, 100]')

    rows, columns = np.indices(
        (base_geometry.height, base_geometry.width), dtype=np.float64)
    local_x = (columns + 0.5) * base_geometry.resolution
    local_y = (rows + 0.5) * base_geometry.resolution
    cosine = math.cos(base_geometry.origin_yaw)
    sine = math.sin(base_geometry.origin_yaw)
    world_x = base_geometry.origin_x + cosine * local_x - sine * local_y
    world_y = base_geometry.origin_y + sine * local_x + cosine * local_y

    # TerrainGrid is axis-aligned in its frame today.  Rejecting a non-zero
    # yaw keeps the interface honest until the message carries an orientation.
    if not math.isclose(
            terrain_geometry.origin_yaw, 0.0, abs_tol=1.0e-9):
        raise ValueError('rotated terrain grids are not supported')
    terrain_columns = np.floor(
        (world_x - terrain_geometry.origin_x) /
        terrain_geometry.resolution).astype(np.int64)
    terrain_rows = np.floor(
        (world_y - terrain_geometry.origin_y) /
        terrain_geometry.resolution).astype(np.int64)
    covered = (
        (terrain_columns >= 0) &
        (terrain_columns < terrain_geometry.width) &
        (terrain_rows >= 0) &
        (terrain_rows < terrain_geometry.height))

    sampled = np.full(base.shape, UNKNOWN, dtype=np.int16)
    covered_flat = covered.reshape(-1)
    terrain_indices = (
        terrain_rows * terrain_geometry.width + terrain_columns).reshape(-1)
    sampled[covered_flat] = terrain[terrain_indices[covered_flat]]

    # Terrain's SPECIAL_COST is policy, not a ROS occupancy convention.
    # Production defaults map it to lethal so ordinary Nav2 cannot bypass the
    # calibrated connector state machine. Tests may choose a lower value to
    # inspect the numeric fusion behavior.
    source_special = sampled == 90
    sampled[source_special] = special_cost
    output = np.full(base.shape, UNKNOWN, dtype=np.int16)
    base_known_free = (base >= 0) & (base < occupied_threshold)
    terrain_known = sampled >= 0
    usable = base_known_free & terrain_known
    output[usable] = np.maximum(base[usable], sampled[usable])
    output[base >= occupied_threshold] = LETHAL
    output[sampled >= LETHAL] = LETHAL
    output = output.astype(np.int8)

    return TerrainCostmapResult(
        data=output,
        terrain_covered_cells=int(np.count_nonzero(covered_flat)),
        unknown_cells=int(np.count_nonzero(output == UNKNOWN)),
        caution_cells=int(np.count_nonzero((output >= 50) & (output < special_cost))),
        special_cells=int(np.count_nonzero(source_special & usable)),
        blocked_cells=int(np.count_nonzero(output == LETHAL)),
    )
