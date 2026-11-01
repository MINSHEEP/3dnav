#!/usr/bin/env python3
"""Pure, conservative saved-ground precheck for a restricted flat site.

This does not establish that the live space is free. In particular, a saved
map cannot reveal a new hole or object inside a lidar's unobserved near field.
The caller must independently verify the map artifact, localization and live
obstacle sensing before using a successful result to authorize motion.
"""

from dataclasses import dataclass
import math

import numpy as np

from .terrain_grid import GroundSupportGrid, TerrainGridData


# This rectangle includes the robot body and a short forward lookahead.
# Each grid is additionally inflated by half its cell diagonal while sampled.
REAR_M = 0.30
FRONT_M = 1.30
HALF_WIDTH_M = 0.30
MINIMUM_CONFIDENCE = 0.35
MAXIMUM_SLOPE_DEG = 12.0
MAXIMUM_STEP_M = 0.08


@dataclass(frozen=True)
class MappedGroundCheck:
    safe: bool
    reason: str
    terrain_cells_checked: int = 0
    support_cells_checked: int = 0
    caution_cells: int = 0
    failing_terrain_cell: tuple | None = None  # (row, column)


def _numeric_geometry(resolution, width, height, origin_x, origin_y, yaw=0.0):
    resolution = float(resolution)
    width = int(width)
    height = int(height)
    origin_x = float(origin_x)
    origin_y = float(origin_y)
    yaw = float(yaw)
    if not all(math.isfinite(value) for value in (
            resolution, origin_x, origin_y, yaw)):
        raise ValueError('nonfinite geometry')
    if resolution <= 0 or width <= 0 or height <= 0:
        raise ValueError('nonpositive geometry')
    return resolution, width, height, origin_x, origin_y, yaw


def _covered_cells(geometry, pose):
    """Overapproximate all grid cells touched by the body/lookahead box.

    A cell whose centre is up to half its diagonal outside the physical box
    might intersect it, so it is checked as well. Expanded corners must all
    lie inside the grid; map-edge uncertainty therefore fails closed.
    """
    resolution, width, height, origin_x, origin_y, grid_yaw = geometry
    pose_x, pose_y, pose_yaw = pose
    radius = resolution * math.sqrt(2.0) / 2.0
    rear = REAR_M + radius
    front = FRONT_M + radius
    half_width = HALF_WIDTH_M + radius

    robot_x = np.asarray([-rear, -rear, front, front])
    robot_y = np.asarray([-half_width, half_width, -half_width, half_width])
    pose_cosine, pose_sine = math.cos(pose_yaw), math.sin(pose_yaw)
    world_x = pose_x + pose_cosine * robot_x - pose_sine * robot_y
    world_y = pose_y + pose_sine * robot_x + pose_cosine * robot_y

    grid_cosine, grid_sine = math.cos(grid_yaw), math.sin(grid_yaw)
    delta_x, delta_y = world_x - origin_x, world_y - origin_y
    grid_x = grid_cosine * delta_x + grid_sine * delta_y
    grid_y = -grid_sine * delta_x + grid_cosine * delta_y
    if (np.any(grid_x < 0.0) or np.any(grid_y < 0.0) or
            np.any(grid_x >= width * resolution) or
            np.any(grid_y >= height * resolution)):
        return None

    columns = np.arange(
        int(math.floor(float(np.min(grid_x)) / resolution)),
        int(math.ceil(float(np.max(grid_x)) / resolution)),
        dtype=np.int64,
    )
    rows = np.arange(
        int(math.floor(float(np.min(grid_y)) / resolution)),
        int(math.ceil(float(np.max(grid_y)) / resolution)),
        dtype=np.int64,
    )
    if columns.size == 0 or rows.size == 0:
        return None
    row_mesh, column_mesh = np.meshgrid(rows, columns, indexing='ij')
    cell_x = (column_mesh + 0.5) * resolution
    cell_y = (row_mesh + 0.5) * resolution
    cell_world_x = origin_x + grid_cosine * cell_x - grid_sine * cell_y
    cell_world_y = origin_y + grid_sine * cell_x + grid_cosine * cell_y
    dx, dy = cell_world_x - pose_x, cell_world_y - pose_y
    body_x = pose_cosine * dx + pose_sine * dy
    body_y = -pose_sine * dx + pose_cosine * dy
    covered = (
        (body_x >= -rear) & (body_x <= front) &
        (np.abs(body_y) <= half_width)
    )
    return (
        row_mesh[covered], column_mesh[covered],
        cell_world_x[covered], cell_world_y[covered],
    )


def _first_failure(mask, rows, columns, reason, *, terrain_count,
                   support_count=0, caution_count=0):
    if not np.any(mask):
        return None
    index = int(np.flatnonzero(mask)[0])
    return MappedGroundCheck(
        False, reason,
        terrain_cells_checked=terrain_count,
        support_cells_checked=support_count,
        caution_cells=caution_count,
        failing_terrain_cell=(int(rows[index]), int(columns[index])),
    )


def check_mapped_ground(
        terrain: TerrainGridData,
        ground_support: GroundSupportGrid,
        *,
        expected_map_id: str,
        current_map_id: str,
        pose_x: float,
        pose_y: float,
        pose_yaw: float,
        minimum_confidence: float = MINIMUM_CONFIDENCE) -> MappedGroundCheck:
    """Reject unknown, unsupported or non-flat ground under/lookahead.

    Caution (cost 60), special, blocked and unknown cells are always rejected.
    Current saved-map roughness conflates high returns and potential obstacles,
    so it cannot safely justify a generic caution-cell override.

    ``expected_map_id`` should be a caller-verified content identity, not
    merely a filename. Exact equality here does not authenticate an artifact.
    Invalid or malformed input returns unsafe rather than raising.
    """
    if (not isinstance(expected_map_id, str) or
            not isinstance(current_map_id, str) or
            not expected_map_id.strip() or
            expected_map_id != current_map_id):
        return MappedGroundCheck(False, 'map_mismatch')

    try:
        pose = (float(pose_x), float(pose_y), float(pose_yaw))
        minimum_confidence = float(minimum_confidence)
        if (not all(math.isfinite(value) for value in pose) or
                not math.isfinite(minimum_confidence) or
                not MINIMUM_CONFIDENCE <= minimum_confidence <= 1.0):
            return MappedGroundCheck(False, 'invalid_input')
        terrain_geometry = _numeric_geometry(
            terrain.resolution, terrain.width, terrain.height,
            terrain.origin_x, terrain.origin_y,
        )
        support_geometry = _numeric_geometry(
            ground_support.resolution, ground_support.width,
            ground_support.height, ground_support.origin_x,
            ground_support.origin_y, ground_support.origin_yaw,
        )
        terrain_count = terrain_geometry[1] * terrain_geometry[2]
        support_count = support_geometry[1] * support_geometry[2]
        layers = {
            name: np.asarray(getattr(terrain, name)).reshape(-1)
            for name in (
                'elevation', 'slope_deg', 'roughness_m', 'step_height_m',
                'confidence', 'traversal_cost',
            )
        }
        if (any(values.size != terrain_count for values in layers.values()) or
                layers['traversal_cost'].dtype.kind not in 'iu'):
            return MappedGroundCheck(False, 'invalid_input')
        free_mask = np.asarray(ground_support.free_mask).reshape(-1)
        if (free_mask.size != support_count or
                free_mask.dtype.kind not in 'biu' or
                np.any((free_mask != 0) & (free_mask != 1))):
            return MappedGroundCheck(False, 'invalid_input')
        free_mask = free_mask.astype(bool, copy=False).reshape(
            support_geometry[2], support_geometry[1])

        sampled = _covered_cells(terrain_geometry, pose)
        support_sampled = _covered_cells(support_geometry, pose)
        if sampled is None or support_sampled is None:
            return MappedGroundCheck(False, 'outside_map')
        rows, columns, world_x, world_y = sampled
        support_rows, support_columns, _, _ = support_sampled
        if rows.size == 0 or support_rows.size == 0:
            return MappedGroundCheck(False, 'outside_map')
        index = rows * terrain_geometry[1] + columns
        terrain_cells = int(rows.size)
        support_cells = int(support_rows.size)

        cost = layers['traversal_cost'][index]
        failure = _first_failure(
            (cost != 0) & (cost != 60), rows, columns, 'terrain_unknown_or_blocked',
            terrain_count=terrain_cells, support_count=support_cells)
        if failure is not None:
            return failure
        caution = cost == 60
        caution_count = int(np.count_nonzero(caution))
        if caution_count:
            return _first_failure(
                caution, rows, columns, 'terrain_caution_requires_review',
                terrain_count=terrain_cells, support_count=support_cells,
                caution_count=caution_count)

        for name in ('elevation', 'slope_deg', 'roughness_m',
                     'step_height_m', 'confidence'):
            values = layers[name][index]
            failure = _first_failure(
                ~np.isfinite(values), rows, columns,
                'nonfinite_' + name,
                terrain_count=terrain_cells, support_count=support_cells,
                caution_count=caution_count)
            if failure is not None:
                return failure
        slope = layers['slope_deg'][index]
        step = layers['step_height_m'][index]
        roughness = layers['roughness_m'][index]
        confidence = layers['confidence'][index]
        for condition, reason in (
                ((confidence < minimum_confidence) | (confidence > 1.0),
                 'low_confidence'),
                ((slope < 0.0) | (slope >= MAXIMUM_SLOPE_DEG),
                 'unsafe_slope'),
                ((step < 0.0) | (step >= MAXIMUM_STEP_M),
                 'unsafe_step'),
                (roughness < 0.0, 'invalid_roughness'),
        ):
            failure = _first_failure(
                condition, rows, columns, reason,
                terrain_count=terrain_cells, support_count=support_cells,
                caution_count=caution_count)
            if failure is not None:
                return failure

        # Require both every checked terrain-cell centre and every fine 2-D
        # support cell touched by the lookahead to be explicitly free.
        _, support_width, support_height, sx, sy, syaw = support_geometry
        dx, dy = world_x - sx, world_y - sy
        cosine, sine = math.cos(syaw), math.sin(syaw)
        support_x = cosine * dx + sine * dy
        support_y = -sine * dx + cosine * dy
        support_column = np.floor(
            support_x / support_geometry[0]).astype(np.int64)
        support_row = np.floor(
            support_y / support_geometry[0]).astype(np.int64)
        inside = (
            (support_column >= 0) & (support_column < support_width) &
            (support_row >= 0) & (support_row < support_height)
        )
        terrain_center_free = np.zeros(rows.size, dtype=bool)
        terrain_center_free[inside] = free_mask[
            support_row[inside], support_column[inside]]
        failure = _first_failure(
            ~terrain_center_free, rows, columns, 'not_2d_supported_free',
            terrain_count=terrain_cells, support_count=support_cells,
            caution_count=caution_count)
        if failure is not None:
            return failure
        if not np.all(free_mask[support_rows, support_columns]):
            return MappedGroundCheck(
                False, 'not_2d_supported_free',
                terrain_cells_checked=terrain_cells,
                support_cells_checked=support_cells,
                caution_cells=caution_count)
        return MappedGroundCheck(
            True, 'known_flat_ground',
            terrain_cells_checked=terrain_cells,
            support_cells_checked=support_cells,
            caution_cells=caution_count)
    except (TypeError, ValueError, OverflowError, AttributeError, IndexError):
        return MappedGroundCheck(False, 'invalid_input')
