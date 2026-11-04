#!/usr/bin/env python3
"""CPU-bounded conversion from persistent 3D points to a 2.5D terrain grid."""

from dataclasses import dataclass
import math

import numpy as np


UNKNOWN_COST = -1
FREE_COST = 0
CAUTION_COST = 60
SPECIAL_COST = 90
BLOCKED_COST = 100


@dataclass(frozen=True)
class TerrainGridData:
    """Dense row-major terrain layers with NaN for unobserved cells."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    elevation: np.ndarray
    slope_deg: np.ndarray
    roughness_m: np.ndarray
    step_height_m: np.ndarray
    confidence: np.ndarray
    traversal_cost: np.ndarray
    ground_measured: np.ndarray | None = None


@dataclass(frozen=True)
class GroundSupportGrid:
    """A saved 2-D free-space mask used to reject overhead returns.

    The mask uses ROS OccupancyGrid row order (origin at the lower-left).
    Cells not explicitly marked free never authorize reconstruction.
    """

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    free_mask: np.ndarray
    origin_yaw: float = 0.0

    def validate(self):
        values = (
            self.resolution, self.origin_x, self.origin_y,
            self.origin_yaw,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('ground support geometry must be finite')
        if self.resolution <= 0.0 or self.width <= 0 or self.height <= 0:
            raise ValueError('ground support geometry must be positive')
        mask = np.asarray(self.free_mask)
        if mask.size != self.width * self.height:
            raise ValueError('ground support mask length is invalid')
        return mask.astype(bool, copy=False).reshape(
            self.height, self.width)


def _adaptive_resolution(points, requested, maximum_cells):
    span_x = max(float(np.ptp(points[:, 0])), requested)
    span_y = max(float(np.ptp(points[:, 1])), requested)
    estimated = math.ceil(span_x / requested + 1) * math.ceil(
        span_y / requested + 1)
    if estimated <= maximum_cells:
        return requested
    return max(requested, math.sqrt(span_x * span_y / maximum_cells))


def _sample_ground_support(
        support, *, width, height, resolution, origin_x, origin_y):
    """Sample an explicit 2-D free mask at terrain-cell centres."""

    source = support.validate()
    rows, columns = np.indices((height, width), dtype=np.float64)
    world_x = origin_x + (columns + 0.5) * resolution
    world_y = origin_y + (rows + 0.5) * resolution
    delta_x = world_x - support.origin_x
    delta_y = world_y - support.origin_y
    cosine = math.cos(support.origin_yaw)
    sine = math.sin(support.origin_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    source_columns = np.floor(
        local_x / support.resolution).astype(np.int64)
    source_rows = np.floor(
        local_y / support.resolution).astype(np.int64)
    inside = (
        (source_columns >= 0) & (source_columns < support.width) &
        (source_rows >= 0) & (source_rows < support.height)
    )
    sampled = np.zeros((height, width), dtype=bool)
    sampled[inside] = source[
        source_rows[inside], source_columns[inside]]
    return sampled


def _reconstruct_ground_below_overhead(
        elevation, roughness, confidence, support_free, *,
        resolution, search_radius_m, minimum_clearance_m,
        roughness_caution_m, return_mask=False):
    """Replace unsupported high returns with a nearby lower ground envelope.

    This is deliberately asymmetric: it only lowers a return when the saved
    2-D map says the cell is free and a robust set of nearby lower returns
    agrees on a surface at least ``minimum_clearance_m`` below it.  It never
    invents elevation for an unobserved cell or outside explicit free space.
    """

    radius = max(1, int(math.ceil(search_radius_m / resolution)))
    valid_free = np.isfinite(elevation) & support_free
    corrected = elevation.copy()
    inferred = np.zeros(elevation.shape, dtype=bool)
    row_indices, column_indices = np.nonzero(valid_free)
    for row, column in zip(row_indices.tolist(), column_indices.tolist()):
        row_start = max(0, row - radius)
        row_end = min(elevation.shape[0], row + radius + 1)
        column_start = max(0, column - radius)
        column_end = min(elevation.shape[1], column + radius + 1)
        local_elevation = elevation[
            row_start:row_end, column_start:column_end]
        local_free = support_free[
            row_start:row_end, column_start:column_end]
        samples = local_elevation[
            np.isfinite(local_elevation) & local_free]
        if samples.size < 8:
            continue
        lower_quartile = float(np.percentile(samples, 25.0))
        lower_samples = samples[
            samples <= lower_quartile + max(0.20, resolution)]
        if lower_samples.size < 4:
            continue
        ground_elevation = float(np.median(lower_samples))
        if elevation[row, column] - ground_elevation < minimum_clearance_m:
            continue
        corrected[row, column] = ground_elevation
        inferred[row, column] = True

    if np.any(inferred):
        roughness[inferred] = np.maximum(
            np.nan_to_num(roughness[inferred], nan=0.0),
            roughness_caution_m,
        )
        confidence[inferred] = np.maximum(confidence[inferred], 0.5)
    return (corrected, inferred) if return_mask else corrected


def summarize_terrain_quality(grid, ground_support=None):
    """Return bounded, JSON-safe coverage and connectivity diagnostics."""

    costs = np.asarray(grid.traversal_cost, dtype=np.int16).reshape(
        grid.height, grid.width)
    if ground_support is None:
        support = np.ones(costs.shape, dtype=bool)
    else:
        support = _sample_ground_support(
            ground_support,
            width=grid.width,
            height=grid.height,
            resolution=grid.resolution,
            origin_x=grid.origin_x,
            origin_y=grid.origin_y,
        )
    supported_cells = int(np.count_nonzero(support))
    known = support & (costs >= 0)
    traversable = support & (costs >= 0) & (costs < BLOCKED_COST)
    blocked = support & (costs == BLOCKED_COST)
    labels = np.zeros(costs.shape, dtype=np.int32)
    component_sizes = []
    component = 0
    for row, column in zip(*np.nonzero(traversable)):
        if labels[row, column] != 0:
            continue
        component += 1
        labels[row, column] = component
        stack = [(int(row), int(column))]
        size = 0
        while stack:
            current_row, current_column = stack.pop()
            size += 1
            for delta_row in (-1, 0, 1):
                for delta_column in (-1, 0, 1):
                    if delta_row == 0 and delta_column == 0:
                        continue
                    next_row = current_row + delta_row
                    next_column = current_column + delta_column
                    if not (
                        0 <= next_row < grid.height
                        and 0 <= next_column < grid.width
                    ):
                        continue
                    if (
                        traversable[next_row, next_column]
                        and labels[next_row, next_column] == 0
                    ):
                        labels[next_row, next_column] = component
                        stack.append((next_row, next_column))
        component_sizes.append(size)
    known_cells = int(np.count_nonzero(known))
    traversable_cells = int(np.count_nonzero(traversable))
    blocked_cells = int(np.count_nonzero(blocked))
    largest_component = max(component_sizes, default=0)
    coverage = (
        float(known_cells) / supported_cells if supported_cells else 0.0)
    largest_ratio = (
        float(largest_component) / traversable_cells
        if traversable_cells else 0.0)
    warnings = []
    if coverage < 0.20:
        warnings.append('low_terrain_coverage')
    if traversable_cells and largest_ratio < 0.70:
        warnings.append('fragmented_traversable_area')
    if known_cells and blocked_cells / known_cells > 0.40:
        warnings.append('high_blocked_ratio')
    return {
        'state': 'warning' if warnings else 'ready',
        'supported_free_cells': supported_cells,
        'known_supported_cells': known_cells,
        'traversable_supported_cells': traversable_cells,
        'blocked_supported_cells': blocked_cells,
        'coverage_ratio': round(coverage, 6),
        'component_count': len(component_sizes),
        'largest_component_cells': int(largest_component),
        'largest_component_ratio': round(largest_ratio, 6),
        'warnings': warnings,
    }


def build_terrain_grid(
        points,
        resolution=0.25,
        maximum_cells=400_000,
        caution_slope_deg=12.0,
        special_slope_deg=18.0,
        blocked_slope_deg=28.0,
        caution_step_m=0.08,
        special_step_m=0.12,
        blocked_step_m=0.18,
        roughness_caution_m=0.05,
        minimum_confidence=0.35,
        ground_support=None,
        overhead_search_radius_m=1.0,
        overhead_min_clearance_m=0.75):
    """Build conservative elevation and traversability layers.

    Elevation starts with the lowest persistent return in each XY cell. When a
    saved 2-D free-space mask is supplied, high returns that float above a
    locally consistent lower surface are treated as roof/canopy returns and
    lowered to that surface. Missing cells remain unknown.
    """
    values = np.asarray(points, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('points must be an N x 3 array')
    finite = values[np.isfinite(values).all(axis=1)]
    if finite.shape[0] < 3:
        raise ValueError('at least three finite 3D points are required')
    resolution = float(resolution)
    maximum_cells = int(maximum_cells)
    if resolution <= 0 or maximum_cells < 100:
        raise ValueError('resolution and maximum_cells must be positive')
    resolution = _adaptive_resolution(finite, resolution, maximum_cells)
    padding = resolution
    origin_x = math.floor((float(np.min(finite[:, 0])) - padding) / resolution) * resolution
    origin_y = math.floor((float(np.min(finite[:, 1])) - padding) / resolution) * resolution
    x_index = np.floor((finite[:, 0] - origin_x) / resolution).astype(np.int64)
    y_index = np.floor((finite[:, 1] - origin_y) / resolution).astype(np.int64)
    width = int(np.max(x_index)) + 2
    height = int(np.max(y_index)) + 2
    linear = y_index * width + x_index
    order = np.lexsort((finite[:, 2], linear))
    sorted_linear = linear[order]
    unique, first, counts = np.unique(
        sorted_linear, return_index=True, return_counts=True)
    sorted_z = finite[order, 2]
    low = sorted_z[first]
    # Only the lowest vertically connected cluster supplies floor height,
    # roughness and point-count evidence. Roof/wall returns do not strengthen
    # or roughen a measured floor in the same XY cell.
    ground_band = sorted_z <= np.repeat(low, counts) + 0.08
    ground_counts = np.add.reduceat(ground_band.astype(np.int32), first)
    ground_high = np.maximum.reduceat(np.where(ground_band, sorted_z, -np.inf), first)
    relative_z = sorted_z - np.repeat(low, counts)
    body_hits = np.add.reduceat(((relative_z >= .15) & (relative_z <= 1.10)).astype(np.int32), first)
    collision_cells = np.zeros((height, width), dtype=bool)
    ground_measured = np.zeros((height, width), dtype=np.uint8)

    shape = (height, width)
    elevation = np.full(shape, np.nan, dtype=np.float32)
    roughness = np.full(shape, np.nan, dtype=np.float32)
    confidence = np.zeros(shape, dtype=np.float32)
    rows, columns = np.divmod(unique, width)
    elevation[rows, columns] = low
    roughness[rows, columns] = ground_high - low
    confidence[rows, columns] = np.minimum(1.0, ground_counts.astype(np.float32) / 4.0)
    ground_measured[rows, columns] = (ground_counts >= 2).astype(np.uint8)
    collision_cells[rows, columns] = body_hits >= 3
    if ground_support is not None:
        search_radius = float(overhead_search_radius_m)
        minimum_clearance = float(overhead_min_clearance_m)
        if search_radius <= 0.0 or minimum_clearance <= 0.0:
            raise ValueError('overhead reconstruction limits must be positive')
        support_free = _sample_ground_support(
            ground_support,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        elevation, inferred = _reconstruct_ground_below_overhead(
            elevation,
            roughness,
            confidence,
            support_free,
            resolution=resolution,
            search_radius_m=search_radius,
            minimum_clearance_m=minimum_clearance,
            roughness_caution_m=roughness_caution_m, return_mask=True,
        )
        ground_measured[inferred] = 0
    valid = np.isfinite(elevation)

    slope = np.full(shape, np.nan, dtype=np.float32)
    step = np.full(shape, np.nan, dtype=np.float32)
    def neighbour_differences(offsets):
        result = []
        for delta_row, delta_column in offsets:
            shifted = np.roll(
                elevation, (delta_row, delta_column), axis=(0, 1))
            pair_valid = valid & np.roll(
                valid, (delta_row, delta_column), axis=(0, 1))
            if delta_row > 0:
                pair_valid[:delta_row, :] = False
            if delta_column > 0:
                pair_valid[:, :delta_column] = False
            elif delta_column < 0:
                pair_valid[:, delta_column:] = False
            distance = resolution * math.hypot(delta_row, delta_column)
            difference = np.where(
                pair_valid, np.abs(elevation - shifted), np.nan)
            result.append((difference, distance))
        return result

    # One-cell differences retain abrupt kerbs/steps.  Slope uses a separate
    # two-cell baseline so a 12 cm vertical edge is not mislabelled as a 26°
    # continuous ramp; a real ramp retains the same angle over that baseline.
    differences = neighbour_differences(
        ((0, 1), (1, 0), (1, 1), (1, -1)))
    slope_differences = neighbour_differences(
        ((0, 2), (2, 0), (2, 2), (2, -2)))
    difference_stack = np.stack([item[0] for item in differences])
    observed_neighbour = np.isfinite(difference_stack).any(axis=0)
    step_values = np.zeros(shape, dtype=np.float32)
    slope_values = np.zeros(shape, dtype=np.float32)
    safe_differences = np.where(
        np.isfinite(difference_stack), difference_stack, -np.inf)
    step_values[observed_neighbour] = np.max(
        safe_differences, axis=0)[observed_neighbour]
    short_slope_stack = np.stack([
        np.degrees(np.arctan(item[0] / item[1])) for item in differences
    ])
    slope_stack = np.stack([
        np.degrees(np.arctan(item[0] / item[1]))
        for item in slope_differences
    ])
    long_observed = np.isfinite(slope_stack).any(axis=0)
    safe_slopes = np.where(np.isfinite(slope_stack), slope_stack, -np.inf)
    slope_values[long_observed] = np.max(
        safe_slopes, axis=0)[long_observed]
    fallback = observed_neighbour & ~long_observed
    safe_short_slopes = np.where(
        np.isfinite(short_slope_stack), short_slope_stack, -np.inf)
    slope_values[fallback] = np.max(
        safe_short_slopes, axis=0)[fallback]
    step[observed_neighbour] = step_values[observed_neighbour]
    slope[observed_neighbour] = slope_values[observed_neighbour]

    cost = np.full(shape, UNKNOWN_COST, dtype=np.int8)
    assessed = valid & observed_neighbour & (confidence >= minimum_confidence)
    cost[assessed] = FREE_COST
    caution = assessed & (
        (slope >= caution_slope_deg) |
        (step >= caution_step_m) |
        (roughness >= roughness_caution_m))
    # ``special_*`` are inclusive automatic ceilings. Values above them
    # require an explicitly calibrated connector instead of ordinary Nav2.
    special = assessed & (
        (slope > special_slope_deg) |
        (step > special_step_m))
    blocked = assessed & (
        (slope >= blocked_slope_deg) |
        (step >= blocked_step_m))
    cost[caution] = CAUTION_COST
    cost[special] = SPECIAL_COST
    cost[blocked] = BLOCKED_COST
    cost[assessed & collision_cells] = BLOCKED_COST
    # An inferred surface remains visible but cannot authorize Nav2 motion.
    cost[ground_measured == 0] = UNKNOWN_COST
    return TerrainGridData(
        resolution=resolution, width=width, height=height,
        origin_x=origin_x, origin_y=origin_y,
        elevation=elevation.ravel(), slope_deg=slope.ravel(),
        roughness_m=roughness.ravel(), step_height_m=step.ravel(),
        confidence=confidence.ravel(), traversal_cost=cost.ravel(),
        ground_measured=ground_measured.ravel())
