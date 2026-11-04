"""CPU-bounded local 2.5-D terrain estimation from a registered point cloud."""

from __future__ import annotations

import math

import numpy as np

from .terrain_navigation import TerrainObservation


def _unknown(confidence=0.0):
    return TerrainObservation(
        terrain_type='unknown', confidence=max(0.0, min(1.0, confidence)),
        clearance_width_m=0.0)


def corridor_return_diagnostics(
        points, *, robot_x, robot_y, yaw,
        forward_min=0.10, forward_max=1.30, half_width=0.45):
    """Describe live return coverage without changing traversability.

    The near-field window matches the observer's forward obstacle check.
    Counts include returns at every height: a positive count does *not*
    certify floor or negative-obstacle visibility.
    """
    values = np.asarray(points, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('points must be an N x 3 array')
    finite = values[np.isfinite(values).all(axis=1)]
    dx = finite[:, 0] - float(robot_x)
    dy = finite[:, 1] - float(robot_y)
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    corridor = (
        (local_x >= forward_min) & (local_x <= forward_max)
        & (np.abs(local_y) <= half_width))
    observed_x = local_x[corridor]
    return {
        'corridor_return_count': int(observed_x.size),
        'near_field_return_count': int(np.count_nonzero(
            observed_x <= min(1.0, forward_max))),
        'nearest_forward_return_m': (
            float(np.min(observed_x)) if observed_x.size else None),
    }


def estimate_local_terrain(
        points, *, robot_x, robot_y, yaw,
        forward_min=0.10, forward_max=1.30, half_width=0.45,
        resolution=0.12, minimum_points=30, ground_anchor=None,
        separate_missing_returns=False):
    """Estimate the floor and hazards in a robot-aligned forward corridor.

    This estimator is deliberately conservative. Missing or sparse geometry
    becomes ``unknown``; it never becomes free space merely because no return
    was received.
    """
    values = np.asarray(points, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('points must be an N x 3 array')
    finite = values[np.isfinite(values).all(axis=1)]
    if finite.shape[0] < minimum_points:
        return _unknown(finite.shape[0] / max(1.0, minimum_points))

    dx = finite[:, 0] - float(robot_x)
    dy = finite[:, 1] - float(robot_y)
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    corridor = (
        (local_x >= forward_min) & (local_x <= forward_max)
        & (np.abs(local_y) <= half_width))
    local = np.column_stack((local_x[corridor], local_y[corridor],
                             finite[corridor, 2]))
    if local.shape[0] < minimum_points:
        return _unknown(local.shape[0] / max(1.0, minimum_points))

    x_index = np.floor((local[:, 0] - forward_min) / resolution).astype(int)
    y_index = np.floor((local[:, 1] + half_width) / resolution).astype(int)
    columns = max(1, int(math.ceil((2.0 * half_width) / resolution)))
    linear = x_index * columns + y_index
    order = np.lexsort((local[:, 2], linear))
    unique, first = np.unique(linear[order], return_index=True)
    floor_points = local[order[first]]
    expected_cells = max(1, int(math.ceil(
        (forward_max - forward_min) / resolution)) * columns)
    coverage = min(1.0, unique.size / (expected_cells * 0.62))
    density = min(1.0, local.shape[0] / 200.0)
    confidence = coverage * density
    if floor_points.shape[0] < 8 or confidence < 0.25:
        return _unknown(confidence)

    design = np.column_stack((
        floor_points[:, 0], floor_points[:, 1],
        np.ones(floor_points.shape[0], dtype=np.float32)))
    coefficients, _, _, _ = np.linalg.lstsq(
        design, floor_points[:, 2], rcond=None)
    predicted_floor = design @ coefficients
    residual = floor_points[:, 2] - predicted_floor
    median = float(np.median(residual))
    deviation = float(np.median(np.abs(residual - median)))
    keep = np.abs(residual - median) <= max(0.025, deviation * 3.5)
    if int(np.count_nonzero(keep)) >= 8:
        coefficients, _, _, _ = np.linalg.lstsq(
            design[keep], floor_points[keep, 2], rcond=None)
        predicted_floor = design @ coefficients
        residual = floor_points[:, 2] - predicted_floor

    slope_deg = math.degrees(math.atan(float(coefficients[0])))
    cross_slope_deg = math.degrees(math.atan(float(coefficients[1])))
    anchor_step = 0.0
    if ground_anchor is not None:
        ax, ay, az = map(float, ground_anchor)
        if not all(math.isfinite(v) for v in (ax, ay, az)):
            return _unknown()
        anchor_dx, anchor_dy = ax - robot_x, ay - robot_y
        anchor_x = cosine * anchor_dx + sine * anchor_dy
        anchor_y = -sine * anchor_dx + cosine * anchor_dy
        ground_z = coefficients[0]*anchor_x + coefficients[1]*anchor_y + coefficients[2]
        if not math.isfinite(float(ground_z)) or abs(ground_z - az) > .15:
            return _unknown()
        # A uniformly raised/lowered platform has no within-corridor edge.
        # Its offset from the physical floor reference is still a step.
        anchor_step = abs(float(ground_z)-az)
    roughness = float(np.percentile(np.abs(residual), 90))

    floor_by_cell = {int(key): float(point[2]) for key, point in zip(
        unique.tolist(), floor_points.tolist())}
    step_height = 0.0
    for key, height in floor_by_cell.items():
        neighbours = [key + columns]
        if key % columns + 1 < columns:
            neighbours.append(key + 1)
        for neighbour in neighbours:
            if neighbour in floor_by_cell:
                step_height = max(
                    step_height, abs(floor_by_cell[neighbour] - height))
    expected_rise = abs(math.tan(math.radians(slope_deg))) * resolution
    step_height = max(anchor_step, max(0.0, step_height - expected_rise))

    floor_prediction = (
        coefficients[0] * local[:, 0]
        + coefficients[1] * local[:, 1] + coefficients[2])
    height_above_floor = local[:, 2] - floor_prediction
    obstacle = (
        (height_above_floor >= 0.08) & (height_above_floor <= 0.85)
        & (local[:, 0] <= forward_max) & (np.abs(local[:, 1]) <= half_width))
    obstacle_cells = np.unique(linear[obstacle]).size
    obstacle_present = int(np.count_nonzero(obstacle)) >= 8 and obstacle_cells >= 3
    obstacle_height = float(np.max(height_above_floor[obstacle])) if obstacle_present else 0.0
    low_obstacle = obstacle_present and obstacle_height <= 0.12 and confidence >= 0.70
    blocked = obstacle_present and not low_obstacle
    if low_obstacle:
        step_height = max(step_height, obstacle_height)

    centre_floor = floor_points[np.abs(floor_points[:, 1]) <= 0.20]
    gap_width = 0.0
    negative_obstacle = False
    if centre_floor.shape[0] >= 4:
        occupied_x = np.unique(np.floor(
            (centre_floor[:, 0] - forward_min) / resolution).astype(int))
        for left, right in zip(occupied_x[:-1], occupied_x[1:]):
            missing = max(0, int(right - left - 1))
            gap_width = max(gap_width, missing * resolution)
        # A missing return is uncertainty. A measured downward discontinuity
        # is positive evidence of a drop; do not turn occlusion into a hole.
        negative_obstacle = int(np.count_nonzero(residual < -0.12)) >= 3

    y_span = float(np.ptp(floor_points[:, 1])) if floor_points.size else 0.0
    clearance = min(2.0 * half_width, y_span + resolution)
    if blocked:
        terrain_type = 'blocked'
    elif negative_obstacle:
        terrain_type = 'negative_obstacle'
    elif gap_width > 0.12:
        terrain_type = 'unknown'
    elif abs(slope_deg) >= 8.0:
        terrain_type = 'ramp'
    elif step_height >= 0.05:
        terrain_type = 'curb'
    elif roughness >= 0.03:
        terrain_type = 'rough'
    else:
        terrain_type = 'flat'
    return TerrainObservation(
        terrain_type=terrain_type,
        slope_deg=slope_deg,
        cross_slope_deg=cross_slope_deg,
        step_height_m=step_height,
        gap_width_m=(gap_width if negative_obstacle or not separate_missing_returns else 0.0),
        roughness_m=roughness,
        clearance_width_m=clearance,
        confidence=confidence,
        negative_obstacle=negative_obstacle,
        slippery=False,
        sampling_gap_width_m=gap_width if separate_missing_returns else 0.0)
