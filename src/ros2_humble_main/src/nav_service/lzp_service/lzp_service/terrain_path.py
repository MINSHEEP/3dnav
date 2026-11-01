#!/usr/bin/env python3
"""Pure helpers for visualizing a Nav2 path on global terrain."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .terrain_grid import TerrainGridData


@dataclass(frozen=True)
class ElevatedPath:
    """A display-only XYZ path and its terrain coverage counters."""

    points: np.ndarray
    elevated_count: int
    unknown_count: int


def simplify_path_indices(
        points,
        *,
        minimum_spacing_m: float = 0.10,
        maximum_points: int = 4000) -> np.ndarray:
    """Select display poses while preserving endpoints and visible turns."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('path points must be an N x 3 array')
    if values.shape[0] == 0:
        return np.empty(0, dtype=np.int64)
    if not np.isfinite(values).all():
        raise ValueError('path coordinates must be finite')
    spacing = float(minimum_spacing_m)
    limit = int(maximum_points)
    if not math.isfinite(spacing) or spacing < 0.0 or limit < 2:
        raise ValueError('path display bounds are invalid')
    if values.shape[0] <= 2:
        return np.arange(values.shape[0], dtype=np.int64)

    turn_threshold = math.radians(8.0)
    turn_angles = {}
    kept = [0]
    for index in range(1, values.shape[0] - 1):
        last_index = kept[-1]
        last_x = float(values[last_index, 0])
        last_y = float(values[last_index, 1])
        current_x = float(values[index, 0])
        current_y = float(values[index, 1])
        following_x = float(values[index + 1, 0])
        following_y = float(values[index + 1, 1])
        before_x = current_x - last_x
        before_y = current_y - last_y
        after_x = following_x - current_x
        after_y = following_y - current_y
        distance = math.hypot(before_x, before_y)
        denominator = (
            math.hypot(before_x, before_y)
            * math.hypot(after_x, after_y)
        )
        turn = 0.0
        if denominator > 1.0e-9:
            cosine = max(-1.0, min(
                1.0,
                (before_x * after_x + before_y * after_y) / denominator,
            ))
            turn = math.acos(cosine)
        if turn >= turn_threshold:
            turn_angles[index] = turn
        if distance >= spacing or turn >= turn_threshold:
            kept.append(index)
    kept.append(values.shape[0] - 1)
    selected = np.asarray(kept, dtype=np.int64)
    if selected.size > limit:
        positions = np.linspace(0, selected.size - 1, limit)
        chosen = set(selected[
            np.unique(np.rint(positions).astype(np.int64))].tolist())
        ranked_turns = sorted(
            (
                (turn_angles.get(int(index), 0.0), int(index))
                for index in selected[1:-1]
            ),
            reverse=True,
        )
        mandatory = {
            int(selected[0]),
            int(selected[-1]),
            *(index for angle, index in ranked_turns[:limit - 2]
              if angle >= turn_threshold),
        }
        for turn_index in sorted(mandatory):
            if turn_index in chosen:
                continue
            replaceable = chosen - mandatory - {
                int(selected[0]), int(selected[-1])}
            if not replaceable:
                break
            replacement = min(
                replaceable, key=lambda value: abs(value - turn_index))
            chosen.remove(replacement)
            chosen.add(turn_index)
        selected = np.asarray(sorted(chosen), dtype=np.int64)
    return np.ascontiguousarray(selected)


def simplify_path_points(
        points,
        *,
        minimum_spacing_m: float = 0.10,
        maximum_points: int = 4000) -> np.ndarray:
    """Bound a display path without changing the controller's original path."""

    values = np.asarray(points, dtype=np.float64)
    indices = simplify_path_indices(
        values,
        minimum_spacing_m=minimum_spacing_m,
        maximum_points=maximum_points,
    )
    return np.ascontiguousarray(values[indices])


def path_length_m(points) -> float:
    """Return the XYZ polyline length represented by path points."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('path points must be an N x 3 array')
    if not np.isfinite(values).all():
        raise ValueError('path coordinates must be finite')
    if values.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(values, axis=0), axis=1).sum())


def elevate_path_points(
        points,
        terrain: TerrainGridData,
        *,
        clearance_m: float = 0.08) -> ElevatedPath:
    """Copy XYZ points and lift known cells onto the terrain surface."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('path points must be an N x 3 array')
    if not np.isfinite(values).all():
        raise ValueError('path coordinates must be finite')
    clearance = float(clearance_m)
    if not math.isfinite(clearance) or clearance < 0.0:
        raise ValueError('path clearance must be finite and non-negative')
    resolution = float(terrain.resolution)
    width = int(terrain.width)
    height = int(terrain.height)
    if (
        not math.isfinite(resolution) or resolution <= 0.0
        or width <= 0 or height <= 0
    ):
        raise ValueError('terrain geometry is invalid')
    expected = width * height
    elevation = np.asarray(terrain.elevation).reshape(-1)
    cost = np.asarray(terrain.traversal_cost).reshape(-1)
    if elevation.size != expected or cost.size != expected:
        raise ValueError('terrain layer size does not match its geometry')

    result = values.copy()
    if result.shape[0] == 0:
        return ElevatedPath(result, 0, 0)
    columns = np.floor(
        (result[:, 0] - float(terrain.origin_x)) / resolution).astype(np.int64)
    rows = np.floor(
        (result[:, 1] - float(terrain.origin_y)) / resolution).astype(np.int64)
    inside = (
        (columns >= 0) & (columns < width)
        & (rows >= 0) & (rows < height)
    )
    indices = rows * width + columns
    known = np.zeros(result.shape[0], dtype=bool)
    covered_indices = np.flatnonzero(inside)
    if covered_indices.size:
        cells = indices[covered_indices]
        known[covered_indices] = (
            np.isfinite(elevation[cells]) & (cost[cells] >= 0)
        )
        lifted = np.flatnonzero(known)
        result[lifted, 2] = elevation[indices[lifted]] + clearance
    elevated_count = int(np.count_nonzero(known))
    return ElevatedPath(
        points=np.ascontiguousarray(result),
        elevated_count=elevated_count,
        unknown_count=int(result.shape[0] - elevated_count),
    )
