#!/usr/bin/env python3
"""Losslessly persist one frozen ROS OccupancyGrid snapshot."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np
from PIL import Image


DEFAULT_MAXIMUM_CELLS = 25_000_000
MAP_NAME_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}')


class OccupancyGridSnapshotError(ValueError):
    """A frozen OccupancyGrid cannot be saved safely."""


def _finite_float(value: Any, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise OccupancyGridSnapshotError(
            f'{field} is not a number') from error
    if not math.isfinite(number):
        raise OccupancyGridSnapshotError(f'{field} must be finite')
    return number


def _validated_snapshot(
    snapshot: dict[str, Any],
    *,
    maximum_cells: int,
) -> tuple[int, int, float, list[float], np.ndarray]:
    """Validate geometry and return an immutable-shape integer grid."""

    try:
        width = int(snapshot['width'])
        height = int(snapshot['height'])
    except (KeyError, TypeError, ValueError) as error:
        raise OccupancyGridSnapshotError(
            'width and height must be integers') from error
    if width <= 0 or height <= 0:
        raise OccupancyGridSnapshotError(
            'width and height must be positive')
    cell_count = width * height
    if maximum_cells <= 0 or cell_count > int(maximum_cells):
        raise OccupancyGridSnapshotError(
            f'grid cell count {cell_count} exceeds limit {maximum_cells}')

    try:
        resolution = _finite_float(snapshot['resolution'], 'resolution')
        origin_values = snapshot['origin']
    except KeyError as error:
        raise OccupancyGridSnapshotError(
            f'missing snapshot field: {error.args[0]}') from error
    if resolution <= 0.0:
        raise OccupancyGridSnapshotError('resolution must be positive')
    if not isinstance(origin_values, (list, tuple)) or len(origin_values) != 3:
        raise OccupancyGridSnapshotError(
            'origin must contain exactly three values')
    origin = [
        _finite_float(value, f'origin[{index}]')
        for index, value in enumerate(origin_values)
    ]

    try:
        values = np.asarray(snapshot['data'])
    except KeyError as error:
        raise OccupancyGridSnapshotError('missing snapshot field: data') from error
    if values.ndim != 1 or int(values.size) != cell_count:
        raise OccupancyGridSnapshotError(
            f'data length {values.size} does not match {cell_count} cells')
    if values.dtype.kind not in ('i', 'u'):
        raise OccupancyGridSnapshotError(
            'occupancy data must contain integers')
    outside_range = np.any(values > 100)
    if values.dtype.kind == 'i':
        outside_range = outside_range or np.any(values < -1)
    if outside_range:
        raise OccupancyGridSnapshotError(
            'occupancy data contains a value outside -1..100')
    values = values.astype(np.int16, copy=False)
    return width, height, resolution, origin, values


def _trinary_pgm(
    values: np.ndarray,
    width: int,
    height: int,
    *,
    free_threshold: float,
    occupied_threshold: float,
) -> np.ndarray:
    """Convert ROS bottom-up occupancy rows into top-down PGM pixels."""

    free_cell = int(np.rint(free_threshold * 100.0))
    occupied_cell = int(np.rint(occupied_threshold * 100.0))
    occupancy = np.flipud(values.reshape((height, width)))
    pixels = np.full((height, width), 205, dtype=np.uint8)
    known = occupancy >= 0
    pixels[known & (occupancy <= free_cell)] = 254
    pixels[known & (occupancy >= occupied_cell)] = 0
    return pixels


def write_occupancy_grid_snapshot(
    snapshot: dict[str, Any],
    output_directory: Path,
    map_name: str,
    *,
    free_threshold: float,
    occupied_threshold: float,
    maximum_cells: int = DEFAULT_MAXIMUM_CELLS,
) -> dict[str, Any]:
    """Write a matched PGM/YAML pair from exactly one frozen snapshot."""

    if not MAP_NAME_PATTERN.fullmatch(str(map_name)):
        raise OccupancyGridSnapshotError('map name is invalid')
    free_threshold = _finite_float(free_threshold, 'free_threshold')
    occupied_threshold = _finite_float(
        occupied_threshold, 'occupied_threshold')
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise OccupancyGridSnapshotError(
            'occupancy thresholds are inconsistent')
    width, height, resolution, origin, values = _validated_snapshot(
        snapshot, maximum_cells=maximum_cells)
    pixels = _trinary_pgm(
        values, width, height,
        free_threshold=free_threshold,
        occupied_threshold=occupied_threshold,
    )

    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    pgm_path = output_directory / f'{map_name}.pgm'
    yaml_path = output_directory / f'{map_name}.yaml'
    pgm_temporary = output_directory / f'.{map_name}.pgm.part'
    yaml_temporary = output_directory / f'.{map_name}.yaml.part'
    document = (
        f'image: {map_name}.pgm\n'
        'mode: trinary\n'
        f'resolution: {repr(resolution)}\n'
        f'origin: {json.dumps(origin, allow_nan=False)}\n'
        'negate: 0\n'
        f'occupied_thresh: {repr(occupied_threshold)}\n'
        f'free_thresh: {repr(free_threshold)}\n'
    )

    try:
        Image.fromarray(pixels, mode='L').save(
            pgm_temporary, format='PPM')
        with yaml_temporary.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(document)
            stream.flush()
        pgm_temporary.replace(pgm_path)
        yaml_temporary.replace(yaml_path)
    except Exception:
        pgm_temporary.unlink(missing_ok=True)
        yaml_temporary.unlink(missing_ok=True)
        raise

    return {
        'image': f'{map_name}.pgm',
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': occupied_threshold,
        'free_thresh': free_threshold,
    }
