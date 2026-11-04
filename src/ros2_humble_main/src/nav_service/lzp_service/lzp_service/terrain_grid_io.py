#!/usr/bin/env python3
"""Safe, versioned persistence for the global 2.5-D terrain grid."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import json
import math
import os
import operator
from pathlib import Path
import re
from typing import BinaryIO
import zipfile

import numpy as np

from .terrain_grid import TerrainGridData


TERRAIN_GRID_SCHEMA = 'quadruped_terrain_grid'
TERRAIN_GRID_VERSION = 2
MAX_TERRAIN_CELLS = 400_000
MAX_TERRAIN_FILE_BYTES = 64 * 1024 * 1024
MAX_TERRAIN_UNCOMPRESSED_BYTES = 32 * 1024 * 1024
MAX_NPY_HEADER_BYTES = 4096
MAX_GENERATION_JSON_BYTES = 16 * 1024
MAP_NAME_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}')
ALLOWED_TRAVERSAL_COSTS = np.asarray([-1, 0, 60, 90, 100], dtype=np.int8)
SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')

_FLOAT_LAYERS = (
    'elevation',
    'slope_deg',
    'roughness_m',
    'step_height_m',
    'confidence',
)
_ALL_LAYERS = _FLOAT_LAYERS + ('traversal_cost',)
_SCALARS = (
    'schema',
    'version',
    'map_name',
    'frame_id',
    'resolution',
    'width',
    'height',
    'origin_x',
    'origin_y',
    'generation_json',
)
_EXPECTED_MEMBERS = {
    f'{name}.npy' for name in _SCALARS + _ALL_LAYERS
}


@dataclass(frozen=True)
class SavedTerrainGrid:
    """One validated terrain artifact and its provenance."""

    map_name: str
    frame_id: str
    grid: TerrainGridData
    generation: dict
    version: int = 1


def terrain_artifact_signature(path: str | Path) -> tuple[int, int]:
    """Return the cheap signature used to suppress repeated map announcements."""

    stat_result = Path(path).stat()
    return int(stat_result.st_mtime_ns), int(stat_result.st_size)


def _safe_map_name(value) -> str:
    name = str(value or '').strip()
    if not MAP_NAME_PATTERN.fullmatch(name):
        raise ValueError('terrain map_name is invalid')
    return name


def _safe_frame_id(value) -> str:
    frame_id = str(value or '').strip().lstrip('/')
    if frame_id != 'map':
        raise ValueError('terrain frame_id must be map')
    return frame_id


def _finite_float(value, field_name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'terrain {field_name} is not numeric') from error
    if not math.isfinite(result):
        raise ValueError(f'terrain {field_name} must be finite')
    return result


def _scalar(array, name: str):
    value = np.asarray(array)
    if value.ndim != 0:
        raise ValueError(f'terrain {name} must be a scalar')
    return value.item()


def _integer_scalar(array, name: str) -> int:
    value = np.asarray(array)
    if value.ndim != 0 or value.dtype.kind not in ('i', 'u'):
        raise ValueError(f'terrain {name} must be an integer scalar')
    return int(value.item())


def _metadata_integer(value, field_name: str) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
            value, (int, np.integer)):
        raise ValueError(f'terrain metadata {field_name} must be an integer')
    return int(value)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256(value, field_name: str) -> str:
    digest = str(value or '').strip().lower()
    if not SHA256_PATTERN.fullmatch(digest):
        raise ValueError(f'terrain {field_name} must be a SHA-256 digest')
    return digest


def _validated_generation(value) -> tuple[dict, str]:
    document = {} if value is None else value
    if not isinstance(document, dict):
        raise ValueError('terrain generation metadata must be an object')
    try:
        encoded = json.dumps(
            document,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(',', ':'),
        )
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'terrain generation metadata is not JSON-safe: {error}'
        ) from error
    if len(encoded.encode('utf-8')) > MAX_GENERATION_JSON_BYTES:
        raise ValueError('terrain generation metadata is too large')
    return document, encoded


def _validate_grid(grid: TerrainGridData) -> TerrainGridData:
    resolution = _finite_float(grid.resolution, 'resolution')
    origin_x = _finite_float(grid.origin_x, 'origin_x')
    origin_y = _finite_float(grid.origin_y, 'origin_y')
    try:
        width = operator.index(grid.width)
        height = operator.index(grid.height)
    except (TypeError, ValueError) as error:
        raise ValueError('terrain width and height must be integers') from error
    cell_count = width * height
    if resolution <= 0.0 or width <= 0 or height <= 0:
        raise ValueError('terrain resolution, width and height must be positive')
    if cell_count > MAX_TERRAIN_CELLS:
        raise ValueError(
            f'terrain grid has {cell_count} cells, limit is {MAX_TERRAIN_CELLS}'
        )

    layers = {}
    for name in _FLOAT_LAYERS:
        source = np.asarray(getattr(grid, name))
        if source.dtype.kind != 'f':
            raise ValueError(f'terrain {name} must use a floating dtype')
        layer = source.astype(np.float32, copy=False).reshape(-1)
        if layer.size != cell_count:
            raise ValueError(
                f'terrain {name} has {layer.size} cells, expected {cell_count}'
            )
        if np.any(np.isinf(layer)):
            raise ValueError(f'terrain {name} contains infinite values')
        layers[name] = np.ascontiguousarray(layer)

    confidence = layers['confidence']
    if np.any(~np.isfinite(confidence)) or np.any(
            (confidence < 0.0) | (confidence > 1.0)):
        raise ValueError('terrain confidence must be finite and in [0, 1]')

    source_cost = np.asarray(grid.traversal_cost)
    if source_cost.dtype.kind not in ('i', 'u'):
        raise ValueError('terrain traversal_cost must use an integer dtype')
    source_cost = source_cost.reshape(-1)
    if source_cost.size != cell_count:
        raise ValueError(
            'terrain traversal_cost has '
            f'{source_cost.size} cells, expected {cell_count}'
        )
    if not np.all(np.isin(source_cost, ALLOWED_TRAVERSAL_COSTS)):
        raise ValueError('terrain traversal_cost contains unsupported values')
    traversal_cost = source_cost.astype(np.int8, copy=False)
    measured = getattr(grid, 'ground_measured', None)
    if measured is not None:
        measured = np.asarray(measured)
        if (measured.dtype.kind not in ('b', 'i', 'u')
                or measured.size != cell_count
                or not np.all(np.isin(measured, [0, 1]))):
            raise ValueError('terrain ground_measured must contain one 0/1 per cell')
        measured = np.ascontiguousarray(measured.astype(np.uint8).reshape(-1))

    return TerrainGridData(
        resolution=resolution,
        width=width,
        height=height,
        origin_x=origin_x,
        origin_y=origin_y,
        elevation=layers['elevation'],
        slope_deg=layers['slope_deg'],
        roughness_m=layers['roughness_m'],
        step_height_m=layers['step_height_m'],
        confidence=confidence,
        traversal_cost=np.ascontiguousarray(traversal_cost),
        ground_measured=measured,
    )


def write_terrain_grid(
        path: str | Path,
        *,
        map_name: str,
        frame_id: str,
        grid: TerrainGridData,
        generation: dict | None = None) -> dict:
    """Write one complete compressed artifact and return metadata fields."""

    target = Path(path)
    name = _safe_map_name(map_name)
    frame = _safe_frame_id(frame_id)
    validated = _validate_grid(grid)
    version = 2 if validated.ground_measured is not None else 1
    generation_document, generation_json = _validated_generation(generation)
    with target.open('wb') as stream:
        np.savez_compressed(
            stream,
            schema=np.asarray(TERRAIN_GRID_SCHEMA),
            version=np.asarray(version, dtype=np.int64),
            map_name=np.asarray(name),
            frame_id=np.asarray(frame),
            resolution=np.asarray(validated.resolution, dtype=np.float64),
            width=np.asarray(validated.width, dtype=np.int64),
            height=np.asarray(validated.height, dtype=np.int64),
            origin_x=np.asarray(validated.origin_x, dtype=np.float64),
            origin_y=np.asarray(validated.origin_y, dtype=np.float64),
            generation_json=np.asarray(generation_json),
            elevation=validated.elevation,
            slope_deg=validated.slope_deg,
            roughness_m=validated.roughness_m,
            step_height_m=validated.step_height_m,
            confidence=validated.confidence,
            traversal_cost=validated.traversal_cost,
            **({'ground_measured': validated.ground_measured}
               if validated.ground_measured is not None else {}),
        )
        stream.flush()
        os.fsync(stream.fileno())
    loaded = load_terrain_grid(target, expected_map_name=name)
    cell_count = validated.width * validated.height
    return {
        'schema': TERRAIN_GRID_SCHEMA,
        'version': version,
        'map_name': name,
        'frame_id': frame,
        'resolution': validated.resolution,
        'width': validated.width,
        'height': validated.height,
        'origin_x': validated.origin_x,
        'origin_y': validated.origin_y,
        'cell_count': cell_count,
        'known_cells': int(np.count_nonzero(
            validated.traversal_cost >= 0)),
        'blocked_cells': int(np.count_nonzero(
            validated.traversal_cost == 100)),
        'generation': loaded.generation or generation_document,
    }


def create_terrain_grid_metadata(
        path: str | Path,
        *,
        summary: dict,
        source_pcd_sha256: str) -> dict:
    """Bind a validated terrain artifact to one localization PCD digest."""

    source = Path(path)
    pcd_digest = _sha256(source_pcd_sha256, 'source_pcd_sha256')
    if not isinstance(summary, dict):
        raise ValueError('terrain summary must be an object')
    generation = summary.get('generation')
    if not isinstance(generation, dict):
        raise ValueError('terrain summary generation must be an object')
    if _sha256(
            generation.get('source_pcd_sha256'),
            'generation.source_pcd_sha256') != pcd_digest:
        raise ValueError('terrain generation is not bound to the source PCD')
    size = source.stat().st_size
    if size <= 0 or size > MAX_TERRAIN_FILE_BYTES:
        raise ValueError('terrain artifact file size is invalid')
    return {
        'schema': TERRAIN_GRID_SCHEMA,
        'version': int(summary['version']),
        'filename': source.name,
        'map_name': _safe_map_name(summary.get('map_name')),
        'frame_id': _safe_frame_id(summary.get('frame_id')),
        'source_pcd_sha256': pcd_digest,
        'size_bytes': int(size),
        'sha256': _sha256_file(source),
        'resolution': _finite_float(summary.get('resolution'), 'resolution'),
        'width': int(summary.get('width')),
        'height': int(summary.get('height')),
        'origin_x': _finite_float(summary.get('origin_x'), 'origin_x'),
        'origin_y': _finite_float(summary.get('origin_y'), 'origin_y'),
        'cell_count': int(summary.get('cell_count')),
        'known_cells': int(summary.get('known_cells')),
        'blocked_cells': int(summary.get('blocked_cells')),
        'generation': generation,
    }


def _inspect_npy_header(archive: zipfile.ZipFile, member: zipfile.ZipInfo) -> None:
    """Bound headers before NumPy parses them, including on older NumPy APIs.

    NPY 1.0 stores a two-byte little-endian header length; 2.0/3.0 use
    four bytes. No array, Python literal or pickle is decoded by this check.
    """
    with archive.open(member, 'r') as stream:
        prefix = stream.read(8)
        if len(prefix) != 8 or prefix[:6] != b'\x93NUMPY':
            raise ValueError('terrain NPY magic/version header is invalid')
        version = tuple(prefix[6:8])
        if version == (1, 0):
            length_bytes = 2
        elif version in ((2, 0), (3, 0)):
            length_bytes = 4
        else:
            raise ValueError('terrain NPY format version is unsupported')
        encoded_length = stream.read(length_bytes)
        if len(encoded_length) != length_bytes:
            raise ValueError('terrain NPY header length is truncated')
        header_length = int.from_bytes(encoded_length, 'little')
        if not 0 < header_length <= MAX_NPY_HEADER_BYTES:
            raise ValueError('terrain NPY header exceeds the safety limit')
        if 8 + length_bytes + header_length > member.file_size:
            raise ValueError('terrain NPY header is truncated')
        if len(stream.read(header_length)) != header_length:
            raise ValueError('terrain NPY header is truncated')


def _inspect_archive(source: BinaryIO) -> None:
    size = os.fstat(source.fileno()).st_size
    if size <= 0 or size > MAX_TERRAIN_FILE_BYTES:
        raise ValueError('terrain artifact file size is invalid')
    try:
        with zipfile.ZipFile(source, 'r') as archive:
            members = archive.infolist()
            names = [member.filename for member in members]
            if len(names) != len(set(names)):
                raise ValueError('terrain artifact has duplicate ZIP members')
            if set(names) not in (_EXPECTED_MEMBERS,
                                 _EXPECTED_MEMBERS | {'ground_measured.npy'}):
                raise ValueError('terrain artifact has missing or unexpected members')
            if any(member.is_dir() or (member.flag_bits & 0x1) for member in members):
                raise ValueError('terrain artifact contains an invalid ZIP member')
            expanded = sum(int(member.file_size) for member in members)
            if expanded > MAX_TERRAIN_UNCOMPRESSED_BYTES:
                raise ValueError('terrain artifact expands beyond the safety limit')
            for member in members:
                _inspect_npy_header(archive, member)
    except zipfile.BadZipFile as error:
        raise ValueError('terrain artifact is not a valid NPZ archive') from error


@contextmanager
def _open_terrain_archive(path: Path):
    if not path.is_file():
        raise ValueError(f'terrain artifact does not exist: {path.name}')
    # Keep one descriptor from preflight through array decoding so replacing
    # the pathname cannot swap an unchecked file into np.load().
    with path.open('rb') as source:
        _inspect_archive(source)
        source.seek(0)
        # max_header_size is absent from older NumPy load() signatures. The
        # independent preflight enforces the same 4096-byte limit on all APIs.
        with np.load(source, allow_pickle=False) as archive:
            yield archive


def load_terrain_grid(
        path: str | Path,
        *,
        expected_map_name: str | None = None) -> SavedTerrainGrid:
    """Load a strictly bounded NPZ without pickle or object arrays."""

    source = Path(path)
    try:
        with _open_terrain_archive(source) as archive:
            schema = str(_scalar(archive['schema'], 'schema'))
            version = _integer_scalar(archive['version'], 'version')
            if ('ground_measured' in archive.files) != (version == 2):
                raise ValueError('terrain version and ground_measured layer disagree')
            map_name = _safe_map_name(_scalar(archive['map_name'], 'map_name'))
            frame_id = _safe_frame_id(_scalar(archive['frame_id'], 'frame_id'))
            resolution = _finite_float(
                _scalar(archive['resolution'], 'resolution'), 'resolution')
            width = _integer_scalar(archive['width'], 'width')
            height = _integer_scalar(archive['height'], 'height')
            origin_x = _finite_float(
                _scalar(archive['origin_x'], 'origin_x'), 'origin_x')
            origin_y = _finite_float(
                _scalar(archive['origin_y'], 'origin_y'), 'origin_y')
            generation_text = str(_scalar(
                archive['generation_json'], 'generation_json'))
            if len(generation_text.encode('utf-8')) > MAX_GENERATION_JSON_BYTES:
                raise ValueError('terrain generation metadata is too large')
            generation = json.loads(generation_text)
            float_layers = {}
            for name in _FLOAT_LAYERS:
                layer = np.asarray(archive[name])
                if layer.dtype != np.dtype(np.float32):
                    raise ValueError(
                        f'terrain {name} must use float32 storage')
                float_layers[name] = layer.copy()
            traversal_cost = np.asarray(archive['traversal_cost'])
            if traversal_cost.dtype != np.dtype(np.int8):
                raise ValueError(
                    'terrain traversal_cost must use int8 storage')
            ground_measured = None
            if version == 2:
                ground_measured = np.asarray(archive['ground_measured'])
                if ground_measured.dtype != np.dtype(np.uint8):
                    raise ValueError('terrain ground_measured must use uint8 storage')
                ground_measured = ground_measured.copy()
            grid = TerrainGridData(
                resolution=resolution,
                width=width,
                height=height,
                origin_x=origin_x,
                origin_y=origin_y,
                elevation=float_layers['elevation'],
                slope_deg=float_layers['slope_deg'],
                roughness_m=float_layers['roughness_m'],
                step_height_m=float_layers['step_height_m'],
                confidence=float_layers['confidence'],
                traversal_cost=traversal_cost.copy(),
                ground_measured=ground_measured,
            )
    except (
            EOFError, KeyError, OSError, OverflowError, TypeError, ValueError,
            json.JSONDecodeError, zipfile.BadZipFile) as error:
        raise ValueError(f'terrain artifact validation failed: {error}') from error

    if schema != TERRAIN_GRID_SCHEMA or version not in (1, 2):
        raise ValueError('terrain artifact schema or version is unsupported')
    if expected_map_name is not None and map_name != _safe_map_name(
            expected_map_name):
        raise ValueError('terrain artifact map_name does not match the selected map')
    generation, _ = _validated_generation(generation)
    validated = _validate_grid(grid)
    return SavedTerrainGrid(
        map_name=map_name,
        frame_id=frame_id,
        grid=validated,
        generation=generation,
        version=version,
    )


def load_bound_terrain_grid(
        directory: str | Path,
        *,
        map_name: str,
        localization_metadata: dict,
        actual_pcd_sha256: str | None = None) -> SavedTerrainGrid | None:
    """Load terrain declared by localization metadata, or return legacy None.

    A present ``terrain_grid`` section is authoritative. Any mismatch is an
    error and must not silently fall back to reconstructing from the PCD.
    """

    name = _safe_map_name(map_name)
    if not isinstance(localization_metadata, dict):
        raise ValueError('localization metadata must be an object')
    if 'terrain_grid' not in localization_metadata:
        return None
    terrain_metadata = localization_metadata.get('terrain_grid')
    if not isinstance(terrain_metadata, dict):
        raise ValueError('terrain_grid metadata must be an object')

    expected_filename = f'{name}.terrain.npz'
    if str(terrain_metadata.get('filename') or '') != expected_filename:
        raise ValueError(
            f'terrain filename must be {expected_filename}')
    if terrain_metadata.get('schema') != TERRAIN_GRID_SCHEMA:
        raise ValueError('terrain metadata schema is unsupported')
    metadata_version = _metadata_integer(
        terrain_metadata.get('version'), 'version')
    if metadata_version not in (1, 2):
        raise ValueError('terrain metadata version is unsupported')
    if _safe_map_name(terrain_metadata.get('map_name')) != name:
        raise ValueError('terrain metadata map_name does not match')
    _safe_frame_id(terrain_metadata.get('frame_id'))

    pcd_digest = _sha256(
        localization_metadata.get('pcd_sha256'), 'pcd_sha256')
    if actual_pcd_sha256 is None:
        pcd_path = Path(directory) / f'{name}.pcd'
        if not pcd_path.is_file():
            raise ValueError(f'localization PCD is missing: {pcd_path.name}')
        actual_pcd_sha256 = _sha256_file(pcd_path)
    if _sha256(actual_pcd_sha256, 'actual_pcd_sha256') != pcd_digest:
        raise ValueError('localization PCD SHA-256 does not match metadata')
    if _sha256(
            terrain_metadata.get('source_pcd_sha256'),
            'source_pcd_sha256') != pcd_digest:
        raise ValueError('terrain metadata is bound to a different PCD')

    path = Path(directory) / expected_filename
    if not path.is_file():
        raise ValueError(f'terrain artifact is missing: {expected_filename}')
    actual_size = path.stat().st_size
    recorded_size = _metadata_integer(
        terrain_metadata.get('size_bytes'), 'size_bytes')
    if recorded_size != actual_size:
        raise ValueError('terrain artifact size does not match metadata')
    recorded_digest = _sha256(terrain_metadata.get('sha256'), 'sha256')
    if _sha256_file(path) != recorded_digest:
        raise ValueError('terrain artifact SHA-256 does not match metadata')

    loaded = load_terrain_grid(path, expected_map_name=name)
    if loaded.version != metadata_version:
        raise ValueError('terrain artifact version does not match metadata')
    grid = loaded.grid
    expected_values = {
        'resolution': grid.resolution,
        'width': grid.width,
        'height': grid.height,
        'origin_x': grid.origin_x,
        'origin_y': grid.origin_y,
        'cell_count': grid.width * grid.height,
        'known_cells': int(np.count_nonzero(grid.traversal_cost >= 0)),
        'blocked_cells': int(np.count_nonzero(grid.traversal_cost == 100)),
    }
    for field_name, expected in expected_values.items():
        recorded = terrain_metadata.get(field_name)
        if isinstance(expected, float):
            if not math.isclose(
                    _finite_float(recorded, field_name), expected,
                    rel_tol=0.0, abs_tol=1.0e-9):
                raise ValueError(
                    f'terrain metadata {field_name} does not match artifact')
        else:
            recorded_integer = _metadata_integer(recorded, field_name)
            if recorded_integer != expected:
                raise ValueError(
                    f'terrain metadata {field_name} does not match artifact')
    _, recorded_generation_json = _validated_generation(
        terrain_metadata.get('generation'))
    _, loaded_generation_json = _validated_generation(loaded.generation)
    if recorded_generation_json != loaded_generation_json:
        raise ValueError('terrain generation metadata does not match artifact')
    if _sha256(
            loaded.generation.get('source_pcd_sha256'),
            'generation.source_pcd_sha256') != pcd_digest:
        raise ValueError('terrain artifact is bound to a different PCD')
    return loaded
