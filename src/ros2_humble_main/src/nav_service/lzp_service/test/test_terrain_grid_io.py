#!/usr/bin/env python3
"""Regression tests for the versioned terrain-grid map artifact."""

from pathlib import Path
import hashlib
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.terrain_grid import TerrainGridData  # noqa: E402
from lzp_service.terrain_grid_io import (  # noqa: E402
    TERRAIN_GRID_SCHEMA,
    create_terrain_grid_metadata,
    load_bound_terrain_grid,
    load_terrain_grid,
    terrain_artifact_signature,
    write_terrain_grid,
)


def _grid():
    return TerrainGridData(
        resolution=0.2,
        width=3,
        height=2,
        origin_x=-1.0,
        origin_y=2.0,
        elevation=np.asarray([0.0, 0.1, np.nan, 0.2, 0.3, 0.4], dtype=np.float32),
        slope_deg=np.asarray([0, 5, np.nan, 8, 10, 12], dtype=np.float32),
        roughness_m=np.asarray([0, .01, np.nan, .02, .03, .04], dtype=np.float32),
        step_height_m=np.asarray([0, .1, np.nan, .12, .14, .16], dtype=np.float32),
        confidence=np.asarray([1, .75, 0, .5, .75, 1], dtype=np.float32),
        traversal_cost=np.asarray([0, 60, -1, 0, 90, 100], dtype=np.int8),
    )


def test_round_trip_preserves_every_layer_without_pickle(tmp_path):
    path = tmp_path / 'campus.terrain.npz'
    summary = write_terrain_grid(
        path,
        map_name='campus',
        frame_id='map',
        grid=_grid(),
        generation={'blocked_step_m': 0.18, 'source': 'complete_localization_pcd'},
    )
    loaded = load_terrain_grid(path, expected_map_name='campus')

    assert summary['schema'] == TERRAIN_GRID_SCHEMA
    assert summary['cell_count'] == 6
    assert loaded.map_name == 'campus'
    assert loaded.frame_id == 'map'
    assert loaded.generation['source'] == 'complete_localization_pcd'
    assert loaded.grid.resolution == 0.2
    assert loaded.grid.width == 3
    assert loaded.grid.height == 2
    np.testing.assert_allclose(
        loaded.grid.elevation, _grid().elevation, equal_nan=True)
    np.testing.assert_array_equal(
        loaded.grid.traversal_cost, _grid().traversal_cost)


def test_loader_rejects_wrong_map_and_truncated_layer(tmp_path):
    path = tmp_path / 'campus.terrain.npz'
    write_terrain_grid(
        path, map_name='campus', frame_id='map', grid=_grid())
    try:
        load_terrain_grid(path, expected_map_name='other')
    except ValueError as error:
        assert 'map_name' in str(error)
    else:
        raise AssertionError('terrain artifact from another map was accepted')

    broken = tmp_path / 'broken.terrain.npz'
    with broken.open('wb') as stream:
        np.savez_compressed(
            stream,
            schema=np.asarray(TERRAIN_GRID_SCHEMA),
            version=np.asarray(1, dtype=np.int64),
            map_name=np.asarray('campus'),
            frame_id=np.asarray('map'),
            resolution=np.asarray(0.2),
            width=np.asarray(3),
            height=np.asarray(2),
            origin_x=np.asarray(-1.0),
            origin_y=np.asarray(2.0),
            generation_json=np.asarray('{}'),
            elevation=np.zeros(5, dtype=np.float32),
            slope_deg=np.zeros(6, dtype=np.float32),
            roughness_m=np.zeros(6, dtype=np.float32),
            step_height_m=np.zeros(6, dtype=np.float32),
            confidence=np.ones(6, dtype=np.float32),
            traversal_cost=np.zeros(6, dtype=np.int8),
        )
    try:
        load_terrain_grid(broken, expected_map_name='campus')
    except ValueError as error:
        assert 'elevation' in str(error)
    else:
        raise AssertionError('terrain artifact with a truncated layer was accepted')


def test_localization_metadata_binds_artifact_to_exact_pcd(tmp_path):
    pcd_path = tmp_path / 'campus.pcd'
    pcd_path.write_bytes(b'validated localization point cloud')
    pcd_sha256 = hashlib.sha256(pcd_path.read_bytes()).hexdigest()
    terrain_path = tmp_path / 'campus.terrain.npz'
    summary = write_terrain_grid(
        terrain_path,
        map_name='campus',
        frame_id='map',
        grid=_grid(),
        generation={
            'algorithm': 'lowest_persistent_return_v1',
            'source_pcd_sha256': pcd_sha256,
        },
    )
    descriptor = create_terrain_grid_metadata(
        terrain_path,
        summary=summary,
        source_pcd_sha256=pcd_sha256,
    )
    metadata = {
        'pcd_sha256': pcd_sha256,
        'terrain_grid': descriptor,
    }

    loaded = load_bound_terrain_grid(
        tmp_path, map_name='campus', localization_metadata=metadata)
    assert loaded is not None
    assert loaded.grid.width == 3

    metadata['pcd_sha256'] = '0' * 64
    try:
        load_bound_terrain_grid(
            tmp_path, map_name='campus', localization_metadata=metadata)
    except ValueError as error:
        assert 'PCD' in str(error)
    else:
        raise AssertionError('terrain artifact bound to another PCD was accepted')


def test_bound_metadata_rejects_coerced_integer_types(tmp_path):
    pcd_path = tmp_path / 'campus.pcd'
    pcd_path.write_bytes(b'validated localization point cloud')
    pcd_sha256 = hashlib.sha256(pcd_path.read_bytes()).hexdigest()
    terrain_path = tmp_path / 'campus.terrain.npz'
    summary = write_terrain_grid(
        terrain_path,
        map_name='campus',
        frame_id='map',
        grid=_grid(),
        generation={'source_pcd_sha256': pcd_sha256},
    )
    descriptor = create_terrain_grid_metadata(
        terrain_path,
        summary=summary,
        source_pcd_sha256=pcd_sha256,
    )

    for field_name, invalid_value in (
            ('version', 1.0),
            ('size_bytes', float(descriptor['size_bytes'])),
            ('width', float(descriptor['width'])),
            ('known_cells', True)):
        broken_descriptor = dict(descriptor)
        broken_descriptor[field_name] = invalid_value
        try:
            load_bound_terrain_grid(
                tmp_path,
                map_name='campus',
                localization_metadata={
                    'pcd_sha256': pcd_sha256,
                    'terrain_grid': broken_descriptor,
                },
            )
        except ValueError as error:
            assert 'integer' in str(error)
        else:
            raise AssertionError(
                f'terrain metadata accepted non-integer {field_name}')


def test_current_map_signature_changes_only_when_artifact_changes(tmp_path):
    path = tmp_path / 'campus.terrain.npz'
    path.write_bytes(b'a')
    first = terrain_artifact_signature(path)
    assert terrain_artifact_signature(path) == first

    path.write_bytes(b'ab')
    assert terrain_artifact_signature(path) != first


def test_loader_rejects_noncanonical_storage_dtype(tmp_path):
    valid = tmp_path / 'valid.terrain.npz'
    write_terrain_grid(valid, map_name='valid', frame_id='map', grid=_grid())
    with np.load(valid, allow_pickle=False) as archive:
        payload = {name: np.asarray(archive[name]).copy() for name in archive.files}
    payload['traversal_cost'] = payload['traversal_cost'].astype(np.uint8)
    invalid = tmp_path / 'invalid.terrain.npz'
    with invalid.open('wb') as stream:
        np.savez_compressed(stream, **payload)

    try:
        load_terrain_grid(invalid, expected_map_name='valid')
    except ValueError as error:
        assert 'int8 storage' in str(error)
    else:
        raise AssertionError('terrain artifact with uint8 costs was accepted')


def test_missing_terrain_section_is_the_only_legacy_fallback(tmp_path):
    assert load_bound_terrain_grid(
        tmp_path,
        map_name='legacy',
        localization_metadata={'pcd_sha256': '0' * 64},
    ) is None

    try:
        load_bound_terrain_grid(
            tmp_path,
            map_name='legacy',
            localization_metadata={
                'pcd_sha256': '0' * 64,
                'terrain_grid': None,
            },
        )
    except ValueError as error:
        assert 'object' in str(error)
    else:
        raise AssertionError('explicit null terrain silently fell back')

    try:
        load_bound_terrain_grid(
            tmp_path,
            map_name='broken',
            localization_metadata={
                'pcd_sha256': '0' * 64,
                'terrain_grid': {},
            },
        )
    except ValueError as error:
        assert 'filename' in str(error)
    else:
        raise AssertionError('declared but invalid terrain silently fell back')
