"""The map-save terrain reader must be safe with old and current NumPy APIs."""
import hashlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import zipfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lzp_service import terrain_grid_io as terrain_io
from test_terrain_grid_io import _grid


def valid_artifact(tmp_path):
    path = tmp_path / 'campus.terrain.npz'
    terrain_io.write_terrain_grid(path, map_name='campus', frame_id='map', grid=_grid())
    return path


def legacy_numpy():
    original = np.load
    calls = []

    def load(file, mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII'):
        calls.append((file, allow_pickle))
        assert allow_pickle is False, 'compatibility must never enable pickle'
        return original(file, mmap_mode=mmap_mode, allow_pickle=allow_pickle,
                        fix_imports=fix_imports, encoding=encoding)

    return mock.patch.object(terrain_io.np, 'load', load), calls


def replace_member(path, member, value):
    with zipfile.ZipFile(path) as archive:
        contents = {name:archive.read(name) for name in archive.namelist()}
    contents[member] = value
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in contents.items(): archive.writestr(name, data)


class TerrainNumpyCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.valid = valid_artifact(self.directory)

    def test_round_trip_with_legacy_numpy_signature(self):
        patch, calls = legacy_numpy()
        with patch:
            loaded = terrain_io.load_terrain_grid(self.valid, expected_map_name='campus')
        self.assertEqual(loaded.map_name, 'campus')
        np.testing.assert_allclose(loaded.grid.elevation, _grid().elevation, equal_nan=True)
        self.assertEqual(len(calls), 1)

    def test_bound_map_save_artifact_with_legacy_numpy(self):
        pcd = self.directory / 'campus.pcd'
        pcd.write_bytes(b'validated localization point cloud')
        digest = hashlib.sha256(pcd.read_bytes()).hexdigest()
        patch, calls = legacy_numpy()
        with patch:
            summary = terrain_io.write_terrain_grid(self.valid, map_name='campus', frame_id='map',
                grid=_grid(), generation={'source_pcd_sha256':digest})
            descriptor = terrain_io.create_terrain_grid_metadata(self.valid, summary=summary,
                                                                source_pcd_sha256=digest)
            loaded = terrain_io.load_bound_terrain_grid(self.directory, map_name='campus',
                localization_metadata={'pcd_sha256':digest, 'terrain_grid':descriptor})
        self.assertEqual(loaded.map_name, 'campus')
        self.assertEqual(len(calls), 2)  # writer read-back and bound metadata verification

    def test_npy_versions_remain_readable_on_legacy_api(self):
        for version in ((1,0),(2,0),(3,0)):
            with self.subTest(version=version):
                buffer = io.BytesIO()
                np.lib.format.write_array(buffer, _grid().elevation, version=version, allow_pickle=False)
                replace_member(self.valid, 'elevation.npy', buffer.getvalue())
                patch, _ = legacy_numpy()
                with patch: loaded = terrain_io.load_terrain_grid(self.valid)
                np.testing.assert_allclose(loaded.grid.elevation, _grid().elevation, equal_nan=True)

    def test_oversized_npy_header_rejected_before_numpy(self):
        for version, length_bytes in (((1,0),2),((2,0),4),((3,0),4)):
            with self.subTest(version=version):
                content = b'\x93NUMPY' + bytes(version) + (4097).to_bytes(length_bytes,'little') + b' '
                replace_member(self.valid, 'elevation.npy', content)
                with mock.patch.object(terrain_io.np, 'load', side_effect=AssertionError(
                        'NumPy reached before bounded preflight')) as load:
                    with self.assertRaisesRegex(ValueError, 'NPY header.*limit'):
                        terrain_io.load_terrain_grid(self.valid)
                load.assert_not_called()

    def test_npy_header_at_exact_limit_remains_readable(self):
        for version, length_bytes in (((1,0),2),((2,0),4),((3,0),4)):
            with self.subTest(version=version):
                buffer = io.BytesIO()
                np.lib.format.write_array(buffer, _grid().elevation, version=version, allow_pickle=False)
                raw = buffer.getvalue()
                offset = 8 + length_bytes
                length = int.from_bytes(raw[8:offset], 'little')
                header = raw[offset:offset+length]
                padded = header[:-1] + b' ' * (4096-length) + b'\n'
                content = raw[:8] + (4096).to_bytes(length_bytes,'little') + padded + raw[offset+length:]
                replace_member(self.valid, 'elevation.npy', content)
                patch, _ = legacy_numpy()
                with patch: loaded = terrain_io.load_terrain_grid(self.valid)
                np.testing.assert_allclose(loaded.grid.elevation, _grid().elevation, equal_nan=True)

    def test_invalid_or_truncated_npy_header_rejected_before_numpy(self):
        for content in (b'', b'NOTNPY\x01\x00\x01\x00\n', b'\x93NUMPY\x09\x00',
                b'\x93NUMPY\x01\x00\x01', b'\x93NUMPY\x01\x00\x10\x00{',
                b'\x93NUMPY\x02\x00\x10\x00\x00\x00{'):
            with self.subTest(content=content):
                replace_member(self.valid, 'elevation.npy', content)
                with mock.patch.object(terrain_io.np, 'load', side_effect=AssertionError(
                        'NumPy reached before bounded preflight')) as load:
                    with self.assertRaisesRegex(ValueError, 'NPY'):
                        terrain_io.load_terrain_grid(self.valid)
                load.assert_not_called()

    def test_object_arrays_still_rejected_on_legacy_api(self):
        buffer = io.BytesIO()
        np.lib.format.write_array(buffer, np.asarray('campus', dtype=object), allow_pickle=True)
        replace_member(self.valid, 'map_name.npy', buffer.getvalue())
        patch, calls = legacy_numpy()
        with patch, self.assertRaisesRegex(ValueError, '(?i)object'):
            terrain_io.load_terrain_grid(self.valid)
        self.assertTrue(calls)

    def test_numpy_uses_same_open_file_as_preflight(self):
        original = np.load

        def load(file, **kwargs):
            self.assertTrue(hasattr(file, 'read'), 'preflight and NumPy must share an open file')
            self.assertEqual(file.tell(), 0)
            return original(file, **kwargs)

        with mock.patch.object(terrain_io.np, 'load', load):
            self.assertEqual(terrain_io.load_terrain_grid(self.valid).map_name, 'campus')


if __name__ == '__main__': unittest.main()
