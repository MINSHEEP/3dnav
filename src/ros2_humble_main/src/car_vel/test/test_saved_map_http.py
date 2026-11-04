"""Complete saved-file contract, temporary files only; no ROS or robot I/O."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import threading
from urllib.request import urlopen, Request
from urllib.parse import urlencode
from urllib.error import HTTPError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.saved_map_http import SavedMapRepository, SavedMapError, SavedMapServer


class SavedMapTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.state = {'map_name': 'room', 'load_id': 'load-1', 'stamp_ns': '1000000001'}
        self.repo = SavedMapRepository(self.root, lambda: dict(self.state))
        self.write_map(150001)

    def write_map(self, count):
        import struct
        header = (f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
                  f'COUNT 1 1 1 1\nWIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA binary\n').encode()
        self.raw = header + struct.pack('<ffff', 1, 2, 3, 9) * count
        (self.root / 'room.pcd').write_bytes(self.raw)
        self.metadata = dict(schema='ros2_mid360_localization_map', version=1,
            map_name='room', frame_id='map', pcd_file='room.pcd', point_count=count,
            pcd_size_bytes=len(self.raw), pcd_sha256=hashlib.sha256(self.raw).hexdigest())
        self.write_metadata()

    def write_metadata(self):
        (self.root / 'room.localization.yaml').write_text(json.dumps(self.metadata), encoding='utf-8')

    def manifest(self):
        return self.repo.manifest(**self.state)

    def test_all_file_bytes_and_points_are_preserved_beyond_preview_cap(self):
        info = self.manifest()
        self.assertEqual(info['point_count'], 150001)
        self.assertEqual(info['size_bytes'], len(self.raw))
        with self.repo.open_file(**self.state, sha256=info['sha256']) as stream:
            self.assertEqual(stream.read(), self.raw)

    def test_pending_cloud_edit_blocks_even_previously_cached_manifest(self):
        self.manifest()
        marker=self.root/'.room.cloud-edit-pending'
        marker.write_text('{}',encoding='utf-8')
        with self.assertRaises(SavedMapError): self.manifest()
        marker.unlink()
        self.assertEqual(self.manifest()['point_count'],150001)

    def test_manifest_bootstraps_backend_stamp_for_late_browser(self):
        info=self.repo.manifest(map_name='room',load_id='load-1')
        self.assertEqual(info['stamp_ns'],self.state['stamp_ns'])
        self.assertEqual(info['point_count'],150001)

    def test_wrong_active_load_or_stamp_never_returns_another_map(self):
        for field in ['map_name', 'load_id', 'stamp_ns']:
            args = dict(self.state); args[field] = 'other'
            with self.assertRaises(SavedMapError): self.repo.manifest(**args)

    def test_file_change_same_name_invalidates_old_digest(self):
        old = self.manifest()
        self.write_map(150002)
        new = self.manifest()
        self.assertNotEqual(old['sha256'], new['sha256'])
        with self.assertRaises(SavedMapError):
            with self.repo.open_file(**self.state, sha256=old['sha256']): pass

    def test_hash_count_and_coordinate_metadata_fail_closed(self):
        for field, value in [('pcd_sha256', '0'*64), ('point_count', 7), ('frame_id', 'odom')]:
            original = self.metadata[field]
            self.metadata[field] = value; self.write_metadata()
            with self.assertRaises(SavedMapError): self.manifest()
            self.metadata[field] = original
        self.write_metadata()

    def test_no_path_traversal_or_unconfirmed_identity(self):
        for name in ['../room', '/room', 'a\\room', '..']:
            with self.assertRaises(SavedMapError):
                self.repo.manifest(map_name=name, load_id='load-1', stamp_ns='1000000001')
        self.state.clear()
        with self.assertRaises(SavedMapError):
            self.repo.manifest(map_name='room', load_id='load-1', stamp_ns='1000000001')


class HttpTests(unittest.TestCase):
    write_map=SavedMapTests.write_map
    write_metadata=SavedMapTests.write_metadata

    def setUp(self):
        SavedMapTests.setUp(self)
        self.server=SavedMapServer(('127.0.0.1',0),self.repo)
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.base='http://127.0.0.1:'+str(self.server.server_port)

    def test_actual_manifest_and_original_binary_response(self):
        query=urlencode(self.state)
        with urlopen(self.base+'/api/saved-map/manifest?'+query,timeout=3) as response:
            info=json.load(response)
        with urlopen(self.base+'/api/saved-map/file?'+query+'&sha256='+info['sha256'],timeout=3) as response:
            self.assertEqual(response.headers['Content-Length'],str(len(self.raw)))
            self.assertEqual(response.read(),self.raw)
        self.assertEqual(info['point_count'],150001)

    def test_unrelated_origin_and_stale_load_cannot_read_file(self):
        request=Request(self.base+'/api/saved-map/manifest?'+urlencode(self.state),
            headers={'Origin':'http://unrelated.test'})
        with self.assertRaises(HTTPError) as error:urlopen(request,timeout=3)
        self.assertEqual(error.exception.code,403)
        args=dict(self.state);args['load_id']='stale'
        with self.assertRaises(HTTPError) as error:
            urlopen(self.base+'/api/saved-map/manifest?'+urlencode(args),timeout=3)
        self.assertEqual(error.exception.code,409)


if __name__ == '__main__': unittest.main()
