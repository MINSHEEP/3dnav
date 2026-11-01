"""Public configuration and gateway transactions for ordinary camera switching."""
from argparse import Namespace
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest import mock

CAMERA = Path(__file__).resolve().parents[1] / 'hikvision_camera'
sys.path.insert(0, str(CAMERA))
import camera_web
from ordinary_camera import CameraSourceStore, validate_config, discover_usb_cameras


class Source:
    def __init__(self, config, fail=False):
        self.config = copy.deepcopy(config)
        self.fail = fail
        self.closed = False
    def start(self): pass
    def wait_for_first_frame(self, timeout):
        if self.fail: raise TimeoutError('no first frame')
    def close(self): self.closed = True
    def info(self):
        return {'backend': 'http' if self.config['source'] == 'body' else 'v4l2',
                'error': None, 'encoded_frames': 1}
    def next_frame(self, version): return 1, self.latest_frame()
    def latest_frame(self): return self.config['source'].encode()


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / 'ordinary_camera.json'
    def tearDown(self): self.tmp.cleanup()
    def test_persistent_selection_and_optimistic_version(self):
        store = CameraSourceStore(self.path)
        self.assertEqual(store.current['source'], 'usb')
        draft = {**store.current, 'source': 'body'}
        saved = store.save(draft, expected_version=0)
        self.assertEqual(saved['version'], 1)
        self.assertEqual(CameraSourceStore(self.path).current, saved)
        with self.assertRaises(ValueError): store.save(draft, expected_version=0)
        self.assertEqual(CameraSourceStore(self.path).current, saved)
    def test_invalid_config_and_thermal_device_are_rejected(self):
        base = CameraSourceStore(None).current
        for patch in ({'source':'other'}, {'body_url':'file:///etc/passwd'},
                      {'body_url':'http://user:secret@192.168.96.2:8080'},
                      {'body_url':'http://127.0.0.1:8766/api/stream'},
                      {'body_url':'http://127.0.0.1:8767/api/stream.mjpg'},
                      {'usb_device':'/dev/v4l/by-id/usb-HIK_HikCamera_F1-video-index0'},
                      {'usb_device':'/dev/ttyUSB0'}, {'usb_fps':float('nan')},
                      {'usb_width':True}, {'usb_height':0}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                validate_config({**base, **patch})
    def test_corrupt_config_fails_closed_instead_of_selecting_usb(self):
        self.path.write_text('{broken', encoding='utf-8')
        store = CameraSourceStore(self.path)
        self.assertTrue(store.error)
        with self.assertRaises(RuntimeError): store.active()
        saved = store.save({**store.current, 'source':'body'}, expected_version=0)
        self.assertEqual(CameraSourceStore(self.path).active(), saved)
        self.assertEqual(len(list(self.path.parent.glob('ordinary_camera.json.invalid-*'))), 1)
    def test_sysfs_discovery_excludes_thermal_and_metadata(self):
        root = Path(self.tmp.name)
        for number, product, index, vid, pid in (
            (0,'HikCamera',0,'2bdf','0102'),
            (1,'HikCamera',1,'2bdf','0102'),
            (2,'USB Camera',0,'2bc5','050f'),
            (3,'USB Camera',1,'2bc5','050f')):
            entry = root / 'sys' / f'video{number}'
            (entry / 'device').mkdir(parents=True)
            for name,value in (('name',product),('index',str(index))):
                (entry/name).write_text(value)
            for name,value in (('idVendor',vid),('idProduct',pid)):
                (entry/'device'/name).write_text(value)
        items = discover_usb_cameras(sys_root=root/'sys', dev_root=root/'dev')
        self.assertEqual([d['node'] for d in items], ['/dev/video2'])


class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.args = Namespace(speed=4, camera_type='usb', usb_device='/dev/video2',
            usb_width=1280, usb_height=720, usb_fps=30.0, usb_fourcc='MJPG',
            video_start_timeout=0.05,
            camera_source_config=Path(self.tmp.name)/'source.json')
        self.sources = []
        self.fail_body = False
        def factory(config):
            result = Source(config, self.fail_body and config['source']=='body')
            self.sources.append(result)
            return result
        self.gateway = camera_web.CameraGateway(self.args, ordinary_source_factory=factory)
        self.gateway._recognition_store = mock.MagicMock()
        self.gateway._recognition_store.return_value.cleanup_expired.return_value = {}
        self.gateway._thermal = mock.MagicMock()
        self.gateway._yolo = mock.MagicMock()
        self.gateway._yolo.status.return_value = {'running':False}
        self.gateway.connect()
    def tearDown(self):
        self.gateway.close()
        self.tmp.cleanup()
    def switch(self, **updates):
        config = self.gateway.camera_source_status()['config']
        return self.gateway.configure_camera_source({
            'config':{**config, **updates}, 'expected_version':config['version']})
    def test_switch_releases_old_and_persists_only_after_first_frame(self):
        old = self.gateway.video
        result = self.switch(source='body')
        self.assertTrue(result['success'])
        self.assertTrue(old.closed)
        self.assertEqual(self.gateway.snapshot(), b'body')
        self.assertEqual(CameraSourceStore(self.args.camera_source_config).active()['source'], 'body')
        self.gateway._thermal.set_detection.assert_not_called()
    def test_failed_connection_rolls_back_and_does_not_save(self):
        self.fail_body = True
        old_config = self.gateway.camera_source_status()['config']
        with self.assertRaisesRegex(RuntimeError, '首帧超时'):
            self.switch(source='body')
        self.assertEqual(self.gateway.camera_source_status()['config'], old_config)
        self.assertEqual(self.gateway.snapshot(), b'usb')
        self.assertTrue(self.sources[1].closed)
        self.assertFalse(self.args.camera_source_config.exists())
    def test_failed_disk_save_restores_previous_stream(self):
        with mock.patch('ordinary_camera.os.replace', side_effect=OSError('disk full')):
            with self.assertRaisesRegex(RuntimeError, 'disk full'):
                self.switch(source='body')
        self.assertEqual(self.gateway.snapshot(), b'usb')
        self.assertEqual(self.gateway.camera_source_status()['config']['source'], 'usb')
    def test_activity_and_stale_version_do_not_close_camera(self):
        old = self.gateway.video
        for mode, task, running in (('manual','',True),('transit','t1',False),
                                    ('paused','t1',False),('inspection','t1',False)):
            self.gateway._inspection_mode, self.gateway._inspection_task_id = mode, task
            self.gateway._yolo.status.return_value = {'running':running}
            with self.assertRaises(RuntimeError): self.switch(source='body')
            self.assertFalse(old.closed)
        self.gateway._inspection_mode, self.gateway._inspection_task_id = 'manual',''
        self.gateway._yolo.status.return_value = {'running':False}
        config = self.gateway.camera_source_status()['config']
        with self.assertRaises(ValueError):
            self.gateway.configure_camera_source({'config':config,'expected_version':999})
        self.assertFalse(old.closed)
    def test_restart_uses_saved_body_source_and_never_opens_usb(self):
        self.switch(source='body')
        self.gateway.restart_video()
        self.assertEqual(self.sources[-1].config['source'], 'body')
        self.assertEqual(self.gateway.snapshot(), b'body')

    def test_new_process_uses_persisted_body_selection(self):
        self.switch(source='body')
        self.gateway.close()
        created = []
        def factory(config):
            created.append(config['source'])
            return Source(config)
        self.gateway = camera_web.CameraGateway(self.args, ordinary_source_factory=factory)
        self.gateway._yolo = mock.MagicMock()
        self.gateway.connect()
        self.assertEqual(created, ['body'])
        self.assertEqual(self.gateway.snapshot(), b'body')

    def test_unreleased_old_source_never_opens_competing_source(self):
        old = self.gateway.video
        old.close = mock.Mock(side_effect=RuntimeError('USB read blocked'))
        with self.assertRaisesRegex(RuntimeError, 'USB read blocked'): self.switch(source='body')
        self.assertEqual(len(self.sources), 1)
        self.assertEqual(self.gateway.camera_source_status()['config']['version'], 0)
        self.assertIs(self.gateway._ordinary_pending_close, old)
        old.close = mock.Mock()
        self.switch(source='body')
        old.close.assert_called_once()
        self.assertEqual(self.gateway.snapshot(), b'body')

    def test_old_source_frame_in_flight_is_not_returned_after_switch(self):
        old = self.gateway.video
        def in_flight():
            self.switch(source='body')
            return b'old-usb'
        old.latest_frame = in_flight
        with self.assertRaisesRegex(RuntimeError, '已切换'): self.gateway.snapshot()
        self.assertEqual(self.gateway.snapshot(), b'body')

    def test_public_http_configuration_and_thermal_routes(self):
        server = camera_web.CameraWebServer(('127.0.0.1',0), camera_web.CameraRequestHandler)
        server.gateway = self.gateway
        worker = threading.Thread(target=lambda: server.serve_forever(poll_interval=0.02), daemon=True)
        worker.start()
        base = f'http://127.0.0.1:{server.server_port}'
        self.gateway._thermal.status.return_value = {'ready':True, 'marker':'independent'}
        try:
            with urlopen(base + '/api/camera/source', timeout=2) as response:
                current = json.load(response)
            payload = dict(config={**current['config'],'source':'body'},expected_version=0)
            request = Request(base + '/api/camera/source',data=json.dumps(payload).encode(),
                headers={'Content-Type':'application/json'})
            with urlopen(request, timeout=2) as response: result = json.load(response)
            self.assertEqual(result['config']['source'], 'body')
            self.assertFalse(result['switching'])
            with self.assertRaises(HTTPError) as caught: urlopen(request,timeout=2)
            self.assertEqual(caught.exception.code, 400)
            with urlopen(base + '/api/thermal/status',timeout=2) as response:
                thermal = json.load(response)
            self.assertEqual(thermal['marker'], 'independent')
            self.gateway._thermal.set_detection.assert_not_called()
        finally:
            server.shutdown(); server.server_close(); worker.join(timeout=2)


if __name__ == '__main__': unittest.main()
