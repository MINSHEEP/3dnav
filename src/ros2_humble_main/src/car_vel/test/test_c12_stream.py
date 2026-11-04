"""C12 pipe framing and real HTTP handlers; no camera or motor access."""
from argparse import Namespace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest import mock

import pytest
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'hikvision_camera'))
import camera_web
from c12_stream import C12Source, C12Streams, jpeg_frames
from ordinary_camera import CameraSourceStore

JPEG = cv2.imencode('.jpg', np.full((12, 16, 3), 100, dtype=np.uint8))[1].tobytes()


def wait_until(predicate, seconds=3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('C12 worker did not reach expected state')


@pytest.fixture
def streams(tmp_path, monkeypatch):
    # Execute a real child/pipe while replacing only the absent RTSP decoder.
    decoder = tmp_path / 'ffmpeg'
    decoder.write_text(f'#!{sys.executable}\nimport os, time\n'
                       f'while True:\n os.write(1, {JPEG!r}); time.sleep(.03)\n')
    decoder.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    bridge = C12Streams(camera_web.LatestJpegFrame)
    yield bridge
    bridge.close()


@pytest.fixture
def api(streams):
    server = camera_web.CameraWebServer(('127.0.0.1', 0), camera_web.CameraRequestHandler,
                                        max_stream_clients=1)
    server.gateway = Namespace(c12=streams, next_frame=lambda _version: (1, JPEG))
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    worker.start()
    yield server, f'http://127.0.0.1:{server.server_port}'
    server.shutdown()
    streams.close()
    server.server_close()
    worker.join(2)


def test_jpeg_split_markers_and_multiple_frames():
    buffer = bytearray(b'garbage\xff')
    assert list(jpeg_frames(buffer)) == []
    buffer.extend(JPEG[1:-1])
    assert list(jpeg_frames(buffer)) == []
    buffer.extend(JPEG[-1:] + JPEG)
    assert list(jpeg_frames(buffer)) == [JPEG, JPEG]
    assert len(buffer) <= 1


def test_two_channels_are_lazy_and_share_process(streams):
    assert not any(value['running'] for value in streams.status().values())
    source = streams.sources['visible']
    version, image = source.next_frame()
    assert image == JPEG
    process = source._process
    assert source.next_frame(version)[1] == JPEG
    assert source._process is process
    assert not streams.status()['infrared']['running']
    assert streams.sources['infrared'].next_frame()[1] == JPEG
    assert streams.sources['infrared']._process is not process
    streams.close()
    assert process.poll() is not None


def test_idle_releases_process_and_reopening_reconnects(streams, monkeypatch):
    source = streams.sources['visible']
    source.next_frame()
    process = source._process
    monkeypatch.setattr(source, 'IDLE_SECONDS', .1)
    wait_until(lambda: not source._worker.is_alive())
    assert process.poll() is not None
    monkeypatch.setattr(source, 'IDLE_SECONDS', 20)
    assert source.next_frame()[1] == JPEG
    assert source._process is not process


def test_status_snapshot_mjpeg_and_original_budget(api):
    server, base = api
    with urlopen(base + '/api/c12/status', timeout=2) as response:
        assert b'/api/c12/visible/stream.mjpg' in response.read()
    # Occupying every original camera slot must not disable C12.
    assert server.stream_slots.acquire(blocking=False)
    try:
        for channel in ('visible', 'infrared'):
            with urlopen(base + f'/api/c12/{channel}/frame.jpg', timeout=3) as response:
                assert response.headers['Access-Control-Allow-Origin'] == '*'
                assert response.headers['Content-Type'] == 'image/jpeg'
                assert response.read() == JPEG
            with urlopen(base + f'/api/c12/{channel}/stream.mjpg', timeout=3) as response:
                assert response.headers['Content-Type'].startswith('multipart/x-mixed-replace')
                assert response.readline() == b'--frame\r\n'
                assert response.readline() == b'Content-Type: image/jpeg\r\n'
                assert response.readline() == f'Content-Length: {len(JPEG)}\r\n'.encode()
                assert response.readline() == b'\r\n'
                assert response.read(len(JPEG)) == JPEG
    finally:
        server.stream_slots.release()
    with urlopen(base + '/api/stream', timeout=2) as response:
        assert response.readline() == b'--frame\r\n'


def test_offline_source_returns_503_without_stale_frames(api):
    server, base = api
    source = server.gateway.c12.sources['visible']
    with mock.patch('c12_stream.shutil.which', return_value=None):
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/api/c12/visible/frame.jpg', timeout=3)
        assert error.value.code == 503
        assert 'FFmpeg' in error.value.read().decode()
    with pytest.raises(HTTPError) as error:
        urlopen(base + '/api/c12/unknown/frame.jpg', timeout=2)
    assert error.value.code == 404


@pytest.fixture
def gateway(tmp_path, streams):
    class LegacyCamera(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(JPEG)))
            self.end_headers()
            self.wfile.write(JPEG)
        def log_message(self, *_args):
            pass
    upstream = ThreadingHTTPServer(('127.0.0.1', 0), LegacyCamera)
    thread = threading.Thread(target=upstream.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    thread.start()
    config = tmp_path / 'ordinary.json'
    store = CameraSourceStore(config, {'body_url': f'http://127.0.0.1:{upstream.server_port}/image'})
    store.save({**store.current, 'source': 'body'}, expected_version=0)
    args = Namespace(speed=4, camera_type='usb', camera_source_config=config,
                     video_start_timeout=2, behavior_config=tmp_path / 'behavior.json')
    instance = camera_web.CameraGateway(args)
    instance._thermal = mock.Mock()
    instance._thermal.status.return_value = {'ready': True, 'marker': 'original-thermal'}
    instance._yolo = mock.Mock()
    instance._yolo.status.return_value = {'running': False}
    instance._wait_for_first_frame(instance._start_usb_source())
    try:
        yield instance
    finally:
        instance.close()
        upstream.shutdown()
        upstream.server_close()
        thread.join(2)


def test_real_pipeline_http_roundtrip_preserves_legacy_and_shared_c12(gateway):
    server = camera_web.CameraWebServer(('127.0.0.1', 0), camera_web.CameraRequestHandler)
    server.gateway = gateway
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    worker.start()
    base = f'http://127.0.0.1:{server.server_port}'
    original = gateway.camera_source_status()['config']
    assert not gateway.c12.status()['visible']['running']
    try:
        for pipeline in ('c12', 'legacy'):
            current = gateway.camera_source_status()['config']
            payload = dict(config={**current, 'pipeline': pipeline}, expected_version=current['version'])
            request = Request(base + '/api/camera/source', data=json.dumps(payload).encode(),
                              headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=5) as response:
                changed = json.load(response)
            assert changed['config']['pipeline'] == pipeline
            assert changed['config']['source'] == original['source']
            assert changed['config']['usb_device'] == original['usb_device']
            assert changed['config']['body_url'] == original['body_url']
            with urlopen(base + '/api/snapshot', timeout=3) as response:
                assert response.read() == JPEG
            if pipeline == 'c12':
                assert gateway.video.source is gateway.c12.sources['visible']
                with urlopen(base + '/api/inspection/state', timeout=3) as response:
                    inspection = json.load(response)
                assert inspection['camera_type'] == 'c12'
                assert inspection['camera_source']['pipeline'] == 'c12'
                assert inspection['thermal']['max_temperature_c'] is None
                assert inspection['thermal']['ready'] is False
                assert inspection['thermal']['alarm_active'] is False
                assert 'marker' not in inspection['thermal']
                gateway._legacy_thermal.reset_mock()
                with urlopen(base + '/api/display/status', timeout=3) as response:
                    display = json.load(response)
                assert display['infrared_label'] == 'C12 红外'
                assert display['thermal']['supports_temperature'] is False
                assert display['thermal']['max_temperature_c'] is None
                with urlopen(base + '/api/thermal/frame.jpg', timeout=3) as response:
                    assert response.read() == JPEG
                with urlopen(base + '/api/thermal/stream.mjpg', timeout=3) as response:
                    assert response.readline() == b'--frame\r\n'
                with pytest.raises(RuntimeError, match='测温数据未接入'):
                    gateway._thermal.set_detection(True)
                with pytest.raises(RuntimeError, match='测温数据未接入'):
                    gateway.update_thermal_config({'threshold_c': 42})
                assert gateway._thermal.events(0)['events'] == []
                assert gateway._legacy_thermal.mock_calls == []
                process = gateway.video.source._process
        assert gateway.c12.sources['visible']._process is process
        assert gateway.c12.sources['visible'].next_frame()[1] == JPEG
        gateway._thermal.set_detection.assert_not_called()
        assert gateway.thermal_status()['marker'] == 'original-thermal'
        assert gateway.display_status()['thermal']['supports_temperature'] is True
    finally:
        server.shutdown()
        server.server_close()
        worker.join(2)


def test_failed_c12_switch_restores_real_legacy_source(gateway):
    original = gateway.camera_source_status()['config']
    with mock.patch('c12_stream.shutil.which', return_value=None):
        with pytest.raises(RuntimeError, match='FFmpeg'):
            gateway.configure_camera_source(dict(config={**original, 'pipeline': 'c12'},
                                                  expected_version=original['version']))
    assert gateway.camera_source_status()['config'] == original
    assert CameraSourceStore(gateway.args.camera_source_config).active() == original
    assert gateway.snapshot() == JPEG
