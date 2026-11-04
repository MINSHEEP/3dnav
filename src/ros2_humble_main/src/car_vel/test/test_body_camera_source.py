"""Offline HTTP integration: the real robot proxy and shared ordinary source."""
import ast
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import ipaddress
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / 'hikvision_camera'))
from car_vel.camera_stream_lease import CameraStreamLease
from camera_web import LatestJpegFrame
from ordinary_camera import HttpCameraSource

# Load the production HTTP class without starting ROS, sockets or a robot SDK.
tree = ast.parse((PACKAGE / 'car_vel/dog_remote_bridge.py').read_text(encoding='utf-8'))
proxy_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'CameraProxyHandler')
namespace = dict(BaseHTTPRequestHandler=BaseHTTPRequestHandler, ipaddress=ipaddress,
    json=json, urlsplit=urlsplit, Request=Request, urlopen=urlopen, URLError=URLError)
exec(compile(ast.Module(body=[proxy_class], type_ignores=[]), '<production-camera-proxy>', 'exec'), namespace)
CameraProxyHandler = namespace['CameraProxyHandler']
JPEG = b'\xff\xd8offline-test-frame\xff\xd9'


@contextmanager
def server(handler):
    instance = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    instance.daemon_threads = True
    worker = threading.Thread(target=lambda: instance.serve_forever(poll_interval=0.02), daemon=True)
    worker.start()
    try:
        yield f'http://127.0.0.1:{instance.server_port}'
    finally:
        instance.shutdown()
        instance.server_close()
        worker.join(timeout=2)


def wait_until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(0.01)
    raise AssertionError('timed out waiting for offline HTTP worker')


class Upstream(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html>login</html>')
            return
        if self.path == '/redirect':
            self.send_response(302)
            self.send_header('Location', '/snapshot')
            self.end_headers()
            return
        self.send_response(200)
        if self.path == '/snapshot':
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(JPEG)))
            self.end_headers()
            self.wfile.write(JPEG)
            return
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=robot')
        self.end_headers()
        try:
            for _ in range(150):
                self.wfile.write(b'--robot\r\nContent-Type: image/jpeg\r\n\r\n' + JPEG + b'\r\n')
                self.wfile.flush()
                time.sleep(0.025)
        except OSError:
            pass
    def log_message(self, *args): pass


class BodySourceTests(unittest.TestCase):
    @staticmethod
    def source(url):
        return HttpCameraSource(url, LatestJpegFrame(), decoder=lambda image: (640,480))

    def test_snapshot_first_frame_and_close_invalidation(self):
        with server(Upstream) as base:
            source = self.source(base + '/snapshot')
            source.start()
            try:
                self.assertEqual(source.latest_frame(1), JPEG)
                self.assertEqual(source.info()['width'], 640)
                self.assertFalse(source.error)
            finally: source.close()
            with self.assertRaises(RuntimeError): source.latest_frame(0.01)

    def test_real_proxy_mjpeg_acquires_renews_and_releases_camera_only(self):
        commands = []
        lease = CameraStreamLease(lambda enabled: commands.append(enabled))
        with server(Upstream) as upstream:
            proxy = type('TestProxy', (CameraProxyHandler,), {
                'upstream_url':upstream + '/stream',
                'session_callback':staticmethod(lease.request)})
            with server(proxy) as base:
                source = self.source(base + '/camera')
                source.start()
                try:
                    self.assertEqual(source.latest_frame(2), JPEG)
                    self.assertEqual(commands, [True])
                    self.assertFalse(lease._remote)
                    first_expiry = lease._leases[source._token]
                    source._renew_at = 0
                    wait_until(lambda: lease._leases[source._token] > first_expiry)
                    self.assertGreater(source.info()['encoded_frames'], 0)
                finally: source.close()
                self.assertEqual(commands, [True, False])
                self.assertFalse(lease._leases)
                self.assertFalse(lease._remote)

    def test_broken_upstream_drops_previous_cache(self):
        with server(Upstream) as base:
            source = self.source(base + '/html')
            source.frames.publish(b'old-camera-frame')
            source.start()
            try:
                wait_until(lambda: bool(source.error))
                self.assertIn('JPEG/MJPEG', source.error)
                with self.assertRaises(RuntimeError): source.latest_frame(0.01)
                self.assertEqual(source.encoded_frames, 0)
            finally: source.close()

    def test_redirect_is_rejected_without_publishing_another_endpoint(self):
        with server(Upstream) as base:
            source = self.source(base + '/redirect')
            source.start()
            try:
                wait_until(lambda: bool(source.error))
                self.assertIn('重定向', source.error)
                self.assertEqual(source.encoded_frames, 0)
            finally: source.close()

    def test_fragmented_mjpeg_and_oversize_are_bounded(self):
        class Fragments(io.BytesIO):
            headers = {'Content-Type':'multipart/x-mixed-replace; boundary=test'}
            def read1(self, size): return self.read(min(size, 3))
        source = self.source('http://192.0.2.1/stream')
        with self.assertRaisesRegex(RuntimeError, '断开'):
            source._read(Fragments(b'header\r\n' + JPEG + b'\r\n'))
        self.assertEqual(source.frames.latest(0.01), JPEG)
        source.MAX_FRAME = 20
        with self.assertRaisesRegex(RuntimeError, '过大'):
            source._read(Fragments(b'\xff\xd8' + b'x' * 30))

    def test_proxy_validation_returns_utf8_json_not_a_broken_status_line(self):
        lease = CameraStreamLease(lambda enabled: None)
        proxy = type('TestProxy', (CameraProxyHandler,), {'session_callback':staticmethod(lease.request)})
        with server(proxy) as base:
            request = Request(base + '/camera/session', data=b'{}',
                              headers={'Content-Type':'application/json'})
            with self.assertRaises(HTTPError) as caught: urlopen(request, timeout=1)
            self.assertEqual(caught.exception.code, 400)
            value = json.loads(caught.exception.read())
            self.assertFalse(value['success'])
            self.assertIn('字段无效', value['error'])
            self.assertFalse(lease._leases)

    def test_bridge_camera_setter_sends_only_stream_command(self):
        bridge_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'DogRemoteBridge')
        method = next(node for node in bridge_class.body if isinstance(node, ast.FunctionDef) and node.name == '_set_camera_stream')
        scope = {'CMD_CAMERA_STREAM':9}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<production-camera-command>', 'exec'), scope)
        commands = []
        class Bridge:
            state_lock = threading.RLock()
            remote_active = False
            requested_velocity = (0.0, 0.0, 0.0)
            def _send_command(self, code, value):
                commands.append((code,value)); return True
        bridge = Bridge()
        self.assertTrue(scope['_set_camera_stream'](bridge, True))
        self.assertEqual(commands, [(9,1)])
        self.assertFalse(bridge.remote_active)
        self.assertEqual(bridge.requested_velocity, (0.0,0.0,0.0))


if __name__ == '__main__': unittest.main()
