import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.request import Request, build_opener, ProxyHandler


MODULE_DIR = Path(__file__).resolve().parents[1] / "hikvision_camera"
sys.path.insert(0, str(MODULE_DIR))

from thermal_monitor import ThermalConfigStore, ThermalMonitor, ThermalSample  # noqa: E402
from thermal_web import ThermalHttpServer  # noqa: E402


JPEG = b"\xff\xd8thermal\xff\xd9"


class ThermalWebTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        store = ThermalConfigStore(Path(self.temporary.name) / "config.json")
        self.monitor = ThermalMonitor(store, hold_seconds=0.0)
        self.monitor.set_backend_state(running=True, sdk_version="test", device="fake")
        self.monitor.ingest(ThermalSample(
            raw_jpeg=JPEG, annotated_jpeg=JPEG, max_temperature_c=42.0,
            x_normalized=500, y_normalized=500, frame_number=1,
            captured_at_epoch=__import__("time").time(), width=256, height=192))
        self.server = ThermalHttpServer(("127.0.0.1", 0), self.monitor)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.temporary.cleanup()

    def request(self, path, payload=None):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(
            self.base + path, data=data,
            headers={"Content-Type": "application/json"} if data else {},
            method="POST" if data else "GET")
        with self.opener.open(request, timeout=2) as response:
            return response.status, response.headers, response.read()

    def test_status_frame_and_versioned_config_routes(self):
        status, _headers, body = self.request("/api/status")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["ready"])
        status, headers, body = self.request("/api/frame.jpg")
        self.assertEqual(status, 200)
        self.assertEqual(headers.get_content_type(), "image/jpeg")
        self.assertEqual(body, JPEG)
        status, _headers, body = self.request("/api/config", {
            "threshold_c": 66.5, "scope": "global", "expected_version": 1,
            "actor": "test"})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["threshold_c"], 66.5)

    def test_detection_and_inspection_mode_are_independent(self):
        self.request("/api/detection", {"enabled": True})
        status, _headers, body = self.request("/api/mode", {
            "mode": "inspection", "task_id": "task-1"})
        payload = json.loads(body)
        self.assertEqual(status, 200)
        self.assertTrue(payload["detection_enabled"])
        self.assertEqual(payload["task_id"], "task-1")


if __name__ == "__main__":
    unittest.main()
