from pathlib import Path
import sys
import unittest
from unittest import mock
import urllib.error


MODULE_DIR = Path(__file__).resolve().parents[1] / "hikvision_camera"
sys.path.insert(0, str(MODULE_DIR))

from thermal_client import ThermalHttpClient  # noqa: E402


class _Response:
    headers = {"Content-Type": "application/json"}

    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def read(self, *_args):
        return self.payload


class ThermalHttpClientTests(unittest.TestCase):
    def test_status_degrades_without_raising_when_process_is_offline(self):
        client = ThermalHttpClient("http://127.0.0.1:8767")
        client._opener = mock.Mock()
        client._opener.open.side_effect = urllib.error.URLError("offline")
        status = client.status()
        self.assertFalse(status["available"])
        self.assertIn("offline", status["error"])

    def test_task_config_query_and_detection_post_cross_small_interface(self):
        client = ThermalHttpClient("http://127.0.0.1:8767")
        client._opener = mock.Mock()
        client._opener.open.return_value = _Response(b'{"threshold_c":65.0}')
        self.assertEqual(client.config("task-1")["threshold_c"], 65.0)
        first_request = client._opener.open.call_args.args[0]
        self.assertIn("task_id=task-1", first_request.full_url)
        client.set_detection(True)
        second_request = client._opener.open.call_args.args[0]
        self.assertEqual(second_request.method, "POST")
        self.assertIn(b'"enabled": true', second_request.data)


if __name__ == "__main__":
    unittest.main()
