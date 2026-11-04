"""Regression tests for the camera web service's YOLO MJPEG proxy."""

from argparse import Namespace
import io
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
import base64
from unittest import mock


CAMERA_DIR = Path(__file__).resolve().parents[1] / "hikvision_camera"
sys.path.insert(0, str(CAMERA_DIR))

import camera_web  # noqa: E402
try:  # Windows planning host does not need OpenCV for state-only tests.
    import cv2  # noqa: F401
except ModuleNotFoundError:
    sys.modules["cv2"] = mock.MagicMock()
try:
    import numpy  # noqa: F401
except ModuleNotFoundError:
    sys.modules["numpy"] = mock.MagicMock()
import yolo_hk  # noqa: E402


class _RunningProcess:
    """Small subprocess stand-in that remains alive for the test."""

    @staticmethod
    def poll():
        return None


class _MjpegResponse:
    """Minimal successful response returned by the local YOLO server."""

    headers = {"Content-Type": "multipart/x-mixed-replace; boundary=frame"}

    def close(self):
        """Match the response interface used by the production code."""


class TestYoloStreamStartup(unittest.TestCase):
    """The web proxy must survive the YOLO subprocess startup window."""

    def test_open_stream_retries_connection_refused_until_server_is_ready(self):
        manager = camera_web.YoloProcessManager(
            Namespace(yolo_stream_start_timeout=1.0)
        )
        manager._process = _RunningProcess()
        response = _MjpegResponse()
        attempts = 0

        def open_after_startup(_url, timeout):
            nonlocal attempts
            self.assertGreater(timeout, 0)
            attempts += 1
            if attempts < 3:
                raise urllib.error.URLError(ConnectionRefusedError(111, "refused"))
            return response

        manager._open_local = open_after_startup
        with mock.patch.object(camera_web.time, "sleep", return_value=None):
            self.assertIs(manager.open_stream(), response)
        self.assertEqual(attempts, 3)


class TestThermalStreamProxy(unittest.TestCase):
    """A failed loopback stream must fail fast so the browser can reconnect."""

    def test_unavailable_upstream_returns_http_503_instead_of_hanging(self):
        gateway = mock.Mock()
        gateway.open_thermal_stream.side_effect = RuntimeError("thermal offline")
        server = camera_web.CameraWebServer(
            ("127.0.0.1", 0), camera_web.CameraRequestHandler,
            max_stream_clients=2, stream_write_timeout=1.0)
        server.gateway = gateway
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = (
            f"http://127.0.0.1:{server.server_address[1]}"
            "/api/thermal/stream.mjpg"
        )
        try:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(url, timeout=1.0)
            self.assertEqual(caught.exception.code, 503)
            self.assertIn("thermal offline", caught.exception.read().decode("utf-8"))
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)

    def test_available_upstream_preserves_boundary_and_forwards_frame(self):
        frame = b"--thermal\r\nContent-Type: image/jpeg\r\n\r\nJPEG\r\n"

        class Response(io.BytesIO):
            headers = {
                "Content-Type":
                    "multipart/x-mixed-replace; boundary=thermal"
            }

        gateway = mock.Mock()
        gateway.open_thermal_stream.return_value = Response(frame)
        server = camera_web.CameraWebServer(
            ("127.0.0.1", 0), camera_web.CameraRequestHandler,
            max_stream_clients=2, stream_write_timeout=1.0)
        server.gateway = gateway
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = (
            f"http://127.0.0.1:{server.server_address[1]}"
            "/api/thermal/stream.mjpg"
        )
        try:
            with urllib.request.urlopen(url, timeout=1.0) as response:
                self.assertEqual(
                    response.headers.get("Content-Type"),
                    "multipart/x-mixed-replace; boundary=thermal")
                self.assertEqual(response.read(), frame)
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)

    def test_child_command_enables_bundled_face_and_plate_models(self):
        manager = camera_web.YoloProcessManager(Namespace(
            yolo_conda_env="current",
        ))

        command = manager._build_command()

        self.assertTrue(manager._available())
        self.assertEqual(
            Path(command[command.index("--model") + 1]).name,
            "cigarette_multiclass_best.pt",
        )
        self.assertEqual(
            Path(command[command.index("--face-model") + 1]).name,
            "face_detection_yunet_2023mar.onnx",
        )
        self.assertEqual(
            Path(command[command.index("--plate-model") + 1]).name,
            "license_plate_best.pt",
        )
        self.assertEqual(
            Path(command[command.index("--sface-model") + 1]).name,
            "face_recognition_sface_2021dec.onnx",
        )
        self.assertEqual(
            Path(command[command.index("--recognition-db") + 1]).name,
            "recognition.db",
        )
        self.assertIn("--recognition-evidence-dir", command)
        self.assertEqual(
            command[command.index("--aux-transit-fps") + 1], "3.0")
        self.assertEqual(
            command[command.index("--inspection-imgsz") + 1], "1280")

    def test_gateway_forwards_task_id_and_recognition_health_to_child(self):
        gateway = camera_web.CameraGateway(Namespace(
            speed=4, camera_type='usb', usb_device='/dev/video0'))
        child = mock.MagicMock()
        child.status.return_value = {'running': True}
        child.inspection_request.side_effect = [
            {'success': True, 'mode': 'transit', 'task_id': 'task-42'},
            {'success': True, 'ready': True, 'sequence': 2,
             'recognition': {'sface': {'available': True}}},
        ]
        gateway._yolo = child
        try:
            gateway.set_inspection_mode({
                'mode': 'transit', 'task_id': 'task-42'})
            child.inspection_request.assert_any_call(
                'mode', {'mode': 'transit', 'task_id': 'task-42'},
                timeout=30.0)
            state = gateway.inspection_state()
            self.assertTrue(state['recognition']['sface']['available'])
            child.inspection_request.side_effect = None
            child.inspection_request.return_value = {
                'success': True, 'mode': 'manual', 'task_id': ''}
            gateway.set_inspection_mode({'mode': 'manual'})
            child.inspection_request.assert_called_with(
                'mode', {'mode': 'manual', 'task_id': ''}, timeout=30.0)
            self.assertEqual(gateway._inspection_task_id, '')
        finally:
            gateway.close()


class TestInspectionYoloState(unittest.TestCase):
    """Inspection inference must preserve paired frames and best evidence."""

    def test_mode_event_and_best_frame_capture(self):
        state = yolo_hk.InspectionYoloState()
        changed = state.set_mode("transit", task_id="task-42")
        self.assertEqual(changed["mode"], "transit")
        self.assertEqual(changed["task_id"], "task-42")
        jpeg_a = b"\xff\xd8a\xff\xd9"
        jpeg_b = b"\xff\xd8b\xff\xd9"

        def publish_frames():
            time.sleep(0.02)
            state.publish(jpeg_a, jpeg_a, [
                {"category": "smoke", "confidence": 0.4}])
            time.sleep(0.02)
            state.publish(jpeg_b, jpeg_b, [
                {"category": "smoke", "confidence": 0.9}])

        worker = threading.Thread(target=publish_frames)
        worker.start()
        capture = state.capture_window(0.08)
        worker.join()
        self.assertEqual(capture["sequence"], 2)
        self.assertEqual(
            base64.b64decode(capture["raw_jpeg_base64"]), jpeg_b)
        events = state.events_since(0)
        self.assertEqual([item["sequence"] for item in events["events"]], [1, 2])
        self.assertEqual(state.set_mode("paused")["mode"], "paused")
        self.assertFalse(state.wait_for_inference_turn())

    def test_evidence_store_rejects_paths_outside_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            gateway = camera_web.CameraGateway(Namespace(
                inspection_evidence_root=root,
                speed=4,
            ))
            payload = {
                "sequence": 3,
                "timestamp_epoch": 1.0,
                "raw_jpeg_base64": base64.b64encode(
                    b"\xff\xd8raw\xff\xd9").decode("ascii"),
                "annotated_jpeg_base64": base64.b64encode(
                    b"\xff\xd8ann\xff\xd9").decode("ascii"),
            }
            result = gateway._store_inspection_pair(
                payload, root / "task", "front")
            self.assertTrue(Path(result["raw_image"]).is_file())
            with self.assertRaises(ValueError):
                gateway.inspection_evidence(str(root.parent / "secret.jpg"))

    def test_non_alert_detections_do_not_create_transit_events(self):
        state = yolo_hk.InspectionYoloState()
        jpeg = b"\xff\xd8face\xff\xd9"
        state.set_mode("transit")

        state.publish(jpeg, jpeg, [{
            "category": "face", "confidence": 0.9, "alert": False,
        }])

        self.assertEqual(state.events_since(0)["events"], [])
        capture = state.capture_window(0.01)
        self.assertEqual(capture["detections"][0]["category"], "face")

    def test_safety_detection_outranks_higher_confidence_face_in_capture(self):
        state = yolo_hk.InspectionYoloState()
        jpeg_face = b"\xff\xd8face\xff\xd9"
        jpeg_fire = b"\xff\xd8fire\xff\xd9"
        state.set_mode("inspection")

        def publish_frames():
            time.sleep(0.02)
            state.publish(jpeg_face, jpeg_face, [{
                "category": "face", "confidence": 0.99, "alert": False,
            }])
            time.sleep(0.02)
            state.publish(jpeg_fire, jpeg_fire, [{
                "category": "fire", "confidence": 0.80, "alert": True,
            }])

        worker = threading.Thread(target=publish_frames)
        worker.start()
        capture = state.capture_window(0.08)
        worker.join()

        self.assertEqual(capture["sequence"], 2)
        self.assertEqual(capture["detections"][0]["category"], "fire")


class TestInspectionYoloHttp(unittest.TestCase):
    """The gateway timeout must include the blocking capture window."""

    def test_two_second_capture_has_http_timeout_headroom(self):
        server = yolo_hk.YoloMJPEGServer("127.0.0.1", 0)
        server.start()
        jpeg = b"\xff\xd8inspection\xff\xd9"
        server.publish_inspection_frame(jpeg, jpeg, [])
        port = int(server._server.server_address[1])
        manager = camera_web.YoloProcessManager(Namespace(
            yolo_host="127.0.0.1",
            yolo_port=port,
        ))
        manager._process = _RunningProcess()

        started = time.monotonic()
        try:
            capture = manager.inspection_request(
                "capture", {"duration_seconds": 2.0}, timeout=5.0)
        finally:
            server.close()

        self.assertEqual(capture["sequence"], 1)
        self.assertLess(time.monotonic() - started, 4.0)

    def test_timed_out_capture_is_not_submitted_twice(self):
        manager = camera_web.YoloProcessManager(Namespace(
            yolo_host="127.0.0.1",
            yolo_port=9,
        ))
        manager._process = _RunningProcess()
        opener = mock.Mock()
        opener.open.side_effect = TimeoutError("timed out")

        with mock.patch.object(
                camera_web.urllib.request, "build_opener",
                return_value=opener):
            with self.assertRaisesRegex(
                    TimeoutError, "YOLO 精检采集.*未完成"):
                manager.inspection_request(
                    "capture", {"duration_seconds": 2.0}, timeout=0.5)

        self.assertEqual(opener.open.call_count, 1)


if __name__ == "__main__":
    unittest.main()
