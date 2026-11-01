import json
from pathlib import Path
import sys
import tempfile
import unittest


MODULE_DIR = Path(__file__).resolve().parents[1] / "hikvision_camera"
sys.path.insert(0, str(MODULE_DIR))

from thermal_monitor import (  # noqa: E402
    HighTemperatureDetector,
    ThermalConfigConflict,
    ThermalConfigStore,
    ThermalMonitor,
    ThermalSample,
)


JPEG = b"\xff\xd8thermal\xff\xd9"


def sample(temperature, epoch=100.0, frame=1, frozen=False):
    return ThermalSample(
        raw_jpeg=JPEG,
        annotated_jpeg=JPEG,
        max_temperature_c=temperature,
        x_normalized=400,
        y_normalized=600,
        frame_number=frame,
        captured_at_epoch=epoch,
        width=256,
        height=192,
        frozen=frozen,
    )


class ThermalConfigStoreTests(unittest.TestCase):
    def test_global_and_task_thresholds_persist_and_inherit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "thermal.json"
            store = ThermalConfigStore(path, default_threshold_c=60.0)
            global_config = store.update(65.2, expected_version=1, actor="usb-page")
            task_config = store.update(
                72.5, scope="task", task_id="task-1", expected_version=0,
                actor="inspection-page")
            self.assertEqual(global_config["threshold_c"], 65.2)
            self.assertEqual(task_config["scope"], "task")
            self.assertEqual(ThermalConfigStore(path).get("task-1")["threshold_c"], 72.5)
            inherited = store.clear_task("task-1", expected_version=1)
            self.assertEqual(inherited["scope"], "global")
            self.assertEqual(inherited["threshold_c"], 65.2)

    def test_stale_editor_cannot_overwrite_newer_revision(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ThermalConfigStore(Path(directory) / "thermal.json")
            store.update(61.0, expected_version=1)
            with self.assertRaises(ThermalConfigConflict):
                store.update(62.0, expected_version=1)

    def test_invalid_threshold_is_rejected_without_corrupting_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "thermal.json"
            store = ThermalConfigStore(path)
            store.update(70.0, expected_version=1)
            with self.assertRaises(ValueError):
                store.update(float("nan"), expected_version=2)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["global"]["threshold_c"], 70.0)


class HighTemperatureDetectorTests(unittest.TestCase):
    def test_hold_hysteresis_and_one_event_per_hot_incident(self):
        ticks = iter([0.0, 0.5, 1.0, 1.1, 1.2, 2.0, 3.0])
        detector = HighTemperatureDetector(
            hold_seconds=1.0, clear_delta_c=3.0, clock=lambda: next(ticks))
        arguments = {"threshold_c": 60.0, "enabled": True, "mode": "transit", "task_id": "t"}
        self.assertIsNone(detector.process(sample(61.0), **arguments))
        self.assertIsNone(detector.process(sample(62.0), **arguments))
        event = detector.process(sample(63.0), **arguments)
        self.assertEqual(event["measurement_type"], "temperature")
        self.assertIsNone(detector.process(sample(70.0), **arguments))
        self.assertTrue(detector.active)
        self.assertIsNone(detector.process(sample(58.0), **arguments))
        self.assertTrue(detector.active)
        self.assertIsNone(detector.process(sample(56.9), **arguments))
        self.assertFalse(detector.active)


class ThermalMonitorTests(unittest.TestCase):
    def test_manual_detection_generates_typed_event_and_status(self):
        with tempfile.TemporaryDirectory() as directory:
            monotonic = iter([0.0, 1.0])
            store = ThermalConfigStore(Path(directory) / "thermal.json")
            monitor = ThermalMonitor(
                store, hold_seconds=1.0, wall_clock=lambda: 101.0,
                monotonic_clock=lambda: next(monotonic))
            monitor.set_backend_state(running=True, sdk_version="2.5.0.3", device="2bdf:0102")
            monitor.set_detection(True)
            self.assertIsNone(monitor.ingest(sample(61.0, frame=1)))
            event = monitor.ingest(sample(62.0, frame=2))
            self.assertEqual(event["category"], "high_temperature")
            self.assertIsNone(event["confidence"])
            status = monitor.status()
            self.assertTrue(status["ready"])
            self.assertTrue(status["alarm_active"])
            self.assertEqual(status["config"]["threshold_c"], 60.0)
            self.assertEqual(monitor.events(0)["events"][0]["sequence"], 1)

    def test_inspection_mode_forces_detection_but_paused_does_not(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ThermalConfigStore(Path(directory) / "thermal.json")
            monitor = ThermalMonitor(store, hold_seconds=0.0, wall_clock=lambda: 100.0)
            monitor.set_backend_state(running=True)
            monitor.set_mode("inspection", "task-1")
            self.assertTrue(monitor.status()["detection_enabled"])
            self.assertIsNotNone(monitor.ingest(sample(80.0)))
            monitor.set_mode("paused", "task-1")
            self.assertFalse(monitor.status()["detection_enabled"])

    def test_frozen_or_old_frame_is_never_reported_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ThermalConfigStore(Path(directory) / "thermal.json")
            monitor = ThermalMonitor(store, wall_clock=lambda: 105.0, stale_seconds=2.0)
            monitor.set_backend_state(running=True)
            monitor.ingest(sample(20.0, epoch=100.0))
            self.assertTrue(monitor.status()["stale"])
            self.assertFalse(monitor.status()["ready"])


if __name__ == "__main__":
    unittest.main()
