#!/usr/bin/env python3
"""Behavior tests for priority-aware multi-model inference."""

from pathlib import Path
import sys
import unittest
from unittest import mock

CAMERA_DIR = Path(__file__).resolve().parents[1] / "hikvision_camera"
sys.path.insert(0, str(CAMERA_DIR))

try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    sys.modules["cv2"] = mock.MagicMock()
try:
    import numpy  # noqa: F401
except ModuleNotFoundError:
    sys.modules["numpy"] = mock.MagicMock()

from multimodel_detection import MultiModelDetector  # noqa: E402


class _Scalar:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def item(self):
        return self.value


class _Vector:
    def __init__(self, values):
        self.values = values

    def cpu(self):
        return self

    def tolist(self):
        return list(self.values)


class _Box:
    def __init__(self, class_id, confidence, bbox):
        self.cls = [_Scalar(class_id)]
        self.conf = [_Scalar(confidence)]
        self.xyxy = [_Vector(bbox)]


class _Result:
    def __init__(self, names, boxes):
        self.names = names
        self.boxes = boxes


class _Model:
    def __init__(self, result):
        self.result = result
        self.calls = []

    def predict(self, frame, **kwargs):
        self.calls.append(kwargs)
        return [self.result]


class _FaceDetector:
    def __init__(self):
        self.calls = 0
        self.input_sizes = []

    def setInputSize(self, size):
        self.input_sizes.append(size)

    def detect(self, _frame):
        self.calls += 1
        return True, [[10, 20, 30, 40, 0.90]]


class _Frame:
    shape = (100, 120, 3)


def _pipeline(transit_fps=2.0):
    primary = _Model(_Result(
        {0: "smoke", 1: "fire", 2: "CigaretteButt"},
        [_Box(1, 0.80, [1, 2, 20, 30])],
    ))
    plate = _Model(_Result(
        {0: "['plate']"},
        [_Box(0, 0.75, [40, 50, 80, 70])],
    ))
    face = _FaceDetector()
    detector = MultiModelDetector(
        primary, face_detector=face, plate_model=plate,
        transit_aux_fps=transit_fps, manual_aux_fps=5.0,
        display_ttl_seconds=1.0,
    )
    return detector, primary, face, plate


class MultiModelDetectorTests(unittest.TestCase):

    def test_transit_auxiliary_models_are_limited_and_cached_for_display(self):
        detector, primary, face, plate = _pipeline(transit_fps=2.0)
        frame = _Frame()

        first = detector.analyze(frame, mode="transit", now=0.0)
        second = detector.analyze(frame, mode="transit", now=0.1)
        third = detector.analyze(frame, mode="transit", now=0.5)

        self.assertEqual(len(primary.calls), 3)
        self.assertTrue(all(call["imgsz"] == 640 for call in primary.calls))
        self.assertEqual(face.calls, 2)
        self.assertEqual(len(plate.calls), 2)
        self.assertEqual(
            [item.category for item in first.detections],
            ["fire", "face", "plate"],
        )
        self.assertEqual(
            [item.category for item in second.detections], ["fire"])
        self.assertEqual(
            [item.category for item in second.display_detections],
            ["fire", "face", "plate"],
        )
        self.assertEqual(
            third.ran_detectors, ("primary", "face", "plate"))

    def test_inspection_runs_all_enabled_models_on_every_frame(self):
        detector, primary, face, plate = _pipeline(transit_fps=0.5)
        frame = _Frame()

        detector.analyze(frame, mode="inspection", now=0.0)
        result = detector.analyze(frame, mode="inspection", now=0.01)

        self.assertEqual(face.calls, 2)
        self.assertEqual(len(plate.calls), 2)
        self.assertTrue(all(call["imgsz"] == 1280 for call in primary.calls))
        self.assertEqual(
            result.ran_detectors, ("primary", "face", "plate"))

    def test_only_safety_categories_are_alerts(self):
        detector, _primary, _face, _plate = _pipeline()
        frame = _Frame()

        result = detector.analyze(frame, mode="transit", now=0.0)
        public = result.public_detections()

        self.assertEqual(result.alarm_detections(), [("fire", 0.80)])
        self.assertTrue(public[0]["alert"])
        self.assertFalse(public[1]["alert"])
        self.assertFalse(public[2]["alert"])
        self.assertEqual(public[2]["category"], "plate")
        self.assertEqual(public[2]["bbox"], [40, 50, 80, 70])


if __name__ == "__main__":
    unittest.main()
