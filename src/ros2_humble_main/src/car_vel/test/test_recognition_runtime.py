#!/usr/bin/env python3
"""Integration contract for camera-frame recognition enrichment."""

from pathlib import Path
import sys
import tempfile
import unittest


CAMERA_DIR = Path(__file__).resolve().parents[1] / 'hikvision_camera'
sys.path.insert(0, str(CAMERA_DIR))

from recognition_core import RecognitionDatabase  # noqa: E402
from recognition_runtime import RecognitionRuntime  # noqa: E402


class _FaceExtractor:
    available = True
    error = ''

    def __init__(self):
        self.landmarks = None

    def extract(self, _frame, _bbox, landmarks=None):
        self.landmarks = landmarks
        return [1.0, 0.0, 0.0]


class _PlateRecognizer:
    available = True
    error = ''

    def recognize(self, _frame, _bbox):
        return '粤B·12345', 0.96


class RecognitionRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.database = RecognitionDatabase(Path(self.temp.name) / 'recognition.db')
        self.database.upsert_person('p-1', '张三', [[1.0, 0.0, 0.0]])
        self.database.upsert_vehicle(
            'v-1', '粤B12345', owner='张三', plate_color='blue_white')
        self.saved = []
        self.face_extractor = _FaceExtractor()
        self.runtime = RecognitionRuntime(
            self.database,
            face_extractor=self.face_extractor,
            plate_recognizer=_PlateRecognizer(),
            plate_color_classifier=lambda _frame, _bbox: 'blue_white',
            evidence_writer=lambda *_args: self.saved.append('saved') or '/tmp/evidence.jpg',
        )

    def tearDown(self):
        self.database.close()
        self.temp.cleanup()

    def test_enriches_but_never_raises_safety_alert_and_records_after_two_frames(self):
        detections = [
            {'category': 'face', 'confidence': 0.95, 'bbox': [1, 2, 20, 30], 'landmarks': [[3, 4], [8, 4], [6, 8], [4, 12], [8, 12]], 'alert': False},
            {'category': 'plate', 'confidence': 0.91, 'bbox': [30, 5, 80, 25], 'alert': False},
            {'category': 'smoke', 'confidence': 0.88, 'bbox': [0, 0, 5, 5], 'alert': True},
        ]
        first = self.runtime.process(object(), detections, task_id='task-a', now=1.0)
        second = self.runtime.process(object(), detections, task_id='task-a', now=1.1)

        self.assertEqual(first[0]['status'], 'registered')
        self.assertEqual(first[0]['display_name'], '张三')
        self.assertFalse(first[0]['alert'])
        self.assertEqual(first[1]['display_name'], '粤B12345')
        self.assertEqual(first[1]['plate_color'], 'blue_white')
        self.assertFalse(first[1]['alert'])
        self.assertTrue(first[2]['alert'])
        self.assertEqual(len(self.database.list_events(task_id='task-a')), 2)
        self.assertEqual(len(self.saved), 2)
        self.assertIn('recognition_event_id', second[0])
        self.assertEqual(self.face_extractor.landmarks[0], [3, 4])

    def test_preview_matches_but_does_not_persist_without_task_id(self):
        result = self.runtime.process(object(), [{
            'category': 'face', 'confidence': 0.95,
            'bbox': [1, 2, 20, 30], 'alert': False,
        }], task_id='', now=1.0)
        self.runtime.process(object(), result, task_id='', now=1.1)
        self.assertEqual(result[0]['status'], 'registered')
        self.assertEqual(self.database.list_events(), [])

    def test_task_change_resets_deduplication(self):
        detection = [{
            'category': 'plate', 'confidence': 0.91,
            'bbox': [30, 5, 80, 25], 'alert': False,
        }]
        self.runtime.process(object(), detection, task_id='task-a', now=1.0)
        self.runtime.process(object(), detection, task_id='task-a', now=1.1)
        self.runtime.process(object(), detection, task_id='task-b', now=2.0)
        self.runtime.process(object(), detection, task_id='task-b', now=2.1)
        self.assertEqual(len(self.database.list_events(task_id='task-a')), 1)
        self.assertEqual(len(self.database.list_events(task_id='task-b')), 1)

    def test_runtime_restart_does_not_reuse_persisted_track_identifier(self):
        detection = [{
            'category': 'plate', 'confidence': 0.91,
            'bbox': [30, 5, 80, 25], 'alert': False,
        }]
        self.runtime.process(object(), detection, task_id='task-a', now=1.0)
        self.runtime.process(object(), detection, task_id='task-a', now=1.1)
        restarted = RecognitionRuntime(
            self.database, plate_recognizer=_PlateRecognizer(),
            plate_color_classifier=lambda *_args: 'blue_white',
            evidence_writer=lambda *_args: '')
        restarted.process(object(), detection, task_id='task-a', now=2.0)
        restarted.process(object(), detection, task_id='task-a', now=2.1)
        self.assertEqual(len(self.database.list_events(task_id='task-a')), 2)


if __name__ == '__main__':
    unittest.main()
