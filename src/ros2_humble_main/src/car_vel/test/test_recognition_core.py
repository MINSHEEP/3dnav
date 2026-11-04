#!/usr/bin/env python3
"""Behavior tests for inspection face/plate matching and record deduplication."""

from pathlib import Path
from datetime import datetime, timezone
import sys
import tempfile
import unittest


CAMERA_DIR = Path(__file__).resolve().parents[1] / 'hikvision_camera'
sys.path.insert(0, str(CAMERA_DIR))

from recognition_core import (  # noqa: E402
    CapacityError,
    RecognitionDatabase,
    TaskTrackDeduplicator,
    cosine_similarity,
    normalize_plate_number,
)


class RecognitionCoreTests(unittest.TestCase):

    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.database_path = (
            Path(self.temporary_directory.name) / 'recognition.db')
        self.database = RecognitionDatabase(self.database_path)

    def tearDown(self):
        self.database.close()
        self.temporary_directory.cleanup()

    def test_registered_unregistered_and_uncertain_are_never_alerts(self):
        self.database.upsert_person(
            'p-001', '张三', [[1.0, 0.0, 0.0]], notes='运维')
        registered = self.database.identify_face(
            [0.99, 0.01, 0.0], quality=0.95,
            match_threshold=0.80, quality_threshold=0.60)
        unregistered = self.database.identify_face(
            [0.0, 1.0, 0.0], quality=0.95,
            match_threshold=0.80, quality_threshold=0.60)
        uncertain = self.database.identify_face(
            [1.0, 0.0, 0.0], quality=0.30,
            match_threshold=0.80, quality_threshold=0.60)

        self.assertEqual(registered['status'], 'registered')
        self.assertEqual(registered['display_name'], '张三')
        self.assertEqual(unregistered['status'], 'unregistered')
        self.assertEqual(uncertain['status'], 'uncertain')
        self.assertTrue(all(
            item['alert'] is False
            for item in (registered, unregistered, uncertain)))

    def test_plate_matching_normalizes_text_and_respects_ocr_confidence(self):
        self.database.upsert_vehicle(
            'v-001', '粤B·12345', owner='设备部',
            plate_color='blue_white')

        registered = self.database.identify_plate(
            ' 粤b 12345 ', confidence=0.92, confidence_threshold=0.70,
            plate_color='blue_white')
        unregistered = self.database.identify_plate(
            '粤B99999', confidence=0.92, confidence_threshold=0.70,
            plate_color='yellow_black')
        uncertain = self.database.identify_plate(
            '粤B12345', confidence=0.40, confidence_threshold=0.70,
            plate_color='blue_white')

        self.assertEqual(normalize_plate_number(' 粤b·12345 '), '粤B12345')
        self.assertEqual(registered['status'], 'registered')
        self.assertEqual(registered['display_name'], '粤B12345')
        self.assertEqual(unregistered['status'], 'unregistered')
        self.assertEqual(uncertain['status'], 'uncertain')
        self.assertTrue(all(
            item['alert'] is False
            for item in (registered, unregistered, uncertain)))

    def test_people_and_vehicle_registries_are_limited_to_fifty_each(self):
        for index in range(50):
            self.database.upsert_person(
                f'p-{index:03d}', f'人员{index}', [[1.0, float(index), 0.0]])
            self.database.upsert_vehicle(
                f'v-{index:03d}', f'粤B{index:05d}', owner='测试')

        with self.assertRaises(CapacityError):
            self.database.upsert_person(
                'p-overflow', '超额人员', [[0.0, 1.0, 0.0]])
        with self.assertRaises(CapacityError):
            self.database.upsert_vehicle(
                'v-overflow', '粤B99999', owner='测试')

        self.assertEqual(len(self.database.list_people()), 50)
        self.assertEqual(len(self.database.list_vehicles()), 50)

    def test_duplicate_plate_is_rejected_as_a_validation_error(self):
        self.database.upsert_vehicle('v-1', '粤B12345')
        with self.assertRaisesRegex(ValueError, '车牌'):
            self.database.upsert_vehicle('v-2', '粤B12345')

    def test_track_requires_two_frames_and_reopens_after_120_seconds(self):
        tracker = TaskTrackDeduplicator(
            stable_frames=2, retention_seconds=120.0)
        tracker.start_task('task-a')
        bbox = [10, 20, 60, 100]

        self.assertIsNone(tracker.observe(
            'face', bbox, identity_key='person:p-001', now=0.0))
        first = tracker.observe(
            'face', bbox, identity_key='person:p-001', now=0.1)
        self.assertEqual(first, 'face:1')
        self.assertIsNone(tracker.observe(
            'face', bbox, identity_key='person:p-001', now=60.0))
        self.assertIsNone(tracker.observe(
            'face', bbox, identity_key='person:p-001', now=181.0))
        second = tracker.observe(
            'face', bbox, identity_key='person:p-001', now=181.1)
        self.assertEqual(second, 'face:2')

    def test_tracks_never_merge_across_inspection_tasks(self):
        tracker = TaskTrackDeduplicator(stable_frames=2)
        bbox = [0, 0, 40, 40]
        tracker.start_task('task-a')
        tracker.observe('plate', bbox, identity_key='plate:粤B12345', now=1.0)
        self.assertEqual(
            tracker.observe(
                'plate', bbox, identity_key='plate:粤B12345', now=1.1),
            'plate:1')

    def test_identity_must_be_stable_for_two_frames_before_emitting(self):
        tracker = TaskTrackDeduplicator(stable_frames=2)
        tracker.start_task('task-a')
        bbox = [0, 0, 40, 40]
        self.assertIsNone(tracker.observe(
            'plate', bbox, identity_key='plate:粤B11111', now=1.0))
        self.assertIsNone(tracker.observe(
            'plate', bbox, identity_key='plate:粤B22222', now=1.1))
        self.assertEqual(tracker.observe(
            'plate', bbox, identity_key='plate:粤B22222', now=1.2), 'plate:1')

        tracker.start_task('task-b')
        self.assertIsNone(tracker.observe(
            'plate', bbox, identity_key='plate:粤B12345', now=2.0))
        self.assertEqual(
            tracker.observe(
                'plate', bbox, identity_key='plate:粤B12345', now=2.1),
            'plate:1')

    def test_cosine_similarity_rejects_invalid_or_zero_vectors(self):
        self.assertAlmostEqual(
            cosine_similarity([1.0, 0.0], [0.8, 0.0]), 1.0)
        self.assertEqual(cosine_similarity([0.0, 0.0], [1.0, 0.0]), 0.0)
        self.assertEqual(cosine_similarity([1.0], [1.0, 0.0]), 0.0)

    def test_stats_audit_review_and_status_specific_evidence_retention(self):
        self.database.upsert_person('p-1', '张三', [[1.0, 0.0]])
        evidence_root = Path(self.temporary_directory.name) / 'evidence'
        evidence_root.mkdir()
        uncertain_file = evidence_root / 'uncertain.jpg'
        registered_file = evidence_root / 'registered.jpg'
        uncertain_file.write_bytes(b'uncertain')
        registered_file.write_bytes(b'registered')
        old = datetime.fromtimestamp(
            1_700_000_000 - 40 * 86400, timezone.utc).isoformat(timespec='seconds')
        uncertain_id = self.database.record_event(
            'task-a', 'face:1', {
                'category': 'face', 'status': 'uncertain', 'confidence': 0.2,
                'identifier': '', 'display_name': '', 'alert': False,
            }, evidence_path=str(uncertain_file), occurred_at=old)
        self.database.record_event(
            'task-a', 'face:2', {
                'category': 'face', 'status': 'registered', 'confidence': 0.9,
                'identifier': 'p-1', 'display_name': '张三', 'alert': False,
            }, evidence_path=str(registered_file), occurred_at=old)
        self.assertTrue(self.database.review_event(
            uncertain_id, 'unregistered', reviewer='operator'))

        cleanup = self.database.cleanup_expired(
            now_epoch=1_700_000_000, evidence_root=evidence_root)
        self.assertEqual(cleanup['evidence'], 1)
        self.assertFalse(uncertain_file.exists())
        self.assertTrue(registered_file.exists())
        self.assertEqual(self.database.stats()['events'], 2)
        self.assertGreaterEqual(len(self.database.list_audit()), 2)


if __name__ == '__main__':
    unittest.main()
