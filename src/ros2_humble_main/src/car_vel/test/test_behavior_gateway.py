"""Inspection behavior stays available when the independent YOLO branch fails."""
from argparse import Namespace
import base64
import copy
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'hikvision_camera'))
import camera_web


class GatewayBehaviorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.gateway = object.__new__(camera_web.CameraGateway)
        self.gateway.args = Namespace(inspection_evidence_root=Path(self.temp.name), camera_type='usb')
        self.gateway._camera_change_lock = threading.RLock()
        self.gateway._inspection_task_id = 'task-1'
        self.gateway._inspection_mode = 'transit'
        self.gateway._camera_sources = Namespace(current={'pipeline':'legacy'})
        self.gateway._thermal_event_sequences = {'legacy':0, 'c12':0}
        self.gateway._thermal = Mock()
        self.gateway._thermal.status.return_value = {'ready': True}
        self.gateway._thermal.set_mode.return_value = {'ready': True}
        self.gateway._thermal.capture.return_value = {'detections': []}
        self.gateway._thermal.events.return_value = {'events': []}
        self.gateway._yolo = Mock()
        self.gateway._yolo.status.return_value = {'running': False, 'ready': False, 'error': 'YOLO unavailable'}
        self.gateway._yolo.start.side_effect = RuntimeError('YOLO unavailable')
        self.gateway._yolo.inspection_request.side_effect = RuntimeError('YOLO unavailable')
        self.gateway._behavior_speech = Mock()
        self.gateway._behavior_speech.status.return_value = {'ready': True, 'error': ''}
        self.gateway._behavior = Mock()
        self.gateway._behavior.status.return_value = {'running': True, 'ready': True, 'fresh': True,
            'enabled': {'fall': True, 'fight': True}, 'branches': {}, 'partial': False}
        self.jpeg = base64.b64encode(b'\xff\xd8behavior-frame\xff\xd9').decode()

    def tearDown(self):
        self.temp.cleanup()

    def evidence(self):
        return dict(mode='transit', task_id='task-1', sequence=5, source_epoch=7, stamp_ns=800,
            fresh=True, detections=[dict(category='fall', label='fall', confidence=None,
                state='valid', updated=True, fresh=True, alert=True, score_kind='rule_evidence',
                source_epoch=7, stamp_ns=800)],
            raw_jpeg_base64=self.jpeg, annotated_jpeg_base64=self.jpeg,
            behavior=copy.deepcopy(self.gateway._behavior.status.return_value))

    def test_inspection_behavior_mode_stops_yolo_and_reports_partial_coverage(self):
        result = self.gateway.set_inspection_mode({'mode': 'transit', 'task_id': 'task-1'})
        self.assertTrue(result['success'])
        self.assertTrue(result['partial'])
        self.gateway._yolo.stop.assert_called_once()
        self.gateway._yolo.start.assert_not_called()
        self.gateway._behavior.set_mode.assert_called_once_with('transit', 'task-1')

    def test_manual_start_selects_behavior_and_stops_yolo_first(self):
        calls = []
        self.gateway._yolo.stop.side_effect = lambda: calls.append('stop_yolo')
        self.gateway._behavior.start.side_effect = lambda: calls.append('start_behavior')
        self.gateway.start_detection()
        self.assertEqual(calls, ['stop_yolo', 'start_behavior'])
        self.gateway._yolo.start.assert_not_called()

    def test_manual_start_does_not_start_behavior_if_yolo_cannot_stop(self):
        self.gateway._yolo.stop.side_effect = RuntimeError('cannot stop YOLO')
        result = self.gateway.start_detection()
        self.gateway._behavior.start.assert_not_called()
        self.assertIn('behavior', result['errors'])

    def test_enabling_behavior_stops_running_yolo_and_starts_behavior(self):
        self.gateway._inspection_mode = 'manual'
        self.gateway._yolo.status.return_value = {'running': True, 'ready': True}
        self.gateway.configure_behavior({'fall_enabled': True, 'fight_enabled': True, 'smoking_enabled': True})
        self.gateway._yolo.stop.assert_called_once()
        self.gateway._behavior.start.assert_called_once()

    def test_disabling_active_behavior_resumes_yolo(self):
        self.gateway._inspection_mode = 'manual'
        self.gateway._behavior.status.return_value['manual_detection_enabled'] = True
        self.gateway._yolo.start.side_effect = None
        self.gateway.configure_behavior({'fall_enabled': False, 'fight_enabled': False, 'smoking_enabled': False})
        self.gateway._yolo.start.assert_called_once()

    def test_explicit_yolo_start_disables_and_stops_behavior(self):
        self.gateway._inspection_mode = 'manual'
        self.gateway._yolo.start.side_effect = None
        self.gateway.start_yolo()
        self.gateway._behavior.configure.assert_called_once_with({'fall_enabled': False, 'fight_enabled': False, 'smoking_enabled': False})
        self.gateway._behavior.stop.assert_called_once()
        self.gateway._yolo.start.assert_called_once()

    def test_patrol_notice_uses_local_audio_only_for_the_current_task(self):
        self.gateway._behavior_speech.announce_patrol_safety.return_value = True
        request = {'key': 'patrol_safety', 'task_id': 'task-1'}
        for running in (False, True):
            with self.subTest(running=running):
                self.gateway._yolo.status.return_value['running'] = running
                self.gateway._inspection_mode = 'transit'
                self.gateway._behavior_speech.announce_patrol_safety.reset_mock()
                self.assertFalse(self.gateway.inspection_speech({**request, 'task_id': 'old-task'})['queued'])
                self.assertEqual(self.gateway.inspection_speech(request),
                                 dict(success=True, key='patrol_safety', queued=True, voice_backend='local'))
                self.gateway._inspection_mode = 'paused'
                self.assertFalse(self.gateway.inspection_speech(request)['queued'])
                self.gateway._behavior_speech.announce_patrol_safety.assert_called_once_with()
        self.gateway._yolo.inspection_request.assert_not_called()


    def test_environment_speech_always_uses_local_queue_regardless_of_yolo(self):
        speaker = self.gateway._behavior_speech.announce
        for running in (False, True):
            self.gateway._yolo.status.return_value['running'] = running
            for key in ('thermal_high', 'co2_high'):
                for queued in (False, True):
                    with self.subTest(running=running, key=key, queued=queued):
                        speaker.reset_mock()
                        speaker.return_value = queued
                        self.assertEqual(self.gateway.inspection_speech({'key': key}),
                            dict(success=True, key=key, queued=queued, voice_backend='local'))
                        speaker.assert_called_once_with(key)
        self.gateway._yolo.inspection_request.assert_not_called()

    def test_environment_speech_rejects_unknown_keys_without_playback(self):
        for key in ('', 'arbitrary phrase', 'fire'):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.gateway.inspection_speech({'key': key})
        self.gateway._behavior_speech.announce.assert_not_called()
        self.gateway._yolo.inspection_request.assert_not_called()

    def test_capture_persists_behavior_own_pair_even_if_yolo_is_missing(self):
        self.gateway._behavior.capture.return_value = self.evidence()
        result = self.gateway.inspection_capture(dict(task_id='task-1', loop_number=1,
            route_point_id=3, direction='current', duration_seconds=0.01, save_images=True))
        self.assertTrue(result['success'])
        self.assertTrue(result['partial'])
        self.assertEqual(result['raw_image'], '')
        self.assertEqual(len(result['detections']), 1)
        image = result['detections'][0]['evidence']['raw_image']
        self.assertIn('_behavior_', image)
        self.assertEqual(Path(image).read_bytes(), b'\xff\xd8behavior-frame\xff\xd9')
        self.assertEqual(result['behavior']['source_epoch'], 7)

    def test_old_task_capture_and_events_cannot_transfer_to_new_task(self):
        self.gateway._behavior.capture.return_value = self.evidence()
        self.gateway._inspection_task_id = 'task-2'
        result = self.gateway.inspection_capture(dict(task_id='task-1', loop_number=1,
            route_point_id=3, direction='current', duration_seconds=0.01, save_images=True))
        self.assertEqual(result['detections'], [])
        self.gateway._behavior.events.return_value = [self.evidence()]
        self.assertEqual(self.gateway.inspection_events(0)['events'], [])

    def test_full_project_actions_share_the_same_inference_evidence(self):
        result = self.evidence()
        result['detections'].append(dict(category='fight', label='fight', scope='scene', confidence=.91,
                                        source_epoch=7, stamp_ns=800, fresh=True, alert=True))
        self.gateway._behavior.capture.return_value = result
        captured = self.gateway.inspection_capture(dict(task_id='task-1', loop_number=1,
            route_point_id=3, direction='current', duration_seconds=.01, save_images=True))
        paths = {d['category']: Path(d['evidence']['raw_image']) for d in captured['detections']}
        self.assertEqual(paths['fall'], paths['fight'])
        self.assertEqual(paths['fall'].read_bytes(), b'\xff\xd8behavior-frame\xff\xd9')


if __name__ == '__main__':
    unittest.main()
