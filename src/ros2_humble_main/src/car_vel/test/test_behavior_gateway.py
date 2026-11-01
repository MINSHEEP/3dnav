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
        self.gateway._thermal_event_sequence = 0
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

    def test_inspection_mode_survives_failed_yolo_start_and_records_partial_state(self):
        result = self.gateway.set_inspection_mode({'mode': 'transit', 'task_id': 'task-1'})
        self.assertTrue(result['success'])
        self.assertTrue(result['partial'])
        self.assertIn('yolo', result['errors'])
        self.gateway._behavior.set_mode.assert_called_once_with('transit', 'task-1')

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

    def test_independent_branch_evidence_stores_its_own_frame_pair(self):
        fall = self.evidence()
        fall['worker_branch'] = 'fall'
        fight = self.evidence()
        fight.update(worker_branch='fight', stamp_ns=900,
                     raw_jpeg_base64=base64.b64encode(b'\xff\xd8fight-frame\xff\xd9').decode())
        fight['detections'] = [dict(category='fight', label='fight', scope='scene', confidence=.91,
                                  source_epoch=7, stamp_ns=900, fresh=True, alert=True)]
        fall['branch_captures'] = [fight]
        self.gateway._behavior.capture.return_value = fall
        result = self.gateway.inspection_capture(dict(task_id='task-1', loop_number=1,
            route_point_id=3, direction='current', duration_seconds=.01, save_images=True))
        paths = {d['category']: Path(d['evidence']['raw_image']) for d in result['detections']}
        self.assertNotEqual(paths['fall'], paths['fight'])
        self.assertEqual(paths['fall'].read_bytes(), b'\xff\xd8behavior-frame\xff\xd9')
        self.assertEqual(paths['fight'].read_bytes(), b'\xff\xd8fight-frame\xff\xd9')


if __name__ == '__main__':
    unittest.main()
