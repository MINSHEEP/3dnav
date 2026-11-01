"""Temporal behavior evidence must not become invented probability alerts."""
import ast
import json
import math
from pathlib import Path
import tempfile
import unittest
from typing import Any, Dict, Iterable, List

from lzp_service.inspection_database import InspectionDatabase


SOURCE = Path(__file__).resolve().parents[1] / 'lzp_service' / 'inspection_task_manager.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
names = {'_confirmed_detections', '_alert_storage_fields', '_detection_summary',
         '_behavior_capture_detail'}
namespace = dict(Any=Any, Dict=Dict, Iterable=Iterable, List=List, math=math,
                 MIN_CONFIRMED_DETECTION_CONFIDENCE=0.5)
exec(compile(ast.Module(body=[node for node in tree.body
                            if isinstance(node, ast.FunctionDef) and node.name in names],
                        type_ignores=[]), str(SOURCE), 'exec'), namespace)


def sample(**changes):
    return dict(category='fall', label='fall', confidence=None, score_kind='rule_evidence',
                state='valid', updated=True, fresh=True, alert=True,
                source_epoch='camera-2', stamp_ns=123456, **changes)


class BehaviorInspectionTests(unittest.TestCase):
    def test_fall_is_valid_without_probability(self):
        item = sample()
        self.assertEqual(namespace['_confirmed_detections']([item]), [item])
        self.assertNotIn('%', namespace['_detection_summary'](item))

    def test_unknown_stale_lying_and_fabricated_scores_are_not_alerts(self):
        for patch in ({'state': 'unknown'}, {'updated': False}, {'fresh': False},
                      {'label': 'lying'}, {'confidence': 0.99}, {'stamp_ns': 0},
                      {'source_epoch': ''}, {'alert': False}):
            item = sample()
            item.update(patch)
            self.assertEqual(namespace['_confirmed_detections']([item]), [], patch)

    def test_fight_requires_scene_evidence_and_finite_probability(self):
        item = sample()
        item.update(category='fight', label='fight', confidence=0.91,
                    score_kind='model_probability', scope='scene')
        self.assertEqual(namespace['_confirmed_detections']([item]), [item])
        for patch in ({'scope': 'person'}, {'confidence': float('inf')},
                      {'confidence': float('nan')}, {'confidence': 1.1}):
            changed = dict(item, **patch)
            self.assertEqual(namespace['_confirmed_detections']([changed]), [])

    def test_behavior_never_borrows_a_different_yolo_frame(self):
        item = sample()
        fields = namespace['_alert_storage_fields'](item, {'raw_image': 'yolo.jpg'})
        self.assertEqual(fields['raw_image'], '')
        item['evidence'] = {'raw_image': 'behavior-123.jpg', 'annotated_image': 'behavior-123-box.jpg'}
        fields = namespace['_alert_storage_fields'](item, {'raw_image': 'yolo.jpg'})
        self.assertEqual(fields['raw_image'], 'behavior-123.jpg')
        self.assertIsNone(fields['confidence'])
        self.assertEqual(fields['measurement']['stamp_ns'], 123456)

    def test_rule_evidence_roundtrips_through_existing_database(self):
        with tempfile.TemporaryDirectory() as directory:
            database = InspectionDatabase(str(Path(directory) / 'inspection.db'))
            database.create_task('task', 'test', {'id': 1, 'name': 'route', 'map_name': 'map'},
                                 {}, {}, {'task_id': 'task', 'state': 'preparing'})
            fields = namespace['_alert_storage_fields'](sample(), {})
            identifier = database.upsert_alert('task', dict(fields, dedup_key='fall-1', category='fall'))
            database.upsert_alert('task', dict(fields, dedup_key='fall-1', category='fall'))
            alerts = database.get_alerts('task')
            self.assertEqual(len(alerts), 1)
            self.assertEqual(alerts[0]['id'], identifier)
            self.assertIsNone(alerts[0]['confidence'])
            self.assertEqual(alerts[0]['measurement_type'], 'rule_evidence')
            self.assertEqual(alerts[0]['measurement']['source_epoch'], 'camera-2')
            json.dumps(alerts, allow_nan=False)

    def test_partial_temporal_capture_is_not_described_as_normal(self):
        detail = namespace['_behavior_capture_detail']({'behavior': {'branches': {
            'fall': {'enabled': True, 'state': 'warming'},
            'fight': {'enabled': False, 'state': 'disabled'}}}})
        self.assertIn('未形成有效结论', detail)
        self.assertIn('跌倒', detail)
        self.assertNotIn('打架', detail)


if __name__ == '__main__':
    unittest.main()
