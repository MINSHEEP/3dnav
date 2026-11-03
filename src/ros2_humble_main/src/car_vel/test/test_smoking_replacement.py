"""Smoking integration: real rules/protocol, injected predictor, no camera/robot."""
import copy
import ast
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PACKAGE / 'person_behavior'), str(PACKAGE / 'hikvision_camera')]
from pphuman_runtime.runtime_yolo import RuntimeYolo, load_config
from pphuman_runtime.protocol import read_request, write_message, ProtocolError
from pphuman_runtime.yolo_backend import YoloBackend
from behavior_supervisor import BehaviorSupervisor
from behavior_parallel import ParallelBehaviorSupervisor


def config():
    return load_config(PACKAGE / 'person_behavior/config/inference.yaml')


def person():
    points = np.zeros((17, 3))
    points[:7] = [[100, 60, .95], [110, 50, .95], [90, 50, .95],
                  [125, 55, .9], [75, 55, .9], [140, 110, .95], [60, 110, .95]]
    points[9], points[10] = [115, 110, .99], [50, 200, .99]
    return dict(track_id=1, bbox=[20, 20, 180, 260], keypoints=points, score=.9, association='associated')


class Backend:
    identities = {k: {} for k in ('yolo_pose', 'smoking')}
    last_timing = {}
    smoking = YoloBackend.smoking

    def __init__(self, cfg):
        self.cfg = cfg
        self.smoking_model = self
        self.pose_calls = self.calls = 0
        self.box = (96, 70, 106, 76)
        self.score = .8
        self.count = 1

    def reset(self): pass
    def observe(self, frame, stamp):
        self.pose_calls += 1
        return [dict(person(), observation_stamp_ns=stamp)] if self.count else []
    def crops(self, rgb, people):
        return {p['track_id']: (rgb, [0, 0, 300, 300], p['bbox']) for p in people}
    def predict_image(self, images, visual=False):
        self.calls += 1
        rows = [[0, self.score, *self.box]] if self.score else []
        return dict(boxes_num=[len(rows)], boxes=np.asarray(rows).reshape(-1, 6))


class SmokingRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.cfg['branches']['fight'] = False
        self.backend = Backend(self.cfg)
        self.runtime = RuntimeYolo(self.cfg, self.backend)
        self.image = np.zeros((300, 300, 3), np.uint8)

    def frame(self, t, epoch=1, branches=None):
        h = dict(stamp_ns=round(t * 1e9), request_id=str(t), source_epoch=epoch)
        if branches is not None: h['enabled_branches'] = branches
        return self.runtime.infer(self.image, h)

    def test_one_pose_shared_by_fall_and_smoking_and_actual_detection_score(self):
        result = self.frame(0)
        self.assertEqual(self.backend.pose_calls, 1)
        action = result['persons'][0]['actions']['smoking']
        self.assertEqual((action['label'], action['score'], action['score_kind']), ('smoking', .8, 'detection_score'))
        self.assertIn('fall', result['persons'][0]['actions'])
        self.assertTrue(action['evidence']['selected_detection']['accepted'])

    def test_off_switch_does_not_run_ppyoloe_and_smoking_only_still_uses_pose(self):
        r = self.frame(0, branches={'fall': True, 'smoking': False, 'fight': False})
        self.assertNotIn('smoking', r['persons'][0]['actions'])
        self.assertEqual(self.backend.calls, 0)
        r = self.frame(.1, branches={'fall': False, 'smoking': True, 'fight': False})
        self.assertEqual(self.backend.pose_calls, 2)
        self.assertEqual(r['persons'][0]['actions']['fall']['state'], 'disabled')
        self.assertEqual(r['persons'][0]['actions']['smoking']['label'], 'smoking')

    def test_near_face_needs_distinct_real_hits_then_reset_drops_evidence(self):
        self.backend.box = (142, 70, 152, 76)
        self.assertNotEqual(self.frame(0)['persons'][0]['actions']['smoking']['label'], 'smoking')
        for n in range(1, 5): self.frame(n / 10)
        self.backend.score = .4
        result = self.frame(.5)
        action = result['persons'][0]['actions']['smoking']
        self.assertEqual((action['label'], action['score']), ('smoking', .4))
        self.assertEqual(action['evidence']['hand_to_mouth']['detection_stamp_ns'], 500000000)
        self.assertFalse(self.frame(.6)['persons'][0]['actions']['smoking']['updated'])
        reset = self.frame(.7, epoch=2)
        self.assertEqual(reset['reset_reason'], 'source_epoch_changed')
        self.assertNotEqual(reset['persons'][0]['actions']['smoking']['label'], 'smoking')

    def test_global_two_hz_clock_not_repeated_for_each_pose_frame(self):
        for n in range(11): self.frame(n / 10)
        self.assertEqual(self.backend.pose_calls, 11)
        self.assertEqual(self.backend.calls, 3)
        self.backend.count = 0
        result = self.frame(1.1)
        self.assertEqual(result['persons'], [])
        self.assertFalse(self.runtime.smoking.smoking_hand.tracks)
        self.frame(1.31)  # Bounded bookkeeping may remain for at most the tracker gap.
        self.assertFalse(self.runtime.smoking.people)

    def test_protocol_accepts_three_switches_but_never_smoking_fight_clips(self):
        packet = io.BytesIO()
        head = dict(op='infer', request_id=1, source_epoch=1, stamp_ns=1,
                    shape=[1, 1, 3], payload_bytes=3,
                    enabled_branches={'fall': False, 'fight': False, 'smoking': True})
        write_message(packet, head); packet.write(b'abc'); packet.seek(0)
        self.assertEqual(read_request(packet)[1], b'abc')
        packet = io.BytesIO()
        write_message(packet, dict(head, clip_stamps_ns=[1])); packet.write(b'abc'); packet.seek(0)
        with self.assertRaises(ProtocolError): read_request(packet)

    def test_smoking_cpu_load_shares_one_pose_session_without_loading_fight(self):
        loaded = []
        class Pose:
            version = 'injected'
            session = types.SimpleNamespace(get_providers=lambda: ['CPUExecutionProvider'], get_provider_options=lambda: {})
            def __init__(self, *a): loaded.append('pose')
        def detector(path, cfg):
            loaded.append('smoking')
            self.assertEqual(cfg['device'], 'CPU')
            return object()

        modules = {'paddle': types.SimpleNamespace(enable_static=lambda: None, __version__='injected'),
                   'python.keypoint_preprocess': types.SimpleNamespace(expand_crop=lambda *a: None)}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'vendor/PaddleDetection/deploy').mkdir(parents=True)
            (root / 'pose.xml').write_bytes(b'injected')
            (root / 'pose.bin').write_bytes(b'injected')
            (root / 'smoking').mkdir()
            for name in ('model.xml', 'model.bin', 'infer_cfg.yml'):
                (root / 'smoking' / name).write_bytes(b'injected')
            cfg = config()
            cfg.update(project_root=str(root), models={'yolo_pose': 'pose.xml', 'smoking': 'smoking', 'fight': 'fight'},
                       branches={'fall': True, 'smoking': True, 'fight': False})
            with patch.dict(sys.modules, modules), patch('pphuman_runtime.yolo_backend.YoloPosePredictor', Pose), \
                    patch('pphuman_runtime.openvino_cpu.create_smoking_detector', side_effect=detector):
                backend = YoloBackend(cfg)
        self.assertEqual(loaded, ['pose', 'smoking'])
        self.assertIsNone(backend.fight_model)
        self.assertEqual(set(backend.identities), {'yolo_pose', 'smoking'})



class SmokingGatewayTests(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.speech = []
        self.worker = BehaviorSupervisor(branch='pose', clock=lambda: self.now, speech=self.speech.append)
        self.worker._manual = self.worker._loaded = True
        self.worker._worker = Mock(is_alive=lambda: True)
        self.worker._render = lambda *args: (b'pose-raw', b'pose-annotated')
        self.worker._mode, self.worker._task_id = 'transit', 'task'

    def result(self, **patches):
        action = dict(label='smoking', score=.4, score_kind='temporally_associated_detection_score',
                      state='valid', updated=True, prediction_stamp_ns=self.now,
                      evidence={'head_observable': True, 'hand_to_mouth': {'confirmed': True,
                                'detection': {'score': .4}, 'detection_stamp_ns': self.now,
                                'strong_detection': {'score': .8}, 'short_occlusion': False}})
        action.update(patches)
        return dict(source_epoch=self.worker._epoch, stamp_ns=self.now, status='valid',
                    persons=[dict(track_id=7, bbox=[0, 0, 100, 100], actions={'smoking': action})],
                    pose_status={'state': 'valid'}, scene={'fight': {'state': 'disabled'}})

    def test_smoking_alert_keeps_own_person_score_epoch_and_speech(self):
        self.assertTrue(self.worker._accept(self.result(), None))
        event = self.worker.events('task')[0]
        item = event['detections'][0]
        self.assertEqual((item['category'], item['confidence'], item['scope'], item['track_id']), ('smoking', .4, 'person', 7))
        self.assertEqual(item['measurement_type'], 'associated_detection_score')
        self.assertEqual(self.speech, ['smoking'])
        self.assertTrue(event['raw_jpeg_base64'])

    def test_inspection_notice_callback_uses_worker_context_and_repeat_control(self):
        inspection = []
        self.worker.inspection_speech = inspection.append
        for mode in ('transit', 'inspection', 'manual'):
            with self.subTest(mode=mode):
                self.now += 21_000_000_000
                self.worker._mode = mode
                self.worker._task_id = '' if mode == 'manual' else 'task'
                self.speech.clear()
                inspection.clear()
                self.assertTrue(self.worker._accept(self.result(), None))
                self.worker._accept(self.result(), None)
                self.assertEqual(inspection, ['smoking'] if mode != 'manual' else [])
                self.assertEqual(self.speech, ['smoking'] if mode == 'manual' else [])

    def test_cached_occluded_stale_results_do_not_produce_new_alerts(self):
        inspection = []
        self.worker.inspection_speech = inspection.append
        for patch in ({'updated': False}, {'prediction_stamp_ns': self.now - 2_000_000_000},
                      {'evidence': {'hand_to_mouth': {'short_occlusion': True}}},
                      {'score_kind': 'model_probability'}):
            self.worker._accept(self.result(**patch), None)
            self.assertFalse(self.worker.events('task'))
            self.assertFalse(self.speech)
            self.assertFalse(inspection)

    def test_two_switch_settings_migrate_and_pose_worker_cannot_enable_fight(self):
        old = {'fall_enabled': False, 'fight_enabled': False}
        self.assertTrue(BehaviorSupervisor._validate_config(old)['smoking'])
        # Avoid launching fake liveness sentinels; inspect actual configuration routing.
        self.worker._worker = None
        self.worker._manual = False
        self.worker._mode = 'manual'
        self.worker.configure({'fall_enabled': False, 'fight_enabled': True, 'smoking_enabled': True})
        self.assertEqual(self.worker._enabled, {'fall': False, 'fight': False, 'smoking': True})
        service = ParallelBehaviorSupervisor()
        self.assertEqual(set(service._workers), {'pose', 'fight'})
        self.assertEqual(service._workers['pose']._enabled, {'fall': True, 'fight': False, 'smoking': True})
        service.close()

    def test_legacy_voice_no_longer_accepts_cigarette_categories(self):
        text = (PACKAGE / 'hikvision_camera/yolo_hk_voice.py').read_text(encoding='utf-8')
        tree = ast.parse(text)
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in ('normalize_label', 'classify_alarm_label')]
        namespace = {}
        exec(compile(ast.Module(body=functions, type_ignores=[]), '<voice-categories>', 'exec'), namespace)
        classify = namespace['classify_alarm_label']
        self.assertEqual(classify('FIRE'), 'fire')
        self.assertEqual(classify('smoke'), 'smoke')
        for label in ('cigarette', 'CigaretteButt', 'butt', 'smoking'):
            self.assertIsNone(classify(label))


if __name__ == '__main__': unittest.main()
