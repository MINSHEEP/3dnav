"""Persistent switches gate actual work and evidence, without cameras or ROS."""
import ast
import json
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import types
from typing import Any, Iterable
import unittest
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PACKAGE / 'hikvision_camera'), str(PACKAGE.parent / 'nav_service/lzp_service')]
from detection_switches import DetectionSettings, DetectionSettingsConflict, LABELS
from test_multimodel_detection import _pipeline, _Frame, _Result, _Box
from multimodel_detection import MultiModelDetector
from test_thermal_monitor import sample
from thermal_monitor import ThermalMonitor, ThermalConfigStore
import test_behavior_gateway as gateway_fixture
from test_recognition_runtime import _FaceExtractor, _PlateRecognizer
from recognition_core import RecognitionDatabase
from recognition_runtime import RecognitionRuntime
import camera_web


class PolicyFixture:
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'switches.json'
        self.legacy = Path(self.temp.name) / 'legacy.json'
        self.policy = DetectionSettings(self.path, legacy_behavior_path=self.legacy)
    def tearDown(self): self.temp.cleanup()


class PolicyTests(PolicyFixture, unittest.TestCase):
    def test_standalone_share_copy_loads_without_ros_or_models(self):
        runtime = self.path.parent / 'camera-share'
        runtime.mkdir()
        shutil.copyfile(PACKAGE / 'hikvision_camera/detection_switches.py', runtime / 'detection_switches.py')
        shutil.copyfile(PACKAGE.parent / 'nav_service/lzp_service/lzp_service/detection_settings.py', runtime / 'detection_settings.py')
        script = f"import sys; sys.path.insert(0, {str(runtime)!r}); from detection_switches import LABELS; assert 'rclpy' not in sys.modules; print(len(LABELS))"
        result = subprocess.run([sys.executable, '-I', '-c', script], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '10')
    def test_defaults_and_existing_behavior_preferences_migrate(self):
        self.assertEqual(len(self.policy.snapshot()['enabled']), 10)
        self.assertTrue(all(self.policy.snapshot()['enabled'].values()))
        self.legacy.write_text(json.dumps({'fall_enabled': False, 'fight_enabled': True}))
        state = self.policy.snapshot()
        self.assertFalse(state['enabled']['fall'])
        self.assertTrue(state['enabled']['smoking'])
        self.assertFalse(self.path.exists(), 'read-only attach does not save or launch anything')

    def test_strict_boolean_partial_updates_and_revision_conflict(self):
        for value in ({}, {'fire': 1}, {'unknown': True}, {'fire': 'false'}, []):
            with self.assertRaises(ValueError): self.policy.update(value)
        saved = self.policy.update({'fire': False}, expected_version=0)
        self.assertTrue(saved['enabled']['smoke'])
        self.assertEqual(saved['version'], 1)
        with self.assertRaises(DetectionSettingsConflict):
            self.policy.update({'smoke': False}, expected_version=0)
        with self.assertRaises(DetectionSettingsConflict):
            self.policy.update({'smoke': False}, expected_version=True)
        self.assertTrue(self.policy.enabled('smoke'))

    def test_second_reader_and_restart_see_atomic_updates(self):
        reader = DetectionSettings(self.path, legacy_behavior_path=self.legacy)
        self.assertTrue(reader.enabled('thermal'))
        self.policy.update({'thermal': False})
        self.assertFalse(reader.enabled('thermal'))
        self.assertFalse(DetectionSettings(self.path).enabled('thermal'))
        self.policy.update({'thermal': True})
        self.assertTrue(reader.enabled('thermal'))

    def test_co2_alarm_depends_on_m701_without_overwriting_preference(self):
        self.policy.update({'m701': False})
        state = self.policy.snapshot()
        self.assertTrue(state['enabled']['co2_alarm'])
        self.assertFalse(state['effective']['co2_alarm'])
        self.policy.update({'m701': True, 'co2_alarm': False})
        self.assertTrue(self.policy.enabled('m701'))
        self.assertFalse(self.policy.allows('co2_high'))

    def test_corrupt_file_disables_detection_and_explicit_save_repairs(self):
        self.path.write_text('{invalid')
        state = self.policy.snapshot()
        self.assertTrue(state['error'])
        self.assertFalse(any(state['enabled'].values()))
        self.policy.update({'fire': True}, expected_version=0)
        self.assertTrue(self.policy.enabled('fire'))
        self.assertFalse(self.policy.enabled('smoke'))
        self.assertFalse(self.policy.snapshot()['error'])

    def test_failed_atomic_replace_keeps_saved_switches(self):
        self.policy.update({'fire': False})
        with patch('os.replace', side_effect=OSError('write failed')):
            with self.assertRaises(OSError): self.policy.update({'fire': True})
        self.assertFalse(self.policy.enabled('fire'))
        self.assertEqual(list(self.path.parent.glob('.detection-settings-*.tmp')), [])

    def test_filter_removes_disabled_and_retired_alerts(self):
        self.policy.update({'smoke': False, 'thermal': False, 'smoking': False})
        values = [dict(category=k) for k in ('fire', 'smoke', 'cigarettes', 'smoking', 'high_temperature', 'face')]
        self.assertEqual([x['category'] for x in self.policy.filter_detections(values)], ['fire', 'face'])


class ModelSwitchTests(unittest.TestCase):
    def test_fire_and_smoke_outputs_independent_despite_shared_weight(self):
        detector, primary, _, _ = _pipeline()
        primary.result = _Result({0: 'fire', 1: 'smoke'}, [_Box(0, .8, [1, 2, 20, 30]), _Box(1, .9, [4, 5, 24, 35])])
        result = detector.analyze(_Frame(), enabled={'fire': False, 'smoke': True, 'face': False, 'plate': False})
        self.assertEqual([v.category for v in result.detections], ['smoke'])
        self.assertEqual(result.alarm_detections(), [('smoke', .9)])
        self.assertEqual([v.category for v in result.display_detections], ['smoke'])

    def test_disabled_models_skip_forwards_and_clear_old_display_cache(self):
        detector, primary, face, plate = _pipeline()
        detector.analyze(_Frame(), now=0)
        before = len(primary.calls), face.calls, len(plate.calls)
        off = dict.fromkeys(('fire', 'smoke', 'face', 'plate'), False)
        result = detector.analyze(_Frame(), now=.1, enabled=off)
        self.assertEqual((len(primary.calls), face.calls, len(plate.calls)), before)
        self.assertEqual(result.display_detections, ())
        self.assertEqual(result.ran_detectors, ())
        detector.analyze(_Frame(), now=.11, enabled={'fire': False, 'smoke': False, 'face': True, 'plate': False})
        self.assertEqual(face.calls, before[1] + 1, 'reenabling observes a fresh face immediately')

    def test_off_missing_model_factory_does_not_block_other_features(self):
        forbidden = Mock(side_effect=RuntimeError('disabled model missing'))
        detector = MultiModelDetector(None, model_factories={'primary': forbidden, 'face': forbidden, 'plate': forbidden})
        off = dict.fromkeys(('fire', 'smoke', 'face', 'plate'), False)
        detector.warmup(off)
        self.assertEqual(detector.analyze(_Frame(), enabled=off).detections, ())
        forbidden.assert_not_called()


class SensorSwitchTests(PolicyFixture, unittest.TestCase):
    def test_disabling_during_thermal_capture_discards_old_hot_sample(self):
        monitor = ThermalMonitor(ThermalConfigStore(self.path.parent / 'thermal.json'),
                                 detection_settings=self.policy, hold_seconds=0)
        monitor.set_detection(True)
        monitor.ingest(sample(90))
        with patch.object(monitor._condition, 'wait', side_effect=lambda timeout: self.policy.update({'thermal': False})):
            result = monitor.capture(1)
        self.assertTrue(result['disabled'])
        self.assertEqual(result['detections'], [])
        self.assertTrue(monitor.snapshot(), 'turning detection off preserves preview')

    def test_thermal_switch_preserves_preview_and_clears_queued_events(self):
        monitor = ThermalMonitor(ThermalConfigStore(self.path.parent / 'thermal.json'),
                                 detection_settings=self.policy, hold_seconds=0, wall_clock=lambda: 100)
        monitor.set_backend_state(running=True)
        monitor.set_detection(True)
        self.assertIsNotNone(monitor.ingest(sample(80)))
        self.assertTrue(monitor.events(0)['events'])
        self.policy.update({'thermal': False})
        self.assertFalse(monitor.status()['detection_enabled'])
        self.assertTrue(monitor.status()['ready'])
        self.assertTrue(monitor.snapshot())
        self.assertFalse(monitor.events(0)['events'])
        self.assertEqual(monitor.capture(0)['detections'], [])
        self.assertIsNone(monitor.ingest(sample(90, frame=2)))
        self.policy.update({'thermal': True})
        self.assertTrue(monitor.status()['detection_enabled'], 'manual intent survives permission off/on')
        self.assertIsNotNone(monitor.ingest(sample(85, frame=3)))

    def test_m701_off_closes_serial_without_polling_or_warning(self):
        text = (PACKAGE / 'car_vel/m701_air_quality_bridge.py').read_text(encoding='utf-8')
        node = next(n for n in ast.walk(ast.parse(text)) if isinstance(n, ast.FunctionDef) and n.name == '_poll')
        namespace = dict(json=json, String=lambda **v: types.SimpleNamespace(**v))
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<m701-poll>', 'exec'), namespace)
        self.policy.update({'m701': False})
        owner = types.SimpleNamespace(_detection_settings=self.policy, _close_serial=Mock(),
                                      _base_payload=lambda: {'sensor': 'M701'}, publisher=Mock(), _read_response=Mock())
        namespace['_poll'](owner)
        owner._close_serial.assert_called_once()
        owner._read_response.assert_not_called()
        value = json.loads(owner.publisher.publish.call_args.args[0].data)
        self.assertEqual((value['enabled'], value['valid'], value['values']), (False, False, None))

    def test_disabled_identity_never_records_or_reuses_prior_evidence(self):
        database = RecognitionDatabase(self.path.parent / 'recognition.db')
        try:
            database.upsert_person('p', 'person', [[1., 0., 0.]])
            extractor = _FaceExtractor()
            owner = RecognitionRuntime(database, face_extractor=extractor, plate_recognizer=_PlateRecognizer(),
                                       evidence_writer=lambda *a: 'face.jpg', detection_settings=self.policy)
            detections = [dict(category='face', bbox=[1, 2, 20, 30], confidence=.9, alert=False)]
            owner.process(object(), detections, task_id='task', now=1)
            self.policy.update({'face': False})
            self.assertEqual(owner.process(object(), detections, task_id='task', now=1.1), [])
            self.assertFalse(database.list_events())
            self.policy.update({'face': True})
            owner.process(object(), detections, task_id='task', now=1.2)
            self.assertFalse(database.list_events(), 'pre-disable observations cannot confirm a record')
            owner.process(object(), detections, task_id='task', now=1.3)
            self.assertEqual(len(database.list_events()), 1)
        finally: database.close()

    def test_pending_speech_is_filtered_again_before_playback(self):
        tree = ast.parse((PACKAGE / 'hikvision_camera/yolo_hk_voice.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SpeechQueueService')
        namespace = dict(Any=Any, Iterable=Iterable, queue=queue, threading=threading, sys=sys,
                         ENVIRONMENT_SPEECH={'co2_high': '二氧化碳浓度异常', 'thermal_high': '环境温度异常'})
        exec(compile(ast.Module(body=[cls], type_ignores=[]), '<speech>', 'exec'), namespace)
        owner = object.__new__(namespace['SpeechQueueService'])
        owner._output = Mock(); owner._queue = queue.Queue(); owner._stop = threading.Event()
        owner._stop.set(); owner._closed = False; owner._detection_settings = self.policy
        owner.announce(['fire', 'smoke']); owner.announce_environment('co2_high')
        self.policy.update({'fire': False, 'co2_alarm': False})
        owner._run()
        owner._output.announce.assert_called_once_with(('smoke',))
        owner._output.announce_texts.assert_not_called()


class GatewaySwitchTests(unittest.TestCase):
    def setUp(self):
        self.fixture = gateway_fixture.GatewayBehaviorTests(); self.fixture.setUp()
        self.gateway = self.fixture.gateway
        self.gateway._inspection_mode = 'manual'
        self.policy = DetectionSettings(Path(self.fixture.temp.name) / 'switches.json',
                                        legacy_behavior_path=Path(self.fixture.temp.name) / 'missing.json')
        self.gateway._detection_settings = self.policy
        self.gateway._detection_settings_lock = threading.RLock()
        self.gateway._applied_detection_settings = None
        def configure(value):
            enabled = {key: value[key + '_enabled'] for key in ('fall', 'fight', 'smoking')}
            self.gateway._behavior.status.return_value.update(enabled=enabled, running=any(enabled.values()), ready=any(enabled.values()))
        self.gateway._behavior.configure.side_effect = configure
    def tearDown(self): self.fixture.tearDown()

    def test_settings_and_old_behavior_endpoint_share_authoritative_revision(self):
        self.gateway.configure_detection_settings({'changes': {'fire': False}, 'expected_version': 0})
        self.gateway.configure_behavior({'fall_enabled': False, 'fight_enabled': True, 'smoking_enabled': False, 'expected_version': 1})
        state = self.gateway.detection_settings_status()
        self.assertFalse(state['enabled']['fire'])
        self.assertFalse(state['enabled']['fall'])
        self.assertFalse(state['enabled']['smoking'])
        self.assertTrue(state['enabled']['smoke'])
        self.assertEqual(self.gateway.behavior_status()['settings_version'], 2)

    def test_all_off_does_not_require_models_or_thermal_and_environment_speech_is_gated(self):
        self.gateway.configure_detection_settings({'changes': dict.fromkeys(LABELS, False)})
        status = self.gateway.detection_status()
        self.assertTrue(status['ready'])
        self.assertFalse(status['partial'])
        self.assertEqual(status['requirements'], dict(yolo=False, thermal=False, behavior=False))
        self.assertTrue(self.gateway.inspection_speech({'key': 'co2_high'})['disabled'])
        self.gateway._behavior_speech.announce.assert_not_called()

    def test_environment_speech_does_not_require_visual_yolo(self):
        result = self.gateway.inspection_speech({'key': 'thermal_high'})
        self.assertEqual(result['voice_backend'], 'local')
        self.gateway._behavior_speech.announce.assert_called_once_with('thermal_high')

    def test_http_handler_dispatch_preserves_switches_and_rejects_old_revision(self):
        # Exercise the actual HTTP handlers without requiring local sockets.
        handler = object.__new__(camera_web.CameraRequestHandler)
        handler.server = types.SimpleNamespace(gateway=self.gateway)
        handler.path = '/api/detection/settings'
        replies = []
        handler.send_json = lambda code, value: replies.append((int(code), value))
        handler.do_GET()
        self.assertEqual(replies[-1][1]['version'], 0)
        handler._read_json = lambda limit: {'changes': {'fire': False}, 'expected_version': 0}
        handler.do_POST()
        self.assertEqual(replies[-1][0], 200)
        self.assertTrue(replies[-1][1]['enabled']['smoke'])
        handler._read_json = lambda limit: {'changes': {'smoke': False}, 'expected_version': 0}
        handler.do_POST()
        self.assertEqual(replies[-1][0], 409)
        self.assertTrue(self.policy.enabled('smoke'))


if __name__ == '__main__': unittest.main()
