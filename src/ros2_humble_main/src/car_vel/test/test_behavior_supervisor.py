"""Real pipe/lifecycle tests plus temporal safety contracts without ML imports."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

CAMERA = Path(__file__).resolve().parents[1] / 'hikvision_camera'
sys.path.insert(0, str(CAMERA))
from behavior_supervisor import BehaviorSupervisor, isolated_environment


class Frame:
    shape = (2, 3, 3)
    nbytes = 18
    def copy(self): return copy.copy(self)
    def tobytes(self, order='C'): return b'\0' * self.nbytes


FAKE_WORKER = '''import json, os, struct, sys, time
def read(n):
    data = b''
    while len(data) < n:
        chunk = sys.stdin.buffer.read(n-len(data))
        if not chunk: raise EOFError()
        data += chunk
    return data
def send(v):
    data=json.dumps(v).encode()
    sys.stdout.buffer.write(struct.pack('>I',len(data))+data)
    sys.stdout.buffer.flush()
send({'type':'ready','sampling':{'fight':{'result_ttl_seconds':1}}})
while True:
    value=json.loads(read(struct.unpack('>I',read(4))[0]))
    op=value['op']
    if op=='shutdown':
        send({'type':'shutdown_ack'})
        break
    if op=='reset':
        send({'type':'reset_ack','source_epoch':value['source_epoch']})
        continue
    data=read(value['payload_bytes'])
    if os.path.exists('hang'): time.sleep(10)
    send({'type':'result','request_id':value['request_id'],'source_epoch':value['source_epoch'],
          'stamp_ns':value['stamp_ns'],'persons':[], 'pose_status':{'state':'valid','person_count':0},
          'scene':{'fight':{'label':'other','score':.95,'state':'valid','updated':True,
                            'prediction_stamp_ns':value['stamp_ns']}},
          'environment':{k:os.environ.get(k) for k in ('PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH','OMP_NUM_THREADS')},
          'enabled_branches':value['enabled_branches']})
'''


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value: return value
        time.sleep(.01)
    raise AssertionError('condition did not become true')


class BehaviorProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        package = self.root / 'pphuman_runtime'
        package.mkdir()
        (package / '__init__.py').write_text('')
        (package / 'worker.py').write_text(FAKE_WORKER)
        self.service = BehaviorSupervisor(runtime_dir=self.root, python=sys.executable,
                                          startup_timeout=2, inference_timeout=.4)
        self.service._render = lambda frame, result, raw: (raw or b'raw', b'annotated')
    def tearDown(self):
        self.service.close()
        self.temp.cleanup()
    def test_real_worker_protocol_clean_environment_and_own_child_stop(self):
        with mock.patch.dict('os.environ', {'PYTHONPATH':'untrusted', 'PYTHONHOME':'bad', 'LD_LIBRARY_PATH':'ros'}):
            self.service.start()
            wait_for(lambda: self.service._loaded)
        process = self.service._process
        self.assertTrue(self.service.submit(Frame(), stamp_ns=time.monotonic_ns(), source_key='usb', raw_jpeg=b'jpeg'))
        status = wait_for(lambda: self.service.status() if self.service.status()['fresh'] else None)
        self.assertTrue(status['ready'])
        self.assertEqual(status['branches']['fall']['label'], 'no_person')
        self.assertEqual(status['result']['environment'], {'PYTHONPATH':None,'PYTHONHOME':None,'LD_LIBRARY_PATH':None,'OMP_NUM_THREADS':'2'})
        self.assertEqual(status['result']['enabled_branches'], {'fall':True, 'fight':True, 'smoking':True})
        self.assertEqual(self.service.frame()[0], b'annotated')
        self.service.stop()
        self.assertIsNotNone(process.poll())
        self.assertFalse(self.service.status()['running'])
        with self.assertRaises(RuntimeError): self.service.frame()
    def test_hung_inference_is_terminated_and_reports_partial_failure(self):
        (self.root / 'hang').write_text('')
        self.service.start()
        wait_for(lambda: self.service._loaded)
        process = self.service._process
        self.service.submit(Frame(), stamp_ns=time.monotonic_ns(), source_key='usb')
        wait_for(lambda: self.service.status()['error'])
        wait_for(lambda: process.poll() is not None)
        self.assertFalse(self.service.status()['fresh'])
        self.assertEqual(self.service.status()['branches']['fight']['state'], 'error')
    def test_missing_environment_is_explicit_and_does_not_load_host_models(self):
        self.service.python = self.root / 'missing-python'
        self.service.start()
        wait_for(lambda: self.service.status()['error'])
        self.assertFalse(self.service.status()['ready'])
        self.assertIsNone(self.service._process)


class BehaviorTemporalTests(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.speech = []
        self.service = BehaviorSupervisor(clock=lambda:self.now, speech=self.speech.append)
        self.service._manual = True
        self.service._loaded = True
        self.service._worker = mock.Mock(is_alive=lambda: True)
        self.service._render = lambda frame, result, raw: (raw or b'raw', b'annotated')
    def result(self, fall='fall', fight='fight', updated=True):
        return dict(type='result', source_epoch=self.service._epoch, stamp_ns=self.now,
            persons=[dict(track_id=4, bbox=[0,0,2,2], score=.99,
                actions={'fall':dict(label=fall, score=None, score_kind='rule_evidence', state='valid', updated=updated)})],
            scene={'fight':dict(label=fight, score=.91, state='valid', updated=updated, prediction_stamp_ns=self.now)})
    def test_fall_is_unscored_and_lying_does_not_trigger_alert_or_voice(self):
        self.service._accept(self.result(fall='lying', fight='other'), Frame())
        detection = self.service.status()['detections'][0]
        self.assertIsNone(detection['confidence'])
        self.assertEqual(detection['score_kind'], 'rule_evidence')
        self.assertFalse(detection['alert'])
        self.assertEqual(self.speech, [])
    def test_fight_scene_only_and_cache_does_not_realert(self):
        self.service._accept(self.result(), Frame())
        fight = self.service.status()['detections'][1]
        self.assertEqual(fight['scope'], 'scene')
        self.assertNotIn('track_id', fight)
        self.assertEqual(fight['confidence'], .91)
        self.assertEqual(set(self.speech), {'fall','fight'})
        self.now += 10_000_000
        self.service._accept(self.result(updated=False), Frame())
        self.assertTrue(all(not value['alert'] for value in self.service.status()['detections']))
        self.assertEqual(len(self.speech), 2)
    def test_wall_elapsed_fight_ttl_hides_detections_events_and_overlay(self):
        self.service._mode, self.service._task_id = 'transit', 'task1'
        self.service._accept(self.result(), Frame())
        self.assertTrue(self.service.status()['fresh'])
        self.now += 1_000_000_001
        self.assertFalse(self.service.status()['fresh'])
        self.assertEqual(self.service.status()['detections'], [])
        self.assertEqual(self.service.events('task1'), [])
        with self.assertRaises(RuntimeError): self.service.frame()
    def test_preview_request_can_retrieve_the_fresh_frame_reported_before_next_result(self):
        self.service._render = lambda frame, result, raw: (b'raw', str(result['stamp_ns']).encode())
        self.service._accept(self.result(), Frame())
        status = self.service.status()
        epoch, stamp = status['source_epoch'], status['frame_stamp_ns']
        self.now += 500_000_000
        self.service._accept(self.result(), Frame())
        self.assertNotEqual(self.service.frame()[2], int(stamp))
        self.assertEqual(self.service.frame(epoch=str(epoch), stamp=stamp), (stamp.encode(), epoch, int(stamp)))
        self.now += 500_000_001
        with self.assertRaises(LookupError):
            self.service.frame(epoch=str(epoch), stamp=stamp)
        current = self.service.status()
        self.service.reset_source()
        with self.assertRaises(LookupError):
            self.service.frame(epoch=str(epoch), stamp=current['frame_stamp_ns'])
    def test_preview_can_distinguish_pose_and_fight_at_the_same_capture_time(self):
        self.service._render = lambda frame, result, raw: (raw, raw)
        result = self.result(updated=False)
        self.service._accept(result, Frame(), b'fight', stream='fight')
        self.service._accept(result, Frame(), b'pose', stream='pose')
        for view in ('pose', 'fight'):
            image, _, _ = self.service.frame(epoch=self.service._epoch, stamp=self.now, view=view)
            self.assertEqual(image, view.encode())
    def test_duplicate_stamp_source_switch_size_change_clear_history_and_inflight_result(self):
        frame = Frame()
        self.assertTrue(self.service.submit(frame, stamp_ns=self.now, source_key='usb'))
        epoch = self.service._epoch
        stale = self.result()
        self.assertFalse(self.service.submit(frame, stamp_ns=self.now, source_key='usb'))
        self.now += 1
        self.assertTrue(self.service.submit(frame, stamp_ns=self.now, source_key='body'))
        self.assertGreater(self.service._epoch, epoch)
        self.assertFalse(self.service._accept(stale, frame))
        epoch = self.service._epoch
        frame.shape, frame.nbytes = (3,3,3), 27
        self.now += 1
        self.assertTrue(self.service.submit(frame, stamp_ns=self.now, source_key='body'))
        self.assertGreater(self.service._epoch, epoch)
    def test_paused_task_blocks_voice_and_task_scope_clears_old_events(self):
        self.service._mode, self.service._task_id = 'paused', 'task1'
        self.service._accept(self.result(), Frame())
        self.assertEqual(self.speech, [])
        self.assertEqual(self.service.events('task1'), [])
        self.service._mode = 'transit'
        with self.service._condition:
            self.service._invalidate_locked('resume')
        self.service._accept(self.result(), Frame())
        self.assertEqual(self.service.events('task2'), [])
        with self.service._condition:
            self.service._invalidate_locked('new_task')
        self.assertFalse(self.service.status()['fresh'])
    def test_result_expiring_during_render_never_emits_alert(self):
        def slow_render(*args):
            self.now += 2_000_000_000
            return b'raw', b'annotated'
        self.service._render = slow_render
        self.assertFalse(self.service._accept(self.result(), Frame()))
        self.assertEqual(self.speech, [])
    def test_configuration_requires_explicit_booleans(self):
        with self.assertRaises(ValueError): self.service.configure({'fall_enabled':'false','fight_enabled':True})
        env = isolated_environment({'PYTHONPATH':'bad','PYTHONHOME':'bad','LD_LIBRARY_PATH':'bad','LD_PRELOAD':'bad'})
        self.assertFalse(any(key in env for key in ('PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH','LD_PRELOAD')))


if __name__ == '__main__': unittest.main()
