"""Regression for measured 1.03 s pose starvation; real captures, no padded clips."""
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch, Mock

import numpy as np

from test_behavior_runtime_cpu import config, FakeBackend, RuntimeYolo
from pphuman_runtime.protocol import ProtocolError, read_request, write_message, read_message
from pphuman_runtime.worker import serve

CAMERA = Path(__file__).resolve().parents[1] / 'hikvision_camera'
sys.path.insert(0, str(CAMERA))
from behavior_supervisor import BehaviorSupervisor
from behavior_parallel import ParallelBehaviorSupervisor


class ClipRuntimeTests(unittest.TestCase):
    def test_old_two_switch_clip_cannot_implicitly_enable_new_smoking_branch(self):
        from pphuman_runtime.runtime_yolo import load_config
        cfg = load_config(Path(__file__).resolve().parents[1] / 'person_behavior/config/inference.yaml')
        backend = FakeBackend()
        runtime = RuntimeYolo(cfg, backend)
        frame = np.zeros((16, 16, 3), np.uint8)
        samples = [(i * 240_000_000, frame) for i in range(8)]
        result = runtime.infer(frame, dict(stamp_ns=samples[-1][0], source_epoch=1, request_id=1,
                                          enabled_branches={'fall': False, 'fight': True}), fight_samples=samples)
        self.assertEqual(result['scene']['fight']['state'], 'valid')
        self.assertFalse(result['enabled_branches']['smoking'])
        self.assertEqual(backend.pose_calls, 0)
    def test_slow_consumer_still_classifies_eight_real_capture_frames(self):
        backend = FakeBackend()
        runtime = RuntimeYolo(config(), backend)
        frame = np.zeros((32, 64, 3), np.uint8)
        # Process requests >1 s apart while retaining actual 0.24 s captures.
        for end in (1.68, 2.88, 4.08):
            stamps = [round((end - 1.68 + index * .24) * 1e9) for index in range(8)]
            result = runtime.infer(frame, dict(request_id=str(end), stamp_ns=stamps[-1], source_epoch=1,
                enabled_branches={'fall': False, 'fight': True}), fight_samples=[(stamp, frame) for stamp in stamps])
            self.assertEqual(result['scene']['fight']['state'], 'valid')
            self.assertEqual(result['scene']['fight']['window']['samples'], 8)
        self.assertEqual(backend.fight_calls, 3)
        self.assertEqual(backend.pose_calls, 0)

    def test_gap_is_not_repaired_by_padding_or_larger_threshold(self):
        runtime = RuntimeYolo(config(), FakeBackend())
        frame = np.zeros((32, 64, 3), np.uint8)
        stamps = [0, .24, .48, .72, 1.5, 1.74, 1.98, 2.22]
        stamps = [round(s * 1e9) for s in stamps]
        result = runtime.infer(frame, dict(request_id=1, stamp_ns=stamps[-1], source_epoch=1,
            enabled_branches={'fall': False, 'fight': True}), fight_samples=[(stamp, frame) for stamp in stamps])
        self.assertEqual(result['scene']['fight']['state'], 'warming')
        self.assertEqual(result['scene']['fight']['window']['samples'], 4)
        self.assertEqual(runtime.backend.fight_calls, 0)

    def test_jittered_capture_clock_not_sampled_twice(self):
        runtime = RuntimeYolo(config(), FakeBackend())
        frame = np.zeros((32, 64, 3), np.uint8)
        stamps = [round(s * 1e9) for s in (0.24, .47, .71, .94, 1.17, 1.41, 1.64, 1.87)]
        result = runtime.infer(frame, dict(request_id=1, stamp_ns=stamps[-1], source_epoch=1,
            enabled_branches={'fall': False, 'fight': True}), fight_samples=[(stamp, frame) for stamp in stamps])
        self.assertEqual(result['scene']['fight']['state'], 'valid')

    def packet(self, stamps, branches=None, payload=None):
        stream = io.BytesIO()
        write_message(stream, dict(op='infer', request_id=1, source_epoch='camera-2', stamp_ns=stamps[-1],
            enabled_branches=branches or {'fall': False, 'fight': True}, shape=[2, 3, 3],
            clip_stamps_ns=stamps, payload_bytes=18*len(stamps)))
        stream.write(b'\0' * (18*len(stamps)) if payload is None else payload)
        stream.seek(0)
        return stream

    def test_protocol_checks_count_order_length_scope_and_bounds(self):
        for stamps in ([1, 1], [2, 1], list(range(9))):
            with self.assertRaises(ProtocolError): read_request(self.packet(stamps))
        with self.assertRaises(ProtocolError): read_request(self.packet([1], {'fall': True, 'fight': True}))
        with self.assertRaises(ProtocolError): read_request(self.packet([1, 2], payload=b'\0' * 18))
        header, payload = read_request(self.packet(list(range(8))))
        self.assertEqual(len(payload), 8*18)
        self.assertEqual(header['source_epoch'], 'camera-2')

    def test_real_worker_serve_consumes_capture_clip(self):
        runtime = RuntimeYolo(config(), FakeBackend())
        stamps = [i * 240_000_000 for i in range(8)]
        incoming, outgoing = self.packet(stamps), io.BytesIO()
        serve(runtime, config(), incoming, outgoing)
        outgoing.seek(0)
        self.assertEqual(read_message(outgoing)['type'], 'ready')
        result = read_message(outgoing)
        self.assertEqual(result['scene']['fight']['state'], 'valid')
        self.assertEqual(result['stamp_ns'], stamps[-1])


class CaptureSamplerTests(unittest.TestCase):
    @patch.dict(sys.modules, {'cv2': types.SimpleNamespace(resize=lambda image, size:
        np.zeros((size[1], size[0], 3), np.uint8))})
    def test_pending_overwrite_keeps_bounded_real_clip_and_source_reset_clears(self):
        now = [1_000_000_000]
        service = BehaviorSupervisor(branch='fight', clock=lambda: now[0])
        service._manual = True
        service._fight_sampling = config()['fight']
        frame = np.zeros((720, 1280, 3), np.uint8)
        for index in range(70):
            now[0] = 1_000_000_000 + index * 50_000_000
            service.submit(frame, stamp_ns=now[0], source_key='camera')
        self.assertEqual(len(service._clip), 8)
        stamps = [stamp for stamp, _ in service._clip]
        self.assertEqual(len(set(stamps)), 8)
        self.assertLessEqual(service._clip[-1][1].shape[1], 640)
        self.assertEqual(service._pending[1], stamps[-1])
        self.assertLess(sum(img.nbytes for _, img in service._clip), 8*640*640*3+1)
        now[0] += 700_000_000
        service.submit(frame, stamp_ns=now[0], source_key='camera')
        self.assertEqual(len(service._clip), 1, 'actual camera gap discards history')
        now[0] += 250_000_000
        service.submit(frame, stamp_ns=now[0], source_key='different-camera')
        self.assertEqual(len(service._clip), 1, 'source epoch cannot inherit old captures')
        service.reset_source()
        self.assertFalse(service._clip)
        self.assertIsNone(service._pending)

    def test_expired_fight_result_not_made_fresh_by_other_worker(self):
        now = 10_000_000_000
        service = BehaviorSupervisor(branch='fight', clock=lambda: now)
        service._manual = True
        result = dict(stamp_ns=now-2_000_000_000, source_epoch=service._epoch,
            scene={'fight': {'state': 'valid', 'prediction_stamp_ns': now-2_000_000_000}}, timing_ms={'fight': 1200})
        self.assertFalse(service._accept(result, None))
        self.assertEqual(service.status()['stale_results'], 1)
        self.assertFalse(service.status()['fresh'])


class IndependentWorkerTests(unittest.TestCase):
    def test_failed_fall_does_not_hide_fresh_fight_or_mix_preview_provenance(self):
        service = ParallelBehaviorSupervisor()
        for key, worker in service._workers.items():
            worker._manual = True
            worker._stop.clear()
            worker._worker = Mock(is_alive=lambda: True)
            worker._loaded = True
            worker._render = lambda frame, result, raw: (b'raw', b'annotated')
        fall, fight = service._workers['pose'], service._workers['fight']
        fall._error = 'pose model failed'
        stamp = time.monotonic_ns()
        fight._accept(dict(source_epoch=fight._epoch, stamp_ns=stamp, persons=[],
            scene={'fight': {'state': 'valid', 'label': 'fight', 'score': .95,
                             'updated': True, 'prediction_stamp_ns': stamp}}), None)
        status = service.status()
        self.assertTrue(status['running'])
        self.assertFalse(status['ready'])
        self.assertTrue(status['partial'])
        self.assertTrue(status['branches']['fight']['ready'])
        self.assertTrue(status['branches']['fight']['fresh'])
        self.assertEqual(status['branches']['fall']['state'], 'error')
        self.assertEqual(status['preview_branch'], 'fight')
        frame, epoch, frame_stamp = service.frame()
        self.assertEqual((epoch, frame_stamp), (status['source_epoch'], status['stamp_ns']))
        self.assertEqual(frame, b'annotated')
        self.assertIn('pose model failed', status['error'])
        # Do not stop fake threads, which are only used as liveness sentinels.

    def test_real_pipe_slow_pose_does_not_starve_fight_and_stop_owns_both_children(self):
        test_root = Path(__file__).resolve().parent
        deps = Path(__file__).resolve().parents[5] / 'tmp/behavior-test-deps'
        # Use production runtime, protocol, supervisor, sampler and coordinator;
        # replace only expensive model forwards with deterministic fake models.
        script = f'''import sys, time
sys.path.insert(0, {str(deps)!r})
sys.path.insert(0, {str(test_root)!r})
from test_behavior_runtime_cpu import config, FakeBackend, RuntimeYolo
from pphuman_runtime.worker import serve
class Backend(FakeBackend):
    def observe(self, frame, stamp):
        time.sleep(1.05)
        return []
cfg = config()
branches = set(sys.argv[-1].split(','))
cfg['branches'] = {{key:key in branches for key in ('fall','fight','smoking')}}
serve(RuntimeYolo(cfg, Backend()), cfg, sys.stdin.buffer, sys.stdout.buffer)
'''
        with tempfile.TemporaryDirectory() as folder:
            package = Path(folder) / 'pphuman_runtime'
            package.mkdir()
            (package / '__init__.py').write_text('')
            (package / 'worker.py').write_text(script, encoding='utf-8')
            # Avoid shadowing real runtime in the fixture package.
            script = script.replace('from test_behavior_runtime_cpu import',
                "sys.modules.pop('pphuman_runtime', None)\nfrom test_behavior_runtime_cpu import")
            (package / 'worker.py').write_text(script, encoding='utf-8')
            service = ParallelBehaviorSupervisor(runtime_dir=folder, python=sys.executable, startup_timeout=5)
            for worker in service._workers.values():
                worker._render = lambda frame, result, raw: (b'raw', b'annotated')
            processes = []
            try:
                service.start()
                deadline = time.monotonic()+7
                observed = None
                while time.monotonic() < deadline:
                    service.submit(np.zeros((32,64,3), np.uint8), stamp_ns=time.monotonic_ns(), source_key='camera')
                    status = service.status()
                    if status['branches']['fight']['ready'] and status['branches']['fall']['ready']:
                        observed = status
                        break
                    time.sleep(.04)
                self.assertIsNotNone(observed, service.status())
                self.assertTrue(observed['branches']['fight']['fresh'])
                self.assertEqual(observed['branches']['fight']['window']['samples'], 8)
                self.assertTrue(observed['workers']['pose']['running'])
                processes = [worker._process for worker in service._workers.values()]
                self.assertEqual(len({p.pid for p in processes}), 2)
                fight_pid = service._workers['fight']._process.pid
                pose_pid = service._workers['pose']._process.pid
                service.configure({'fall_enabled': True, 'fight_enabled': True, 'smoking_enabled': False})
                restart_deadline = time.monotonic() + 4
                while time.monotonic() < restart_deadline and not service._workers['pose']._loaded:
                    time.sleep(.02)
                self.assertTrue(service._workers['pose']._loaded)
                self.assertNotEqual(service._workers['pose']._process.pid, pose_pid)
                self.assertEqual(service._workers['fight']._process.pid, fight_pid)
                processes += [service._workers['pose']._process]
                service.reset_source()
                self.assertFalse(service.status()['fresh'])
            finally:
                service.close()
            self.assertTrue(all(p.poll() is not None for p in processes))


if __name__ == '__main__':
    unittest.main()
