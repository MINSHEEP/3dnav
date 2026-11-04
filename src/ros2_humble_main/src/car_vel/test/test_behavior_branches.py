"""Exercise branch selection through the real integrated runtime."""
from pathlib import Path
import copy
import sys
import unittest
from unittest.mock import patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE / 'person-behavior-recognition'
sys.path[:0] = [str(ROOT), str(PACKAGE / 'hikvision_camera')]
from behavior_worker import RobotRuntime
from pphuman_runtime.runtime_yolo import load_config


class Backend:
    identities = {}
    last_timing = {}

    def __init__(self):
        self.smoking_calls = self.fight_calls = self.fight_samples = 0

    def reset(self):
        pass

    def observe(self, frame, stamp):
        return [{'track_id': 1, 'bbox': [5, 5, 50, 70], 'score': .9,
                 'keypoints': np.zeros((17, 3)), 'association': 'associated'}]

    def crops(self, frame, people):
        return {p['raw_id']: frame for p in people}

    def smoking(self, crops, observations, shape):
        self.smoking_calls += 1
        return {track_id: {'label': 'other', 'state': 'valid', 'score': None} for track_id in crops}

    def prepare_fight(self, frame):
        self.fight_samples += 1
        return frame

    def fight(self, frames):
        self.fight_calls += 1
        return 0, .9

    def start_fight(self, frames):
        self.fight_calls += 1

    def poll_fight(self):
        return {'class': 0, 'score': .9, 'model_inference_ms': 1.0}


class BranchTests(unittest.TestCase):
    def setUp(self):
        cfg = copy.deepcopy(load_config(ROOT / 'config/inference.robot.yaml'))
        for name in ('temporal', 'hand_to_mouth', 'profile_refinement'):
            cfg['smoking'][name]['enabled'] = False
        self.backend = Backend()
        self.runtime = RobotRuntime(cfg, self.backend)
        self.frame = np.zeros((80, 100, 3), dtype=np.uint8)

    def run_frames(self, enabled, epoch):
        result = None
        for i in range(12):
            result = self.runtime.infer(self.frame, {'stamp_ns': i * 250_000_000 + epoch * 10_000_000_000,
                'source_epoch': epoch, 'request_id': i, 'enabled_branches': enabled})
        return result

    def test_selected_branches_control_real_runtime_work_and_reset(self):
        with patch.object(self.runtime.rules.rules, 'update', side_effect=AssertionError('fall must not run')):
            result = self.run_frames({'fall': False, 'fight': False, 'smoking': True}, 1)
        self.assertGreater(self.backend.smoking_calls, 0)
        self.assertEqual(self.backend.fight_samples, 0)
        self.assertEqual(result['scene']['fight']['state'], 'disabled')
        self.assertEqual(result['persons'][0]['actions']['fall']['state'], 'disabled')
        previous_smoking = self.backend.smoking_calls
        result = self.run_frames({'fall': True, 'fight': True, 'smoking': False}, 2)
        self.assertEqual(self.backend.smoking_calls, previous_smoking)
        self.assertGreater(self.backend.fight_calls, 0)
        self.assertEqual(result['persons'][0]['actions']['smoking']['state'], 'disabled')
        self.assertEqual(result['scene']['fight']['state'], 'valid')


if __name__ == '__main__':
    unittest.main()
