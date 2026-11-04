"""The integrated engine and skeleton renderer must be the supplied project."""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import numpy as np
import cv2
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE / 'person-behavior-recognition'
sys.path.insert(0, str(PACKAGE / 'hikvision_camera'))
from behavior_project import BehaviorSupervisor


class ImportedProjectTests(unittest.TestCase):
    def test_complete_source_matches_import_manifest(self):
        manifest = json.loads((PACKAGE / 'behavior_source_manifest.json').read_text())
        for name, expected in manifest['files'].items():
            with self.subTest(file=name):
                self.assertEqual(hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), expected)

    def test_yolo26_package_is_selected_by_default(self):
        config = yaml.safe_load((ROOT / 'config/inference.yaml').read_text())
        self.assertEqual(config['device'], 'CPU')
        self.assertEqual(config['yolo_pose']['backend'], 'openvino')
        self.assertEqual(config['yolo_pose']['output_format'], 'yolo26_end2end')
        self.assertEqual(config['fight']['backend'], 'openvino')
        self.assertEqual(config['smoking']['backend'], 'openvino')
        for model in config['models'].values():
            self.assertTrue((ROOT / model).exists())

    def test_robot_config_only_changes_behavior_timing(self):
        original = yaml.safe_load((ROOT / 'config/inference.yaml').read_text())
        robot = yaml.safe_load((ROOT / 'config/inference.robot.yaml').read_text())
        self.assertEqual(original['fight']['result_ttl_seconds'], 1.0)
        original['fight']['result_ttl_seconds'] = 2.0
        original['smoking']['result_ttl_seconds'] = 2.0
        original['fight']['max_sample_gap_seconds'] = 1.0
        self.assertEqual(robot, original)
        sys.path.insert(0, str(ROOT))
        from pphuman_runtime.runtime import current
        cached = {'state': 'valid', 'label': 'fight', 'score': .9, 'prediction_stamp_ns': 0}
        self.assertEqual(current(cached, 2_000_000_000, robot['fight'])['state'], 'valid')
        self.assertEqual(current(cached, 2_000_000_001, robot['fight'])['state'], 'unknown')

    def test_fight_window_allows_one_second_gap_without_fabricating_frames(self):
        sys.path.insert(0, str(ROOT))
        from pphuman_runtime.runtime import TimeWindow
        cfg = yaml.safe_load((ROOT / 'config/inference.robot.yaml').read_text())['fight']
        window = TimeWindow(cfg)
        for stamp in (0, .3, .6, 1.4, 1.7, 2.0, 2.3, 2.6):
            window.add(round(stamp * 1e9), stamp)
        self.assertTrue(window.full)
        self.assertEqual(len({value for _, value in window.samples}), 8)
        window.add(round(3.61 * 1e9), 3.61)
        self.assertEqual(len(window.samples), 1)
        window.clear()
        window.add(0, 0)
        window.add(1_000_000_000, 1)
        self.assertEqual(len(window.samples), 2)
        window.add(2_000_000_001, 2)
        self.assertEqual(len(window.samples), 1)

    def test_upstream_skeleton_is_drawn_and_low_confidence_is_not(self):
        service = BehaviorSupervisor()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        points = [[250, 250, 0.] for _ in range(17)]
        points[5], points[6] = [220, 250, .9], [320, 250, .9]
        result = {'persons': [{'bbox': [180, 180, 400, 420], 'keypoints': points}], 'scene': {}}
        _, jpeg = service._render(frame, result, None)
        rendered = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertGreater(int(rendered[250, 270, 0]), 150)
        points[5][2] = .1
        _, jpeg = service._render(frame, result, None)
        rendered = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertLess(int(rendered[250, 270, 0]), 30)

    def test_selection_preserves_each_branch(self):
        self.assertEqual(BehaviorSupervisor._validate_config({'fall_enabled': False, 'fight_enabled': True}),
                         {'fall': False, 'fight': True, 'smoking': False})
