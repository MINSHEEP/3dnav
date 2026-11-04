"""Regression seams for the 2026-10-13 Xingji 3D audit."""
import ast
import math
from pathlib import Path
import sys
import textwrap
import threading
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lzp_service.automatic_terrain_navigation import AutomaticTerrainController
from lzp_service.terrain_navigation import TerrainCapabilityModel, TerrainObservation
from lzp_service.terrain_observer import estimate_local_terrain


def floor_cloud():
    x, y = np.meshgrid(np.arange(.11, 1.30, .03), np.arange(-.44, .45, .03))
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))


class XingjiTerrainRegressions(unittest.TestCase):
    def test_absolute_map_height_allowed_only_by_3d_executor(self):
        points = [{'name': 'floor', 'z': -.58, 'terrain_type': 'flat'}]
        model = TerrainCapabilityModel()
        self.assertTrue(model.planar_execution_blockers(points))
        self.assertEqual(model.planar_execution_blockers(
            points, allow_ground_elevation=True), [])
        self.assertTrue(model.planar_execution_blockers(
            [{**points[0], 'terrain_type': 'stairs_up'}],
            allow_ground_elevation=True))

    def test_disabled_obstacle_mode_is_never_requested(self):
        controller = AutomaticTerrainController()
        controller.configure(enabled=True, enabled_modes={'ramp'})
        observation = TerrainObservation(
            terrain_type='curb', step_height_m=.08, confidence=1)
        for _ in range(5):
            decision = controller.update(observation, observation_fresh=True,
                                         navigation_active=True, campus_active=False)
            self.assertFalse(decision.safe)
            self.assertEqual(decision.requested_mode, '')

    def test_disabling_automatic_modes_cannot_disable_terrain_safety(self):
        controller=AutomaticTerrainController()
        controller.configure(enabled=False,enabled_modes=[])
        decision=controller.update(TerrainObservation(terrain_type='unknown'),
            observation_fresh=True,navigation_active=True,campus_active=False)
        self.assertFalse(decision.safe)
        decision=controller.update(TerrainObservation(),observation_fresh=True,
            navigation_active=True,campus_active=False)
        self.assertTrue(decision.safe)

    def test_one_observation_cannot_supply_three_confirmations(self):
        controller = AutomaticTerrainController()
        observation = TerrainObservation(terrain_type='rough', roughness_m=.04)
        for _ in range(5):
            decision = controller.update(observation, observation_fresh=True,
                navigation_active=True, campus_active=False, observation_sequence=1)
            self.assertEqual(decision.requested_mode, '')
        controller.update(observation, observation_fresh=True,
            navigation_active=True, campus_active=False, observation_sequence=2)
        decision = controller.update(observation, observation_fresh=True,
            navigation_active=True, campus_active=False, observation_sequence=3)
        self.assertEqual(decision.requested_mode, 'rough_ground')

    def test_observed_ten_cm_cluster_uses_low_obstacle_mode(self):
        x, y = np.meshgrid(np.arange(.50, .81, .02), np.arange(-.15, .16, .02))
        low = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .10)))
        observation = estimate_local_terrain(np.vstack((floor_cloud(), low)),
                                             robot_x=0, robot_y=0, yaw=0)
        self.assertEqual(observation.terrain_type, 'curb')
        self.assertAlmostEqual(observation.step_height_m, .10, places=2)
        assessment = TerrainCapabilityModel().assess(observation, {'obstacle'})
        self.assertTrue(assessment.traversable)
        self.assertEqual(assessment.required_mode, 'obstacle')

    def test_missing_returns_are_uncertainty_not_confirmed_hole(self):
        points = floor_cloud()
        points = points[(points[:, 0] < .46) | (points[:, 0] > .73)]
        observation = estimate_local_terrain(points, robot_x=0, robot_y=0, yaw=0)
        self.assertFalse(observation.negative_obstacle)
        self.assertEqual(observation.terrain_type, 'unknown')
        self.assertFalse(TerrainCapabilityModel().assess(observation).traversable)

    def test_recent_measured_ground_survives_near_field_occlusion(self):
        from lzp_service.rolling_terrain import RollingGroundMemory
        memory = RollingGroundMemory(max_age_seconds=2)
        memory.update(floor_cloud(), stamp_seconds=1, robot_x=0, robot_y=0, yaw=0)
        visible = floor_cloud()[floor_cloud()[:, 0] >= 1]
        memory.update(visible, stamp_seconds=1.2, robot_x=0, robot_y=0, yaw=0)
        observation = memory.observe(visible, stamp_seconds=1.2,
                                     robot_x=0, robot_y=0, yaw=0)
        self.assertTrue(TerrainCapabilityModel().assess(observation).traversable)
        expired = memory.observe(visible, stamp_seconds=4,
                                 robot_x=0, robot_y=0, yaw=0)
        self.assertFalse(TerrainCapabilityModel().assess(expired).traversable)
        memory.clear()
        unseen = memory.observe(visible, stamp_seconds=4.1,
                                robot_x=0, robot_y=0, yaw=0)
        self.assertFalse(TerrainCapabilityModel().assess(unseen).traversable)

    def test_mode_switch_timeout_is_explicit_and_late_success_ignored(self):
        source = (ROOT / 'lzp_service/automatic_terrain_navigation_node.py').read_text('utf-8')
        node = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef)
                    and n.name == 'AutomaticTerrainNavigationNode')
        methods = [n for n in node.body if isinstance(n, ast.FunctionDef)
                   and n.name in ('_tick', '_consume_future')]
        clock = SimpleNamespace(monotonic=lambda: 100)
        scope = {'time': clock, 'MAP_2D': 'map_2d',
                 'SetTraversalMode': SimpleNamespace(Request=SimpleNamespace)}
        exec('class Harness:\n' + '\n'.join(textwrap.indent(
            ast.get_source_segment(source, n), '    ') for n in methods), scope)
        h = scope['Harness']()
        h._lock = threading.RLock()
        h._controller = AutomaticTerrainController()
        h._navigation_known = h._navigation_active = True
        h._navigation_mode = 'terrain_3d'; h._campus_active = False
        h._observation_at = 100; h._observation_timeout = .75
        h._observation = TerrainObservation()
        h._observation_sequence = 1
        h._pending_future = SimpleNamespace(done=lambda: False)
        h._pending_fixed_identity = None
        h._pending_mode = 'ramp'; h._pending_at = 1
        h._switch_timeout = 12; h._switch_fault = ''; h._pending_timed_out = False
        h._mode_client = SimpleNamespace(service_is_ready=lambda: True)
        decisions = []; h._publish = decisions.append
        h.get_logger = lambda: SimpleNamespace(info=lambda *a: None, error=lambda *a: None)
        h._tick()
        self.assertEqual(decisions[-1].state, 'switch_failed')
        self.assertFalse(decisions[-1].safe)
        h._pending_future = SimpleNamespace(done=lambda: True,
            result=lambda: SimpleNamespace(success=True, message='late'))
        h._tick()
        self.assertEqual(decisions[-1].state, 'switch_failed')
        self.assertNotEqual(h._controller.current_mode, 'ramp')

    def test_new_hazard_overrides_recent_floor_and_pose_jump_clears_it(self):
        from lzp_service.rolling_terrain import RollingGroundMemory, matched_pose
        memory = RollingGroundMemory()
        memory.update(floor_cloud(), stamp_seconds=1, robot_x=0, robot_y=0, yaw=0)
        x,y,z = np.meshgrid(np.arange(.50,.80,.03), np.arange(-.15,.16,.03),
                           np.arange(.1,.7,.04))
        hazard = np.column_stack((x.ravel(),y.ravel(),z.ravel()))
        result = memory.observe(np.vstack((floor_cloud(),hazard)),
            stamp_seconds=1.2,robot_x=0,robot_y=0,yaw=0)
        self.assertEqual(result.terrain_type,'blocked')
        memory.update(np.empty((0,3)),stamp_seconds=1.3,robot_x=5,robot_y=0,yaw=0)
        self.assertEqual(memory.cells,{})
        self.assertIsNone(matched_pose([(1,0,0,0)],2))
        pose = matched_pose([(1,0,0,3.1),(1.1,.1,0,-3.1)],1.05)
        self.assertAlmostEqual(pose[0],.05)
        self.assertLess(abs(abs(pose[2])-math.pi),.01)


if __name__ == '__main__':
    unittest.main()
