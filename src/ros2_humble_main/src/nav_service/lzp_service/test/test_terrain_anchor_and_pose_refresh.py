"""Production seams for floor anchoring and bounded AMCL refresh."""
from pathlib import Path
import runpy
import unittest
from types import SimpleNamespace as NS
import math
import numpy as np

ROOT = Path(__file__).resolve().parent
runtime = runpy.run_path(str(ROOT / 'test_terrain_observer_runtime.py'))


class TerrainAnchorTests(unittest.TestCase):
    def test_only_overhead_returns_do_not_authorize_or_enter_floor_memory(self):
        node = runtime['harness']()
        node._ground_anchor = lambda _message: (0., 0., 0.)
        node._points[:, 2] = 2.5
        node._cloud_callback(NS(header=NS(frame_id='odom', stamp=NS(sec=2, nanosec=0))))
        self.assertFalse(node._memory.cells)
        self.assertTrue(node.errors or node.outputs[0].terrain_type == 'unknown')

    def test_height_anchor_follows_the_physical_body_not_planar_odom_zero(self):
        node = runtime['harness']()
        node._ground_anchor = lambda _message: (0., 0., .6)
        node._points[:, 2] = .6
        node._cloud_callback(NS(header=NS(frame_id='odom', stamp=NS(sec=2, nanosec=0))))
        self.assertEqual(node.outputs[0].terrain_type, 'flat')
        self.assertTrue(node._memory.cells)

    def test_anchor_uses_sensor_height_and_exact_cloud_timestamp(self):
        node=runtime['harness']();node._sensor_height=.52
        calls=[]
        node._tf_buffer=NS(lookup_transform=lambda *args: calls.append(args) or NS(
            transform=NS(translation=NS(x=.23,y=0,z=1.12))))
        message=NS(header=NS(stamp=NS(sec=2,nanosec=123)))
        anchor=type(node)._ground_anchor(node,message)
        self.assertAlmostEqual(anchor[2],.60)
        self.assertEqual(calls[0],('odom','livox_frame',message.header.stamp))

    def test_missing_physical_tf_cannot_fall_back_to_unanchored_ground(self):
        node=runtime['harness']()
        def unavailable(_):
            raise RuntimeError('capture-time TF unavailable')
        node._ground_anchor=unavailable
        node._cloud_callback(NS(header=NS(frame_id='odom',stamp=NS(sec=2,nanosec=0))))
        self.assertTrue(node.errors);self.assertFalse(node.outputs)

    def test_uniform_platform_above_autonomous_step_limit_is_not_flat(self):
        from lzp_service.terrain_observer import estimate_local_terrain
        from lzp_service.automatic_terrain_navigation import AutomaticTerrainController
        node=runtime['harness']();node._points[:,2]=.14
        observation=estimate_local_terrain(node._points,robot_x=0,robot_y=0,yaw=0,
                                           ground_anchor=(0,0,0))
        self.assertNotEqual(observation.terrain_type,'flat')
        decision=AutomaticTerrainController().update(observation,observation_fresh=True,
            navigation_active=True,campus_active=False)
        self.assertFalse(decision.safe)


def prior_node():
    from lzp_service.terrain_grid import build_terrain_grid
    node = runtime['harness']()
    x, y = np.meshgrid(np.arange(-3, 3, .06), np.arange(-3, 3, .06))
    grid = build_terrain_grid(np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))))
    node._grid = NS(**grid.__dict__, map_name='map', map_load_id='uuid',
                    header=NS(frame_id='map', stamp=NS(sec=1, nanosec=0)))
    node._grid_status = {'state': 'ready', 'artifact_verified': True,
        'algorithm': 'clustered_measured_ground_v3', 'map_name': 'map',
        'map_load_id': 'uuid', 'stamp_ns': 1000000000}
    node._map_pose = NS(header=NS(stamp=NS(sec=2, nanosec=0)), pose=NS(
        covariance=[0.]*36, pose=NS(position=NS(x=0, y=0),
        orientation=NS(x=0, y=0, z=0, w=1))))
    node._map_pose_samples = 2; node._restricted_enabled = True
    node._odom_samples.extend([(2.3, .02, 0, 0), (2.6, .04, 0, 0)])
    return node


class LocalizationRefreshTests(unittest.TestCase):
    def test_matching_roof_map_cannot_bypass_physical_height_anchor(self):
        node=prior_node()
        node._grid.elevation[:]=2.5
        node._points[:,2]=2.5
        self.assertIsNone(node._restricted_prior(node._points,2,0,0,0,ground_anchor=(0,0,0)))

    def test_real_odometry_can_propagate_a_0_6s_old_pose_not_forge_its_stamp(self):
        node = prior_node()
        result = node._restricted_prior(node._points, 2.6, .04, 0, 0, ground_anchor=(0,0,0))
        self.assertIsNotNone(result)
        self.assertEqual(node._map_pose.header.stamp.sec, 2)

    def test_pose_older_than_bound_or_without_capture_time_odometry_is_rejected(self):
        node = prior_node()
        self.assertIsNone(node._restricted_prior(node._points, 5, 0, 0, 0, ground_anchor=(0,0,0)))
        node._odom_samples.clear()
        self.assertIsNone(node._restricted_prior(node._points, 2.6, .04, 0, 0, ground_anchor=(0,0,0)))

    def test_pose_refresh_has_one_in_flight_and_is_not_used_in_2d_or_gangben(self):
        node = prior_node()
        node._navigation_active = True; node._refresh_future = None
        node._refresh_at = 0.; node._refresh_started_at = 0.; node._refresh_failures = 0
        node._refresh_backoff_until = 0.
        node._refresh_state = 'idle'
        requests = []
        node._refresh_client = NS(service_is_ready=lambda: True,
            call_async=lambda request: requests.append(request) or NS(done=lambda: False))
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=2400000000))
        for _ in range(5):
            node._pose_refresh_tick()
        self.assertEqual(len(requests), 1)
        for name, mode in [('gangben_zsl1', 'terrain_3d'), ('xingji', 'map_2d')]:
            other = prior_node(); other._body = name; other._navigation_mode = mode
            other._navigation_active = True
            other._pose_refresh_tick()

    def test_no_reply_is_not_retried_and_old_queued_pose_is_not_reused_after_reset(self):
        node=prior_node();node._navigation_active=True
        node._refresh_future=NS(done=lambda:False);node._refresh_started_at=98.
        node._pose_refresh_tick()
        self.assertEqual(node._refresh_state,'timeout')
        node._clear_memory()
        old=NS(header=NS(frame_id='map',stamp=NS(sec=1,nanosec=999)),pose=NS())
        node._map_pose_callback(old)
        self.assertIsNone(node._map_pose)


if __name__ == '__main__':
    unittest.main()
