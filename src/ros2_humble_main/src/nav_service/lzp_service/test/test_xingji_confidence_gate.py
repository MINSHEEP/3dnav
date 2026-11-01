"""Scoped confidence policy through real supervision and velocity callbacks.

ROS transport, clocks and SDK I/O are replaced; production methods are run.
"""
import ast
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / 'lzp_service/automatic_terrain_navigation_node.py'
CAR = PACKAGE.parents[1] / 'car_vel'
sys.path[:0] = [str(PACKAGE), str(CAR / 'test')]
from lzp_service.automatic_terrain_navigation import AutomaticTerrainController
from lzp_service.navigation_mode import MAP_2D, TERRAIN_3D, normalize_navigation_mode
from lzp_service.terrain_navigation import TerrainCapabilityModel, TerrainCapabilityProfile, TerrainObservation
import test_velocity_arbiter_callbacks as velocity_callbacks


class FakeNode:
    def __init__(self, _name):
        self.parameters = {}
        self.subscriptions = {}
        self.logger = Mock()

    def declare_parameter(self, name, value):
        self.parameters[name] = value
        return NS(value=value)

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_subscription(self, _type, name, callback, *_args, **_kwargs):
        self.subscriptions[name] = callback

    def create_publisher(self, *_args, **_kwargs):
        return Mock()

    def create_client(self, *_args, **_kwargs):
        return Mock()

    def create_timer(self, *_args, **_kwargs):
        return Mock()

    def get_logger(self):
        return self.logger


def supervisor(*, body='xingji', mode=TERRAIN_3D, confidence=.40, fixed_obstacle=False):
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == 'AutomaticTerrainNavigationNode')
    methods = [n for n in original.body if isinstance(n, ast.FunctionDef)]
    clock = NS(value=100.)
    scope = dict(json=json, threading=threading,
                 time=NS(monotonic=lambda: clock.value),
                 MAP_2D=MAP_2D, TERRAIN_3D=TERRAIN_3D,
                 normalize_navigation_mode=normalize_navigation_mode,
                 TerrainCapabilityProfile=TerrainCapabilityProfile,
                 TerrainObservation=TerrainObservation,
                 AutomaticTerrainController=AutomaticTerrainController,
                 Node=FakeNode, ReentrantCallbackGroup=lambda: NS(),
                 QoSProfile=lambda **kw: NS(**kw), TerrainObservationMessage=NS,
                 HistoryPolicy=NS(KEEP_LAST=1),
                 ReliabilityPolicy=NS(RELIABLE=2, BEST_EFFORT=3),
                 DurabilityPolicy=NS(TRANSIENT_LOCAL=4, VOLATILE=5),
                 Bool=lambda **kw: NS(**kw), Float32=lambda **kw: NS(**kw),
                 String=lambda **kw: NS(**kw),
                 SetTraversalMode=NS(Request=lambda: NS()))
    program = ast.Module(body=[ast.ClassDef(name='Supervisor', bases=[ast.Name(id='Node', ctx=ast.Load())],
        keywords=[], body=methods, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(program), str(SOURCE), 'exec'), scope)
    node = scope['Supervisor']()
    # These legacy tests isolate the independently configurable confidence
    # policy. Fixed execution and its initial confirmation have their own suite.
    node.parameters['xingji_3d_fixed_obstacle_enabled'] = fixed_obstacle
    node.clock = clock
    node._lock = threading.RLock()
    node._controller = AutomaticTerrainController()
    node._observation_timeout = .75
    node._observation = TerrainObservation(terrain_type='flat', confidence=confidence,
                                          clearance_width_m=1.)
    node._observation_at = clock.value
    node._observation_stamp = clock.value
    node._observation_sequence = 1
    node._evidence_source = 'current_anchored'
    node._navigation_active = node._navigation_known = True
    node._campus_active = False
    node._navigation_mode = mode
    node._pending_future = None
    node._pending_mode = ''
    node._pending_at = 0.
    node._pending_timed_out = False
    node._switch_timeout = 12.
    node._switch_fault = node._last_status = ''
    node._body_id = body
    node._body_at = clock.value
    node._body_generation = 1
    node.statuses = []
    node._safe_publisher = NS(publish=lambda _: None)
    node._speed_publisher = NS(publish=lambda _: None)
    node._status_publisher = NS(publish=lambda msg: node.statuses.append(json.loads(msg.data)))
    node._mode_client = NS(service_is_ready=lambda: False)
    node.logger = Mock()
    node.get_logger = lambda: node.logger
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(clock.value*1e9)))
    return node


def body_message(body='xingji', **updates):
    value = dict(active_body=body, connected=True, switching=False,
                 generation=1, error='', status={}, config={
                     'selected': body, 'profiles': {body: {'navigation_verified': True}}})
    value.update(updates)
    return NS(data=json.dumps(value))


class XingjiConfidenceGateTests(unittest.TestCase):
    def test_low_score_xingji_3d_reaches_real_velocity_output(self):
        node = supervisor()
        node._tick()
        case = velocity_callbacks.VelocityArbiterCallbacksTest()
        case.setUp()
        case.body('xingji')
        case.node._terrain_status_callback(velocity_callbacks.Message(json.dumps(node.statuses[-1])))
        case.command()
        case.node._tick()
        self.assertEqual(case.last_velocity(), (.2, .1), node.statuses[-1]['reason'])
        self.assertFalse(node.statuses[-1]['confidence_check_enabled'])
        self.assertEqual(node.statuses[-1]['confidence_policy'], 'bypassed_xingji_3d')

    def test_gangben_and_unconfirmed_body_keep_original_confidence_gate(self):
        for body in ('gangben_zsl1', ''):
            with self.subTest(body=body):
                node = supervisor(body=body)
                node._tick()
                self.assertFalse(node.statuses[-1]['safe'])
                self.assertIn('置信度不足', node.statuses[-1]['reason'])

    def test_stale_body_and_explicit_restore_do_not_bypass(self):
        for restore, age in ((True, 0.), (False, 2.01)):
            with self.subTest(restore=restore, age=age):
                node = supervisor()
                node.parameters['xingji_3d_confidence_check_enabled'] = restore
                node._body_at -= age
                node._tick()
                self.assertFalse(node.statuses[-1]['safe'])
                self.assertIn('置信度不足', node.statuses[-1]['reason'])

    def test_2d_is_still_suspended_without_terrain_gate(self):
        for body in ('xingji', 'gangben_zsl1'):
            node = supervisor(body=body, mode=MAP_2D, confidence=0.)
            node._tick()
            self.assertTrue(node.statuses[-1]['safe'])
            self.assertEqual(node.statuses[-1]['state'], 'suspended_2d')

    def test_missing_stale_unknown_and_hazards_still_block(self):
        changes = (
            {'terrain_type': 'unknown', 'confidence': 0.},
            {'terrain_type': 'blocked'}, {'negative_obstacle': True},
            {'slippery': True}, {'slope_deg': 18.1}, {'cross_slope_deg': 12.1},
            {'step_height_m': .121}, {'step_height_m': -.121},
            {'clearance_width_m': .64}, {'gap_width_m': .11}, {'roughness_m': .061},
        )
        for change in changes:
            with self.subTest(change=change):
                node = supervisor()
                node._observation = replace(node._observation, **change)
                node._tick()
                self.assertFalse(node.statuses[-1]['safe'])
        for observation in (None, 'stale'):
            node = supervisor()
            if observation is None:
                node._observation = None
            else:
                node._observation_at -= .76
            node._tick()
            self.assertFalse(node.statuses[-1]['safe'])

    def test_config_refresh_cannot_reenable_threshold_or_raise_geometry_limits(self):
        node = supervisor()
        node._config_callback(NS(data=json.dumps({'capability_profile': {
            'min_confidence': .99, 'max_up_step_m': .50},
            'automatic_navigation': {'enabled': True,
                'enabled_modes': ['rough_ground', 'obstacle', 'ramp']}})))
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        self.assertEqual(node._controller.capability.profile.min_confidence, .99)
        node._observation = replace(node._observation, step_height_m=.13)
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])

    def test_disabled_auto_gait_does_not_restore_xingji_confidence_gate(self):
        node = supervisor()
        node._controller.configure(enabled=False, enabled_modes=[])
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['state'], 'disabled')

    def test_body_callback_requires_matching_fresh_valid_identity(self):
        node = supervisor(body='')
        node._body_callback(body_message())
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        for message in (body_message(active_body='gangben_zsl1'),
                        body_message(connected=False), body_message(switching=True),
                        body_message(status={'stale': True}),
                        body_message(status={'emergency_latched': True}),
                        body_message(status={'hardware_error': True}),
                        body_message(status=[]), body_message(status=None),
                        body_message(generation=True), NS(data='invalid json')):
            with self.subTest(message=message.data):
                node._body_callback(message)
                node._tick()
                self.assertFalse(node.statuses[-1]['safe'])

    def test_missing_anchored_evidence_still_zeroes_actual_velocity(self):
        node = supervisor()
        node._evidence_source = 'current'
        node._tick()
        case = velocity_callbacks.VelocityArbiterCallbacksTest()
        case.setUp()
        case.body('xingji')
        case.node._terrain_status_callback(velocity_callbacks.Message(json.dumps(node.statuses[-1])))
        case.command()
        case.node._tick()
        self.assertEqual(case.last_velocity(), (0., 0.))

    def test_model_default_is_strict_for_all_existing_callers(self):
        observation = TerrainObservation(confidence=.4)
        model = TerrainCapabilityModel()
        self.assertFalse(model.assess(observation).traversable)
        self.assertTrue(model.assess(observation, check_confidence=False).traversable)
        self.assertFalse(model.assess(replace(observation, terrain_type='unknown'),
                                     check_confidence=False).traversable)

    def test_actual_estimator_low_score_flat_passes_but_unknown_still_blocks(self):
        from lzp_service.terrain_observer import estimate_local_terrain
        points = np.asarray([(0.16+.12*i+.002*k, -.42+.12*j+.001*k, 0.)
            for i in range(5) for j in range(0, 8, 2) for k in range(10)])
        node = supervisor()
        node._observation = estimate_local_terrain(points, robot_x=0, robot_y=0,
            yaw=0, ground_anchor=(0, 0, 0))
        self.assertEqual(node._observation.terrain_type, 'flat')
        self.assertTrue(.25 <= node._observation.confidence < .7)
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        node._observation = estimate_local_terrain(points[:2], robot_x=0, robot_y=0, yaw=0)
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertIn('未知', node.statuses[-1]['reason'])

    def test_non_finite_or_wrong_time_frame_observations_are_still_rejected(self):
        for change in ({'confidence': float('nan')}, {'slope_deg': float('inf')},
                       {'frame': 'map'}, {'stamp': 98.}):
            with self.subTest(change=change):
                node = supervisor()
                node._observation_stamp = 99.
                value = asdict(node._observation)
                value.update({k: v for k, v in change.items() if k not in ('frame', 'stamp')})
                value['header'] = NS(frame_id=change.get('frame', 'odom'),
                    stamp=NS(sec=int(change.get('stamp', 100.)), nanosec=0))
                node._observation_callback(NS(**value))
                node._tick()
                self.assertFalse(node.statuses[-1]['safe'])

    def test_parameter_restore_is_honored_on_the_next_tick(self):
        node = supervisor()
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        node.parameters['xingji_3d_confidence_check_enabled'] = True
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertTrue(node.statuses[-1]['confidence_check_enabled'])
        node.parameters['xingji_3d_confidence_check_enabled'] = False
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])

    def test_prior_speed_limit_and_forward_only_motion_remain(self):
        node = supervisor(confidence=.9)
        node._evidence_source = 'restricted_map'
        node._tick()
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], .1)
        for x, y in ((-.1, 0.), (.1, .1)):
            case = velocity_callbacks.VelocityArbiterCallbacksTest()
            case.setUp()
            case.body('xingji')
            case.node._terrain_status_callback(velocity_callbacks.Message(json.dumps(node.statuses[-1])))
            command = velocity_callbacks.Twist()
            command.linear.x, command.linear.y = x, y
            case.node._nav_callback(command)
            case.node._tick()
            self.assertEqual(case.last_velocity(), (0., 0.))

    def test_special_gait_still_needs_new_observations_and_successful_switch(self):
        node = supervisor()
        node._observation = replace(node._observation, terrain_type='ramp', slope_deg=10.)
        future = NS(done=lambda: False)
        node._mode_client = NS(service_is_ready=lambda: True,
                               call_async=lambda _request: future)
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['state'], 'enter_confirming')
        for sequence in (2, 3):
            node._observation_sequence = sequence
            node._tick()
        self.assertEqual(node.statuses[-1]['state'], 'switching')
        self.assertFalse(node.statuses[-1]['safe'])
        future.done = lambda: True
        future.result = lambda: NS(success=True, message='fake-confirmed')
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['current_mode'], 'ramp')

    def test_installation_declares_and_wires_scoped_policy(self):
        node = supervisor(body='')
        self.assertIs(node.parameters['xingji_3d_confidence_check_enabled'], False)
        self.assertIs(node.subscriptions['/robot_body/status'].__self__, node)
        source = SOURCE.read_text(encoding='utf-8')
        launch = (CAR / 'launch/car_vel.launch.py').read_text(encoding='utf-8')
        self.assertIn("'xingji_3d_confidence_check_enabled', False", source)
        self.assertIn("String, '/robot_body/status', self._body_callback", source)
        self.assertIn("'xingji_3d_confidence_check_enabled': False", launch)


if __name__ == '__main__':
    unittest.main()
