"""Offline fixed-execution policy tests; never send commands to a robot."""
from dataclasses import replace
import json
from types import SimpleNamespace as NS
import unittest

from test_xingji_confidence_gate import supervisor, body_message, velocity_callbacks
from lzp_service.automatic_terrain_navigation import AutomaticTerrainController
from lzp_service.terrain_navigation import TerrainObservation


def flat(**changes):
    return replace(TerrainObservation(terrain_type='flat', confidence=.95,
                                     clearance_width_m=1.), **changes)


class FixedObstacleTests(unittest.TestCase):
    def ready_node(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        node.calls = []
        node._mode_client = NS(service_is_ready=lambda: True,
            call_async=lambda request: node.calls.append(request.mode) or NS(
                done=lambda: True, result=lambda: NS(success=True, message='confirmed')))
        for sequence in range(4):
            node._observation_sequence = sequence + 1
            node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        return node

    def run_observation(self, controller, observation, **changes):
        kwargs = dict(observation_fresh=True, navigation_active=True,
                      campus_active=False, fixed_execution_mode='obstacle')
        kwargs.update(changes)
        return controller.update(observation, **kwargs)

    def test_alternating_traversable_classes_never_switch_or_stop(self):
        controller = AutomaticTerrainController()
        controller.confirm_mode('obstacle', success=True)
        observations = [flat(), flat(terrain_type='curb', step_height_m=.08),
                        flat(terrain_type='ramp', slope_deg=10),
                        flat(terrain_type='rough', roughness_m=.04)] * 20
        for sequence, observation in enumerate(observations):
            decision = self.run_observation(controller, observation,
                                            observation_sequence=sequence)
            self.assertTrue(decision.safe, decision.reason)
            self.assertEqual(decision.current_mode, 'obstacle')
            self.assertEqual(decision.requested_mode, '')
            self.assertEqual(decision.speed_limit_mps, .25)

    def test_initial_mode_must_be_confirmed_and_failure_blocks(self):
        controller = AutomaticTerrainController()
        for sequence in range(3):
            decision = self.run_observation(controller, flat(), observation_sequence=sequence)
            self.assertFalse(decision.safe)
        self.assertEqual(decision.requested_mode, 'obstacle')
        controller.confirm_mode('obstacle', success=False, message='offline failure')
        self.assertFalse(self.run_observation(controller, flat()).safe)

    def test_fixed_execution_does_not_turn_hazards_into_flat(self):
        controller = AutomaticTerrainController()
        controller.confirm_mode('obstacle', success=True)
        hazards = [flat(terrain_type='unknown'), flat(terrain_type='blocked'),
                   flat(negative_obstacle=True), flat(slippery=True),
                   flat(step_height_m=.121), flat(slope_deg=18.1),
                   flat(clearance_width_m=.64), flat(terrain_type='stairs_up')]
        for observation in hazards:
            self.assertFalse(self.run_observation(controller, observation).safe)
        self.assertFalse(self.run_observation(controller, flat(), observation_fresh=False).safe)
        self.assertFalse(self.run_observation(controller, None).safe)

    def test_idle_keeps_fixed_mode_without_hardware_requests(self):
        controller = AutomaticTerrainController()
        controller.confirm_mode('obstacle', success=True)
        decision = self.run_observation(controller, None, navigation_active=False)
        self.assertEqual(decision.current_mode, 'obstacle')
        self.assertEqual(decision.requested_mode, '')

    def test_removed_mode_and_disabled_auto_still_block_special_terrain(self):
        controller = AutomaticTerrainController()
        controller.confirm_mode('obstacle', success=True)
        controller.configure(enabled=True, enabled_modes=['obstacle'])
        self.assertFalse(self.run_observation(controller, flat(terrain_type='ramp', slope_deg=10)).safe)
        controller.configure(enabled=False, enabled_modes=[])
        self.assertFalse(self.run_observation(controller, flat(terrain_type='curb', step_height_m=.08)).safe)
        controller.configure(enabled=True, enabled_modes=['ramp'])
        decision = self.run_observation(controller, flat())
        self.assertFalse(decision.safe)
        self.assertEqual(decision.requested_mode, '')

    def test_node_only_requests_obstacle_then_holds_across_classes(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        calls = []
        node._mode_client = NS(service_is_ready=lambda: True,
            call_async=lambda request: calls.append(request.mode) or NS(
                done=lambda: True, result=lambda: NS(success=True, message='confirmed')))
        for sequence in range(20):
            node._observation_sequence = sequence + 1
            node._observation = flat(terrain_type='curb', step_height_m=.08) if sequence % 2 else flat()
            node._tick()
        self.assertEqual(calls, ['obstacle'])
        self.assertTrue(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['execution_policy'], 'fixed_obstacle')

    def test_policy_is_not_enabled_for_gangben_or_2d(self):
        for body, mode in [('gangben_zsl1', 'terrain_3d'), ('xingji', 'map_2d')]:
            node = supervisor(body=body, mode=mode, confidence=.95, fixed_obstacle=True)
            node._tick()
            self.assertTrue(node.statuses[-1]['safe'])
            self.assertEqual(node.statuses[-1]['current_mode'], 'normal')
            self.assertNotEqual(node.statuses[-1]['execution_policy'], 'fixed_obstacle')

    def test_real_arbiter_preserves_controller_speed_and_still_stops_for_hazards(self):
        node = self.ready_node()
        case = velocity_callbacks.VelocityArbiterCallbacksTest()
        case.setUp()
        case.body('xingji')
        case.node._global_localization_callback(velocity_callbacks.Message(json.dumps(
            dict(ready=True, scope='xingji_3d', body_generation=1,
                 stamp_ns=int(case.now * 1e9)))))
        case.node._terrain_status_callback(velocity_callbacks.Message(json.dumps(node.statuses[-1])))
        case.command()
        case.node._tick()
        self.assertEqual(case.last_velocity(), (.2, .1))
        node._observation = flat(negative_obstacle=True)
        node._tick()
        case.node._terrain_status_callback(velocity_callbacks.Message(json.dumps(node.statuses[-1])))
        case.command()
        case.node._tick()
        self.assertEqual(case.last_velocity(), (0., 0.))

    def test_stale_body_requires_reconfirmation(self):
        node = self.ready_node()
        node.clock.value += 2.01
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertIn('身份', node.statuses[-1]['reason'])
        node._body_callback(body_message())
        node._observation_at = node.clock.value
        node._observation_sequence += 1
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertEqual(node._controller.current_mode, 'unconfirmed')

    def test_generation_change_does_not_reuse_old_confirmation(self):
        node = self.ready_node()
        node._body_callback(body_message(generation=2))
        node._observation_sequence += 1
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertEqual(node._controller.current_mode, 'unconfirmed')

    def test_invalid_body_emergency_or_disconnect_never_authorizes(self):
        for changes in ({'connected': False}, {'switching': True},
                        {'status': {'emergency_latched': True}}, {'generation': True}):
            node = self.ready_node()
            node._body_callback(body_message(**changes))
            node._tick()
            self.assertFalse(node.statuses[-1]['safe'])

    def test_late_success_after_body_or_2d_change_is_ignored(self):
        for new_body, new_mode in [('xingji', 'map_2d'), ('gangben_zsl1', 'terrain_3d')]:
            node = supervisor(confidence=.95, fixed_obstacle=True)
            done = NS(value=False)
            future = NS(done=lambda: done.value,
                        result=lambda: NS(success=True, message='old success'))
            node._mode_client = NS(service_is_ready=lambda: True, call_async=lambda _: future)
            for seq in range(3):
                node._observation_sequence = seq + 1
                node._tick()
            node._body_callback(body_message(new_body))
            node._mode_callback(NS(data=json.dumps({'mode': new_mode})))
            done.value = True
            node._tick()
            self.assertNotEqual(node._controller.current_mode, 'obstacle')
            self.assertTrue(node.statuses[-1]['safe'])

    def test_campus_owner_release_does_not_restore_normal_stage(self):
        node = self.ready_node()
        node._campus_callback(NS(data=True))
        node._tick()
        self.assertEqual(node.statuses[-1]['state'], 'suspended')
        node._campus_callback(NS(data=False))
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        self.assertEqual(node.calls, ['obstacle'])

    def test_switch_failure_stays_stopped_and_idle_sends_nothing(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        node._navigation_active = False
        node._tick()
        self.assertIsNone(node._pending_future)
        node._navigation_active = True
        node._mode_client = NS(service_is_ready=lambda: True, call_async=lambda _: NS(
            done=lambda: True, result=lambda: NS(success=False, message='failure')))
        for seq in range(5):
            node._observation_sequence = seq + 1
            node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['state'], 'switch_failed')

    def test_policy_is_default_and_can_be_disabled_without_affecting_2d(self):
        from test_xingji_confidence_gate import SOURCE
        self.assertIn("declare_parameter('xingji_3d_fixed_obstacle_enabled', True)", SOURCE.read_text('utf-8'))
        node = self.ready_node()
        node.parameters['xingji_3d_fixed_obstacle_enabled'] = False
        node._tick()
        self.assertEqual(node.statuses[-1]['current_mode'], 'normal')
        self.assertTrue(node.statuses[-1]['safe'])


if __name__ == '__main__':
    unittest.main()
