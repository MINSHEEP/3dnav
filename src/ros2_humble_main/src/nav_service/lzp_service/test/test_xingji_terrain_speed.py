"""Xingji fixed gait uses Nav2/body caps while hazards still stop motion."""
from dataclasses import replace
import json
from types import SimpleNamespace as NS
import unittest

from test_xingji_fixed_obstacle import flat
from test_xingji_confidence_gate import supervisor, velocity_callbacks
from lzp_service.automatic_terrain_navigation import AutomaticTerrainController
from lzp_service.terrain_navigation import TerrainCapabilityProfile


class XingjiTerrainSpeedTests(unittest.TestCase):
    def controller(self):
        controller = AutomaticTerrainController()
        controller.confirm_mode('obstacle', success=True)
        return controller

    def update(self, controller, observation=None, sequence=1, **changes):
        args = dict(observation_fresh=True, navigation_active=True,
                    campus_active=False, fixed_execution_mode='obstacle',
                    observation_sequence=sequence)
        args.update(changes)
        return controller.update(observation or flat(), **args)

    def settled(self, controller, observation=None, first=1, **changes):
        decisions = [self.update(controller, observation, sequence, **changes)
                     for sequence in range(first, first + 5)]
        return decisions

    def test_real_flat_assessment_speed_is_reached_without_changing_gait(self):
        controller = self.controller()
        decisions = self.settled(controller)
        self.assertTrue(all(item.safe for item in decisions))
        self.assertTrue(all(item.current_mode == 'obstacle' and not item.requested_mode
                            for item in decisions))
        self.assertTrue(all(item.speed_limit_mps == .25 for item in decisions))
        self.assertTrue(all('提速确认' not in item.reason for item in decisions))
        self.assertEqual(decisions[-1].speed_limit_mps, .25)

    def test_traversable_caution_and_special_terrain_add_no_speed_cap(self):
        for observation, expected in (
                (flat(cross_slope_deg=7), .25),
                (flat(terrain_type='rough', roughness_m=.04), .25),
                (flat(terrain_type='ramp', slope_deg=10), .25),
                (flat(terrain_type='curb', step_height_m=.08), .25)):
            with self.subTest(observation=observation):
                decision = self.settled(self.controller(), observation)[-1]
                self.assertTrue(decision.safe)
                self.assertEqual(decision.speed_limit_mps, expected)
                self.assertEqual(decision.current_mode, 'obstacle')

    def test_five_centimeter_boundary_does_not_trigger_speed_changes(self):
        controller = self.controller()
        self.assertEqual(self.settled(controller)[-1].speed_limit_mps, .25)
        heights = [.057, .049, .048, .051, .049, .048, .047, .046, .045, .052, .049]
        for sequence, height in enumerate(heights, 6):
            observation = flat(terrain_type='curb' if height >= .05 else 'flat',
                               step_height_m=height)
            decision = self.update(controller, observation, sequence)
            self.assertTrue(decision.safe)
            self.assertEqual(decision.speed_limit_mps, .25)
            self.assertNotIn('提速确认', decision.reason)

    def test_no_speed_confirmation_is_needed_even_for_repeated_observation(self):
        controller = self.controller()
        for _ in range(100):
            self.assertEqual(self.update(controller, sequence=1).speed_limit_mps, .25)
        self.assertEqual(self.settled(controller, first=2)[-1].speed_limit_mps, .25)

    def test_mode_confirmation_setting_does_not_delay_a_confirmed_gaits_speed_cap(self):
        controller = AutomaticTerrainController(enter_confirmations=2, exit_confirmations=2)
        controller.confirm_mode('obstacle', success=True)
        decisions = self.settled(controller)
        self.assertTrue(all(item.speed_limit_mps == .25 for item in decisions))
        self.assertEqual(decisions[-1].speed_limit_mps, .25)

    def test_hazards_still_stop_and_recovery_does_not_require_speed_confirmation(self):
        for changes in ({'observation_fresh': False}, {'navigation_active': False},
                        {'campus_active': True}):
            controller = self.controller()
            self.settled(controller)
            self.update(controller, sequence=6, **changes)
            self.assertEqual(self.update(controller, sequence=7).speed_limit_mps, .25)
        controller = self.controller()
        self.settled(controller)
        hazard = self.update(controller, flat(negative_obstacle=True), 6)
        self.assertFalse(hazard.safe)
        self.assertEqual(hazard.speed_limit_mps, 0.)
        self.assertEqual(self.update(controller, sequence=7).speed_limit_mps, .25)

    def test_confirmed_body_and_config_do_not_start_a_second_speed_confirmation(self):
        for reset in (
                lambda c: c.configure(enabled=True, enabled_modes=c.enabled_modes),
                lambda c: c.assume_mode('unconfirmed'),
                lambda c: c.confirm_mode('obstacle', success=True)):
            controller = self.controller()
            self.settled(controller)
            reset(controller)
            controller.confirm_mode('obstacle', success=True)
            self.assertEqual(self.update(controller, sequence=6).speed_limit_mps, .25)

    def test_disabled_auto_gait_does_not_enable_special_terrain(self):
        controller = self.controller()
        controller.configure(enabled=False, enabled_modes=[])
        decisions = self.settled(controller)
        self.assertTrue(all(item.speed_limit_mps == .25 for item in decisions))
        self.assertEqual(decisions[-1].speed_limit_mps, .25)
        self.assertFalse(self.update(controller,
            flat(terrain_type='curb', step_height_m=.08), 6).safe)

    def test_lower_configured_profile_is_respected_without_minimum_speed_boost(self):
        controller = self.controller()
        controller.configure(enabled=True, enabled_modes=controller.enabled_modes,
            profile=replace(TerrainCapabilityProfile(), normal_speed_mps=.06))
        self.assertEqual(self.update(controller).speed_limit_mps, .06)
        self.assertEqual(self.update(controller,
            flat(terrain_type='curb', step_height_m=.08)).speed_limit_mps, .06)

    def test_node_and_real_arbiter_pass_flat_speed_and_keep_map_prior_limit(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        node._fixed_body_identity = ('xingji', 1)
        node._controller.confirm_mode('obstacle', success=True)
        for sequence in range(1, 6):
            node._observation_sequence = sequence
            node._tick()
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], .25)
        self.assertEqual(node.statuses[-1]['speed_policy'], 'nav2_body_limits')
        self.assertFalse(node.statuses[-1]['terrain_speed_limit_enabled'])
        self.assertEqual(node.statuses[-1]['speed_limit_source'], 'nav2_body_limits')
        case = velocity_callbacks.VelocityArbiterCallbacksTest()
        case.setUp()
        case.body('xingji')
        case.node._global_localization_callback(velocity_callbacks.Message(json.dumps(dict(
            ready=True, scope='xingji_3d', body_generation=1, stamp_ns=int(case.now*1e9)))))
        case.node._terrain_status_callback(velocity_callbacks.Message(json.dumps(node.statuses[-1])))
        case.command()
        case.node._tick()
        self.assertEqual(case.last_velocity(), (.2, .1))

        node._evidence_source = 'restricted_map'
        for sequence in range(6, 16):
            node._observation_sequence = sequence
            node._tick()
            self.assertLessEqual(node.statuses[-1]['speed_limit_mps'], .10)
            self.assertEqual(node.statuses[-1]['speed_limit_source'], 'restricted_map')
        # Map-prior limits end only when real anchored live evidence returns.
        node._evidence_source = 'current_anchored'
        node._observation_sequence = 16
        node._tick()
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], .25)
        for sequence in range(17, 21):
            node._observation_sequence = sequence
            node._tick()
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], .25)

    def test_2d_and_gangben_keep_existing_ungated_flat_speed(self):
        for body, mode in (('xingji', 'map_2d'), ('gangben_zsl1', 'terrain_3d')):
            node = supervisor(body=body, mode=mode, confidence=.95, fixed_obstacle=True)
            node._tick()
            self.assertEqual(node.statuses[-1]['speed_limit_mps'], .25)
            self.assertEqual(node.statuses[-1]['current_mode'], 'normal')
        node = supervisor(body='gangben_zsl1', mode='terrain_3d',
                          confidence=.95, fixed_obstacle=True)
        node._controller.confirm_mode('obstacle', success=True)
        node._observation = flat(terrain_type='curb', step_height_m=.08)
        node._tick()
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], .10)
        self.assertEqual(node.statuses[-1]['speed_policy'], 'terrain_adaptive')
        self.assertTrue(node.statuses[-1]['terrain_speed_limit_enabled'])
        node = supervisor(mode='map_2d', confidence=.95, fixed_obstacle=True)
        node._observation = flat(negative_obstacle=True)
        node._tick()
        self.assertTrue(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['speed_policy'], 'suspended_2d')

    def test_hazard_status_stops_and_identifies_the_safety_override(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        node._fixed_body_identity = ('xingji', 1)
        node._controller.confirm_mode('obstacle', success=True)
        node._observation = flat(negative_obstacle=True)
        node._tick()
        self.assertFalse(node.statuses[-1]['safe'])
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], 0.)
        self.assertEqual(node.statuses[-1]['speed_limit_source'], 'safety_stop')

    def test_unanchored_observations_retain_their_separate_evidence_cap(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        node._fixed_body_identity = ('xingji', 1)
        node._controller.confirm_mode('obstacle', success=True)
        node._evidence_source = 'current'
        for sequence in range(1, 11):
            node._observation_sequence = sequence
            node._tick()
            self.assertLessEqual(node.statuses[-1]['speed_limit_mps'], .10)
            self.assertEqual(node.statuses[-1]['speed_limit_source'], 'unanchored_observation')
        node._evidence_source = 'recent_measured_anchored'
        for sequence in range(11, 15):
            node._observation_sequence = sequence
            node._tick()
            self.assertEqual(node.statuses[-1]['speed_limit_mps'], .25)
        node._observation_sequence = 15
        node._tick()
        self.assertEqual(node.statuses[-1]['speed_limit_mps'], .25)

    def test_inspection_hold_resume_and_new_loop_do_not_restore_terrain_speed_stages(self):
        node = supervisor(confidence=.95, fixed_obstacle=True)
        node._fixed_body_identity = ('xingji', 1)
        node._controller.confirm_mode('obstacle', success=True)
        mode_requests = []
        node._mode_client = NS(service_is_ready=lambda: True,
            call_async=lambda request: mode_requests.append(request.mode))
        case = velocity_callbacks.VelocityArbiterCallbacksTest()
        case.setUp()
        case.body('xingji')
        case.node._global_localization_callback(velocity_callbacks.Message(json.dumps(dict(
            ready=True, scope='xingji_3d', body_generation=1, stamp_ns=int(case.now*1e9)))))

        # Inspection uses the same route owner while travelling and holding at
        # checkpoints; completing a round briefly makes navigation idle.
        phases = [('travel', True, .057, True),
                  ('checkpoint_hold', True, .049, False),
                  ('resume_after_checkpoint', True, .051, True),
                  ('round_complete', False, .048, False),
                  ('return_to_start', True, .056, True),
                  ('between_rounds', False, .049, False),
                  ('next_round', True, .052, True)]
        for sequence, (phase, active, height, moving) in enumerate(phases, 1):
            with self.subTest(phase=phase):
                node._navigation_active = active
                node._observation_sequence = sequence
                node._observation = flat(
                    terrain_type='curb' if height >= .05 else 'flat',
                    step_height_m=height)
                node._tick()
                status = node.statuses[-1]
                self.assertEqual(status['current_mode'], 'obstacle')
                self.assertEqual(status['requested_mode'], '')
                self.assertNotIn('提速确认', status['reason'])
                if active:
                    self.assertTrue(status['safe'], status['reason'])
                    self.assertEqual(status['speed_limit_mps'], .25)
                    self.assertEqual(status['speed_policy'], 'nav2_body_limits')
                    case.node._terrain_status_callback(
                        velocity_callbacks.Message(json.dumps(status)))
                    command = velocity_callbacks.Twist()
                    command.linear.x = .2 if moving else 0.
                    command.angular.z = .1 if moving else 0.
                    case.node._nav_callback(command)
                    case.node._tick()
                    self.assertEqual(case.last_velocity(),
                                     (.2, .1) if moving else (0., 0.))
        self.assertEqual(mode_requests, [], 'checkpoint or loop must not switch gait')


if __name__ == '__main__':
    unittest.main()
