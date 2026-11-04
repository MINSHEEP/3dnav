#!/usr/bin/env python3
"""Regression tests for the robot's effective in-place turn speed."""

import pathlib
import sys
import unittest


CAR_VEL_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CAR_VEL_ROOT))

from car_vel.velocity_shaping import TurnCommandShaper, shape_velocity_command


class CmdVelTurnFloorTests(unittest.TestCase):

    def shape(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        return shape_velocity_command(
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            linear_scale=1.5,
            angular_scale=1.0,
            allow_lateral=False,
            deadband=0.02,
            min_in_place_turn_command=0.15,
        )

    def test_low_pure_turn_is_raised_to_the_calibrated_floor(self):
        self.assertEqual(self.shape(angular_z=0.08), (0.0, 0.0, 0.15))
        self.assertEqual(self.shape(angular_z=-0.08), (0.0, 0.0, -0.15))

    def test_zero_and_deadband_turns_remain_stopped(self):
        self.assertEqual(self.shape(angular_z=0.0), (0.0, 0.0, 0.0))
        self.assertEqual(self.shape(angular_z=0.01), (0.0, 0.0, 0.0))

    def test_walking_turn_is_not_raised_to_the_in_place_floor(self):
        vx, vy, vtheta = self.shape(linear_x=0.05, angular_z=0.08)
        self.assertAlmostEqual(vx, 0.075)
        self.assertEqual(vy, 0.0)
        self.assertEqual(vtheta, 0.08)

    def test_turns_already_above_the_floor_are_preserved(self):
        self.assertEqual(self.shape(angular_z=0.20), (0.0, 0.0, 0.20))

    def test_field_turn_floor_ramps_then_stops_without_changing_walking_arcs(self):
        shaper = TurnCommandShaper()
        shaper.set_target(0.0, 0.0, 0.08)
        values = [shaper.step(0.05)[2] for _ in range(20)]
        self.assertAlmostEqual(values[-1], 0.25)
        self.assertTrue(all(abs(b - a) <= 0.020001 for a, b in zip([0.0] + values, values)))
        shaper.set_target(0.0, 0.0, 0.0)
        self.assertEqual(shaper.step(0.05), (0.0, 0.0, 0.0))
        shaper.set_target(0.05, 0.0, 0.08)
        for _ in range(20):
            output = shaper.step(0.05)
        self.assertAlmostEqual(output[2], 0.08)

    def test_bridge_and_launch_use_the_calibrated_floor(self):
        bridge = (CAR_VEL_ROOT / 'car_vel' / 'cmdvel_bridge_ros2.py').read_text(
            encoding='utf-8')
        launch = (CAR_VEL_ROOT / 'launch' / 'car_vel.launch.py').read_text(
            encoding='utf-8')

        self.assertIn('TurnCommandShaper(', bridge)
        self.assertIn("'min_in_place_turn_command'", bridge)
        selected = (CAR_VEL_ROOT / 'car_vel' / 'robot_body_bridge.py').read_text(encoding='utf-8')
        self.assertIn('self.shaper = TurnCommandShaper()', selected)
        self.assertEqual(TurnCommandShaper().turn_floor, 0.25)
        self.assertNotIn("executable='cmdvel_bridge_ros2'", launch)


if __name__ == '__main__':
    unittest.main()
