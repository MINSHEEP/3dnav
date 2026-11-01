#!/usr/bin/env python3
"""Deployment contracts for continuous, safely arbitrated navigation."""

from pathlib import Path
import re


CAR_VEL_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = CAR_VEL_ROOT.parent
ARBITER = CAR_VEL_ROOT / 'car_vel' / 'velocity_arbiter.py'
BRIDGE = CAR_VEL_ROOT / 'car_vel' / 'robot_body_bridge.py'
CAR_LAUNCH = CAR_VEL_ROOT / 'launch' / 'car_vel.launch.py'
NAV_PARAMS = (
    WORKSPACE_SRC / 'navigation' / 'robot_navigation2' /
    'config' / 'nav2_params.yaml'
)
POINTCLOUD_LAUNCH = (
    WORKSPACE_SRC / 'mapper' / 'pointcloud_to_laserscan' /
    'launch' / 'pointcloud_to_laserscan_launch.py'
)


def _source(path):
    return path.read_text(encoding='utf-8')


def _yaml_number(source, key):
    match = re.search(rf'^\s*{re.escape(key)}:\s*([-+0-9.]+)', source, re.M)
    assert match, f'missing {key}'
    return float(match.group(1))


def test_hardware_velocity_has_one_safety_owner():
    arbiter = _source(ARBITER)
    bridge = _source(BRIDGE)
    launch = _source(CAR_LAUNCH)

    # Nav2 owns /cmd_vel as its smoothed internal output.  Only the arbiter may
    # forward a command to the hardware-facing topic consumed by the bridge.
    assert "'navigation_input_topic', '/cmd_vel'" in arbiter
    assert "'output_topic', '/cmd_vel_safe'" in arbiter
    assert "Twist, '/cmd_vel_safe'" in bridge
    assert re.search(
        r'create_publisher\(\s*Twist, self\.output_topic,', arbiter)
    assert re.search(
        r'Twist, self\.navigation_input_topic, self\._nav_callback', arbiter)
    assert re.search(
        r'create_subscription\(Twist,\s*\'/cmd_vel_safe\',', bridge)
    assert "executable='cmdvel_bridge_ros2'" not in launch
    assert 'self.navigation_input_topic == self.output_topic' in arbiter
    assert "'navigation_input_topic': '/cmd_vel'" in launch
    assert "'output_topic': '/cmd_vel_safe'" in launch


def test_dynamic_obstacle_tuning_prefers_replan_over_immediate_abort():
    params = _source(NAV_PARAMS)

    # Multiple fresh stochastic attempts absorb a one-frame obstacle/TF glitch
    # before FollowPath escalates into clear-costmap recovery.
    assert _yaml_number(params, 'retry_attempt_limit') >= 3
    assert _yaml_number(params, 'batch_size') >= 800
    assert _yaml_number(params, 'wz_std') >= 0.35

    # A moving obstacle must not be artificially retained for three scans.
    persistence = [
        float(value) for value in re.findall(
            r'^\s*observation_persistence:\s*([-+0-9.]+)', params, re.M)
    ]
    assert persistence
    assert max(persistence) <= 0.1

    # Collision checking remains enabled: continuity must never mean driving
    # through an occupied footprint.
    assert re.search(
        r'CostCritic:\s*\n\s*enabled:\s*true.*?consider_footprint:\s*true',
        params,
        re.S,
    )


def test_scan_projection_drops_backlog_instead_of_replaying_old_obstacles():
    launch = _source(POINTCLOUD_LAUNCH)
    assert "'queue_size': 1" in launch
    assert "'use_inf': True" in launch
