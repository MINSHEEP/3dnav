#!/usr/bin/env python3
"""Pure tests for exclusive velocity ownership."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from car_vel.velocity_ownership import (  # noqa: E402
    constrain_navigation_velocity,
    select_velocity_source,
    should_stop_for_terrain_update,
)


def test_nav_owns_velocity_outside_connector():
    assert select_velocity_source(
        connector_active=False, emergency=False,
        nav_fresh=True, campus_fresh=True) == 'nav'


def test_fresh_teleop_owns_velocity_without_bypassing_connector_or_emergency():
    assert select_velocity_source(
        connector_active=False, emergency=False,
        teleop_fresh=True, nav_fresh=True, campus_fresh=True) == 'teleop'
    assert select_velocity_source(
        connector_active=True, emergency=False,
        teleop_fresh=True, nav_fresh=True, campus_fresh=True) == 'campus'
    assert select_velocity_source(
        connector_active=True, emergency=False,
        teleop_fresh=True, nav_fresh=True, campus_fresh=False) == 'zero'
    assert select_velocity_source(
        connector_active=False, emergency=True,
        teleop_fresh=True, nav_fresh=True, campus_fresh=True) == 'zero'


def test_campus_owns_velocity_only_inside_connector():
    assert select_velocity_source(
        connector_active=True, emergency=False,
        nav_fresh=True, campus_fresh=True) == 'campus'


def test_stale_owner_and_emergency_both_command_zero():
    assert select_velocity_source(
        connector_active=True, emergency=False,
        nav_fresh=True, campus_fresh=False) == 'zero'
    assert select_velocity_source(
        connector_active=False, emergency=True,
        nav_fresh=True, campus_fresh=True) == 'zero'


def test_terrain_guard_stops_or_scales_only_navigation_velocity():
    assert constrain_navigation_velocity(
        0.20, 0.0, 0.10, terrain_safe=False,
        speed_limit_mps=0.10) == (0.0, 0.0, 0.0)
    limited = constrain_navigation_velocity(
        0.30, 0.40, 0.20, terrain_safe=True,
        speed_limit_mps=0.10)
    assert limited == (0.06, 0.08, 0.20)
    assert constrain_navigation_velocity(
        float('nan'), 0.0, 0.2, terrain_safe=True,
        speed_limit_mps=0.10) == (0.0, 0.0, 0.0)


def test_2d_mode_bypasses_only_the_terrain_guard():
    assert constrain_navigation_velocity(
        0.30, 0.40, 0.20, terrain_safe=False,
        speed_limit_mps=0.10,
        terrain_guard_enabled=False) == (0.30, 0.40, 0.20)
    assert constrain_navigation_velocity(
        float('nan'), 0.0, 0.20, terrain_safe=False,
        speed_limit_mps=0.10,
        terrain_guard_enabled=False) == (0.0, 0.0, 0.0)
    assert not should_stop_for_terrain_update(
        terrain_safe=False, connector_active=False, teleop_fresh=False,
        terrain_guard_enabled=False)


def test_terrain_update_never_interrupts_teleop_or_connector_ownership():
    assert should_stop_for_terrain_update(
        terrain_safe=False, connector_active=False, teleop_fresh=False)
    assert not should_stop_for_terrain_update(
        terrain_safe=False, connector_active=False, teleop_fresh=True)
    assert not should_stop_for_terrain_update(
        terrain_safe=False, connector_active=True, teleop_fresh=False)
    assert not should_stop_for_terrain_update(
        terrain_safe=True, connector_active=False, teleop_fresh=False)
