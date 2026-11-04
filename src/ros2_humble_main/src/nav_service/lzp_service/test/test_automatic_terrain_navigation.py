#!/usr/bin/env python3
"""Pure safety tests for automatic same-map terrain traversal."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.automatic_terrain_navigation import (  # noqa: E402
    AutomaticTerrainController,
)
from lzp_service.terrain_navigation import TerrainObservation  # noqa: E402


def ramp():
    return TerrainObservation(
        terrain_type='ramp', slope_deg=10.0,
        clearance_width_m=1.0, confidence=0.95)


def flat():
    return TerrainObservation(
        terrain_type='flat', clearance_width_m=1.0, confidence=0.95)


def controller():
    value = AutomaticTerrainController(
        enter_confirmations=3, exit_confirmations=5)
    value.configure(
        enabled=True,
        enabled_modes={'ramp', 'obstacle', 'rough_ground'})
    return value


def test_special_mode_requires_consecutive_observations_and_confirmation():
    value = controller()

    first = value.update(
        ramp(), observation_fresh=True,
        navigation_active=True, campus_active=False)
    second = value.update(
        ramp(), observation_fresh=True,
        navigation_active=True, campus_active=False)
    third = value.update(
        ramp(), observation_fresh=True,
        navigation_active=True, campus_active=False)

    assert first.safe is False and first.requested_mode == ''
    assert second.safe is False and second.requested_mode == ''
    assert third.safe is False and third.requested_mode == 'ramp'
    value.confirm_mode('ramp', success=True)
    ready = value.update(
        ramp(), observation_fresh=True,
        navigation_active=True, campus_active=False)
    assert ready.safe is True
    assert ready.current_mode == 'ramp'
    assert ready.speed_limit_mps <= 0.10


def test_unknown_stale_and_unapproved_stairs_fail_closed():
    value = controller()
    stale = value.update(
        flat(), observation_fresh=False,
        navigation_active=True, campus_active=False)
    stairs = value.update(
        TerrainObservation(
            terrain_type='stairs_up', step_height_m=0.10,
            clearance_width_m=1.0, confidence=0.95),
        observation_fresh=True,
        navigation_active=True, campus_active=False)
    unknown = value.update(
        TerrainObservation(terrain_type='unknown', confidence=0.0),
        observation_fresh=True,
        navigation_active=True, campus_active=False)

    assert stale.safe is False
    assert stairs.safe is False
    assert unknown.safe is False


def test_editable_connector_profile_cannot_raise_automatic_envelope():
    value = controller()
    high_step = value.update(
        TerrainObservation(
            terrain_type='curb', step_height_m=0.14,
            clearance_width_m=1.0, confidence=0.95),
        observation_fresh=True,
        navigation_active=True, campus_active=False)
    steep_ramp = value.update(
        TerrainObservation(
            terrain_type='ramp', slope_deg=20.0,
            clearance_width_m=1.0, confidence=0.95),
        observation_fresh=True,
        navigation_active=True, campus_active=False)

    assert high_step.safe is False
    assert '台阶高度超过' in high_step.reason
    assert steep_ramp.safe is False
    assert '纵坡超过' in steep_ramp.reason


def test_exit_hysteresis_keeps_special_mode_until_normal_is_confirmed():
    value = controller()
    for _ in range(3):
        decision = value.update(
            ramp(), observation_fresh=True,
            navigation_active=True, campus_active=False)
    assert decision.requested_mode == 'ramp'
    value.confirm_mode('ramp', success=True)

    waiting = [value.update(
        flat(), observation_fresh=True,
        navigation_active=True, campus_active=False) for _ in range(4)]
    switch = value.update(
        flat(), observation_fresh=True,
        navigation_active=True, campus_active=False)

    assert all(item.safe and item.current_mode == 'ramp' for item in waiting)
    assert switch.safe is False
    assert switch.requested_mode == 'normal'


def test_campus_mission_suspends_automatic_mode_ownership():
    value = controller()
    decision = value.update(
        ramp(), observation_fresh=True,
        navigation_active=True, campus_active=True)
    assert decision.safe is True
    assert decision.state == 'suspended'
    assert decision.requested_mode == ''
