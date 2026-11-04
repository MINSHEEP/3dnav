#!/usr/bin/env python3
"""Unit tests for conservative terrain and campus planning policy."""

from pathlib import Path
import math
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.terrain_navigation import (  # noqa: E402
    CampusArea,
    CampusTopology,
    CampusTransition,
    TerrainCapabilityModel,
    TerrainObservation,
    TransitionPose,
)


def test_flat_ground_is_normal_without_hardware_adapter():
    result = TerrainCapabilityModel().assess(TerrainObservation())

    assert result.decision == 'normal'
    assert result.required_mode == 'normal'
    assert result.speed_limit_mps == 0.25


def test_stairs_are_geometrically_valid_but_require_confirmed_adapter():
    observation = TerrainObservation(
        terrain_type='stairs_up', step_height_m=0.15,
        clearance_width_m=1.0, confidence=0.95)

    missing = TerrainCapabilityModel().assess(observation)
    available = TerrainCapabilityModel().assess(
        observation, {'stairs_up'})

    assert missing.decision == 'special_required'
    assert missing.required_mode == 'stairs_up'
    assert '缺少' in missing.reason
    assert available.decision == 'special_required'
    assert available.speed_limit_mps == 0.10


def test_seventeen_centimetre_step_is_not_an_automatic_product_limit():
    observation = TerrainObservation(
        terrain_type='stairs_up', step_height_m=0.17,
        clearance_width_m=1.0, confidence=0.95)

    result = TerrainCapabilityModel().assess(
        observation, {'stairs_up'})

    assert result.decision == 'blocked'


def test_negative_obstacle_always_blocks():
    observation = TerrainObservation(
        terrain_type='flat', negative_obstacle=True,
        clearance_width_m=5.0, confidence=1.0)

    result = TerrainCapabilityModel().assess(observation)

    assert result.decision == 'blocked'
    assert result.speed_limit_mps == 0.0


def test_unknown_terrain_blocks_instead_of_becoming_flat_ground():
    result = TerrainCapabilityModel().assess(
        TerrainObservation(terrain_type='unknown'))

    assert result.decision == 'blocked'


def test_planar_executor_rejects_nonzero_z_and_floor_changes():
    blockers = TerrainCapabilityModel().planar_execution_blockers([
        {'name': 'F1', 'z': 0.0, 'floor_id': 'F1'},
        {'name': 'F2', 'z': 3.2, 'floor_id': 'F2'},
    ])

    assert len(blockers) == 1
    assert 'nonzero_z' in blockers[0]
    assert 'floor_change' in blockers[0]


def test_planar_executor_rejects_unknown_or_blocked_terrain_labels():
    model = TerrainCapabilityModel()

    assert model.planar_execution_blockers([
        {'name': 'unknown', 'terrain_type': 'unknown'}])
    assert model.planar_execution_blockers([
        {'name': 'cliff', 'terrain_type': 'cliff'}])


def test_topology_avoids_unavailable_stairs_and_uses_ramp():
    areas = [
        CampusArea('outside', 'campus'),
        CampusArea('f1', 'building_f1', 'F1'),
        CampusArea('f2', 'building_f2', 'F2'),
    ]
    transitions = [
        CampusTransition(
            'stairs', 'outside', 'f2',
            TerrainObservation(
                terrain_type='stairs_up', step_height_m=0.15,
                clearance_width_m=1.0, confidence=0.95),
            distance_m=5.0),
        CampusTransition(
            'entrance', 'outside', 'f1', TerrainObservation(),
            distance_m=4.0),
        CampusTransition(
            'ramp', 'f1', 'f2',
            TerrainObservation(
                terrain_type='ramp', slope_deg=8.0,
                clearance_width_m=1.0, confidence=0.95),
            distance_m=6.0),
    ]
    topology = CampusTopology(areas, transitions)

    plan = topology.plan('outside', 'f2', available_modes={'ramp'})

    assert [
        segment.transition.transition_id for segment in plan.segments
    ] == ['entrance', 'ramp']


def test_planning_route_with_unavailable_hardware_can_be_inspected_only():
    topology = CampusTopology(
        [CampusArea('f1', 'f1'), CampusArea('f2', 'f2')],
        [CampusTransition(
            'stairs', 'f1', 'f2',
            TerrainObservation(
                terrain_type='stairs_up', step_height_m=0.15,
                clearance_width_m=1.0, confidence=0.95))],
    )

    plan = topology.plan(
        'f1', 'f2', allow_unavailable_modes=True)

    assert plan.requires_hardware_adapter is True
    assert plan.segments[0].assessment.required_mode == 'stairs_up'


def test_available_special_mode_is_not_reported_as_missing_adapter():
    topology = CampusTopology(
        [CampusArea('f1', 'f1'), CampusArea('f2', 'f2')],
        [CampusTransition(
            'stairs', 'f1', 'f2',
            TerrainObservation(
                terrain_type='stairs_up', step_height_m=0.15,
                clearance_width_m=1.0, confidence=0.95))],
    )

    plan = topology.plan('f1', 'f2', available_modes={'stairs_up'})

    assert plan.requires_hardware_adapter is False
    assert plan.missing_modes == ()


def test_reverse_transition_swaps_calibrated_endpoint_poses():
    topology = CampusTopology(
        [CampusArea('f1', 'f1'), CampusArea('f2', 'f2')],
        [CampusTransition(
            'stairs', 'f1', 'f2', TerrainObservation(),
            source_pose=TransitionPose(1.0, 2.0, yaw=0.5),
            target_pose=TransitionPose(8.0, 9.0, z=3.2, yaw=-0.5),
            calibrated=True)],
    )

    reverse = topology.plan('f2', 'f1').segments[0].transition

    assert reverse.executable is True
    assert reverse.source_pose.x == 8.0
    assert reverse.target_pose.x == 1.0
    assert math.isclose(reverse.source_pose.yaw, math.pi - 0.5)
    assert math.isclose(reverse.target_pose.yaw, -math.pi + 0.5)


def test_transition_without_endpoint_poses_is_preview_only():
    transition = CampusTransition(
        'stairs', 'f1', 'f2', TerrainObservation())

    assert transition.executable is False
