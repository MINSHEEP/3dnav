#!/usr/bin/env python3
"""Safety tests for calibrated special-terrain traversal."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.connector_traversal import (  # noqa: E402
    ConnectorTraversal, DogHealth, OdomSample)


def odom(x=0.0, y=0.0, z=0.0, yaw=0.0, at=0.0):
    return OdomSample(x, y, z, yaw, at)


def healthy(at=0.0):
    return DogHealth(True, True, False, at)


def test_connector_completes_with_distance_and_height():
    policy = ConnectorTraversal(
        distance_m=2.0, speed_mps=0.1, expected_height_delta_m=1.0)
    assert policy.begin(0.0, odom(), healthy()).state == 'running'

    result = policy.update(
        20.0, odom(x=1.85, z=0.8, at=20.0), healthy(20.0))

    assert result.state == 'completed'
    assert result.linear_x == 0.0


def test_stale_health_fails_closed():
    policy = ConnectorTraversal(distance_m=2.0, speed_mps=0.1)
    policy.begin(0.0, odom(), healthy())

    result = policy.update(2.0, odom(x=0.1, at=2.0), healthy(0.0))

    assert result.state == 'failed'
    assert '状态反馈' in result.message


def test_lateral_drift_and_large_yaw_fail_closed():
    lateral = ConnectorTraversal(distance_m=2.0, speed_mps=0.1)
    lateral.begin(0.0, odom(), healthy())
    assert lateral.update(
        1.0, odom(x=0.1, y=0.3, at=1.0), healthy(1.0)).state == 'failed'

    yaw = ConnectorTraversal(distance_m=2.0, speed_mps=0.1)
    yaw.begin(0.0, odom(), healthy())
    assert yaw.update(
        1.0, odom(x=0.1, yaw=0.25, at=1.0), healthy(1.0)).state == 'failed'


def test_stall_fails_closed_and_commands_zero():
    policy = ConnectorTraversal(distance_m=2.0, speed_mps=0.1)
    policy.begin(0.0, odom(), healthy())
    policy.update(1.0, odom(x=0.04, at=1.0), healthy(1.0))

    result = policy.update(5.2, odom(x=0.04, at=5.2), healthy(5.2))

    assert result.state == 'failed'
    assert result.linear_x == 0.0
