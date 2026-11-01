#!/usr/bin/env python3
"""Regression tests for split frontend/backend deployment diagnosis."""

from pathlib import Path
import sys


TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))

from diagnose_global_mapping_log import inspect_log  # noqa: E402


def test_split_deployment_log_is_detected():
    signals = inspect_log('\n'.join((
        'Subscribed to /global_mapping_cloud',
        'Localization PCD accumulation enabled',
        'Config file: /home/a/nav_ws/config/MID360s_config.json',
    )))

    assert signals['browser_subscribed']
    assert signals['lio_mapping_enabled']
    assert signals['old_workspace_active']
    assert not signals['global_publisher_started']


def test_current_backend_log_has_publisher_contract():
    signals = inspect_log('\n'.join((
        'Subscribed to /global_mapping_cloud',
        'Localization PCD accumulation enabled',
        'Global mapping cloud preview enabled on /global_mapping_cloud',
        'Config file: /home/a/3dnav/config/MID360s_config.json',
    )))

    assert signals['global_publisher_started']
    assert signals['new_workspace_active']
    assert not signals['old_workspace_active']
