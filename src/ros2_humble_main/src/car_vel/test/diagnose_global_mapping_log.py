#!/usr/bin/env python3
"""Diagnose a mapping log where the global 3-D layer stays empty."""

import argparse
from pathlib import Path
import sys


def inspect_log(text):
    """Return the deployment and runtime contracts visible in one log."""
    return {
        'browser_subscribed': 'Subscribed to /global_mapping_cloud' in text,
        'lio_mapping_enabled': 'Localization PCD accumulation enabled' in text,
        'global_publisher_started': (
            'Global mapping cloud preview enabled on' in text),
        'terrain_grid_started': (
            'terrain_grid_node-' in text and 'process started' in text),
        'old_workspace_active': '/home/a/nav_ws' in text,
        'new_workspace_active': '/home/a/3dnav' in text,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('log', type=Path)
    args = parser.parse_args(argv)
    signals = inspect_log(args.log.read_text('utf-8', errors='replace'))
    for name, value in signals.items():
        print(f'{name}={str(value).lower()}')

    split_deployment = (
        signals['browser_subscribed']
        and signals['lio_mapping_enabled']
        and not signals['global_publisher_started']
        and signals['old_workspace_active']
    )
    if split_deployment:
        print(
            'FAIL: frontend requests the global cloud, but the running ROS '
            'backend is the old /home/a/nav_ws build.',
            file=sys.stderr,
        )
        return 1
    if not signals['global_publisher_started']:
        print(
            'INCONCLUSIVE: the log has no global-map publisher startup '
            'contract.',
            file=sys.stderr,
        )
        return 2
    print('PASS: the running LIO contains the global-map publisher.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
