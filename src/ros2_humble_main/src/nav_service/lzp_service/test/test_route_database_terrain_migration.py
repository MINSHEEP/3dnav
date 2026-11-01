#!/usr/bin/env python3
"""Regression tests for additive terrain fields in saved routes."""

from pathlib import Path
import sqlite3
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.route_database import RouteDatabase  # noqa: E402


def test_existing_route_database_is_migrated_without_data_loss(tmp_path):
    database_path = tmp_path / 'routes.db'
    connection = sqlite3.connect(database_path)
    connection.executescript('''
        CREATE TABLE routes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            map_name TEXT NOT NULL,
            name TEXT NOT NULL COLLATE NOCASE,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            UNIQUE(map_name, name));
        CREATE TABLE route_points (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            route_id INTEGER NOT NULL,
            sequence INTEGER NOT NULL,
            name TEXT NOT NULL,
            x REAL NOT NULL,
            y REAL NOT NULL,
            yaw REAL NOT NULL,
            dwell_seconds REAL NOT NULL DEFAULT 0.0,
            FOREIGN KEY(route_id) REFERENCES routes(id) ON DELETE CASCADE,
            UNIQUE(route_id, sequence));
        INSERT INTO routes(id, map_name, name) VALUES (1, 'old', 'legacy');
        INSERT INTO route_points(
            route_id, sequence, name, x, y, yaw, dwell_seconds)
        VALUES (1, 0, 'point', 1.0, 2.0, 0.5, 3.0);
    ''')
    connection.commit()
    connection.close()

    route = RouteDatabase(str(database_path)).get_route(1)

    assert route['points'][0]['x'] == 1.0
    assert route['points'][0]['z'] == 0.0
    assert route['points'][0]['terrain_type'] == 'flat'
    assert route['points'][0]['traversal_mode'] == 'normal'


def test_new_terrain_route_fields_round_trip(tmp_path):
    database = RouteDatabase(str(tmp_path / 'routes.db'))
    route_id = database.save_route({
        'map_name': 'campus', 'name': 'ramp route',
        'points': [{
            'name': 'ramp entry', 'x': 1.0, 'y': 2.0, 'z': 0.3,
            'yaw': 0.2, 'dwell_seconds': 0.0,
            'area_id': 'building_a', 'floor_id': 'F1',
            'terrain_type': 'ramp', 'traversal_mode': 'ramp',
            'transition_id': 'ramp_a', 'speed_limit_mps': 0.1,
        }],
    })

    point = database.get_route(route_id)['points'][0]

    assert point['z'] == 0.3
    assert point['floor_id'] == 'F1'
    assert point['traversal_mode'] == 'ramp'
    assert point['speed_limit_mps'] == 0.1

