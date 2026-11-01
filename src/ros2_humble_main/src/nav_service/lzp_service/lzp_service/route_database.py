"""Persistent, map-scoped storage for ordered navigation routes."""

from __future__ import annotations

import os
import sqlite3
import threading
from contextlib import contextmanager
from typing import Dict, List, Optional


class RouteNameConflict(ValueError):
    """Raised when a map already owns a route with the requested name."""


class RouteDatabase:
    """Store named routes and their ordered points in SQLite."""

    def __init__(self, db_path: Optional[str] = None):
        default_path = os.path.join(
            os.path.expanduser('~'), '.ros', 'robot_navigation', 'routes.db')
        self.db_path = os.path.abspath(os.path.expanduser(db_path or default_path))
        self._lock = threading.RLock()
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.db_path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA foreign_keys = ON')
        connection.execute('PRAGMA busy_timeout = 10000')
        return connection

    @contextmanager
    def _connection(self):
        """Yield a transactional connection and always release its file handle."""
        connection = self._connect()
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self._lock, self._connection() as connection:
            connection.execute('PRAGMA journal_mode = WAL')
            connection.executescript(
                '''
                CREATE TABLE IF NOT EXISTS routes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL COLLATE NOCASE,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    UNIQUE(map_name, name)
                );

                CREATE TABLE IF NOT EXISTS route_points (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    route_id INTEGER NOT NULL,
                    sequence INTEGER NOT NULL,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL NOT NULL DEFAULT 0.0,
                    yaw REAL NOT NULL,
                    dwell_seconds REAL NOT NULL DEFAULT 0.0,
                    area_id TEXT NOT NULL DEFAULT '',
                    floor_id TEXT NOT NULL DEFAULT '',
                    terrain_type TEXT NOT NULL DEFAULT 'flat',
                    traversal_mode TEXT NOT NULL DEFAULT 'normal',
                    transition_id TEXT NOT NULL DEFAULT '',
                    speed_limit_mps REAL NOT NULL DEFAULT 0.0,
                    FOREIGN KEY(route_id) REFERENCES routes(id)
                        ON DELETE CASCADE,
                    UNIQUE(route_id, sequence)
                );

                CREATE INDEX IF NOT EXISTS idx_routes_map_name
                    ON routes(map_name);
                CREATE INDEX IF NOT EXISTS idx_route_points_route
                    ON route_points(route_id, sequence);
                ''')
            # Existing installations predate campus/floor/terrain fields.
            # Additive migration preserves every saved route and lets old Web
            # clients continue to provide only x/y/yaw/dwell.
            columns = {
                str(row['name'])
                for row in connection.execute(
                    'PRAGMA table_info(route_points)').fetchall()
            }
            additions = {
                'z': "REAL NOT NULL DEFAULT 0.0",
                'area_id': "TEXT NOT NULL DEFAULT ''",
                'floor_id': "TEXT NOT NULL DEFAULT ''",
                'terrain_type': "TEXT NOT NULL DEFAULT 'flat'",
                'traversal_mode': "TEXT NOT NULL DEFAULT 'normal'",
                'transition_id': "TEXT NOT NULL DEFAULT ''",
                'speed_limit_mps': "REAL NOT NULL DEFAULT 0.0",
            }
            for name, declaration in additions.items():
                if name not in columns:
                    connection.execute(
                        f'ALTER TABLE route_points ADD COLUMN '
                        f'{name} {declaration}')

    @staticmethod
    def _point_from_row(row: sqlite3.Row) -> Dict:
        return {
            'id': int(row['id']),
            'sequence': int(row['sequence']),
            'name': str(row['name']),
            'x': float(row['x']),
            'y': float(row['y']),
            'z': float(row['z']),
            'yaw': float(row['yaw']),
            'dwell_seconds': float(row['dwell_seconds']),
            'area_id': str(row['area_id']),
            'floor_id': str(row['floor_id']),
            'terrain_type': str(row['terrain_type']),
            'traversal_mode': str(row['traversal_mode']),
            'transition_id': str(row['transition_id']),
            'speed_limit_mps': float(row['speed_limit_mps']),
        }

    def _points_for_route(
            self, connection: sqlite3.Connection, route_id: int) -> List[Dict]:
        rows = connection.execute(
            '''
            SELECT id, sequence, name, x, y, z, yaw, dwell_seconds,
                   area_id, floor_id, terrain_type, traversal_mode,
                   transition_id, speed_limit_mps
            FROM route_points
            WHERE route_id = ?
            ORDER BY sequence
            ''',
            (route_id,),
        ).fetchall()
        return [self._point_from_row(row) for row in rows]

    def _route_from_row(
            self, connection: sqlite3.Connection, row: sqlite3.Row) -> Dict:
        route_id = int(row['id'])
        return {
            'id': route_id,
            'map_name': str(row['map_name']),
            'name': str(row['name']),
            'points': self._points_for_route(connection, route_id),
        }

    def get_route(self, route_id: int) -> Optional[Dict]:
        with self._lock, self._connection() as connection:
            row = connection.execute(
                'SELECT id, map_name, name FROM routes WHERE id = ?',
                (int(route_id),),
            ).fetchone()
            return self._route_from_row(connection, row) if row else None

    def list_routes(self, map_name: str) -> List[Dict]:
        with self._lock, self._connection() as connection:
            rows = connection.execute(
                '''
                SELECT id, map_name, name
                FROM routes
                WHERE map_name = ?
                ORDER BY name COLLATE NOCASE, id
                ''',
                (map_name,),
            ).fetchall()
            return [self._route_from_row(connection, row) for row in rows]

    def save_route(self, route: Dict) -> int:
        route_id = int(route.get('id', 0))
        map_name = str(route['map_name'])
        name = str(route['name'])
        points = list(route['points'])

        with self._lock, self._connection() as connection:
            try:
                connection.execute('BEGIN IMMEDIATE')
                if route_id > 0:
                    existing = connection.execute(
                        'SELECT id FROM routes WHERE id = ?',
                        (route_id,),
                    ).fetchone()
                    if existing is None:
                        raise KeyError(f'Route {route_id} does not exist')
                    connection.execute(
                        '''
                        UPDATE routes
                        SET map_name = ?, name = ?,
                            updated_at = CURRENT_TIMESTAMP
                        WHERE id = ?
                        ''',
                        (map_name, name, route_id),
                    )
                    connection.execute(
                        'DELETE FROM route_points WHERE route_id = ?',
                        (route_id,),
                    )
                else:
                    cursor = connection.execute(
                        'INSERT INTO routes(map_name, name) VALUES (?, ?)',
                        (map_name, name),
                    )
                    route_id = int(cursor.lastrowid)

                connection.executemany(
                    '''
                    INSERT INTO route_points(
                        route_id, sequence, name, x, y, z, yaw, dwell_seconds,
                        area_id, floor_id, terrain_type, traversal_mode,
                        transition_id, speed_limit_mps)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ''',
                    [
                        (
                            route_id,
                            index,
                            point['name'],
                            point['x'],
                            point['y'],
                            point.get('z', 0.0),
                            point['yaw'],
                            point['dwell_seconds'],
                            point.get('area_id', ''),
                            point.get('floor_id', ''),
                            point.get('terrain_type', 'flat'),
                            point.get('traversal_mode', 'normal'),
                            point.get('transition_id', ''),
                            point.get('speed_limit_mps', 0.0),
                        )
                        for index, point in enumerate(points)
                    ],
                )
                connection.commit()
                return route_id
            except sqlite3.IntegrityError as error:
                connection.rollback()
                if 'routes.map_name, routes.name' in str(error):
                    raise RouteNameConflict(
                        f'Route name already exists on map {map_name}: {name}'
                    ) from error
                raise
            except Exception:
                connection.rollback()
                raise

    def delete_route(self, route_id: int) -> bool:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                'DELETE FROM routes WHERE id = ?',
                (int(route_id),),
            )
            return cursor.rowcount > 0

    def delete_routes_by_map(self, map_name: str) -> int:
        with self._lock, self._connection() as connection:
            cursor = connection.execute(
                'DELETE FROM routes WHERE map_name = ?',
                (map_name,),
            )
            return int(cursor.rowcount)
