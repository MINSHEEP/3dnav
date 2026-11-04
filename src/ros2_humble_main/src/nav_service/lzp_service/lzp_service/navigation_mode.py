"""Persisted policy for choosing planar or terrain-enhanced navigation.

This module deliberately contains no ROS code.  Nodes consume its small
semantic interface while storage, validation and safe defaults remain
consistent across route execution and tests.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import uuid


MAP_2D = 'map_2d'
TERRAIN_3D = 'terrain_3d'
DEFAULT_NAVIGATION_MODE = TERRAIN_3D
DEFAULT_NAVIGATION_SPEED = 0.25
NAVIGATION_SPEED_GEARS = (DEFAULT_NAVIGATION_SPEED, 0.45)
# Keep saved settings and older clients compatible without automatically
# raising the speed of a robot already configured for a legacy gear.
_SUPPORTED_NAVIGATION_SPEEDS = (*NAVIGATION_SPEED_GEARS, 0.15, 0.30)

MODE_LABELS = {
    MAP_2D: '2D地图导航',
    TERRAIN_3D: '3D地形增强导航',
}

_ALIASES = {
    MAP_2D: MAP_2D,
    '2d': MAP_2D,
    '2d_map': MAP_2D,
    TERRAIN_3D: TERRAIN_3D,
    '3d': TERRAIN_3D,
    'terrain': TERRAIN_3D,
}


def normalize_navigation_mode(value) -> str:
    """Return a canonical navigation mode or reject ambiguous values."""
    key = str(value or '').strip().lower()
    try:
        return _ALIASES[key]
    except KeyError as error:
        raise ValueError(
            f'unsupported navigation mode {value!r}; expected '
            f'{MAP_2D!r} or {TERRAIN_3D!r}') from error


def uses_terrain(mode) -> bool:
    """Return whether terrain fusion, preflight and speed guards apply."""
    return normalize_navigation_mode(mode) == TERRAIN_3D


def navigation_mode_label(mode) -> str:
    return MODE_LABELS[normalize_navigation_mode(mode)]


def normalize_navigation_speed(value) -> float:
    if type(value) not in (int, float) or value not in _SUPPORTED_NAVIGATION_SPEEDS:
        raise ValueError('请选择 0.25 或 0.45 m/s 导航速度挡位')
    return float(value)


class NavigationModeStore:
    """Thread-safe, atomic JSON storage for the globally selected mode."""

    def __init__(self, path, *, default=DEFAULT_NAVIGATION_MODE):
        self._path = Path(path).expanduser()
        self._default = normalize_navigation_mode(default)
        self._lock = threading.RLock()
        self._state = self._default_state()
        self._load()

    @property
    def path(self) -> Path:
        return self._path

    def _default_state(self, load_error=''):
        return {
            'mode': self._default,
            'version': 0,
            'updated_at': '',
            'updated_by': 'system-default',
            'load_error': str(load_error),
            'speed_mps': DEFAULT_NAVIGATION_SPEED,
        }

    def _load(self):
        if not self._path.exists():
            return
        try:
            payload = json.loads(self._path.read_text(encoding='utf-8'))
            mode = normalize_navigation_mode(payload.get('mode'))
            version = max(0, int(payload.get('version', 0)))
            try:
                speed_mps = normalize_navigation_speed(
                    payload.get('speed_mps', DEFAULT_NAVIGATION_SPEED))
            except ValueError:
                speed_mps = DEFAULT_NAVIGATION_SPEED
            self._state = {
                'mode': mode,
                'version': version,
                'updated_at': str(payload.get('updated_at', '')),
                'updated_by': str(payload.get('updated_by', 'persisted')),
                'load_error': '',
                'speed_mps': speed_mps,
            }
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            # Terrain mode is the conservative fallback: it never silently
            # disables a safety policy because a settings file is damaged.
            self._state = self._default_state(load_error=str(error))

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def set(self, mode, *, updated_by='unknown'):
        canonical = normalize_navigation_mode(mode)
        actor = str(updated_by or 'unknown').strip() or 'unknown'
        with self._lock:
            if canonical == self._state['mode']:
                return dict(self._state)
            state = {
                **self._state,
                'mode': canonical,
                'version': int(self._state['version']) + 1,
                'updated_at': datetime.now(timezone.utc).isoformat(),
                'updated_by': actor,
                'load_error': '',
            }
            self._write_atomic(state)
            self._state = state
            return dict(self._state)

    def set_speed(self, speed_mps):
        speed = normalize_navigation_speed(speed_mps)
        with self._lock:
            if speed != self._state['speed_mps']:
                state = {**self._state, 'speed_mps': speed}
                self._write_atomic(state)
                self._state = state
            return speed

    def _write_atomic(self, state):
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._path.with_name(
            f'.{self._path.name}.{uuid.uuid4().hex}.tmp')
        try:
            with temporary.open('w', encoding='utf-8', newline='\n') as stream:
                json.dump(
                    state, stream, ensure_ascii=False,
                    sort_keys=True, separators=(',', ':'))
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self._path)
        finally:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
