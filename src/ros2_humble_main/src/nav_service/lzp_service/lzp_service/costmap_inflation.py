"""Persisted obstacle inflation policy and two-costmap sync state.

The module is ROS independent.  The route manager adapts this small interface
to Nav2 parameter services while validation, persistence and stale-result
handling stay deterministic and directly testable.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import uuid


DEFAULT_INFLATION_RADIUS_M = 0.55
MIN_INFLATION_RADIUS_M = 0.30
MAX_INFLATION_RADIUS_M = 1.50
INFLATION_TARGETS = ('global', 'local')


def normalize_inflation_radius(value) -> float:
    """Return a finite radius in metres, stable to one millimetre."""
    if isinstance(value, bool):
        raise ValueError('inflation radius must be a number in metres')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError('inflation radius must be a number in metres') from error
    if not math.isfinite(numeric):
        raise ValueError('inflation radius must be finite')
    if not MIN_INFLATION_RADIUS_M <= numeric <= MAX_INFLATION_RADIUS_M:
        raise ValueError(
            'inflation radius must be between '
            f'{MIN_INFLATION_RADIUS_M:.2f} and '
            f'{MAX_INFLATION_RADIUS_M:.2f} metres')
    return round(numeric, 3)


class InflationRadiusStore:
    """Thread-safe atomic JSON storage for the desired inflation radius."""

    def __init__(self, path, *, default=DEFAULT_INFLATION_RADIUS_M):
        self._path = Path(path).expanduser()
        self._default = normalize_inflation_radius(default)
        self._lock = threading.RLock()
        self._state = self._default_state()
        self._load()

    @property
    def path(self) -> Path:
        return self._path

    def _default_state(self, load_error=''):
        return {
            'radius_m': self._default,
            'version': 0,
            'updated_at': '',
            'updated_by': 'system-default',
            'load_error': str(load_error),
        }

    def _load(self):
        if not self._path.exists():
            return
        try:
            payload = json.loads(self._path.read_text(encoding='utf-8'))
            radius_m = normalize_inflation_radius(payload.get('radius_m'))
            version = max(0, int(payload.get('version', 0)))
            self._state = {
                'radius_m': radius_m,
                'version': version,
                'updated_at': str(payload.get('updated_at', '')),
                'updated_by': str(payload.get('updated_by', 'persisted')),
                'load_error': '',
            }
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            # Keep the existing Nav2 default when a settings file is unsafe.
            self._state = self._default_state(load_error=str(error))

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def set(self, radius_m, *, updated_by='unknown'):
        canonical = normalize_inflation_radius(radius_m)
        actor = str(updated_by or 'unknown').strip() or 'unknown'
        with self._lock:
            if canonical == self._state['radius_m']:
                return dict(self._state)
            state = {
                'radius_m': canonical,
                'version': int(self._state['version']) + 1,
                'updated_at': datetime.now(timezone.utc).isoformat(),
                'updated_by': actor,
                'load_error': '',
            }
            self._write_atomic(state)
            self._state = state
            return dict(self._state)

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


class InflationSyncState:
    """Versioned acknowledgement state for global and local costmaps."""

    def __init__(self, configured):
        self._radius_m = DEFAULT_INFLATION_RADIUS_M
        self._version = 0
        self._targets = {}
        self.configure(configured)

    def configure(self, configured) -> None:
        self._radius_m = normalize_inflation_radius(configured['radius_m'])
        self._version = max(0, int(configured['version']))
        self._targets = {
            target: {'applied': False, 'error': ''}
            for target in INFLATION_TARGETS
        }

    def reset(self, reason='') -> None:
        message = str(reason or '')
        for state in self._targets.values():
            state['applied'] = False
            state['error'] = message

    @property
    def ready(self) -> bool:
        return all(state['applied'] for state in self._targets.values())

    def needs(self, target) -> bool:
        self._validate_target(target)
        return not self._targets[target]['applied']

    def mark_unavailable(self, target, reason) -> None:
        self._validate_target(target)
        self._targets[target]['applied'] = False
        self._targets[target]['error'] = str(reason or 'service unavailable')

    def mark_result(
        self,
        target,
        *,
        radius_m,
        version,
        success,
        reason='',
    ) -> bool:
        self._validate_target(target)
        try:
            result_radius = normalize_inflation_radius(radius_m)
            result_version = int(version)
        except (TypeError, ValueError):
            return False
        if (
            result_radius != self._radius_m
            or result_version != self._version
        ):
            return False
        state = self._targets[target]
        state['applied'] = bool(success)
        state['error'] = '' if success else str(
            reason or 'parameter update failed')
        return True

    def snapshot(self):
        targets = {
            target: dict(state) for target, state in self._targets.items()
        }
        errors = [
            f'{target}: {state["error"]}'
            for target, state in targets.items() if state['error']
        ]
        return {
            'configured_radius_m': self._radius_m,
            'version': self._version,
            'applied': self.ready,
            'pending': not self.ready,
            'targets': targets,
            'error': '; '.join(errors),
        }

    @staticmethod
    def _validate_target(target) -> None:
        if target not in INFLATION_TARGETS:
            raise ValueError(f'unknown inflation target {target!r}')
