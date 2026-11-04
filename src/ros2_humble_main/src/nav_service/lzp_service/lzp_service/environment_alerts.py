"""ROS-independent inspection environment alert policy and persistence."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import uuid


DEFAULT_CO2_THRESHOLD_PPM = 1000.0
MIN_CO2_THRESHOLD_PPM = 400.0
MAX_CO2_THRESHOLD_PPM = 10000.0
ENVIRONMENT_SPEECH = {
    'thermal_high': '环境温度异常',
    'co2_high': '二氧化碳浓度异常',
}


def normalize_co2_threshold(value) -> float:
    """Validate and canonicalize an operator-configured CO2 threshold."""
    if isinstance(value, bool):
        raise ValueError('二氧化碳阈值必须是 ppm 数值')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError('二氧化碳阈值必须是 ppm 数值') from error
    if not math.isfinite(numeric):
        raise ValueError('二氧化碳阈值必须是有限数值')
    if not MIN_CO2_THRESHOLD_PPM <= numeric <= MAX_CO2_THRESHOLD_PPM:
        raise ValueError(
            f'二氧化碳阈值必须在 {MIN_CO2_THRESHOLD_PPM:.0f} 至 '
            f'{MAX_CO2_THRESHOLD_PPM:.0f} ppm 之间')
    return round(numeric, 1)


class EnvironmentAlertConfigStore:
    """Thread-safe atomic storage for robot-wide environment thresholds."""

    def __init__(self, path, *, default=DEFAULT_CO2_THRESHOLD_PPM):
        self._path = Path(path).expanduser()
        self._default = normalize_co2_threshold(default)
        self._lock = threading.RLock()
        self._state = self._default_state()
        self._load()

    def _default_state(self, load_error=''):
        return {
            'co2_threshold_ppm': self._default,
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
            self._state = {
                'co2_threshold_ppm': normalize_co2_threshold(
                    payload.get('co2_threshold_ppm')),
                'version': max(0, int(payload.get('version', 0))),
                'updated_at': str(payload.get('updated_at', '')),
                'updated_by': str(payload.get('updated_by', 'persisted')),
                'load_error': '',
            }
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            self._state = self._default_state(load_error=str(error))

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def set(self, value, *, updated_by='settings-page'):
        canonical = normalize_co2_threshold(value)
        actor = str(updated_by or 'unknown').strip() or 'unknown'
        with self._lock:
            if canonical == self._state['co2_threshold_ppm']:
                return dict(self._state)
            state = {
                'co2_threshold_ppm': canonical,
                'version': int(self._state['version']) + 1,
                'updated_at': datetime.now(timezone.utc).isoformat(),
                'updated_by': actor,
                'load_error': '',
            }
            self._write_atomic(state)
            self._state = state
            return dict(state)

    def _write_atomic(self, state):
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._path.with_name(
            f'.{self._path.name}.{uuid.uuid4().hex}.tmp')
        try:
            with temporary.open('w', encoding='utf-8', newline='\n') as stream:
                json.dump(
                    state, stream, ensure_ascii=False, sort_keys=True,
                    separators=(',', ':'))
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self._path)
        finally:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


class EnvironmentAlertPolicy:
    """Debounce, hysteresis and cooldown for inspection-only alerts."""

    def __init__(
            self, threshold_ppm=DEFAULT_CO2_THRESHOLD_PPM, *,
            confirmations=3, hysteresis_ppm=100.0, cooldown_seconds=60.0):
        self.threshold_ppm = normalize_co2_threshold(threshold_ppm)
        self.confirmations = max(1, int(confirmations))
        self.hysteresis_ppm = max(0.0, float(hysteresis_ppm))
        self.cooldown_seconds = max(1.0, float(cooldown_seconds))
        self.reset()

    def configure(self, threshold_ppm):
        canonical = normalize_co2_threshold(threshold_ppm)
        if canonical != self.threshold_ppm:
            self.threshold_ppm = canonical
            self._reset_co2()

    def reset(self):
        self._reset_co2()
        self._last_thermal_announcement = None

    def _reset_co2(self):
        self.co2_alarm_active = False
        self._co2_high_count = 0
        self._co2_recovery_count = 0
        self._last_co2_announcement = None
        self.latest_co2_ppm = None
        self.co2_sample_valid = False

    def observe_co2(self, value_ppm, *, valid, active, now):
        """Return fixed speech keys due for one validated M701 sample."""
        if not active:
            self._reset_co2()
            return ()
        try:
            value = float(value_ppm)
        except (TypeError, ValueError):
            valid = False
            value = math.nan
        valid = bool(valid) and math.isfinite(value) and value >= 0.0
        self.co2_sample_valid = valid
        self.latest_co2_ppm = value if valid else None
        if not valid:
            self._co2_high_count = 0
            self._co2_recovery_count = 0
            return ()

        current = float(now)
        if not self.co2_alarm_active:
            self._co2_high_count = (
                self._co2_high_count + 1
                if value >= self.threshold_ppm else 0)
            self._co2_recovery_count = 0
            if self._co2_high_count < self.confirmations:
                return ()
            self.co2_alarm_active = True
            self._last_co2_announcement = current
            return ('co2_high',)

        recovery_threshold = max(
            0.0, self.threshold_ppm - self.hysteresis_ppm)
        self._co2_recovery_count = (
            self._co2_recovery_count + 1
            if value <= recovery_threshold else 0)
        if self._co2_recovery_count >= self.confirmations:
            self._reset_co2()
            self.latest_co2_ppm = value
            self.co2_sample_valid = True
            return ()
        if (
            self._last_co2_announcement is None
            or current - self._last_co2_announcement >= self.cooldown_seconds
        ):
            self._last_co2_announcement = current
            return ('co2_high',)
        return ()

    def observe_thermal(self, detection, *, active, now):
        """Return whether one already-confirmed thermal detection is due."""
        if not active:
            self._last_thermal_announcement = None
            return False
        if str(detection.get('measurement_type')) != 'temperature':
            return False
        try:
            maximum = float(detection.get('max_temperature_c'))
            threshold = float(detection.get('threshold_c'))
        except (TypeError, ValueError):
            return False
        if not (
            math.isfinite(maximum) and math.isfinite(threshold)
            and maximum >= threshold
        ):
            return False
        current = float(now)
        if (
            self._last_thermal_announcement is not None
            and current - self._last_thermal_announcement
            < self.cooldown_seconds
        ):
            return False
        self._last_thermal_announcement = current
        return True

    def snapshot(self):
        return {
            'co2_threshold_ppm': self.threshold_ppm,
            'co2_alarm_active': self.co2_alarm_active,
            'current_co2_ppm': self.latest_co2_ppm,
            'co2_sample_valid': self.co2_sample_valid,
        }
