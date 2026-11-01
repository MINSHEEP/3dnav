"""Small persisted interface for the two existing mapping filter chains."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import uuid


MODES = frozenset(('protected', 'bypass', 'strict'))


def normalize_mode(value):
    if not isinstance(value, str) or value.strip().lower() not in MODES:
        raise ValueError('建图过滤模式只能为 protected、bypass 或 strict')
    return value.strip().lower()


class MappingFilterStore:
    """Atomic settings; running mapping snapshots this only when it starts."""

    def __init__(self, path, *, default_mode='protected'):
        self.path = Path(path).expanduser()
        self._lock = threading.RLock()
        self._state = self._default(normalize_mode(default_mode))
        try:
            if not self.path.exists():
                return
            if self.path.stat().st_size > 4096:
                raise ValueError('建图过滤配置文件过大')
            payload = json.loads(self.path.read_text(encoding='utf-8'))
            if (not isinstance(payload, dict) or type(payload.get('schema_version')) is not int
                    or payload['schema_version'] != 1):
                raise ValueError('建图过滤配置格式或版本无效')
            mode = normalize_mode(payload.get('mode'))
            version = payload.get('version')
            if type(version) is not int or not 1 <= version <= 2**31 - 1:
                raise ValueError('建图过滤配置版本号无效')
            if type(payload.get('enabled')) is not bool or payload['enabled'] != (mode != 'bypass'):
                raise ValueError('建图过滤开关与模式不一致')
            self._state = dict(schema_version=1, mode=mode, enabled=mode != 'bypass',
                version=version, updated_at=str(payload.get('updated_at', '')),
                updated_by=str(payload.get('updated_by', 'persisted')), load_error='')
        except (OSError, ValueError, TypeError) as error:
            # A damaged file must never silently switch filtering off or be
            # overwritten by a misleading "save succeeded" response.
            self._state = self._default('protected', str(error))

    @staticmethod
    def _default(mode, error=''):
        return dict(schema_version=1, mode=mode, enabled=mode != 'bypass',
                    version=0, updated_at='', updated_by='startup-parameter', load_error=error)

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def set_enabled(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('建图动态障碍过滤开关必须为布尔值')
        with self._lock:
            if self._state['load_error']:
                raise ValueError('配置损坏，请先检查配置文件：' + self._state['load_error'])
            mode = ('strict' if self._state['mode'] == 'strict' else 'protected') if enabled else 'bypass'
            if mode == self._state['mode'] and self._state['version'] > 0:
                return dict(self._state)
            state = dict(self._state, mode=mode, enabled=enabled,
                         version=self._state['version'] + 1,
                         updated_at=datetime.now(timezone.utc).isoformat(), updated_by='settings')
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_name(f'.{self.path.name}.{uuid.uuid4().hex}.tmp')
            try:
                with temporary.open('w', encoding='utf-8', newline='\n') as stream:
                    json.dump(state, stream, ensure_ascii=False, sort_keys=True)
                    stream.write('\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass
            self._state = state
            return dict(state)
