"""One persisted detection policy shared by camera and ROS consumers.

Readers refresh on atomic file replacement. Only the camera settings API writes;
partial changes use a revision to avoid overwriting another page's preferences.
No ROS, camera, model or hardware imports belong in this module.
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import tempfile
import threading


LABELS = {
    'fire': '火焰检测', 'smoke': '烟雾检测', 'face': '人脸识别', 'plate': '车牌识别',
    'fall': '疑似跌倒', 'fight': '疑似打架', 'smoking': '疑似吸烟',
    'thermal': '热成像高温检测', 'm701': 'M701 气体/环境检测', 'co2_alarm': '二氧化碳超限告警',
}
VISUAL = ('fire', 'smoke', 'face', 'plate')
BEHAVIOR = ('fall', 'fight', 'smoking')


class DetectionSettingsConflict(ValueError):
    pass


class DetectionSettings:
    def __init__(self, path=None, *, legacy_behavior_path=None):
        home = Path.home() / '.ros/robot_navigation'
        self.path = Path(path or os.environ.get('ROBOT_DETECTION_SETTINGS') or home / 'detection_settings.json').expanduser()
        self.legacy_behavior_path = Path(legacy_behavior_path or home / 'behavior.json').expanduser()
        self._lock = threading.RLock()
        self._signature = object()
        self._state = None

    def _defaults(self):
        enabled = dict.fromkeys(LABELS, True)
        error = ''
        if self.legacy_behavior_path.is_file():
            try:
                legacy = json.loads(self.legacy_behavior_path.read_text(encoding='utf-8'))
                if not isinstance(legacy, dict):
                    raise ValueError('行为开关格式错误')
                for key in BEHAVIOR:
                    value = legacy.get(key + '_enabled', True)
                    if type(value) is not bool:
                        raise ValueError('行为开关必须是布尔值')
                    enabled[key] = value
            except (OSError, ValueError) as exc:
                enabled.update(dict.fromkeys(BEHAVIOR, False))
                error = '旧行为开关读取失败，请在设置页重新保存：' + str(exc)
        return dict(schema=1, version=0, enabled=enabled, error=error)

    def _read_locked(self):
        try:
            stat = self.path.stat()
            signature = (stat.st_mtime_ns, stat.st_size, stat.st_ino)
        except FileNotFoundError:
            # Before migration the behavior file is still the source of defaults.
            self._state, self._signature = self._defaults(), None
            return
        except OSError as exc:
            self._state = dict(schema=1, version=0, enabled=dict.fromkeys(LABELS, False), error=str(exc))
            return
        if self._state is not None and signature == self._signature:
            return
        try:
            value = json.loads(self.path.read_text(encoding='utf-8'))
            if (not isinstance(value, dict) or value.get('schema') != 1
                    or type(value.get('version')) is not int or value['version'] < 1
                    or not isinstance(value.get('enabled'), dict) or set(value['enabled']) != set(LABELS)
                    or any(type(v) is not bool for v in value['enabled'].values())):
                raise ValueError('检测开关配置格式无效')
            self._state = dict(schema=1, version=value['version'], enabled=value['enabled'], error='')
        except (OSError, ValueError) as exc:
            self._state = dict(schema=1, version=0, enabled=dict.fromkeys(LABELS, False),
                               error='检测开关读取失败，已停止检测，请重新保存：' + str(exc))
        self._signature = signature

    def snapshot(self):
        with self._lock:
            self._read_locked()
            state = copy.deepcopy(self._state)
        state['effective'] = dict(state['enabled'])
        state['effective']['co2_alarm'] &= state['enabled']['m701']
        state.update(config_file=str(self.path), labels=dict(LABELS))
        return state

    def enabled(self, key):
        return self.snapshot()['effective'].get(key, False)

    def allows(self, category):
        key = {'high_temperature': 'thermal', 'thermal_high': 'thermal', 'co2_high': 'co2_alarm'}.get(category, category)
        if key in ('cigarette', 'cigarettes', 'cigarette_butt'):
            return False
        return self.enabled(key) if key in LABELS else True

    def filter_detections(self, items):
        state = self.snapshot()['effective']
        output = []
        for item in items or []:
            category = item.get('category')
            key = {'high_temperature': 'thermal', 'co2_high': 'co2_alarm'}.get(category, category)
            if category in ('cigarette', 'cigarettes', 'cigarette_butt') or state.get(key) is False:
                continue
            output.append(item)
        return output

    def update(self, changes, *, expected_version=None):
        if (not isinstance(changes, dict) or not changes or not set(changes) <= set(LABELS)
                or any(type(v) is not bool for v in changes.values())):
            raise ValueError('请提交有效检测名称与布尔开关值')
        with self._lock:
            self._read_locked()
            if expected_version is not None and (type(expected_version) is not int
                                                 or expected_version != self._state['version']):
                raise DetectionSettingsConflict('检测开关已被其他页面修改，请刷新后重试')
            enabled = {**self._state['enabled'], **changes}
            if enabled == self._state['enabled'] and not self._state['error'] and self.path.exists():
                return self.snapshot()
            state = dict(schema=1, version=self._state['version'] + 1, enabled=enabled)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=self.path.parent,
                                                 prefix='.detection-settings-', suffix='.tmp', delete=False) as stream:
                    temporary = Path(stream.name)
                    json.dump(state, stream, ensure_ascii=False, indent=2)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                if temporary is not None and temporary.exists():
                    temporary.unlink()
            self._signature = object()
            return self.snapshot()
