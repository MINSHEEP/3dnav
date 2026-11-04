"""Regression tests for the operator-confirmed YOLO confidence threshold."""

import ast
from pathlib import Path
import sys
import tempfile
from types import ModuleType
from unittest import mock


CAMERA_DIR = Path(__file__).resolve().parents[1] / 'hikvision_camera'
VOICE_PATH = CAMERA_DIR / 'yolo_hk_voice.py'
CAMERA_WEB_PATH = CAMERA_DIR / 'camera_web.py'
YOLO_PATH = CAMERA_DIR / 'yolo_hk.py'


def _import_voice_module():
    fake_yolo_hk = ModuleType('yolo_hk')
    for name in (
            'DeviceInfoV30', 'HikvisionJpegSource', 'HikvisionRtspSource',
            'HikvisionV40Source', 'RtspSource', 'YoloMJPEGServer'):
        setattr(fake_yolo_hk, name, type(name, (), {}))
    for name in (
            'draw_detections', 'draw_status', 'ensure_player_library_path',
            'open_capture_source'):
        setattr(fake_yolo_hk, name, mock.MagicMock())
    fake_ultralytics = ModuleType('ultralytics')
    fake_ultralytics.YOLO = mock.MagicMock()

    sys.path.insert(0, str(CAMERA_DIR))
    try:
        with mock.patch.dict(sys.modules, {
                'cv2': mock.MagicMock(),
                'numpy': mock.MagicMock(),
                'ultralytics': fake_ultralytics,
                'yolo_hk': fake_yolo_hk,
        }):
            sys.modules.pop('yolo_hk_voice', None)
            __import__('yolo_hk_voice')
            return sys.modules['yolo_hk_voice']
    finally:
        sys.path.remove(str(CAMERA_DIR))


class _Scalar:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def item(self):
        return self.value


class _Box:
    def __init__(self, class_id, confidence):
        self.cls = [_Scalar(class_id)]
        self.conf = [_Scalar(confidence)]


class _Boxes(list):
    pass


class _Result:
    names = {0: 'fire', 1: 'smoke'}
    boxes = _Boxes([_Box(0, 0.49), _Box(1, 0.50)])


def _argument_default(path, option):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Attribute):
            continue
        if call.func.attr != 'add_argument' or not call.args:
            continue
        if isinstance(call.args[0], ast.Constant) and call.args[0].value == option:
            default = next(
                keyword.value for keyword in call.keywords
                if keyword.arg == 'default'
            )
            return ast.literal_eval(default)
    raise AssertionError(f'{option} was not found in {path}')


def test_only_detections_at_or_above_fifty_percent_are_confirmed():
    voice = _import_voice_module()
    assert voice.extract_alarm_detections(_Result()) == [('smoke', 0.50)]


def test_all_yolo_entry_points_default_to_fifty_percent():
    assert _argument_default(VOICE_PATH, '--conf') == 0.50
    assert _argument_default(CAMERA_WEB_PATH, '--yolo-conf') == 0.50
    assert _argument_default(YOLO_PATH, '--conf') == 0.50


def test_missing_local_prefix_tts_is_attempted_only_once_per_process():
    voice = _import_voice_module()
    with tempfile.TemporaryDirectory() as directory:
        speaker = voice.LocalComputerSpeaker.__new__(voice.LocalComputerSpeaker)
        speaker.audio_dir = Path(directory)
        speaker.speech_prefix = '巡检检测前缀测试'
        speaker._missing = set()
        attempts = []
        speaker._synthesize_missing_audio = (
            lambda text, path: attempts.append((text, path)) or None)

        assert speaker._audio_path(speaker.speech_prefix) is None
        assert speaker._audio_path(speaker.speech_prefix) is None
        assert len(attempts) == 1


if __name__ == '__main__':
    test_only_detections_at_or_above_fifty_percent_are_confirmed()
    test_all_yolo_entry_points_default_to_fifty_percent()
    test_missing_local_prefix_tts_is_attempted_only_once_per_process()
