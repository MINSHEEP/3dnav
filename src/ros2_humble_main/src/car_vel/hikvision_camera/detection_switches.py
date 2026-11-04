"""Load the stdlib policy in ROS, source-tree and standalone share deployments."""
import importlib.util
from pathlib import Path
import sys

try:
    from lzp_service.detection_settings import DetectionSettings, DetectionSettingsConflict, LABELS, VISUAL, BEHAVIOR
except ModuleNotFoundError as exc:
    if exc.name not in ('lzp_service', 'lzp_service.detection_settings'):
        raise
    candidates = [Path(__file__).with_name('detection_settings.py')]
    candidates += [parent / 'nav_service/lzp_service/lzp_service/detection_settings.py'
                   for parent in Path(__file__).resolve().parents]
    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        raise RuntimeError('缺少检测开关模块，请完整同步并构建 car_vel 与 lzp_service') from exc
    spec = importlib.util.spec_from_file_location('_robot_detection_settings', path)
    core = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = core
    spec.loader.exec_module(core)
    DetectionSettings, DetectionSettingsConflict = core.DetectionSettings, core.DetectionSettingsConflict
    LABELS, VISUAL, BEHAVIOR = core.LABELS, core.VISUAL, core.BEHAVIOR
