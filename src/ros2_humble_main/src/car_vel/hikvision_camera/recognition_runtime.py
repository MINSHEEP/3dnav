#!/usr/bin/env python3
"""Camera-bound SFace/HyperLPR adapters and recognition event runtime.

Safety detections stay in ``multimodel_detection``.  This module enriches only
face and plate detections, forces ``alert`` to false for them, and writes a
deduplicated informational event after the same target is stable for two
frames in one inspection task.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import re
import threading
import uuid
from typing import Any, Callable, Iterable, Sequence

from recognition_core import RecognitionDatabase, TaskTrackDeduplicator


ROOT = Path(__file__).resolve().parent
DEFAULT_SFACE_MODEL = ROOT / 'models' / 'face_recognition_sface_2021dec.onnx'
DEFAULT_EVIDENCE_DIR = (
    Path.home() / '.ros' / 'robot_navigation' / 'recognition_evidence')


def _crop(frame: Any, bbox: Sequence[Any]) -> Any:
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = [int(round(float(item))) for item in bbox[:4]]
    x1, x2 = sorted((max(0, x1), min(width, x2)))
    y1, y2 = sorted((max(0, y1), min(height, y2)))
    if x2 <= x1 or y2 <= y1:
        raise ValueError('识别目标框为空')
    return frame[y1:y2, x1:x2]


class SFaceFeatureExtractor:
    """Extract an OpenCV SFace embedding from a detected face crop."""

    def __init__(self, model_path: str | Path = DEFAULT_SFACE_MODEL) -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        self.available = False
        self.error = ''
        self._recognizer: Any | None = None
        try:
            import cv2
            if not self.model_path.is_file():
                raise FileNotFoundError(f'SFace模型不存在: {self.model_path}')
            if not hasattr(cv2, 'FaceRecognizerSF'):
                raise RuntimeError('当前OpenCV不支持FaceRecognizerSF')
            self._recognizer = cv2.FaceRecognizerSF.create(
                str(self.model_path), '')
            self.available = True
        except (ImportError, OSError, RuntimeError, FileNotFoundError) as error:
            self.error = str(error)

    def extract(
            self, frame: Any, bbox: Sequence[Any],
            landmarks: Sequence[Sequence[Any]] | None = None) -> list[float]:
        if not self.available or self._recognizer is None:
            raise RuntimeError(self.error or 'SFace尚未就绪')
        import cv2
        import numpy as np

        face = None
        if landmarks and len(landmarks) >= 5:
            x1, y1, x2, y2 = [float(item) for item in bbox[:4]]
            detector_row = [x1, y1, x2 - x1, y2 - y1]
            for point in landmarks[:5]:
                detector_row.extend((float(point[0]), float(point[1])))
            detector_row.append(1.0)
            face = self._recognizer.alignCrop(
                frame, np.asarray(detector_row, dtype=np.float32))
        if face is None:
            face = cv2.resize(_crop(frame, bbox), (112, 112))
        feature = self._recognizer.feature(face)
        if feature is None:
            raise RuntimeError('SFace未返回有效特征')
        return [float(item) for item in feature.reshape(-1).tolist()]


class HyperLPRPlateRecognizer:
    """Read a detected plate crop with HyperLPR3."""

    def __init__(self) -> None:
        self.available = False
        self.error = ''
        self._catcher: Any | None = None
        try:
            import hyperlpr3 as lpr3
            self._catcher = lpr3.LicensePlateCatcher()
            self.available = True
        except (ImportError, AttributeError, RuntimeError) as error:
            self.error = str(error)

    @staticmethod
    def _candidate(item: Any) -> tuple[str, float]:
        if isinstance(item, dict):
            text = item.get('code') or item.get('plate') or item.get('text')
            confidence = item.get('confidence', item.get('score', 0.0))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            text, confidence = item[0], item[1]
        else:
            return '', 0.0
        return str(text or ''), float(confidence or 0.0)

    def recognize(self, frame: Any, bbox: Sequence[Any]) -> tuple[str, float]:
        if not self.available or self._catcher is None:
            raise RuntimeError(self.error or 'HyperLPR3尚未就绪')
        results = self._catcher(_crop(frame, bbox)) or []
        candidates = [self._candidate(item) for item in results]
        candidates = [item for item in candidates if item[0]]
        return max(candidates, key=lambda item: item[1]) if candidates else ('', 0.0)


def classify_plate_color(frame: Any, bbox: Sequence[Any]) -> str:
    """Classify the dominant HSV background into the product color contract."""
    import cv2
    import numpy as np

    crop = _crop(frame, bbox)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    pixels = hsv.reshape(-1, 3)
    if not len(pixels):
        return 'unknown'
    hue, saturation, value = pixels[:, 0], pixels[:, 1], pixels[:, 2]
    useful = value >= 55
    if int(np.count_nonzero(useful)) < max(10, len(pixels) // 8):
        return 'unknown'
    scores = {
        'yellow_black': int(np.count_nonzero(
            useful & (hue >= 15) & (hue <= 38) & (saturation >= 70))),
        'blue_white': int(np.count_nonzero(
            useful & (hue >= 90) & (hue <= 140) & (saturation >= 65))),
        'green_black': int(np.count_nonzero(
            useful & (hue >= 35) & (hue < 90) & (saturation >= 55))),
        'white_black': int(np.count_nonzero(
            useful & (saturation <= 60) & (value >= 130))),
    }
    color, score = max(scores.items(), key=lambda item: item[1])
    return color if score >= max(10, len(pixels) // 6) else 'unknown'


class RecognitionRuntime:
    """Enrich live detections and persist task-scoped, stable recognition."""

    def __init__(
            self, database: RecognitionDatabase, *,
            face_extractor: Any | None = None,
            plate_recognizer: Any | None = None,
            plate_color_classifier: Callable[[Any, Sequence[Any]], str] = classify_plate_color,
            evidence_writer: Callable[..., str] | None = None,
            evidence_dir: str | Path = DEFAULT_EVIDENCE_DIR,
            stable_frames: int = 2, retention_seconds: float = 120.0) -> None:
        self.database = database
        self.face_extractor = face_extractor
        self.plate_recognizer = plate_recognizer
        self.plate_color_classifier = plate_color_classifier
        self.evidence_dir = Path(evidence_dir).expanduser().resolve()
        self.evidence_writer = evidence_writer or self._write_evidence
        self.tracker = TaskTrackDeduplicator(
            stable_frames=stable_frames, retention_seconds=retention_seconds)
        self._task_id = ''
        self._task_generation = 0
        self._session_id = uuid.uuid4().hex[:12]
        self._lock = threading.RLock()

    @classmethod
    def create_default(
            cls, database_path: str | Path | None = None, *,
            sface_model: str | Path = DEFAULT_SFACE_MODEL) -> 'RecognitionRuntime':
        return cls(
            RecognitionDatabase(database_path),
            face_extractor=SFaceFeatureExtractor(sface_model),
            plate_recognizer=HyperLPRPlateRecognizer())

    def model_status(self) -> dict[str, Any]:
        def status(adapter: Any | None) -> dict[str, Any]:
            return {
                'available': bool(adapter and getattr(adapter, 'available', True)),
                'error': '' if adapter is None else str(getattr(adapter, 'error', '') or ''),
            }
        return {
            'sface': status(self.face_extractor),
            'hyperlpr3': status(self.plate_recognizer),
        }

    def _switch_task(self, task_id: str) -> None:
        scope = str(task_id or '__preview__')
        if scope != self._task_id:
            self.tracker.start_task(scope)
            self._task_id = scope
            self._task_generation += 1

    @staticmethod
    def _identity_key(result: dict[str, Any]) -> str:
        category = str(result.get('category') or '')
        identifier = str(result.get('identifier') or '')
        name = str(result.get('display_name') or '')
        if identifier:
            return f'{category}:{identifier}'
        if category == 'plate' and name:
            return f'plate:{name}'
        return ''

    def _recognize(self, frame: Any, detection: dict[str, Any]) -> dict[str, Any]:
        category = str(detection.get('category') or '')
        bbox = detection.get('bbox') or []
        if category == 'face':
            if self.face_extractor is None:
                return self.database.identify_face([], quality=0.0)
            feature = self.face_extractor.extract(
                frame, bbox, detection.get('landmarks'))
            return self.database.identify_face(
                feature, quality=float(detection.get('confidence') or 0.0))
        if self.plate_recognizer is None:
            return self.database.identify_plate('', confidence=0.0)
        text, confidence = self.plate_recognizer.recognize(frame, bbox)
        color = self.plate_color_classifier(frame, bbox)
        return self.database.identify_plate(
            text, confidence=confidence, plate_color=color)

    @staticmethod
    def _uncertain(category: str, error: BaseException) -> dict[str, Any]:
        return {
            'category': category, 'status': 'uncertain', 'confidence': 0.0,
            'identifier': '', 'display_name': '', 'plate_color': 'unknown',
            'alert': False, 'recognition_error': str(error),
        }

    def _write_evidence(
            self, frame: Any, bbox: Sequence[Any], result: dict[str, Any],
            task_id: str, track_id: str) -> str:
        import cv2

        day = datetime.now(timezone.utc).strftime('%Y%m%d')
        directory = self.evidence_dir / day
        directory.mkdir(parents=True, exist_ok=True)
        safe_task = re.sub(r'[^A-Za-z0-9_.-]+', '_', task_id)[:80] or 'task'
        safe_track = re.sub(r'[^A-Za-z0-9_.-]+', '_', track_id)[:40] or 'track'
        target = directory / f'{safe_task}_{safe_track}.jpg'
        if not cv2.imwrite(str(target), _crop(frame, bbox)):
            return ''
        return str(target)

    def process(
            self, frame: Any, detections: Iterable[dict[str, Any]], *,
            task_id: str = '', now: float | None = None) -> list[dict[str, Any]]:
        with self._lock:
            self._switch_task(task_id)
            enriched = []
            for raw in detections:
                detection = dict(raw)
                category = str(detection.get('category') or '')
                if category not in {'face', 'plate'}:
                    enriched.append(detection)
                    continue
                detection['alert'] = False
                try:
                    result = self._recognize(frame, detection)
                except (ImportError, OSError, RuntimeError, TypeError, ValueError) as error:
                    result = self._uncertain(category, error)
                detection.update(result)
                track_id = self.tracker.observe(
                    category, detection.get('bbox') or [],
                    identity_key=self._identity_key(result), now=now)
                if track_id and task_id:
                    database_track_id = (
                        f'{self._session_id}:{self._task_generation}:{track_id}')
                    evidence_path = ''
                    try:
                        evidence_path = self.evidence_writer(
                            frame, detection.get('bbox') or [], result,
                            str(task_id), database_track_id)
                    except (ImportError, OSError, RuntimeError, TypeError, ValueError):
                        evidence_path = ''
                    detection['recognition_event_id'] = self.database.record_event(
                        str(task_id), database_track_id, result,
                        evidence_path=evidence_path)
                enriched.append(detection)
            return enriched

    @staticmethod
    def annotate(frame: Any, detections: Iterable[dict[str, Any]]) -> int:
        """Overlay an ASCII-safe match label under live face/plate boxes."""
        import cv2

        count = 0
        for item in detections:
            category = str(item.get('category') or '')
            if category not in {'face', 'plate'} or 'status' not in item:
                continue
            bbox = item.get('bbox') or []
            if len(bbox) < 4:
                continue
            x1, _y1, _x2, y2 = [int(float(value)) for value in bbox[:4]]
            status = str(item.get('status') or 'uncertain')
            identity = str(item.get('display_name') or item.get('identifier') or '')
            if category == 'face' and any(ord(char) > 127 for char in identity):
                identity = str(item.get('identifier') or '')
            label = f'{category} {status}' + (f' {identity}' if identity else '')
            color = {
                'registered': (40, 190, 90),
                'unregistered': (0, 170, 255),
                'uncertain': (150, 150, 150),
            }.get(status, (150, 150, 150))
            cv2.putText(
                frame, label, (max(0, x1), max(14, y2 + 16)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
            count += 1
        return count
