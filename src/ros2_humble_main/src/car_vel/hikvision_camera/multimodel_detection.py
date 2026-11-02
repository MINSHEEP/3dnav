#!/usr/bin/env python3
"""Priority-aware smoke/fire, face and plate detection.

The module exposes one small interface to the camera child process:
``analyze(frame, mode)`` returns normalized detections and
``annotate(frame, analysis)`` renders them.  Model-specific scheduling,
coordinate conversion and display caching stay behind that interface.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import cv2
import numpy as np


ALARM_CATEGORIES = ("fire", "smoke")
CATEGORY_COLORS = {
    "fire": (0, 0, 230),
    "smoke": (160, 160, 160),
    "face": (255, 255, 0),
    "plate": (0, 200, 255),
}


def normalize_primary_category(label: str) -> str:
    """Normalize training-label variants to the product categories."""
    key = str(label).strip().lower()
    if "fire" in key:
        return "fire"
    if "smoke" in key:
        return "smoke"
    if "cigarette" in key or "butt" in key:
        return "cigarettes"
    return key or "unknown"


def _plain(value: Any) -> Any:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "item"):
        try:
            return value.item()
        except (ValueError, RuntimeError):
            pass
    return value


def _vector(value: Any) -> list[float]:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return [float(item) for item in value]


def _class_name(names: Any, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    try:
        return str(names[class_id])
    except (IndexError, KeyError, TypeError):
        return str(class_id)


def _clip_bbox(
        bbox: tuple[float, float, float, float], width: int,
        height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    return (
        max(0, min(int(round(x1)), max(width - 1, 0))),
        max(0, min(int(round(y1)), max(height - 1, 0))),
        max(0, min(int(round(x2)), max(width - 1, 0))),
        max(0, min(int(round(y2)), max(height - 1, 0))),
    )


@dataclass(frozen=True)
class Detection:
    """One normalized detection shared by rendering and inspection."""

    category: str
    confidence: float
    bbox: tuple[int, int, int, int]
    source: str
    alert: bool
    landmarks: tuple[tuple[int, int], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        result = {
            "category": self.category,
            "confidence": round(float(self.confidence), 6),
            "bbox": list(self.bbox),
            "source": self.source,
            "alert": self.alert,
        }
        if self.landmarks:
            result["landmarks"] = [list(point) for point in self.landmarks]
        return result


@dataclass(frozen=True)
class FrameAnalysis:
    """Observable result of one multi-model analysis call."""

    detections: tuple[Detection, ...]
    display_detections: tuple[Detection, ...]
    ran_detectors: tuple[str, ...]
    timings_ms: dict[str, float]

    def public_detections(self) -> list[dict[str, Any]]:
        return [item.as_dict() for item in self.detections]

    def alarm_detections(self) -> list[tuple[str, float]]:
        highest: dict[str, float] = {}
        for item in self.detections:
            if item.alert:
                highest[item.category] = max(
                    highest.get(item.category, 0.0), item.confidence)
        return [
            (category, highest[category]) for category in ALARM_CATEGORIES
            if category in highest
        ]


class MultiModelDetector:
    """Run the primary model every turn and auxiliary models by mode budget."""

    def __init__(
            self, primary_model: Any, *, face_detector: Any | None = None,
            plate_model: Any | None = None, primary_conf: float = 0.50,
            primary_iou: float = 0.70, primary_imgsz: int = 640,
            inspection_primary_imgsz: int = 1280, device: str = "cpu",
            face_conf: float = 0.60,
            face_max_width: int = 640, plate_conf: float = 0.25,
            plate_imgsz: int = 640, transit_aux_fps: float = 3.0,
            manual_aux_fps: float = 5.0,
            display_ttl_seconds: float = 0.60, model_factories=None) -> None:
        self.primary_model = primary_model
        self.face_detector = face_detector
        self.plate_model = plate_model
        self._model_factories = model_factories or {}
        self.primary_conf = float(primary_conf)
        self.primary_iou = float(primary_iou)
        self.primary_imgsz = int(primary_imgsz)
        self.inspection_primary_imgsz = int(inspection_primary_imgsz)
        self.device = device
        self.face_conf = float(face_conf)
        self.face_max_width = int(face_max_width)
        self.plate_conf = float(plate_conf)
        self.plate_imgsz = int(plate_imgsz)
        self.transit_aux_fps = float(transit_aux_fps)
        self.manual_aux_fps = float(manual_aux_fps)
        self.display_ttl_seconds = float(display_ttl_seconds)
        self._last_run: dict[str, float | None] = {
            "face": None,
            "plate": None,
        }
        self._display_cache: dict[
            str, tuple[float, tuple[Detection, ...]]] = {}

    def _ensure_models(self, enabled):
        for name, attribute in (('primary', 'primary_model'), ('face', 'face_detector'), ('plate', 'plate_model')):
            needed = (enabled.get('fire', True) or enabled.get('smoke', True)) if name == 'primary' else enabled.get(name, True)
            if needed and getattr(self, attribute) is None and name in self._model_factories:
                setattr(self, attribute, self._model_factories[name]())

    def warmup(self, enabled=None) -> None:
        """Warm YOLO models without affecting auxiliary scheduling."""
        enabled = {} if enabled is None else enabled
        self._ensure_models(enabled)
        primary_frame = np.zeros(
            (self.primary_imgsz, self.primary_imgsz, 3), dtype=np.uint8)
        if enabled.get('fire', True) or enabled.get('smoke', True):
            self.primary_model.predict(
                primary_frame, conf=self.primary_conf, iou=self.primary_iou,
                imgsz=self.primary_imgsz, device=self.device, verbose=False)
        if enabled.get('plate', True) and self.plate_model is not None:
            plate_frame = np.zeros(
                (self.plate_imgsz, self.plate_imgsz, 3), dtype=np.uint8)
            self.plate_model.predict(
                plate_frame, conf=self.plate_conf, imgsz=self.plate_imgsz,
                device=self.device, verbose=False)

    def _auxiliary_due(self, name: str, mode: str, now: float) -> bool:
        if mode == "inspection":
            return True
        rate = (
            self.transit_aux_fps if mode == "transit"
            else self.manual_aux_fps
        )
        if rate <= 0.0:
            return False
        previous = self._last_run[name]
        return previous is None or now - previous >= (1.0 / rate)

    def _primary_detections(
            self, result: Any, width: int,
            height: int, enabled=None) -> tuple[Detection, ...]:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return ()
        detections = []
        for box in boxes:
            class_id = int(_plain(box.cls[0]))
            confidence = float(_plain(box.conf[0]))
            xyxy = _vector(box.xyxy[0])
            if len(xyxy) < 4:
                continue
            category = normalize_primary_category(
                _class_name(result.names, class_id))
            # The shared weight still supplies fire/smoke. Legacy cigarette
            # boxes must not leak into display, speech or inspection evidence.
            if category not in ALARM_CATEGORIES or (enabled is not None and not enabled.get(category, True)):
                continue
            detections.append(Detection(
                category=category,
                confidence=confidence,
                bbox=_clip_bbox(tuple(xyxy[:4]), width, height),
                source="primary_yolo",
                alert=(
                    category in ALARM_CATEGORIES
                    and confidence >= self.primary_conf),
            ))
        return tuple(detections)

    def _face_detections(
            self, frame: np.ndarray) -> tuple[Detection, ...]:
        height, width = frame.shape[:2]
        scale = 1.0
        work = frame
        if self.face_max_width > 0 and width > self.face_max_width:
            scale = self.face_max_width / width
            work = cv2.resize(frame, (
                self.face_max_width,
                max(1, int(round(height * scale))),
            ))
        self.face_detector.setInputSize((work.shape[1], work.shape[0]))
        ok, faces = self.face_detector.detect(work)
        if not ok or faces is None:
            return ()
        detections = []
        for face in faces:
            x, y, box_width, box_height = [float(value) for value in face[:4]]
            confidence = float(face[-1])
            if confidence < self.face_conf:
                continue
            detections.append(Detection(
                category="face",
                confidence=confidence,
                bbox=_clip_bbox((
                    x / scale, y / scale,
                    (x + box_width) / scale,
                    (y + box_height) / scale,
                ), width, height),
                source="yunet_face",
                alert=False,
                landmarks=tuple(
                    (int(round(float(face[offset]) / scale)),
                     int(round(float(face[offset + 1]) / scale)))
                    for offset in range(4, min(14, len(face) - 1), 2)
                ),
            ))
        return tuple(detections)

    def _plate_detections(
            self, frame: np.ndarray) -> tuple[Detection, ...]:
        height, width = frame.shape[:2]
        result = self.plate_model.predict(
            frame, conf=self.plate_conf, imgsz=self.plate_imgsz,
            device=self.device, verbose=False)[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return ()
        detections = []
        for box in boxes:
            confidence = float(_plain(box.conf[0]))
            xyxy = _vector(box.xyxy[0])
            if len(xyxy) < 4:
                continue
            detections.append(Detection(
                category="plate",
                confidence=confidence,
                bbox=_clip_bbox(tuple(xyxy[:4]), width, height),
                source="plate_yolo",
                alert=False,
            ))
        return tuple(detections)

    def analyze(
            self, frame: np.ndarray, mode: str = "manual",
            now: float | None = None, enabled=None) -> FrameAnalysis:
        """Analyze the latest frame without queueing or blocking on old frames."""
        current = time.monotonic() if now is None else float(now)
        enabled = {} if enabled is None else enabled
        self._ensure_models(enabled)
        for name in ('face', 'plate'):
            if not enabled.get(name, True):
                self._display_cache.pop(name, None)
                self._last_run[name] = None
        height, width = frame.shape[:2]
        timings: dict[str, float] = {}
        ran = []

        started = time.perf_counter()
        primary_imgsz = (
            self.inspection_primary_imgsz
            if mode == "inspection" else self.primary_imgsz
        )
        primary = ()
        if enabled.get('fire', True) or enabled.get('smoke', True):
            primary_result = self.primary_model.predict(
                frame, conf=self.primary_conf, iou=self.primary_iou,
                imgsz=primary_imgsz, device=self.device, verbose=False)[0]
            timings["primary"] = (time.perf_counter() - started) * 1000.0
            primary = self._primary_detections(primary_result, width, height, enabled)
            ran.append('primary')
        fresh = list(primary)

        if (
            self.face_detector is not None
            and enabled.get('face', True)
            and self._auxiliary_due("face", mode, current)
        ):
            started = time.perf_counter()
            faces = self._face_detections(frame)
            timings["face"] = (time.perf_counter() - started) * 1000.0
            self._last_run["face"] = current
            self._display_cache["face"] = (current, faces)
            fresh.extend(faces)
            ran.append("face")

        if (
            self.plate_model is not None
            and enabled.get('plate', True)
            and self._auxiliary_due("plate", mode, current)
        ):
            started = time.perf_counter()
            plates = self._plate_detections(frame)
            timings["plate"] = (time.perf_counter() - started) * 1000.0
            self._last_run["plate"] = current
            self._display_cache["plate"] = (current, plates)
            fresh.extend(plates)
            ran.append("plate")

        display = list(primary)
        for name in ("face", "plate"):
            cached = self._display_cache.get(name)
            if cached is not None and current - cached[0] <= self.display_ttl_seconds:
                display.extend(cached[1])

        return FrameAnalysis(
            detections=tuple(fresh),
            display_detections=tuple(display),
            ran_detectors=tuple(ran),
            timings_ms=timings,
        )

    @staticmethod
    def annotate(frame: np.ndarray, analysis: FrameAnalysis) -> int:
        """Draw normalized detections in place and return visible count."""
        for detection in analysis.display_detections:
            x1, y1, x2, y2 = detection.bbox
            color = CATEGORY_COLORS.get(detection.category, (255, 128, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{detection.category} {detection.confidence:.2f}"
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_y = max(0, y1 - text_height - baseline - 4)
            cv2.rectangle(
                frame, (x1, text_y),
                (x1 + text_width + 4,
                 text_y + text_height + baseline + 4), color, -1)
            cv2.putText(
                frame, label, (x1 + 2, text_y + text_height + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
                cv2.LINE_AA)
        return len(analysis.display_detections)
