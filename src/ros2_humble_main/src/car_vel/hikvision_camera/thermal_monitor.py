"""Shared thermal state, threshold persistence and high-temperature events.

The module intentionally knows nothing about HCUSBSDK or HTTP.  Production feeds
``ThermalSample`` values from the vendor adapter; tests feed the same interface
from memory.  Browser pages and patrol code therefore never own the USB device.
"""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable


DEFAULT_THRESHOLD_C = 60.0
DEFAULT_CLEAR_DELTA_C = 3.0
DEFAULT_HOLD_SECONDS = 0.8
DEFAULT_STALE_SECONDS = 2.0
MIN_THRESHOLD_C = -40.0
MAX_THRESHOLD_C = 550.0


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class ThermalConfigConflict(ValueError):
    """Raised when a caller tries to overwrite a newer threshold revision."""


@dataclass(frozen=True)
class ThermalSample:
    """One SDK frame copied out of the vendor callback's temporary buffer."""

    raw_jpeg: bytes
    annotated_jpeg: bytes
    max_temperature_c: float
    x_normalized: int
    y_normalized: int
    frame_number: int
    captured_at_epoch: float
    width: int
    height: int
    frozen: bool = False
    temperature_source: str = "SDK frame header fMaxTmp"

    def validate(self) -> None:
        if not math.isfinite(float(self.max_temperature_c)):
            raise ValueError("最高温不是有限数")
        if not 0 <= int(self.x_normalized) <= 1000:
            raise ValueError("最高温横坐标必须在 0..1000")
        if not 0 <= int(self.y_normalized) <= 1000:
            raise ValueError("最高温纵坐标必须在 0..1000")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("热图尺寸无效")
        if not self.raw_jpeg.startswith(b"\xff\xd8"):
            raise ValueError("热图原图不是 JPEG")
        if not self.annotated_jpeg.startswith(b"\xff\xd8"):
            raise ValueError("热图标注图不是 JPEG")


class ThermalConfigStore:
    """Persist one global default and optional per-task threshold overrides."""

    def __init__(
        self,
        path: Path,
        *,
        default_threshold_c: float = DEFAULT_THRESHOLD_C,
        minimum_c: float = MIN_THRESHOLD_C,
        maximum_c: float = MAX_THRESHOLD_C,
    ) -> None:
        self.path = Path(path).expanduser().resolve()
        self.minimum_c = float(minimum_c)
        self.maximum_c = float(maximum_c)
        self.default_threshold_c = self._validate_threshold(default_threshold_c)
        self._lock = threading.RLock()
        self._document = self._load()

    def _validate_threshold(self, value: Any) -> float:
        try:
            threshold = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError("高温阈值必须是数字") from error
        if not math.isfinite(threshold):
            raise ValueError("高温阈值必须是有限数")
        if not self.minimum_c <= threshold <= self.maximum_c:
            raise ValueError(
                f"高温阈值必须在 {self.minimum_c:g}..{self.maximum_c:g} ℃")
        return round(threshold, 1)

    def _default_document(self) -> dict[str, Any]:
        now = _utc_now()
        return {
            "schema_version": 1,
            "global": {
                "threshold_c": self.default_threshold_c,
                "version": 1,
                "updated_at": now,
                "updated_by": "system-default",
            },
            "tasks": {},
        }

    def _load(self) -> dict[str, Any]:
        if not self.path.is_file():
            return self._default_document()
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            global_config = dict(payload.get("global") or {})
            global_config["threshold_c"] = self._validate_threshold(
                global_config.get("threshold_c", self.default_threshold_c))
            global_config["version"] = max(1, int(global_config.get("version", 1)))
            global_config.setdefault("updated_at", _utc_now())
            global_config.setdefault("updated_by", "unknown")
            tasks: dict[str, Any] = {}
            for task_id, raw in dict(payload.get("tasks") or {}).items():
                if not task_id or not isinstance(raw, dict):
                    continue
                entry = dict(raw)
                entry["threshold_c"] = self._validate_threshold(entry.get("threshold_c"))
                entry["version"] = max(1, int(entry.get("version", 1)))
                tasks[str(task_id)] = entry
            return {"schema_version": 1, "global": global_config, "tasks": tasks}
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise RuntimeError(f"热成像阈值配置损坏：{self.path}: {error}") from error

    def _save_locked(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + ".tmp")
        temporary.write_text(
            json.dumps(self._document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def get(self, task_id: str = "") -> dict[str, Any]:
        with self._lock:
            global_config = dict(self._document["global"])
            task = dict(self._document["tasks"].get(str(task_id), {})) if task_id else {}
        effective = task or global_config
        return {
            "threshold_c": float(effective["threshold_c"]),
            "version": int(effective["version"]),
            "scope": "task" if task else "global",
            "task_id": str(task_id) if task else "",
            "updated_at": str(effective.get("updated_at") or ""),
            "updated_by": str(effective.get("updated_by") or ""),
            "global_threshold_c": float(global_config["threshold_c"]),
            "global_version": int(global_config["version"]),
            "minimum_c": self.minimum_c,
            "maximum_c": self.maximum_c,
        }

    def update(
        self,
        threshold_c: Any,
        *,
        scope: str = "global",
        task_id: str = "",
        expected_version: int | None = None,
        actor: str = "web-ui",
    ) -> dict[str, Any]:
        threshold = self._validate_threshold(threshold_c)
        scope = str(scope or "global").strip().lower()
        if scope not in {"global", "task"}:
            raise ValueError("阈值作用域必须是 global 或 task")
        task_id = str(task_id or "").strip()
        if scope == "task" and not task_id:
            raise ValueError("任务阈值必须提供 task_id")
        with self._lock:
            container = self._document["global"] if scope == "global" else self._document["tasks"].get(task_id)
            current_version = int((container or {}).get("version", 0))
            if expected_version is not None and int(expected_version) != current_version:
                raise ThermalConfigConflict(
                    f"阈值已被其他页面修改（当前版本 {current_version}）")
            entry = {
                "threshold_c": threshold,
                "version": current_version + 1,
                "updated_at": _utc_now(),
                "updated_by": str(actor or "web-ui")[:80],
            }
            if scope == "global":
                self._document["global"] = entry
            else:
                self._document["tasks"][task_id] = entry
            self._save_locked()
        return self.get(task_id if scope == "task" else "")

    def clear_task(self, task_id: str, *, expected_version: int | None = None) -> dict[str, Any]:
        task_id = str(task_id or "").strip()
        if not task_id:
            raise ValueError("必须提供 task_id")
        with self._lock:
            entry = self._document["tasks"].get(task_id)
            if entry is not None and expected_version is not None:
                if int(expected_version) != int(entry.get("version", 0)):
                    raise ThermalConfigConflict("任务阈值已被其他页面修改")
            self._document["tasks"].pop(task_id, None)
            self._save_locked()
        return self.get(task_id)


class HighTemperatureDetector:
    """Debounce one continuous maximum-temperature alarm with hysteresis."""

    def __init__(
        self,
        *,
        hold_seconds: float = DEFAULT_HOLD_SECONDS,
        clear_delta_c: float = DEFAULT_CLEAR_DELTA_C,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if hold_seconds < 0 or clear_delta_c <= 0:
            raise ValueError("高温持续时间不能为负且解除回差必须大于 0")
        self.hold_seconds = float(hold_seconds)
        self.clear_delta_c = float(clear_delta_c)
        self._clock = clock
        self._pending_since: float | None = None
        self._active = False
        self._event_number = 0
        self._peak_temperature_c: float | None = None

    @property
    def active(self) -> bool:
        return self._active

    @property
    def peak_temperature_c(self) -> float | None:
        return self._peak_temperature_c

    def reset(self) -> None:
        self._pending_since = None
        self._active = False
        self._peak_temperature_c = None

    def process(
        self,
        sample: ThermalSample,
        *,
        threshold_c: float,
        enabled: bool,
        mode: str,
        task_id: str,
    ) -> dict[str, Any] | None:
        if not enabled or sample.frozen:
            self.reset()
            return None
        temperature = float(sample.max_temperature_c)
        now = self._clock()
        if self._active:
            self._peak_temperature_c = max(
                temperature, float(self._peak_temperature_c or temperature))
            if temperature < float(threshold_c) - self.clear_delta_c:
                self.reset()
            return None
        if temperature < float(threshold_c):
            self._pending_since = None
            return None
        if self._pending_since is None:
            self._pending_since = now
        if now - self._pending_since < self.hold_seconds:
            return None
        self._active = True
        self._peak_temperature_c = temperature
        self._event_number += 1
        return {
            "category": "high_temperature",
            "display_name": "高温",
            "alert": True,
            "measurement_type": "temperature",
            "severity": "alarm",
            "confidence": None,
            "max_temperature_c": round(temperature, 1),
            "threshold_c": round(float(threshold_c), 1),
            "unit": "C",
            "x_normalized": int(sample.x_normalized),
            "y_normalized": int(sample.y_normalized),
            "frame_number": int(sample.frame_number),
            "timestamp_epoch": float(sample.captured_at_epoch),
            "temperature_source": sample.temperature_source,
            "event_key": f"thermal:{task_id or 'manual'}:{mode}:{self._event_number}",
        }


class ThermalMonitor:
    """Deep module used by the thermal HTTP process and its in-memory tests."""

    VALID_MODES = {"manual", "paused", "transit", "inspection"}

    def __init__(
        self,
        config_store: ThermalConfigStore,
        *,
        hold_seconds: float = DEFAULT_HOLD_SECONDS,
        clear_delta_c: float = DEFAULT_CLEAR_DELTA_C,
        stale_seconds: float = DEFAULT_STALE_SECONDS,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if stale_seconds <= 0:
            raise ValueError("热成像帧过期时间必须大于 0")
        self.config_store = config_store
        self.stale_seconds = float(stale_seconds)
        self._wall_clock = wall_clock
        self._condition = threading.Condition(threading.RLock())
        self._detector = HighTemperatureDetector(
            hold_seconds=hold_seconds,
            clear_delta_c=clear_delta_c,
            clock=monotonic_clock,
        )
        self._latest: ThermalSample | None = None
        self._frame_sequence = 0
        self._event_sequence = 0
        self._events: deque[dict[str, Any]] = deque(maxlen=256)
        self._backend_running = False
        self._backend_generation = 0
        self._backend_error = ""
        self._sdk_version = ""
        self._device = ""
        self._manual_detection_enabled = False
        self._mode = "manual"
        self._task_id = ""

    def set_backend_state(
        self, *, running: bool, error: str = "", sdk_version: str = "", device: str = ""
    ) -> None:
        with self._condition:
            self._backend_running = bool(running)
            self._backend_error = str(error or "")
            if not running:
                self._backend_generation += 1
                self._latest = None
                self._detector.reset()
                self._sdk_version = ""
                self._device = ""
            if sdk_version:
                self._sdk_version = str(sdk_version)
            if device:
                self._device = str(device)
            self._condition.notify_all()

    def set_detection(self, enabled: bool) -> dict[str, Any]:
        with self._condition:
            self._manual_detection_enabled = bool(enabled)
            if not self._detection_enabled_locked():
                self._detector.reset()
        return self.status()

    def set_mode(self, mode: str, task_id: str = "") -> dict[str, Any]:
        mode = str(mode or "").strip().lower()
        if mode not in self.VALID_MODES:
            raise ValueError("热成像巡检模式无效")
        with self._condition:
            self._mode = mode
            self._task_id = str(task_id or "").strip()
            if not self._detection_enabled_locked():
                self._detector.reset()
        return self.status()

    def _detection_enabled_locked(self) -> bool:
        if self._mode == "paused":
            return False
        return self._manual_detection_enabled or self._mode in {"transit", "inspection"}

    def ingest(self, sample: ThermalSample) -> dict[str, Any] | None:
        sample.validate()
        with self._condition:
            self._latest = sample
            self._frame_sequence += 1
            config = self.config_store.get(self._task_id)
            event = self._detector.process(
                sample,
                threshold_c=float(config["threshold_c"]),
                enabled=self._detection_enabled_locked(),
                mode=self._mode,
                task_id=self._task_id,
            )
            if event is not None:
                self._event_sequence += 1
                event = {
                    **event,
                    "sequence": self._event_sequence,
                    "mode": self._mode,
                    "task_id": self._task_id,
                    "raw_jpeg_base64": base64.b64encode(sample.raw_jpeg).decode("ascii"),
                    "annotated_jpeg_base64": base64.b64encode(sample.annotated_jpeg).decode("ascii"),
                }
                self._events.append(event)
            self._condition.notify_all()
            return dict(event) if event is not None else None

    def next_frame(self, sequence: int, timeout: float = 5.0) -> tuple[int, bytes]:
        deadline = time.monotonic() + max(0.01, float(timeout))
        with self._condition:
            while self._frame_sequence <= int(sequence):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("等待热成像帧超时")
                self._condition.wait(remaining)
            if self._latest is None:
                raise RuntimeError("热成像尚无画面")
            return self._frame_sequence, self._latest.annotated_jpeg

    def snapshot(self, *, annotated: bool = True) -> bytes:
        with self._condition:
            if self._latest is None:
                raise RuntimeError("热成像尚无画面")
            return self._latest.annotated_jpeg if annotated else self._latest.raw_jpeg

    def events(self, since: int) -> dict[str, Any]:
        with self._condition:
            items = [dict(item) for item in self._events if int(item["sequence"]) > int(since)]
            sequence = self._event_sequence
        return {"success": True, "sequence": sequence, "events": items}

    def capture(self, duration_seconds: float = 2.0) -> dict[str, Any]:
        duration = min(10.0, max(0.0, float(duration_seconds)))
        deadline = time.monotonic() + duration
        candidates: list[ThermalSample] = []
        with self._condition:
            starting_sequence = self._frame_sequence
            starting_generation = self._backend_generation
            if self._latest is not None:
                candidates.append(self._latest)
            observed_sequence = self._frame_sequence
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                self._condition.wait(min(remaining, 0.25))
                if self._backend_generation != starting_generation:
                    raise RuntimeError("热成像采集已中断，本次抓拍作废，请恢复后重试")
                if self._frame_sequence != observed_sequence and self._latest is not None:
                    observed_sequence = self._frame_sequence
                    candidates.append(self._latest)
        if not candidates:
            raise RuntimeError("抓拍窗口内没有热成像帧")
        valid = [item for item in candidates if not item.frozen]
        if not valid:
            raise RuntimeError("抓拍窗口内热成像帧全部冻结")
        peak = max(valid, key=lambda item: item.max_temperature_c)
        config = self.config_store.get(self._task_id)
        detection = {
            "category": "high_temperature",
            "display_name": "高温",
            "alert": True,
            "measurement_type": "temperature",
            "severity": "alarm",
            "confidence": None,
            "max_temperature_c": round(float(peak.max_temperature_c), 1),
            "threshold_c": float(config["threshold_c"]),
            "unit": "C",
            "x_normalized": int(peak.x_normalized),
            "y_normalized": int(peak.y_normalized),
            "timestamp_epoch": float(peak.captured_at_epoch),
            "temperature_source": peak.temperature_source,
        }
        is_alert = (
            self._detection_enabled_locked()
            and float(peak.max_temperature_c) >= float(config["threshold_c"])
        )
        return {
            "success": True,
            "sequence": starting_sequence,
            "timestamp_epoch": float(peak.captured_at_epoch),
            "max_temperature_c": round(float(peak.max_temperature_c), 1),
            "threshold_c": float(config["threshold_c"]),
            "detections": [detection] if is_alert else [],
            "raw_jpeg_base64": base64.b64encode(peak.raw_jpeg).decode("ascii"),
            "annotated_jpeg_base64": base64.b64encode(peak.annotated_jpeg).decode("ascii"),
        }

    def status(self) -> dict[str, Any]:
        with self._condition:
            latest = self._latest
            now = self._wall_clock()
            age = None if latest is None else max(0.0, now - latest.captured_at_epoch)
            stale = age is None or age > self.stale_seconds or bool(latest and latest.frozen)
            config = self.config_store.get(self._task_id)
            enabled = self._detection_enabled_locked()
            return {
                "available": self._backend_running and not bool(self._backend_error),
                "running": self._backend_running,
                "ready": self._backend_running and latest is not None and not stale,
                "error": self._backend_error,
                "sdk_version": self._sdk_version,
                "device": self._device,
                "mode": self._mode,
                "task_id": self._task_id,
                "detection_enabled": enabled,
                "manual_detection_enabled": self._manual_detection_enabled,
                "alarm_active": self._detector.active,
                "peak_temperature_c": self._detector.peak_temperature_c,
                "max_temperature_c": None if latest is None else round(float(latest.max_temperature_c), 1),
                "x_normalized": None if latest is None else int(latest.x_normalized),
                "y_normalized": None if latest is None else int(latest.y_normalized),
                "frame_number": None if latest is None else int(latest.frame_number),
                "frame_sequence": self._frame_sequence,
                "frame_age_seconds": None if age is None else round(age, 3),
                "stale": stale,
                "frozen": bool(latest and latest.frozen),
                "width": None if latest is None else int(latest.width),
                "height": None if latest is None else int(latest.height),
                "temperature_source": "" if latest is None else latest.temperature_source,
                "stream_proxy_url": "/api/thermal/stream.mjpg",
                "frame_url": "/api/thermal/frame.jpg",
                "config": config,
            }

