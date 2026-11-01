"""Timestamp primitives adapted from the supplied person-behavior-recognition runtime."""
import math
from collections import deque

NS = 1_000_000_000

def load_config(path):
    from .runtime_yolo import load_config as load
    return load(path)

class TimeWindow:
    def __init__(self, cfg):
        self.cfg = cfg
        self.period = round(NS / cfg["sample_hz"])
        self.samples = deque(maxlen=cfg.get("window_frames", 1))
        self.next_ns = None

    def clear(self):
        self.samples.clear()
        self.next_ns = None

    def due(self, stamp):
        return self.next_ns is None or stamp >= self.next_ns

    def _advance_clock(self, stamp):
        if self.next_ns is None:
            self.next_ns = stamp + self.period
        else:
            self.next_ns += ((stamp - self.next_ns) // self.period + 1) * self.period

    def skip_invalid(self, stamp):
        """Discard evidence, retaining the configured rate for failed attempts."""
        self.samples.clear()
        self._advance_clock(stamp)

    def add(self, stamp, value):
        discontinuity = bool(self.samples and stamp - self.samples[-1][0]
                             > self.cfg.get("max_sample_gap_seconds", float("inf")) * NS)
        if discontinuity:
            self.clear()
        self.samples.append((stamp, value))
        # Never duplicate a sample to fill missed time slots.
        self._advance_clock(stamp)
        limit = self.cfg.get("max_window_seconds", float("inf")) * NS
        while self.samples and stamp - self.samples[0][0] > limit:
            self.samples.popleft()
        return discontinuity

    @property
    def full(self):
        return len(self.samples) == self.samples.maxlen

    def info(self):
        span = (self.samples[-1][0] - self.samples[0][0]) / NS if len(self.samples) > 1 else 0.0
        return {"samples": len(self.samples), "required_samples": self.samples.maxlen,
                "target_sample_hz": self.cfg["sample_hz"], "window_seconds": span,
                "observed_sample_hz": (len(self.samples) - 1) / span if span else None,
                "window_start_stamp_ns": self.samples[0][0] if self.samples else None}


def empty_action(state="warming", reason=None, previous_stamp=None):
    result = {"label": None, "score": None, "state": state, "updated": False,
              "prediction_stamp_ns": previous_stamp}
    if reason:
        result["reason"] = reason
    return result


def classification(kind, raw, cfg, stamp, window):
    details = {}
    if isinstance(raw, dict):
        cls, score = raw["class"], raw["score"]
        details = {key: raw[key] for key in ("score_kind", "raw_model_logits") if key in raw}
    else:
        cls, score = raw
    if cls not in (0, 1) or not math.isfinite(score) or not 0 <= score <= 1:
        raise RuntimeError(kind + " returned an invalid class/score")
    valid = score >= cfg["minimum_score"]
    return {"label": (kind if cls == cfg["positive_class"] else "other") if valid else None,
            "score": score if valid else None, "state": "valid" if valid else "unknown", "updated": valid,
            "model_class": cls, "score_kind": "softmax_probability", **details,
            "prediction_stamp_ns": stamp, "window": window.info(),
            **({"reason": "below_confidence_threshold", "raw_model_score": score} if not valid else {})}


def current(cache, stamp, cfg):
    if cache is None:
        return empty_action()
    if stamp - cache["prediction_stamp_ns"] > cfg["result_ttl_seconds"] * NS:
        return empty_action("unknown", "expired", cache["prediction_stamp_ns"])
    return dict(cache, updated=(cache["state"] == "valid" and stamp == cache["prediction_stamp_ns"]))


