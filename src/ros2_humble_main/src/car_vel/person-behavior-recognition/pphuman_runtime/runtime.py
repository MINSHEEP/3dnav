"""Timestamp-governed temporal inference, decoupled from ROS/capture."""

import math
import time
from collections import deque
from pathlib import Path

import numpy as np


NS = 1_000_000_000


def load_config(path):
    import yaml
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if "profile" in cfg:
        from .runtime_yolo import load_config as load_yolo_config
        return load_yolo_config(path)
    cfg["project_root"] = str(path.parent.parent)
    if cfg.get("protocol_version") != 1:
        raise ValueError("Only protocol version 1 is supported")
    if set(cfg["models"]) != {"mot", "pose", "fall", "smoking", "fight"}:
        raise ValueError("All five real model directories are required")
    for name in ("fall", "fight", "smoking"):
        if not 0 < cfg[name]["sample_hz"] <= 120 or cfg[name]["result_ttl_seconds"] <= 0:
            raise ValueError("Invalid sampling rate or TTL for " + name)
    for name in ("fall", "fight"):
        block = cfg[name]
        expected = (block["window_frames"] - 1) / block["sample_hz"]
        if not math.isclose(block["nominal_window_seconds"], expected, abs_tol=1e-6):
            raise ValueError("nominal_window_seconds inconsistent for " + name)
        if block["max_window_seconds"] < expected or block["max_sample_gap_seconds"] < 1 / block["sample_hz"]:
            raise ValueError("Temporal limits too short for " + name)
    return cfg


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
            "prediction_stamp_ns": stamp, "window": dict(window) if isinstance(window, dict) else window.info(),
            **({"reason": "below_confidence_threshold", "raw_model_score": score} if not valid else {})}


def current(cache, stamp, cfg):
    if cache is None:
        return empty_action()
    if stamp - cache["prediction_stamp_ns"] > cfg["result_ttl_seconds"] * NS:
        return empty_action("unknown", "expired", cache["prediction_stamp_ns"])
    return dict(cache, updated=(cache["state"] == "valid" and stamp == cache["prediction_stamp_ns"]))


class TrackState:
    def __init__(self, public_id, cfg, stamp):
        self.public_id = public_id
        self.last_seen = stamp
        self.fall_window = TimeWindow(cfg["fall"])
        self.smoking_clock = TimeWindow(cfg["smoking"])
        self.fall = self.smoking = None

    def interrupt(self):
        self.fall_window.clear()
        self.smoking_clock.clear()
        self.fall = self.smoking = None


class Runtime:
    def __init__(self, cfg, backend):
        self.cfg = cfg
        self.backend = backend
        self.next_public_id = 1
        self.generation = 0
        self.reset("startup")

    def reset(self, reason="explicit_reset"):
        self.backend.reset()
        self.tracks = {}
        self.fight_window = TimeWindow(self.cfg["fight"])
        self.fight_cache = None
        self.last_stamp = self.last_shape = self.source_epoch = None
        self.generation += 1
        self.reset_reason = reason

    def infer(self, bgr, header):
        started = time.perf_counter()
        stamp, epoch = header["stamp_ns"], header["source_epoch"]
        reason = None
        if self.last_stamp is not None:
            if epoch != self.source_epoch:
                reason = "source_epoch_changed"
            elif bgr.shape != self.last_shape:
                reason = "shape_changed"
            elif stamp <= self.last_stamp:
                reason = "timestamp_nonincreasing"
            elif stamp - self.last_stamp > self.cfg["stream"]["reset_gap_seconds"] * NS:
                reason = "stream_gap"
        if reason:
            self.reset(reason)
        reset_reason = self.reset_reason
        self.reset_reason = None
        self.last_stamp, self.source_epoch, self.last_shape = stamp, epoch, bgr.shape
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        timing = {}

        def call(name, fn, *args):
            t = time.perf_counter()
            result = fn(*args)
            timing[name] = timing.get(name, 0.0) + (time.perf_counter() - t) * 1000
            return result

        # Scene sampling is independent of MOT and runs even on an empty frame.
        if self.fight_window.due(stamp):
            sample = call("fight_preprocess", self.backend.prepare_fight, rgb)
            if self.fight_window.add(stamp, sample):
                self.fight_cache = None
            if self.fight_window.full and self._prediction_due(self.fight_cache, stamp, "fight"):
                raw = call("fight", self.backend.fight, [x[1] for x in self.fight_window.samples])
                self.fight_cache = classification("fight", raw, self.cfg["fight"], stamp, self.fight_window)

        people = call("mot", self.backend.tracks, rgb)
        seen = {p["raw_id"] for p in people}
        for raw_id in list(self.tracks):
            state = self.tracks[raw_id]
            if raw_id not in seen:
                state.interrupt()
            if stamp - state.last_seen > self.cfg["mot"]["track_retention_seconds"] * NS:
                del self.tracks[raw_id]
        for person in people:
            raw_id = person["raw_id"]
            if raw_id not in self.tracks:
                self.tracks[raw_id] = TrackState(self.next_public_id, self.cfg, stamp)
                self.next_public_id += 1
            self.tracks[raw_id].last_seen = stamp

        crops = self.backend.crops(rgb, people)
        pose_crops = {i: crop for i, crop in crops.items() if self.tracks[i].fall_window.due(stamp)}
        smoke_crops = {i: crop for i, crop in crops.items() if self.tracks[i].smoking_clock.due(stamp)}
        poses = call("pose", self.backend.pose, pose_crops) if pose_crops else {}
        smoking = call("smoking", self.backend.smoking, smoke_crops) if smoke_crops else {}
        result_people = []
        for person in people:
            raw_id = person["raw_id"]
            state = self.tracks[raw_id]
            pose = poses.get(raw_id)
            pose_valid = pose is not None and self._pose_valid(pose)
            if raw_id not in crops:
                state.interrupt()
            elif pose is not None:
                if pose_valid:
                    discontinuity = state.fall_window.add(stamp, (pose.copy(), crops[raw_id][2]))
                    if discontinuity:
                        state.fall = None
                    if state.fall_window.full and self._prediction_due(state.fall, stamp, "fall"):
                        raw = call("fall", self.backend.fall, [x[1] for x in state.fall_window.samples])
                        state.fall = classification("fall", raw, self.cfg["fall"], stamp, state.fall_window)
                else:
                    state.fall_window.skip_invalid(stamp)
                    state.fall = None
            if raw_id in smoking:
                state.smoking_clock.add(stamp, None)
                state.smoking = dict(smoking[raw_id], state="valid", updated=True,
                                     prediction_stamp_ns=stamp)
            fall_result = current(state.fall, stamp, self.cfg["fall"])
            if state.fall is None:
                fall_result["window"] = state.fall_window.info()
            if pose is not None and not pose_valid:
                fall_result = empty_action("unknown", "insufficient_visible_keypoints")
            smoke_result = current(state.smoking, stamp, self.cfg["smoking"])
            if raw_id not in crops:
                fall_result = empty_action("unknown", "invalid_person_crop")
                smoke_result = empty_action("unknown", "invalid_person_crop")
            output = {"track_id": state.public_id, "bbox": person["bbox"],
                      "score": person["score"], "actions": {"fall": fall_result, "smoking": smoke_result}}
            if pose_valid:
                output["keypoints"] = pose.tolist()
                output["keypoints_stamp_ns"] = stamp
            result_people.append(output)
        fight = current(self.fight_cache, stamp, self.cfg["fight"])
        if self.fight_cache is None:
            fight["window"] = self.fight_window.info()
        timing["total"] = (time.perf_counter() - started) * 1000
        actions = [fight] + [a for p in result_people for a in p["actions"].values()]
        status = "valid" if all(a["state"] == "valid" for a in actions) else "warming"
        if any(a["state"] == "unknown" for a in actions):
            status = "partial"
        return {"type": "result", "protocol_version": 1,
                "request_id": header["request_id"], "stamp_ns": stamp, "source_epoch": epoch,
                "status": status, "persons": result_people, "scene": {"fight": fight},
                "timing_ms": timing, "generation": self.generation,
                "reset_reason": reset_reason, "models": self.backend.identities}

    def _prediction_due(self, cache, stamp, kind):
        return cache is None or stamp - cache["prediction_stamp_ns"] >= self.cfg[kind]["prediction_interval_seconds"] * NS

    def _pose_valid(self, points):
        return (points.shape == (17, 3) and np.isfinite(points).all()
                and np.count_nonzero(points[:, 2] >= self.cfg["pose"]["keypoint_threshold"])
                >= self.cfg["pose"]["minimum_visible_joints"])
