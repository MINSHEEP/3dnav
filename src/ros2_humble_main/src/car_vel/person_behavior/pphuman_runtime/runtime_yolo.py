"""Two-model CPU runtime adapted from the supplied shared-pose implementation.

Fall is person-level rule evidence, with no numeric confidence. Fight is a
scene-level model result. Capture timestamps, not processed-frame counts,
govern all temporal history. Disabled branches do not run model forwards.
"""
import math
import time
from pathlib import Path

import numpy as np

from .fall_rules import FallRuleEngine
from .runtime import NS, TimeWindow, classification, current, empty_action


def load_config(path):
    import yaml
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    cfg["project_root"] = str(path.parent.parent)
    if cfg.get("protocol_version") != 1 or cfg.get("profile") != "yolo_pose_rules_paddle_aux_v1":
        raise ValueError("Expected two-model shared-pose profile and protocol 1")
    if set(cfg["models"]) != {"yolo_pose", "fight"} or "smoking" in cfg:
        raise ValueError("Only yolo_pose and fight models are supported")
    if cfg.get("device") != "CPU" or cfg.get("run_mode") != "paddle":
        raise ValueError("This deployed runtime requires CPU and paddle run mode")
    for threads in (cfg.get("cpu_threads"), cfg["yolo_pose"].get("cpu_threads")):
        if type(threads) is not int or not 1 <= threads <= 16:
            raise ValueError("CPU thread count must be between 1 and 16")
    branches = cfg.setdefault("branches", {"fall": True, "fight": True})
    if set(branches) != {"fall", "fight"} or any(type(v) is not bool for v in branches.values()):
        raise ValueError("branches must contain boolean fall and fight switches")
    fight = cfg["fight"]
    if not 0 < fight["sample_hz"] <= 120 or fight["result_ttl_seconds"] <= 0:
        raise ValueError("Invalid fight sampling or TTL")
    if type(fight["window_frames"]) is not int or fight["window_frames"] != 8:
        raise ValueError("The bundled ppTSM export requires eight frames")
    expected = (fight["window_frames"] - 1) / fight["sample_hz"]
    if not math.isclose(fight["nominal_window_seconds"], expected, abs_tol=1e-6):
        raise ValueError("Inconsistent fight sample span")
    if fight["max_window_seconds"] < expected or fight["max_sample_gap_seconds"] < 1 / fight["sample_hz"]:
        raise ValueError("Fight window limits too short")
    if not 0 <= fight["minimum_score"] <= fight.get("positive_minimum_score", fight["minimum_score"]) <= 1:
        raise ValueError("Invalid fight thresholds")
    if fight["prediction_interval_seconds"] <= 0 or cfg["stream"]["reset_gap_seconds"] <= 0:
        raise ValueError("Prediction interval and reset gap must be positive")
    fall = cfg["fall"]
    if fall["lying_confirm_seconds"] <= 0 or fall["max_observation_gap_seconds"] <= 0:
        raise ValueError("Rule confirmation/gap must be positive")
    if fall.get("minimum_lying_observations", 3) < 2 or fall.get("max_visibility_gap_seconds", .4) <= 0:
        raise ValueError("Rules require repeated observations and a positive visibility deadline")
    return cfg


def fight_classification(raw, cfg, stamp, window):
    result = classification("fight", raw, cfg, stamp, window)
    cls, score = (raw["class"], raw["score"]) if isinstance(raw, dict) else raw
    result.update(raw_class=cls, raw_score=score)
    if cls == cfg["positive_class"] and score < cfg.get("positive_minimum_score", cfg["minimum_score"]):
        result.update(label=None, score=None, state="unknown", updated=False, reason="low_positive_score")
    return result


class RuntimeYolo:
    def __init__(self, cfg, backend):
        self.cfg, self.backend = cfg, backend
        self.generation = 0
        self.reset("startup")

    def reset(self, reason="explicit_reset"):
        self.backend.reset()
        self.rules = FallRuleEngine(self.cfg["fall"])
        self.fight_window = TimeWindow(self.cfg["fight"])
        self.fight_cache = None
        self.last_stamp = self.last_shape = self.source_epoch = None
        self.enabled_branches = None
        self.generation += 1
        self.reset_reason = reason

    def infer(self, bgr, header, fight_samples=None):
        started = time.perf_counter()
        stamp, epoch = header["stamp_ns"], header["source_epoch"]
        requested = header.get("enabled_branches", self.cfg["branches"])
        enabled = {name: bool(self.cfg["branches"][name] and requested.get(name, True))
                   for name in ("fall", "fight")}
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
            elif enabled != self.enabled_branches:
                reason = "branches_changed"
        if reason:
            self.reset(reason)
        reset_reason, self.reset_reason = self.reset_reason, None
        self.last_stamp, self.source_epoch, self.last_shape = stamp, epoch, bgr.shape
        self.enabled_branches = enabled
        timing = {}

        def call(name, fn, *args):
            start = time.perf_counter()
            result = fn(*args)
            timing[name] = timing.get(name, 0.) + (time.perf_counter() - start) * 1000
            return result

        people = []
        if enabled["fall"]:
            observations = call("shared_pose", self.backend.observe, bgr, stamp)
            timing.update(getattr(self.backend, "last_timing", {}))
            for track_id in getattr(getattr(self.backend, "tracker", None), "retired_ids", ()):
                self.rules.forget(track_id)
            self.rules.expire(stamp, {p["track_id"] for p in observations})
            for person in observations:
                track_id = person["track_id"]
                fall = call("fall_rules", self.rules.update, track_id, person["keypoints"], person["bbox"], stamp,
                            bgr.shape, person.get("association_evidence"),
                            {"track_id": track_id, "source_stamp_ns": person.get("observation_stamp_ns"),
                             "rotation_cw": person.get("observation_rotation_cw", 0),
                             "association": person.get("association")})
                people.append({"track_id": track_id, "bbox": person["bbox"], "score": person["score"],
                               "keypoints": np.asarray(person["keypoints"]).tolist(), "keypoints_stamp_ns": stamp,
                               "observation_rotation_cw": person.get("observation_rotation_cw", 0),
                               "pose_observation_stamp_ns": person.get("observation_stamp_ns", stamp),
                               "association": person.get("association"),
                               "association_evidence": person.get("association_evidence"),
                               "actions": {"fall": fall}})

        # Scene sampling is independent of person detection, but never fills
        # missed slots with duplicates or claims sparse history is continuous.
        if enabled["fight"]:
            if fight_samples is not None:
                # Rebuild from bounded REAL captures, not consumer processing
                # times. Validate the original gap/span even when inference is
                # slow; a stream gap or reset never licenses invented frames.
                self.fight_window.clear()
                self.fight_cache = None
                if (enabled["fall"] or not 1 <= len(fight_samples) <= 8
                        or fight_samples[-1][0] != stamp
                        or any(a[0] >= b[0] for a, b in zip(fight_samples, fight_samples[1:]))):
                    raise ValueError("Invalid fight-only capture clip")
                for captured, frame in fight_samples:
                    if frame.shape != bgr.shape:
                        raise ValueError("Fight capture clip shape changed")
                    # The producer already samples on its capture clock. Do
                    # not re-anchor that clock at the oldest sliding sample,
                    # which would drop valid jittered captures a second time.
                    sample = call("fight_preprocess", self.backend.prepare_fight,
                                  np.ascontiguousarray(frame[:, :, ::-1]))
                    self.fight_window.add(captured, sample)
                if self.fight_window.full:
                    raw = call("fight", self.backend.fight, [x[1] for x in self.fight_window.samples])
                    self.fight_cache = fight_classification(raw, self.cfg["fight"], stamp, self.fight_window)
            elif self.fight_window.due(stamp):
                rgb = np.ascontiguousarray(bgr[:, :, ::-1])
                sample = call("fight_preprocess", self.backend.prepare_fight, rgb)
                if self.fight_window.add(stamp, sample):
                    self.fight_cache = None
                if self.fight_window.full and (self.fight_cache is None or stamp - self.fight_cache["prediction_stamp_ns"]
                        >= self.cfg["fight"]["prediction_interval_seconds"] * NS):
                    raw = call("fight", self.backend.fight, [x[1] for x in self.fight_window.samples])
                    self.fight_cache = fight_classification(raw, self.cfg["fight"], stamp, self.fight_window)
            fight = current(self.fight_cache, stamp, self.cfg["fight"])
            if self.fight_cache is None:
                fight["window"] = self.fight_window.info()
        else:
            fight = empty_action("disabled", "branch_disabled")
        actions = ([fight] if enabled["fight"] else []) + [p["actions"]["fall"] for p in people]
        status = "valid" if actions and all(a["state"] == "valid" for a in actions) else "warming"
        if any(a["state"] == "unknown" for a in actions):
            status = "partial"
        timing["total"] = (time.perf_counter() - started) * 1000
        return {"type": "result", "protocol_version": 1, "profile": self.cfg["profile"],
                "request_id": header["request_id"], "stamp_ns": stamp, "source_epoch": epoch,
                "status": status, "persons": people, "scene": {"fight": fight}, "timing_ms": timing,
                "enabled_branches": enabled,
                "pose_status": {"state": "valid" if enabled["fall"] else "disabled", "person_count": len(people)},
                "pose_observation": getattr(self.backend, "last_pose_observation", None) if enabled["fall"] else None,
                "generation": self.generation, "reset_reason": reset_reason, "models": self.backend.identities}
