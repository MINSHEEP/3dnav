"""Candidate shared-pose runtime; timestamp protocol compatible with legacy."""
import math
import time
from pathlib import Path

import numpy as np

from .fall_rules import FallRuleEngine
from .smoking_branch import SmokingBranch
from .runtime import NS, TimeWindow, classification, current, empty_action


def load_config(path):
    import yaml
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    cfg["project_root"] = str(path.parent.parent)
    if cfg.get("protocol_version") != 1 or cfg.get("profile") != "yolo_pose_rules_paddle_aux_v1":
        raise ValueError("Expected candidate shared-pose profile/protocol")
    if set(cfg["models"]) != {"yolo_pose", "smoking", "fight"}:
        raise ValueError("Exactly three actual model identities are required")
    branches = cfg.setdefault("branches", {"fall": True, "fight": True, "smoking": True})
    if set(branches) != {"fall", "fight", "smoking"} or any(type(value) is not bool for value in branches.values()):
        raise ValueError("branches must contain boolean fall, fight and smoking switches")
    engine = cfg["yolo_pose"].get("backend", "onnxruntime")
    if cfg["yolo_pose"].get("output_format", "yolov8_raw") not in ("yolov8_raw", "yolo26_end2end"):
        raise ValueError("Unknown pose output format")
    if engine not in ("onnxruntime", "openvino"):
        raise ValueError("Unknown pose inference backend: " + str(engine))
    fight_engine = cfg["fight"].get("backend", "paddle")
    if fight_engine not in ("paddle", "openvino"):
        raise ValueError("Unknown fight inference backend: " + str(fight_engine))
    smoking_engine = cfg["smoking"].get("backend", "paddle")
    if smoking_engine not in ("paddle", "openvino"):
        raise ValueError("Unknown smoking inference backend: " + str(smoking_engine))
    if type(cfg["smoking"].get("skip_unobservable", False)) is not bool:
        raise ValueError("smoking.skip_unobservable must be boolean")
    if "opencv_threads" in cfg and (type(cfg["opencv_threads"]) is not int or not 1 <= cfg["opencv_threads"] <= 16):
        raise ValueError("opencv_threads must be an integer in [1,16]")
    if fight_engine == "openvino":
        if not isinstance(cfg["models"]["fight"], str) or not cfg["models"]["fight"].endswith(".xml"):
            raise ValueError("OpenVINO fight requires an XML model path")
        if cfg["fight"]["window_frames"] != 8 or cfg["fight"]["target_size"] not in (224, 320):
            raise ValueError("The CPU fight candidate requires eight 224x224 or 320x320 frames")
    if type(cfg["fight"].get("async_inference", False)) is not bool:
        raise ValueError("fight.async_inference must be a boolean")
    if cfg["fight"].get("async_inference", False):
        if fight_engine != "openvino":
            raise ValueError("Asynchronous fight requires the OpenVINO CPU backend")
        timeout = cfg["fight"].get("async_timeout_seconds", 5.0)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("fight.async_timeout_seconds must be finite and positive")
    if type(cfg.get("enable_mkldnn", False)) is not bool:
        raise ValueError("enable_mkldnn must be a boolean")
    if "openvino" in (engine, fight_engine, smoking_engine) or cfg.get("enable_mkldnn", False):
        if cfg["device"].upper() != "CPU":
            raise ValueError("CPU optimization options require device: CPU")
        for count in (cfg["cpu_threads"], cfg["yolo_pose"].get("cpu_threads", 4),
                      cfg["fight"].get("cpu_threads", cfg["cpu_threads"]),
                      cfg["smoking"].get("cpu_threads", cfg["cpu_threads"])):
            if type(count) is not int or not 1 <= count <= 16:
                raise ValueError("CPU inference threads must be an integer in [1,16]")
    for name in ("smoking", "fight"):
        if not 0 < cfg[name]["sample_hz"] <= 120 or cfg[name]["result_ttl_seconds"] <= 0:
            raise ValueError("Invalid sampling/TTL: " + name)
    fight = cfg["fight"]
    if type(fight.get('capture_buffer', False)) is not bool:
        raise ValueError('fight.capture_buffer must be boolean')
    if fight.get('capture_buffer', False):
        if not fight.get('async_inference', False):
            raise ValueError('Capture buffer requires asynchronous fight inference')
        from .fight_sampling import FightSampler
        FightSampler(fight)
    expected = (fight["window_frames"] - 1) / fight["sample_hz"]
    if not math.isclose(fight["nominal_window_seconds"], expected, abs_tol=1e-6):
        raise ValueError("Inconsistent fight sample span")
    if fight["max_window_seconds"] < expected or fight["max_sample_gap_seconds"] < 1 / fight["sample_hz"]:
        raise ValueError("Fight window limits too short")
    if not fight["minimum_score"] <= fight.get("positive_minimum_score", fight["minimum_score"]) <= 1:
        raise ValueError("Fight positive threshold must be between minimum_score and one")
    if not 1 <= cfg["smoking"]["max_rois_per_tick"] <= cfg["smoking"]["batch_size"]:
        raise ValueError("Smoking budget must fit one batch")
    diagnostic_limit = cfg["smoking"].get("diagnostic_max_detections", 20)
    if type(diagnostic_limit) is not int or not 0 <= diagnostic_limit <= 100:
        raise ValueError("Smoking diagnostic limit must be an integer between 0 and 100")
    if cfg["smoking"].get("hand_to_mouth", {}).get("enabled") and not cfg["smoking"].get("association", {}).get("enabled"):
        raise ValueError("Hand-to-mouth confirmation requires smoking spatial association")
    refinement = cfg['smoking'].get('profile_refinement', {})
    if refinement.get('enabled'):
        if not cfg['smoking'].get('association', {}).get('enabled'):
            raise ValueError('Profile refinement requires smoking spatial association')
        high = refinement.get('threshold', cfg['smoking']['threshold'])
        low = refinement.get('continue_threshold', high)
        if not all(math.isfinite(v) for v in (low, high)) or not 0 < low <= high <= 1:
            raise ValueError('Profile continuation/start scores must satisfy 0 < low <= high <= 1')
        budget = refinement.get('max_rois_per_tick', 2)
        if type(budget) is not int or not 1 <= budget <= cfg['smoking']['max_rois_per_tick']:
            raise ValueError('Profile ROI budget must be a positive integer within the smoking ROI budget')
        for key, default in [('crop_radius_heads', 2.), ('minimum_profile_ratio', .35), ('maximum_head_pixels', 64.)]:
            if not math.isfinite(refinement.get(key, default)) or refinement.get(key, default) <= 0:
                raise ValueError('Invalid profile refinement parameter: '+key)
    if cfg["fall"]["lying_confirm_seconds"] <= 0 or cfg["fall"]["max_observation_gap_seconds"] <= 0:
        raise ValueError("Rule confirmation/gap must be positive")
    if cfg["fall"].get("minimum_lying_observations", 3) < 2 or cfg["fall"].get("max_visibility_gap_seconds", .4) <= 0:
        raise ValueError("Rule confirmation requires repeated observations and a positive visibility deadline")
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
        self.fight_pending = None
        self.fight_serial = 0
        self.fight_stats = {"submitted_windows": 0, "completed_windows": 0,
                            "discarded_windows": 0, "busy_sample_ticks": 0,
                            "last_model_inference_ms": None, "last_drop_reason": None}
        self.reset("startup")

    def reset(self, reason="explicit_reset", *, preserve_fight=False):
        self.backend.reset()
        self.rules = FallRuleEngine(self.cfg["fall"])
        self.smoking = SmokingBranch(self.cfg, self.backend)
        self.enabled_branches = None
        if not preserve_fight:
            self.fight_window = TimeWindow(self.cfg["fight"])
            self.fight_capture_used = None
            self.fight_serial += 1
            self.last_fight_submit = None
            self.fight_cache = None
        self.last_stamp = self.last_shape = self.source_epoch = None
        self.generation += 1
        self.reset_reason = reason

    def close(self):
        close = getattr(self.backend, "close", None)
        if close is not None:
            close()
        self.fight_pending = None

    def _fight_tick(self, stamp, as_rgb, call, timing, samples=None, poll_only=False):
        cfg = self.cfg["fight"]
        sampled = False
        if not poll_only:
            if samples is None:
                incoming = [(stamp, call("fight_preprocess", self.backend.prepare_fight, as_rgb()))] if self.fight_window.due(stamp) else []
            else:
                from PIL import Image
                last = self.fight_window.samples[-1][0] if self.fight_window.samples else -1
                incoming = [(t, Image.fromarray(rgb) if cfg.get("capture_buffer", False) else rgb)
                            for t, rgb in samples if t > last]
            for source_stamp, sample in incoming:
                sampled = True
                if self.fight_window.add(source_stamp, sample):
                    self.fight_cache = None
                    self.fight_serial += 1
                    self.last_fight_submit = None
        # A buffered job belongs to its actual last capture, never the later
        # main-inference/completion time. Empty input cannot fabricate a window.
        evidence_stamp = self.fight_window.samples[-1][0] if self.fight_window.samples else stamp
        if samples is not None and self.fight_window.samples and stamp-evidence_stamp > cfg['max_sample_gap_seconds']*NS:
            if not poll_only:
                self.fight_window.clear()
                self.fight_cache = None
                self.fight_serial += 1
                self.last_fight_submit = None
                sampled = False
        if not cfg.get("async_inference", False):
            if sampled and self.fight_window.full and (self.fight_cache is None or
                    stamp - self.fight_cache["prediction_stamp_ns"] >= cfg["prediction_interval_seconds"] * NS):
                raw = call("fight", self.backend.fight, [x[1] for x in self.fight_window.samples])
                self.fight_cache = fight_classification(raw, cfg, stamp, self.fight_window)
            result = current(self.fight_cache, stamp, cfg)
        else:
            newly_completed = False
            if self.fight_pending is not None:
                job = self.fight_pending
                raw = call("fight_poll", self.backend.poll_fight)
                if raw is not None:
                    self.fight_pending = None
                    self.fight_stats["completed_windows"] += 1
                    self.fight_stats["last_model_inference_ms"] = raw["model_inference_ms"]
                    timing["fight_async_model"] = raw["model_inference_ms"]
                    if job["serial"] != self.fight_serial:
                        self.fight_stats["discarded_windows"] += 1
                        self.fight_stats["last_drop_reason"] = "source_or_window_reset"
                    elif stamp - job["stamp"] > cfg["result_ttl_seconds"] * NS:
                        self.fight_stats["discarded_windows"] += 1
                        self.fight_stats["last_drop_reason"] = "expired_before_completion"
                        self.fight_cache = empty_action("unknown", "expired_before_completion", job["stamp"])
                    else:
                        self.fight_cache = fight_classification(raw, cfg, job["stamp"], job["window"])
                        newly_completed = self.fight_cache["state"] == "valid"
                        self.fight_stats["last_drop_reason"] = None
                elif time.monotonic() - job["started"] > cfg.get("async_timeout_seconds", 5.0):
                    raise RuntimeError("Background fight inference timed out")
            if sampled and self.fight_window.full:
                if self.fight_pending is not None:
                    self.fight_stats["busy_sample_ticks"] += 1
                elif (self.last_fight_submit is None or
                      evidence_stamp - self.last_fight_submit >= cfg["prediction_interval_seconds"] * NS):
                    job = {"serial": self.fight_serial, "stamp": evidence_stamp,
                           "window": self.fight_window.info(), "started": time.monotonic()}
                    call("fight_submit", self.backend.start_fight, [x[1] for x in self.fight_window.samples])
                    self.fight_pending = job
                    self.last_fight_submit = evidence_stamp
                    self.fight_stats["submitted_windows"] += 1
            result = current(self.fight_cache, stamp, cfg)
            # Completion is a new observation once; its evidence timestamp is
            # still the source window end, never the later completion timestamp.
            result["updated"] = newly_completed and result["state"] == "valid"
        if self.fight_cache is None:
            result["window"] = self.fight_window.info()
        return result

    def infer(self, bgr, header, fight_samples=None):
        started = time.perf_counter()
        stamp, epoch = header["stamp_ns"], header["source_epoch"]
        requested = header.get("enabled_branches", self.cfg["branches"])
        enabled = {name: active and requested.get(name, True) for name, active in self.cfg["branches"].items()}
        legacy_clip = fight_samples is not None and "fight_samples" not in header
        if legacy_clip and "enabled_branches" in header and "smoking" not in requested:
            enabled["smoking"] = False
        pose_enabled = enabled["fall"] or enabled["smoking"]
        captured = enabled["fight"] and fight_samples is not None
        if captured:
            fight_cfg = self.cfg["fight"]
            if legacy_clip:
                if pose_enabled or not fight_samples or fight_samples[-1][0] != stamp:
                    raise ValueError("Legacy BGR clips require a fight-only worker and a matching final frame")
                if fight_cfg.get("capture_buffer", False):
                    from .fight_sampling import scale_capture_frame
                    fight_samples = [(t, scale_capture_frame(frame, fight_cfg["short_size"]))
                                     for t, frame in fight_samples]
                else:
                    fight_samples = [(t, self.backend.prepare_fight(np.ascontiguousarray(frame[:, :, ::-1])))
                                     for t, frame in fight_samples]
            elif not fight_cfg.get("capture_buffer", False):
                raise ValueError("Capture buffer is not enabled in this configuration")
            if fight_cfg.get("capture_buffer", False):
                from .fight_sampling import validate_samples
                validate_samples(fight_samples, stamp, fight_cfg)
        reason = None
        if self.last_stamp is not None:
            if epoch != self.source_epoch: reason = "source_epoch_changed"
            elif bgr.shape != self.last_shape: reason = "shape_changed"
            elif stamp <= self.last_stamp: reason = "timestamp_nonincreasing"
            elif stamp - self.last_stamp > self.cfg["stream"]["reset_gap_seconds"] * NS: reason = "stream_gap"
            elif enabled != self.enabled_branches: reason = "branches_changed"
        if reason:
            # The camera capture clock keeps feeding the fight window even
            # when a slow pose forward leaves a gap between requests.
            self.reset(reason, preserve_fight=(
                reason == "stream_gap" and captured and self.fight_capture_used is True
                and enabled == self.enabled_branches))
        self.enabled_branches = enabled
        reset_reason, self.reset_reason = self.reset_reason, None
        self.last_stamp, self.source_epoch, self.last_shape = stamp, epoch, bgr.shape
        timing = {}

        def call(name, fn, *args):
            start = time.perf_counter()
            result = fn(*args)
            timing[name] = timing.get(name, 0.) + (time.perf_counter() - start) * 1000
            return result

        if self.fight_capture_used is not None and captured != self.fight_capture_used:
            self.fight_window.clear()
            self.fight_cache = None
            self.fight_serial += 1
            self.last_fight_submit = None
        self.fight_capture_used = captured
        early_fight = self._fight_tick(stamp, None, call, timing, samples=fight_samples) if captured else None
        people = []
        smoking_actions, smoking_schedule = {}, {}
        if enabled["fall"] or enabled["smoking"]:
            observations = call("shared_pose", self.backend.observe, bgr, stamp)
            timing.update(getattr(self.backend, "last_timing", {}))
            for track_id in getattr(getattr(self.backend, "tracker", None), "retired_ids", ()):
                self.rules.forget(track_id)
            self.rules.expire(stamp, {p["track_id"] for p in observations})
            if enabled["smoking"]:
                smoking_actions, smoking_schedule = self.smoking.update(bgr, observations, stamp, epoch, call)
            for person in observations:
                track_id = person["track_id"]
                fall = call("fall_rules", self.rules.update, track_id, person["keypoints"], person["bbox"], stamp,
                            bgr.shape, person.get("association_evidence"),
                            {"track_id": track_id, "source_stamp_ns": person.get("observation_stamp_ns"),
                             "rotation_cw": person.get("observation_rotation_cw", 0),
                             "association": person.get("association")}) if enabled["fall"] else empty_action("disabled", "branch_disabled")
                people.append({"track_id": track_id, "bbox": person["bbox"], "score": person["score"],
                               "keypoints": np.asarray(person["keypoints"]).tolist(), "keypoints_stamp_ns": stamp,
                               "observation_rotation_cw": person.get("observation_rotation_cw", 0),
                               "pose_observation_stamp_ns": person.get("observation_stamp_ns", stamp),
                               "association": person.get("association"),
                               "association_evidence": person.get("association_evidence"),
                               "actions": {"fall": fall, **({"smoking": smoking_actions[track_id]} if enabled["smoking"] else {})}})

        def as_rgb():
            return np.ascontiguousarray(bgr[:, :, ::-1])

        # Sampling continues while the one background fight request is busy.
        if not enabled["fight"]:
            fight = empty_action("disabled", "branch_disabled")
        elif captured:
            # Poll after the heavy branches too, so completed work can be
            # returned in this request. Account for time spent in inference.
            completed_at = stamp + round((time.perf_counter()-started)*NS)
            fight = self._fight_tick(completed_at, None, call, timing, poll_only=True)
            if (early_fight.get('updated') and fight['state'] == 'valid' and
                    early_fight.get('prediction_stamp_ns') == fight.get('prediction_stamp_ns')):
                fight['updated'] = True
        else:
            fight = self._fight_tick(stamp, as_rgb, call, timing)

        actions = [fight] + [a for p in people for a in p["actions"].values()]
        status = "valid" if all(a["state"] in ("valid", "disabled") for a in actions) else "warming"
        if any(a["state"] == "unknown" for a in actions): status = "partial"
        timing["total"] = (time.perf_counter() - started) * 1000
        return {"type": "result", "protocol_version": 1, "profile": self.cfg["profile"],
                "request_id": header["request_id"], "stamp_ns": stamp, "source_epoch": epoch,
                "status": status, "persons": people, "scene": {"fight": fight}, "timing_ms": timing,
                "enabled_branches": enabled,
                "pose_status": {"state": "valid" if pose_enabled else "disabled", "person_count": len(people)},
                "pose_observation": getattr(self.backend, "last_pose_observation", None) if pose_enabled else None,
                "generation": self.generation, "reset_reason": reset_reason, "models": self.backend.identities,
                "scheduling": {"fight": {**self.fight_stats,
                    "sampling_source": "capture_buffer" if captured else "inference_frames",
                    "mode": "async" if self.cfg["fight"].get("async_inference", False) else "sync",
                    "pending": self.fight_pending is not None,
                    "pending_source_stamp_ns": self.fight_pending["stamp"] if self.fight_pending else None,
                    "window": self.fight_window.info()}, "smoking": smoking_schedule}}
