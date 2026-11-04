"""Candidate shared-pose runtime; timestamp protocol compatible with legacy."""
import math
import time
from pathlib import Path

import numpy as np

from .fall_rules import FallRuleEngine
from .smoking_temporal import SmokingTemporal
from .smoking_hand_to_mouth import SmokingHandToMouth
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

    def reset(self, reason="explicit_reset"):
        self.backend.reset()
        self.rules = FallRuleEngine(self.cfg["fall"])
        temporal_cfg = self.cfg["smoking"].get("temporal", {})
        self.smoking_temporal = (SmokingTemporal(dict(temporal_cfg,
            result_ttl_seconds=self.cfg["smoking"]["result_ttl_seconds"]))
            if temporal_cfg.get("enabled", False) else None)
        self.smoking_clock = TimeWindow(self.cfg["smoking"])
        hand_cfg = self.cfg["smoking"].get("hand_to_mouth", {})
        self.smoking_hand = (SmokingHandToMouth(hand_cfg, self.cfg["smoking"].get("association"),
                             self.cfg["tracking"]["max_gap_seconds"]) if hand_cfg.get("enabled") else None)
        refinement = self.cfg['smoking'].get('profile_refinement', {})
        self.profile_smoking = (SmokingTemporal(dict(track_gap_seconds=self.cfg['tracking']['max_gap_seconds'],
                                confirmation_window_seconds=1.25, max_prediction_gap_seconds=1.25))
                                if refinement.get('enabled') else None)
        self.profile_strong = {}
        self.fight_window = TimeWindow(self.cfg["fight"])
        self.fight_serial += 1
        self.last_fight_submit = None
        self.people = {}
        self.fight_cache = None
        self.last_stamp = self.last_shape = self.source_epoch = None
        self.last_smoking_forward = None
        self.smoking_forward_count = 0
        self.generation += 1
        self.reset_reason = reason

    def close(self):
        close = getattr(self.backend, "close", None)
        if close is not None:
            close()
        self.fight_pending = None

    def _fight_tick(self, stamp, as_rgb, call, timing):
        cfg = self.cfg["fight"]
        sampled = self.fight_window.due(stamp)
        if sampled:
            sample = call("fight_preprocess", self.backend.prepare_fight, as_rgb())
            if self.fight_window.add(stamp, sample):
                self.fight_cache = None
                self.fight_serial += 1
                self.last_fight_submit = None
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
                      stamp - self.last_fight_submit >= cfg["prediction_interval_seconds"] * NS):
                    job = {"serial": self.fight_serial, "stamp": stamp,
                           "window": self.fight_window.info(), "started": time.monotonic()}
                    call("fight_submit", self.backend.start_fight, [x[1] for x in self.fight_window.samples])
                    self.fight_pending = job
                    self.last_fight_submit = stamp
                    self.fight_stats["submitted_windows"] += 1
            result = current(self.fight_cache, stamp, cfg)
            # Completion is a new observation once; its evidence timestamp is
            # still the source window end, never the later completion timestamp.
            result["updated"] = newly_completed and result["state"] == "valid"
        if self.fight_cache is None:
            result["window"] = self.fight_window.info()
        return result

    def infer(self, bgr, header):
        started = time.perf_counter()
        stamp, epoch = header["stamp_ns"], header["source_epoch"]
        reason = None
        if self.last_stamp is not None:
            if epoch != self.source_epoch: reason = "source_epoch_changed"
            elif bgr.shape != self.last_shape: reason = "shape_changed"
            elif stamp <= self.last_stamp: reason = "timestamp_nonincreasing"
            elif stamp - self.last_stamp > self.cfg["stream"]["reset_gap_seconds"] * NS: reason = "stream_gap"
        if reason: self.reset(reason)
        reset_reason, self.reset_reason = self.reset_reason, None
        self.last_stamp, self.source_epoch, self.last_shape = stamp, epoch, bgr.shape
        timing = {}

        def call(name, fn, *args):
            start = time.perf_counter()
            result = fn(*args)
            timing[name] = timing.get(name, 0.) + (time.perf_counter() - start) * 1000
            return result

        observations = call("shared_pose", self.backend.observe, bgr, stamp)
        timing.update(getattr(self.backend, "last_timing", {}))
        for track_id in getattr(getattr(self.backend, "tracker", None), "retired_ids", ()):
            self.people.pop(track_id, None)
            self.rules.forget(track_id)
            if self.smoking_temporal is not None:
                self.smoking_temporal.forget(track_id)
            if self.smoking_hand is not None:
                self.smoking_hand.forget(track_id)
        seen = {p["track_id"] for p in observations}
        if self.profile_smoking is not None:
            self.profile_smoking.begin_frame(stamp, seen, epoch)
            for person in observations:
                if person.get('association', 'associated') != 'associated':
                    self.profile_smoking.forget(person['track_id'])
                    self.profile_strong.pop(person['track_id'], None)
            self.profile_strong = {i: t for i, t in self.profile_strong.items()
                                   if i in self.profile_smoking.tracks and stamp-t <= 1.25*NS}
            for track_id, state in self.people.items():
                cached = state.get('smoking') or {}
                if cached.get('reason') == 'profile_head_refinement' and track_id not in self.profile_smoking.tracks:
                    state['smoking'] = dict(empty_action('unknown', 'profile_sequence_invalidated', stamp))
        if self.smoking_hand is not None:
            invalidated = self.smoking_hand.begin_frame(stamp, observations, epoch)
            for track_id, state in self.people.items():
                cached = state.get("smoking") or {}
                if (track_id not in seen or (track_id in invalidated
                        and cached.get("reason") in ("cigarette_hand_to_mouth", "cigarette_near_face"))):
                    state["smoking"] = dict(empty_action("unknown", "hand_sequence_invalidated", stamp))
        if self.smoking_temporal is not None:
            self.smoking_temporal.begin_frame(stamp, seen, epoch)
            for track_id, state in self.people.items():
                if track_id not in seen:
                    state["smoking"] = dict(empty_action("unknown", "track_not_observed", stamp))
        self.rules.expire(stamp, seen)
        for track_id, state in list(self.people.items()):
            if stamp - state["last_seen"] > self.cfg["tracking"]["max_gap_seconds"] * NS:
                del self.people[track_id]
                # Fall candidates expire independently and may survive a short,
                # explicitly verified tracker reassociation. Smoking does not.
        for p in observations:
            state = self.people.setdefault(p["track_id"], {"first_seen": stamp, "last_seen": stamp,
                                                         "last_attempt": None, "smoking": None})
            if ((self.smoking_temporal is not None or self.smoking_hand is not None)
                    and p.get("association", "associated") != "associated"):
                if self.smoking_temporal is not None:
                    self.smoking_temporal.forget(p["track_id"])
                if self.smoking_hand is not None:
                    self.smoking_hand.forget(p["track_id"])
                state["smoking"] = dict(empty_action("unknown", "track_association_changed", stamp))
            state["last_seen"] = stamp
        rgb = None

        def as_rgb():
            nonlocal rgb
            if rgb is None: rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            return rgb

        smoke_cfg = self.cfg["smoking"]
        period = self.smoking_clock.period

        def eligible_since(track_id):
            state = self.people[track_id]
            return state["first_seen"] if state["last_attempt"] is None else state["last_attempt"] + period

        selected, wait_ms, interval = [], [], None
        smoking_diagnostic = {}
        if self.smoking_clock.due(stamp):
            # One clock governs every person, so newly appearing IDs cannot
            # accidentally cause additional GPU batches between global ticks.
            self.smoking_clock.add(stamp, None)
            ordered = sorted(observations, key=lambda p: (eligible_since(p["track_id"]), p["track_id"]))
            selected = ordered[:smoke_cfg["max_rois_per_tick"]]
            if selected:
                wait_ms = [max(0, stamp - eligible_since(p["track_id"])) / 1e6 for p in selected]
                crops = call("smoking_crops", self.backend.crops, as_rgb(),
                             [dict(p, raw_id=p["track_id"]) for p in selected])
                observations_by_id = {p["track_id"]: dict(p, keypoints_stamp_ns=stamp) for p in selected}
                results = call("smoking", self.backend.smoking, crops, observations_by_id, bgr.shape) if crops else {}
                smoking_diagnostic = dict(getattr(self.backend, "last_smoking_diagnostic", {})) if crops else {}
                if crops and smoking_diagnostic.get("inferred_people", len(crops)):
                    interval = (stamp - self.last_smoking_forward) / NS if self.last_smoking_forward is not None else None
                    self.last_smoking_forward = stamp
                    self.smoking_forward_count += 1
                for p, delay in zip(selected, wait_ms):
                    track_id = p["track_id"]
                    state = self.people[track_id]
                    state["last_attempt"] = stamp
                    if track_id not in results:
                        if self.profile_smoking is not None:
                            self.profile_smoking.forget(track_id)
                            self.profile_strong.pop(track_id, None)
                        if self.smoking_temporal is not None:
                            self.smoking_temporal.forget(track_id)
                        if self.smoking_hand is not None:
                            self.smoking_hand.forget(track_id)
                        state["smoking"] = dict(empty_action("unknown", "invalid_person_crop", stamp))
                        continue
                    raw = results[track_id]
                    if self.smoking_hand is not None:
                        raw = self.smoking_hand.update(track_id, raw, stamp,
                                                      association=p.get("association", "associated"))
                    refined = raw.get('evidence', {}).get('profile_refinement', {}).get('result')
                    if self.profile_smoking is not None:
                        if refined is None:
                            self.profile_smoking.forget(track_id); self.profile_strong.pop(track_id, None)
                        else:
                            association = p.get('association', 'associated')
                            if association != 'associated' or not refined.get('evidence', {}).get('head_observable'):
                                self.profile_strong.pop(track_id, None)
                            strong_threshold = smoke_cfg.get('profile_refinement', {}).get('threshold', smoke_cfg['threshold'])
                            if association in ('associated', 'new_track') and refined['label'] == 'smoking' and refined['score'] >= strong_threshold:
                                self.profile_strong[track_id] = stamp
                            if track_id not in self.profile_strong:
                                refined = dict(refined, label=None, score=None, state='unknown', reason='no_strong_profile_seed')
                            # One missed model tick does not erase genuine prior
                            # hits, but it emits NO positive or refreshed timestamp.
                            confirmed = (self.profile_smoking.update(track_id, refined, stamp, association=association)
                                         if refined['label'] == 'smoking' or track_id not in self.profile_strong
                                         else {'label': None})
                            raw['evidence']['profile_confirmation'] = dict(confirmed.get('temporal', {}),
                                strong_detection_stamp_ns=self.profile_strong.get(track_id))
                            if raw['label'] != 'smoking' and confirmed['label'] == 'smoking':
                                raw = dict(raw, label='smoking', state='valid', score=confirmed['score'],
                                           score_kind='temporally_associated_detection_score', reason='profile_head_refinement')
                    score = raw.get("score")
                    raw_state = raw.get("state", "valid")
                    if ((raw_state == "valid" and raw["label"] not in ("smoking", "other"))
                            or (raw_state == "unknown" and (raw["label"] is not None or score is not None))
                            or raw_state not in ("valid", "unknown") or (score is not None and
                            (not math.isfinite(score) or not 0 <= score <= 1))):
                        raise RuntimeError("Smoking predictor returned invalid result")
                    if self.smoking_temporal is not None:
                        raw = self.smoking_temporal.update(track_id, raw, stamp,
                            association=p.get("association", "associated"))
                        state["smoking"] = dict(raw, scheduling_wait_ms=delay)
                    else:
                        state["smoking"] = dict(raw, state=raw_state, updated=raw_state == "valid", prediction_stamp_ns=stamp,
                                                scheduling_wait_ms=delay)

        # Sampling continues while the one background fight request is busy.
        fight = self._fight_tick(stamp, as_rgb, call, timing)

        people = []
        for p in observations:
            track_id = p["track_id"]
            fall = call("fall_rules", self.rules.update, track_id, p["keypoints"], p["bbox"], stamp,
                        bgr.shape, p.get("association_evidence"),
                        {"track_id": track_id, "source_stamp_ns": p.get("observation_stamp_ns"),
                         "rotation_cw": p.get("observation_rotation_cw", 0),
                         "association": p.get("association")})
            smoke = current(self.people[track_id]["smoking"], stamp, smoke_cfg)
            people.append({"track_id": track_id, "bbox": p["bbox"], "score": p["score"],
                           "keypoints": np.asarray(p["keypoints"]).tolist(), "keypoints_stamp_ns": stamp,
                           "observation_rotation_cw": p.get("observation_rotation_cw", 0),
                           "pose_observation_stamp_ns": p.get("observation_stamp_ns", stamp),
                           "association": p.get("association"),
                           "association_evidence": p.get("association_evidence"),
                           "actions": {"fall": fall, "smoking": smoke}})
        actions = [fight] + [a for p in people for a in p["actions"].values()]
        status = "valid" if all(a["state"] == "valid" for a in actions) else "warming"
        if any(a["state"] == "unknown" for a in actions): status = "partial"
        timing["total"] = (time.perf_counter() - started) * 1000
        return {"type": "result", "protocol_version": 1, "profile": self.cfg["profile"],
                "request_id": header["request_id"], "stamp_ns": stamp, "source_epoch": epoch,
                "status": status, "persons": people, "scene": {"fight": fight}, "timing_ms": timing,
                "pose_observation": getattr(self.backend, "last_pose_observation", None),
                "generation": self.generation, "reset_reason": reset_reason, "models": self.backend.identities,
                "scheduling": {"fight": {**self.fight_stats,
                    "mode": "async" if self.cfg["fight"].get("async_inference", False) else "sync",
                    "pending": self.fight_pending is not None,
                    "pending_source_stamp_ns": self.fight_pending["stamp"] if self.fight_pending else None,
                    "window": self.fight_window.info()}, "smoking": {"target_global_hz": smoke_cfg["sample_hz"],
                    "max_rois_per_tick": smoke_cfg["max_rois_per_tick"], "selected_ids": [p["track_id"] for p in selected],
                    "visible_people": len(people), "forward_count": self.smoking_forward_count,
                    "current_tick": smoking_diagnostic,
                    "last_forward_interval_seconds": interval, "max_selected_wait_ms": max(wait_ms, default=0.),
                    "max_visible_wait_ms": max([max(0, stamp - eligible_since(i)) / 1e6 for i in seen], default=0.)}}}
