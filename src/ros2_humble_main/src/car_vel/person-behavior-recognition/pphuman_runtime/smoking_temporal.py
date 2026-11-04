"""Independent CPU temporal confirmation after per-frame mouth association.

Call begin_frame once per source frame (including empty scenes), update only for
actual detector observations, and current on frames without a new observation.
Prediction timestamps, not display timestamps or updated flags, identify data.
No configuration, worker, model or IO side effects; standard library only.
"""
from collections import deque
from copy import deepcopy
import math

NS = 1_000_000_000
DEFAULTS = {"minimum_positive_observations": 2, "confirmation_window_seconds": 1.0,
            "minimum_confirmation_seconds": .25, "max_prediction_gap_seconds": .75,
            "result_ttl_seconds": 1.5, "track_gap_seconds": .3}


class SmokingTemporal:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        n = self.cfg["minimum_positive_observations"]
        if type(n) is not int or n < 2:
            raise ValueError("Confirmation needs at least two distinct observations")
        for key in DEFAULTS:
            if key != "minimum_positive_observations":
                if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                    raise ValueError(key + " must be positive and finite")
        if self.cfg["minimum_confirmation_seconds"] > self.cfg["confirmation_window_seconds"]:
            raise ValueError("Minimum confirmation span exceeds window")
        if self.cfg["max_prediction_gap_seconds"] > self.cfg["confirmation_window_seconds"]:
            raise ValueError("Prediction gap exceeds confirmation window")
        self.reset()

    def reset(self):
        self.tracks = {}
        self.frame_stamp = None
        self.epoch = None

    def forget(self, track_id):
        self.tracks.pop(track_id, None)

    def begin_frame(self, stamp_ns, visible_track_ids, epoch=None):
        """Use a source-epoch/generation token; missing IDs are forgotten now."""
        self._stamp(stamp_ns)
        if ((self.frame_stamp is not None and stamp_ns <= self.frame_stamp)
                or (self.frame_stamp is not None and epoch != self.epoch)):
            self.reset()
        seen = set(visible_track_ids)
        for track_id, track in list(self.tracks.items()):
            if (track_id not in seen or stamp_ns - track["last_seen"]
                    > self.cfg["track_gap_seconds"] * NS):
                self.forget(track_id)
            else:
                track["last_seen"] = stamp_ns
        self.frame_stamp, self.epoch = stamp_ns, epoch

    @staticmethod
    def _stamp(value):
        if type(value) is not int or value < 0:
            raise ValueError("Source timestamp must be a nonnegative integer nanosecond value")

    @staticmethod
    def _unknown(reason, stamp=None):
        return {"label": None, "score": None, "state": "unknown", "updated": False,
                "score_kind": "unavailable", "reason": reason, "prediction_stamp_ns": stamp}

    def current(self, track_id, stamp_ns):
        """Return cache without adding evidence or claiming a fresh prediction."""
        self._stamp(stamp_ns)
        track = self.tracks.get(track_id)
        if track is None or track["result"] is None:
            return self._unknown("no_smoking_observation")
        if stamp_ns < track["stamp"]:
            self.forget(track_id)
            return self._unknown("timestamp_nonincreasing")
        if stamp_ns - track["stamp"] > self.cfg["result_ttl_seconds"] * NS:
            track["positives"].clear(); track["confirmed_at"] = None
            track["result"] = self._unknown("expired", track["stamp"])
        return dict(deepcopy(track["result"]), updated=False)

    def update(self, track_id, action, prediction_stamp_ns, association="associated"):
        """Accept one genuine spatial-classifier observation, including unknown.

        Root adds prediction_stamp_ns when calling after backend.smoking. If the
        action already has a non-null timestamp it must match this argument.
        Duplicates never count again; older observations clear history and yield
        unknown. Unknown is processed even when its input updated flag is false.
        """
        self._stamp(prediction_stamp_ns)
        embedded = action.get("prediction_stamp_ns")
        if embedded is not None and embedded != prediction_stamp_ns:
            raise ValueError("Use the original prediction timestamp, not the display timestamp")
        state, label, score = action.get("state"), action.get("label"), action.get("score")
        if state not in ("valid", "unknown", "warming"):
            raise ValueError("Unexpected spatial action state")
        if state == "valid" and label not in ("smoking", "other"):
            raise ValueError("Valid spatial action must be smoking or other")
        if state != "valid" and (label is not None or score is not None):
            raise ValueError("Unknown/warming cannot carry a positive label or score")
        if label == "smoking" and (score is None or not math.isfinite(score) or not 0 <= score <= 1):
            raise ValueError("Positive detection needs its actual model score")
        old = self.tracks.get(track_id)
        # Timestamp deduplication precedes association reset: a repeated first
        # observation tagged new_track must not acquire a second vote.
        if old is not None and prediction_stamp_ns <= old["stamp"]:
            if prediction_stamp_ns == old["stamp"]:
                return self.current(track_id, max(prediction_stamp_ns, self.frame_stamp or 0))
            old["positives"].clear(); old["confirmed_at"] = None
            old["result"] = self._unknown("out_of_order_prediction", old["stamp"])
            return deepcopy(old["result"])
        if association != "associated":
            self.forget(track_id); old = None
        if old is None:
            old = {"stamp": None, "last_seen": self.frame_stamp if self.frame_stamp is not None else prediction_stamp_ns,
                   "positives": deque(), "confirmed_at": None, "result": None}
            self.tracks[track_id] = old
        raw_state, raw_label, raw_score = state, label, score
        unstable = association not in ("associated", "new_track")
        if unstable:
            state, label, score = "unknown", None, None
        if old["stamp"] is not None and prediction_stamp_ns - old["stamp"] > self.cfg["max_prediction_gap_seconds"] * NS:
            old["positives"].clear(); old["confirmed_at"] = None
        old["stamp"] = prediction_stamp_ns
        if self.frame_stamp is None:
            old["last_seen"] = prediction_stamp_ns
        result = deepcopy(action)
        result.update(label=label, state=state, score=score,
                      prediction_stamp_ns=prediction_stamp_ns, updated=state == "valid",
                      raw_spatial_label=raw_label, raw_spatial_state=raw_state, raw_model_score=raw_score,
                      raw_spatial_reason=action.get("reason"))
        if unstable:
            result.update(reason="unstable_track_association", score_kind="unavailable")
        if state != "valid" or label != "smoking":
            old["positives"].clear(); old["confirmed_at"] = None
        else:
            old["positives"].append(prediction_stamp_ns)
            while old["positives"] and prediction_stamp_ns - old["positives"][0] > self.cfg["confirmation_window_seconds"] * NS:
                old["positives"].popleft()
            span = (prediction_stamp_ns - old["positives"][0]) / NS
            enough = len(old["positives"]) >= self.cfg["minimum_positive_observations"] and span >= self.cfg["minimum_confirmation_seconds"]
            if enough:
                if old["confirmed_at"] is None: old["confirmed_at"] = prediction_stamp_ns
                # The score is the CURRENT model score, not an invented temporal probability.
                result.update(reason="repeated_smoking_confirmed", score_kind="detection_score")
            else:
                result.update(label=None, score=None, state="warming", updated=False,
                              score_kind="temporal_candidate", reason="awaiting_smoking_confirmation")
        result["temporal"] = {"positive_observations": len(old["positives"]),
            "required_observations": self.cfg["minimum_positive_observations"],
            "first_candidate_stamp_ns": old["positives"][0] if old["positives"] else None,
            "confirmed_stamp_ns": old["confirmed_at"],
            "confirmation_window_seconds": self.cfg["confirmation_window_seconds"],
            "minimum_confirmation_seconds": self.cfg["minimum_confirmation_seconds"],
            "max_prediction_gap_seconds": self.cfg["max_prediction_gap_seconds"]}
        old["result"] = result
        return deepcopy(result)
