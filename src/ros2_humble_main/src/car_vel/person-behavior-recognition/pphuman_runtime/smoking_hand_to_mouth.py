"""Causal cigarette-in-hand -> same wrist near mouth confirmation, no new model.

The original single-frame mouth decision remains available. An approach path
requires a cigarette away from the face followed by observed hand motion.
The optional near-face path instead needs a strong detection followed by a
distinct, spatially consistent observation. Memory never crosses track loss,
unobservable geometry, discontinuous timestamps or source epochs.
"""
import math

import numpy as np

from .smoking_filter import DEFAULTS as ASSOCIATION_DEFAULTS, NEAR_FACE_DEFAULTS, _head, visible_wrists

NS = 1_000_000_000
DEFAULTS = {
    "evidence_ttl_seconds": 3., "confirmation_seconds": .2,
    "minimum_far_distance_heads": 2., "maximum_near_distance_heads": 1.5,
    "minimum_approach_heads": 1., "max_wrist_distance_heads": 1.5,
    "wrist_ambiguity_margin_heads": .25,
}


class SmokingHandToMouth:
    def __init__(self, cfg, association=None, track_gap_seconds=.3):
        self.cfg = dict(DEFAULTS, **cfg)
        for key in DEFAULTS:
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError("hand_to_mouth." + key + " must be positive and finite")
        if self.cfg["confirmation_seconds"] >= self.cfg["evidence_ttl_seconds"]:
            raise ValueError("Hand confirmation must fit inside evidence lifetime")
        if self.cfg["maximum_near_distance_heads"] >= self.cfg["minimum_far_distance_heads"]:
            raise ValueError("Hand near distance must be smaller than far distance")
        self.near = dict(NEAR_FACE_DEFAULTS, **cfg.get("near_face", {}))
        for key in NEAR_FACE_DEFAULTS:
            if key != "enabled" and (not math.isfinite(self.near[key]) or self.near[key] <= 0):
                raise ValueError("near_face." + key + " must be positive and finite")
        if not self.near["continue_threshold"] < self.near["start_threshold"] <= 1:
            raise ValueError("Near-face continuation threshold must be below start threshold <= 1")
        if self.near["occlusion_grace_seconds"] > self.cfg["evidence_ttl_seconds"]:
            raise ValueError("Occlusion grace cannot exceed strong-evidence lifetime")
        self.association = dict(ASSOCIATION_DEFAULTS, **(association or {}))
        self.track_gap_ns = track_gap_seconds * NS
        self.tracks = {}
        self.stamp = self.epoch = None

    def forget(self, track_id):
        self.tracks.pop(track_id, None)

    def begin_frame(self, stamp, observations, epoch):
        """Observe motion on every pose frame, never by repeating cached predictions.

        Return IDs whose sequence was invalidated so the runtime also clears a
        cached positive derived from that sequence immediately.
        """
        before = set(self.tracks)
        if (self.stamp is not None and
                (stamp <= self.stamp or stamp - self.stamp > self.track_gap_ns or epoch != self.epoch)):
            self.tracks.clear()
        self.stamp, self.epoch = stamp, epoch
        people = {p["track_id"]: p for p in observations}
        for track_id, seed in list(self.tracks.items()):
            p = people.get(track_id)
            if (p is None or p.get("association", "associated") != "associated"
                    or p.get("observation_stamp_ns", stamp) != stamp
                    or stamp - seed["stamp_ns"] > self.cfg["evidence_ttl_seconds"] * NS):
                self.forget(track_id)
                continue
            person = np.asarray(p["bbox"], dtype=float)
            head, _ = _head(p["keypoints"], person, self.association)
            wrists = visible_wrists(p["keypoints"], person, self.association["keypoint_confidence"])
            wrist = seed["detection"]["wrist_index"]
            if head is None or wrist not in wrists:
                self.forget(track_id)
                continue
            distance = float(np.linalg.norm(wrists[wrist] - head[0])) / head[3]
            seed["current_distance_heads"] = distance
            if seed.get("mode") == "near_face":
                if (distance > self.near["maximum_wrist_mouth_distance_heads"]
                        or stamp - seed["last_detection_stamp_ns"] > self.near["occlusion_grace_seconds"] * NS):
                    self.forget(track_id)
                continue
            near = (distance <= self.cfg["maximum_near_distance_heads"]
                    and seed["detection"]["wrist_mouth_distance_heads"] - distance >= self.cfg["minimum_approach_heads"])
            if not near:
                seed["near_since_ns"] = None
                # A completed approach is one event; moving away ends it.
                if seed["confirmed"]:
                    self.forget(track_id)
            else:
                if seed["near_since_ns"] is None:
                    seed["near_since_ns"] = stamp
                seed["confirmed"] = stamp - seed["near_since_ns"] >= self.cfg["confirmation_seconds"] * NS
        return before - set(self.tracks)

    def _near_result(self, action, seed, stamp):
        evidence = dict(confirmed=True, mode="near_face", detection=seed["detection"],
                        detection_stamp_ns=seed["last_detection_stamp_ns"], pose_stamp_ns=stamp,
                        strong_detection=seed["strong_detection"], strong_detection_stamp_ns=seed["stamp_ns"],
                        evidence_age_seconds=(stamp-seed["last_detection_stamp_ns"])/NS,
                        strong_evidence_age_seconds=(stamp-seed["stamp_ns"])/NS,
                        observation_count=seed["observations"],
                        short_occlusion=stamp != seed["last_detection_stamp_ns"],
                        semantic_limit="repeated cigarette detections near this person's raised hand; not tobacco identity")
        return dict(action, label="smoking", state="valid", score=seed["detection"]["score"],
                    score_kind="temporally_associated_detection_score", reason="cigarette_near_face",
                    evidence=dict(action["evidence"], hand_to_mouth=evidence))

    def update(self, track_id, action, stamp, association="associated"):
        """Called ONLY for a fresh smoking-model observation after begin_frame."""
        if self.stamp != stamp:
            raise ValueError("Hand sequence requires same-frame pose before a fresh detector result")
        if action.get("prediction_stamp_ns", stamp) != stamp:
            raise ValueError("A cached detector result cannot seed a new hand sequence")
        evidence = action.get("evidence", {})
        if (association != "associated" or not evidence.get("head_observable")
                or not evidence.get("candidates_complete")):
            self.forget(track_id)
            return action
        seed = self.tracks.get(track_id)
        # Direct mouth contact retains its original score and decision semantics.
        if action.get("label") == "smoking":
            return action
        candidates = evidence.get("hand_candidates", [])
        near_candidates = [d for d in candidates if d.get("mode") == "near_face"] if self.near["enabled"] else []
        if seed and seed.get("mode") == "near_face":
            matches = [d for d in near_candidates
                       if d["wrist_index"] == seed["detection"]["wrist_index"]
                       and np.linalg.norm(np.asarray(d["mouth_offset_heads"])
                                          - seed["detection"]["mouth_offset_heads"]) <= self.near["max_candidate_shift_heads"]]
            if matches and stamp > seed["last_detection_stamp_ns"]:
                latest = max(matches, key=lambda d: d["score"])
                seed.update(detection=latest, last_detection_stamp_ns=stamp, observations=seed["observations"]+1)
                if latest["score"] >= self.near["start_threshold"]:
                    seed.update(strong_detection=latest, stamp_ns=stamp)
                seed["confirmed"] = (seed["observations"] >= 2 and
                                     stamp-seed["first_stamp_ns"] >= self.cfg["confirmation_seconds"]*NS)
            if seed["confirmed"]:
                return self._near_result(action, seed, stamp)
            if matches:
                return action
        if seed and seed.get("mode") != "near_face" and seed["confirmed"]:
            sequence = dict(confirmed=True, detection=seed["detection"],
                            detection_stamp_ns=seed["stamp_ns"], pose_stamp_ns=stamp,
                            evidence_age_seconds=(stamp - seed["stamp_ns"]) / NS,
                            near_since_ns=seed["near_since_ns"],
                            current_distance_heads=seed["current_distance_heads"],
                            semantic_limit="cigarette detector plus observed hand motion; not tobacco identity")
            return dict(action, label="smoking", state="valid", score=seed["detection"]["score"],
                        score_kind="temporally_associated_detection_score", reason="cigarette_hand_to_mouth",
                        evidence=dict(evidence, hand_to_mouth=sequence))
        strong = [d for d in near_candidates if d["score"] >= self.near["start_threshold"]]
        if strong:
            best = max(strong, key=lambda d: d["score"])
            self.tracks[track_id] = dict(mode="near_face", detection=best, strong_detection=best,
                stamp_ns=stamp, first_stamp_ns=stamp, last_detection_stamp_ns=stamp, observations=1, confirmed=False)
            return action
        candidates = [d for d in candidates if d.get("mode", "approach") == "approach"]
        if candidates:
            best = max(candidates, key=lambda d: d["score"])
            # One bounded seed per person. Never extend it using a cached display
            # result or weak detector score. A new real detection may replace it.
            self.tracks[track_id] = dict(detection=best, stamp_ns=stamp,
                                        near_since_ns=None, confirmed=False)
        return action
