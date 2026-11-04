"""Narrow candidate: native pose first, one same-frame rotated retry if empty.

Reuse the existing predictor/session; model/lease ownership stays in the worker.
Only timestamps persist: never cache a person, pose or action. A recently seen
reliable native person seeds searches at <=5 Hz. Following a successful retry,
actual rotated observations <=0.3 s apart permit one retry on each empty native
frame. Native success always returns the unchanged native list immediately.

Coverage limits: no search before a reliable native person has been observed;
no recovery of one missing person while any other person is natively detected.
"""
import math
import time

import numpy as np

NS = 1_000_000_000
DEFAULTS = {"search_hz": 5., "active_gap_seconds": .3, "recent_evidence_seconds": 1.,
            "native_seed_min_score": .5, "native_seed_min_keypoints": 8, "keypoint_confidence": .3}


def back_to_original(person, height, width):
    """Invert CCW90: boxes use edges, joints use centers; COCO IDs unchanged."""
    x1, y1, x2, y2 = person["bbox"]
    points = np.asarray(person["keypoints"], dtype=float).copy()
    x, y = points[:, 0].copy(), points[:, 1].copy()
    points[:, 0], points[:, 1] = width - 1 - y, x
    return dict(person, bbox=[width - y2, x1, width - y1, x2], keypoints=points)


class PoseRecovery:
    def __init__(self, predictor, cfg=None):
        self.predictor = predictor
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        for key in ("search_hz", "active_gap_seconds", "recent_evidence_seconds"):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key + " must be finite and positive")
        for key in ("native_seed_min_score", "keypoint_confidence"):
            if not 0 < self.cfg[key] <= 1:
                raise ValueError(key + " must be in (0,1]")
        n = self.cfg["native_seed_min_keypoints"]
        if type(n) is not int or not 1 <= n <= 17:
            raise ValueError("native_seed_min_keypoints must be in [1,17]")
        if (self.cfg["search_hz"] > 5 or self.cfg["active_gap_seconds"] > .3 or
                self.cfg["recent_evidence_seconds"] > 1):
            raise ValueError("Candidate budgets: at most 5 Hz search, 0.3 s active, 1 s recent evidence")
        self.search_period_ns = round(NS / self.cfg["search_hz"])
        self.reset()

    def reset(self):
        self.last_native_seed_ns = self.last_recovered_ns = None
        self.active = False
        self.next_search_ns = 0
        self.last_stamp = self.last_shape = None
        self.last_timing = {}
        self.last_diagnostic = {}

    def predict(self, bgr, stamp_ns):
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError("Expected nonnegative source timestamp in ns")
        if self.last_stamp is not None and (stamp_ns <= self.last_stamp or bgr.shape != self.last_shape):
            self.reset()
        self.last_stamp, self.last_shape = stamp_ns, bgr.shape
        started = time.perf_counter()
        timings, angles = {}, []
        previous_recovered = self.last_recovered_ns
        previous_seed = self.last_native_seed_ns

        def forward(image, angle):
            result = self.predictor.predict(image)
            angles.append(angle)
            for key, value in self.predictor.last_timing.items():
                timings[key] = timings.get(key, 0.) + value
            return result

        people = forward(bgr, 0)
        native_count, recovered_count = len(people), 0
        trigger = "native_nonempty"
        if people:
            self.active = False
            height, width = bgr.shape[:2]
            for person in people:
                k = np.asarray(person["keypoints"])
                visible = ((k[:, 2] >= self.cfg["keypoint_confidence"]) & (k[:, 0] >= 0) &
                           (k[:, 0] < width) & (k[:, 1] >= 0) & (k[:, 1] < height))
                if (person["score"] >= self.cfg["native_seed_min_score"] and
                        visible.sum() >= self.cfg["native_seed_min_keypoints"]):
                    self.last_native_seed_ns = stamp_ns
                    break
        else:
            active = (self.active and self.last_recovered_ns is not None and
                      stamp_ns - self.last_recovered_ns <= self.cfg["active_gap_seconds"] * NS)
            evidence = [x for x in (self.last_native_seed_ns, self.last_recovered_ns) if x is not None]
            recent = bool(evidence) and stamp_ns - max(evidence) <= self.cfg["recent_evidence_seconds"] * NS
            due = stamp_ns >= self.next_search_ns
            trigger = "active_recent_recovery" if active else ("recent_person_search" if recent and due else
                      "no_recent_person" if not recent else "search_cooldown")
            if active or (recent and due):
                import cv2
                self.next_search_ns = stamp_ns + self.search_period_ns
                before = time.perf_counter()
                rotated = cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
                timings["recovery_transform"] = (time.perf_counter() - before) * 1000
                detected = forward(rotated, 270)
                before = time.perf_counter()
                people = [dict(back_to_original(p, *bgr.shape[:2]), observation_rotation_cw=270,
                               observation_stamp_ns=stamp_ns) for p in detected]
                timings["recovery_coordinate_mapping"] = (time.perf_counter() - before) * 1000
                recovered_count = len(people)
                self.active = bool(people)
                if people:
                    self.last_recovered_ns = stamp_ns
            else:
                self.active = False
        self.last_timing = dict(timings, pose_recovery_total=(time.perf_counter() - started) * 1000)
        self.last_diagnostic = {"source_stamp_ns": stamp_ns, "recovery_enabled": True,
            "native_person_count": native_count, "recovered_person_count": recovered_count,
            "forward_rotations_cw": angles, "actual_forward_count": len(angles),
            "extra_forward_count": len(angles) - 1, "persons_cached": False,
            "trigger": trigger, "previous_native_seed_stamp_ns": previous_seed,
            "previous_recovered_stamp_ns": previous_recovered,
            "last_native_seed_stamp_ns": self.last_native_seed_ns,
            "last_recovered_stamp_ns": self.last_recovered_ns}
        return people
