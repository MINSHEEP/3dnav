"""Bounded conservative box/motion/pose association for shared pose observations."""

from dataclasses import dataclass
import numpy as np


def iou(a, b):
    overlap = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]))
    intersection = float(np.prod(overlap))
    union = float(np.prod(np.maximum(0, a[2:] - a[:2])) + np.prod(np.maximum(0, b[2:] - b[:2])) - intersection)
    return intersection / union if union > 0 else 0.0


@dataclass
class Track:
    bbox: np.ndarray
    keypoints: np.ndarray
    stamp: int
    velocity: np.ndarray
    strong_observations: int = 0


class ObservationTracker:
    def __init__(self, config=None):
        self.cfg = {"max_gap_seconds": .3, "retention_seconds": .7, "ambiguity_margin": .08,
                    "min_iou": .15, "max_center_distance": .45,
                    "reassociation_max_gap_seconds": .6, "reassociation_min_iou": .5,
                    "reassociation_max_center_distance": .25,
                    "reassociation_max_pose_distance": .3, "reassociation_max_shape_distance": .25,
                    "reassociation_min_body_keypoints": 4, "reassociation_min_prior_observations": 3,
                    **(config or {})}
        self.next_id = 1
        self.reset()

    def reset(self):
        self.tracks = {}
        self.last_stamp = None
        self.retired_ids = []
        self.last_reassociation_events = []

    @staticmethod
    def _strong_pose(points):
        points = np.asarray(points, dtype=float)
        visible = np.flatnonzero(points[5:, 2] > .3) + 5
        return len(visible) >= 4 and np.linalg.norm(np.ptp(points[visible, :2], axis=0)) > 5

    def _reassociation(self, track_id, track, detection, stamp):
        dt = (stamp - track.stamp) / 1e9
        if not self.cfg["max_gap_seconds"] < dt <= self.cfg["reassociation_max_gap_seconds"]:
            return None
        if track.strong_observations < self.cfg["reassociation_min_prior_observations"]:
            return None
        box = np.asarray(detection["bbox"], dtype=float)
        points = np.asarray(detection["keypoints"], dtype=float)
        common = (track.keypoints[:, 2] > .3) & (points[:, 2] > .3)
        common[:5] = False  # A half-body face cluster is not body identity evidence.
        if common.sum() < self.cfg["reassociation_min_body_keypoints"]:
            return None
        diagonal = max(float(np.linalg.norm(track.bbox[2:] - track.bbox[:2])),
                       float(np.linalg.norm(box[2:] - box[:2])), 1)
        if min(np.linalg.norm(np.ptp(points[common, :2], axis=0)),
               np.linalg.norm(np.ptp(track.keypoints[common, :2], axis=0))) < .15 * diagonal:
            return None
        area_ratio = float(np.prod(box[2:] - box[:2]) / max(np.prod(track.bbox[2:] - track.bbox[:2]), 1))
        if not .5 <= area_ratio <= 2: return None
        # A falling person may stop on contact; bound velocity extrapolation.
        predicted = track.bbox + np.tile(track.velocity * min(dt, .15), 2)
        overlap = max(iou(track.bbox, box), iou(predicted, box))
        distance = min(np.linalg.norm((reference[:2] + reference[2:] - box[:2] - box[2:]) / 2)
                       for reference in (track.bbox, predicted)) / diagonal
        old_xy, new_xy = track.keypoints[common, :2], points[common, :2]
        pose_distance = float(np.median(np.linalg.norm(old_xy - new_xy, axis=1))) / diagonal
        shape_distance = float(np.median(np.linalg.norm((old_xy - old_xy.mean(0)) - (new_xy - new_xy.mean(0)), axis=1))) / diagonal
        if (overlap < self.cfg["reassociation_min_iou"] or distance > self.cfg["reassociation_max_center_distance"]
                or pose_distance > self.cfg["reassociation_max_pose_distance"]
                or shape_distance > self.cfg["reassociation_max_shape_distance"]):
            return None
        return {"kind": "bounded_reassociation", "track_id": track_id, "previous_stamp_ns": track.stamp,
                "stamp_ns": stamp, "gap_seconds": dt, "common_body_keypoints": np.flatnonzero(common).tolist(),
                "iou": overlap, "normalized_center_distance": float(distance),
                "normalized_pose_distance": pose_distance, "normalized_shape_distance": shape_distance}

    def _score(self, track, detection, stamp):
        dt = (stamp - track.stamp) / 1e9
        if dt <= 0 or dt > self.cfg["max_gap_seconds"]:
            return None
        box = np.asarray(detection["bbox"], dtype=float)
        predicted = track.bbox + np.tile(track.velocity * dt, 2)
        overlap = iou(predicted, box)
        diagonal = max(float(np.linalg.norm(track.bbox[2:] - track.bbox[:2])),
                       float(np.linalg.norm(box[2:] - box[:2])), 1)
        distance = float(np.linalg.norm((predicted[:2] + predicted[2:] - box[:2] - box[2:]) / 2)) / diagonal
        if overlap < self.cfg["min_iou"] and distance > self.cfg["max_center_distance"]:
            return None
        new_kps = np.asarray(detection["keypoints"], dtype=float)
        common = (track.keypoints[:, 2] > .3) & (new_kps[:, 2] > .3)
        pose_similarity = 0.0
        if common.sum() >= 3:
            delta = np.tile(track.velocity * dt, (int(common.sum()), 1))
            pose_distance = float(np.median(np.linalg.norm(track.keypoints[common, :2] + delta - new_kps[common, :2], axis=1))) / diagonal
            if pose_distance > .7:
                return None
            pose_similarity = max(0, 1 - pose_distance)
        return .55 * overlap + .3 * max(0, 1 - distance) + .15 * pose_similarity

    def update(self, detections, stamp_ns):
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            self.reset()
        self.last_stamp = stamp_ns
        self.last_reassociation_events = []
        self.retired_ids = [track_id for track_id, track in self.tracks.items()
                            if stamp_ns - track.stamp > self.cfg["retention_seconds"] * 1e9]
        for track_id in self.retired_ids:
            del self.tracks[track_id]
        recovery = []
        for track_id, track in self.tracks.items():
            for index, detection in enumerate(detections):
                evidence = self._reassociation(track_id, track, detection, stamp_ns)
                if evidence is None: continue
                # Never take an observation away from another established track.
                competitors = [other_id for other_id, other in self.tracks.items() if other_id != track_id
                               and other.strong_observations > 0 and self._score(other, detection, stamp_ns) is not None]
                if not competitors: recovery.append((track_id, index, evidence))
        recovered, recovered_tracks = {}, set()
        for track_id, index, evidence in recovery:
            if sum(t == track_id for t, _, _ in recovery) != 1 or sum(i == index for _, i, _ in recovery) != 1:
                continue
            recovered[index] = (track_id, evidence); recovered_tracks.add(track_id)
        weak_retired = set()
        for index, (track_id, evidence) in recovered.items():
            # A one-joint placeholder has no established identity/history. Drop
            # it only after a unique mature track passed every recovery gate.
            provisional = {i for i, t in self.tracks.items() if i != track_id and t.strong_observations == 0
                           and iou(t.bbox, np.asarray(detections[index]["bbox"])) >= self.cfg["reassociation_min_iou"]}
            weak_retired.update(provisional)
            evidence["retired_provisional_ids"] = sorted(provisional)
            self.last_reassociation_events.append(evidence)
        for track_id in weak_retired:
            del self.tracks[track_id]
        self.retired_ids.extend(sorted(weak_retired))
        candidates = []
        for track_id, track in self.tracks.items():
            if track_id in recovered_tracks: continue
            for index, detection in enumerate(detections):
                if index in recovered: continue
                score = self._score(track, detection, stamp_ns)
                if score is not None:
                    candidates.append((score, track_id, index))
        by_track, by_detection = {}, {}
        for score, track_id, index in candidates:
            by_track.setdefault(track_id, []).append(score)
            by_detection.setdefault(index, []).append(score)
        ambiguous_tracks = {key for key, scores in by_track.items() if len(scores) > 1 and
                            sorted(scores, reverse=True)[0] - sorted(scores, reverse=True)[1] < self.cfg["ambiguity_margin"]}
        ambiguous_detections = {key for key, scores in by_detection.items() if len(scores) > 1 and
                                sorted(scores, reverse=True)[0] - sorted(scores, reverse=True)[1] < self.cfg["ambiguity_margin"]}
        # Retire disputed histories, otherwise these unmatched ghosts compete
        # with each new identity forever and prevent association from recovering.
        disputed = ambiguous_tracks | {track_id for _, track_id, index in candidates if index in ambiguous_detections}
        for track_id in disputed:
            del self.tracks[track_id]
        self.retired_ids.extend(sorted(disputed))
        matched = {index: pair[0] for index, pair in recovered.items()}
        used = set(recovered_tracks)
        for score, track_id, index in sorted(candidates, reverse=True):
            if track_id in disputed or index in ambiguous_detections or track_id in used or index in matched:
                continue
            matched[index] = track_id
            used.add(track_id)
        output = []
        for index, detection in enumerate(detections):
            track_id = matched.get(index)
            box = np.asarray(detection["bbox"], dtype=float)
            keypoints = np.asarray(detection["keypoints"], dtype=float)
            velocity = np.zeros(2)
            strong_observations = int(self._strong_pose(keypoints))
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
                reason = "ambiguous_new_track" if index in ambiguous_detections else "new_track"
            else:
                old = self.tracks[track_id]
                dt = (stamp_ns - old.stamp) / 1e9
                measured = ((box[:2] + box[2:]) - (old.bbox[:2] + old.bbox[2:])) / (2 * dt)
                velocity = .5 * old.velocity + .5 * measured
                strong_observations += old.strong_observations
                reason = "reassociated" if index in recovered else "associated"
            self.tracks[track_id] = Track(box.copy(), keypoints.copy(), stamp_ns, velocity, strong_observations)
            output.append(dict(detection, track_id=track_id, association=reason,
                               **({"association_evidence": recovered[index][1]} if index in recovered else {})))
        return output
