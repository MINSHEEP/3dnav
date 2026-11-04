"""Observable posture and observed fall transitions; no learned probabilities.

Geometry is adapted from the local Apache-2.0 zhahoi rule port. A static lying
pose is explicitly distinct from an observed fall event. No model is imported.
"""

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

NS = 1_000_000_000
DEFAULTS = {"keypoint_confidence": .3, "min_visible_keypoints": 8,
            "min_torso_tilt": 30.0, "aspect_ratio_threshold": .9,
            "thigh_min_angle": 30.0, "thigh_max_angle": 70.0,
            "hip_angle_threshold": 70.0, "knee_angle_threshold": 30.0,
            "lying_confirm_seconds": .4, "recovery_confirm_seconds": .8,
            "max_observation_gap_seconds": .3, "max_visibility_gap_seconds": .4,
            "candidate_retention_seconds": .8, "reassociation_max_gap_seconds": .6,
            "minimum_lying_observations": 3, "history_seconds": 2.0,
            "minimum_baseline_seconds": .2, "minimum_drop_torso_lengths": .35,
            "minimum_rotation_degrees": 20.0, "minimum_height_loss_ratio": .2,
            "projection_max_transition_seconds": 2.0, "projection_max_support_seconds": 5.0,
            "projection_min_height_loss": .45, "projection_min_torso_loss": .35,
            "projection_min_drop_torso_lengths": .65, "projection_min_straight_angle": 150.0}


def angle(a, b):
    length = np.linalg.norm(a) * np.linalg.norm(b)
    return None if length <= 1e-6 else math.degrees(math.acos(float(np.clip(np.dot(a, b) / length, -1, 1))))


def horizontal_angle(vector):
    return math.degrees(math.atan2(abs(float(vector[1])), abs(float(vector[0]))))


def pose_features(keypoints, bbox, config=None, image_shape=None):
    cfg = {**DEFAULTS, **(config or {})}
    points = np.asarray(keypoints, dtype=float)
    invalid = {"observable": False, "posture": "unknown", "reason": "invalid_keypoints"}
    if points.shape != (17, 3) or not np.isfinite(points).all():
        return invalid
    visible = points[:, 2] > cfg["keypoint_confidence"]
    if image_shape is not None:
        h, w = image_shape[:2]
        visible &= (points[:, 0] >= 0) & (points[:, 0] < w) & (points[:, 1] >= 0) & (points[:, 1] < h)
    evidence = {"visible_joints": np.flatnonzero(visible).tolist()}
    if int(visible.sum()) < cfg["min_visible_keypoints"]:
        return dict(invalid, reason="insufficient_visible_joints", evidence=evidence)
    chains = [chain for chain in ([5, 11, 13, 15], [6, 12, 14, 16]) if visible[chain].all()]
    if not chains:
        return dict(invalid, reason="insufficient_lower_body", evidence=evidence)
    xy = points[:, :2]
    if not any(all(np.linalg.norm(xy[a] - xy[b]) > 1e-3 for a, b in zip(chain, chain[1:])) for chain in chains):
        return dict(invalid, reason="degenerate_body_chain", evidence=evidence)

    trusted = {i for chain in chains for i in chain}
    def center(indices):
        indices = [i for i in indices if i in trusted]
        return xy[indices].mean(axis=0)

    shoulder, hip, knee, ankle = [center(ids) for ids in ([5, 6], [11, 12], [13, 14], [15, 16])]
    torso = hip - shoulder
    torso_length = float(np.linalg.norm(torso))
    if torso_length <= 1e-3:
        return dict(invalid, reason="degenerate_torso", evidence=evidence)
    tilt = math.degrees(math.atan2(abs(float(torso[0])), float(torso[1])))
    span = np.ptp(xy[visible], axis=0)
    aspect = float(span[0] / max(span[1], 1e-6))
    hip_angle = angle(shoulder - hip, knee - hip)
    knee_angle = angle(hip - knee, ankle - knee)
    thigh_angles = [horizontal_angle(xy[chain[2]] - xy[chain[1]]) for chain in chains]
    shin_angles = [horizontal_angle(xy[chain[3]] - xy[chain[2]]) for chain in chains]
    chain_hip_angles = [angle(xy[s] - xy[h], xy[k] - xy[h]) for s, h, k, a in chains]
    chain_knee_angles = [angle(xy[h] - xy[k], xy[a] - xy[k]) for s, h, k, a in chains]
    # Midpoints of crossed legs can hide genuine joint flexion. Keep the
    # complete left/right chains separate when checking seated support.
    seated_flexion = any(thigh < 45 and ((ha is not None and 0 < ha <= 140)
                         or (ka is not None and 0 < ka <= 140))
                         for thigh, ha, ka in zip(thigh_angles, chain_hip_angles, chain_knee_angles))
    tilted = tilt >= cfg["min_torso_tilt"]
    # A chair sit/forward bend can have horizontal thighs and a tilted torso.
    # Bent knees + upright shins preserve that seated alternative.
    seated_legs = min(thigh_angles) < 40 and min(shin_angles) > 55 and knee_angle is not None and 45 <= knee_angle <= 140
    matches = []
    if shoulder[1] >= ankle[1]:
        matches.append("shoulder_below_ankle")
    if shoulder[1] > knee[1]:
        matches.append("shoulder_below_knee")
    if tilted and aspect > cfg["aspect_ratio_threshold"]:
        matches.append("wide_pose")
    if tilted and min(thigh_angles) < cfg["thigh_min_angle"] and max(thigh_angles) < cfg["thigh_max_angle"]:
        matches.append("horizontal_thighs")
    if tilted and hip_angle is not None and knee_angle is not None and hip_angle < cfg["hip_angle_threshold"] and knee_angle < cfg["knee_angle_threshold"]:
        matches.append("folded_body")
    lying = tilted and bool(matches) and not seated_legs
    if lying:
        posture, reason = "lying", matches[0]
    elif seated_legs or (tilt < cfg["min_torso_tilt"] and seated_flexion):
        posture, reason = "sitting", "supported_seated_pose"
    elif tilted:
        posture, reason = "bending", "tilt_without_supported_lying_geometry"
    elif min(thigh_angles) < 45:
        posture, reason = "unknown", "ambiguous_upright_projection"
    else:
        posture, reason = "standing", "upright_body"
    evidence.update(torso_tilt_degrees=tilt, torso_length_pixels=torso_length,
                    shoulder_center=shoulder.tolist(), hip_center=hip.tolist(),
                    aspect_ratio=aspect, hip_angle_degrees=hip_angle, knee_angle_degrees=knee_angle,
                    matched_rules=matches, trusted_body_chains=chains, seated_leg_geometry=seated_legs,
                    thigh_angles_degrees=thigh_angles, shin_angles_degrees=shin_angles,
                    chain_hip_angles_degrees=chain_hip_angles, chain_knee_angles_degrees=chain_knee_angles,
                    seated_chain_flexion=seated_flexion,
                    bbox_height_pixels=max(0.0, float(bbox[3] - bbox[1])))
    return {"observable": posture != "unknown", "posture": posture, "reason": reason, "evidence": evidence}


@dataclass
class PersonState:
    last_stamp: int = None
    last_observable_stamp: int = None
    history: deque = field(default_factory=deque)
    lying_since: int = None
    lying_observations: int = 0
    visibility_interruptions: int = 0
    recovery_since: int = None
    candidate_onset: int = None
    event_stamp: int = None
    transition_evidence: dict = field(default_factory=dict)
    reassociation_deadline: int = None
    reassociation_evidence: dict = field(default_factory=dict)
    rotation_continuation_deadline: int = None
    rotation_continuation_evidence: dict = field(default_factory=dict)
    projection_reference: dict = None
    projection_since: int = None


class FallRuleEngine:
    def __init__(self, config=None):
        self.cfg = {**DEFAULTS, **(config or {})}
        self.people = {}

    def reset(self):
        self.people.clear()

    def forget(self, track_id):
        self.people.pop(track_id, None)

    def expire(self, stamp_ns, active_ids):
        for track_id, state in list(self.people.items()):
            deadline = (state.last_stamp or 0) + self.cfg["max_observation_gap_seconds"] * NS
            if state.lying_since is not None and state.last_observable_stamp is not None:
                deadline = max(deadline, state.last_observable_stamp + self.cfg["candidate_retention_seconds"] * NS)
            if track_id not in active_ids and state.last_stamp is not None and stamp_ns > deadline:
                del self.people[track_id]

    def _projected_lying(self, features, state, stamp_ns):
        """Short-lived temporal evidence for depth-directed, foreshortened lying.

        Every promoted observation still requires complete current body geometry.
        A same-track baseline is never borrowed across ID/reset/visibility limits.
        """
        e = features.get("evidence", {})
        straight = self.cfg["projection_min_straight_angle"]
        if (not e.get("trusted_body_chains") or e.get("seated_leg_geometry")
                or (e.get("hip_angle_degrees") or 0) < straight
                or (e.get("knee_angle_degrees") or 0) < straight
                or any(ka is None or ka <= 140 for ka in e.get("chain_knee_angles_degrees", []))
                or not any(ha is not None and ka is not None and min(ha, ka) >= straight
                           for ha, ka in zip(e.get("chain_hip_angles_degrees", []), e.get("chain_knee_angles_degrees", [])))):
            state.projection_reference = state.projection_since = None
            return features
        refs = []
        if state.projection_reference is not None:
            if stamp_ns - state.projection_since <= self.cfg["projection_max_support_seconds"] * NS:
                refs.append(state.projection_reference)
            else:
                state.projection_reference = state.projection_since = None
        refs.extend({"stamp_ns": stamp, "evidence": old["evidence"]}
                    for stamp, old in state.history if old["posture"] == "standing"
                    and self.cfg["minimum_baseline_seconds"] * NS <= stamp_ns - stamp <= self.cfg["projection_max_transition_seconds"] * NS
                    and old["evidence"]["torso_tilt_degrees"] <= 20
                    and min(old["evidence"]["thigh_angles_degrees"]) >= 65
                    and (old["evidence"]["hip_angle_degrees"] or 0) >= straight
                    and (old["evidence"]["knee_angle_degrees"] or 0) >= straight)
        for reference in refs:
            old = reference["evidence"]
            height_loss = 1 - e["bbox_height_pixels"] / max(old["bbox_height_pixels"], 1)
            torso_loss = 1 - e["torso_length_pixels"] / max(old["torso_length_pixels"], 1)
            drop = (e["shoulder_center"][1] - old["shoulder_center"][1]) / max(old["torso_length_pixels"], 1)
            if (height_loss >= self.cfg["projection_min_height_loss"]
                    and torso_loss >= self.cfg["projection_min_torso_loss"]
                    and drop >= self.cfg["projection_min_drop_torso_lengths"]):
                if state.projection_since is None:
                    state.projection_since = stamp_ns
                state.projection_reference = reference
                proof = {"baseline_stamp_ns": reference["stamp_ns"], "support_since_ns": state.projection_since,
                         "support_deadline_ns": round(state.projection_since + self.cfg["projection_max_support_seconds"] * NS),
                         "height_loss_ratio": height_loss, "torso_loss_ratio": torso_loss,
                         "drop_torso_lengths": drop}
                return dict(features, observable=True, posture="lying", reason="observed_projected_body_lowering",
                            evidence=dict(e, projected_lying=proof))
        state.projection_reference = state.projection_since = None
        return features

    def update(self, track_id, keypoints, bbox, stamp_ns, image_shape=None, association=None,
               observation_context=None):
        features = pose_features(keypoints, bbox, self.cfg, image_shape)
        state = self.people.setdefault(track_id, PersonState())
        recovered = (association is not None and association.get("kind") == "bounded_reassociation"
            and association.get("track_id") == track_id and association.get("previous_stamp_ns") == state.last_stamp
            and association.get("stamp_ns") == stamp_ns and state.last_stamp is not None
            and 0 < stamp_ns - state.last_stamp <= self.cfg["reassociation_max_gap_seconds"] * NS
            and len(association.get("common_body_keypoints", [])) >= 4)
        if recovered and state.lying_since is not None and state.last_observable_stamp is not None:
            state.reassociation_deadline = round(state.last_observable_stamp + self.cfg["candidate_retention_seconds"] * NS)
            state.reassociation_evidence = dict(association)
        if state.last_stamp is not None and (stamp_ns <= state.last_stamp or
                (stamp_ns - state.last_stamp > self.cfg["max_observation_gap_seconds"] * NS and not recovered)):
            state = self.people[track_id] = PersonState()
        if (state.last_observable_stamp is not None
                and stamp_ns - state.last_observable_stamp > self.cfg["max_visibility_gap_seconds"] * NS
                and stamp_ns > max(state.reassociation_deadline or 0, state.rotation_continuation_deadline or 0)):
            state = self.people[track_id] = PersonState()
        # A fresh rotated observation can keep tracking continuous, removing
        # the old reassociation path. Preserve one bounded candidate deadline
        # only when this actual pose independently confirms prior lying.
        context = observation_context if isinstance(observation_context, dict) else {}
        if (state.rotation_continuation_deadline is None and state.lying_since is not None
                and state.lying_observations >= 2 and state.last_observable_stamp is not None
                and state.last_stamp is not None
                and 0 < stamp_ns - state.last_stamp <= self.cfg["max_observation_gap_seconds"] * NS
                and features["observable"] and features["posture"] == "lying"
                and context.get("track_id") == track_id and context.get("source_stamp_ns") == stamp_ns
                and context.get("rotation_cw") == 270
                and context.get("association") == "associated"):
            deadline = round(state.last_observable_stamp + self.cfg["candidate_retention_seconds"] * NS)
            state.rotation_continuation_deadline = deadline
            state.rotation_continuation_evidence = {
                "kind": "bounded_rotation_continuation", "track_id": track_id,
                "source_stamp_ns": stamp_ns, "rotation_cw": context["rotation_cw"],
                "prior_observable_stamp_ns": state.last_observable_stamp,
                "prior_lying_observations": state.lying_observations, "deadline_ns": deadline}
        features = self._projected_lying(features, state, stamp_ns)
        state.last_stamp = stamp_ns
        result = {"label": None, "score": None, "score_kind": "rule_evidence", "state": "unknown",
                  "updated": False, "prediction_stamp_ns": None, "posture": features["posture"],
                  "fall_event": "unobserved", "reason": features["reason"],
                  "event_semantics": "observed_descent_to_lying; intent_and_support_surface_unresolved",
                  "accidental_fall_confirmed": None,
                  "event_stamp_ns": state.event_stamp, "onset_stamp_ns": state.candidate_onset,
                  "evidence": features.get("evidence", {})}
        if not features["observable"]:
            if state.rotation_continuation_evidence:
                result["evidence"] = dict(result["evidence"], rotation_continuation=state.rotation_continuation_evidence)
            # Keep a bounded candidate through visibility flicker, but emit
            # unknown on this frame and never count it as a lying observation.
            # The last-observable deadline above also expires confirmed events.
            if state.lying_since is not None:
                state.visibility_interruptions += 1
            state.recovery_since = None
            return result
        state.last_observable_stamp = stamp_ns
        evidence = features["evidence"]
        while state.history and stamp_ns - state.history[0][0] > self.cfg["history_seconds"] * NS:
            state.history.popleft()
        if features["posture"] == "lying":
            state.recovery_since = None
            if state.lying_since is None:
                state.lying_since = stamp_ns
            state.lying_observations += 1
            anchors = [(stamp, old) for stamp, old in state.history
                       if old["posture"] in ("standing", "sitting")
                       and stamp_ns - stamp >= self.cfg["minimum_baseline_seconds"] * NS]
            if anchors and state.candidate_onset is None:
                anchor_stamp, anchor = anchors[0]
                old = anchor["evidence"]
                drop = (evidence["shoulder_center"][1] - old["shoulder_center"][1]) / old["torso_length_pixels"]
                rotation = evidence["torso_tilt_degrees"] - old["torso_tilt_degrees"]
                height_loss = 1 - evidence["bbox_height_pixels"] / max(old["bbox_height_pixels"], 1)
                if drop >= self.cfg["minimum_drop_torso_lengths"] and (rotation >= self.cfg["minimum_rotation_degrees"] or height_loss >= self.cfg["minimum_height_loss_ratio"]):
                    state.candidate_onset = state.lying_since
                    state.transition_evidence = {"baseline_stamp_ns": anchor_stamp,
                                                 "drop_torso_lengths": drop, "rotation_degrees": rotation,
                                                 "height_loss_ratio": height_loss}
            duration = (stamp_ns - state.lying_since) / NS
            result["evidence"] = dict(evidence, confirmation_seconds=duration,
                confirmation_mode="repeated_observable_lying_with_bounded_visibility_gap",
                confirmation_observations=state.lying_observations,
                unknown_observations=state.visibility_interruptions,
                reassociation=state.reassociation_evidence or None, **state.transition_evidence)
            if state.rotation_continuation_evidence:
                result["evidence"]["rotation_continuation"] = dict(state.rotation_continuation_evidence)
            if duration < self.cfg["lying_confirm_seconds"] or state.lying_observations < self.cfg["minimum_lying_observations"]:
                result.update(state="warming", fall_event="candidate" if state.candidate_onset is not None else "unobserved",
                              reason="confirming_lying_posture", onset_stamp_ns=state.candidate_onset)
            else:
                if state.candidate_onset is not None and state.event_stamp is None:
                    state.event_stamp = stamp_ns
                result.update(label="fall" if state.event_stamp is not None else "lying", state="valid",
                              updated=True, prediction_stamp_ns=stamp_ns,
                              fall_event="confirmed" if state.event_stamp is not None else "unobserved",
                              reason="observed_descent_to_lying" if state.event_stamp is not None else "lying_without_observed_fall",
                              event_stamp_ns=state.event_stamp, onset_stamp_ns=state.candidate_onset)
        else:
            state.lying_since = None
            state.reassociation_deadline = None
            state.reassociation_evidence.clear()
            state.rotation_continuation_deadline = None
            state.rotation_continuation_evidence.clear()
            state.lying_observations = state.visibility_interruptions = 0
            if state.event_stamp is None:
                state.candidate_onset = None
                state.transition_evidence.clear()
            # Publish the current non-lying posture immediately, retaining only
            # historical event identity during the recovery debounce.
            if state.recovery_since is None:
                state.recovery_since = stamp_ns
            if stamp_ns - state.recovery_since >= self.cfg["recovery_confirm_seconds"] * NS:
                state.candidate_onset = state.event_stamp = None
                state.transition_evidence.clear()
            result.update(label="other", state="valid", updated=True, prediction_stamp_ns=stamp_ns,
                          fall_event="none", event_stamp_ns=state.event_stamp, onset_stamp_ns=None)
        state.history.append((stamp_ns, features))
        return result
