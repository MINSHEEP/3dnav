"""Same-frame cigarette association and evidence for a bounded hand sequence.

All coordinates are ORIGINAL-IMAGE pixels. keypoints are COCO17 (x,y,score).
Pass every detector candidate before any threshold/top-k diagnostic truncation.
The caller supplies keypoints/person box from the SAME inference frame and adds
timestamps, caching/TTL and updated fields afterwards. No temporal claim here.

COCO has no mouth joint: this is a conservative estimated mouth neighbourhood,
not a mouth detector. Scores are never multiplied by a geometry confidence.

The fixed core is +/-0.20 head scales laterally
and +/-0.15 vertically. Existing score/class/person/face prerequisites still
apply. If all formerly accepted boxes fail contact, return unknown with no
label or score. Do not interpret that as evidence of a negative action.
"""
from collections import Counter
import math

import numpy as np


CORE_HALF_WIDTH_HEADS = .20
CORE_HALF_HEIGHT_HEADS = .15


def contact_evidence(box, evidence, half_width=CORE_HALF_WIDTH_HEADS,
                     half_height=CORE_HALF_HEIGHT_HEADS):
    if not evidence.get('head_observable'):
        return {'available':False, 'reason':'baseline_head_unobservable'}
    center=evidence.get('mouth_center');poly=evidence.get('mouth_roi_polygon')
    scale=evidence.get('head_scale_pixels')
    try:
        values=list(box)+list(center)+[scale]+[v for p in poly for v in p]+[half_width,half_height]
        if len(box)!=4 or len(center)!=2 or len(poly)!=4 or any(len(p)!=2 for p in poly):raise ValueError()
        if not all(math.isfinite(v) for v in values) or scale<=0 or half_width<=0 or half_height<=0:raise ValueError()
        x1,y1,x2,y2=box
        if not x2>x1 or not y2>y1:raise ValueError()
        lateral=[poly[1][i]-poly[0][i] for i in (0,1)]
        down=[poly[3][i]-poly[0][i] for i in (0,1)]
        ln=math.hypot(*lateral);dn=math.hypot(*down)
        if ln<1e-8 or dn<1e-8:raise ValueError()
        lateral=[v/ln for v in lateral];down=[v/dn for v in down]
        if abs(sum(a*b for a,b in zip(lateral,down)))>1e-6:raise ValueError()
    except (TypeError,ValueError,IndexError):
        return {'available':False,'reason':'invalid_estimated_mouth_geometry'}
    core=[[center[i]+su*half_width*scale*lateral[i]+sv*half_height*scale*down[i] for i in (0,1)]
          for su,sv in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    rectangle=[[x1,y1],[x2,y1],[x2,y2],[x1,y2]]
    # Separating-axis theorem for the original-image AABB and rotated mouth
    # rectangle: all four independent edge-normal directions must overlap.
    axes=[lateral,down,[1.,0.],[0.,1.]]
    separation=[]
    for axis in axes:
        project=lambda p:sum(a*b for a,b in zip(p,axis))
        a=[project(p) for p in rectangle];b=[project(p) for p in core]
        separation.append(max(min(a)-max(b),min(b)-max(a),0.)/scale)
    return {'available':True,'intersects':not any(g>1e-9 for g in separation),
        'core_polygon':core,'core_half_size_heads':[half_width,half_height],
        'separating_axis_gaps_heads':separation,
        'semantic_limit':'estimated mouth contact only, not tobacco identity'}


DEFAULTS = {
    "threshold": .5, "positive_class": 0, "keypoint_confidence": .5,
    "minimum_head_pixels": 8., "mouth_offset_heads": .25,
    "mouth_half_width_heads": .55, "mouth_half_height_heads": .30,
    "minimum_person_overlap": .25, "maximum_candidate_side_heads": 1.5,
    "max_evidence_candidates": 8, "candidates_complete": True,
}

NEAR_FACE_DEFAULTS = {
    "enabled": False, "start_threshold": .7, "continue_threshold": .35,
    "mouth_half_width_heads": 1.25, "mouth_half_height_heads": .5,
    "maximum_wrist_mouth_distance_heads": 2., "max_candidate_shift_heads": .75,
    "occlusion_grace_seconds": 1.,
}


def profile_head_roi(keypoints, evidence, crop_box, config):
    """Bounded same-frame side-face crop; no extrapolated head or person cache."""
    if not evidence.get('head_observable'):
        return None
    points = np.asarray(keypoints, dtype=float)
    scale = evidence['head_scale_pixels']
    if scale > config.get('maximum_head_pixels', 64.):
        return None
    confidence = config.get('keypoint_confidence', .5)
    if all(points[[0, 1, 2], 2] >= confidence):
        axis = points[1, :2] - points[2, :2]
        length2 = float(np.dot(axis, axis))
        if length2 < 4:
            return None
        ratio = abs(float(np.dot(points[0, :2] - points[[1, 2], :2].mean(0), axis))) / length2
        if ratio < config.get('minimum_profile_ratio', .35):
            return None
    elif evidence.get('head_method') != 'profile_face_and_shoulders':
        return None
    center = np.asarray(evidence['mouth_center'])
    radius = scale * config.get('crop_radius_heads', 2.)
    box = np.r_[np.floor(center-radius), np.ceil(center+radius)].astype(int)
    box[:2] = np.maximum(box[:2], crop_box[:2])
    box[2:] = np.minimum(box[2:], crop_box[2:])
    if np.min(box[2:]-box[:2]) < 16:
        return None
    return box.tolist()


def visible_wrists(keypoints, person, confidence):
    """COCO wrists only; a confident extrapolation outside the person is invalid."""
    points = np.asarray(keypoints, dtype=float)
    if points.shape != (17, 3):
        return {}
    return {i: points[i, :2] for i in (9, 10)
            if np.isfinite(points[i]).all() and confidence <= points[i, 2] <= 1
            and (points[i, :2] >= person[:2]).all()
            and (points[i, :2] <= person[2:]).all()}


def hand_candidates(parsed, keypoints, person, head, config):
    """Use ALL model boxes before diagnostic truncation; holding is not smoking."""
    cfg = config.get("hand_to_mouth", {})
    if head is None or not cfg.get("enabled", False) or not config["candidates_complete"]:
        return []
    center, lateral, down, scale, _, _ = head
    near_cfg = dict(NEAR_FACE_DEFAULTS, **cfg.get("near_face", {}))
    threshold = min(config["threshold"], near_cfg["continue_threshold"]) if near_cfg["enabled"] else config["threshold"]
    wrists = visible_wrists(keypoints, person, config["keypoint_confidence"])
    best = {}
    for index, cls, score, box in parsed:
        if cls != config["positive_class"] or score < threshold:
            continue
        overlap = np.maximum(0, np.minimum(box[2:], person[2:]) - np.maximum(box[:2], person[:2]))
        if (np.prod(overlap) / np.prod(box[2:] - box[:2]) < config["minimum_person_overlap"]
                or np.max(box[2:] - box[:2]) > config["maximum_candidate_side_heads"] * scale):
            continue
        # Distance to the box, not its centre: COCO locates the wrist, not fingers.
        distances = sorted((float(np.linalg.norm(point - np.clip(point, box[:2], box[2:]))) / scale, i)
                           for i, point in wrists.items())
        if not distances or distances[0][0] > cfg.get("max_wrist_distance_heads", 1.5):
            continue
        distance, wrist = distances[0]
        if len(distances) > 1 and distances[1][0] - distance < cfg.get("wrist_ambiguity_margin_heads", .25):
            continue
        mouth_distance = float(np.linalg.norm(wrists[wrist] - center)) / scale
        offset = (box[:2] + box[2:]) / 2 - center
        u, v = float(np.dot(offset, lateral)) / scale, float(np.dot(offset, down)) / scale
        item = dict(index=index, **{"class": cls}, score=score, bbox=box.tolist(),
                    wrist_index=wrist, wrist_distance_heads=distance,
                    wrist_mouth_distance_heads=mouth_distance, mouth_offset_heads=[u, v])
        modes = []
        if score >= config["threshold"] and mouth_distance >= cfg.get("minimum_far_distance_heads", 2.):
            modes.append("approach")
        if (near_cfg["enabled"] and score >= near_cfg["continue_threshold"]
                and mouth_distance <= near_cfg["maximum_wrist_mouth_distance_heads"]
                and abs(u) <= near_cfg["mouth_half_width_heads"]
                and abs(v) <= near_cfg["mouth_half_height_heads"]):
            modes.append("near_face")
        for mode in modes:
            key = (mode, wrist)
            if key not in best or score > best[key]["score"]:
                best[key] = dict(item, mode=mode)
    return list(best.values())


def _box(value):
    box = np.asarray(value, dtype=float)
    if box.shape != (4,) or not np.isfinite(box).all() or np.any(box[2:] <= box[:2]):
        raise ValueError("Expected a finite positive-area original-image xyxy box")
    return box


def _head(keypoints, person, cfg):
    evidence = {"visible_face_joints": [], "head_observable": False}
    try:
        points = np.asarray(keypoints, dtype=float)
    except (TypeError, ValueError):
        return None, dict(evidence, head_reason="invalid_keypoints")
    if points.shape != (17, 3):
        return None, dict(evidence, head_reason="invalid_keypoints")
    visible = (np.isfinite(points).all(axis=1) & (points[:, 2] >= cfg["keypoint_confidence"])
               & (points[:, 2] <= 1))
    # Off-person / off-image extrapolated joints must not become visible merely
    # because a detector assigned confidence to them. The person box is clipped.
    visible &= ((points[:, :2] >= person[:2]).all(axis=1)
                & (points[:, :2] <= person[2:]).all(axis=1))
    if cfg.get("image_shape") is not None:
        height, width = cfg["image_shape"][:2]
        visible &= ((points[:, :2] >= 0).all(axis=1)
                    & (points[:, 0] < width) & (points[:, 1] < height))
    evidence["visible_face_joints"] = np.flatnonzero(visible[:5]).tolist()
    if not visible[0] or sum(visible[:5]) < 2:
        return None, dict(evidence, head_reason="insufficient_face_keypoints")
    nose = points[0, :2]
    both_eyes, shoulders = all(visible[[1, 2]]), all(visible[[5, 6]])
    scales = []
    if both_eyes:
        eye_axis = points[1, :2] - points[2, :2]
        eye_span = float(np.linalg.norm(eye_axis))
        if eye_span < 2:
            return None, dict(evidence, head_reason="degenerate_face_geometry")
        down = np.asarray([-eye_axis[1], eye_axis[0]]) / eye_span
        toward_nose = nose - points[[1, 2], :2].mean(axis=0)
        orientation = float(np.dot(down, toward_nose))
        if abs(orientation) < 1 and shoulders:
            orientation = float(np.dot(down, points[[5, 6], :2].mean(axis=0) - nose))
        if abs(orientation) < 1:
            return None, dict(evidence, head_reason="ambiguous_head_orientation")
        if orientation < 0:
            down = -down
        scales.append(eye_span * 2.5)
        method = "nose_and_both_eyes"
    elif shoulders and any(visible[[1, 2]]):
        down = points[[5, 6], :2].mean(axis=0) - nose
        length = float(np.linalg.norm(down))
        if length < 4:
            return None, dict(evidence, head_reason="degenerate_head_shoulder_geometry")
        down /= length
        scales.extend(float(np.linalg.norm(nose - points[i, :2])) * 2.4
                      for i in (1, 2) if visible[i])
        method = "profile_face_and_shoulders"
    else:
        return None, dict(evidence, head_reason="insufficient_head_orientation")
    if all(visible[[3, 4]]):
        scales.append(float(np.linalg.norm(points[3, :2] - points[4, :2])))
    elif not both_eyes:
        scales.extend(float(np.linalg.norm(nose - points[i, :2])) * 1.1
                      for i in (3, 4) if visible[i])
    scale = max(scales)
    if scale < cfg["minimum_head_pixels"]:
        return None, dict(evidence, head_reason="head_too_small")
    if scale > float(np.linalg.norm(person[2:] - person[:2])) * .8:
        return None, dict(evidence, head_reason="inconsistent_face_scale")
    lateral = np.asarray([down[1], -down[0]])
    center = nose + cfg["mouth_offset_heads"] * scale * down
    half_w, half_h = cfg["mouth_half_width_heads"] * scale, cfg["mouth_half_height_heads"] * scale
    polygon = [center + sx * half_w * lateral + sy * half_h * down
               for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
    evidence.update(head_observable=True, head_reason="observable", head_method=method,
                    head_scale_pixels=float(scale), mouth_center=center.tolist(),
                    mouth_roi_polygon=[p.tolist() for p in polygon],
                    mouth_half_size_pixels=[float(half_w), float(half_h)],
                    mouth_roi_is_estimated=True)
    return (center, lateral, down, scale, half_w, half_h), evidence


def classify_smoking(detections, keypoints, person_bbox, cfg=None):
    """Return action/evidence; no imports of a model, ROS, OpenCV or GPU runtime.

    detections: iterable of {class:int, score:float, bbox:[x1,y1,x2,y2]}.
    None means detector not run, [] means a completed empty result. cfg may set
    threshold=.5/.6, image_shape=[h,w], or max_evidence_candidates=None for a full
    audit. Diagnostic limits NEVER limit the candidates used in classification.
    candidates_complete=False makes a truncated observation unknown.
    """
    config = dict(DEFAULTS, **(cfg or {}))
    for name in ("threshold", "keypoint_confidence", "minimum_person_overlap"):
        if not 0 <= config[name] <= 1:
            raise ValueError(name + " must be in [0,1]")
    for name in ("minimum_head_pixels", "mouth_half_width_heads", "mouth_half_height_heads", "maximum_candidate_side_heads"):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(name + " must be positive and finite")
    if not math.isfinite(config["mouth_offset_heads"]):
        raise ValueError("mouth_offset_heads must be finite")
    limit = config["max_evidence_candidates"]
    if limit is not None and (not isinstance(limit, int) or limit < 1):
        raise ValueError("max_evidence_candidates must be a positive integer or None")
    person = _box(person_bbox)
    head, evidence = _head(keypoints, person, config)
    evidence.update(threshold=float(config["threshold"]), person_bbox=person.tolist(),
                    positive_class=config["positive_class"], candidates_complete=bool(config["candidates_complete"]))
    parsed = []
    for index, candidate in enumerate(detections if detections is not None else []):
        score = float(candidate["score"])
        if not math.isfinite(score) or not 0 <= score <= 1:
            raise ValueError("Smoking model score must be finite and in [0,1]")
        parsed.append((index, int(candidate["class"]), score, _box(candidate["bbox"])))
    positive_scores = [score for _, cls, score, _ in parsed if cls == config["positive_class"]]
    evidence.update(candidate_count=len(parsed), raw_max_score=max(positive_scores, default=None))
    evidence["hand_candidates"] = hand_candidates(parsed, keypoints, person, head, config)
    unknown = ("detector_not_observed" if detections is None else
               "incomplete_detector_candidates" if not config["candidates_complete"] else
               "face_unobservable" if head is None else None)
    decisions, accepted = [], []
    baseline_accepted_count = 0
    core_unavailable = False
    for index, cls, score, box in parsed:
        decision = {"index": index, "class": cls, "score": score, "bbox": box.tolist(), "accepted": False}
        if unknown:
            reason = unknown
        elif cls != config["positive_class"]:
            reason = "different_model_class"
        else:
            center, lateral, down, scale, half_w, half_h = head
            overlap = np.maximum(0, np.minimum(box[2:], person[2:]) - np.maximum(box[:2], person[:2]))
            overlap = float(np.prod(overlap) / np.prod(box[2:] - box[:2]))
            offset = (box[:2] + box[2:]) / 2 - center
            u, v = float(np.dot(offset, lateral)), float(np.dot(offset, down))
            decision.update(person_overlap_fraction=overlap, mouth_offset_heads=[u / scale, v / scale])
            if overlap < config["minimum_person_overlap"]:
                reason = "outside_person"
            elif np.max(box[2:] - box[:2]) > config["maximum_candidate_side_heads"] * scale:
                reason = "implausible_candidate_extent"
            elif abs(u) > half_w or abs(v) > half_h:
                reason = "outside_mouth_neighborhood"
            elif score < config["threshold"]:
                reason = "below_model_threshold"
            else:
                baseline_accepted_count += 1
                contact = contact_evidence(box.tolist(), evidence)
                decision["mouth_core_contact"] = contact
                if not contact["available"]:
                    reason = "estimated_mouth_core_unavailable"
                    core_unavailable = True
                elif not contact["intersects"]:
                    reason = "outside_estimated_mouth_core"
                else:
                    reason = "associated_with_estimated_mouth_core"
                    decision["accepted"] = True
                    accepted.append(decision)
        decision["reason"] = reason
        decisions.append(decision)
    best = max(accepted, key=lambda x: x["score"], default=None)
    # Failure of a new spatial prerequisite is insufficient evidence of a
    # negative action. Preserve uncertainty; do not fabricate a negative score.
    if unknown is None and baseline_accepted_count and best is None:
        unknown = ("estimated_mouth_core_unavailable" if core_unavailable else
                   "estimated_mouth_core_not_contacted")
    evidence.update(baseline_accepted_count=baseline_accepted_count,
        mouth_core_rejected_count=baseline_accepted_count - len(accepted),
        mouth_core_half_size_heads=[CORE_HALF_WIDTH_HEADS, CORE_HALF_HEIGHT_HEADS],
        mouth_core_policy="original_image_box_intersects_rotated_estimated_core",
        mouth_core_is_estimated=True)
    rejected = [x for x in decisions if not x["accepted"]]
    ordered = sorted(decisions, key=lambda x: (-x["score"], x["index"]))
    retained = ordered if limit is None else ordered[:limit]
    if best is not None and best not in retained:
        retained = retained[:-1] + [best]
    evidence.update(accepted_count=len(accepted), rejected_count=len(rejected),
        decision_counts=dict(Counter(x["reason"] for x in decisions)),
        selected_detection=best, highest_rejected=max(rejected, key=lambda x: x["score"], default=None),
        candidate_decisions=retained, diagnostics_truncated=len(retained) < len(decisions))
    if unknown:
        return {"label": None, "score": None, "score_kind": "unavailable", "state": "unknown",
                "reason": unknown, "evidence": evidence}
    return {"label": "smoking" if best is not None else "other",
            "score": best["score"] if best is not None else None,
            "score_kind": "detection_score" if best is not None else "no_associated_positive_detection",
            "state": "valid", "reason": "mouth_associated_candidate" if best is not None else "no_associated_positive_candidate",
            "evidence": evidence}
