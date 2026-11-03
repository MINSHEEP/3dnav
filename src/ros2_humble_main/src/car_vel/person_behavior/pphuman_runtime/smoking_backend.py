"""PPYOLOE adapter from the supplied CPU release; no legacy MOT or extra pose model."""
import numpy as np
from .smoking_filter import classify_smoking, profile_head_roi


class SmokingAdapter:
    def crops(self, rgb, people):
        crops = {}
        for person in people:
            rect = np.asarray([0, person["score"], *person["bbox"]])
            crop, new_box, original_box = self.expand_crop(rgb, rect)
            if crop is not None and crop.size and original_box[2] > original_box[0] \
                    and original_box[3] > original_box[1]:
                crops[person["raw_id"]] = (crop, new_box, original_box)
        return crops

    def smoking(self, crops, observations=None, image_shape=None):
        """Keep detector boxes/scores for review, without changing threshold."""
        self.last_smoking_diagnostic = {"requested_people": len(crops), "inferred_people": 0,
                                        "skipped_unobservable": 0, "profile_forwards": 0}
        if not crops: return {}
        skipped = {}
        association = self.cfg["smoking"].get("association", {})
        if association.get("enabled") and self.cfg["smoking"].get("skip_unobservable", False):
            eligible = {}
            for track_id, crop_info in crops.items():
                person = (observations or {}).get(track_id)
                if person is None:
                    raise RuntimeError("Smoking association requires the same-frame person observation")
                action = classify_smoking(None, person["keypoints"], person["bbox"],
                    {**association, "threshold": self.cfg["smoking"]["threshold"],
                     "positive_class": self.cfg["smoking"]["positive_class"], "image_shape": image_shape,
                     "candidates_complete": False, "hand_to_mouth": self.cfg["smoking"].get("hand_to_mouth", {})})
                if action["evidence"]["head_observable"]:
                    eligible[track_id] = crop_info
                else:
                    # Every existing smoking path requires the same observable
                    # head. Preserve unknown; no model score or fresh detection.
                    action["evidence"]["detector_skipped"] = "face_unobservable"
                    skipped[track_id] = dict(action, reason="face_unobservable",
                        association_stamp_ns=person.get("keypoints_stamp_ns"), detections=[],
                        crop_bbox=list(crop_info[1]), person_bbox=list(person["bbox"]),
                        total_detection_count=None, diagnostics_truncated=False,
                        coordinate_system="original_image_xyxy_pixels; bbox_crop is crop-relative xyxy")
            crops = eligible
            self.last_smoking_diagnostic["skipped_unobservable"] = len(skipped)
        self.last_smoking_diagnostic["inferred_people"] = len(crops)
        if not crops:
            return skipped
        ids = list(crops)
        raw = self.smoking_model.predict_image([crops[i][0] for i in ids], visual=False)
        counts = np.asarray(raw["boxes_num"]).reshape(-1)
        boxes = np.asarray(raw["boxes"]).reshape(-1, 6)
        if len(counts) != len(ids) or sum(counts) != len(boxes) or not np.isfinite(boxes).all():
            raise RuntimeError("Invalid smoking detector result")
        result, start = dict(skipped), 0
        refinement = self.cfg['smoking'].get('profile_refinement', {})
        extra_calls = 0
        for track_id, count in zip(ids, counts):
            subset = boxes[start:start + int(count)]; start += int(count)
            crop, expanded, original = crops[track_id]
            if np.any((subset[:, 1] < 0) | (subset[:, 1] > 1)):
                raise RuntimeError("Invalid smoking detection probability")
            limit = self.cfg["smoking"].get("diagnostic_max_detections", 20)
            detections = []
            for row in subset:
                score = float(row[1])
                if not 0 <= score <= 1: raise RuntimeError("Invalid smoking detection probability")
                detections.append({"class": int(row[0]), "score": score, "bbox_crop": row[2:].tolist(),
                                   "bbox": (row[2:] + np.tile(np.asarray(expanded[:2]), 2)).tolist()})
            association = self.cfg["smoking"].get("association", {})
            person = (observations or {}).get(track_id)
            if association.get("enabled"):
                if person is None:
                    raise RuntimeError("Smoking association requires the same-frame person observation")
                filter_cfg = {**association, "threshold": self.cfg["smoking"]["threshold"],
                    "positive_class": self.cfg["smoking"]["positive_class"], "image_shape": image_shape,
                    "candidates_complete": True,
                    "hand_to_mouth": self.cfg["smoking"].get("hand_to_mouth", {})}
                # All raw detections enter the pure association function. Its
                # selected detection remains in evidence even below raw top-20.
                action = classify_smoking(detections, person["keypoints"], person["bbox"], filter_cfg)
                action["association_stamp_ns"] = person.get("keypoints_stamp_ns")
            else:
                positives = subset[(subset[:, 0] == self.cfg["smoking"]["positive_class"])
                                   & (subset[:, 1] >= self.cfg["smoking"]["threshold"])]
                action = {"label": "smoking" if len(positives) else "other", "state": "valid",
                                "score": float(positives[:, 1].max()) if len(positives) else None,
                                "score_kind": "detection_score" if len(positives) else "no_positive_detection"}
            if (association.get('enabled') and refinement.get('enabled', False)
                    and action['label'] != 'smoking'
                    and extra_calls < refinement.get('max_rois_per_tick', 2)):
                roi = profile_head_roi(person['keypoints'], action['evidence'], expanded,
                                       dict(refinement, keypoint_confidence=association.get('keypoint_confidence', .5)))
                if roi is not None:
                    x1, y1, x2, y2 = np.asarray(roi) - np.tile(np.asarray(expanded[:2]), 2)
                    refined = self.smoking_model.predict_image([crop[y1:y2, x1:x2].copy()], visual=False)
                    extra_calls += 1
                    self.last_smoking_diagnostic["profile_forwards"] = extra_calls
                    refined_boxes = np.asarray(refined['boxes']).reshape(-1, 6)
                    refined_counts = np.asarray(refined['boxes_num']).reshape(-1)
                    if (len(refined_counts) != 1 or refined_counts[0] != len(refined_boxes)
                            or not np.isfinite(refined_boxes).all()
                            or np.any((refined_boxes[:, 1] < 0) | (refined_boxes[:, 1] > 1))):
                        raise RuntimeError('Invalid profile smoking detector result')
                    candidates = [{'class': int(d[0]), 'score': float(d[1]),
                                   'bbox': (d[2:] + np.tile(roi[:2], 2)).tolist()}
                                  for d in refined_boxes]
                    checked = classify_smoking(candidates, person['keypoints'], person['bbox'],
                        dict(filter_cfg, threshold=refinement.get('continue_threshold', self.cfg['smoking']['threshold']), hand_to_mouth={}))
                    diagnostic = dict(roi=roi, threshold=refinement.get('continue_threshold', self.cfg['smoking']['threshold']),
                                      detections=candidates, result=dict(checked, evidence=dict(checked['evidence'])), extra_forwards=1)
                    # The runtime merges this only AFTER the original hand sequence
                    # has consumed the body result, preserving its causal history.
                    action['evidence']['profile_refinement'] = diagnostic
            retained = sorted(detections, key=lambda item: -item["score"])[:limit]
            result[track_id] = {**action,
                                "detections": retained, "crop_bbox": list(expanded),
                                "person_bbox": list(person["bbox"] if person is not None else original),
                                "total_detection_count": len(subset), "diagnostics_truncated": len(retained) < len(subset),
                                "coordinate_system": "original_image_xyxy_pixels; bbox_crop is crop-relative xyxy"}
        return result
