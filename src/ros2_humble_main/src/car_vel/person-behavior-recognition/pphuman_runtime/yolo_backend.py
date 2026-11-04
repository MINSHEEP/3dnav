"""Shared COCO17 pose observations from YOLOv8 raw or YOLO26 end-to-end exports."""
import hashlib
import sys
import time
from pathlib import Path

import numpy as np

from .backend import PaddleBackend, model_identity, predictor_options
from .tracking import ObservationTracker
from .pose_recovery import PoseRecovery
from .smoking_filter import classify_smoking, profile_head_roi


def letterbox(bgr, size=640):
    import cv2
    h, w = bgr.shape[:2]
    ratio = min(size / h, size / w)
    nw, nh = round(w * ratio), round(h * ratio)
    left, top = round((size - nw) / 2 - .1), round((size - nh) / 2 - .1)
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[top:top + nh, left:left + nw] = resized
    tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
    # Match the exporter/reference scale_boxes versus scale_coords conventions:
    # boxes use rounded theoretical padding, keypoints use floating padding.
    px, py = (size - w * ratio) / 2, (size - h * ratio) / 2
    return tensor, {"gain": ratio, "left": round(px - .1), "top": round(py - .1),
                    "keypoint_pad_x": px, "keypoint_pad_y": py, "shape": (h, w)}


def decode_pose(raw, transform, cfg):
    raw = np.asarray(raw)
    end2end = cfg.get("output_format", "yolov8_raw") == "yolo26_end2end"
    if end2end:
        if raw.shape != (1, 300, 57):
            raise RuntimeError("Expected YOLO26 pose output [1,300,57], received " + str(raw.shape))
        # Export already selects its one-to-one detections: xyxy, score,
        # person class 0, then COCO17 xyz. Do not apply a second NMS.
        if np.any(raw[0, :, 5] != 0):
            raise RuntimeError("YOLO26 pose export must contain only person class 0")
        candidates = np.concatenate((raw[0, :, :5], raw[0, :, 6:]), axis=1)
    else:
        if raw.ndim != 3 or raw.shape[0] != 1 or raw.shape[1] != 56:
            raise RuntimeError("Expected YOLO pose output [1,56,N], received " + str(raw.shape))
        candidates = raw[0].T
    valid = np.isfinite(candidates).all(axis=1) & (candidates[:, 4] >= cfg["confidence"])
    valid &= candidates[:, 4] <= 1
    valid &= ((candidates[:, 2] > candidates[:, 0]) & (candidates[:, 3] > candidates[:, 1]) if end2end
              else (candidates[:, 2] > 0) & (candidates[:, 3] > 0))
    rows = candidates[valid]
    if not len(rows):
        return []
    order = np.argsort(-rows[:, 4], kind="stable")[:cfg.get("max_nms_candidates", 30000)]
    rows = rows[order]
    boxes = rows[:, :4] if end2end else np.column_stack([rows[:, :2] - rows[:, 2:4] / 2, rows[:, :2] + rows[:, 2:4] / 2])
    remaining = np.array([], dtype=int) if end2end else np.arange(len(rows))
    keep = list(range(min(len(rows), cfg["max_det"]))) if end2end else []
    while len(remaining) and len(keep) < cfg["max_det"]:
        chosen = int(remaining[0]); keep.append(chosen)
        others = remaining[1:]
        if not len(others): break
        intersection = np.maximum(0, np.minimum(boxes[chosen, 2:], boxes[others, 2:]) -
                                  np.maximum(boxes[chosen, :2], boxes[others, :2])).prod(axis=1)
        areas = (boxes[others, 2:] - boxes[others, :2]).prod(axis=1)
        area = float(np.prod(boxes[chosen, 2:] - boxes[chosen, :2]))
        overlap = intersection / np.maximum(area + areas - intersection, 1e-9)
        remaining = others[overlap <= cfg["nms_iou"]]
    output = []
    h, w = transform["shape"]
    for index in keep:
        box = boxes[index].astype(float)
        box[[0, 2]] = (box[[0, 2]] - transform["left"]) / transform["gain"]
        box[[1, 3]] = (box[[1, 3]] - transform["top"]) / transform["gain"]
        box[[0, 2]] = np.clip(box[[0, 2]], 0, w)
        box[[1, 3]] = np.clip(box[[1, 3]], 0, h)
        if box[2] <= box[0] or box[3] <= box[1]: continue
        points = rows[index, 5:].reshape(17, 3).astype(float).copy()
        if np.any((points[:, 2] < 0) | (points[:, 2] > 1)):
            raise RuntimeError("YOLO keypoint confidence is not a probability")
        points[:, 0] = (points[:, 0] - transform["keypoint_pad_x"]) / transform["gain"]
        points[:, 1] = (points[:, 1] - transform["keypoint_pad_y"]) / transform["gain"]
        # Keep off-image coordinates: clipping would turn truncated limbs into
        # seemingly visible boundary joints in the downstream quality gate.
        output.append({"bbox": box.tolist(), "score": float(rows[index, 4]), "keypoints": points})
    return output


class YoloPosePredictor:
    def __init__(self, path, cfg, device="GPU", session=None):
        self.cfg = cfg
        output_format = cfg.get("output_format", "yolov8_raw")
        if output_format not in ("yolov8_raw", "yolo26_end2end"):
            raise ValueError("Unknown pose output format: " + str(output_format))
        engine = cfg.get("backend", "onnxruntime")
        if engine == "openvino" and device.upper() != "CPU":
            raise ValueError("OpenVINO pose backend requires device: CPU")
        if session is None and engine == "openvino":
            from .openvino_cpu import OpenVINOSession
            session = OpenVINOSession(path, cfg.get("cpu_threads", 4))
        if session is None:
            import onnxruntime as ort
            requested = "CUDAExecutionProvider" if device.upper() == "GPU" else "CPUExecutionProvider"
            if requested not in ort.get_available_providers():
                raise RuntimeError("Required ONNX Runtime provider unavailable: " + requested)
            options = ort.SessionOptions()
            options.intra_op_num_threads = int(cfg.get("cpu_threads", 2))
            if device.upper() == "GPU": options.add_session_config_entry("session.disable_cpu_ep_fallback", "1")
            providers = [(requested, {"device_id": 0, "arena_extend_strategy": "kSameAsRequested",
                "use_tf32": "0", "cudnn_conv_algo_search": "HEURISTIC", "gpu_mem_limit": 2 * 1024**3})
                ] if device.upper() == "GPU" else [requested]
            session = ort.InferenceSession(str(path), sess_options=options, providers=providers)
            if device.upper() == "GPU": session.disable_fallback()
            if requested not in session.get_providers():
                raise RuntimeError("ONNX Runtime silently failed to initialize " + requested)
            self.version = ort.__version__
        else:
            self.version = getattr(session, "version", "injected_test_session")
        self.session = session
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != "images" or list(inputs[0].shape) != [1, 3, 640, 640] or inputs[0].type != "tensor(float)":
            raise ValueError("Static FP32 images[1,3,640,640] input contract required")
        expected = [1, 300, 57] if output_format == "yolo26_end2end" else [1, 56, 8400]
        if (len(outputs) != 1 or outputs[0].name != "output0" or list(outputs[0].shape) != expected
                or (output_format == "yolo26_end2end" and outputs[0].type != "tensor(float)")):
            raise ValueError(f"Pose {output_format} requires FP32 output0{expected}")
        self.last_timing = {}

    def predict(self, bgr):
        started = time.perf_counter()
        tensor, transform = letterbox(bgr)
        forward = time.perf_counter()
        raw = self.session.run(["output0"], {"images": tensor})[0]
        post = time.perf_counter()
        people = decode_pose(raw, transform, self.cfg)
        done = time.perf_counter()
        self.last_timing = {"yolo_preprocess": (forward - started) * 1000,
                            "yolo_forward": (post - forward) * 1000,
                            "yolo_postprocess": (done - post) * 1000}
        return people


class YoloBackend:
    # These adapters retain official preprocessing/postprocessing of auxiliaries.
    crops = PaddleBackend.crops
    prepare_fight = PaddleBackend.prepare_fight
    fight = PaddleBackend.fight

    def __init__(self, cfg, pose_predictor=None):
        self.cfg = cfg
        if "opencv_threads" in cfg:
            import cv2
            cv2.setNumThreads(cfg["opencv_threads"])
        root = Path(cfg["project_root"])
        vendor = root / cfg["vendor_dir"]
        for suffix in ("", "deploy", "deploy/pptracking/python", "deploy/python",
                       "deploy/pipeline", "deploy/pipeline/pphuman"):
            sys.path.insert(0, str(vendor / suffix))
        import paddle
        paddle.enable_static()
        from python.infer import Detector
        from python.keypoint_preprocess import expand_crop
        from python.preprocess import ShortSizeScale
        from pphuman.video_action_infer import VideoActionRecognizer

        paths = {key: root / value for key, value in cfg["models"].items()}
        digest = hashlib.sha256()
        with paths["yolo_pose"].open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
        fight_openvino = cfg["fight"].get("backend", "paddle") == "openvino"
        smoking_openvino = cfg["smoking"].get("backend", "paddle") == "openvino"
        if smoking_openvino:
            files = [paths["smoking"] / name for name in ("model.xml", "model.bin", "infer_cfg.yml")]
            self.identities = {"smoking": {"name": "ppyoloe_smoking_openvino_fp32", "path": str(paths["smoking"]),
                "files_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}}
        else:
            self.identities = {"smoking": model_identity(paths["smoking"])}
        if not fight_openvino:
            self.identities["fight"] = model_identity(paths["fight"])
        pose_name = "yolo26s_pose" if cfg["yolo_pose"].get("output_format") == "yolo26_end2end" else "yolov8m_pose"
        self.identities["yolo_pose"] = {"name": "official_" + pose_name + "_640", "path": str(paths["yolo_pose"]),
                                        "files_sha256": {paths["yolo_pose"].name: digest.hexdigest()}}
        if cfg["yolo_pose"].get("backend") == "openvino":
            weights = paths["yolo_pose"].with_suffix(".bin")
            if paths["yolo_pose"].suffix != ".xml" or not weights.is_file():
                raise FileNotFoundError("OpenVINO pose requires both model.xml and model.bin")
            self.identities["yolo_pose"]["name"] = pose_name + "_openvino_cpu_candidate"
            self.identities["yolo_pose"]["files_sha256"][weights.name] = hashlib.sha256(weights.read_bytes()).hexdigest()
        self.pose_predictor = pose_predictor or YoloPosePredictor(paths["yolo_pose"], cfg["yolo_pose"], cfg["device"])
        recovery_cfg = cfg["yolo_pose"].get("recovery", {})
        self.pose_recovery = PoseRecovery(self.pose_predictor, recovery_cfg) if recovery_cfg.get("enabled", True) else None
        self.last_pose_observation = None
        self.tracker = ObservationTracker(cfg["tracking"])
        self.expand_crop = expand_crop
        common = {"device": cfg["device"], "run_mode": cfg["run_mode"],
                  "cpu_threads": cfg["cpu_threads"], "enable_mkldnn": cfg.get("enable_mkldnn", False)}
        if smoking_openvino:
            from .openvino_cpu import create_smoking_detector
            self.smoking_model = create_smoking_detector(paths["smoking"], cfg)
        else:
            with predictor_options(cfg):
                self.smoking_model = Detector(str(paths["smoking"]), batch_size=cfg["smoking"]["batch_size"],
                                              threshold=cfg["smoking"]["threshold"], **common)
        if fight_openvino:
            if cfg["device"].upper() != "CPU":
                raise ValueError("OpenVINO fight requires device: CPU")
            from .openvino_cpu import create_fight_recognizer
            fight_path = paths["fight"]
            fight_weights = fight_path.with_suffix(".bin")
            if fight_path.suffix != ".xml" or not fight_path.is_file() or not fight_weights.is_file():
                raise FileNotFoundError("OpenVINO fight requires both model.xml and model.bin")
            self.identities["fight"] = {"name": "ppTSM_openvino_cpu_candidate", "path": str(fight_path),
                "files_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (fight_path, fight_weights)}}
            self.fight_model = create_fight_recognizer(fight_path, cfg)
        else:
            with predictor_options(cfg):
                self.fight_model = VideoActionRecognizer(str(paths["fight"]), batch_size=1,
                    num_seg=cfg["fight"]["window_frames"], short_size=cfg["fight"]["short_size"],
                    target_size=cfg["fight"]["target_size"], **common)
        self.scale = ShortSizeScale(cfg["fight"]["short_size"])
        record = root / "environment/paddledetection.commit"
        self.metadata = {"profile": cfg["profile"], "paddle_version": paddle.__version__,
                         "onnxruntime_version": self.pose_predictor.version,
                         "onnxruntime_providers": self.pose_predictor.session.get_providers(),
                         "onnxruntime_provider_options": self.pose_predictor.session.get_provider_options(),
                         "onnxruntime_cpu_fallback_disabled": cfg["device"].upper() == "GPU",
                         "device": cfg["device"], "source_commit": record.read_text().strip() if record.is_file() else None,
                         "fall_score_semantics": "rule_evidence; score is null",
                         "shared_observation": "one " + pose_name + " model/session; native forward first, at most one same-frame rotated retry on empty native result; mapped boxes and COCO17 feed tracking/actions",
                         "pose_recovery": {"enabled": self.pose_recovery is not None, "max_extra_forwards_per_frame": 1,
                             "parameters": dict(self.pose_recovery.cfg) if self.pose_recovery is not None else {},
                             "limitations": ["requires recently observed reliable native person", "does not recover a missing person when native detects any other person"],
                             "cached_persons": False}}
        self.metadata["smoking_association"] = {"enabled": bool(cfg["smoking"].get("association", {}).get("enabled")),
            "module_sha256": hashlib.sha256(Path(__file__).with_name("smoking_filter.py").read_bytes()).hexdigest(),
            "score_semantics": "unchanged raw detector score after this person's estimated mouth association"}
        self.metadata["smoking_hand_to_mouth"] = {
            "enabled": bool(cfg["smoking"].get("hand_to_mouth", {}).get("enabled")),
            "parameters": dict(cfg["smoking"].get("hand_to_mouth", {})),
            "module_sha256": hashlib.sha256(Path(__file__).with_name("smoking_hand_to_mouth.py").read_bytes()).hexdigest(),
            "score_semantics": "prior real cigarette detection score; evidence carries its source timestamp; not action probability"}
        self.metadata['smoking_profile_refinement'] = {
            'enabled': bool(cfg['smoking'].get('profile_refinement', {}).get('enabled')),
            'parameters': dict(cfg['smoking'].get('profile_refinement', {})),
            'confirmation': 'two distinct mouth-contact detections, a strong seed within 1.25 seconds, stable same-person tracking; a missed tick never emits a positive',
            'merge': 'supplement only after the unchanged body and hand-to-mouth decisions',
            'extra_models': 0}
        if cfg["yolo_pose"].get("backend") == "openvino":
            # Avoid reporting an OpenVINO version as an ONNX Runtime version.
            self.metadata.pop("onnxruntime_version", None)
            self.metadata.pop("onnxruntime_providers", None)
            self.metadata.pop("onnxruntime_provider_options", None)
            self.metadata.pop("onnxruntime_cpu_fallback_disabled", None)
            self.metadata.update(pose_backend="OpenVINO:CPU",
                openvino_version=self.pose_predictor.version,
                pose_cpu_threads=cfg["yolo_pose"].get("cpu_threads", 4),
                paddle_cpu_threads=cfg["cpu_threads"],
                paddle_mkldnn_enabled=cfg.get("enable_mkldnn", False))
        if fight_openvino:
            self.metadata["fight_inference"] = {
                "backend": "OpenVINO:CPU", "version": self.fight_model.predictor.session.version,
                "cpu_threads": cfg["fight"].get("cpu_threads", cfg["cpu_threads"]), "streams": 1,
                "target_size": cfg["fight"]["target_size"], "window_frames": 8,
                "asynchronous": cfg["fight"].get("async_inference", False), "max_inflight_windows": 1,
                "precision": "INT8 backbone with floating classifier; FP32 I/O",
                "preprocess_postprocess": "unchanged official VideoActionRecognizer",
                "validation": "model and configuration dependent; see deployment notes"}
        if smoking_openvino:
            self.metadata["smoking_inference"] = {"backend": "OpenVINO:CPU", "precision": "FP32",
                "cpu_threads": cfg["smoking"].get("cpu_threads", cfg["cpu_threads"]), "batch_execution": "one image per NMS graph",
                "skip_unobservable": cfg["smoking"].get("skip_unobservable", False),
                "model_change": "unchanged source weights; explicit singleton NMS axes"}
        self.metadata["paddle_predictors_loaded"] = int(not smoking_openvino) + int(not fight_openvino)
        if not self.metadata["paddle_predictors_loaded"]:
            self.metadata["paddle_mkldnn_enabled"] = False
        self.last_timing = {}

    def start_fight(self, samples):
        inputs = self.fight_model.preprocess_frames(samples)
        self.fight_model.predictor.start_async(inputs)

    def poll_fight(self):
        completed = self.fight_model.predictor.poll_async()
        if completed is None:
            return None
        output, latency = completed
        classes, scores = self.fight_model.postprocess(output)
        return {"class": int(classes[0]), "score": float(scores[0]), "model_inference_ms": latency}

    def close(self):
        if self.cfg["fight"].get("backend", "paddle") == "openvino":
            self.fight_model.predictor.close()
            self.fight_model.predictor.session.close()
        if self.cfg["smoking"].get("backend", "paddle") == "openvino":
            self.smoking_model.predictor.session.close()
        if self.cfg["yolo_pose"].get("backend", "onnxruntime") == "openvino":
            self.pose_predictor.session.close()

    def reset(self):
        self.tracker.reset()
        if self.pose_recovery is not None:
            self.pose_recovery.reset()
        self.last_pose_observation = None

    def observe(self, bgr, stamp):
        if self.pose_recovery is not None:
            detections = self.pose_recovery.predict(bgr, stamp)
            pose_timing = self.pose_recovery.last_timing
            self.last_pose_observation = dict(self.pose_recovery.last_diagnostic)
        else:
            detections = self.pose_predictor.predict(bgr)
            pose_timing = self.pose_predictor.last_timing
            self.last_pose_observation = {"source_stamp_ns": stamp, "recovery_enabled": False,
                "native_person_count": len(detections), "recovered_person_count": 0,
                "forward_rotations_cw": [0], "actual_forward_count": 1, "extra_forward_count": 0,
                "persons_cached": False, "trigger": "disabled"}
        started = time.perf_counter()
        people = self.tracker.update(detections, stamp)
        self.last_timing = {**pose_timing, "tracking": (time.perf_counter() - started) * 1000}
        return people

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
