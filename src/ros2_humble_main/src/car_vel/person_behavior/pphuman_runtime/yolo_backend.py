"""Shared COCO17 pose observations from YOLOv8 raw or YOLO26 end-to-end exports."""
import hashlib
import sys
import time
from pathlib import Path

import numpy as np

from .assets import sha256
from .backend import FightAdapter, model_identity, predictor_options
from .smoking_backend import SmokingAdapter
from .tracking import ObservationTracker
from .pose_recovery import PoseRecovery


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


class YoloBackend(SmokingAdapter, FightAdapter):
    def __init__(self, cfg, pose_predictor=None):
        self.cfg = cfg
        root = Path(cfg["project_root"])
        branches = cfg["branches"]
        pose_enabled = branches["fall"] or branches["smoking"]
        paddle_version = None
        if branches["smoking"] or branches["fight"]:
            vendor = root / cfg["vendor_dir"]
            if not (vendor / "deploy").is_dir():
                vendor = Path(sys.prefix).parent / "vendor" / "PaddleDetection"
            if not (vendor / "deploy").is_dir():
                raise FileNotFoundError("Pinned PaddleDetection deploy source missing; run install_behavior_deps.sh")
            for suffix in ("", "deploy", "deploy/pptracking/python", "deploy/python",
                           "deploy/pipeline", "deploy/pipeline/pphuman"):
                sys.path.insert(0, str(vendor / suffix))
            import paddle
            paddle.enable_static()
            paddle_version = paddle.__version__

        paths = {key: root / value for key, value in cfg["models"].items()}
        self.identities = {}
        self.pose_predictor = None
        self.smoking_model = None
        self.fight_model = None
        fight_openvino = cfg["fight"].get("backend", "paddle") == "openvino"
        smoking_openvino = cfg["smoking"].get("backend", "paddle") == "openvino"
        pose_name = "yolo26s_pose" if cfg["yolo_pose"].get("output_format") == "yolo26_end2end" else "yolov8m_pose"
        if pose_enabled:
            pose_path = paths["yolo_pose"]
            files = [pose_path]
            name = "official_" + pose_name + "_640"
            if cfg["yolo_pose"].get("backend") == "openvino":
                files.append(pose_path.with_suffix(".bin"))
                name = pose_name + "_openvino_fp32"
            self.identities["yolo_pose"] = {"name": name, "path": str(pose_path),
                "files_sha256": {file.name: sha256(file) for file in files}}
            self.pose_predictor = pose_predictor or YoloPosePredictor(pose_path, cfg["yolo_pose"], cfg["device"])
        recovery_cfg = cfg["yolo_pose"].get("recovery", {})
        self.pose_recovery = PoseRecovery(self.pose_predictor, recovery_cfg) if self.pose_predictor is not None and recovery_cfg.get("enabled", True) else None
        self.last_pose_observation = None
        self.tracker = ObservationTracker(cfg["tracking"])
        common = {"device": cfg["device"], "run_mode": cfg["run_mode"],
                  "cpu_threads": cfg["cpu_threads"], "enable_mkldnn": cfg.get("enable_mkldnn", False)}
        if branches["smoking"]:
            from python.keypoint_preprocess import expand_crop
            self.expand_crop = expand_crop
            if smoking_openvino:
                from .openvino_cpu import create_smoking_detector
                files = [paths["smoking"] / name for name in ("model.xml", "model.bin", "infer_cfg.yml")]
                self.identities["smoking"] = {"name": "ppyoloe_smoking_openvino_fp32",
                    "path": str(paths["smoking"]), "files_sha256": {p.name: sha256(p) for p in files}}
                self.smoking_model = create_smoking_detector(paths["smoking"], cfg)
            else:
                from python.infer import Detector
                self.identities["smoking"] = model_identity(paths["smoking"])
                with predictor_options(cfg):
                    self.smoking_model = Detector(str(paths["smoking"]), batch_size=cfg["smoking"]["batch_size"],
                                                  threshold=cfg["smoking"]["threshold"], **common)
        if branches["fight"]:
            from python.preprocess import ShortSizeScale
            if fight_openvino:
                if cfg["device"].upper() != "CPU":
                    raise ValueError("OpenVINO fight requires device: CPU")
                from .openvino_cpu import create_fight_recognizer
                fight_path = paths["fight"]
                fight_weights = fight_path.with_suffix(".bin")
                if fight_path.suffix != ".xml" or not fight_path.is_file() or not fight_weights.is_file():
                    raise FileNotFoundError("OpenVINO fight requires both model.xml and model.bin")
                self.identities["fight"] = {"name": "ppTSM_openvino_int8_224", "path": str(fight_path),
                    "files_sha256": {p.name: sha256(p)
                                     for p in (fight_path, fight_weights)}}
                self.fight_model = create_fight_recognizer(fight_path, cfg)
            else:
                from pphuman.video_action_infer import VideoActionRecognizer
                self.identities["fight"] = model_identity(paths["fight"])
                with predictor_options(cfg):
                    self.fight_model = VideoActionRecognizer(str(paths["fight"]), batch_size=1,
                        num_seg=cfg["fight"]["window_frames"], short_size=cfg["fight"]["short_size"],
                        target_size=cfg["fight"]["target_size"], **common)
            self.scale = ShortSizeScale(cfg["fight"]["short_size"])
        record = root / "environment/paddledetection.commit"
        self.metadata = {"profile": cfg["profile"], "paddle_version": paddle_version,
                         "onnxruntime_version": self.pose_predictor.version if pose_enabled else None,
                         "onnxruntime_providers": self.pose_predictor.session.get_providers() if pose_enabled else [],
                         "onnxruntime_provider_options": self.pose_predictor.session.get_provider_options() if pose_enabled else {},
                         "onnxruntime_cpu_fallback_disabled": cfg["device"].upper() == "GPU",
                         "device": cfg["device"], "source_commit": record.read_text().strip() if record.is_file() else None,
                         "fall_score_semantics": "rule_evidence; score is null",
                         "shared_observation": "one " + pose_name + " model/session; native forward first, at most one same-frame rotated retry on empty native result; mapped boxes and COCO17 feed tracking/actions",
                         "pose_recovery": {"enabled": self.pose_recovery is not None, "max_extra_forwards_per_frame": 1,
                             "parameters": dict(self.pose_recovery.cfg) if self.pose_recovery is not None else {},
                             "limitations": ["requires recently observed reliable native person", "does not recover a missing person when native detects any other person"],
                             "cached_persons": False}}
        self.metadata["smoking_association"] = {"enabled": branches["smoking"] and bool(cfg["smoking"].get("association", {}).get("enabled")),
            "module_sha256": hashlib.sha256(Path(__file__).with_name("smoking_filter.py").read_bytes()).hexdigest(),
            "score_semantics": "unchanged raw detector score after this person's estimated mouth association"}
        self.metadata["smoking_hand_to_mouth"] = {
            "enabled": branches["smoking"] and bool(cfg["smoking"].get("hand_to_mouth", {}).get("enabled")),
            "parameters": dict(cfg["smoking"].get("hand_to_mouth", {})),
            "module_sha256": hashlib.sha256(Path(__file__).with_name("smoking_hand_to_mouth.py").read_bytes()).hexdigest(),
            "score_semantics": "prior real cigarette detection score; evidence carries its source timestamp; not action probability"}
        self.metadata['smoking_profile_refinement'] = {
            'enabled': branches["smoking"] and bool(cfg['smoking'].get('profile_refinement', {}).get('enabled')),
            'parameters': dict(cfg['smoking'].get('profile_refinement', {})),
            'confirmation': 'two distinct mouth-contact detections, a strong seed within 1.25 seconds, stable same-person tracking; a missed tick never emits a positive',
            'merge': 'supplement only after the unchanged body and hand-to-mouth decisions',
            'extra_models': 0}
        if pose_enabled and cfg["yolo_pose"].get("backend") == "openvino":
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
        if branches["fight"] and fight_openvino:
            self.metadata["fight_inference"] = {
                "backend": "OpenVINO:CPU", "version": self.fight_model.predictor.session.version,
                "cpu_threads": cfg["fight"].get("cpu_threads", cfg["cpu_threads"]), "streams": 1,
                "target_size": cfg["fight"]["target_size"], "window_frames": 8,
                "asynchronous": cfg["fight"].get("async_inference", False), "max_inflight_windows": 1,
                "precision": "INT8 backbone with floating classifier; FP32 I/O",
                "preprocess_postprocess": "unchanged official VideoActionRecognizer",
                "validation": "model and configuration dependent; see deployment notes"}
        if branches["smoking"] and smoking_openvino:
            self.metadata["smoking_inference"] = {"backend": "OpenVINO:CPU", "precision": "FP32",
                "cpu_threads": cfg["smoking"].get("cpu_threads", cfg["cpu_threads"]), "batch_execution": "one image per NMS graph",
                "skip_unobservable": cfg["smoking"].get("skip_unobservable", False),
                "model_change": "unchanged source weights; explicit singleton NMS axes"}
        self.metadata["paddle_predictors_loaded"] = int(branches["smoking"] and not smoking_openvino) + int(branches["fight"] and not fight_openvino)
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
        if self.fight_model is not None and self.cfg["fight"].get("backend", "paddle") == "openvino":
            self.fight_model.predictor.close()
            self.fight_model.predictor.session.close()
        if self.smoking_model is not None and self.cfg["smoking"].get("backend", "paddle") == "openvino":
            self.smoking_model.predictor.session.close()
        if self.pose_predictor is not None and self.cfg["yolo_pose"].get("backend", "onnxruntime") == "openvino":
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
