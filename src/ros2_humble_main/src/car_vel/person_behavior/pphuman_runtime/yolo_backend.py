"""One official YOLO pose observation shared by tracking and action branches.

The exported graph performs no NMS: xywh + person probability + COCO17 xyz.
Only the scene fight predictor is loaded from PaddleDetection.
"""
import hashlib
import sys
import time
from pathlib import Path

import numpy as np

from .backend import FightAdapter, model_identity, predictor_options
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
    if raw.ndim != 3 or raw.shape[0] != 1 or raw.shape[1] != 56:
        raise RuntimeError("Expected YOLO pose output [1,56,N], received " + str(raw.shape))
    candidates = raw[0].T
    valid = np.isfinite(candidates).all(axis=1) & (candidates[:, 4] >= cfg["confidence"])
    valid &= (candidates[:, 4] <= 1) & (candidates[:, 2] > 0) & (candidates[:, 3] > 0)
    rows = candidates[valid]
    if not len(rows):
        return []
    order = np.argsort(-rows[:, 4], kind="stable")[:cfg.get("max_nms_candidates", 30000)]
    rows = rows[order]
    boxes = np.column_stack([rows[:, :2] - rows[:, 2:4] / 2, rows[:, :2] + rows[:, 2:4] / 2])
    remaining = np.arange(len(rows))
    keep = []
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
    def __init__(self, path, cfg, device="CPU", session=None):
        self.cfg = cfg
        if session is None:
            import onnxruntime as ort
            requested = "CUDAExecutionProvider" if device.upper() == "GPU" else "CPUExecutionProvider"
            if requested not in ort.get_available_providers():
                raise RuntimeError("Required ONNX Runtime provider unavailable: " + requested)
            options = ort.SessionOptions()
            options.intra_op_num_threads = int(cfg.get("cpu_threads", 2))
            options.inter_op_num_threads = 1
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
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
            self.version = "injected_test_session"
        self.session = session
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != "images" or list(inputs[0].shape) != [1, 3, 640, 640] or inputs[0].type != "tensor(float)":
            raise ValueError("Official static FP32 images[1,3,640,640] contract required")
        if len(outputs) != 1 or outputs[0].name != "output0" or list(outputs[0].shape) != [1, 56, 8400]:
            raise ValueError("Official no-NMS output0[1,56,8400] contract required")
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
    prepare_fight = FightAdapter.prepare_fight
    fight = FightAdapter.fight

    def __init__(self, cfg):
        self.cfg = cfg
        root = Path(cfg["project_root"])
        vendor = root / cfg["vendor_dir"]
        if cfg['branches']['fight'] and not (vendor / "deploy").is_dir():
            vendor = Path(sys.prefix).parent / "vendor" / "PaddleDetection"
        if cfg['branches']['fight'] and not (vendor / "deploy").is_dir():
            raise FileNotFoundError("Pinned PaddleDetection deploy source missing; run install_behavior_deps.sh")
        for suffix in ("", "deploy", "deploy/pptracking/python", "deploy/python",
                       "deploy/pipeline", "deploy/pipeline/pphuman"):
            sys.path.insert(0, str(vendor / suffix))
        paths = {key: root / value for key, value in cfg["models"].items()}
        self.identities = {}
        self.pose_predictor = None
        if cfg["branches"]["fall"]:
            digest = hashlib.sha256()
            with paths["yolo_pose"].open("rb") as stream:
                for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                    digest.update(block)
            self.identities["yolo_pose"] = {"name": "official_yolov8m_pose_640", "path": str(paths["yolo_pose"]),
                                            "files_sha256": {paths["yolo_pose"].name: digest.hexdigest()}}
            self.pose_predictor = YoloPosePredictor(paths["yolo_pose"], cfg["yolo_pose"], cfg["device"])
        recovery_cfg = cfg["yolo_pose"].get("recovery", {})
        self.pose_recovery = (PoseRecovery(self.pose_predictor, recovery_cfg)
                              if self.pose_predictor is not None and recovery_cfg.get("enabled", True) else None)
        self.last_pose_observation = None
        self.tracker = ObservationTracker(cfg["tracking"])
        common = {"device": cfg["device"], "run_mode": cfg["run_mode"],
                  "cpu_threads": cfg["cpu_threads"], "enable_mkldnn": False}
        paddle_version = None
        if cfg["branches"]["fight"]:
            import paddle
            paddle.enable_static()
            from python.preprocess import ShortSizeScale
            from pphuman.video_action_infer import VideoActionRecognizer
            self.identities["fight"] = model_identity(paths["fight"])
            with predictor_options(cfg):
                self.fight_model = VideoActionRecognizer(str(paths["fight"]), batch_size=1,
                    num_seg=cfg["fight"]["window_frames"], short_size=cfg["fight"]["short_size"],
                    target_size=cfg["fight"]["target_size"], **common)
            self.scale = ShortSizeScale(cfg["fight"]["short_size"])
            paddle_version = paddle.__version__
        record = root / "environment/paddledetection.commit"
        self.metadata = {"profile": cfg["profile"], "paddle_version": paddle_version,
                         "onnxruntime_version": self.pose_predictor.version if self.pose_predictor else None,
                         "onnxruntime_providers": self.pose_predictor.session.get_providers() if self.pose_predictor else [],
                         "onnxruntime_provider_options": self.pose_predictor.session.get_provider_options() if self.pose_predictor else {},
                         "onnxruntime_cpu_fallback_disabled": cfg["device"].upper() == "GPU",
                         "device": cfg["device"], "source_commit": record.read_text().strip() if record.is_file() else None,
                         "fall_score_semantics": "rule_evidence; score is null",
                         "shared_observation": "one YOLOv8m-pose model/session; native forward first, at most one same-frame rotated retry on empty native result; mapped boxes and COCO17 feed tracking/actions",
                         "pose_recovery": {"enabled": self.pose_recovery is not None, "max_extra_forwards_per_frame": 1,
                             "parameters": dict(self.pose_recovery.cfg) if self.pose_recovery is not None else {},
                             "limitations": ["requires recently observed reliable native person", "does not recover a missing person when native detects any other person"],
                             "cached_persons": False}}
        self.last_timing = {}

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

