"""Thin adapters over pinned PaddleDetection deploy prediction classes."""

import copy
import contextlib
import hashlib
import subprocess
import sys
from pathlib import Path

import numpy as np


def normalized_classification(output):
    """Normalize the full two-class tensor, never fabricate from top-1 alone."""
    logits = np.asarray(output, dtype=np.float64).reshape(-1)
    if logits.shape != (2,) or not np.isfinite(logits).all():
        raise RuntimeError("Expected two finite STGCN logits")
    weights = np.exp(logits - np.max(logits))
    probabilities = weights / weights.sum()
    cls = int(np.argmax(logits))
    return {"class": cls, "score": float(probabilities[cls]),
            "score_kind": "softmax_of_full_model_logits", "raw_model_logits": logits.tolist()}


@contextlib.contextmanager
def predictor_options(cfg):
    """Apply deployment IR options without editing pinned upstream sources.

    Upstream loaders do not expose delete_pass through their constructors. Scope
    the create_predictor hook to construction only; preserve official preprocess,
    predictor creation, forward and postprocess implementations.
    """
    previous = []
    for name in ("python.infer", "infer", "det_infer", "pphuman.video_action_infer"):
        module = sys.modules.get(name)
        if module is None or not hasattr(module, "create_predictor"):
            continue
        original = module.create_predictor

        def create(config, factory=original):
            for pass_name in cfg.get("disabled_ir_passes", []):
                config.delete_pass(pass_name)
            if cfg.get("enable_mkldnn", False):
                if cfg["device"].upper() != "CPU":
                    raise ValueError("MKLDNN acceleration requires device: CPU")
                # The upstream video classifier stores cpu_threads but does not
                # apply it to its predictor. Bound both auxiliary predictors.
                config.set_cpu_math_library_num_threads(cfg["cpu_threads"])
                config.set_mkldnn_cache_capacity(10)
                config.enable_mkldnn()
            return factory(config)

        previous.append((module, original))
        module.create_predictor = create
    try:
        yield
    finally:
        for module, original in reversed(previous):
            module.create_predictor = original


def model_identity(path):
    files = sorted(p for p in path.iterdir()
                   if p.suffix in {".pdmodel", ".pdiparams", ".yml", ".yaml"})
    if not any(p.suffix == ".pdmodel" for p in files) or not any(
            p.suffix == ".pdiparams" for p in files):
        raise FileNotFoundError("Incomplete exported model: " + str(path))
    hashes = {}
    for file in files:
        digest = hashlib.sha256()
        with file.open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
        hashes[file.name] = digest.hexdigest()
    return {"name": path.name, "path": str(path), "files_sha256": hashes}


class PaddleBackend:
    def __init__(self, cfg):
        root = Path(cfg["project_root"])
        vendor = root / cfg["vendor_dir"]
        if not (vendor / "deploy").is_dir():
            raise FileNotFoundError("Official deploy source missing: " + str(vendor))
        # Match pipeline.py's import paths, without importing its video loop.
        for suffix in ("", "deploy", "deploy/pptracking/python", "deploy/python",
                       "deploy/pipeline", "deploy/pipeline/pphuman"):
            sys.path.insert(0, str(vendor / suffix))
        import paddle
        paddle.enable_static()
        # Import general deploy first, as official pipeline.py does. Both trees
        # have bare `preprocess` imports; MOT-first would cache a smaller module.
        from python.infer import Detector
        from python.keypoint_infer import KeyPointDetector
        from pptracking.python.mot_sde_infer import SDE_Detector
        from python.keypoint_preprocess import expand_crop
        from python.keypoint_postprocess import translate_to_ori_images
        from python.preprocess import ShortSizeScale
        from pphuman.action_infer import SkeletonActionRecognizer
        from pphuman.video_action_infer import VideoActionRecognizer
        from pipe_utils import parse_mot_res, refine_keypoint_coordinary

        self.cfg = cfg
        self.expand_crop = expand_crop
        self.translate = translate_to_ori_images
        self.parse_mot_res = parse_mot_res
        self.refine = refine_keypoint_coordinary
        paths = {key: root / value for key, value in cfg["models"].items()}
        self.identities = {key: model_identity(path) for key, path in paths.items()}
        commit = subprocess.run(["git", "-C", str(vendor), "rev-parse", "HEAD"],
                                capture_output=True, text=True, check=False)
        source_record = root / "environment/paddledetection.commit"
        source_commit = commit.stdout.strip() if commit.returncode == 0 else None
        if not source_commit and source_record.is_file():
            source_commit = source_record.read_text(encoding="utf-8").strip()
        self.metadata = {"paddle_version": paddle.__version__,
                         "source_commit": source_commit,
                         "device": cfg["device"], "run_mode": cfg["run_mode"],
                         "disabled_ir_passes": cfg.get("disabled_ir_passes", []),
                         "fall_score_semantics": "stable_softmax_of_full_two_class_logits; not calibrated"}
        common = {"device": cfg["device"], "run_mode": cfg["run_mode"],
                  "cpu_threads": cfg["cpu_threads"], "enable_mkldnn": False}
        def build(cls, *args, **kwargs):
            with predictor_options(cfg):
                return cls(*args, **kwargs)

        self.mot = build(SDE_Detector, str(paths["mot"]),
                               tracker_config=str(root / cfg["tracker_config"]),
                               batch_size=1, threshold=cfg["mot"]["threshold"],
                               skip_frame_num=-1, **common)
        if self.mot.num_classes != 1:
            raise ValueError("The shared MOT must be a one-class human detector")
        self.empty_tracker = copy.deepcopy(self.mot.tracker)
        self.pose_model = build(KeyPointDetector, str(paths["pose"]),
                                          batch_size=cfg["pose"]["batch_size"],
                                          use_dark=True, **common)
        # Official Detector, before DetActionRecognizer's frame-based history.
        self.smoking_model = build(Detector, str(paths["smoking"]),
                                      batch_size=cfg["smoking"]["batch_size"],
                                      threshold=cfg["smoking"]["threshold"], **common)
        self.fall_model = build(SkeletonActionRecognizer, str(paths["fall"]),
                                                   batch_size=1, **common)
        self.fight_model = build(VideoActionRecognizer,
            str(paths["fight"]), batch_size=1,
            num_seg=cfg["fight"]["window_frames"],
            short_size=cfg["fight"]["short_size"],
            target_size=cfg["fight"]["target_size"], **common)
        self.scale = ShortSizeScale(cfg["fight"]["short_size"])
        self.frame_count = 0

    def reset(self):
        # Preserve loaded predictors but discard tracker motion/counters/history.
        self.mot.tracker = copy.deepcopy(self.empty_tracker)
        self.mot.previous_det_result = None
        self.frame_count = 0

    def tracks(self, rgb):
        self.frame_count += 1
        result = self.mot.predict_image([rgb.copy()], visual=False,
                                        reuse_det_result=False,
                                        frame_count=self.frame_count)
        boxes = self.parse_mot_res(result)["boxes"]
        people = []
        h, w = rgb.shape[:2]
        for row in boxes:
            if not np.isfinite(row).all():
                continue
            bbox = [float(np.clip(row[3], 0, w)), float(np.clip(row[4], 0, h)),
                    float(np.clip(row[5], 0, w)), float(np.clip(row[6], 0, h))]
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue
            people.append({"raw_id": int(row[0]), "bbox": bbox, "score": float(row[2])})
        return people

    def crops(self, rgb, people):
        crops = {}
        for person in people:
            rect = np.asarray([0, person["score"], *person["bbox"]])
            crop, new_box, original_box = self.expand_crop(rgb, rect)
            if crop is not None and crop.size and original_box[2] > original_box[0] \
                    and original_box[3] > original_box[1]:
                crops[person["raw_id"]] = (crop, new_box, original_box)
        return crops

    def pose(self, crops):
        if not crops:
            return {}
        ids = list(crops)
        result = self.pose_model.predict_image([crops[i][0] for i in ids], visual=False)
        points, _ = self.translate(result, np.asarray([crops[i][1] for i in ids]))
        if len(points) != len(ids):
            raise RuntimeError("HRNet result count disagrees with person crop count")
        return dict(zip(ids, points))

    def smoking(self, crops):
        if not crops:
            return {}
        ids = list(crops)
        result = self.smoking_model.predict_image([crops[i][0] for i in ids], visual=False)
        counts = np.asarray(result["boxes_num"]).reshape(-1)
        boxes = np.asarray(result["boxes"]).reshape(-1, 6)
        if len(counts) != len(ids) or sum(counts) != len(boxes):
            raise RuntimeError("Smoking detector result count mismatch")
        output, start = {}, 0
        for raw_id, count in zip(ids, counts):
            subset = boxes[start:start + int(count)]
            start += int(count)
            if not np.isfinite(subset).all():
                raise RuntimeError("Smoking detector returned non-finite values")
            positive = subset[(subset[:, 0] == self.cfg["smoking"]["positive_class"])
                              & (subset[:, 1] >= self.cfg["smoking"]["threshold"])]
            output[raw_id] = {"label": "smoking" if len(positive) else "other",
                              "score": float(positive[:, 1].max()) if len(positive) else None,
                              "score_kind": "detection_score" if len(positive)
                              else "no_positive_detection"}
        return output

    def fall(self, samples):
        points = np.asarray([sample[0] for sample in samples], dtype=np.float32)
        boxes = np.asarray([sample[1] for sample in samples], dtype=np.float32)
        skeleton = np.expand_dims(points[:, :, :2].transpose(2, 0, 1), -1)
        skeleton = self.refine(skeleton, boxes, self.cfg["fall"]["coord_size"])
        # Official predict_skeleton postprocess selects the top raw logit and
        # discards all other logits. Use its official preprocess/predict methods
        # to retain the complete tensor and compute a genuine normalized score.
        self.fall_model.preprocess(skeleton)
        raw = self.fall_model.predict()
        return normalized_classification(raw["output"])

    def prepare_fight(self, rgb):
        # Official frame-list path expects already scaled RGB/PIL inputs.
        return self.scale(rgb)

    def fight(self, samples):
        classes, scores = self.fight_model.predict(samples)
        return int(classes[0]), float(scores[0])
