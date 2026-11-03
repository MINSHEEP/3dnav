"""Protocol, temporal and two-model isolation tests; no inference accuracy claim."""
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
import yaml

RUNTIME_ROOT = Path(__file__).resolve().parents[1] / "person_behavior"
sys.path.insert(0, str(RUNTIME_ROOT))

from pphuman_runtime.protocol import ProtocolError, read_message, read_request, write_message
from pphuman_runtime.runtime import TimeWindow
from pphuman_runtime.runtime_yolo import RuntimeYolo, fight_classification, load_config
from pphuman_runtime.worker import serve
from pphuman_runtime.yolo_backend import YoloBackend


def config():
    cfg = load_config(RUNTIME_ROOT / "config/inference.yaml")
    cfg["branches"]["smoking"] = False  # Existing fall/fight regression scope.
    cfg["fight"].update(capture_buffer=False, async_inference=False)
    return cfg


def skeleton(kind="standing"):
    points = np.array([[100, 30], [95, 25], [105, 25], [90, 30], [110, 30],
                       [85, 60], [115, 60], [75, 100], [125, 100], [70, 130], [130, 130],
                       [90, 130], [110, 130], [90, 190], [110, 190], [90, 250], [110, 250]], float)
    if kind == "lying":
        points = np.column_stack([points[:, 1] + 60, 300 - points[:, 0]])
    points = np.column_stack([points, np.ones(17) * .95])
    if kind == "half_body":
        points[11:, 2] = 0
    return points


class FakeBackend:
    identities = {key: {"name": key} for key in ("yolo_pose", "fight")}
    metadata = {"test_only": True, "device": "CPU"}
    last_timing = {}

    def __init__(self):
        self.pose_calls = self.fight_calls = self.preprocess_calls = self.resets = 0
        self.kind = "standing"
        self.count = 1

    def reset(self):
        self.resets += 1

    def observe(self, frame, stamp):
        self.pose_calls += 1
        points = skeleton(self.kind)
        bbox = [*points[:, :2].min(axis=0), *points[:, :2].max(axis=0)]
        return [{"track_id": index + 1, "keypoints": points, "bbox": bbox, "score": .9,
                 "association": "associated", "observation_stamp_ns": stamp} for index in range(self.count)]

    def prepare_fight(self, rgb):
        self.preprocess_calls += 1
        return 1

    def fight(self, samples):
        assert len(samples) == 8
        self.fight_calls += 1
        return 0, .9

    def smoking(self, *args):
        raise AssertionError("Smoking must never be called")


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.backend = FakeBackend()
        self.runtime = RuntimeYolo(self.cfg, self.backend)
        self.bgr = np.zeros((400, 400, 3), np.uint8)

    def frame(self, stamp, epoch="source-1", branches=None):
        header = {"request_id": str(stamp), "stamp_ns": round(stamp * 1e9), "source_epoch": epoch}
        if branches is not None:
            header["enabled_branches"] = branches
        return self.runtime.infer(self.bgr, header)

    def test_latest_default_models_and_original_gaps(self):
        cfg = load_config(RUNTIME_ROOT / "config/inference.yaml")
        self.assertEqual(set(cfg["models"]), {"yolo_pose", "fight", "smoking"})
        self.assertEqual(cfg["device"], "CPU")
        self.assertEqual(cfg["cpu_threads"], 4)
        self.assertEqual(cfg["yolo_pose"]["output_format"], "yolo26_end2end")
        self.assertEqual({cfg[key]["backend"] for key in cfg["models"]}, {"openvino"})
        self.assertTrue(cfg["smoking"]["profile_refinement"]["enabled"])
        self.assertTrue(cfg["smoking"]["skip_unobservable"])
        self.assertTrue(cfg["fight"]["capture_buffer"] and cfg["fight"]["async_inference"])
        self.assertEqual(cfg["fight"]["result_ttl_seconds"], 2.0)
        self.assertEqual(self.cfg["fall"]["max_observation_gap_seconds"], .3)
        self.assertEqual(cfg["fight"]["max_sample_gap_seconds"], 1.0)
        self.assertEqual(cfg["smoking"]["result_ttl_seconds"], 2.0)
        self.assertIn("smoking", self.cfg)

    def test_result_has_no_smoking_or_fake_fall_score(self):
        result = self.frame(0)
        self.assertEqual(set(result["persons"][0]["actions"]), {"fall"})
        fall = result["persons"][0]["actions"]["fall"]
        self.assertIsNone(fall["score"])
        self.assertEqual(fall["score_kind"], "rule_evidence")
        self.assertEqual(result["source_epoch"], "source-1")
        self.assertEqual(result["stamp_ns"], 0)
        self.assertEqual(result["pose_status"], {"state": "valid", "person_count": 1})

    def test_half_visible_body_is_unknown(self):
        self.backend.kind = "half_body"
        fall = self.frame(0)["persons"][0]["actions"]["fall"]
        self.assertEqual(fall["state"], "unknown")
        self.assertIsNone(fall["label"])

    def test_lying_requires_repeated_observation_not_confidence(self):
        self.backend.kind = "lying"
        self.assertNotEqual(self.frame(0)["persons"][0]["actions"]["fall"]["state"], "valid")
        self.frame(.2)
        fall = self.frame(.4)["persons"][0]["actions"]["fall"]
        self.assertEqual((fall["label"], fall["state"]), ("lying", "valid"))
        self.assertIsNone(fall["score"])

    def test_sparse_lying_does_not_accumulate_confirmation(self):
        self.backend.kind = "lying"
        for stamp in (0, .4, .8, 1.2, 1.6):
            fall = self.frame(stamp)["persons"][0]["actions"]["fall"]
            self.assertNotEqual(fall["state"], "valid")

    def test_fight_is_scene_level_without_people(self):
        self.backend.count = 0
        for index in range(8):
            result = self.frame(index * .25)
        self.assertEqual(self.backend.fight_calls, 1)
        self.assertEqual(result["persons"], [])
        self.assertEqual(result["scene"]["fight"]["state"], "valid")
        self.assertEqual(result["pose_status"], {"state": "valid", "person_count": 0})

    def test_fight_sample_gap_does_not_reuse_old_window(self):
        for index in range(7):
            self.frame(index * .25)
        result = self.frame(2.6)
        self.assertEqual(self.backend.fight_calls, 0)
        self.assertEqual(result["scene"]["fight"]["window"]["samples"], 1)

    def test_source_reset_clears_pending_lying_and_fight(self):
        self.backend.kind = "lying"
        self.frame(0)
        self.frame(.25)
        result = self.frame(.5, "source-2")
        self.assertEqual(result["reset_reason"], "source_epoch_changed")
        self.assertEqual(result["scene"]["fight"]["window"]["samples"], 1)
        self.assertNotEqual(result["persons"][0]["actions"]["fall"]["state"], "valid")

    def test_backward_time_and_stream_gap_reset(self):
        self.frame(1)
        self.assertEqual(self.frame(.9)["reset_reason"], "timestamp_nonincreasing")
        self.assertEqual(self.frame(2.1)["reset_reason"], "stream_gap")

    def test_branch_disable_skips_actual_work_and_reenable_resets(self):
        for index in range(8):
            self.frame(index * .25, branches={"fall": False, "fight": True})
        self.assertEqual(self.backend.pose_calls, 0)
        self.assertEqual(self.backend.fight_calls, 1)
        result = self.frame(2., branches={"fall": True, "fight": False})
        self.assertEqual(result["reset_reason"], "branches_changed")
        self.assertEqual(result["scene"]["fight"]["state"], "disabled")
        self.assertEqual(self.backend.fight_calls, 1)
        self.assertEqual(self.backend.pose_calls, 1)
        result = self.frame(2.25)
        self.assertEqual(result["scene"]["fight"]["window"]["samples"], 1)

    def test_config_disabled_branch_cannot_be_enabled_by_header(self):
        self.cfg["branches"]["fight"] = False
        result = self.frame(0, branches={"fall": True, "fight": True})
        self.assertFalse(result["enabled_branches"]["fight"])
        self.assertEqual(self.backend.preprocess_calls, 0)

    def test_weak_fight_positive_unknown(self):
        window = TimeWindow(self.cfg["fight"])
        action = fight_classification((1, .79), self.cfg["fight"], 0, window)
        self.assertEqual(action["state"], "unknown")
        self.assertIsNone(action["label"])
        self.assertIsNone(action["score"])


class ConfigAndBackendTests(unittest.TestCase):
    def test_single_branch_workers_load_only_owned_model(self):
        for branch in ('fall', 'fight'):
            loaded = []
            class Pose:
                version = 'test'
                session = types.SimpleNamespace(get_providers=lambda: ['CPUExecutionProvider'], get_provider_options=lambda: {})
                def __init__(self, *args): loaded.append('pose')
            class Fight:
                def __init__(self, *args, **kwargs): loaded.append('fight')
            modules = {'paddle': types.SimpleNamespace(enable_static=lambda: None, __version__='test'),
                       'python.preprocess': types.SimpleNamespace(ShortSizeScale=lambda size: lambda rgb: rgb),
                       'pphuman.video_action_infer': types.SimpleNamespace(VideoActionRecognizer=Fight)}
            with tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                (root / 'vendor/PaddleDetection/deploy').mkdir(parents=True)
                (root / 'pose.onnx').write_bytes(b'fake test model')
                cfg = config()
                cfg.update(project_root=str(root), models={'yolo_pose': 'pose.onnx', 'fight': 'fight'},
                           branches={key: key == branch for key in ('fall', 'fight', 'smoking')})
                cfg["yolo_pose"]["backend"] = "onnxruntime"
                cfg["fight"]["backend"] = "paddle"
                with patch.dict(sys.modules, modules), patch('pphuman_runtime.yolo_backend.YoloPosePredictor', Pose), patch('pphuman_runtime.yolo_backend.model_identity', return_value={}):
                    backend = YoloBackend(cfg)
            self.assertEqual(loaded, ['pose'] if branch == 'fall' else ['fight'])
            self.assertEqual(set(backend.identities), {'yolo_pose'} if branch == 'fall' else {'fight'})

    def test_invalid_configs_rejected(self):
        original = config()
        for key, value in (("device", "GPU"), ("models", {"yolo_pose": "a", "fight": "b", "unexpected": "c"}),
                           ("branches", {"fall": 1, "fight": True}), ("cpu_threads", 0)):
            cfg = copy.deepcopy(original)
            cfg[key] = value
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "inference.yaml"
                path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_config(path)

    def test_backend_constructs_only_pose_and_scene_fight(self):
        loaded = []
        class Pose:
            version = "test"
            session = types.SimpleNamespace(get_providers=lambda: ["CPUExecutionProvider"], get_provider_options=lambda: {})
            def __init__(self, *args):
                loaded.append("pose")
        class Fight:
            def __init__(self, *args, **kwargs):
                self.kwargs = kwargs
                loaded.append("fight")
        modules = {"paddle": types.SimpleNamespace(enable_static=lambda: None, __version__="test"),
                   "python.preprocess": types.SimpleNamespace(ShortSizeScale=lambda size: lambda rgb: rgb),
                   "pphuman.video_action_infer": types.SimpleNamespace(VideoActionRecognizer=Fight)}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "vendor/PaddleDetection/deploy").mkdir(parents=True)
            (root / "pose.onnx").write_bytes(b"fake test model")
            cfg = config()
            cfg["project_root"] = str(root)
            cfg["models"] = {"yolo_pose": "pose.onnx", "fight": "fight"}
            cfg["yolo_pose"]["backend"] = "onnxruntime"
            cfg["fight"]["backend"] = "paddle"
            with patch.dict(sys.modules, modules), patch("pphuman_runtime.yolo_backend.YoloPosePredictor", Pose), patch("pphuman_runtime.yolo_backend.model_identity", return_value={}):
                backend = YoloBackend(cfg)
            self.assertEqual(loaded, ["pose", "fight"])
            self.assertIsNone(backend.smoking_model)
            self.assertTrue(callable(backend.smoking))  # Adapter exists, disabled predictor was not loaded.
            self.assertEqual(backend.fight_model.kwargs["device"], "CPU")
            self.assertEqual(backend.fight_model.kwargs["cpu_threads"], cfg["cpu_threads"])


class ProtocolTests(unittest.TestCase):
    def packet(self, header, payload=b""):
        stream = io.BytesIO()
        write_message(stream, header)
        stream.write(payload)
        stream.seek(0)
        return stream

    def test_worker_ready_reset_result_shutdown_contract(self):
        cfg = config()
        runtime = RuntimeYolo(cfg, FakeBackend())
        incoming, outgoing = io.BytesIO(), io.BytesIO()
        write_message(incoming, {"op": "reset", "request_id": "reset-2", "source_epoch": 2})
        frame = np.zeros((400, 400, 3), np.uint8)
        write_message(incoming, {"op": "infer", "request_id": 3, "source_epoch": 2, "stamp_ns": 100,
                                 "shape": list(frame.shape), "payload_bytes": frame.nbytes})
        incoming.write(frame.tobytes())
        write_message(incoming, {"op": "shutdown", "request_id": 4, "source_epoch": 2})
        incoming.seek(0)
        serve(runtime, cfg, incoming, outgoing)
        outgoing.seek(0)
        ready, reset, result, shutdown = [read_message(outgoing) for _ in range(4)]
        self.assertEqual([r["type"] for r in (ready, reset, result, shutdown)], ["ready", "reset_ack", "result", "shutdown_ack"])
        self.assertEqual(set(ready["models"]), {"yolo_pose", "fight"})
        self.assertEqual(set(ready["sampling"]), {"fall", "fight", "smoking"})
        self.assertEqual((result["source_epoch"], result["stamp_ns"], result["request_id"]), (2, 100, 3))
        self.assertEqual(shutdown["request_id"], 4)

    def test_invalid_branch_values_and_truncated_payload_rejected(self):
        header = {"op": "infer", "request_id": 1, "source_epoch": 1, "stamp_ns": 0, "shape": [1, 1, 3], "payload_bytes": 3}
        for switches in ({"unexpected": True}, {"fall": "false"}, []):
            with self.assertRaises(ProtocolError):
                read_request(self.packet(dict(header, enabled_branches=switches), b"123"))
        with self.assertRaises(ProtocolError):
            read_request(self.packet(header, b"12"))


class ModelLoadCheckTests(unittest.TestCase):
    def test_owned_cpu_worker_ready_checked_without_camera(self):
        from pphuman_runtime import self_check
        def factory(command, **kwargs):
            self.assertEqual(command[-2:], ['--config', 'config/inference.yaml'])
            self.assertEqual(kwargs['stdin'], self_check.subprocess.DEVNULL)
            self.assertEqual(kwargs['env']['CUDA_VISIBLE_DEVICES'], '')
            write_message(kwargs['stdout'], {'type': 'ready', 'runtime': {'device': 'CPU'},
                                            'models': {'yolo_pose': {}, 'fight': {}, 'smoking': {}}})
            return types.SimpleNamespace(returncode=0, wait=lambda timeout: None)
        with patch.object(self_check.subprocess, 'Popen', side_effect=factory):
            result = self_check.check(timeout=1)
        self.assertEqual(result['models'], ['yolo_pose', 'fight', 'smoking'])

    def test_initialization_timeout_kills_only_owned_child(self):
        from unittest.mock import Mock
        from pphuman_runtime import self_check
        process = Mock()
        process.wait.side_effect = [self_check.subprocess.TimeoutExpired('worker', 1), None]
        with patch.object(self_check.subprocess, 'Popen', return_value=process):
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                self_check.check(timeout=1)
        process.kill.assert_called_once()


if __name__ == "__main__":
    unittest.main()
