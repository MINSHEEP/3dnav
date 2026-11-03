"""Optional CPU sessions; preserve existing FP32 tensor and classifier contracts."""
from types import SimpleNamespace
from pathlib import Path

import numpy as np


def disable_openvino_telemetry():
    # OpenVINO 2024.6 imports its converter, which otherwise starts a
    # telemetry HTTP thread and can hang offline interpreter shutdown.
    # Decline only in this process, before import; never edit the installed
    # environment or the user's machine-wide consent file.
    try:
        from openvino_telemetry.utils.opt_in_checker import ConsentCheckResult, OptInChecker
    except ImportError:
        pass  # OpenVINO already supplies a stub when telemetry is absent.
    else:
        OptInChecker.check = lambda *args, **kwargs: ConsentCheckResult.DECLINED


class OpenVINOSession:
    def __init__(self, path, threads=4):
        disable_openvino_telemetry()
        import openvino as ov

        if type(threads) is not int or not 1 <= threads <= 16:
            raise ValueError("OpenVINO CPU threads must be an integer in [1,16]")
        self.version, self.threads = ov.__version__, threads
        # Single stream bounds CPU use. Float layers remain FP32; the exported
        # quantizers/INT8 weights control low-precision convolutions in the graph.
        self.compiled = ov.Core().compile_model(str(path), "CPU", {
            "INFERENCE_NUM_THREADS": threads, "NUM_STREAMS": 1,
            "INFERENCE_PRECISION_HINT": "f32", "ENABLE_CPU_PINNING": False})
        self.request = self.compiled.create_infer_request()
        self.inputs = [self._describe(port) for port in self.compiled.inputs]
        self.outputs = [self._describe(port) for port in self.compiled.outputs]
        self.output_ports = {port.get_any_name(): port for port in self.compiled.outputs}

    @staticmethod
    def _describe(port):
        types = {"f32": "tensor(float)", "i64": "tensor(int64)", "i32": "tensor(int32)"}
        return SimpleNamespace(name=port.get_any_name(),
            shape=[d.get_length() if d.is_static else None for d in port.get_partial_shape()],
            type=types.get(port.get_element_type().get_type_name(), str(port.get_element_type())))

    def get_inputs(self):
        return self.inputs

    def get_outputs(self):
        return self.outputs

    def get_providers(self):
        return ["OpenVINO:CPU"]

    def get_provider_options(self):
        return {"OpenVINO:CPU": {"threads": str(self.threads), "streams": "1"}}

    def run(self, names, inputs):
        result = self.request.infer(inputs)
        # The next infer call reuses the request buffers; observations must own
        # their output arrays, including recovery rotations of the same image.
        return [np.array(result[self.output_ports[name]], copy=True) for name in names]

    def close(self):
        # Release native requests/models before Python interpreter finalization.
        # The owner must first finish/cancel any asynchronous request.
        self.request = None
        self.output_ports.clear()
        self.compiled = None


class OpenVINOFightPredictor:
    """Single-window Paddle handle adapter for the fixed ppTSM INT8 export.

    VideoActionRecognizer still owns preprocessing and softmax/top-1 decisions.
    Only its predictor is replaced, without loading a second Paddle predictor.
    """
    def __init__(self, path, threads=4, target_size=320):
        if target_size not in (224, 320):
            raise ValueError("Supported ppTSM spatial sizes are 224 and 320")
        self.expected_shape = (1, 8, 3, target_size, target_size)
        self.session = OpenVINOSession(path, threads)
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].shape != list(self.expected_shape)
                or inputs[0].type != "tensor(float)"
                or len(outputs) != 1 or outputs[0].shape != [1, 2]
                or outputs[0].type != "tensor(float)"):
            raise ValueError(f"OpenVINO fight requires FP32 {self.expected_shape} -> [1,2]")
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.input, self.output = None, None
        self.async_running = False

    def get_input_names(self):
        return [self.input_name]

    def get_output_names(self):
        return [self.output_name]

    def get_input_handle(self, name):
        if name != self.input_name:
            raise KeyError(name)
        return SimpleNamespace(copy_from_cpu=self._set_input)

    def get_output_handle(self, name):
        if name != self.output_name:
            raise KeyError(name)
        return SimpleNamespace(copy_to_cpu=self._get_output)

    def _set_input(self, data):
        if self.async_running:
            raise RuntimeError("A fight request is already running; no queued windows allowed")
        self.input, self.output = None, None
        value = np.asarray(data)
        # Upstream wraps the window in a list and adds singleton batch axes.
        while value.ndim > 5 and value.shape[0] == 1:
            value = value[0]
        if value.shape != self.expected_shape or value.dtype != np.float32:
            raise ValueError("Unexpected ppTSM window shape or dtype")
        self.input = np.ascontiguousarray(value)

    def run(self):
        if self.async_running:
            raise RuntimeError("Cannot run synchronously while fight request is busy")
        self.output = None
        if self.input is None:
            raise RuntimeError("No fight window supplied")
        result = self.session.run([self.output_name], {self.input_name: self.input})[0]
        if result.shape != (1, 2) or not np.isfinite(result).all():
            raise RuntimeError("Expected two finite ppTSM logits")
        self.output = result

    def start_async(self, data):
        self._set_input(data)
        # Copy into request-owned memory; the main pipeline may process new frames.
        self.session.request.start_async({self.input_name: self.input}, share_inputs=False)
        self.async_running = True

    def poll_async(self):
        if not self.async_running or not self.session.request.wait_for(0):
            return None
        result = np.array(self.session.request.get_output_tensor(0).data, copy=True)
        latency = float(self.session.request.latency)
        self.async_running = False
        self.input = None
        if result.shape != (1, 2) or not np.isfinite(result).all() or not np.isfinite(latency) or latency < 0:
            raise RuntimeError("Invalid asynchronous ppTSM output or latency")
        self.output = result
        return result.copy(), latency

    def close(self):
        if self.async_running:
            request = self.session.request
            if not request.wait_for(0):
                request.cancel()
                try:
                    request.wait()
                except RuntimeError:
                    pass  # OpenVINO reports cancellation through wait().
            self.async_running = False
        self.input, self.output = None, None

    def _get_output(self):
        if self.output is None:
            raise RuntimeError("No completed fight prediction")
        return self.output.copy()


class OpenVINOSmokingPredictor:
    """Static single-image FP32 export; retain every person's NMS output."""
    def __init__(self, path, threads=4, max_batch=8):
        self.session = OpenVINOSession(path, threads)
        expected = {"image": [1, 3, 640, 640], "scale_factor": [1, 2]}
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if ({p.name: p.shape for p in inputs} != expected or
                any(p.type != "tensor(float)" for p in inputs) or len(outputs) != 2 or
                len(outputs[0].shape) != 2 or outputs[0].shape[1] != 6 or
                outputs[0].type != "tensor(float)" or outputs[1].shape != [1]):
            raise ValueError("Expected static FP32 smoking inputs and boxes[N,6], counts[1]")
        self.max_batch, self.inputs, self.outputs = max_batch, {}, {}

    def get_input_names(self):
        return [p.name for p in self.session.get_inputs()]

    def get_output_names(self):
        return [p.name for p in self.session.get_outputs()]

    def get_input_handle(self, name):
        if name not in self.get_input_names():
            raise KeyError(name)
        def put(value):
            self.inputs[name] = np.asarray(value)
        return SimpleNamespace(copy_from_cpu=put)

    def get_output_handle(self, name):
        if name not in self.get_output_names():
            raise KeyError(name)
        return SimpleNamespace(copy_to_cpu=lambda: self.outputs[name].copy())

    def run(self):
        self.outputs = {}
        if set(self.inputs) != {"image", "scale_factor"}:
            raise ValueError("Both smoking inputs are required")
        images, scales = self.inputs["image"], self.inputs["scale_factor"]
        batch = len(images)
        if (not 1 <= batch <= self.max_batch or images.shape != (batch, 3, 640, 640)
                or scales.shape != (batch, 2) or images.dtype != np.float32 or scales.dtype != np.float32):
            raise ValueError("Invalid smoking batch shape or dtype")
        names = self.get_output_names()
        rows = []
        for index in range(batch):
            boxes, counts = self.session.run(names, {key: value[index:index+1] for key, value in self.inputs.items()})
            if (boxes.ndim != 2 or boxes.shape[1] != 6 or counts.shape != (1,)
                    or counts[0] != len(boxes) or not np.isfinite(boxes).all()):
                raise RuntimeError("Invalid OpenVINO smoking detector output")
            rows.append((boxes, counts))
        self.outputs = {name: np.concatenate([row[i] for row in rows], axis=0) for i, name in enumerate(names)}


def create_smoking_detector(directory, cfg):
    """Reuse pinned preprocessing/postprocessing without loading Paddle weights."""
    from python.infer import Detector, PredictConfig
    from python.utils import Timer

    detector = Detector.__new__(Detector)
    detector.pred_config = PredictConfig(str(directory))
    detector.predictor = OpenVINOSmokingPredictor(Path(directory) / "model.xml",
        cfg["smoking"].get("cpu_threads", cfg["cpu_threads"]), cfg["smoking"]["batch_size"])
    detector.config = None
    detector.det_times = Timer()
    detector.cpu_mem = detector.gpu_mem = detector.gpu_util = 0
    detector.batch_size = cfg["smoking"]["batch_size"]
    detector.output_dir, detector.threshold, detector.device = "output", cfg["smoking"]["threshold"], "CPU"
    return detector


def create_fight_recognizer(path, cfg):
    """Reuse pinned upstream methods without requiring the old FP32 weights.

    Its constructor only configures a Paddle engine and the fields below. The
    original predict/preprocess/postprocess methods run on this CPU predictor.
    """
    from python.utils import Timer
    from pphuman.video_action_infer import VideoActionRecognizer

    recognizer = VideoActionRecognizer.__new__(VideoActionRecognizer)
    recognizer.num_seg, recognizer.seg_len = cfg["fight"]["window_frames"], 1
    recognizer.short_size, recognizer.target_size = cfg["fight"]["short_size"], cfg["fight"]["target_size"]
    recognizer.batch_size, recognizer.top_k = 1, 1
    recognizer.recognize_times = Timer()
    recognizer.predictor = OpenVINOFightPredictor(path, cfg["fight"].get("cpu_threads", cfg["cpu_threads"]),
                                                cfg["fight"]["target_size"])
    return recognizer
