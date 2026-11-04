"""python -m pphuman_runtime.worker --config config/inference.yaml"""

import argparse
import contextlib
import os
import sys
import traceback
from pathlib import Path

from .protocol import read_request, write_message


@contextlib.contextmanager
def gpu_lease(cfg):
    if cfg["device"].upper() != "GPU":
        yield
        return
    import fcntl
    path = Path(cfg["project_root"]) / cfg["gpu_lock"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Project GPU is already in use: " + str(path)) from exc
        try:
            yield
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def serve(runtime, cfg, input_stream, output_stream):
    write_message(output_stream, {"type": "ready", "protocol_version": 1,
                                  "models": runtime.backend.identities,
                                  "runtime": runtime.backend.metadata,
                                  "sampling": {k: cfg[k] for k in ("fall", "fight", "smoking")}})
    import numpy as np
    limits = {key: cfg["stream"][key] for key in (
        "max_header_bytes", "max_payload_bytes", "max_dimension")}
    while True:
        packet = read_request(input_stream, **limits)
        if packet is None:
            return
        header, payload = packet
        if header["op"] == "shutdown":
            write_message(output_stream, {"type": "shutdown_ack", "request_id": header.get("request_id"),
                                          "source_epoch": header.get("source_epoch")})
            return
        if header["op"] == "reset":
            runtime.reset()
            write_message(output_stream, {"type": "reset_ack", "request_id": header.get("request_id"),
                                          "source_epoch": header.get("source_epoch"),
                                          "generation": runtime.generation})
            continue
        frame = np.frombuffer(payload, dtype=np.uint8).reshape(header["shape"])
        write_message(output_stream, runtime.infer(frame, header))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    # Redirect descriptor 1 too: Paddle/native libraries may bypass Python print.
    sys.stdout.flush()
    output = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr
    runtime = None
    try:
        from .runtime import Runtime, load_config
        from .backend import PaddleBackend
        cfg = load_config(args.config)
        if cfg.get("profile") == "yolo_pose_rules_paddle_aux_v1":
            from .runtime_yolo import RuntimeYolo as Runtime
            from .yolo_backend import YoloBackend as PaddleBackend
        with gpu_lease(cfg):
            backend = PaddleBackend(cfg)
            runtime = Runtime(cfg, backend)
            serve(runtime, cfg, sys.stdin.buffer, output)
    except BrokenPipeError:
        return 1
    except Exception:
        traceback.print_exc(file=sys.stderr)
        return 1
    finally:
        try:
            if runtime is not None and hasattr(runtime, "close"):
                runtime.close()
        finally:
            output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
