"""Independent candidate: python -m pphuman_runtime.worker_yolo --config <yaml>."""
import argparse
import os
import sys
import traceback

from .worker import gpu_lease, serve


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="pphuman_runtime/inference_yolo.candidate.yaml")
    args = parser.parse_args()
    sys.stdout.flush()
    output = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr
    runtime = None
    try:
        from .runtime_yolo import RuntimeYolo, load_config
        from .yolo_backend import YoloBackend
        cfg = load_config(args.config)
        with gpu_lease(cfg):
            backend = YoloBackend(cfg)
            runtime = RuntimeYolo(cfg, backend)
            serve(runtime, cfg, sys.stdin.buffer, output)
    except BrokenPipeError:
        return 1
    except Exception:
        traceback.print_exc(file=sys.stderr)
        return 1
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            output.close()
    return 0


if __name__ == "__main__": raise SystemExit(main())
