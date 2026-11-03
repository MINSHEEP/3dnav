"""python -m pphuman_runtime.worker --config config/inference.yaml"""
import argparse
import os
import sys
import traceback

from .protocol import read_request, write_message


def serve(runtime, cfg, input_stream, output_stream):
    write_message(output_stream, {"type": "ready", "protocol_version": 1,
                                  "models": runtime.backend.identities,
                                  "runtime": runtime.backend.metadata,
                                  "branches": cfg["branches"],
                                  "capabilities": ["fight_samples_rgb_v1"] if cfg["fight"].get("capture_buffer") else [],
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
        stamps = header.get("clip_stamps_ns")
        samples = header.get("fight_samples")
        if samples is not None:
            height, width, channels = header["shape"]
            frame_bytes = height * width * channels
            frame = np.frombuffer(payload, dtype=np.uint8, count=frame_bytes).reshape(header["shape"])
            capture = []
            offset = frame_bytes
            for sample in samples:
                h, w, c = sample["shape"]
                size = h * w * c
                image = np.frombuffer(payload, dtype=np.uint8, count=size, offset=offset).reshape(sample["shape"])
                capture.append((sample["stamp_ns"], image))
                offset += size
            result = runtime.infer(frame, header, fight_samples=capture)
        elif stamps is None:
            frame = np.frombuffer(payload, dtype=np.uint8).reshape(header["shape"])
            result = runtime.infer(frame, header)
        else:
            frames = np.frombuffer(payload, dtype=np.uint8).reshape([len(stamps), *header["shape"]])
            result = runtime.infer(frames[-1], header, fight_samples=list(zip(stamps, frames)))
        write_message(output_stream, result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--branch", choices=("fall", "fight", "smoking", "pose"))
    parser.add_argument("--branches", help="Comma-separated enabled branches within worker scope")
    args = parser.parse_args()
    # Native Paddle output must not corrupt the binary response stream.
    sys.stdout.flush()
    output = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr
    try:
        from .runtime_yolo import RuntimeYolo, load_config
        from .yolo_backend import YoloBackend
        cfg = load_config(args.config)
        if args.branch:
            scope = {"fall", "smoking"} if args.branch == "pose" else {args.branch}
            cfg["branches"] = {key: value and key in scope for key, value in cfg["branches"].items()}
        if args.branches is not None:
            selected = set(args.branches.split(","))
            if not selected or not selected <= set(cfg["branches"]):
                raise ValueError("Unknown enabled branch")
            cfg["branches"] = {key: value and key in selected for key, value in cfg["branches"].items()}
        import cv2
        cv2.setNumThreads(cfg.get("opencv_threads", cfg["cpu_threads"]))
        runtime = RuntimeYolo(cfg, YoloBackend(cfg))
        try:
            serve(runtime, cfg, sys.stdin.buffer, output)
        finally:
            runtime.close()
    except BrokenPipeError:
        return 1
    except Exception:
        traceback.print_exc(file=sys.stderr)
        return 1
    finally:
        output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
