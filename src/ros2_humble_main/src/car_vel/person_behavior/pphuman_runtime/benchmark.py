"""Bounded, file-only CPU benchmark. Does not access cameras or ROS.

Default playback follows media time and drops frames missed during inference.
--mode offline processes every frame and measures capacity separately. Neither
mode duplicates temporal samples. State coverage is not detection accuracy.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

from .protocol import read_message, write_message


def clean_environment():
    env = dict(os.environ)
    for name in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        env.pop(name, None)
    env.update(PYTHONNOUSERSITE="1", OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="2", MKL_NUM_THREADS="2")
    return env


def summaries(values):
    import numpy as np
    return {"count": len(values), "mean_ms": float(np.mean(values)),
            "p50_ms": float(np.percentile(values, 50)), "p95_ms": float(np.percentile(values, 95)),
            "max_ms": float(max(values))} if values else {"count": 0}


def run(args):
    import cv2
    import numpy as np
    root = Path(__file__).resolve().parents[1]
    video = args.video.resolve(strict=True)
    if not video.is_file():
        raise ValueError("--video must be a local file; cameras and URLs are not supported")
    output = args.output.resolve()
    if output == video or output.with_suffix(output.suffix + ".worker.log") == video:
        raise ValueError("Benchmark output must not overwrite the input video")
    output.parent.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError("Cannot open video file")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0:
        capture.release()
        raise ValueError("Video has no usable frame-rate metadata")
    timing = defaultdict(list)
    selected = {'fall', 'fight'} if args.branches == 'both' else (
        {'fall', 'fight', 'smoking'} if args.branches == 'all' else {args.branches})
    coverage = {name: Counter() for name in ("fall", "fight", "smoking")}
    reasons = {name: Counter() for name in coverage}
    report = {"schema_version": 1, "video": str(video), "mode": args.mode,
              "fps_metadata": fps, "inference_device": "CPU", "cpu_threads": 2,
              "limits": {key: getattr(args, key) for key in ("max_frames", "max_seconds", "wall_timeout", "request_timeout")},
              "coverage_semantics": "Counts of observed branch states; not accuracy, recall, or false-positive rate.",
              "timestamp_policy": "Media POS_MSEC when monotonic; otherwise frame index / metadata FPS. No synthetic duplicate samples.",
              "frames_inferred": 0, "frames_dropped": 0, "timestamp_fallback_frames": 0}
    responses = queue.Queue()
    process = None
    started = time.monotonic()
    deadline = started + args.wall_timeout
    error = None

    def receive(timeout):
        remaining = min(timeout, deadline - time.monotonic())
        if remaining <= 0:
            raise TimeoutError("Overall benchmark wall-time limit reached")
        try:
            response = responses.get(timeout=remaining)
        except queue.Empty as exc:
            raise TimeoutError("Worker reply exceeded the benchmark deadline") from exc
        if isinstance(response, Exception):
            raise response
        if response is None:
            raise RuntimeError("Worker closed its response stream")
        return response

    def send(header, payload=b""):
        errors = queue.Queue()

        def write():
            try:
                write_message(process.stdin, header)
                if payload:
                    process.stdin.write(payload)
                    process.stdin.flush()
                errors.put(None)
            except Exception as exc:
                errors.put(exc)

        threading.Thread(target=write, daemon=True).start()
        try:
            failure = errors.get(timeout=max(.001, min(args.request_timeout, deadline - time.monotonic())))
        except queue.Empty as exc:
            raise TimeoutError("Worker input exceeded the benchmark deadline") from exc
        if failure:
            raise failure

    try:
        log_path = output.with_suffix(output.suffix + ".worker.log")
        with log_path.open("wb") as worker_log:
            process = subprocess.Popen([sys.executable, "-s", "-m", "pphuman_runtime.worker",
                                        "--config", "config/inference.yaml", "--branches", ','.join(sorted(selected))], cwd=str(root),
                                       env=clean_environment(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=worker_log, bufsize=0)

            def reader():
                try:
                    while True:
                        response = read_message(process.stdout)
                        responses.put(response)
                        if response is None:
                            return
                except Exception as exc:
                    responses.put(exc)

            threading.Thread(target=reader, daemon=True).start()
            ready = receive(args.startup_timeout)
            if ready.get("type") != "ready":
                raise RuntimeError("Worker did not send ready")
            report["ready"] = ready
            report["startup_seconds"] = time.monotonic() - started
            playback_started = time.monotonic()
            decoded = 0
            last_media = -1.0
            last_sent_ns = None
            while report["frames_inferred"] < args.max_frames:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Overall benchmark wall-time limit reached")
                ok, frame = capture.read()
                if not ok:
                    report["stop_reason"] = "end_of_file"
                    break
                media = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                if not math.isfinite(media) or media < 0 or media <= last_media:
                    media = decoded / fps
                    report["timestamp_fallback_frames"] += 1
                if media <= last_media:
                    raise ValueError("Unreliable video timestamps; refusing to fabricate continuous history")
                decoded += 1
                last_media = media
                if media >= args.max_seconds:
                    report["stop_reason"] = "media_duration_limit"
                    break
                if args.mode == "realtime":
                    target = time.monotonic() - playback_started
                    if media + 1.0 / fps < target:
                        report["frames_dropped"] += 1
                        continue
                    if media > target:
                        time.sleep(min(media - target, max(0.0, deadline - time.monotonic())))
                if args.max_width and frame.shape[1] > args.max_width:
                    height = max(1, round(frame.shape[0] * args.max_width / frame.shape[1]))
                    frame = cv2.resize(frame, (args.max_width, height), interpolation=cv2.INTER_AREA)
                frame = np.ascontiguousarray(frame, dtype=np.uint8)
                payload = frame.tobytes()
                stamp_ns = round(media * 1_000_000_000)
                if last_sent_ns is not None:
                    timing["capture_gap"].append((stamp_ns - last_sent_ns) / 1e6)
                last_sent_ns = stamp_ns
                header = {"op": "infer", "request_id": report["frames_inferred"], "source_epoch": "benchmark-file-1",
                          "stamp_ns": stamp_ns, "shape": list(frame.shape), "payload_bytes": len(payload),
                          "enabled_branches": {key: key in selected for key in coverage}}
                inference_started = time.monotonic()
                send(header, payload)
                result = receive(args.request_timeout)
                if result.get("type") != "result" or result.get("request_id") != header["request_id"]:
                    raise RuntimeError("Unexpected worker response")
                timing["roundtrip"].append((time.monotonic() - inference_started) * 1000)
                for name, value in result.get("timing_ms", {}).items():
                    timing[name].append(value)
                report["frames_inferred"] += 1
                report["media_seconds_observed"] = media
                fight = result["scene"]["fight"]
                actions = {"fight": [fight], **{key: [p['actions'].get(key, {'state': 'disabled'}) for p in result['persons']]
                                               for key in ('fall', 'smoking')}}
                for key in ('fall', 'smoking'):
                    if not actions[key]:
                        coverage[key]["no_person" if key in selected else "disabled"] += 1
                for branch, entries in actions.items():
                    for action in entries:
                        coverage[branch][action["state"]] += 1
                        if action.get("reason"):
                            reasons[branch][action["reason"]] += 1
                report["last_fight_window"] = fight.get("window")
            else:
                report["stop_reason"] = "frame_count_limit"
            report["processing_wall_seconds"] = time.monotonic() - playback_started
            send({"op": "shutdown", "request_id": "benchmark-end", "source_epoch": "benchmark-file-1"})
            if receive(min(5., args.request_timeout)).get("type") != "shutdown_ack":
                raise RuntimeError("Missing shutdown acknowledgement")
    except Exception as exc:
        error = str(exc)
        report["error"] = error
        report["stop_reason"] = "error_or_timeout"
    finally:
        capture.release()
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            report["worker_exit_code"] = process.returncode
        report["wall_seconds"] = time.monotonic() - started
        report["timing"] = {name: summaries(values) for name, values in timing.items()}
        report["coverage_counts"] = {name: dict(counts) for name, counts in coverage.items()}
        report["unknown_or_warming_reasons"] = {name: dict(counts) for name, counts in reasons.items()}
        for name, counts in coverage.items():
            total = sum(counts.values())
            report.setdefault("valid_fraction", {})[name] = counts["valid"] / total if total else None
        report["assessment"] = ("Temporal coverage is insufficient for one or both enabled branches; inspect unknown reasons and capture gaps."
            if any(name in selected and report["valid_fraction"][name] in (None, 0) for name in coverage)
            else "This is a performance/state-coverage result only; labeled-video accuracy validation is still required.")
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps({"report": str(output), "frames_inferred": report["frames_inferred"],
                          "coverage_counts": report["coverage_counts"], "error": error}, ensure_ascii=False))
    return 1 if error else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", choices=("realtime", "offline"), default="realtime")
    parser.add_argument("--branches", choices=("all", "both", "fall", "fight", "smoking"), default="all")
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--max-seconds", type=float, default=20.)
    parser.add_argument("--wall-timeout", type=float, default=180.)
    parser.add_argument("--startup-timeout", type=float, default=60.)
    parser.add_argument("--request-timeout", type=float, default=30.)
    parser.add_argument("--max-width", type=int, default=1280)
    args = parser.parse_args()
    if not 1 <= args.max_frames <= 10000 or not 0 < args.max_seconds <= 3600 or not 0 < args.wall_timeout <= 7200:
        parser.error("Frame/duration/wall-time bounds exceeded")
    if not 0 < args.startup_timeout <= 300 or not 0 < args.request_timeout <= 300 or not 1 <= args.max_width <= 4096:
        parser.error("Invalid timeout or frame width")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
