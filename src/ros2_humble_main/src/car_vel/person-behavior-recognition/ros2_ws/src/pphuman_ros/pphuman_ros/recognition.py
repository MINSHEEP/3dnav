import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import IMAGE_QOS, decode_image, image_message, json_message, node_lease, parameter, project_root
from .core import LatestFrames, WorkerClient, expire_result, sampling_ttls_ns
from .overlay import Overlay
from .preview import PreviewPipeline, Rates


class Recognition(Node):
    def __init__(self):
        super().__init__('pphuman_recognition')
        self.lease = node_lease('recognition')
        p = lambda name, default: parameter(self, name, default)
        self.image_topic = p('image_topic', '/pp_human/image_raw')
        self.results = self.create_publisher(String, p('result_topic', '/pp_human/results'), 1)
        self.annotated = self.create_publisher(Image, p('annotated_topic', '/pp_human/annotated_image'), IMAGE_QOS)
        self.status_pub = self.create_publisher(String, p('status_topic', '/pp_human/status'), 1)
        self.preview_status_pub = self.create_publisher(String, p('preview_status_topic', '/pp_human/preview_status'), 1)
        self.root = project_root()
        self.python = p('worker_python', '/home/a/anaconda3/envs/pp_humanv2/bin/python')
        self.worker_module = p('worker_module', 'pphuman_runtime.worker')
        self.worker_config = p('worker_config', 'config/inference.yaml')
        self.worker_env_script = p('worker_env_script', 'environment/runtime_env.sh')
        self.fps = max(.1, float(p('processing_fps', 10.0)))
        self.frames = LatestFrames(max(.2, float(p('stream_timeout_seconds', 2.0))))
        self.request_timeout = float(p('worker_timeout_seconds', 30.0))
        self.startup_timeout = float(p('worker_startup_seconds', 180.0))
        self.max_age = float(p('result_max_age_seconds', 5.0))
        self.prediction_age_ns = int(float(p('prediction_max_age_seconds', 5.0)) * 1e9)
        self.restart_seconds = float(p('restart_seconds', 5.0))
        self.preview = bool(p('preview', True))
        self.preview_fps = max(1, min(60, float(p('preview_fps', 20.0))))
        self.preview_max_age = max(.01, float(p('preview_overlay_max_age_seconds', .35)))
        self.preview_max_width = max(0, int(p('preview_max_width', 0)))
        self.overlay = Overlay(p('font_path', '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')) if self.preview else None
        self.rates = Rates()
        self.preview_pipeline = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.completed = None
        self.worker_state = 'starting'
        self.worker_error = ''
        self.models = {}
        self.runtime_identity = {}
        self.last_result_time = None
        self.last_frame_identity = (0, '')
        self.last_frame_shape = (360, 640, 3)
        self.last_preview_time = 0
        self.last_status_time = 0
        self.invalid_key = None
        self.display_epoch = None
        self.started = time.monotonic()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, IMAGE_QOS)
        self.timer = self.create_timer(.05, self.tick)
        self.thread = threading.Thread(target=self.inference_loop, name='pphuman-worker-io', daemon=True)
        self.thread.start()
        if self.overlay:
            self.preview_pipeline = PreviewPipeline(self.overlay, self.publish_preview, self.frames.snapshot,
                self.rates, fps=self.preview_fps, max_age=self.preview_max_age,
                prediction_age_ns=self.prediction_age_ns, max_width=self.preview_max_width,
                on_error=self.preview_error)
            self.preview_pipeline.start()

    def publish_preview(self, image, stamp_ns, frame_id, meta):
        if self.stop.is_set() or not self.context.ok():
            return
        try:
            self.annotated.publish(image_message(image, stamp_ns, frame_id))
            self.preview_status_pub.publish(json_message(dict(type='preview_status', **meta)))
        except Exception:
            if self.context.ok() and not self.stop.is_set():
                raise

    def preview_error(self, error):
        if self.context.ok() and not self.stop.is_set():
            self.get_logger().error('预览渲染异常: ' + error)

    def on_image(self, msg):
        if self.count_publishers(self.image_topic) > 1:
            self.frames.invalidate('multiple_image_publishers')
            return
        try:
            image = decode_image(msg)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            frame = self.frames.put(image, stamp_ns, msg.header.frame_id)
            self.last_frame_identity = (stamp_ns, msg.header.frame_id)
            self.last_frame_shape = tuple(image.shape)
            rates = getattr(self, 'rates', None)
            if rates:
                rates.mark('input')
            pipeline = getattr(self, 'preview_pipeline', None)
            if pipeline:
                pipeline.submit_frame(frame)
        except (ValueError, TypeError) as exc:
            self.frames.invalidate('invalid_image')
            self.get_logger().error(str(exc))

    def worker_status(self, state, error=''):
        with self.lock:
            self.worker_state, self.worker_error = state, error

    def inference_loop(self):
        # The GPU worker owns run/gpu.lock. Do not lock it twice in the ROS parent.
        command = [self.python, '-m', self.worker_module, '--config', self.worker_config]
        env_script = self.root / self.worker_env_script
        if self.worker_env_script and env_script.is_file():
            # Source only in the model child. No Conda CUDA libraries enter ROS.
            command = ['bash', '-c', 'source "$1"; shift; exec "$@"',
                       'pphuman-model', str(env_script)] + command
        while not self.stop.is_set():
            client = WorkerClient(command, str(self.root), self.startup_timeout, self.request_timeout, self.stop)
            try:
                self.worker_status('loading')
                ready = client.start()
                action_ttls_ns = sampling_ttls_ns(ready.get('sampling'))
                with self.lock:
                    self.models = ready.get('models', {})
                    self.runtime_identity = ready.get('runtime', {})
                self.frames.invalidate('worker_ready')
                self.worker_status('ready')
                sent_epoch = None
                next_frame_time = 0
                while not self.stop.is_set():
                    epoch, connected, _, _ = self.frames.snapshot()
                    if epoch != sent_epoch:
                        client.reset(epoch)
                        sent_epoch = epoch
                    if not connected or time.monotonic() < next_frame_time:
                        self.stop.wait(.02)
                        continue
                    frame = self.frames.take()
                    if frame is None:
                        self.stop.wait(.02)
                        continue
                    # Epoch may change between snapshot and take; send reset first.
                    if frame.epoch != sent_epoch:
                        client.reset(frame.epoch)
                        sent_epoch = frame.epoch
                    self.worker_status('inferencing')
                    result = client.infer(frame)
                    self.worker_status('ready')
                    next_frame_time = max(next_frame_time + 1 / self.fps, time.monotonic())
                    epoch, connected, _, _ = self.frames.snapshot()
                    if frame.epoch != epoch or not connected or time.monotonic() - frame.received > self.max_age:
                        continue
                    result = expire_result(result, frame.stamp_ns, self.prediction_age_ns, action_ttls_ns)
                    with self.lock:
                        self.completed = (frame, result, ready.get('models', {}), action_ttls_ns)
            except InterruptedError:
                break
            except Exception as exc:
                self.worker_status('error', f'{type(exc).__name__}: {exc}')
                self.frames.invalidate('worker_failed')
                self.get_logger().error(f'模型进程失败，将重建：{exc}')
            finally:
                client.close()
            self.stop.wait(self.restart_seconds)

    def tick(self):
        now = time.monotonic()
        epoch, connected, reason, dropped = self.frames.snapshot(now)
        if epoch != self.display_epoch:
            self.last_result_time = None
            self.display_epoch = epoch
        with self.lock:
            completed, self.completed = self.completed, None
            state, error, models = self.worker_state, self.worker_error, self.models
            runtime_identity = self.runtime_identity
        if completed:
            frame, result, models, action_ttls_ns = completed
            if frame.epoch == epoch and connected and now - frame.received <= self.max_age:
                action_states = [a.get('state', 'unknown') for person in result.get('persons', [])
                                 for a in person.get('actions', {}).values() if isinstance(a, dict)]
                action_states += [a.get('state', 'unknown') for a in result.get('scene', {}).values() if isinstance(a, dict)]
                result.update(schema_version=1, frame_id=frame.frame_id, models=models, runtime=runtime_identity,
                              source_connected=True, valid=bool(action_states) and all(s == 'valid' for s in action_states),
                              validity='valid' if action_states and all(s == 'valid' for s in action_states) else 'partial_or_warming',
                              latency_ms=round((now - frame.received) * 1000, 2))
                self.results.publish(json_message(result))
                self.last_result_time = now
                self.invalid_key = None
                self.rates.mark('processing', now)
                if self.preview_pipeline:
                    self.preview_pipeline.submit_result(frame, result, action_ttls_ns)
        stale = self.last_result_time is None or now - self.last_result_time > self.max_age
        if not connected or stale or state in ('error', 'loading'):
            invalid_reason = reason if not connected else state if state in ('error', 'loading') else 'result_expired'
            key = (epoch, invalid_reason)
            if self.invalid_key != key:
                stamp_ns, frame_id = self.last_frame_identity
                self.results.publish(json_message(dict(type='result', schema_version=1, request_id=None,
                    stamp_ns=stamp_ns, frame_id=frame_id, source_epoch=epoch, source_connected=connected,
                    valid=False, validity='unknown', status=invalid_reason, persons=[], models=models, runtime=runtime_identity,
                    timing_ms={'bridge': 0.0},
                    scene={'fight': {'label': None, 'score': None, 'state': 'unknown', 'updated': False,
                                     'prediction_stamp_ns': None}})))
                if self.overlay and getattr(self, 'preview_pipeline', None) is None:
                    self.annotated.publish(image_message(self.overlay.unavailable(
                        shape=self.last_frame_shape, text='等待有效结果／断流后重新预热'), stamp_ns, frame_id))
                self.invalid_key = key
        if now - self.last_status_time >= .5:
            rates = getattr(self, 'rates', None)
            pipeline = getattr(self, 'preview_pipeline', None)
            self.status_pub.publish(json_message(dict(type='status', component='recognition', source_epoch=epoch,
                source_connected=connected, reason=reason, worker_state=state, error=error, models=models,
                dropped_pending_frames=dropped, image_topic=self.image_topic, buffered_frames=1,
                max_in_flight=1, result_expired=stale,
                rates=rates.snapshot(now) if rates else {}, preview=pipeline.diagnostics() if pipeline else {})))
            self.last_status_time = now

    def destroy_node(self):
        self.stop.set()
        if self.preview_pipeline:
            self.preview_pipeline.close()
        self.thread.join(timeout=8)
        self.lease.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Recognition()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
