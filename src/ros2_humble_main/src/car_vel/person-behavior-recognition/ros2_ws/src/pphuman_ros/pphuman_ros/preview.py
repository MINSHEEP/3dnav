"""Bounded latest-frame preview, independent of inference and the ROS executor."""
from collections import deque
import copy
import threading
import time

import cv2

from .core import expire_result


class Rates:
    def __init__(self):
        self.lock = threading.Lock()
        self.events = {name: deque(maxlen=256) for name in ('input', 'processing', 'preview')}

    def mark(self, name, now=None):
        with self.lock:
            self.events[name].append(time.monotonic() if now is None else now)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            result = {}
            for name, events in self.events.items():
                recent = [stamp for stamp in events if now - stamp <= 2]
                result[name + '_hz'] = (round((len(recent) - 1) / (recent[-1] - recent[0]), 2)
                                       if len(recent) > 1 and recent[-1] > recent[0] else 0.0)
            return result


def unknown_result(reason):
    return {'status': 'unknown', 'persons': [], 'scene': {'fight': {
        'label': None, 'score': None, 'state': 'unknown', 'updated': False,
        'prediction_stamp_ns': None}}, 'preview_reason': reason}


def select_overlay(frame, prediction, epoch, connected, now, max_age, prediction_age_ns, action_ttls_ns=None):
    """Associate cached geometry only with a short-lived, matching source stream."""
    meta = dict(source_epoch=epoch, image_stamp_ns=frame.stamp_ns, frame_id=frame.frame_id,
                source_connected=connected, result_stamp_ns=None, result_age_ms=None,
                overlay_applied=False, reason='waiting_for_result')
    if not connected or frame.epoch != epoch:
        meta['reason'] = 'stream_lost_or_reset'
        return unknown_result(meta['reason']), meta
    if prediction is None:
        return unknown_result(meta['reason']), meta
    result_frame, result = prediction
    meta['result_stamp_ns'] = result_frame.stamp_ns
    meta['request_id'] = result.get('request_id')
    same_stream = (result_frame.epoch == frame.epoch == epoch and result_frame.frame_id == frame.frame_id
                   and tuple(result_frame.image.shape) == tuple(frame.image.shape))
    if not same_stream or result_frame.stamp_ns > frame.stamp_ns:
        meta['reason'] = 'result_source_mismatch'
        return unknown_result(meta['reason']), meta
    source_age = (frame.stamp_ns - result_frame.stamp_ns) / 1e9
    wall_age = max(0, now - result_frame.received)
    age = max(source_age, wall_age)
    meta.update(result_age_ms=round(age * 1000, 2), result_source_age_ms=round(source_age * 1000, 2),
                result_wall_age_ms=round(wall_age * 1000, 2))
    if age > max_age:
        meta['reason'] = 'overlay_expired'
        return unknown_result(meta['reason']), meta
    rendered = expire_result(result, frame.stamp_ns, prediction_age_ns, action_ttls_ns)
    actions = [action for person in rendered.get('persons', []) for action in person.get('actions', {}).values()]
    actions += list(rendered.get('scene', {}).values())
    ages = []
    for action in actions:
        if not isinstance(action, dict):
            continue
        prediction_stamp = action.get('prediction_stamp_ns')
        if isinstance(prediction_stamp, int):
            ages.append(max(0, (frame.stamp_ns - prediction_stamp) / 1e6))
        if frame.stamp_ns != result_frame.stamp_ns:
            action['updated'] = False
    meta.update(overlay_applied=True, reason='matched_recent_result',
                oldest_prediction_age_ms=round(max(ages), 2) if ages else None,
                cached_on_new_image=frame.stamp_ns != result_frame.stamp_ns)
    return rendered, meta


def scale_preview(image, result, max_width):
    if max_width <= 0 or image.shape[1] <= max_width:
        return image, result
    width = int(max_width)
    height = max(1, round(image.shape[0] * width / image.shape[1]))
    sx, sy = width / image.shape[1], height / image.shape[0]
    result = copy.deepcopy(result)
    for person in result.get('persons', []):
        if len(person.get('bbox', [])) == 4:
            box = person['bbox']
            person['bbox'] = [box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy]
        for point in person.get('keypoints') or []:
            point[0], point[1] = point[0] * sx, point[1] * sy
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA), result


class PreviewPipeline:
    def __init__(self, overlay, publish, source_state, rates, fps=20, max_age=.35,
                 prediction_age_ns=5_000_000_000, max_width=0, on_error=None):
        self.overlay, self.publish, self.source_state, self.rates = overlay, publish, source_state, rates
        self.period = 1 / max(1, min(float(fps), 60))
        self.max_age, self.prediction_age_ns, self.max_width = max_age, prediction_age_ns, max_width
        self.on_error = on_error
        self.lock = threading.Lock()
        self.frame = None
        self.prediction = None
        self.action_ttls_ns = {}
        self.last_meta = {}
        self.last_token = None
        self.blank_key = None
        self.blank_image = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name='pphuman-preview', daemon=True)

    def start(self):
        self.thread.start()

    def submit_frame(self, frame):
        with self.lock:
            self.frame = frame  # a single shared immutable image reference, no queue/copy
            if self.prediction and self.prediction[0].epoch != frame.epoch:
                self.prediction = None

    def submit_result(self, frame, result, action_ttls_ns=None):
        with self.lock:
            self.prediction = (frame, result)
            # Bind the ready policy to this result, including after worker restart.
            self.action_ttls_ns = dict(action_ttls_ns or {})

    def diagnostics(self):
        with self.lock:
            return dict(self.last_meta)

    def run(self):
        deadline = time.monotonic()
        while not self.stop.is_set():
            try:
                self.render_once()
            except Exception as exc:
                with self.lock:
                    self.last_meta = dict(error=f'{type(exc).__name__}: {exc}', overlay_applied=False)
                if self.on_error:
                    self.on_error(str(exc))
                self.stop.wait(1)
            deadline += self.period
            now = time.monotonic()
            if deadline < now:
                deadline = now + self.period
            self.stop.wait(max(0, deadline - now))

    def render_once(self):
        started = time.monotonic()
        with self.lock:
            frame, prediction, action_ttls_ns = self.frame, self.prediction, self.action_ttls_ns
        if frame is None:
            return
        epoch, connected, reason, _ = self.source_state()
        if not connected or frame.epoch != epoch:
            with self.lock:
                if self.prediction is prediction:
                    self.prediction = None
            prediction = None
        result, meta = select_overlay(frame, prediction, epoch, connected, started,
                                      self.max_age, self.prediction_age_ns, action_ttls_ns)
        # Do not manufacture motion frames faster than the source; expiry still clears.
        token = (epoch, frame.stamp_ns, connected, meta['overlay_applied'], meta['reason'],
                 meta.get('request_id') if meta['overlay_applied'] else None)
        if token == self.last_token:
            return
        meta.update(self.rates.snapshot(started))
        result['preview_diagnostics'] = meta
        live_image = connected and frame.epoch == epoch
        if live_image:
            image, result = scale_preview(frame.image, result, self.max_width)
            rendered = self.overlay.render(image, result)
        else:
            key = (tuple(frame.image.shape), reason)
            if self.blank_key != key:
                self.blank_image = self.overlay.unavailable(shape=frame.image.shape, text='图像断流／源已重置，等待恢复')
                self.blank_key = key
            rendered, _ = scale_preview(self.blank_image, {}, self.max_width)
        # Rendering may take time; never publish a now-expired or reset overlay.
        current_epoch, current_connected, _, _ = self.source_state()
        if current_epoch != epoch or current_connected != connected:
            return
        if meta['overlay_applied'] and time.monotonic() - prediction[0].received > self.max_age:
            return
        if meta['result_age_ms'] is not None:
            wall_age_ms = max(0, (time.monotonic() - prediction[0].received) * 1000)
            meta['result_wall_age_ms'] = round(wall_age_ms, 2)
            meta['result_age_ms'] = round(max(wall_age_ms, meta['result_source_age_ms']), 2)
        meta.update(render_ms=round((time.monotonic() - started) * 1000, 2),
                    source_shape=list(frame.image.shape), rendered_shape=list(rendered.shape),
                    display_image_age_ms=round((time.monotonic() - frame.received) * 1000, 2),
                    max_overlay_age_ms=round(self.max_age * 1000, 2))
        self.publish(rendered, frame.stamp_ns, frame.frame_id, meta)
        self.rates.mark('preview')
        with self.lock:
            self.last_meta = meta
        self.last_token = token

    def close(self):
        self.stop.set()
        self.thread.join(timeout=4)
