"""ROS-independent protocol, latest-frame state and result expiry."""
import copy
import json
import math
import os
import select
import signal
import struct
import subprocess
import threading
import time
from dataclasses import dataclass

MAX_JSON = 16 * 1024 * 1024


@dataclass
class Frame:
    image: object
    stamp_ns: int
    frame_id: str
    epoch: int
    received: float


class LatestFrames:
    """One pending frame plus the caller's one in-flight frame."""
    def __init__(self, timeout):
        self.lock = threading.Lock()
        self.timeout = timeout
        self.epoch = 1
        self.pending = None
        self.last_time = None
        self.last_stamp = None
        self.shape = None
        self.frame_id = None
        self.connected = False
        self.reason = 'waiting_for_image'
        self.dropped = 0

    def invalidate(self, reason):
        with self.lock:
            self._invalidate(reason)

    def _invalidate(self, reason):
        self.epoch += 1
        self.pending = None
        self.connected = False
        self.reason = reason
        self.last_stamp = None
        self.shape = None
        self.frame_id = None

    def put(self, image, stamp_ns, frame_id, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            reason = None
            if self.last_time is not None and now - self.last_time > self.timeout:
                reason = 'stream_recovered'
            if self.last_stamp is not None and stamp_ns <= self.last_stamp:
                reason = 'timestamp_not_increasing'
            if self.shape is not None and tuple(image.shape) != self.shape:
                reason = 'image_size_changed'
            if self.frame_id is not None and frame_id != self.frame_id:
                reason = 'frame_id_changed'
            if reason:
                self._invalidate(reason)
            if self.pending is not None:
                self.dropped += 1
            self.last_time, self.last_stamp = now, stamp_ns
            self.shape, self.frame_id = tuple(image.shape), frame_id
            self.connected = True
            self.reason = 'streaming'
            self.pending = Frame(image, stamp_ns, frame_id, self.epoch, now)
            return self.pending

    def take(self):
        with self.lock:
            item, self.pending = self.pending, None
            return item

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.connected and now - self.last_time > self.timeout:
                self._invalidate('stream_lost')
            return self.epoch, self.connected, self.reason, self.dropped


def sampling_ttls_ns(sampling):
    """Read optional per-branch TTLs advertised by either worker profile."""
    limits = {}
    for name, config in (sampling or {}).items():
        ttl = config.get('result_ttl_seconds') if isinstance(config, dict) else None
        if isinstance(ttl, (int, float)) and not isinstance(ttl, bool) and math.isfinite(ttl) and ttl > 0:
            limits[name] = int(ttl * 1_000_000_000)
    return limits


def expire_result(result, stamp_ns, max_age_ns, action_ttls_ns=None):
    """Never label cached/unknown/warming predictions as normal."""
    result = copy.deepcopy(result)
    actions = [item for p in result.get('persons', []) for item in p.get('actions', {}).items()]
    actions += list(result.get('scene', {}).items())
    for name, action in actions:
        if not isinstance(action, dict):
            continue
        prediction_stamp = action.get('prediction_stamp_ns')
        limit = min(max_age_ns, (action_ttls_ns or {}).get(name, max_age_ns))
        if action.get('state') == 'valid':
            if not isinstance(prediction_stamp, int) or not 0 <= stamp_ns - prediction_stamp <= limit:
                action.update(state='unknown', label=None, score=None, updated=False, expired=True, reason='expired')
        elif action.get('state') not in ('warming', 'unknown'):
            action.update(state='unknown', label=None, score=None, updated=False)
        if action.get('expired') or action.get('reason') == 'expired':
            action.update(state='unknown', label=None, score=None, updated=False, expired=True)
            # Event/posture evidence belongs to the expired observation, not the
            # current image. Keep prediction_stamp_ns only as historical identity.
            for key, value in (('fall_event', 'unobserved'), ('event_stamp_ns', None),
                               ('onset_stamp_ns', None), ('posture', None), ('evidence', {})):
                if key in action:
                    action[key] = value
    return result


class WorkerClient:
    """POSIX bounded pipe IO. All IO, including writes, has a deadline."""
    def __init__(self, command, cwd, startup_timeout=180, request_timeout=30, stop_event=None, env=None):
        self.command, self.cwd = command, cwd
        self.startup_timeout, self.request_timeout = startup_timeout, request_timeout
        self.stop_event = stop_event or threading.Event()
        self.proc = None
        self.ready = {}
        self.sequence = 0
        self.env = env

    def start(self):
        self.proc = subprocess.Popen(self.command, cwd=self.cwd, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=None, bufsize=0,
                                     start_new_session=True, env=self.env)
        os.set_blocking(self.proc.stdin.fileno(), False)
        os.set_blocking(self.proc.stdout.fileno(), False)
        self.ready = self._read(time.monotonic() + self.startup_timeout)
        if self.ready.get('type') != 'ready' or self.ready.get('protocol_version') != 1:
            raise RuntimeError('Worker did not return protocol v1 ready')
        return self.ready

    def _wait(self, fd, writing, deadline):
        while True:
            if self.stop_event.is_set():
                raise InterruptedError('Bridge shutting down')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Worker pipe deadline exceeded')
            readable, writable, _ = select.select([] if writing else [fd], [fd] if writing else [], [], min(remaining, 0.1))
            if readable or writable:
                return

    def _read_exact(self, size, deadline):
        parts = bytearray()
        fd = self.proc.stdout.fileno()
        while len(parts) < size:
            self._wait(fd, False, deadline)
            try:
                chunk = os.read(fd, size - len(parts))
            except BlockingIOError:
                continue
            if not chunk:
                raise EOFError('Worker stdout closed')
            parts.extend(chunk)
        return bytes(parts)

    def _read(self, deadline):
        size = struct.unpack('!I', self._read_exact(4, deadline))[0]
        if not 0 < size <= MAX_JSON:
            raise ValueError('Invalid worker JSON size')
        result = json.loads(self._read_exact(size, deadline))
        if not isinstance(result, dict):
            raise ValueError('Worker response is not an object')
        return result

    def _send(self, header, payload=b'', deadline=None):
        deadline = deadline or time.monotonic() + self.request_timeout
        data = json.dumps(header, ensure_ascii=False, allow_nan=False).encode('utf-8')
        fd = self.proc.stdin.fileno()
        for segment in (struct.pack('!I', len(data)), data, payload):
            segment = memoryview(segment)
            while segment:
                self._wait(fd, True, deadline)
                try:
                    written = os.write(fd, segment[:65536])
                except BlockingIOError:
                    continue
                if not written:
                    raise EOFError('Worker stdin closed')
                segment = segment[written:]

    def reset(self, epoch):
        self.sequence += 1
        deadline = time.monotonic() + self.request_timeout
        self._send(dict(op='reset', request_id=self.sequence, stamp_ns=0, source_epoch=epoch,
                        shape=[], payload_bytes=0), deadline=deadline)
        response = self._read(deadline)
        if (response.get('type') != 'reset_ack' or response.get('request_id') != self.sequence
                or response.get('source_epoch') != epoch):
            raise ValueError('Worker reset acknowledgement mismatch')

    def infer(self, frame):
        self.sequence += 1
        request_id = self.sequence
        deadline = time.monotonic() + self.request_timeout
        image = frame.image
        self._send(dict(op='infer', request_id=request_id, stamp_ns=frame.stamp_ns,
                        source_epoch=frame.epoch, shape=list(image.shape), payload_bytes=image.nbytes),
                   image.tobytes(), deadline)
        while True:
            result = self._read(deadline)
            if result.get('type') == 'result' and result.get('request_id') == request_id:
                if result.get('source_epoch') != frame.epoch or result.get('stamp_ns') != frame.stamp_ns:
                    raise ValueError('Worker result frame identity mismatch')
                return result
            if result.get('type') in ('error', 'fatal'):
                raise RuntimeError('Worker returned error: ' + str(result))
            # reset/shutdown confirmations are control responses, never inference.

    def close(self):
        proc, self.proc = self.proc, None
        if proc is None:
            return
        if proc.poll() is None:
            # Best-effort tiny shutdown packet; blocking IO cannot delay teardown.
            try:
                data = json.dumps(dict(op='shutdown', request_id=self.sequence + 1,
                    stamp_ns=0, source_epoch=0, shape=[], payload_bytes=0)).encode()
                os.write(proc.stdin.fileno(), struct.pack('!I', len(data)) + data)
                proc.wait(timeout=.5)
            except (OSError, subprocess.TimeoutExpired):
                pass
        if proc.poll() is None:
            # Kill only our new process group (including descendants of our worker).
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=3)
        for pipe in (proc.stdin, proc.stdout):
            if pipe:
                pipe.close()
