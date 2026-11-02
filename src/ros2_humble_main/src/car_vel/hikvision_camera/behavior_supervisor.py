"""Isolated behavior worker fed exclusively by the gateway's shared camera.

No model or Paddle imports occur here. One replaceable pending frame, one child,
bounded protocol queues and capture timestamps prevent synthetic clip continuity.
"""
from __future__ import annotations

import base64
import copy
import importlib
import json
import math
import os
from pathlib import Path
import queue
import struct
import subprocess
import threading
import time
from collections import deque


def isolated_environment(environ=None):
    env = dict(os.environ if environ is None else environ)
    for name in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD',
                 'VIRTUAL_ENV', 'CONDA_PREFIX', 'CONDA_DEFAULT_ENV'):
        env.pop(name, None)
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                 'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
        env[name] = '2'
    env.update(PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1', CUDA_VISIBLE_DEVICES='')
    return env


def _read_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        chunk = stream.read(size - len(result))
        if not chunk:
            raise EOFError('行为识别子进程已退出')
        result.extend(chunk)
    return bytes(result)


def _read_message(stream):
    size, = struct.unpack('>I', _read_exact(stream, 4))
    if not 1 <= size <= 4 * 1024 * 1024:
        raise RuntimeError('行为识别协议响应过大')
    result = json.loads(_read_exact(stream, size).decode('utf-8'))
    if not isinstance(result, dict):
        raise RuntimeError('行为识别协议响应无效')
    return result


def _write_message(stream, message, payload=b''):
    header = json.dumps(message, allow_nan=False, separators=(',', ':')).encode('utf-8')
    stream.write(struct.pack('>I', len(header)) + header)
    if payload:
        stream.write(payload)
    stream.flush()


class BehaviorSupervisor:
    def __init__(self, *, runtime_dir=None, python=None, config_path=None,
                 freshness_seconds=5.0, startup_timeout=60.0,
                 inference_timeout=30.0, speech=None, popen=subprocess.Popen,
                 clock=time.monotonic_ns, branch=None, log=None):
        self.runtime_dir = Path(runtime_dir or Path(__file__).resolve().parent.parent / 'person_behavior')
        self.python = Path(python or Path(os.environ.get('BEHAVIOR_HOME', str(Path.home() / '.local/share/robot-behavior'))) / 'venv/bin/python')
        self.config_path = Path(config_path).expanduser() if config_path else None
        self.ttl = float(freshness_seconds)
        self.startup_timeout, self.inference_timeout = startup_timeout, inference_timeout
        self.speech, self._popen, self._clock = speech, popen, clock
        if branch not in (None, 'fall', 'fight', 'smoking', 'pose'):
            raise ValueError('Unknown behavior branch')
        self.branch, self._log = branch, log
        self._clip = deque(maxlen=8)
        self._fight_sampling = None
        self._next_capture_ns = None
        self._last_result_latency = None
        self._last_model_timing = {}
        self._stale_results = 0
        self._announced_ready_epoch = None
        self._condition = threading.Condition(threading.RLock())
        self._lifecycle = threading.Lock()
        self._stop = threading.Event()
        self._worker = self._process = None
        self._enabled = {'fall': True, 'fight': True, 'smoking': True}
        if branch:
            self._enabled = {key: key in self._scope() for key in self._enabled}
        self._manual = False
        self._suspended = False
        self._mode, self._task_id = 'manual', ''
        self._epoch, self._request_id, self._sequence = 1, 0, 0
        self._pending = self._latest = None
        self._source_key = self._shape = None
        self._last_stamp = 0
        self._loaded = False
        self._error = ''
        self._events = []
        self._last_alert = {}
        self._stderr = ''
        self._fight_ttl = 1.0
        self._smoking_ttl = 1.5
        if self.config_path and self.config_path.exists():
            try:
                value = json.loads(self.config_path.read_text(encoding='utf-8'))
                self._enabled = self._validate_config(value)
            except (ValueError, OSError) as error:
                self._error = '行为识别配置无效：' + str(error)

    @staticmethod
    def _validate_config(value):
        # Migrate existing two-switch settings; the replacement is on by default.
        if (not isinstance(value, dict) or set(value) not in (
                {'fall_enabled', 'fight_enabled'}, {'fall_enabled', 'fight_enabled', 'smoking_enabled'})
                or any(type(v) is not bool for v in value.values())):
            raise ValueError('行为配置只允许 fall_enabled、fight_enabled 和 smoking_enabled 布尔值')
        return {'fall': value['fall_enabled'], 'fight': value['fight_enabled'],
                'smoking': value.get('smoking_enabled', True)}

    def _scope(self):
        return {'fall', 'smoking'} if self.branch == 'pose' else (
            {self.branch} if self.branch else {'fall', 'fight', 'smoking'})

    def configure(self, value):
        enabled = self._validate_config(value)
        enabled = {key: active and key in self._scope() for key, active in enabled.items()}
        if self.config_path:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.config_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
            os.replace(temporary, self.config_path)
        with self._condition:
            if enabled == self._enabled:
                return self.status()
            self._enabled = enabled
            self._invalidate_locked('config_changed')
        # Model ownership changes with the switches. Recreate only this owned
        # child so enabling smoking actually loads PPYOLOE and disabling it
        # releases it; the independent fight worker continues unchanged.
        with self._lifecycle:
            with self._condition:
                self._stop.set()
                self._condition.notify_all()
                worker = self._worker
            self._shutdown_child()
            if worker and worker.is_alive():
                worker.join(timeout=3)
        self._sync_running()
        return self.status()

    def _wanted(self):
        return not self._suspended and any(self._enabled.values()) and (self._manual or self._mode in ('transit', 'inspection'))

    def start(self):
        with self._condition:
            self._manual = True
            self._suspended = False
        self._sync_running()
        return self.status()

    def stop(self):
        with self._condition:
            self._manual = False
            self._suspended = True
            self._invalidate_locked('stopped')
        self._sync_running()
        return self.status()

    def set_mode(self, mode, task_id=''):
        with self._condition:
            if (mode, task_id) != (self._mode, self._task_id):
                self._mode, self._task_id = mode, task_id
                self._suspended = False
                self._invalidate_locked('task_or_mode_changed')
        self._sync_running()
        return self.status()

    def reset_source(self):
        with self._condition:
            self._source_key = self._shape = None
            self._invalidate_locked('source_changed')

    def _invalidate_locked(self, reason):
        self._epoch += 1
        self._pending = self._latest = None
        self._last_stamp = 0
        self._clip.clear()
        self._next_capture_ns = None
        self._last_result_latency = None
        self._last_model_timing = {}
        self._stale_results = 0
        self._announced_ready_epoch = None
        self._events.clear()
        self._last_alert.clear()
        self._condition.notify_all()

    def _sync_running(self):
        with self._lifecycle:
            with self._condition:
                wanted = self._wanted()
                alive = self._worker is not None and self._worker.is_alive()
                if wanted and not alive:
                    self._stop = threading.Event()
                    self._error = ''
                    self._stderr = ''
                    self._loaded = False
                    self._worker = threading.Thread(target=self._run, name='behavior-supervisor', daemon=True)
                    self._worker.start()
                    return
                if wanted:
                    return
                self._stop.set()
                self._condition.notify_all()
            self._shutdown_child()
            if alive:
                self._worker.join(timeout=3)

    def close(self):
        with self._condition:
            self._manual = False
            self._mode, self._task_id = 'manual', ''
            self._invalidate_locked('closed')
        self._sync_running()

    def submit(self, frame, *, stamp_ns, source_key, raw_jpeg=None):
        """Submit an actual read/capture once; overwrite pending work when busy."""
        shape = tuple(frame.shape)
        if len(shape) != 3 or shape[2] != 3 or not 0 < frame.nbytes <= 25165824:
            return False
        with self._condition:
            if not self._wanted() or self._stop.is_set():
                return False
            if source_key != self._source_key or shape != self._shape:
                self._source_key, self._shape = source_key, shape
                self._invalidate_locked('source_or_size_changed')
            if (type(stamp_ns) is not int or stamp_ns <= self._last_stamp
                    or stamp_ns > self._clock() + 100000000):
                return False
            self._last_stamp = stamp_ns
            if self.branch == 'fight':
                cfg = self._fight_sampling
                if not cfg:
                    return False  # Sampling parameters come from the loaded worker.
                if self._next_capture_ns is not None and stamp_ns < self._next_capture_ns:
                    return False
                period = round(1e9 / cfg['sample_hz'])
                if self._clip and stamp_ns - self._clip[-1][0] > cfg['max_sample_gap_seconds'] * 1e9:
                    self._clip.clear()
                    self._next_capture_ns = None
                # Bound transport/memory, retaining aspect ratio. This is a
                # real capture, never a duplicate or a synthesized timestamp.
                height, width = shape[:2]
                ratio = min(1., 640 / max(height, width))
                small = (importlib.import_module('cv2').resize(frame, (max(1, round(width * ratio)), max(1, round(height * ratio))))
                         if ratio < 1 else frame.copy())
                self._clip.append((stamp_ns, small))
                while self._clip and stamp_ns - self._clip[0][0] > cfg['max_window_seconds'] * 1e9:
                    self._clip.popleft()
                if self._next_capture_ns is None:
                    self._next_capture_ns = stamp_ns + period
                else:
                    self._next_capture_ns += ((stamp_ns - self._next_capture_ns) // period + 1) * period
                self._pending = (self._epoch, stamp_ns, small, None)
                self._condition.notify_all()
                return True
            self._pending = (self._epoch, stamp_ns, frame.copy(), raw_jpeg)
            self._condition.notify_all()
        return True

    def _launch(self):
        if not self.python.is_file():
            raise RuntimeError('行为识别独立环境未安装：' + str(self.python))
        command = [str(self.python), '-m', 'pphuman_runtime.worker', '--config', 'config/inference.yaml']
        if self.branch:
            command += ['--branch', self.branch]
        command += ['--branches', ','.join(key for key, value in self._enabled.items() if value)]
        if self._log:
            self._log(f'启动 {self.branch or "fall+fight"}，解释器={self.python}')
        process = self._popen(command, cwd=str(self.runtime_dir), env=isolated_environment(),
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              bufsize=0)
        with self._condition:
            self._process = process
        process._behavior_write_lock = threading.Lock()
        incoming, outgoing = queue.Queue(maxsize=4), queue.Queue(maxsize=1)

        def reader():
            try:
                while not self._stop.is_set():
                    incoming.put(_read_message(process.stdout), timeout=1)
            except Exception as error:
                try: incoming.put_nowait(error)
                except queue.Full: pass

        def writer():
            try:
                while not self._stop.is_set():
                    try: message, payload = outgoing.get(timeout=.2)
                    except queue.Empty: continue
                    with process._behavior_write_lock:
                        _write_message(process.stdin, message, payload)
            except Exception as error:
                try: incoming.put_nowait(error)
                except queue.Full: pass

        def stderr_reader():
            try:
                while True:
                    chunk = process.stderr.read(1024)
                    if not chunk: break
                    self._stderr = (self._stderr + chunk.decode('utf-8', 'replace'))[-4096:]
            except (ValueError, OSError): pass

        for target, name in ((reader, 'reader'), (writer, 'writer'), (stderr_reader, 'stderr')):
            threading.Thread(target=target, name='behavior-' + name, daemon=True).start()
        return incoming, outgoing

    def _receive(self, incoming, timeout):
        deadline = time.monotonic() + timeout
        while not self._stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('行为识别子进程响应超时')
            try: result = incoming.get(timeout=min(.2, remaining))
            except queue.Empty: continue
            if isinstance(result, Exception): raise RuntimeError(str(result))
            return result
        raise RuntimeError('行为识别已停止')

    @staticmethod
    def _type(result):
        return result.get('type') or result.get('op')

    def _run(self):
        try:
            incoming, outgoing = self._launch()
            ready = self._receive(incoming, self.startup_timeout)
            if self._type(ready) != 'ready':
                raise RuntimeError('行为识别初始化失败：' + str(ready.get('error') or ready))
            with self._condition:
                self._fight_ttl = min(self.ttl, float(ready.get('sampling', {}).get('fight', {}).get('result_ttl_seconds', 1.0)))
                self._smoking_ttl = min(self.ttl, float(ready.get('sampling', {}).get('smoking', {}).get('result_ttl_seconds', 1.5)))
                if self.branch == 'fight':
                    self._fight_sampling = ready['sampling']['fight']
                self._loaded = True
            if self._log:
                self._log(f'{self.branch or "fall+fight"} 模型已加载；等待真实采样')
            child_epoch = None
            last_fight_request_stamp = None
            while not self._stop.is_set():
                with self._condition:
                    while self._pending is None and not self._stop.is_set():
                        self._condition.wait(.2)
                    if self._stop.is_set(): break
                    packet, self._pending = self._pending, None
                    epoch, stamp, frame, raw = packet
                    enabled = dict(self._enabled)
                    clip = list(self._clip) if self.branch == 'fight' else None
                if (self._clock() - stamp) / 1e9 > self.ttl:
                    continue
                if child_epoch != epoch:
                    outgoing.put((dict(op='reset', source_epoch=epoch), b''), timeout=1)
                    ack = self._receive(incoming, self.inference_timeout)
                    if self._type(ack) != 'reset_ack':
                        raise RuntimeError('行为识别重置确认无效')
                    child_epoch = epoch
                    last_fight_request_stamp = None
                if (clip and len(clip) == 8 and last_fight_request_stamp is not None
                        and stamp - last_fight_request_stamp
                        < self._fight_sampling['prediction_interval_seconds'] * 1e9):
                    continue
                self._request_id += 1
                request_id = self._request_id
                payload = frame.tobytes(order='C')
                if clip:
                    # Snapshot and payload belong to one epoch and end exactly
                    # at the frame selected while holding the capture lock.
                    stamp, frame = clip[-1]
                    payload = b''.join(item.tobytes(order='C') for _, item in clip)
                header = dict(op='infer', request_id=request_id, source_epoch=epoch,
                              stamp_ns=stamp, shape=list(frame.shape), payload_bytes=len(payload),
                              enabled_branches=enabled)
                if clip:
                    header['clip_stamps_ns'] = [captured for captured, _ in clip]
                    if len(clip) == 8:
                        last_fight_request_stamp = stamp
                outgoing.put((header, payload), timeout=1)
                result = self._receive(incoming, self.inference_timeout)
                if self._type(result) != 'result' or result.get('request_id') != request_id:
                    raise RuntimeError('行为识别结果与请求不匹配：' + str(result.get('error') or 'protocol'))
                if result.get('source_epoch') != epoch or result.get('stamp_ns') != stamp:
                    raise RuntimeError('行为识别结果来源或时间戳不匹配')
                self._accept(result, frame, raw)
        except Exception as error:
            with self._condition:
                if not self._stop.is_set(): self._error = str(error)
                self._latest = self._pending = None
            if not self._stop.is_set() and self._log:
                self._log(f'{self.branch or "fall+fight"} 失败：{error}；{self._stderr[-1200:]}')
        finally:
            with self._condition: self._loaded = False
            self._shutdown_child()

    def _shutdown_child(self):
        with self._condition:
            process, self._process = self._process, None
        if process is None: return
        # Never signal by name/process group: terminate only the child we own.
        try:
            if process.poll() is None:
                def shutdown():
                    try:
                        with process._behavior_write_lock:
                            _write_message(process.stdin, {'op': 'shutdown'})
                    except (OSError, ValueError): pass
                threading.Thread(target=shutdown, daemon=True).start()
                try: process.wait(timeout=.6)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try: process.wait(timeout=1.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.5)
        finally:
            for pipe in (process.stdin, process.stdout, process.stderr):
                try: pipe.close()
                except (AttributeError, OSError, ValueError): pass

    def _accept(self, result, frame, raw_jpeg=None):
        stamp, epoch = result['stamp_ns'], result['source_epoch']
        expires = self._expires(result)
        with self._condition:
            if epoch != self._epoch or self._stop.is_set():
                return False
            self._last_result_latency = (self._clock() - stamp) / 1e9
            self._last_model_timing = dict(result.get('timing_ms') or {})
            if self._clock() > expires:
                self._stale_results += 1
                if self._stale_results == 1 and self._log:
                    self._log(f'{self.branch or "fall+fight"} 结果已过期；采集到返回耗时 {self._last_result_latency:.3f}s，未发布告警')
                return False
        raw, annotated = self._render(frame, result, raw_jpeg)
        with self._condition:
            if (epoch != self._epoch or self._stop.is_set()
                    or self._clock() > expires): return False
            self._sequence += 1
            if self._log and self._announced_ready_epoch != epoch and result.get('status') == 'valid':
                self._announced_ready_epoch = epoch
                self._log(f'{self.branch or "fall+fight"} 检测已就绪；耗时 {self._last_result_latency:.3f}s')
            self._latest = dict(result=copy.deepcopy(result), raw=raw, annotated=annotated,
                                stamp_ns=stamp, source_epoch=epoch, sequence=self._sequence,
                                expires_ns=expires,
                                timestamp_epoch=time.time(), mode=self._mode, task_id=self._task_id)
            detections = self._detections(result)
            alerts = [v for v in detections if v['alert']]
            keys = [v['category'] for v in alerts
                    if stamp - self._last_alert.get(v['category'], -10**15) >= 20 * 10**9]
            if keys:
                for key in keys: self._last_alert[key] = stamp
                if self._mode in ('transit', 'inspection') and self._task_id:
                    event = self._evidence_locked()
                    event['detections'] = [v for v in alerts if v['category'] in keys]
                    self._events.append(event)
                    self._events = self._events[-16:]
            self._condition.notify_all()
        if self.speech:
            for key in set(keys):
                with self._condition:
                    if epoch != self._epoch or self._mode == 'paused' or self._clock() > expires: break
                    try: self.speech(key)
                    except Exception: pass
        return True

    @staticmethod
    def _render(frame, result, raw):
        cv2 = importlib.import_module('cv2')
        drawn = frame.copy()
        for person in result.get('persons', []):
            actions = person.get('actions') or {}
            bbox = person.get('bbox') or person.get('box')
            if bbox and len(bbox) >= 4:
                x1, y1, x2, y2 = map(int, bbox[:4])
                cv2.rectangle(drawn, (x1, y1), (x2, y2), (0, 180, 255), 2)
                text = ' | '.join(key + ': ' + str(action.get('label') or action.get('state', 'unknown'))
                                  for key, action in actions.items() if action.get('state') != 'disabled')
                cv2.putText(drawn, text,
                            (max(0, x1), max(20, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 180, 255), 2)
                smoke = actions.get('smoking') or {}
                selected = (smoke.get('evidence') or {}).get('selected_detection') or {}
                hand_detection = ((smoke.get('evidence') or {}).get('hand_to_mouth') or {}).get('detection') or {}
                for item in (selected, hand_detection):
                    box = item.get('bbox')
                    if box and len(box) == 4:
                        a, b, c, d = map(int, box)
                        cv2.rectangle(drawn, (a, b), (c, d), (255, 100, 200), 2)
        fight = (result.get('scene') or {}).get('fight') or {}
        if fight.get('state') != 'disabled':
            cv2.putText(drawn, 'scene fight: ' + str(fight.get('label') or fight.get('state', 'unknown')),
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 180, 255), 2)
        def encode(image):
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 88])
            if not ok: raise RuntimeError('行为证据图片编码失败')
            return encoded.tobytes()
        return raw or encode(frame), encode(drawn)

    def _detections(self, result):
        values = []
        for person in result.get('persons', []):
            action = (person.get('actions') or {}).get('fall') or {}
            if self._enabled['fall'] and action.get('label') in ('fall', 'lying'):
                values.append(dict(category='fall', label=action['label'], confidence=None,
                    score_kind='rule_evidence', measurement_type='rule_evidence', scope='person',
                    track_id=person.get('track_id'), bbox=person.get('bbox'),
                    state=action.get('state', 'unknown'), updated=action.get('updated') is True,
                    alert=action.get('label') == 'fall' and action.get('state') == 'valid' and action.get('updated') is True,
                    rule_evidence=action, source_epoch=result['source_epoch'], stamp_ns=result['stamp_ns'], fresh=True))
            smoke = (person.get('actions') or {}).get('smoking') or {}
            score, predicted = smoke.get('score'), smoke.get('prediction_stamp_ns')
            if (self._enabled['smoking'] and smoke.get('label') == 'smoking'
                    and smoke.get('state') == 'valid' and type(predicted) is int
                    and 0 <= self._clock() - predicted <= self._smoking_ttl * 1e9
                    and type(score) in (int, float) and math.isfinite(score) and 0 <= score <= 1
                    and smoke.get('score_kind') in ('detection_score', 'temporally_associated_detection_score')):
                details = smoke.get('evidence') or {}
                hand = details.get('hand_to_mouth') or {}
                # Cached/occluded positives are display context, not new alarms.
                updated = smoke.get('updated') is True and not hand.get('short_occlusion', False)
                values.append(dict(category='smoking', label='smoking', confidence=float(score),
                    score_kind=smoke['score_kind'], measurement_type='associated_detection_score', scope='person',
                    track_id=person.get('track_id'), bbox=person.get('bbox'), state='valid',
                    updated=updated, alert=updated, evidence_details=details, reason=smoke.get('reason'),
                    source_epoch=result['source_epoch'], stamp_ns=result['stamp_ns'],
                    prediction_stamp_ns=predicted, fresh=True))
        action = (result.get('scene') or {}).get('fight') or {}
        score = action.get('score')
        if (self._enabled['fight'] and action.get('label') == 'fight' and
                type(score) in (int, float) and math.isfinite(score) and 0 <= score <= 1):
            values.append(dict(category='fight', label='fight', confidence=float(score),
                score_kind='model_probability', measurement_type='confidence', scope='scene',
                state=action.get('state', 'unknown'), updated=action.get('updated') is True,
                alert=action.get('state') == 'valid' and action.get('updated') is True,
                source_epoch=result['source_epoch'], stamp_ns=result['stamp_ns'], fresh=True))
        return values

    def _fresh_locked(self):
        return bool(self._latest and self._clock() <= self._latest['expires_ns'])

    def _expires(self, result):
        deadline = result['stamp_ns'] + int(self.ttl * 1e9)
        fight = (result.get('scene') or {}).get('fight') or {}
        if self._enabled['fight'] and fight.get('state') == 'valid':
            stamp = fight.get('prediction_stamp_ns')
            if type(stamp) is not int: stamp = result['stamp_ns']
            deadline = min(deadline, stamp + int(self._fight_ttl * 1e9))
        return deadline

    def status(self):
        with self._condition:
            running = bool(self._worker and self._worker.is_alive() and not self._stop.is_set())
            fresh = self._fresh_locked() and running
            result = copy.deepcopy(self._latest['result']) if fresh else {}
            branches = {}
            for key in ('fall', 'fight', 'smoking'):
                if not self._enabled[key]: state = 'disabled'
                elif self._error: state = 'error'
                elif not running: state = 'stopped'
                elif not self._loaded: state = 'starting'
                elif not fresh: state = 'stale' if self._latest else 'warming'
                else: state = 'warming'
                actions = ([(v.get('actions') or {}).get(key) or {} for v in result.get('persons', [])]
                           if key != 'fight' else [(result.get('scene') or {}).get('fight') or {}])
                action = next((a for a in actions if a.get('label') == key), actions[0] if actions else {})
                if fresh and self._enabled[key]: state = action.get('state', 'warming')
                if (fresh and self._enabled[key] and key == 'smoking' and actions
                        and (type(action.get('prediction_stamp_ns')) is not int
                             or self._clock() - action['prediction_stamp_ns'] > self._smoking_ttl * 1e9)):
                    state = 'stale'
                if (fresh and self._enabled[key] and key != 'fight' and not actions
                        and result.get('pose_status', {}).get('state') == 'valid'):
                    state = 'valid'
                    action = {'label': 'no_person'}
                branches[key] = dict(enabled=self._enabled[key], state=state, ready=state == 'valid',
                                     label=action.get('label'), confidence=None if key == 'fall' else action.get('score'),
                                     score_kind=('rule_evidence' if key == 'fall' else 'model_probability'
                                                 if key == 'fight' else action.get('score_kind', 'associated_detection_score')),
                                     window=action.get('window'), reason=action.get('reason'))
            enabled = [v for v in branches.values() if v['enabled']]
            ready = bool(fresh and enabled and all(v['ready'] for v in enabled))
            return dict(success=True, running=running, ready=ready, partial=running and not ready,
                        error=self._error or None, diagnostics=self._stderr[-2048:],
                        last_result_latency_seconds=self._last_result_latency,
                        last_model_timing_ms=dict(self._last_model_timing), stale_results=self._stale_results,
                        capture_window=dict(samples=len(self._clip), required_samples=8,
                                            target_sample_hz=self._fight_sampling['sample_hz'] if self._fight_sampling else None)
                            if self.branch == 'fight' else None,
                        enabled=dict(self._enabled), branches=branches,
                        source_epoch=self._epoch, stamp_ns=self._latest['stamp_ns'] if self._latest else None,
                        frame_stamp_ns=str(self._latest['stamp_ns']) if self._latest else None,
                        frame_age_seconds=round((self._clock() - self._latest['stamp_ns']) / 1e9, 3) if self._latest else None,
                        fresh=bool(fresh), result=result, detections=self._detections(result) if fresh else [],
                        max_result_age_seconds=(self._latest['expires_ns'] - self._latest['stamp_ns']) / 1e9 if self._latest else self.ttl,
                        mode=self._mode, task_id=self._task_id, sequence=self._sequence,
                        frame_url='/api/behavior/frame.jpg', manual_detection_enabled=self._manual)

    def frame(self):
        with self._condition:
            if not self._fresh_locked() or not self._loaded:
                raise RuntimeError('行为标注帧尚未就绪或已过期')
            return self._latest['annotated'], self._latest['source_epoch'], self._latest['stamp_ns']

    def _evidence_locked(self):
        value = self._latest
        return dict(mode=value['mode'], task_id=value['task_id'], sequence=value['sequence'],
                    timestamp_epoch=value['timestamp_epoch'], source_epoch=value['source_epoch'], stamp_ns=value['stamp_ns'],
                    expires_ns=value['expires_ns'],
                    fresh=True, detections=self._detections(value['result']),
                    raw_jpeg_base64=base64.b64encode(value['raw']).decode(),
                    annotated_jpeg_base64=base64.b64encode(value['annotated']).decode())

    def capture(self, duration=2.0, task_id=''):
        duration = max(0, min(15, float(duration)))
        deadline = time.monotonic() + duration
        with self._condition:
            if not any(self._enabled.values()):
                return dict(detections=[], behavior=self.status(), partial=False)
            epoch, sequence = self._epoch, self._sequence
            best = None
            while True:
                if epoch != self._epoch or task_id != self._task_id:
                    return dict(detections=[], error='行为巡检范围已变化', partial=True)
                if self._fresh_locked() and self._sequence > sequence:
                    candidate = self._evidence_locked()
                    if best is None or any(v['alert'] for v in candidate['detections']): best = candidate
                    sequence = self._sequence
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._error or not self._wanted(): break
                self._condition.wait(min(.2, remaining))
            if best is None or self._clock() > best['expires_ns']:
                return dict(detections=[], error=self._error or '采集窗口内没有新鲜行为结果', partial=True, behavior=self.status())
            return {**best, 'behavior': self.status(), 'partial': not self.status()['ready']}

    def events(self, task_id):
        with self._condition:
            events = [copy.deepcopy(event) for event in self._events
                      if event['task_id'] == task_id and event['source_epoch'] == self._epoch
                      and self._clock() <= event['expires_ns']]
            self._events.clear()
            return events
