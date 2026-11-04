"""Bounded, offline fixed-phrase speech independent of the YOLO process."""
from __future__ import annotations

import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path


PHRASES = {'fall': '检测到疑似人员跌倒', 'fight': '检测到疑似打架', 'smoking': '检测到疑似人员吸烟'}
INSPECTION_SMOKING_PHRASE = '吸烟有害健康，请不要在公共场合吸烟，谢谢配合'
INSPECTION_SMOKING_ASSET = 'behavior_smoking_public_notice_v1.mp3'
ENVIRONMENT_PHRASES = {'thermal_high': '环境温度异常', 'co2_high': '二氧化碳浓度异常'}
PATROL_SAFETY_KEY = 'patrol_safety'
PATROL_SAFETY_PHRASE = '正在执行巡检任务，请注意安全'


class BehaviorSpeechService:
    def __init__(self, audio_dir=None, cooldown_seconds=20.0):
        self._audio_dir = Path(audio_dir) if audio_dir else (
            Path(__file__).resolve().parent / 'behavior_audio')
        self._synth = shutil.which('espeak-ng') or shutil.which('espeak')
        self._player = shutil.which('ffplay')
        self._cooldown = max(1.0, float(cooldown_seconds))
        self._queue = queue.Queue(maxsize=2)
        self._patrol_notices = queue.Queue(maxsize=32)
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._generation = 0
        self._last = {}
        self._active = None
        self._active_patrol_notice = False
        self._error = ''
        self._thread = threading.Thread(target=self._run, name='behavior-speech', daemon=True)
        self._thread.start()

    def _command(self, key, *, inspection=False):
        phrase = PHRASES.get(key, ENVIRONMENT_PHRASES.get(key))
        if key == PATROL_SAFETY_KEY:
            phrase = PATROL_SAFETY_PHRASE
        if phrase is None:
            return None
        # Only these fixed filenames and phrases can be played; no shell input.
        public_notice = inspection and key == 'smoking'
        asset = self._audio_dir / (INSPECTION_SMOKING_ASSET if public_notice else f'behavior_{key}.mp3')
        if self._player and asset.is_file():
            return [self._player, '-hide_banner', '-loglevel', 'error',
                    '-nodisp', '-autoexit', '-vn', str(asset)]
        if self._synth:
            return [self._synth, '-v', 'cmn', '-s', '150',
                    INSPECTION_SMOKING_PHRASE if public_notice else phrase]
        return None

    def status(self):
        with self._lock:
            ready = (not self._closed.is_set() and all(self._command(k) for k in PHRASES)
                     and bool(self._command('smoking', inspection=True)))
            return {'ready': ready, 'error': self._error or (
                '' if ready else '行为告警语音不可用：请安装 espeak-ng 或准备固定语音素材')}

    def announce(self, key, *, inspection=False):
        now = time.monotonic()
        inspection = bool(inspection and key == 'smoking')
        with self._lock:
            if ((key not in PHRASES and key not in ENVIRONMENT_PHRASES) or self._closed.is_set()
                    or not self._command(key, inspection=inspection)
                    or now - self._last.get(key, float('-inf')) < self._cooldown):
                return False
            try:
                self._queue.put_nowait((self._generation, now, key, inspection))
            except queue.Full:
                return False
            self._last[key] = now
            return True

    def announce_patrol_safety(self):
        """Queue each arrival independently of alert cooldown and mode resets."""
        if self._closed.is_set() or not self._command(PATROL_SAFETY_KEY):
            return False
        try:
            self._patrol_notices.put_nowait(time.monotonic())
        except queue.Full:
            return False
        return True

    def reset(self):
        """Discard pending speech on stop, task or camera-source changes."""
        with self._lock:
            self._generation += 1
            self._last.clear()
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
            if (self._active is not None and self._active.poll() is None
                    and (self._closed.is_set() or not self._active_patrol_notice)):
                try:
                    self._active.terminate()
                except OSError:
                    pass

    def _run(self):
        while not self._closed.is_set():
            try:
                generation, created, key, inspection = self._queue.get_nowait()
            except queue.Empty:
                try:
                    created = self._patrol_notices.get(timeout=0.2)
                except queue.Empty:
                    continue
                generation, key, inspection = None, PATROL_SAFETY_KEY, False
            patrol_notice = key == PATROL_SAFETY_KEY
            with self._lock:
                if self._closed.is_set():
                    continue
                if patrol_notice and time.monotonic() - created > 30.0:
                    continue
                if not patrol_notice and (generation != self._generation
                                          or time.monotonic() - created > 5.0):
                    continue
                command = self._command(key, inspection=inspection)
                if not command:
                    continue
                try:
                    process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self._active = process
                    self._active_patrol_notice = patrol_notice
                except OSError as error:
                    self._error = str(error)
                    continue
            try:
                code = process.wait(timeout=30.0 if inspection else 12.0)
                with self._lock:
                    if patrol_notice or generation == self._generation:
                        self._error = '' if code == 0 else f'行为语音播放器退出码 {code}'
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
                with self._lock:
                    self._error = '行为语音播放超时'
            finally:
                with self._lock:
                    if self._active is process:
                        self._active = None
                        self._active_patrol_notice = False

    def close(self):
        self._closed.set()
        self.reset()
        self._thread.join(timeout=2.0)
        with self._lock:
            process = self._active
            if process is not None and process.poll() is None:
                try:
                    process.kill()
                except OSError:
                    pass
        self._thread.join(timeout=1.0)
