"""Record both C12 streams independently without transcoding."""
import datetime
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time


class C12Recorder:
    def __init__(self, visible_url, infrared_url, directory=None):
        self.urls = {'visible': visible_url, 'infrared': infrared_url}
        self.directory = Path(directory) if directory else Path.home() / '巡检录像'
        self.lock = threading.RLock()
        self.processes = {}
        self.files = {}
        self.state = 'idle'
        self.error = ''
        self.worker = None

    def status(self):
        with self.lock:
            return {'state': self.state, 'error': self.error,
                    'files': {key: str(value) for key, value in self.files.items()}}

    def start(self):
        with self.lock:
            if self.state in ('starting', 'recording', 'stopping'):
                raise ValueError('录像正在进行或停止中')
            ffmpeg = shutil.which('ffmpeg')
            if not ffmpeg:
                raise RuntimeError('未找到 ffmpeg')
            self.directory.mkdir(parents=True, exist_ok=True)
            label = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.files = {name: self.directory / f'{label}_{name}.mp4' for name in self.urls}
            self.error = ''
            self.state = 'starting'
            try:
                for name, url in self.urls.items():
                    command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
                               '-rtsp_transport', 'tcp', '-stimeout', '5000000', '-i', url,
                               '-map', '0:v:0', '-an', '-c:v', 'copy',
                               '-movflags', 'frag_keyframe+empty_moov', '-n', str(self.files[name])]
                    self.processes[name] = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                                            stderr=subprocess.DEVNULL)
            except (OSError, ValueError):
                self._finish()
                self.state = 'error'
                self.error = '无法启动录像进程'
                raise
            self.worker = threading.Thread(target=self._monitor, daemon=True, name='c12-recording')
            self.worker.start()
            return self.status()

    def _monitor(self):
        started = time.monotonic()
        while True:
            time.sleep(0.5)
            with self.lock:
                if self.state != 'stopping':
                    failed = [name for name, proc in self.processes.items() if proc.poll() is not None]
                    if failed:
                        self.error = f"{'、'.join(failed)} 录像流中断，已停止两路录像"
                        self.state = 'stopping'
                    elif all(path.exists() and path.stat().st_size > 4096 for path in self.files.values()):
                        self.state = 'recording'
                    elif self.state == 'starting' and time.monotonic() - started > 20:
                        self.error = '等待双路视频超时，已停止录像'
                        self.state = 'stopping'
                stopping = self.state == 'stopping'
            if stopping:
                # FFmpeg may need several seconds to finalize MP4. Keep status
                # requests responsive while the existing worker waits for it.
                self._finish()
                with self.lock:
                    self.state = 'error' if self.error else 'idle'
                return

    def _finish(self):
        for proc in self.processes.values():
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in self.processes.values():
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        self.processes = {}

    def stop(self):
        with self.lock:
            if self.state not in ('starting', 'recording'):
                raise ValueError('当前没有正在进行的录像')
            self.state = 'stopping'
            return self.status()

    def close(self):
        with self.lock:
            worker = self.worker
            if self.state in ('starting', 'recording'):
                self.state = 'stopping'
        if worker is not None:
            worker.join()
