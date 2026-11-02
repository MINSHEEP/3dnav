"""Independent pose/fight workers with branch-owned frames and deadlines.

Only the gateway owns the camera. Fall and smoking share pose observations in
one worker; fight sampling uses its independent capture clock and bounded ring.
"""
import copy
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from behavior_supervisor import BehaviorSupervisor


class ParallelBehaviorSupervisor:
    def __init__(self, *, config_path=None, worker_factory=BehaviorSupervisor, **kwargs):
        self._lock = threading.RLock()
        self.config_path = Path(config_path).expanduser() if config_path else None
        self._enabled = {'fall': True, 'fight': True, 'smoking': True}
        self._config_error = ''
        if self.config_path and self.config_path.exists():
            try:
                self._enabled = BehaviorSupervisor._validate_config(json.loads(self.config_path.read_text(encoding='utf-8')))
            except (ValueError, OSError) as error:
                self._config_error = '行为识别配置无效：' + str(error)
        self._workers = {key: worker_factory(branch=key, **kwargs) for key in ('pose', 'fight')}
        self._owners = {'fall': 'pose', 'smoking': 'pose', 'fight': 'fight'}
        self._apply_config()

    def _apply_config(self):
        for key, worker in self._workers.items():
            worker.configure({f'{name}_enabled': active and self._owners[name] == key
                              for name, active in self._enabled.items()})

    def configure(self, value):
        enabled = BehaviorSupervisor._validate_config(value)
        with self._lock:
            if self.config_path:
                self.config_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.config_path.with_suffix('.tmp')
                temporary.write_text(json.dumps({f'{key}_enabled': active for key, active in enabled.items()}), encoding='utf-8')
                temporary.replace(self.config_path)
            self._enabled, self._config_error = enabled, ''
            self._apply_config()
            return self.status()

    def _call_all(self, method, *args, **kwargs):
        with self._lock:
            for worker in self._workers.values():
                getattr(worker, method)(*args, **kwargs)
            return self.status()

    def start(self): return self._call_all('start')
    def stop(self): return self._call_all('stop')
    def close(self): return self._call_all('close')
    def reset_source(self): return self._call_all('reset_source')
    def set_mode(self, mode, task_id=''): return self._call_all('set_mode', mode, task_id)

    def submit(self, frame, **kwargs):
        with self._lock:
            return any([worker.submit(frame, **kwargs) for worker in self._workers.values()])

    def _snapshot(self):
        return {key: worker.status() for key, worker in self._workers.items()}

    @staticmethod
    def _preview(statuses):
        candidates = [(key, value) for key, value in statuses.items() if value['running'] and value['fresh']]
        # Person overlays include both fall and smoking on the same real frame.
        pose = next((item for item in candidates if item[0] == 'pose'), None)
        if pose is not None:
            return pose
        return max(candidates, key=lambda item: item[1]['stamp_ns']) if candidates else (None, None)

    def status(self):
        with self._lock:
            statuses = self._snapshot()
            preview_key, preview = self._preview(statuses)
            base = copy.deepcopy(preview or statuses['pose'])
            active_workers = {self._owners[key] for key, active in self._enabled.items() if active}
            enabled = [s for key, s in statuses.items() if key in active_workers]
            errors = {key: s['error'] for key, s in statuses.items() if key in active_workers and s.get('error')}
            branches = {key: copy.deepcopy(statuses[owner]['branches'][key]) for key, owner in self._owners.items()}
            for key, branch in branches.items():
                s = statuses[self._owners[key]]
                branch.update(error=s.get('error'), fresh=s['fresh'] and branch.get('state') != 'stale', frame_age_seconds=s.get('frame_age_seconds'),
                              timing_ms=s.get('last_model_timing_ms', s.get('result', {}).get('timing_ms', {})),
                              capture_window=s.get('capture_window'),
                              last_result_latency_seconds=s.get('last_result_latency_seconds'),
                              stale_results=s.get('stale_results', 0))
            running = any(s['running'] for s in enabled)
            ready = bool(enabled and all(s['ready'] for s in enabled))
            base.update(enabled=dict(self._enabled), branches=branches, running=running, ready=ready,
                        partial=running and not ready, fresh=preview is not None,
                        error=self._config_error or ('；'.join(f'{key}: {error}' for key, error in errors.items()) or None),
                        diagnostics='\n'.join(f'[{key}] {s["diagnostics"]}' for key, s in statuses.items() if s.get('diagnostics')),
                        diagnostics_by_branch={key: s.get('diagnostics', '') for key, s in statuses.items()},
                        preview_branch=preview_key, source_epoch=f'{preview_key}:{preview["source_epoch"]}' if preview else None,
                        detections=[item for key, s in statuses.items() if key in active_workers for item in s.get('detections', [])],
                        manual_detection_enabled=any(s['manual_detection_enabled'] for s in statuses.values()),
                        workers={key: dict(running=s['running'], ready=s['ready'], error=s.get('error')) for key, s in statuses.items()})
            if not preview:
                base.update(stamp_ns=None, frame_stamp_ns=None, frame_age_seconds=None, result={})
            return base

    def frame(self):
        with self._lock:
            key, status = self._preview(self._snapshot())
            if key is None:
                raise RuntimeError('行为标注帧尚未就绪或已过期')
            image, epoch, stamp = self._workers[key].frame()
            return image, f'{key}:{epoch}', stamp

    def capture(self, duration=2., task_id=''):
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(worker.capture, duration, task_id) for worker in self._workers.values()]
            captures = [{**future.result(), 'worker_branch': key} for key, future in zip(self._workers, futures)]
        # Each capture keeps its OWN raw/annotated pair, epoch and timestamp.
        # Never attach a fight result to the pose worker's frame or vice versa.
        usable = [value for value in captures if value.get('fresh')]
        primary = next((v for v in usable if any(d.get('alert') for d in v['detections'])), usable[0] if usable else {})
        result = copy.deepcopy(primary)
        result.update(branch_captures=[v for v in usable if v is not primary], behavior=self.status(),
                      partial=any(v.get('partial') for v in captures),
                      error='；'.join(v['error'] for v in captures if v.get('error')))
        result.setdefault('detections', [])
        return result

    def events(self, task_id):
        with self._lock:
            return [{**event, 'worker_branch': key} for key, worker in self._workers.items() for event in worker.events(task_id)]
