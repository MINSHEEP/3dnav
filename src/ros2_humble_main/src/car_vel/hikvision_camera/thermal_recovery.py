"""Isolate the vendor native SDK from HTTP; never reset USB or change drivers."""
import multiprocessing
import os
from pathlib import Path
import sys
import threading
import time


def probe_usb(root=Path('/sys/bus/usb/devices'), *, serial='', index=0):
    devices = []
    for path in sorted(root.glob('*')):
        try:
            if (path / 'idVendor').read_text().strip().lower() != '2bdf' or \
                    (path / 'idProduct').read_text().strip().lower() != '0102':
                continue
            identity = (path / 'serial').read_text().strip() if (path / 'serial').exists() else ''
            if serial and identity != serial:
                continue
            bus = int((path / 'busnum').read_text().strip())
            device = int((path / 'devnum').read_text().strip())
            devices.append((path.name, bus, device, identity))
        except (OSError, ValueError):
            continue  # Hot unplug during sysfs traversal.
    if not devices or (not serial and index >= len(devices)):
        return None, 'USB尚未识别目标海康热成像（2bdf:0102）；等待设备接入，未初始化SDK'
    # Device-index ordering in sysfs need not equal the vendor enumeration.
    # Validate access to every candidate; selection itself remains in the SDK.
    for _, bus, device, _ in devices:
        node = f'/dev/bus/usb/{bus:03d}/{device:03d}'
        if not os.access(node, os.R_OK | os.W_OK):
            return None, f'热成像USB权限不足：{node}；请检查udev规则和plugdev组'
    return tuple(devices), ''


class RetryBudget:
    def __init__(self, limit=5, delay=3.0):
        self.limit, self.delay = limit, delay
        self.token, self.failures, self.due = None, 0, 0.0

    def allowed(self, token, now):
        if token != self.token:
            self.token, self.failures, self.due = token, 0, 0.0
        return self.failures < self.limit and now >= self.due

    def failed(self, now):
        self.failures += 1
        self.due = now + min(30.0, self.delay * (2 ** min(self.failures - 1, 4)))


class WorkerMonitor:
    """Only sample/state messages cross the process boundary; no config writes."""
    def __init__(self, connection, threshold):
        self.connection, self.threshold = connection, threshold

    def set_backend_state(self, **state):
        self.connection.send(('state', state))

    def status(self):
        return {'config': {'threshold_c': self.threshold.value}}

    def ingest(self, sample):
        self.connection.send(('sample', sample))


def capture_worker(args, connection, stop, threshold):
    # Spawn (never fork) means each attempt has clean native library state.
    from thermal_web import HCUsbProducer, enable_native_diagnostics
    enable_native_diagnostics()
    lock = None
    try:
        import fcntl
        lock_path = Path.home() / '.ros' / 'robot_navigation' / 'thermal_sdk.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock = lock_path.open('a')
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        producer = HCUsbProducer(WorkerMonitor(connection, threshold), args)
        producer._stop = stop
        # Exactly ONE session. A failed native session is never reused.
        producer._run_session()
    except Exception as error:
        connection.send(('state', {'running': False, 'error': str(error)}))
        print(f'[ThermalRecovery] worker error: {error}', file=sys.stderr, flush=True)
        raise
    finally:
        if lock is not None:
            lock.close()
        connection.close()


class IsolatedThermalProducer:
    def __init__(self, monitor, args, *, probe=None, worker=capture_worker):
        self.monitor, self.args = monitor, args
        self.probe = probe or (lambda: probe_usb(serial=args.serial, index=args.device_index))
        self.worker = worker
        self.context = multiprocessing.get_context('spawn')
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='thermal-supervisor')
        self._last_error = None

    def start(self):
        self._thread.start()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=8)

    def report(self, error):
        self.monitor.set_backend_state(running=False, error=error)
        if error != self._last_error:
            print(f'[ThermalRecovery] {error}', file=sys.stderr, flush=True)
            self._last_error = error

    def _run(self):
        budget = RetryBudget(delay=self.args.reconnect_delay)
        while not self._stop.is_set():
            adapter = self.args.bundle_dir / 'PythonDemo' / 'thermal_max_cv.py'
            token, error = self.probe()
            if not adapter.is_file():
                error = f'找不到附件热成像适配模块：{adapter}'
            if error:
                self.report(error)
                self._stop.wait(1)
                continue
            if not budget.allowed(token, time.monotonic()):
                if budget.failures >= budget.limit:
                    self.report('SDK连续失败5次，已暂停重试；重新接入相机或重启热成像服务后重试')
                self._stop.wait(1)
                continue
            self.report('USB已识别，正在独立进程中启动热成像SDK')
            try:
                healthy_duration, error = self._session(token)
            except Exception as exc:
                healthy_duration, error = 0, f'热成像采集子进程启动失败：{exc}'
            if self._stop.is_set():
                break
            if healthy_duration >= 60:
                budget.failures = 0
            budget.failed(time.monotonic())
            self.report(f'{error}；重试 {budget.failures}/{budget.limit}，等待USB或退避恢复')
        self.report('热成像采集已停止')

    def _session(self, token):
        receiver, sender = self.context.Pipe(duplex=False)
        stop = self.context.Event()
        threshold = self.context.Value('d', self.monitor.status()['config']['threshold_c'])
        child = self.context.Process(target=self.worker,
                                     args=(self.args, sender, stop, threshold), daemon=True)
        started = False
        first_frame = None
        last_message = time.monotonic()
        next_probe = last_message
        error = '热成像采集进程已退出'
        try:
            child.start()
            started = True
            sender.close()
            while not self._stop.is_set():
                now = time.monotonic()
                threshold.value = self.monitor.status()['config']['threshold_c']
                if now >= next_probe:
                    current, usb_error = self.probe()
                    next_probe = now + 1
                    if usb_error or current != token:
                        error = usb_error or '热成像USB设备已更换，重新建立采集'
                        break
                try:
                    message = receiver.recv() if receiver.poll(0.1) else None
                except (EOFError, OSError):
                    child.join(timeout=0.5)
                    error = f'热成像SDK采集进程退出，退出码 {child.exitcode}'
                    break
                if message is not None:
                    kind, payload = message
                    last_message = now
                    if kind == 'state':
                        self.monitor.set_backend_state(**payload)
                        if not payload.get('running'):
                            error = payload.get('error') or error
                            break
                    elif kind == 'sample':
                        if time.time() - payload.captured_at_epoch > self.args.stale_seconds:
                            continue
                        self.monitor.ingest(payload)
                        first_frame = now if first_frame is None else first_frame
                        self._last_error = None
                # Drain messages already written before handling EOF. An exit
                # must not discard a final frame queued behind the state message.
                timeout = 45 if first_frame is None else max(10, self.args.frame_timeout + 5)
                if now - last_message > timeout:
                    error = '热成像SDK长时间没有输出，回收本次采集进程'
                    break
        finally:
            # Invalidate before waiting for SDK cleanup; no old image/alarm can
            # appear as a frame from a replacement device or session.
            self.monitor.set_backend_state(running=False, error=error)
            stop.set()
            if started:
                child.join(timeout=2)
                if child.is_alive():
                    child.terminate()
                    child.join(timeout=1)
                if child.is_alive():
                    child.kill()
                    child.join(timeout=1)
            receiver.close()
            sender.close()
        return (0 if first_frame is None else time.monotonic() - first_frame), error
