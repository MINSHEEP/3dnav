"""C12 infrared video without borrowing another device's measurements."""
from thermal_monitor import ThermalConfigStore, ThermalMonitor


TEMPERATURE_UNAVAILABLE = 'C12 测温数据未接入，待原厂确认温度读取协议'


class C12ThermalClient:
    def __init__(self, source, config_path):
        self.source = source
        self.monitor = ThermalMonitor(ThermalConfigStore(config_path))
        self.monitor.set_backend_state(running=False, error=TEMPERATURE_UNAVAILABLE)

    def status(self):
        return {**self.monitor.status(), 'pipeline': 'c12',
                'supports_temperature': False, 'detection_enabled': False, 'video_ready': self.source.status()['ready']}

    def frame(self, *, raw=False):
        return self.source.next_frame()[1]

    def open_stream(self):
        return C12MjpegStream(self.source)

    def config(self, task_id=''):
        return {'success': True, 'config': self.monitor.config_store.get(task_id),
                'supports_temperature': False, 'error': TEMPERATURE_UNAVAILABLE}

    def update_config(self, payload):
        raise RuntimeError(TEMPERATURE_UNAVAILABLE)

    def set_detection(self, enabled):
        if enabled:
            raise RuntimeError(TEMPERATURE_UNAVAILABLE)
        self.monitor.set_detection(False)
        return self.status()

    def set_mode(self, mode, task_id=''):
        self.monitor.set_mode(mode, task_id)
        return self.status()

    def capture(self, duration_seconds):
        raise RuntimeError(TEMPERATURE_UNAVAILABLE)

    def events(self, since):
        return self.monitor.events(since)


class C12MjpegStream:
    headers = {'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}

    def __init__(self, source):
        self.source = source
        self.sequence, self.pending = source.next_frame()
        self.closed = False

    def read(self, _size):
        if self.closed:
            return b''
        if self.pending is None:
            self.sequence, self.pending = self.source.next_frame(self.sequence)
        image, self.pending = self.pending, None
        return (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                + str(len(image)).encode() + b'\r\n\r\n' + image + b'\r\n')

    read1 = read

    def close(self):
        self.closed = True
