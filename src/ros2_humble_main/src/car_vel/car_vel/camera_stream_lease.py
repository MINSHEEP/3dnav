"""Camera-only demand, independent of the robot's motion/remote-control lease."""
import re
import threading
import time


class CameraStreamLease:
    def __init__(self, send, *, clock=time.monotonic, ttl=8.0):
        self._send, self._clock, self._ttl = send, clock, ttl
        self._lock = threading.RLock()
        self._leases = {}
        self._remote = self._enabled = self._closed = False

    def _expire(self):
        now = self._clock()
        self._leases = {token:expiry for token,expiry in self._leases.items() if expiry > now}

    def _sync(self):
        desired = not self._closed and (self._remote or bool(self._leases))
        if desired != self._enabled:
            if self._send(desired) is False: return False
            self._enabled = desired
        return True

    def request(self, payload):
        if not isinstance(payload,dict) or set(payload) != {'operation','token'}:
            raise ValueError('图传请求字段无效')
        operation, token = payload['operation'], payload['token']
        if operation not in ('acquire','release') or not isinstance(token,str) or not re.fullmatch('[a-f0-9]{32}',token):
            raise ValueError('图传租约无效')
        with self._lock:
            if self._closed: raise RuntimeError('本体图传桥接正在关闭')
            self._expire()
            if operation == 'acquire':
                if token not in self._leases and len(self._leases) >= 32:
                    raise RuntimeError('图传租约数量已满')
                self._leases[token] = self._clock() + self._ttl
            else:
                self._leases.pop(token, None)
            success = self._sync()
            return dict(success=success, enabled=self._enabled, ttl_seconds=self._ttl,
                        error='' if success else '本体图传命令发送失败，请检查本体连接')

    def remote(self, enabled):
        with self._lock:
            self._remote = bool(enabled)
            self._expire()
            self._sync()

    def tick(self):
        with self._lock:
            self._expire()
            self._sync()

    def close(self):
        with self._lock:
            self._closed = True
            self._leases.clear()
            self._remote = False
            self._sync()
