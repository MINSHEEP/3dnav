"""HTTP production adapter for the owned thermal process."""

from __future__ import annotations

import json
import socket
from typing import Any
import urllib.error
import urllib.request
from urllib.parse import urlencode


class ThermalHttpClient:
    """Small interface used by ``CameraGateway`` across the process seam."""

    def __init__(self, base_url: str, timeout: float = 3.0) -> None:
        self.base_url = str(base_url).rstrip("/")
        self.timeout = max(0.2, float(timeout))
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _request(
        self,
        path: str,
        payload: dict[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            self.base_url + path, data=data, headers=headers,
            method="POST" if data is not None else "GET")
        try:
            with self._opener.open(request, timeout=timeout or self.timeout) as response:
                body = response.read(2 * 1024 * 1024)
        except urllib.error.HTTPError as error:
            try:
                detail = json.loads(error.read().decode("utf-8")).get("error")
            except (ValueError, UnicodeDecodeError, AttributeError):
                detail = ""
            if error.code == 409:
                raise ValueError(detail or "热成像阈值配置冲突") from error
            raise RuntimeError(detail or f"热成像服务返回HTTP {error.code}") from error
        except (urllib.error.URLError, TimeoutError, socket.timeout, OSError) as error:
            raise RuntimeError(f"热成像服务不可用：{error}") from error
        try:
            result = json.loads(body.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as error:
            raise RuntimeError("热成像服务返回了无效JSON") from error
        if not isinstance(result, dict):
            raise RuntimeError("热成像服务返回值不是对象")
        return result

    def status(self) -> dict[str, Any]:
        try:
            return self._request("/api/status")
        except RuntimeError as error:
            return {
                "available": False,
                "running": False,
                "ready": False,
                "error": str(error),
                "detection_enabled": False,
                "manual_detection_enabled": False,
                "config": {},
                "stream_proxy_url": "/api/thermal/stream.mjpg",
                "frame_url": "/api/thermal/frame.jpg",
            }

    def set_detection(self, enabled: bool) -> dict[str, Any]:
        return self._request("/api/detection", {"enabled": bool(enabled)})

    def set_mode(self, mode: str, task_id: str = "") -> dict[str, Any]:
        return self._request("/api/mode", {"mode": mode, "task_id": task_id})

    def config(self, task_id: str = "") -> dict[str, Any]:
        suffix = "?" + urlencode({"task_id": task_id}) if task_id else ""
        return self._request("/api/config" + suffix)

    def update_config(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._request("/api/config", payload)

    def capture(self, duration_seconds: float) -> dict[str, Any]:
        return self._request(
            "/api/capture", {"duration_seconds": float(duration_seconds)},
            timeout=max(self.timeout, float(duration_seconds) + 3.0))

    def events(self, since: int) -> dict[str, Any]:
        return self._request("/api/events?" + urlencode({"since": max(0, int(since))}))

    def frame(self, *, raw: bool = False) -> bytes:
        request = urllib.request.Request(
            self.base_url + ("/api/raw.jpg" if raw else "/api/frame.jpg"),
            headers={"Accept": "image/jpeg"})
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                image = response.read(16 * 1024 * 1024)
        except (urllib.error.URLError, TimeoutError, socket.timeout, OSError) as error:
            raise RuntimeError(f"热成像单帧不可用：{error}") from error
        if not image.startswith(b"\xff\xd8"):
            raise RuntimeError("热成像单帧不是JPEG")
        return image

    def open_stream(self):
        request = urllib.request.Request(
            self.base_url + "/api/stream.mjpg",
            headers={"Accept": "multipart/x-mixed-replace"})
        try:
            response = self._opener.open(request, timeout=max(10.0, self.timeout))
        except (urllib.error.URLError, TimeoutError, socket.timeout, OSError) as error:
            raise RuntimeError(f"热成像视频流不可用：{error}") from error
        content_type = str(response.headers.get("Content-Type", "")).lower()
        if "multipart/x-mixed-replace" not in content_type:
            response.close()
            raise RuntimeError("热成像服务没有返回MJPEG流")
        return response

