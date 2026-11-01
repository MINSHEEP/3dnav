"""Fail-closed odometry policy for calibrated terrain connectors."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OdomSample:
    x: float
    y: float
    z: float
    yaw: float
    received_at: float


@dataclass(frozen=True)
class DogHealth:
    connected: bool
    standing: bool
    hardware_error: bool
    received_at: float


@dataclass(frozen=True)
class ConnectorCommand:
    state: str
    linear_x: float
    angular_z: float
    progress: float
    message: str


class ConnectorTraversal:
    """Monitor one straight, pre-aligned ramp/stair/curb connector."""

    def __init__(
            self, *, distance_m, speed_mps, expected_height_delta_m=0.0,
            lateral_limit_m=0.25, yaw_limit_deg=10.0,
            odom_timeout_sec=0.75, health_timeout_sec=1.0,
            stall_timeout_sec=4.0, completion_tolerance_m=0.20,
            height_tolerance_m=0.35):
        self.distance_m = float(distance_m)
        self.speed_mps = min(0.10, max(0.04, float(speed_mps)))
        self.expected_height_delta_m = float(expected_height_delta_m)
        self.lateral_limit_m = float(lateral_limit_m)
        self.yaw_limit = math.radians(float(yaw_limit_deg))
        self.odom_timeout_sec = float(odom_timeout_sec)
        self.health_timeout_sec = float(health_timeout_sec)
        self.stall_timeout_sec = float(stall_timeout_sec)
        self.completion_tolerance_m = float(completion_tolerance_m)
        self.height_tolerance_m = float(height_tolerance_m)
        if not all(math.isfinite(value) for value in (
                self.distance_m, self.speed_mps,
                self.expected_height_delta_m)):
            raise ValueError('connector parameters must be finite')
        if self.distance_m <= 0.3:
            raise ValueError('connector distance must exceed 0.3 m')
        self.started_at = None
        self.start = None
        self.best_progress = 0.0
        self.last_progress_at = None

    @staticmethod
    def _angle_delta(value, reference):
        return math.atan2(
            math.sin(value - reference), math.cos(value - reference))

    def begin(self, now, odom: OdomSample, health: DogHealth):
        initial = self._validate_inputs(now, odom, health)
        if initial:
            return initial
        self.started_at = float(now)
        self.start = odom
        self.last_progress_at = float(now)
        return ConnectorCommand(
            'running', 0.0, 0.0, 0.0,
            '连接器监控已启动，先发送零速确认')

    def update(self, now, odom: OdomSample, health: DogHealth):
        if self.start is None or self.started_at is None:
            return ConnectorCommand(
                'failed', 0.0, 0.0, 0.0, '连接器尚未启动')
        invalid = self._validate_inputs(now, odom, health)
        if invalid:
            return invalid
        elapsed = float(now) - self.started_at
        maximum_duration = self.distance_m / self.speed_mps * 2.0 + 10.0
        if elapsed > maximum_duration:
            return self._failed('连接器执行超时')

        dx = odom.x - self.start.x
        dy = odom.y - self.start.y
        cosine = math.cos(self.start.yaw)
        sine = math.sin(self.start.yaw)
        progress_m = cosine * dx + sine * dy
        lateral_m = -sine * dx + cosine * dy
        yaw_error = self._angle_delta(odom.yaw, self.start.yaw)
        height_delta = odom.z - self.start.z
        progress = max(0.0, min(1.0, progress_m / self.distance_m))
        if progress_m < -0.10:
            return self._failed('机器人在连接器上出现后退')
        if abs(lateral_m) > self.lateral_limit_m:
            return self._failed('连接器横向偏移超过安全限制')
        if abs(yaw_error) > self.yaw_limit:
            return self._failed('连接器航向漂移超过安全限制')

        if progress_m > self.best_progress + 0.03:
            self.best_progress = progress_m
            self.last_progress_at = float(now)
        elif float(now) - self.last_progress_at > self.stall_timeout_sec:
            return self._failed('连接器上持续无有效前进，判定停滞')

        reached_distance = (
            progress_m >= self.distance_m - self.completion_tolerance_m)
        reached_height = (
            abs(self.expected_height_delta_m) < self.height_tolerance_m or
            abs(height_delta - self.expected_height_delta_m) <=
            self.height_tolerance_m)
        if reached_distance and reached_height:
            return ConnectorCommand(
                'completed', 0.0, 0.0, 1.0,
                '已通过连接器并满足距离/高差约束')
        if reached_distance and not reached_height:
            return self._failed('已到预计距离但楼层高差不匹配')
        return ConnectorCommand(
            'running', self.speed_mps, 0.0, progress,
            f'连接器进度 {progress * 100:.0f}%')

    def _validate_inputs(self, now, odom, health):
        if odom is None or float(now) - odom.received_at > self.odom_timeout_sec:
            return self._failed('三维里程计已过期')
        if health is None or float(now) - health.received_at > self.health_timeout_sec:
            return self._failed('机器狗状态反馈已过期')
        if not health.connected:
            return self._failed('机器狗通信中断')
        if not health.standing:
            return self._failed('机器狗不处于站立状态')
        if health.hardware_error:
            return self._failed('机器狗报告硬件故障')
        return None

    @staticmethod
    def _failed(message):
        return ConnectorCommand('failed', 0.0, 0.0, 0.0, message)

