"""Pure hysteresis policy for automatic same-map terrain traversal."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable, Optional

from .terrain_navigation import (
    TerrainCapabilityModel,
    TerrainCapabilityProfile,
    TerrainObservation,
)


AUTOMATIC_MODES = frozenset({'rough_ground', 'obstacle', 'ramp'})
AUTOMATIC_MAX_SLOPE_DEG = 18.0
AUTOMATIC_MAX_STEP_M = 0.12


def _automatic_profile(profile: TerrainCapabilityProfile):
    """Clamp editable connector capability to the autonomous envelope."""
    return replace(
        profile,
        max_up_step_m=min(profile.max_up_step_m, AUTOMATIC_MAX_STEP_M),
        max_down_step_m=min(profile.max_down_step_m, AUTOMATIC_MAX_STEP_M),
        max_up_slope_deg=min(
            profile.max_up_slope_deg, AUTOMATIC_MAX_SLOPE_DEG),
        max_down_slope_deg=min(
            profile.max_down_slope_deg, AUTOMATIC_MAX_SLOPE_DEG),
    )


@dataclass(frozen=True)
class AutomaticTerrainDecision:
    """One fail-closed output consumed by the ROS adapter and arbiter."""

    safe: bool
    state: str
    current_mode: str
    requested_mode: str
    speed_limit_mps: float
    reason: str


class AutomaticTerrainController:
    """Debounce live terrain before changing the quadruped motion mode."""

    def __init__(
            self, capability: Optional[TerrainCapabilityModel] = None,
            *, enter_confirmations: int = 3,
            exit_confirmations: int = 5):
        profile = (
            capability.profile if capability is not None
            else TerrainCapabilityProfile())
        self.capability = TerrainCapabilityModel(_automatic_profile(profile))
        self.enter_confirmations = max(2, int(enter_confirmations))
        self.exit_confirmations = max(
            self.enter_confirmations, int(exit_confirmations))
        self.enabled = True
        self.enabled_modes = set(AUTOMATIC_MODES)
        self.current_mode = 'normal'
        self._candidate_mode = ''
        self._candidate_count = 0
        self._candidate_sequence = None
        self._last_switch_error = ''

    def configure(
            self, *, enabled: bool, enabled_modes: Iterable[str],
            profile: Optional[TerrainCapabilityProfile] = None):
        """Apply validated runtime policy without enabling stair modes."""
        modes = {
            str(value).strip().lower() for value in enabled_modes
            if str(value).strip().lower() in AUTOMATIC_MODES}
        self.enabled = bool(enabled)
        self.enabled_modes = modes
        if profile is not None:
            self.capability = TerrainCapabilityModel(
                _automatic_profile(profile))
        self._reset_candidate()

    def _reset_candidate(self):
        self._candidate_mode = ''
        self._candidate_count = 0
        self._candidate_sequence = None

    def _candidate(self, mode, sequence=None):
        if mode != self._candidate_mode:
            self._candidate_mode = mode
            self._candidate_count = 0
            self._candidate_sequence = None
        if sequence is None or sequence != self._candidate_sequence:
            self._candidate_count += 1
            self._candidate_sequence = sequence
        return self._candidate_count

    def confirm_mode(self, mode: str, *, success: bool, message: str = ''):
        """Commit a hardware-confirmed service result."""
        requested = str(mode).strip().lower()
        if success:
            self.current_mode = requested or 'normal'
            self._last_switch_error = ''
        else:
            self._last_switch_error = str(message or '运动模式切换失败')
        self._reset_candidate()

    def assume_mode(self, mode: str):
        """Synchronise after another exclusive owner controlled the robot."""
        self.current_mode = str(mode).strip().lower() or 'normal'
        self._last_switch_error = ''
        self._reset_candidate()

    def _decision(
            self, safe, state, reason, *, requested='', speed=0.0):
        return AutomaticTerrainDecision(
            safe=bool(safe), state=str(state),
            current_mode=self.current_mode,
            requested_mode=str(requested),
            speed_limit_mps=max(0.0, float(speed)),
            reason=str(reason))

    def update(
            self, observation: Optional[TerrainObservation], *,
            observation_fresh: bool, navigation_active: bool,
            campus_active: bool, observation_sequence=None,
            check_confidence: bool = True,
            fixed_execution_mode: str = '') -> AutomaticTerrainDecision:
        """Return a safe speed/mode decision for the latest observation."""
        fixed = str(fixed_execution_mode).strip().lower()
        if fixed not in ('', 'obstacle'):
            return self._decision(False, 'blocked', '未知固定执行策略')
        # Fixed Xingji execution keeps geometry validation but delegates speed
        # changes to Nav2 and the existing system/body cap. A traversable curb,
        # ramp or rough patch no longer adds a category-driven speed limit.
        fixed_speed = self.capability.profile.normal_speed_mps
        if campus_active:
            self._reset_candidate()
            return self._decision(
                True, 'suspended', '园区连接任务拥有运动模式控制权',
                speed=self.capability.profile.special_speed_mps)

        if fixed and not navigation_active:
            self._reset_candidate()
            return self._decision(
                True, 'idle', '无导航任务；保留越障执行策略，不发送本体指令',
                speed=fixed_speed)

        if not self.enabled:
            self._reset_candidate()
            target = fixed or 'normal'
            requested = target if self.current_mode != target else ''
            if navigation_active:
                if not observation_fresh or observation is None:
                    return self._decision(False, 'blocked', '地形观测缺失；关闭自动步态不会关闭地形安全检查')
                assessment = self.capability.assess(
                    observation, check_confidence=check_confidence)
                if not assessment.traversable or assessment.required_mode != 'normal':
                    return self._decision(False, 'blocked', '自动地形步态已关闭：'+assessment.reason)
            return self._decision(
                not requested, 'disabled', '自动地形步态已关闭，普通地面安全检查仍有效',
                requested=requested,
                speed=fixed_speed if fixed else self.capability.profile.normal_speed_mps)

        if not navigation_active:
            self._reset_candidate()
            requested = '' if fixed else ('normal' if self.current_mode != 'normal' else '')
            return self._decision(
                True, 'idle', 'Nav2当前没有执行导航任务',
                requested=requested,
                speed=fixed_speed if fixed else self.capability.profile.normal_speed_mps)

        if not observation_fresh or observation is None:
            self._reset_candidate()
            return self._decision(
                False, 'blocked', '实时地形观测缺失或已过期')

        assessment = self.capability.assess(
            observation, self.enabled_modes, check_confidence=check_confidence)
        if not assessment.traversable:
            self._reset_candidate()
            return self._decision(
                False, 'blocked', assessment.reason)

        desired = assessment.required_mode or 'normal'
        if desired not in ({'normal'} | AUTOMATIC_MODES):
            self._reset_candidate()
            return self._decision(
                False, 'blocked', f'{desired} 不允许自动执行')
        if desired != 'normal' and desired not in self.enabled_modes:
            self._reset_candidate()
            return self._decision(
                False, 'blocked', f'{desired} 已在自动地形设置中关闭')
        # Validate REAL terrain first. Only the execution mode is held constant;
        # unknown terrain, stairs and geometric hazards remain blocked.
        if fixed:
            if fixed not in self.enabled_modes:
                self._reset_candidate()
                return self._decision(False, 'blocked', '固定越障执行所需模式已在设置中关闭')
            desired = fixed
        if desired == self.current_mode:
            self._reset_candidate()
            speed = fixed_speed if fixed else assessment.speed_limit_mps
            return self._decision(
                True, 'ready',
                ('星玑保持越障模式；速度由导航控制器及本体上限约束'
                 if fixed else assessment.reason),
                speed=speed)

        count = self._candidate(desired, observation_sequence)
        threshold = (
            self.exit_confirmations if desired == 'normal'
            else self.enter_confirmations)
        if count < threshold:
            if desired == 'normal' and self.current_mode != 'normal':
                return self._decision(
                    True, 'exit_confirming',
                    f'继续确认平地 {count}/{threshold}，暂时保持低速模式',
                    speed=self.capability.profile.special_speed_mps)
            return self._decision(
                False, 'enter_confirming',
                f'正在确认 {desired} {count}/{threshold}，切换前保持停车')

        return self._decision(
            False, 'switching', f'请求切换到 {desired}',
            requested=desired)
