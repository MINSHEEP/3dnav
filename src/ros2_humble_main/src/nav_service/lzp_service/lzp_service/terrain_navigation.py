"""Terrain capability checks and campus-scale topological route planning."""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


ORDINARY_TERRAINS = frozenset({'flat', 'paved', 'indoor'})
CAUTION_TERRAINS = frozenset({'grass', 'gravel', 'rough', 'curb'})
SPECIAL_TERRAINS = frozenset({'ramp', 'stairs_up', 'stairs_down'})
BLOCKED_TERRAINS = frozenset({'negative_obstacle', 'cliff', 'water', 'blocked'})
KNOWN_TERRAINS = (
    ORDINARY_TERRAINS | CAUTION_TERRAINS
    | SPECIAL_TERRAINS | BLOCKED_TERRAINS | {'unknown'}
)


def _finite(value: object, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{name} must be finite')
    return number


@dataclass(frozen=True)
class TerrainObservation:
    """A local 2.5-D terrain estimate consumed by the capability model."""

    terrain_type: str = 'flat'
    slope_deg: float = 0.0
    cross_slope_deg: float = 0.0
    step_height_m: float = 0.0
    gap_width_m: float = 0.0
    roughness_m: float = 0.0
    clearance_width_m: float = 1.0
    confidence: float = 1.0
    negative_obstacle: bool = False
    slippery: bool = False

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> 'TerrainObservation':
        terrain_type = str(value.get('terrain_type', 'flat')).strip().lower()
        if terrain_type not in KNOWN_TERRAINS:
            terrain_type = 'unknown'
        return cls(
            terrain_type=terrain_type,
            slope_deg=_finite(value.get('slope_deg', 0.0), 'slope_deg'),
            cross_slope_deg=_finite(
                value.get('cross_slope_deg', 0.0), 'cross_slope_deg'),
            step_height_m=_finite(
                value.get('step_height_m', 0.0), 'step_height_m'),
            gap_width_m=_finite(
                value.get('gap_width_m', 0.0), 'gap_width_m'),
            roughness_m=_finite(
                value.get('roughness_m', 0.0), 'roughness_m'),
            clearance_width_m=_finite(
                value.get('clearance_width_m', 1.0), 'clearance_width_m'),
            confidence=_finite(value.get('confidence', 1.0), 'confidence'),
            negative_obstacle=bool(value.get('negative_obstacle', False)),
            slippery=bool(value.get('slippery', False)),
        )


@dataclass(frozen=True)
class TerrainAssessment:
    """The small decision interface presented to planners and executors."""

    decision: str
    required_mode: str
    cost_multiplier: float
    speed_limit_mps: float
    reason: str

    @property
    def traversable(self) -> bool:
        return self.decision != 'blocked'

    @property
    def requires_special_mode(self) -> bool:
        return self.required_mode not in ('', 'normal')


@dataclass(frozen=True)
class TerrainCapabilityProfile:
    """Conservative limits until field calibration proves higher values."""

    max_up_step_m: float = 0.15
    max_down_step_m: float = 0.12
    max_up_slope_deg: float = 20.0
    max_down_slope_deg: float = 18.0
    max_cross_slope_deg: float = 12.0
    max_gap_width_m: float = 0.10
    max_roughness_m: float = 0.06
    min_clearance_width_m: float = 0.65
    min_confidence: float = 0.70
    normal_speed_mps: float = 0.25
    caution_speed_mps: float = 0.15
    special_speed_mps: float = 0.10

    @classmethod
    def from_mapping(
            cls, value: Optional[Mapping[str, object]] = None,
            fallback: Optional['TerrainCapabilityProfile'] = None,
    ) -> 'TerrainCapabilityProfile':
        """Build a complete profile while ignoring unknown future fields."""
        source = value if isinstance(value, Mapping) else {}
        base = fallback or cls()
        values = {}
        for item in fields(cls):
            values[item.name] = _finite(
                source.get(item.name, getattr(base, item.name)), item.name)
        return cls(**values)

    def as_mapping(self) -> Dict[str, float]:
        """Return a JSON-safe representation for the browser editor."""
        return {key: float(value) for key, value in asdict(self).items()}


class TerrainCapabilityModel:
    """Hide all terrain-limit policy behind one deterministic interface."""

    def __init__(self, profile: Optional[TerrainCapabilityProfile] = None):
        self.profile = profile or TerrainCapabilityProfile()

    def assess(
            self, observation: TerrainObservation,
            available_modes: Iterable[str] = (), *,
            check_confidence: bool = True) -> TerrainAssessment:
        """Assess geometry, retaining score checks unless explicitly opted out.

        The opt-out skips only this aggregate sampling-score threshold. It
        does not turn unknown terrain, invalid data or measured hazards free.
        """
        modes = {str(mode).strip().lower() for mode in available_modes}
        profile = self.profile
        terrain = observation.terrain_type

        if observation.negative_obstacle or terrain in BLOCKED_TERRAINS:
            return self._blocked('检测到坑洞、临边、水体或明确禁行区域')
        if terrain == 'unknown':
            return self._blocked('地形类型未知，必须先完成可通行性确认')
        if (check_confidence is not False
                and observation.confidence < profile.min_confidence):
            return self._blocked('地形观测置信度不足，禁止盲目通行')
        if observation.slippery:
            return self._blocked('地面被标记为湿滑，尚无可靠抓地能力反馈')
        if observation.clearance_width_m < profile.min_clearance_width_m:
            return self._blocked('通行宽度小于机器狗安全包络')
        if observation.gap_width_m > profile.max_gap_width_m:
            return self._blocked('沟槽宽度超过已验证能力')
        if abs(observation.cross_slope_deg) > profile.max_cross_slope_deg:
            return self._blocked('横坡超过保守安全限制')
        slope_limit = (
            profile.max_up_slope_deg
            if observation.slope_deg >= 0.0
            else profile.max_down_slope_deg
        )
        if abs(observation.slope_deg) > slope_limit:
            return self._blocked('纵坡超过保守安全限制')
        step_limit = (
            profile.max_up_step_m
            if observation.step_height_m >= 0.0
            else profile.max_down_step_m
        )
        if abs(observation.step_height_m) > step_limit:
            return self._blocked('台阶高度超过当前产品安全限值')
        if observation.roughness_m > profile.max_roughness_m:
            return self._blocked('地面起伏超过当前可通行能力')

        mode = self.required_mode(observation)
        if mode not in ('', 'normal') and mode not in modes:
            return TerrainAssessment(
                decision='special_required',
                required_mode=mode,
                cost_multiplier=6.0,
                speed_limit_mps=profile.special_speed_mps,
                reason=f'几何可通行，但缺少已确认的 {mode} 硬件适配器',
            )
        if mode not in ('', 'normal'):
            return TerrainAssessment(
                decision='special_required', required_mode=mode,
                cost_multiplier=3.0,
                speed_limit_mps=profile.special_speed_mps,
                reason=f'需要切换 {mode} 模式并持续进行姿态/进度监控',
            )
        if terrain in CAUTION_TERRAINS or (
                abs(observation.slope_deg) >= 8.0
                or abs(observation.cross_slope_deg) >= 6.0
                or observation.roughness_m >= 0.03):
            return TerrainAssessment(
                decision='caution', required_mode='normal',
                cost_multiplier=1.8,
                speed_limit_mps=profile.caution_speed_mps,
                reason='地形可通行，但需要降速并提高局部感知频率',
            )
        return TerrainAssessment(
            decision='normal', required_mode='normal',
            cost_multiplier=1.0,
            speed_limit_mps=profile.normal_speed_mps,
            reason='处于已验证的普通地面能力包络内',
        )

    @staticmethod
    def required_mode(observation: TerrainObservation) -> str:
        terrain = observation.terrain_type
        if terrain in ('stairs_up', 'stairs_down'):
            return terrain
        if terrain == 'ramp' or abs(observation.slope_deg) >= 8.0:
            return 'ramp'
        if terrain == 'curb' or abs(observation.step_height_m) >= 0.05:
            return 'obstacle'
        if terrain in ('grass', 'gravel', 'rough'):
            return 'rough_ground'
        return 'normal'

    @staticmethod
    def _blocked(reason: str) -> TerrainAssessment:
        return TerrainAssessment(
            decision='blocked', required_mode='',
            cost_multiplier=math.inf, speed_limit_mps=0.0, reason=reason)

    def planar_execution_blockers(
            self, points: Sequence[Mapping[str, object]], *,
            allow_ground_elevation: bool = False) -> List[str]:
        """Return route points a floor-local 2-D executor must not enter."""
        blocked = []
        previous_floor = ''
        for point in points:
            terrain = str(point.get('terrain_type', 'flat')).strip().lower()
            explicit_mode = str(
                point.get('traversal_mode', 'normal')).strip().lower()
            floor_id = str(point.get('floor_id', '')).strip()
            reasons = []
            if terrain not in KNOWN_TERRAINS or terrain in (
                    BLOCKED_TERRAINS | {'unknown'}):
                reasons.append('blocked_terrain')
            if explicit_mode not in ('', 'normal'):
                reasons.append(explicit_mode)
            required_mode = self.required_mode(
                TerrainObservation(terrain_type=terrain))
            if required_mode != 'normal':
                reasons.append(required_mode)
            z = _finite(point.get('z', 0.0), 'route point z')
            if not allow_ground_elevation and abs(z) > 0.05:
                reasons.append('nonzero_z')
            if previous_floor and floor_id and floor_id != previous_floor:
                reasons.append('floor_change')
            if floor_id:
                previous_floor = floor_id
            if reasons:
                name = str(point.get('name', '未命名'))
                blocked.append(f'{name}({"/".join(dict.fromkeys(reasons))})')
        return blocked


@dataclass(frozen=True)
class CampusArea:
    """A map/floor/zone node in the campus topology."""

    area_id: str
    map_name: str
    floor_id: str = ''
    environment: str = 'outdoor'


@dataclass(frozen=True)
class TransitionPose:
    """A calibrated floor-local connector endpoint pose."""

    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> 'TransitionPose':
        if not isinstance(value, Mapping):
            raise ValueError('transition endpoint pose must be an object')
        return cls(
            x=_finite(value.get('x'), 'transition pose x'),
            y=_finite(value.get('y'), 'transition pose y'),
            z=_finite(value.get('z', 0.0), 'transition pose z'),
            yaw=_finite(value.get('yaw', 0.0), 'transition pose yaw'),
        )

    def reversed_heading(self) -> 'TransitionPose':
        """Return this endpoint facing back through the connector."""
        return TransitionPose(
            x=self.x, y=self.y, z=self.z,
            yaw=math.atan2(
                math.sin(self.yaw + math.pi),
                math.cos(self.yaw + math.pi)))


@dataclass(frozen=True)
class CampusTransition:
    """A directional connection between campus areas."""

    transition_id: str
    source_area: str
    target_area: str
    observation: TerrainObservation = field(default_factory=TerrainObservation)
    distance_m: float = 1.0
    bidirectional: bool = True
    source_pose: Optional[TransitionPose] = None
    target_pose: Optional[TransitionPose] = None
    elevation_change_m: Optional[float] = None
    calibrated: bool = False

    @property
    def executable(self) -> bool:
        return (
            self.calibrated and self.source_pose is not None
            and self.target_pose is not None)


@dataclass(frozen=True)
class PlannedSegment:
    """One assessed topological transition in a campus route."""

    transition: CampusTransition
    target: CampusArea
    assessment: TerrainAssessment
    cost: float


@dataclass(frozen=True)
class CampusPlan:
    """A complete area-to-area route and its hardware requirements."""

    segments: Tuple[PlannedSegment, ...]
    total_cost: float
    missing_modes: Tuple[str, ...] = ()

    @property
    def requires_hardware_adapter(self) -> bool:
        return bool(self.missing_modes)


class CampusTopology:
    """Plan across buildings, outdoor zones, floors, stairs and ramps."""

    def __init__(
            self, areas: Sequence[CampusArea],
            transitions: Sequence[CampusTransition],
            capability: Optional[TerrainCapabilityModel] = None):
        self.areas = {area.area_id: area for area in areas}
        self.transitions = tuple(transitions)
        self.capability = capability or TerrainCapabilityModel()
        self._adjacency: Dict[str, List[CampusTransition]] = {
            area_id: [] for area_id in self.areas}
        for transition in self.transitions:
            self._validate_transition(transition)
            self._adjacency[transition.source_area].append(transition)
            if transition.bidirectional:
                reverse = CampusTransition(
                    transition_id=f'{transition.transition_id}:reverse',
                    source_area=transition.target_area,
                    target_area=transition.source_area,
                    observation=self._reverse_observation(transition.observation),
                    distance_m=transition.distance_m,
                    bidirectional=False,
                    source_pose=(
                        transition.target_pose.reversed_heading()
                        if transition.target_pose is not None else None),
                    target_pose=(
                        transition.source_pose.reversed_heading()
                        if transition.source_pose is not None else None),
                    elevation_change_m=(
                        -transition.elevation_change_m
                        if transition.elevation_change_m is not None
                        else None),
                    calibrated=transition.calibrated,
                )
                self._adjacency[reverse.source_area].append(reverse)

    @classmethod
    def from_mapping(
            cls, content: Mapping[str, object],
            capability: Optional[TerrainCapabilityModel] = None,
    ) -> 'CampusTopology':
        if not isinstance(content, Mapping):
            raise ValueError('campus topology must be an object')
        if capability is None:
            capability = TerrainCapabilityModel(
                TerrainCapabilityProfile.from_mapping(
                    content.get('capability_profile')))
        areas = [CampusArea(**item) for item in content.get('areas', [])]
        transitions = []
        for item in content.get('transitions', []):
            item = dict(item)
            observation = TerrainObservation.from_mapping(
                item.pop('terrain', item.pop('observation', {})))
            source_pose_value = item.pop('source_pose', None)
            target_pose_value = item.pop('target_pose', None)
            elevation_change_value = item.pop('elevation_change_m', None)
            transitions.append(CampusTransition(
                observation=observation,
                source_pose=(
                    TransitionPose.from_mapping(source_pose_value)
                    if source_pose_value is not None else None),
                target_pose=(
                    TransitionPose.from_mapping(target_pose_value)
                    if target_pose_value is not None else None),
                elevation_change_m=(
                    _finite(elevation_change_value, 'elevation_change_m')
                    if elevation_change_value is not None else None),
                **item))
        return cls(areas, transitions, capability)

    @classmethod
    def from_json_file(
            cls, path: str,
            capability: Optional[TerrainCapabilityModel] = None,
    ) -> 'CampusTopology':
        content = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
        return cls.from_mapping(content, capability)

    def plan(
            self, source_area: str, target_area: str,
            available_modes: Iterable[str] = (),
            allow_unavailable_modes: bool = False) -> CampusPlan:
        modes = {
            str(mode).strip().lower()
            for mode in available_modes if str(mode).strip()
        }
        if source_area not in self.areas or target_area not in self.areas:
            raise ValueError('source_area and target_area must exist in topology')
        if source_area == target_area:
            return CampusPlan(segments=(), total_cost=0.0)

        distances = {source_area: 0.0}
        previous: Dict[str, Tuple[str, CampusTransition, TerrainAssessment, float]] = {}
        queue: List[Tuple[float, str]] = [(0.0, source_area)]
        while queue:
            current_cost, current_area = heapq.heappop(queue)
            if current_cost != distances.get(current_area):
                continue
            if current_area == target_area:
                break
            for transition in self._adjacency[current_area]:
                assessment = self.capability.assess(
                    transition.observation, modes)
                if not assessment.traversable:
                    continue
                unavailable = (
                    assessment.requires_special_mode
                    and assessment.required_mode not in modes)
                if unavailable and not allow_unavailable_modes:
                    continue
                edge_cost = max(0.01, transition.distance_m) * (
                    assessment.cost_multiplier)
                candidate = current_cost + edge_cost
                if candidate >= distances.get(transition.target_area, math.inf):
                    continue
                distances[transition.target_area] = candidate
                previous[transition.target_area] = (
                    current_area, transition, assessment, edge_cost)
                heapq.heappush(queue, (candidate, transition.target_area))

        if target_area not in distances:
            raise ValueError('No traversable campus route is available')
        segments: List[PlannedSegment] = []
        cursor = target_area
        while cursor != source_area:
            parent, transition, assessment, edge_cost = previous[cursor]
            segments.append(PlannedSegment(
                transition=transition,
                target=self.areas[cursor],
                assessment=assessment,
                cost=edge_cost,
            ))
            cursor = parent
        segments.reverse()
        missing_modes = tuple(sorted({
            segment.assessment.required_mode
            for segment in segments
            if segment.assessment.requires_special_mode
            and segment.assessment.required_mode not in modes
        }))
        return CampusPlan(
            segments=tuple(segments), total_cost=distances[target_area],
            missing_modes=missing_modes)

    def _validate_transition(self, transition: CampusTransition) -> None:
        if transition.source_area not in self.areas:
            raise ValueError(
                f'Unknown source area: {transition.source_area}')
        if transition.target_area not in self.areas:
            raise ValueError(
                f'Unknown target area: {transition.target_area}')
        if not transition.transition_id:
            raise ValueError('transition_id must not be empty')
        if not math.isfinite(transition.distance_m) or transition.distance_m <= 0:
            raise ValueError('transition distance_m must be positive')

    @staticmethod
    def _reverse_observation(value: TerrainObservation) -> TerrainObservation:
        terrain = value.terrain_type
        if terrain == 'stairs_up':
            terrain = 'stairs_down'
        elif terrain == 'stairs_down':
            terrain = 'stairs_up'
        return TerrainObservation(
            terrain_type=terrain,
            slope_deg=-value.slope_deg,
            cross_slope_deg=-value.cross_slope_deg,
            step_height_m=-value.step_height_m,
            gap_width_m=value.gap_width_m,
            roughness_m=value.roughness_m,
            clearance_width_m=value.clearance_width_m,
            confidence=value.confidence,
            negative_obstacle=value.negative_obstacle,
            slippery=value.slippery,
        )
