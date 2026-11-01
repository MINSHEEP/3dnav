"""Validated, revisioned and atomic persistence for campus topology JSON."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
from typing import Mapping

from .terrain_navigation import (
    CampusArea,
    CampusTopology,
    KNOWN_TERRAINS,
    TerrainCapabilityModel,
    TerrainCapabilityProfile,
    TerrainObservation,
    TransitionPose,
)


class CampusTopologyError(ValueError):
    """Base error exposed by the topology persistence seam."""


class CampusTopologyConflict(CampusTopologyError):
    """The browser attempted to overwrite a newer topology revision."""


class CampusTopologyValidationError(CampusTopologyError):
    """Topology content violates a schema or safety invariant."""


_SAFE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$')
_AUTOMATIC_TERRAIN_MODES = ('rough_ground', 'obstacle', 'ramp')
_PROFILE_RANGES = {
    'max_up_step_m': (0.02, 0.17),
    'max_down_step_m': (0.02, 0.17),
    'max_up_slope_deg': (1.0, 25.0),
    'max_down_slope_deg': (1.0, 22.0),
    'max_cross_slope_deg': (1.0, 15.0),
    'max_gap_width_m': (0.0, 0.12),
    'max_roughness_m': (0.0, 0.08),
    'min_clearance_width_m': (0.50, 2.0),
    'min_confidence': (0.50, 1.0),
    'normal_speed_mps': (0.04, 0.35),
    'caution_speed_mps': (0.04, 0.20),
    'special_speed_mps': (0.04, 0.10),
}


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise CampusTopologyValidationError(
            f'{name} must be numeric') from error
    if not math.isfinite(number):
        raise CampusTopologyValidationError(f'{name} must be finite')
    return number


def _identifier(value, name):
    text = str(value or '').strip()
    if not _SAFE_ID.fullmatch(text):
        raise CampusTopologyValidationError(
            f'{name} must start with a letter or digit and contain only '
            'letters, digits, dot, underscore, colon or hyphen')
    return text


def _pose(value, name):
    if value is None:
        return None
    try:
        pose = TransitionPose.from_mapping(value)
    except (TypeError, ValueError) as error:
        raise CampusTopologyValidationError(f'{name}: {error}') from error
    if abs(pose.yaw) > math.pi + 1e-6:
        raise CampusTopologyValidationError(
            f'{name}: yaw must be in radians between -pi and pi')
    return asdict(pose)


class CampusTopologyStore:
    """Own one writable topology file and reject unsafe browser input."""

    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        self._lock = threading.RLock()

    @staticmethod
    def empty():
        return {
            'version': 1,
            'revision': 0,
            'automatic_navigation': {
                'enabled': True,
                'enabled_modes': list(_AUTOMATIC_TERRAIN_MODES),
                'restricted_flat_site': False,
            },
            'capability_profile': TerrainCapabilityProfile().as_mapping(),
            'areas': [],
            'transitions': [],
        }

    def load(self):
        with self._lock:
            if not self.path.exists():
                value = self.empty()
                self._atomic_write(value)
                return value
            try:
                content = json.loads(
                    self.path.read_text(encoding='utf-8'))
            except (OSError, json.JSONDecodeError) as error:
                raise CampusTopologyValidationError(
                    f'cannot read campus topology: {error}') from error
            revision = int(content.get('revision', 0))
            return self._normalise(content, revision=revision)

    def save(self, content, *, expected_revision):
        with self._lock:
            current = self.load()
            expected = int(expected_revision)
            if expected != int(current['revision']):
                raise CampusTopologyConflict(
                    f'topology revision changed from {expected} to '
                    f'{current["revision"]}; reload before saving')
            value = self._normalise(
                content, revision=int(current['revision']) + 1)
            self._atomic_write(value)
            return value

    def _normalise(self, content, *, revision):
        if not isinstance(content, Mapping):
            raise CampusTopologyValidationError(
                'campus topology must be an object')
        version = int(content.get('version', 1))
        if version != 1:
            raise CampusTopologyValidationError(
                'unsupported topology version')
        profile = TerrainCapabilityProfile.from_mapping(
            content.get('capability_profile'))
        profile_values = profile.as_mapping()
        for name, (minimum, maximum) in _PROFILE_RANGES.items():
            value = profile_values[name]
            if value < minimum or value > maximum:
                raise CampusTopologyValidationError(
                    f'{name} must be between {minimum} and {maximum}')
        if not (
                profile.special_speed_mps <= profile.caution_speed_mps
                <= profile.normal_speed_mps):
            raise CampusTopologyValidationError(
                'special/caution/normal speeds must be nondecreasing')

        automatic = content.get('automatic_navigation', {})
        if not isinstance(automatic, Mapping):
            raise CampusTopologyValidationError(
                'automatic_navigation must be an object')
        enabled = automatic.get('enabled', True)
        restricted = automatic.get('restricted_flat_site', False)
        if not isinstance(restricted, bool):
            raise CampusTopologyValidationError('restricted_flat_site must be boolean')
        if not isinstance(enabled, bool):
            raise CampusTopologyValidationError(
                'automatic_navigation.enabled must be boolean')
        raw_modes = automatic.get(
            'enabled_modes', list(_AUTOMATIC_TERRAIN_MODES))
        if not isinstance(raw_modes, list):
            raise CampusTopologyValidationError(
                'automatic_navigation.enabled_modes must be a list')
        requested_modes = {
            str(mode).strip().lower() for mode in raw_modes
        }
        invalid_modes = requested_modes.difference(_AUTOMATIC_TERRAIN_MODES)
        if invalid_modes:
            raise CampusTopologyValidationError(
                'automatic terrain mode is not allowed: '
                + ', '.join(sorted(invalid_modes)))
        if enabled and not requested_modes:
            raise CampusTopologyValidationError(
                'enabled automatic navigation requires at least one mode')
        automatic_navigation = {
            'enabled': enabled,
            'restricted_flat_site': restricted,
            'enabled_modes': [
                mode for mode in _AUTOMATIC_TERRAIN_MODES
                if mode in requested_modes
            ],
        }

        raw_areas = content.get('areas', [])
        raw_transitions = content.get('transitions', [])
        if not isinstance(raw_areas, list) or len(raw_areas) > 500:
            raise CampusTopologyValidationError(
                'areas must be a list of at most 500 items')
        if (not isinstance(raw_transitions, list)
                or len(raw_transitions) > 2000):
            raise CampusTopologyValidationError(
                'transitions must be a list of at most 2000 items')

        areas = []
        area_ids = set()
        area_maps = {}
        for index, raw in enumerate(raw_areas):
            if not isinstance(raw, Mapping):
                raise CampusTopologyValidationError(
                    f'area {index} must be an object')
            area_id = _identifier(
                raw.get('area_id'), f'area {index} id')
            if area_id in area_ids:
                raise CampusTopologyValidationError(
                    f'duplicate area_id: {area_id}')
            area_ids.add(area_id)
            map_name = _identifier(
                raw.get('map_name'), f'area {area_id} map_name')
            floor_id = str(raw.get('floor_id', '')).strip()[:64]
            environment = str(
                raw.get('environment', 'outdoor')).strip().lower()
            if environment not in {'outdoor', 'indoor', 'mixed'}:
                raise CampusTopologyValidationError(
                    f'area {area_id} environment is invalid')
            areas.append(asdict(CampusArea(
                area_id, map_name, floor_id, environment)))
            area_maps[area_id] = map_name

        transitions = []
        transition_ids = set()
        for index, raw in enumerate(raw_transitions):
            if not isinstance(raw, Mapping):
                raise CampusTopologyValidationError(
                    f'transition {index} must be an object')
            transition_id = _identifier(
                raw.get('transition_id'), f'transition {index} id')
            if transition_id in transition_ids:
                raise CampusTopologyValidationError(
                    f'duplicate transition_id: {transition_id}')
            transition_ids.add(transition_id)
            source = _identifier(
                raw.get('source_area'), f'{transition_id} source_area')
            target = _identifier(
                raw.get('target_area'), f'{transition_id} target_area')
            if (source not in area_ids or target not in area_ids
                    or source == target):
                raise CampusTopologyValidationError(
                    f'{transition_id} must reference two different '
                    'existing areas')
            distance = _finite(
                raw.get('distance_m'), f'{transition_id} distance_m')
            if distance < 0.3 or distance > 1000.0:
                raise CampusTopologyValidationError(
                    f'{transition_id} distance_m must be between '
                    '0.3 and 1000')
            elevation = raw.get('elevation_change_m')
            elevation = (
                _finite(elevation, f'{transition_id} elevation_change_m')
                if elevation is not None else None)
            terrain_value = raw.get(
                'terrain', raw.get('observation', {}))
            if not isinstance(terrain_value, Mapping):
                raise CampusTopologyValidationError(
                    f'{transition_id} terrain must be an object')
            terrain_type = str(terrain_value.get(
                'terrain_type', 'unknown')).strip().lower()
            if terrain_type not in KNOWN_TERRAINS:
                raise CampusTopologyValidationError(
                    f'{transition_id} terrain_type is invalid')
            try:
                observation = TerrainObservation.from_mapping(terrain_value)
            except (TypeError, ValueError) as error:
                raise CampusTopologyValidationError(
                    f'{transition_id} terrain: {error}') from error
            if not 0.0 <= observation.confidence <= 1.0:
                raise CampusTopologyValidationError(
                    f'{transition_id} confidence must be between 0 and 1')
            if min(
                    observation.gap_width_m,
                    observation.roughness_m,
                    observation.clearance_width_m) < 0.0:
                raise CampusTopologyValidationError(
                    f'{transition_id} gap, roughness and clearance '
                    'must be nonnegative')
            source_pose = _pose(
                raw.get('source_pose'), f'{transition_id} source_pose')
            target_pose = _pose(
                raw.get('target_pose'), f'{transition_id} target_pose')
            calibrated = bool(raw.get('calibrated', False))
            if calibrated and (
                    source_pose is None or target_pose is None):
                raise CampusTopologyValidationError(
                    f'{transition_id} calibrated transition requires '
                    'both endpoint poses')
            if (calibrated and area_maps[source] == area_maps[target]
                    and source_pose is not None and target_pose is not None):
                delta_x = target_pose['x'] - source_pose['x']
                delta_y = target_pose['y'] - source_pose['y']
                endpoint_distance = math.hypot(delta_x, delta_y)
                if abs(endpoint_distance - distance) > 0.5:
                    raise CampusTopologyValidationError(
                        f'{transition_id} distance conflicts with '
                        'same-map endpoint poses')
                heading = math.atan2(delta_y, delta_x)
                for pose_name, pose_value in (
                        ('source_pose', source_pose),
                        ('target_pose', target_pose)):
                    heading_error = abs(math.atan2(
                        math.sin(pose_value['yaw'] - heading),
                        math.cos(pose_value['yaw'] - heading)))
                    if heading_error > math.radians(45.0):
                        raise CampusTopologyValidationError(
                            f'{transition_id} {pose_name} must face through '
                            'the connector within 45 degrees')
            assessment = TerrainCapabilityModel(profile).assess(
                observation, {
                    'ramp', 'stairs_up', 'stairs_down',
                    'obstacle', 'rough_ground'})
            if calibrated and (
                    observation.confidence < profile.min_confidence
                    or not assessment.traversable):
                raise CampusTopologyValidationError(
                    f'{transition_id} cannot be calibrated with '
                    'unsafe terrain data')
            transitions.append({
                'transition_id': transition_id,
                'source_area': source,
                'target_area': target,
                'distance_m': distance,
                'elevation_change_m': elevation,
                'calibrated': calibrated,
                'bidirectional': bool(raw.get('bidirectional', True)),
                'source_pose': source_pose,
                'target_pose': target_pose,
                'terrain': asdict(observation),
            })

        value = {
            'version': 1,
            'revision': max(0, int(revision)),
            'automatic_navigation': automatic_navigation,
            'capability_profile': profile_values,
            'areas': areas,
            'transitions': transitions,
        }
        try:
            CampusTopology.from_mapping(value)
        except (TypeError, ValueError) as error:
            raise CampusTopologyValidationError(str(error)) from error
        return value

    def _atomic_write(self, value):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=f'.{self.path.name}.', suffix='.tmp',
            dir=self.path.parent)
        try:
            with os.fdopen(
                    descriptor, 'w', encoding='utf-8', newline='\n') as stream:
                json.dump(
                    value, stream, ensure_ascii=False,
                    indent=2, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        except Exception:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise
