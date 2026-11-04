#!/usr/bin/env python3
"""ROS interface for terrain assessment and campus topology planning."""

from __future__ import annotations

import json
import math
from typing import Iterable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from lzp_interface.msg import (
    CampusArea,
    CampusRouteSegment,
    CampusTransition,
    TerrainAssessment,
)
from lzp_interface.srv import (
    EvaluateTerrain,
    GetCampusTopology,
    ManageCampusTopology,
    PlanCampusRoute,
)

from .campus_topology_store import (
    CampusTopologyConflict,
    CampusTopologyStore,
    CampusTopologyValidationError,
)
from .terrain_navigation import (
    CampusTopology,
    TerrainCapabilityModel,
    TerrainCapabilityProfile,
    TerrainObservation,
)


class TerrainNavigationService(Node):
    """Expose terrain policy and optional campus topology through ROS."""

    def __init__(self):
        super().__init__('terrain_navigation_service')
        topology_file = str(self.declare_parameter(
            'campus_topology_file',
            '~/.ros/robot_navigation/campus_topology.json').value).strip()
        parameter_profile = TerrainCapabilityProfile(
            max_up_step_m=float(self.declare_parameter(
                'max_up_step_m', 0.15).value),
            max_down_step_m=float(self.declare_parameter(
                'max_down_step_m', 0.12).value),
            max_up_slope_deg=float(self.declare_parameter(
                'max_up_slope_deg', 20.0).value),
            max_down_slope_deg=float(self.declare_parameter(
                'max_down_slope_deg', 18.0).value),
            max_cross_slope_deg=float(self.declare_parameter(
                'max_cross_slope_deg', 12.0).value),
            max_gap_width_m=float(self.declare_parameter(
                'max_gap_width_m', 0.10).value),
            max_roughness_m=float(self.declare_parameter(
                'max_roughness_m', 0.06).value),
            min_clearance_width_m=float(self.declare_parameter(
                'min_clearance_width_m', 0.65).value),
            min_confidence=float(self.declare_parameter(
                'min_terrain_confidence', 0.70).value),
            normal_speed_mps=float(self.declare_parameter(
                'normal_speed_mps', 0.25).value),
            caution_speed_mps=float(self.declare_parameter(
                'caution_speed_mps', 0.15).value),
            special_speed_mps=float(self.declare_parameter(
                'special_speed_mps', 0.10).value),
        )
        if not topology_file:
            topology_file = '~/.ros/robot_navigation/campus_topology.json'
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._automatic_config_publisher = self.create_publisher(
            String, '/automatic_terrain_navigation/config', latched)
        self._store = CampusTopologyStore(topology_file)
        self._mission_active = False
        self.capability = TerrainCapabilityModel(parameter_profile)
        self.topology = CampusTopology([], [], self.capability)
        self._topology_content = None
        try:
            self._apply_topology(self._store.load())
            self.get_logger().info(
                f'Loaded writable campus topology: {self._store.path}')
        except Exception as error:
            self.get_logger().error(
                f'Campus topology is disabled: {error}')

        self.create_subscription(
            Bool, '/campus_mission/active', self._mission_callback, latched)

        self.create_service(
            EvaluateTerrain, '/evaluate_terrain', self._evaluate)
        self.create_service(
            PlanCampusRoute, '/plan_campus_route', self._plan)
        self.create_service(
            GetCampusTopology, '/get_campus_topology', self._get_topology)
        self.create_service(
            ManageCampusTopology, '/manage_campus_topology',
            self._manage_topology)

    def _mission_callback(self, message):
        self._mission_active = bool(message.data)

    def _apply_topology(self, content):
        profile = TerrainCapabilityProfile.from_mapping(
            content.get('capability_profile'), self.capability.profile)
        capability = TerrainCapabilityModel(profile)
        topology = CampusTopology.from_mapping(content, capability)
        self.capability = capability
        self.topology = topology
        self._topology_content = content
        self._automatic_config_publisher.publish(String(data=json.dumps({
            'automatic_navigation': content.get(
                'automatic_navigation', {
                    'enabled': True,
                    'enabled_modes': [
                        'rough_ground', 'obstacle', 'ramp'],
                }),
            'capability_profile': profile.as_mapping(),
            'revision': int(content.get('revision', 0)),
        }, ensure_ascii=False, separators=(',', ':'))))

    def _manage_topology(self, request, response):
        command = str(request.command).strip().lower()
        try:
            if command == 'get':
                content = self._store.load()
                self._apply_topology(content)
            elif command == 'save':
                if self._mission_active:
                    response.success = False
                    response.message = '任务执行中禁止修改园区拓扑'
                    response.topology_json = ''
                    response.revision = int(
                        (self._topology_content or {}).get('revision', 0))
                    return response
                try:
                    submitted = json.loads(str(request.topology_json))
                except (TypeError, ValueError, json.JSONDecodeError) as error:
                    raise CampusTopologyValidationError(
                        f'topology_json is invalid: {error}') from error
                content = self._store.save(
                    submitted,
                    expected_revision=int(request.expected_revision))
                self._apply_topology(content)
                self.get_logger().info(
                    f'Campus topology revision {content["revision"]} '
                    'was saved from the web interface')
            else:
                raise CampusTopologyValidationError(
                    'command must be get or save')
            response.success = True
            response.message = (
                f'园区拓扑已加载，版本 {content["revision"]}，'
                f'{len(content["areas"])} 个区域，'
                f'{len(content["transitions"])} 条连接')
            response.topology_json = json.dumps(
                content, ensure_ascii=False, separators=(',', ':'))
            response.revision = int(content['revision'])
        except CampusTopologyConflict as error:
            response.success = False
            response.message = str(error)
            latest = self._store.load()
            response.topology_json = json.dumps(
                latest, ensure_ascii=False, separators=(',', ':'))
            response.revision = int(latest['revision'])
        except (CampusTopologyValidationError, OSError) as error:
            response.success = False
            response.message = str(error)
            response.topology_json = ''
            response.revision = int(
                (self._topology_content or {}).get('revision', 0))
        return response

    @staticmethod
    def _observation(message) -> TerrainObservation:
        return TerrainObservation.from_mapping({
            'terrain_type': message.terrain_type,
            'slope_deg': message.slope_deg,
            'cross_slope_deg': message.cross_slope_deg,
            'step_height_m': message.step_height_m,
            'gap_width_m': message.gap_width_m,
            'roughness_m': message.roughness_m,
            'clearance_width_m': message.clearance_width_m,
            'confidence': message.confidence,
            'negative_obstacle': message.negative_obstacle,
            'slippery': message.slippery,
        })

    @staticmethod
    def _assessment_message(value) -> TerrainAssessment:
        message = TerrainAssessment()
        message.decision = value.decision
        message.required_mode = value.required_mode
        message.cost_multiplier = (
            value.cost_multiplier
            if math.isfinite(value.cost_multiplier) else -1.0)
        message.speed_limit_mps = value.speed_limit_mps
        message.reason = value.reason
        return message

    @staticmethod
    def _normalise_modes(values: Iterable[str]):
        return [str(value).strip().lower() for value in values if str(value).strip()]

    @staticmethod
    def _copy_pose(target, value):
        if value is None:
            target.orientation.w = 1.0
            return
        target.position.x = value.x
        target.position.y = value.y
        target.position.z = value.z
        target.orientation.z = math.sin(value.yaw * 0.5)
        target.orientation.w = math.cos(value.yaw * 0.5)

    def _evaluate(self, request, response):
        try:
            assessment = self.capability.assess(
                self._observation(request.observation),
                self._normalise_modes(request.available_modes),
            )
            response.success = assessment.traversable
            response.assessment = self._assessment_message(assessment)
        except Exception as error:
            response.success = False
            response.assessment = self._assessment_message(
                self.capability._blocked(f'地形数据无效：{error}'))
        return response

    def _plan(self, request, response):
        if self.topology is None:
            response.success = False
            response.message = 'Campus topology file is not loaded'
            response.total_cost = 0.0
            response.requires_hardware_adapter = False
            return response
        try:
            plan = self.topology.plan(
                str(request.source_area).strip(),
                str(request.target_area).strip(),
                self._normalise_modes(request.available_modes),
                bool(request.allow_unavailable_modes),
            )
            messages = []
            for segment in plan.segments:
                message = CampusRouteSegment()
                message.transition_id = segment.transition.transition_id
                message.source_area = segment.transition.source_area
                message.target_area = segment.transition.target_area
                message.map_name = segment.target.map_name
                message.floor_id = segment.target.floor_id
                observation = segment.transition.observation
                message.observation.terrain_type = observation.terrain_type
                message.observation.slope_deg = observation.slope_deg
                message.observation.cross_slope_deg = observation.cross_slope_deg
                message.observation.step_height_m = observation.step_height_m
                message.observation.gap_width_m = observation.gap_width_m
                message.observation.roughness_m = observation.roughness_m
                message.observation.clearance_width_m = (
                    observation.clearance_width_m)
                message.observation.confidence = observation.confidence
                message.observation.negative_obstacle = (
                    observation.negative_obstacle)
                message.observation.slippery = observation.slippery
                message.assessment = self._assessment_message(
                    segment.assessment)
                message.cost = segment.cost
                message.executable = segment.transition.executable
                self._copy_pose(
                    message.source_pose, segment.transition.source_pose)
                self._copy_pose(
                    message.target_pose, segment.transition.target_pose)
                message.has_elevation_change = (
                    segment.transition.elevation_change_m is not None)
                message.elevation_change_m = float(
                    segment.transition.elevation_change_m or 0.0)
                messages.append(message)
            response.success = True
            response.message = f'Planned {len(messages)} campus transitions'
            response.segments = messages
            response.total_cost = plan.total_cost
            response.requires_hardware_adapter = (
                plan.requires_hardware_adapter)
        except Exception as error:
            response.success = False
            response.message = str(error)
            response.segments = []
            response.total_cost = 0.0
            response.requires_hardware_adapter = False
        return response

    def _get_topology(self, _request, response):
        if self.topology is None:
            response.success = False
            response.message = 'Campus topology file is not loaded'
            response.areas = []
            response.transitions = []
            return response
        areas = []
        for value in self.topology.areas.values():
            message = CampusArea()
            message.area_id = value.area_id
            message.map_name = value.map_name
            message.floor_id = value.floor_id
            message.environment = value.environment
            areas.append(message)
        transitions = []
        for value in self.topology.transitions:
            message = CampusTransition()
            message.transition_id = value.transition_id
            message.source_area = value.source_area
            message.target_area = value.target_area
            message.distance_m = value.distance_m
            message.bidirectional = value.bidirectional
            message.executable = value.executable
            self._copy_pose(message.source_pose, value.source_pose)
            self._copy_pose(message.target_pose, value.target_pose)
            message.has_elevation_change = value.elevation_change_m is not None
            message.elevation_change_m = float(value.elevation_change_m or 0.0)
            observation = value.observation
            message.observation.terrain_type = observation.terrain_type
            message.observation.slope_deg = observation.slope_deg
            message.observation.cross_slope_deg = observation.cross_slope_deg
            message.observation.step_height_m = observation.step_height_m
            message.observation.gap_width_m = observation.gap_width_m
            message.observation.roughness_m = observation.roughness_m
            message.observation.clearance_width_m = (
                observation.clearance_width_m)
            message.observation.confidence = observation.confidence
            message.observation.negative_obstacle = (
                observation.negative_obstacle)
            message.observation.slippery = observation.slippery
            transitions.append(message)
        response.success = True
        response.message = (
            f'Loaded {len(areas)} areas and {len(transitions)} transitions')
        response.areas = areas
        response.transitions = transitions
        return response


def main(args=None):
    """Run the terrain navigation ROS service."""
    rclpy.init(args=args)
    node = TerrainNavigationService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
