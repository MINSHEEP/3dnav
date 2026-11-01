#!/usr/bin/env python3
"""Static integration contract for the global 3D navigation chain."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SERVICE = ROOT / 'nav_service' / 'lzp_service'
INTERFACE = ROOT / 'nav_service' / 'lzp_interface'
CAR = ROOT / 'car_vel'
NAV = ROOT / 'navigation' / 'robot_navigation2'


def test_nav2_consumes_terrain_costmap_and_keeps_dynamic_scan_layer():
    params = (NAV / 'config' / 'nav2_params.yaml').read_text('utf-8')
    launch = (CAR / 'launch' / 'car_vel.launch.py').read_text('utf-8')
    assert 'map_topic: /terrain_costmap' in params
    assert 'obstacle_layer:' in params
    assert 'topic: /scan' in params
    assert "executable='terrain_grid_node'" in launch
    assert "executable='terrain_costmap_node'" in launch
    assert "'special_cost': 100" in launch
    assert "'saved_pcd_directory': '/home/a/3dnav/" in launch


def test_map_generation_is_carried_through_terrain_grid():
    terrain_message = (
        INTERFACE / 'msg' / 'TerrainGrid.msg').read_text('utf-8')
    grid_node = (
        SERVICE / 'lzp_service' / 'terrain_grid_node.py').read_text('utf-8')
    cost_node = (
        SERVICE / 'lzp_service' / 'terrain_costmap_node.py').read_text('utf-8')
    assert 'string map_name' in terrain_message
    assert 'string map_load_id' in terrain_message
    assert "'/current_map_info'" in grid_node
    assert "'/current_map_info'" in cost_node
    assert 'terrain.map_name' in cost_node
    assert "str(getattr(terrain, 'map_load_id', ''))" in cost_node


def test_versioned_map_bundle_persists_and_validates_terrain_grid():
    save = (
        SERVICE / 'lzp_service' / 'map_save_service.py').read_text('utf-8')
    switch = (
        SERVICE / 'lzp_service' / 'map_switcher_service.py').read_text('utf-8')
    grid_node = (
        SERVICE / 'lzp_service' / 'terrain_grid_node.py').read_text('utf-8')
    assert "f'{map_name}.terrain.npz'" in save
    assert "'terrain': transaction_terrain" in save
    assert "install_order = ('pgm', 'yaml', 'pcd', 'terrain', 'metadata')" in save
    assert "'terrain_grid': {" in save
    assert 'LOCALIZATION_MAP_VERSION = 1' in switch
    assert "if 'terrain_grid' not in metadata:" in switch
    assert 'load_bound_terrain_grid(' in switch
    assert switch.index(
        'self.validate_localization_map(map_name, map_path)') < switch.index(
            'self.load_map_blocking(')
    assert "f'{map_name}.terrain.npz'" in switch
    assert 'self.loaded_maps_cache.pop(map_name, None)' in switch
    assert 'load_bound_terrain_grid' in grid_node
    assert 'falling back to PCD reconstruction' in grid_node
    assert 'self.loaded_artifact_signature' in grid_node
    assert 'signature == self.loaded_artifact_signature' in grid_node


def test_repeated_current_map_announcements_do_not_rebuild_terrain():
    grid_node = (
        SERVICE / 'lzp_service' / 'terrain_grid_node.py').read_text('utf-8')
    costmap_node = (
        SERVICE / 'lzp_service' / 'terrain_costmap_node.py').read_text('utf-8')
    assert "metadata_signature = terrain_artifact_signature(metadata_path)" in grid_node
    assert grid_node.count(
        'if signature == self.loaded_artifact_signature:') == 2
    assert 'self.loaded_artifact_signature = signature' in grid_node
    assert grid_node.index(
        'if signature == self.loaded_artifact_signature:') < grid_node.index(
            'load_pcd_xyz(pcd_path)')
    assert grid_node.rindex(
        'if signature == self.loaded_artifact_signature:') < grid_node.index(
            'load_bound_terrain_grid(')
    assert 'if same_name and load_id == self._current_map_load_id:' in costmap_node
    assert 'self._reload_requires_get_map = True' in costmap_node


def test_nav2_plan_is_elevated_for_read_only_frontend_display():
    node = (
        SERVICE / 'lzp_service' / 'terrain_path_node.py').read_text('utf-8')
    setup = (SERVICE / 'setup.py').read_text('utf-8')
    launch = (CAR / 'launch' / 'car_vel.launch.py').read_text('utf-8')
    assert "Path, '/plan'" in node
    assert "Path, '/navigation/global_path'" in node
    assert "String, '/navigation/global_path/status'" in node
    assert "TerrainGrid, '/terrain_grid'" in node
    assert "'/current_map_info'" in node
    assert "Bool, '/navigation_active'" in node
    assert 'elevate_path_points' in node
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in node
    assert 'DurabilityPolicy.VOLATILE' in node
    assert 'self._pending_plan' in node
    assert 'self._terrain_data_cache' in node
    assert "'fallback_count'" in node
    assert "'source_pose_count'" in node
    assert 'terrain_path_node = lzp_service.terrain_path_node:main' in setup
    assert "executable='terrain_path_node'" in launch


def test_campus_mission_owns_goal_results_and_special_velocity():
    manager = (
        SERVICE / 'lzp_service' / 'campus_mission_manager.py').read_text('utf-8')
    route = (
        SERVICE / 'lzp_service' / 'route_navigation_manager.py').read_text('utf-8')
    status = (
        INTERFACE / 'msg' / 'RouteExecutionStatus.msg').read_text('utf-8')
    assert "'/navigate_managed_goal'" in manager
    assert "'/switch_map'" in manager
    assert "'/relocalization/request'" in manager
    assert "'/cmd_vel_campus'" in manager
    assert 'ConnectorTraversal' in manager
    assert "'/terrain_observation'" in manager
    assert '_live_terrain_ready' in manager
    assert "self._fail(terrain_message, latch_stop=True)" in manager
    assert "Empty, '/motion_stop'" in manager
    assert "Empty, '/emergency_stop'" not in manager
    assert 'last_goal_owner_token' in status
    assert '_record_managed_goal_event' in route


def test_velocity_sources_are_exclusive():
    arbiter = (
        CAR / 'car_vel' / 'velocity_arbiter.py').read_text('utf-8')
    bridge = (
        CAR / 'car_vel' / 'robot_body_bridge.py').read_text('utf-8')
    production_launch = (
        CAR / 'launch' / 'car_vel.launch.py').read_text('utf-8')
    assert "'navigation_input_topic', '/cmd_vel'" in arbiter
    assert "'output_topic', '/cmd_vel_safe'" in arbiter
    assert "'/cmd_vel_campus'" in arbiter
    assert "'/campus_mission/connector_active'" in arbiter
    assert "Twist, '/cmd_vel_safe'" in bridge
    assert "executable='cmdvel_bridge_ros2'" not in production_launch
    assert "'navigation_input_topic': '/cmd_vel'" in production_launch
    assert "'output_topic': '/cmd_vel_safe'" in production_launch


def test_example_connectors_fail_closed_until_calibrated():
    example = (
        SERVICE / 'config' / 'campus_topology.example.json').read_text('utf-8')
    assert '"source_pose"' in example
    assert '"target_pose"' in example
    assert '"elevation_change_m"' in example
    assert '"calibrated": false' in example
