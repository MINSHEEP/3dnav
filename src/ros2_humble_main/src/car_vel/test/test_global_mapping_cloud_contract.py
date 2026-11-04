#!/usr/bin/env python3
"""Static contract for the complete global map and terrain-grid pipeline."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LIO = ROOT / 'lio' / 'small_point_lio'
CAR = ROOT / 'car_vel'
INTERFACE = ROOT / 'nav_service' / 'lzp_interface'
SERVICE = ROOT / 'nav_service' / 'lzp_service'
ICP = ROOT / 'registration' / 'icp_registration'


def test_complete_global_cloud_is_bounded_but_saved_snapshot_is_not():
    source = (LIO / 'src' / 'small_point_lio_node.cpp').read_text('utf-8')
    mapping = (LIO / 'src' / 'util' / 'pointcloud_mapping.cpp').read_text('utf-8')

    assert '"/global_mapping_cloud"' in source
    assert 'get_subscription_count() == 0' in source
    assert 'global_mapping_cloud_max_points' in source
    assert 'pointcloud_mapping->get_snapshot()' in source
    assert 'pointcloud_mapping->get_snapshot(' in source
    assert 'maximum_points' in mapping


def test_lio_install_exports_global_mapping_build_contract():
    cmake = (LIO / 'CMakeLists.txt').read_text('utf-8')
    manager = (
        SERVICE / 'lzp_service' / 'launch_manager_service.py'
    ).read_text('utf-8')

    assert 'global_mapping_cloud_filter_modes_v4' in cmake
    assert 'global_mapping_contract.txt' in cmake
    assert (
        "GLOBAL_MAPPING_CONTRACT = 'global_mapping_cloud_filter_modes_v4'"
        in manager
    )
    assert "'global_mapping_contract.txt'" in manager


def test_mapping_launch_and_frontend_keep_three_layers_distinct():
    launch = (CAR / 'launch' / 'mapping.launch.py').read_text('utf-8')
    html = (CAR / 'car_vel' / 'web_frontend' / 'mapping.html').read_text('utf-8')

    assert "executable='terrain_grid_node'" in launch
    assert "subscribe('/global_mapping_cloud'" in html
    assert "subscribe('/octomap_point_cloud_centers'" in html
    assert "subscribe('/projected_map'" in html
    assert '全局三维地图' in html
    assert '障碍占据图' in html
    assert '二维投影' in html


def test_terrain_grid_interface_exposes_required_layers():
    message = (INTERFACE / 'msg' / 'TerrainGrid.msg').read_text('utf-8')
    node = (SERVICE / 'lzp_service' / 'terrain_grid_node.py').read_text('utf-8')
    for field in (
        'elevation', 'slope_deg', 'roughness_m', 'step_height_m',
        'confidence', 'traversal_cost',
    ):
        assert field in message
    assert "'/terrain_grid'" in node
    assert "'/terrain_grid/status'" in node


def test_navigation_republishes_validated_saved_global_map():
    source = (ICP / 'src' / 'icp_registration.cpp').read_text('utf-8')
    launch = (CAR / 'launch' / 'car_vel.launch.py').read_text('utf-8')
    html = (CAR / 'car_vel' / 'web_frontend' / 'navigation.html').read_text(
        'utf-8')

    assert '"/global_localization_map"' in source
    assert 'get_subscription_count() == 0U' in source
    assert 'loadMap(map_name, loaded_map' in source
    assert "'input_topic': '/global_localization_map'" in launch
    assert "subscribe('/global_localization_map/status'" in html
    assert 'new window.SavedMapCloud' in html
    assert 'fullResolution: true' in html
    assert "subscribe('/global_localization_map'," not in html
    assert '全局三维地图' in html


def test_navigation_cloud_has_map_identity_bound_to_exact_stamp():
    source = (ICP / 'src' / 'icp_registration.cpp').read_text('utf-8')
    cmake = (ICP / 'CMakeLists.txt').read_text('utf-8')
    package = (ICP / 'package.xml').read_text('utf-8')

    assert 'global_map_topic_ + "/status"' in source
    assert 'stampNanoseconds(message.header.stamp)' in source
    assert 'global_map_status_publisher_->publish(status)' in source
    assert 'confirmed_reload' in source
    assert 'active_map_load_id_ != map_load_id' in source
    assert 'message->load_id' in source
    assert 'find_package(std_msgs REQUIRED)' in cmake
    assert '<depend>std_msgs</depend>' in package


def test_saved_metadata_records_ground_preservation_provenance():
    source = (
        SERVICE / 'lzp_service' / 'map_save_service.py'
    ).read_text('utf-8')

    assert "'ground_preservation':" in source
    assert "'protected_voxels_at_save':" in source
    assert "'ground_max_height_above_envelope'" in source
