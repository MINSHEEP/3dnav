#!/usr/bin/env python3
"""Static deployment contract for transactional terrain regeneration."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
INTERFACE = ROOT / 'lzp_interface'
SERVICE = ROOT / 'lzp_service'


def test_rebuild_service_is_generated_and_installed_with_map_backend():
    cmake = (INTERFACE / 'CMakeLists.txt').read_text(encoding='utf-8')
    definition = (INTERFACE / 'srv' / 'RebuildTerrain.srv').read_text(
        encoding='utf-8')
    source = (SERVICE / 'lzp_service' / 'map_save_service.py').read_text(
        encoding='utf-8')

    assert 'srv/RebuildTerrain.srv' in cmake
    assert 'string quality_json' in definition
    assert "'/rebuild_terrain'" in source
    assert "install_order=('terrain', 'metadata')" in source
    assert 'clustered_measured_ground_v3' in source


def test_new_map_save_uses_2d_free_space_support_and_quality_summary():
    source = (SERVICE / 'lzp_service' / 'map_save_service.py').read_text(
        encoding='utf-8')

    assert 'ground_support=ground_support' in source
    assert 'summarize_terrain_quality(' in source
    assert 'Terrain quality warning' in source
