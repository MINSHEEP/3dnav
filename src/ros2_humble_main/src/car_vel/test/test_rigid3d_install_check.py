from importlib.util import spec_from_file_location, module_from_spec
from pathlib import Path
from types import SimpleNamespace as NS

from car_vel import rigid_pose
from car_vel import global_pose_authority

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/check_3d_navigation_install.py'
spec = spec_from_file_location('rigid_install_check', SCRIPT)
check = module_from_spec(spec)
spec.loader.exec_module(check)


def test_install_check_rejects_old_interface_old_native_and_other_overlay(tmp_path):
    native = tmp_path / 'share'
    native.mkdir()
    contract = native / 'rigid_transform_contract.txt'
    contract.write_text(rigid_pose.TRANSFORM_MODEL)
    prefix = lambda package: str(tmp_path / 'install' / package)
    share = lambda package: str(native)
    binary = Path(prefix('icp_registration')) / 'lib/libicp_registration.so'
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b'\x7fELF' + rigid_pose.TRANSFORM_MODEL.encode()
                       + b'adaptive_target_sampling_v1')
    runtime = NS(**{name: getattr(rigid_pose, name) for name in (
        'TRANSFORM_MODEL', 'Pose3', 'compose3', 'inverse3')}, __file__=str(tmp_path / 'src/rigid_pose.py'))
    assert check.inspect_install(tmp_path, prefix, share,
        lambda: NS(transform_model=''), runtime) == []
    binary.write_bytes(b'\x7fELF' + rigid_pose.TRANSFORM_MODEL.encode())
    errors = check.inspect_install(tmp_path, prefix, share,
        lambda: NS(transform_model=''), runtime)
    assert any('没有局部地图重采样修复' in error for error in errors)
    binary.write_bytes(b'\x7fELF' + rigid_pose.TRANSFORM_MODEL.encode()
                       + b'adaptive_target_sampling_v1')
    errors = check.inspect_install(tmp_path, prefix, share, NS, runtime)
    assert any('接口未更新' in error for error in errors)
    contract.write_text('old_planar_model')
    errors = check.inspect_install(tmp_path, prefix, share,
        lambda: NS(transform_model=''), runtime)
    assert any('版本不同' in error for error in errors)
    errors = check.inspect_install(tmp_path, lambda _: '/another/install', share,
        lambda: NS(transform_model=''), runtime)
    assert any('其他工作区' in error for error in errors)
    contract.write_text(runtime.TRANSFORM_MODEL)
    binary.write_bytes(b'\x7fELFold_planar')
    errors = check.inspect_install(tmp_path, prefix, share,
        lambda: NS(transform_model=''), runtime)
    assert any('动态库没有新模型' in error for error in errors)


def test_timing_check_rejects_old_lio_binary_or_adapter(tmp_path):
    prefix = lambda package: str(tmp_path/'install'/package)
    binary = Path(prefix('small_point_lio'))/'lib/libsmall_point_lio.so'
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b'ELF-xingji_3d_interval_v1')
    adapter = tmp_path/'src/global_pose_authority_node.py'
    adapter.parent.mkdir()
    adapter.write_text('RUNTIME_TIMING_MODEL', encoding='utf-8')
    policy = NS(RUNTIME_TIMING_MODEL=global_pose_authority.RUNTIME_TIMING_MODEL,
                __file__=str(adapter.with_name('global_pose_authority.py')))
    assert check.inspect_runtime_timing(tmp_path,prefix,policy) == []
    binary.write_bytes(b'ELF-old-lio')
    assert any('未包含间隔补偿' in e for e in check.inspect_runtime_timing(tmp_path,prefix,policy))
    binary.write_bytes(b'ELF-xingji_3d_interval_v1')
    adapter.write_text('old adapter', encoding='utf-8')
    assert any('版本不一致' in e for e in check.inspect_runtime_timing(tmp_path,prefix,policy))
