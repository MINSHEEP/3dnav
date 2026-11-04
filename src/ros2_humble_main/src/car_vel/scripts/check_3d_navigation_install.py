#!/usr/bin/env python3
"""Read-only check for a complete rigid-3D deployment; never starts ROS nodes."""
import argparse
import mmap
from pathlib import Path


def inspect_install(workspace, prefix, share, result_type, runtime):
    errors = []
    expected = (Path(workspace) / 'install').resolve()
    for package in ('lzp_interface', 'icp_registration', 'car_vel', 'lzp_service'):
        try:
            resolved = Path(prefix(package)).resolve()
            if not resolved.is_relative_to(expected):
                errors.append(f'{package} 来自其他工作区：{resolved}')
        except Exception as error:
            errors.append(f'找不到 {package}：{error}')
    try:
        if not hasattr(result_type(), 'transform_model'):
            errors.append('Relocalize 接口未更新，缺少 transform_model')
        contract = Path(share('icp_registration')) / 'rigid_transform_contract.txt'
        if contract.read_text(encoding='utf-8').strip() != runtime.TRANSFORM_MODEL:
            errors.append('配准节点与Python三维模型版本不同')
        binary = Path(prefix('icp_registration')) / 'lib/libicp_registration.so'
        with binary.open('rb') as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            if data.find(runtime.TRANSFORM_MODEL.encode()) < 0:
                errors.append('原生配准动态库没有新模型标记，请重新编译icp_registration')
            if data.find(b'adaptive_target_sampling_v1') < 0:
                errors.append('原生配准动态库没有局部地图重采样修复，请重新编译icp_registration')
        if not Path(runtime.__file__).resolve().is_relative_to(Path(workspace).resolve()):
            errors.append(f'Python三维模块来自其他工作区：{runtime.__file__}')
        pose = runtime.Pose3(1., 2., .434135, .02, -.03, .1)
        restored = runtime.compose3(runtime.inverse3(pose), pose)
        if max(abs(getattr(restored, k)) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')) > 1e-8:
            errors.append('三维坐标组合自检失败')
    except Exception as error:
        errors.append(f'三维安装检查失败：{error}')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default='/home/a/3dnav')
    parser.add_argument('--runtime-timing', action='store_true',
                        help='Also verify the native LIO and Python timing stability update')
    args = parser.parse_args()
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        from lzp_interface.action import Relocalize
        from car_vel import rigid_pose
        errors = inspect_install(args.workspace, get_package_prefix,
                                 get_package_share_directory, Relocalize.Result, rigid_pose)
        if args.runtime_timing:
            from car_vel import global_pose_authority
            errors.extend(inspect_runtime_timing(args.workspace, get_package_prefix,
                                                 global_pose_authority))
    except Exception as error:
        errors = [f'请先加载ROS和当前工作区环境：{error}']
    for error in errors:
        print('[未通过] ' + error)
    if errors:
        extra = ' small_point_lio' if args.runtime_timing else ''
        print('请停止旧后台，重建 lzp_interface icp_registration car_vel lzp_service'
              + extra + '，再加载 install/setup.bash。')
        return 1
    print('[通过] 四个包来自当前工作区，接口/原生动态库/Python三维模型一致；几何自检通过。')
    if args.runtime_timing:
        print('[通过] SmallPointLIO 原生间隔补偿与Python运行时稳定性版本一致。')
    print('这是安装检查。启动后的 REGISTRATION_CONTRACT 日志与 transform_model 状态还需确认，不能替代现场验收。')
    return 0


def inspect_runtime_timing(workspace, prefix, policy):
    """Read installed files only; never creates a ROS node or changes a mode."""
    errors = []
    try:
        installed = Path(prefix('small_point_lio')).resolve()
        if not installed.is_relative_to((Path(workspace)/'install').resolve()):
            errors.append(f'small_point_lio 来自其他工作区：{installed}')
        binary = installed/'lib/libsmall_point_lio.so'
        with binary.open('rb') as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            if data.find(b'xingji_3d_interval_v1') < 0:
                errors.append('SmallPointLIO原生动态库未包含间隔补偿，请重新编译small_point_lio')
        if getattr(policy, 'RUNTIME_TIMING_MODEL', '') != 'xingji_3d_runtime_timing_v1':
            errors.append('Python定位运行时未更新')
        adapter = Path(policy.__file__).resolve().with_name('global_pose_authority_node.py')
        if (not adapter.is_relative_to(Path(workspace).resolve())
                or 'RUNTIME_TIMING_MODEL' not in adapter.read_text(encoding='utf-8')):
            errors.append('定位节点与运行时版本不一致')
    except Exception as error:
        errors.append(f'运行时稳定性安装检查失败：{error}')
    return errors


if __name__ == '__main__':
    raise SystemExit(main())
