from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'car_vel'


def install_tree(source_dir, destination_dir):
    """Install a bundled runtime tree while preserving its subdirectories."""
    entries = []
    for root, directories, files in os.walk(source_dir):
        directories[:] = [
            name for name in directories
            if name not in {'__pycache__', '.pytest_cache'}
        ]
        runtime_files = [
            name for name in files
            if os.path.splitext(name)[1].lower() not in {'.pyc', '.pyo'}
        ]
        if not runtime_files:
            continue
        relative = os.path.relpath(root, source_dir)
        destination = destination_dir
        if relative != '.':
            destination = os.path.join(destination_dir, relative)
        entries.append((destination, [
            os.path.join(root, name) for name in runtime_files
        ]))
    return entries


hikvision_runtime_files = install_tree(
    'hikvision_camera',
    os.path.join('share', package_name, 'hikvision_camera'),
)
behavior_runtime_files = install_tree(
    'person-behavior-recognition',
    os.path.join('share', package_name, 'person-behavior-recognition'),
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'HIKVISION_CAMERA.md',
            'USB_CAMERA.md',
            'BEHAVIOR_RECOGNITION.md',
            'behavior_source_manifest.json',
            'WEB_PREVIEW.md',
            'XINGJI_FIXED_OBSTACLE.md',
            'NAVIGATION_MOTION_LOGS.md',
            'STARTUP_RECOVERY.md',
            'MAP_CLOUD_EDIT.md',
            'THERMAL_CAMERA.md',
            'M701_RS485.md',
            'LIDAR_NETWORK.md',
            'ROBOT_BODY_SELECTION.md',
            'GENISOM_SDK_LICENSE.txt',
            'requirements-hikvision-camera.txt',
            'requirements-usb-camera.txt',
            'requirements-behavior-cpu.txt',
        ]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.*')),
        (os.path.join('share', package_name, 'web_frontend'),
            glob('car_vel/web_frontend/*.*')),
    ] + hikvision_runtime_files + behavior_runtime_files,
    install_requires=['setuptools'],
    include_package_data=True,
    package_data={
        'car_vel': ['lib/zsl-1/*/*'],
    },
    zip_safe=True,
    maintainer='zy',
    maintainer_email='zy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_vel = car_vel.cmd_vel_to_move_motor:main',
            'motor_websocket = car_vel.motor_websocket:main',
            'global_localization = car_vel.global_localization:main',
            'testnode = car_vel.testnode:main',
            'publish_initial_pose = car_vel.publish_initial_pose:main',
            'map_body2d = car_vel.map_body2d:main',
            'cmdvel_bridge_ros2 = car_vel.cmdvel_bridge_ros2:main',
            'velocity_arbiter = car_vel.velocity_arbiter:main',
            'global_pose_authority = car_vel.global_pose_authority_node:main',
            'nav2_timing_profile = car_vel.nav2_timing_profile_node:main',
            'web_cloud_preview = car_vel.web_cloud_preview_node:main',
            'dog_remote_bridge = car_vel.dog_remote_bridge:main',
            'goal_pose_bridge = car_vel.goal_pose_bridge:main',
            'relocalization_coordinator = car_vel.relocalization_coordinator:main',
            'm701_air_quality_bridge = car_vel.m701_air_quality_bridge:main',
            'lidar_network_bridge = car_vel.lidar_network_bridge:main',
        ],
    },
)
