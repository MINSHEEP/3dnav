from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'car_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'web_frontend'),
            glob('car_vel/web_frontend/*.*')),
        
    ],
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
            'dog_remote_bridge = car_vel.dog_remote_bridge:main',
            'goal_pose_bridge = car_vel.goal_pose_bridge:main',
            'relocalization_coordinator = car_vel.relocalization_coordinator:main',
        ],
    },
)
