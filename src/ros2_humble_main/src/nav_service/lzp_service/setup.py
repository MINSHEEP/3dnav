from glob import glob

from setuptools import find_packages, setup

package_name = 'lzp_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "waypoint_service = lzp_service.waypoint_service:main",
            "route_navigation_manager = lzp_service.route_navigation_manager:main",
            "inspection_task_manager = lzp_service.inspection_task_manager:main",
            "terrain_navigation_service = lzp_service.terrain_navigation_service:main",
            "campus_mission_manager = lzp_service.campus_mission_manager:main",
            "terrain_grid_node = lzp_service.terrain_grid_node:main",
            "terrain_path_node = lzp_service.terrain_path_node:main",
            "terrain_costmap_node = lzp_service.terrain_costmap_node:main",
            "terrain_observer_node = lzp_service.terrain_observer_node:main",
            "automatic_terrain_navigation_node = lzp_service.automatic_terrain_navigation_node:main",
            "map_switcher_service = lzp_service.map_switcher_service:main",
            "launch_manager_service = lzp_service.launch_manager_service:main",
            "navigation_startup_monitor = lzp_service.navigation_startup:main",
            "motor_websocket = lzp_service.car.motor_websocket:main",
            "arm_control = lzp_service.arm_control:main",
            "map_save_service = lzp_service.map_save_service:main",
            "system_power_service = lzp_service.system_power_service:main",
            "map_base_tf = lzp_service.map_base_tf:main",
            "initpost_service = lzp_service.initpost_service:main",
        ],
    },
)
