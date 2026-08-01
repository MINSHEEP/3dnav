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
            "map_switcher_service = lzp_service.map_switcher_service:main",
            "launch_manager_service = lzp_service.launch_manager_service:main",
            "motor_websocket = lzp_service.car.motor_websocket:main",
            "arm_control = lzp_service.arm_control:main",
            "map_save_service = lzp_service.map_save_service:main",
            "map_base_tf = lzp_service.map_base_tf:main",
            "initpost_service = lzp_service.initpost_service:main",
        ],
    },
)
