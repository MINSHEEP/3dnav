from setuptools import setup

setup(
    name='pphuman_ros', version='0.1.0', packages=['pphuman_ros'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pphuman_ros']),
                ('share/pphuman_ros', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='PP-HumanV2', maintainer_email='maintainer@example.invalid',
    description='Bounded ROS 2 bridge and USB input for PP-HumanV2', license='Apache-2.0',
    entry_points={'console_scripts': [
        'recognition = pphuman_ros.recognition:main',
        'camera = pphuman_ros.camera:main',
        'view = pphuman_ros.view:main',
        'file_publisher = pphuman_ros.file_publisher:main',
    ]},
)
