import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_path = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                robot_navigation_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description=(
                'Optional map yaml path. Leave empty until /switch_map '
                'loads a user-selected map.'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_yaml_path,
                'use_map_topic': 'false',
            }.items(),
        ),
    ])
