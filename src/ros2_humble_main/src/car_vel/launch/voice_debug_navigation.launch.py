"""2-D navigation for the voice debug process, which owns the robot UDP.

Do not include launch_manager.launch.py here: it owns another body worker.
This launch runs in the debug process's separate ROS domain.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from car_vel.lidar_network import runtime_config_path


def generate_launch_description():
    debug_mode = str(Path.home() / '.ros/robot_navigation/voice_debug_mode.json')
    navigation_share = get_package_share_directory('robot_navigation2')
    # Reuse the deployed Nav2 tuning, changing only the static map source for
    # this private launch. The shared YAML and other pages stay untouched.
    params_2d = RewrittenYaml(
        source_file=str(Path(navigation_share) / 'config/nav2_params.yaml'),
        param_rewrites={'map_topic': '/map'}, convert_types=True)
    perception = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py'])),
            launch_arguments={'user_config_path': runtime_config_path()}.items()),
        # LIO supplies odometry and the cloud projected into /scan; it does
        # not introduce a terrain map or a 3-D goal preview into this launch.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('small_point_lio'), 'launch', 'small_point_lio.launch.py'])),
            launch_arguments={'enable_localization_map': 'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('pointcloud_to_laserscan'), 'launch',
            'pointcloud_to_laserscan_launch.py']))),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('bridge_port', default_value='9091'),
        # Reuse the saved route database; do not copy or edit luxian1.
        Node(package='lzp_service', executable='route_navigation_manager',
             name='route_navigation_manager', output='screen', parameters=[{
                 'navigation_mode_config_path': debug_mode,
             }]),
        # The debug adapter calls Nav2 LoadMap directly and publishes the
        # confirmed map identity. A 2-D map needs no PCD/terrain bundle.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             arguments=[str(Path(navigation_share) / 'urdf/base.xacro')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='voice_debug_planar_laser_tf', arguments=[
                 '--x', '0.23', '--y', '0', '--z', '0',
                 '--roll', '0', '--pitch', '0', '--yaw', '0',
                 '--frame-id', 'base_footprint', '--child-frame-id', 'laser_2d']),
        Node(package='car_vel', executable='velocity_arbiter', name='velocity_arbiter',
             parameters=[{'command_timeout_sec': 0.3,
                          'navigation_input_topic': '/cmd_vel', 'output_topic': '/cmd_vel_safe'}]),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='voice_debug_rosbridge', output='screen', parameters=[{
                 'address': '127.0.0.1', 'port': LaunchConfiguration('bridge_port'),
                 'max_message_size': 134217728,
                 'default_call_service_timeout': 75.0,
                 'call_services_in_new_thread': True,
             }]),
        perception,
        TimerAction(period=12.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_navigation2'), 'launch', 'navigation2.launch.py'])),
            launch_arguments={'params_file': params_2d}.items())]),
    ])
