"""Persistent web/backend services for the page-driven robot interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Mapping and navigation are intentionally absent here.  Their pages call
    # /control_launch, and launch_manager_service owns their lifecycle.
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='Port for the rosbridge WebSocket server'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'launch_package',
            default_value='car_vel',
            description='Package that owns mapping.launch.py and car_vel.launch.py'),
        DeclareLaunchArgument(
            'mode_switch_delay_sec',
            default_value='1.0',
            description='Wait after stopping the opposite hardware mode'),
        DeclareLaunchArgument(
            'startup_check_sec',
            default_value='0.5',
            description='Check briefly that a requested launch did not exit immediately'),
        DeclareLaunchArgument(
            'shutdown_timeout_sec',
            default_value='8.0',
            description='Graceful shutdown timeout for a managed launch tree'),
        DeclareLaunchArgument(
            'force_kill_timeout_sec',
            default_value='2.0',
            description='Forced-shutdown timeout for a managed launch tree'),
        DeclareLaunchArgument(
            'restart_delay_sec',
            default_value='1.0',
            description='Wait between stop and restart requests'),
        DeclareLaunchArgument(
            'route_db_path',
            default_value='',
            description='Optional shared SQLite path for saved navigation routes'),
        DeclareLaunchArgument(
            'map_server_active_timeout_sec',
            default_value='60.0',
            description='Maximum wait for the Nav2 map server to become active'),
        DeclareLaunchArgument(
            'route_point_retry_count',
            default_value='1',
            description='Automatic retries before a failed route point is skipped'),
        DeclareLaunchArgument(
            'settled_xy_tolerance',
            default_value='0.25',
            description='Stable near-goal fallback position tolerance in metres'),
        DeclareLaunchArgument(
            'settled_yaw_tolerance_deg',
            default_value='20.0',
            description='Stable near-goal fallback heading tolerance in degrees'),
        DeclareLaunchArgument(
            'settled_hold_seconds',
            default_value='2.0',
            description='Time a near-goal pose must remain stable before completion'),
        DeclareLaunchArgument(
            'lio_auto_reset_delay_sec',
            default_value='2.0',
            description='Persistent LIO recovery time before an in-process reset'),
        DeclareLaunchArgument(
            'lio_reset_service_wait_sec',
            default_value='3.0',
            description='Maximum wait for the in-process LIO reset service'),
        DeclareLaunchArgument(
            'lio_reset_settle_before_reset_sec',
            default_value='1.0',
            description='Stationary settling time before resetting LIO'),
        DeclareLaunchArgument(
            'lio_reinitialize_timeout_sec',
            default_value='10.0',
            description='Maximum wait for healthy LIO output after each reset'),
        DeclareLaunchArgument(
            'lio_reset_max_attempts',
            default_value='2',
            description='Maximum in-process estimator reset attempts per incident'),
        DeclareLaunchArgument(
            'auto_relocalization_max_attempts',
            default_value='2',
            description='Maximum automatic 3-D relocalization attempts'),
        DeclareLaunchArgument(
            'auto_relocalization_pose_max_age_sec',
            default_value='3.0',
            description='Maximum age of the pre-fault AMCL recovery checkpoint'),
        DeclareLaunchArgument(
            'auto_relocalization_retry_delay_sec',
            default_value='1.0',
            description='Fresh-cloud wait before automatic relocalization retry'),
        DeclareLaunchArgument(
            'recovery_costmap_clear_timeout_sec',
            default_value='2.0',
            description='Maximum wait for Nav2 costmap clearing after recovery'),

        # Persistent backend: do not add mapping or navigation nodes here.
        Node(
            package='lzp_service',
            executable='launch_manager_service',
            name='launch_manager_service',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'launch_package': LaunchConfiguration('launch_package'),
                'mode_switch_delay_sec': LaunchConfiguration('mode_switch_delay_sec'),
                'startup_check_sec': LaunchConfiguration('startup_check_sec'),
                'shutdown_timeout_sec': LaunchConfiguration('shutdown_timeout_sec'),
                'force_kill_timeout_sec': LaunchConfiguration('force_kill_timeout_sec'),
                'restart_delay_sec': LaunchConfiguration('restart_delay_sec'),
            }]),

        Node(
            package='lzp_service',
            executable='route_navigation_manager',
            name='route_navigation_manager',
            output='screen',
            parameters=[{
                'route_db_path': LaunchConfiguration('route_db_path'),
                'route_point_retry_count': LaunchConfiguration(
                    'route_point_retry_count'),
                'settled_xy_tolerance': LaunchConfiguration(
                    'settled_xy_tolerance'),
                'settled_yaw_tolerance_deg': LaunchConfiguration(
                    'settled_yaw_tolerance_deg'),
                'settled_hold_seconds': LaunchConfiguration(
                    'settled_hold_seconds'),
                'lio_auto_reset_delay_sec': LaunchConfiguration(
                    'lio_auto_reset_delay_sec'),
                'lio_reset_service_wait_sec': LaunchConfiguration(
                    'lio_reset_service_wait_sec'),
                'lio_reset_settle_before_reset_sec': LaunchConfiguration(
                    'lio_reset_settle_before_reset_sec'),
                'lio_reinitialize_timeout_sec': LaunchConfiguration(
                    'lio_reinitialize_timeout_sec'),
                'lio_reset_max_attempts': LaunchConfiguration(
                    'lio_reset_max_attempts'),
                'auto_relocalization_max_attempts': LaunchConfiguration(
                    'auto_relocalization_max_attempts'),
                'auto_relocalization_pose_max_age_sec': LaunchConfiguration(
                    'auto_relocalization_pose_max_age_sec'),
                'auto_relocalization_retry_delay_sec': LaunchConfiguration(
                    'auto_relocalization_retry_delay_sec'),
                'recovery_costmap_clear_timeout_sec': LaunchConfiguration(
                    'recovery_costmap_clear_timeout_sec'),
            }]),
        Node(
            package='lzp_service',
            executable='map_switcher_service',
            name='map_switcher_service',
            output='screen',
            parameters=[{
                'route_db_path': LaunchConfiguration('route_db_path'),
                'map_server_active_timeout_sec': LaunchConfiguration(
                    'map_server_active_timeout_sec'),
            }]),
        Node(
            package='lzp_service',
            executable='map_save_service',
            name='map_save_service',
            output='screen'),
        Node(
            package='car_vel',
            executable='dog_remote_bridge',
            name='dog_remote_bridge',
            output='screen'),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}]),
    ])

