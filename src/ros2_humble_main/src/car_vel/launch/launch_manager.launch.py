"""Persistent web/backend services for the page-driven robot interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_M701_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
DEFAULT_USB_CAMERA_DEVICE = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'


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
        DeclareLaunchArgument('saved_map_port', default_value='8769',
            description='Read-only complete saved PCD HTTP port'),
        DeclareLaunchArgument('saved_map_directory',
            default_value='/home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps',
            description='Original saved PCD directory; must match map_switcher'),
        DeclareLaunchArgument(
            'camera_enabled',
            default_value='true',
            description='Start the fixed USB camera web backend'),
        DeclareLaunchArgument(
            'thermal_enabled',
            default_value='true',
            description='Start the independent HCUSBSDK thermal backend'),
        DeclareLaunchArgument('c12_visible_url',
            default_value='rtsp://192.168.144.108:554/stream=1',
            description='C12 visible-light RTSP source'),
        DeclareLaunchArgument('c12_infrared_url',
            default_value='rtsp://192.168.144.108:555/stream=2',
            description='C12 infrared RTSP source'),
        DeclareLaunchArgument(
            'm701_enabled',
            default_value='true',
            description=(
                'Start the persistent M701 bridge by default for the '
                'configured CH341 serial port')),
        DeclareLaunchArgument(
            'm701_port',
            default_value=DEFAULT_M701_PORT,
            description=(
                'Stable CH340 USB-RS485 path for M701; override this '
                'argument when deploying a different converter')),
        DeclareLaunchArgument(
            'm701_address',
            default_value='1',
            description='M701 DIP-switch address (0-7; address 0 is discouraged)'),
        DeclareLaunchArgument(
            'm701_poll_interval_sec',
            default_value='1.0',
            description='M701 polling interval; the manual requires at least 0.5s'),
        DeclareLaunchArgument(
            'camera_python',
            default_value='/usr/bin/python3',
            description='System Python used by the camera backend'),
        DeclareLaunchArgument(
            'camera_device',
            default_value=DEFAULT_USB_CAMERA_DEVICE,
            description=(
                'Stable V4L2 capture node for the Sonix USB camera; do not '
                'select the HIK thermal UVC nodes')),
        DeclareLaunchArgument(
            'camera_width',
            default_value='1280',
            description='USB camera capture width'),
        DeclareLaunchArgument(
            'camera_height',
            default_value='720',
            description='USB camera capture height'),
        DeclareLaunchArgument(
            'camera_fps',
            default_value='30',
            description='USB camera capture frame rate'),
        DeclareLaunchArgument(
            'camera_web_port',
            default_value='8766',
            description='Browser-facing camera API and MJPEG port'),
        DeclareLaunchArgument(
            'thermal_bundle_dir',
            default_value='/opt/hik_thermal_linux64_bundle',
            description='External vendor bundle containing PythonDemo and Linux64 libraries'),
        DeclareLaunchArgument(
            'thermal_device_index',
            default_value='0',
            description='Thermal device index used when no serial is configured'),
        DeclareLaunchArgument(
            'thermal_serial',
            default_value='',
            description='Optional exact thermal camera serial number'),
        DeclareLaunchArgument(
            'thermal_web_port',
            default_value='8767',
            description='Loopback-only thermal service port'),
        DeclareLaunchArgument(
            'thermal_api_url',
            default_value='http://127.0.0.1:8767',
            description='Loopback thermal API consumed only by the camera gateway'),
        DeclareLaunchArgument(
            'thermal_default_threshold_c',
            default_value='60.0',
            description='Initial global high-temperature alarm threshold in Celsius'),
        DeclareLaunchArgument(
            'yolo_imgsz',
            default_value='640',
            description='Primary smoke/fire/cigarette YOLO input size'),
        DeclareLaunchArgument(
            'yolo_inspection_imgsz',
            default_value='1280',
            description='Primary YOLO input size during checkpoint inspection'),
        DeclareLaunchArgument(
            'yolo_aux_transit_fps',
            default_value='3.0',
            description='Maximum face/plate inference rate while moving'),
        DeclareLaunchArgument(
            'yolo_aux_manual_fps',
            default_value='5.0',
            description='Maximum face/plate inference rate in manual preview'),
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
            'fastdds_shm_cleanup_enabled',
            default_value='true',
            description=(
                'Clean Fast DDS shared-memory zombie files before starting '
                'mapping or navigation')),
        DeclareLaunchArgument(
            'fastdds_shm_cleanup_timeout_sec',
            default_value='3.0',
            description='Maximum wait for Fast DDS shared-memory maintenance'),
        DeclareLaunchArgument(
            'fastdds_force_udp',
            default_value='true',
            description=(
                'Run page-managed mapping/navigation trees with UDPv4 only to '
                'avoid shared-memory port reuse during mode switches')),
        DeclareLaunchArgument(
            'fastdds_udp_fallback_enabled',
            default_value='true',
            description=(
                'Use UDPv4 for a new mode when Fast DDS SHM cleanup fails')),
        DeclareLaunchArgument(
            'expected_workspace',
            default_value='/home/a/3dnav',
            description='Expected source workspace for deployment diagnostics'),
        DeclareLaunchArgument(
            'route_db_path',
            default_value='',
            description='Optional shared SQLite path for saved navigation routes'),
        DeclareLaunchArgument(
            'navigation_mode_config_path',
            default_value='',
            description=(
                'Optional persisted 2D/3D navigation mode JSON path')),
        DeclareLaunchArgument(
            'inflation_radius_config_path',
            default_value='',
            description=(
                'Optional persisted Nav2 obstacle inflation radius JSON path')),
        DeclareLaunchArgument(
            'campus_topology_file',
            default_value='~/.ros/robot_navigation/campus_topology.json',
            description=(
                'Writable JSON topology managed by the campus web page')),
        DeclareLaunchArgument(
            'inspection_db_path',
            default_value='',
            description='Optional SQLite path for inspection tasks and history'),
        DeclareLaunchArgument(
            'inspection_evidence_root',
            default_value='~/.ros/robot_navigation/inspection_data',
            description='Optional root directory for inspection evidence images'),
        DeclareLaunchArgument(
            'environment_alert_config_path',
            default_value='',
            description=(
                'Optional persisted inspection environment alert JSON path')),
        DeclareLaunchArgument(
            'camera_api_url',
            default_value='http://127.0.0.1:8766',
            description='Local USB camera API used by inspection tasks'),
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
            'goal_preflight_pose_max_age_sec',
            default_value='3.0',
            description='Maximum AMCL receive age before a bounded refresh'),
        DeclareLaunchArgument(
            'goal_preflight_tolerance_m',
            default_value='0.10',
            description='Traversability search tolerance around a navigation goal'),
        DeclareLaunchArgument(
            'goal_preflight_start_tolerance_m',
            default_value='0.35',
            description='Maximum snap distance from an unknown robot cell to verified terrain'),
        DeclareLaunchArgument(
            'goal_preflight_wait_timeout_sec',
            default_value='8.0',
            description='Bounded wait for fresh localization or terrain preflight'),
        DeclareLaunchArgument(
            'goal_preflight_nomotion_interval_sec',
            default_value='0.35',
            description='Interval between AMCL no-motion refresh requests'),
        DeclareLaunchArgument(
            'goal_preflight_nomotion_max_requests',
            default_value='8',
            description='Maximum AMCL no-motion requests per preflight wait'),
        DeclareLaunchArgument(
            'recovery_costmap_clear_timeout_sec',
            default_value='2.0',
            description='Maximum wait for Nav2 costmap clearing after recovery'),
        DeclareLaunchArgument(
            'system_power_enabled',
            default_value='true',
            description='Expose fixed reboot and power-off services'),
        DeclareLaunchArgument(
            'system_power_delay_sec',
            default_value='3.0',
            description='Delay before an accepted host power action'),

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
                'fastdds_shm_cleanup_enabled': LaunchConfiguration(
                    'fastdds_shm_cleanup_enabled'),
                'fastdds_shm_cleanup_timeout_sec': LaunchConfiguration(
                    'fastdds_shm_cleanup_timeout_sec'),
                'fastdds_force_udp': LaunchConfiguration(
                    'fastdds_force_udp'),
                'fastdds_udp_fallback_enabled': LaunchConfiguration(
                    'fastdds_udp_fallback_enabled'),
                'expected_workspace': LaunchConfiguration(
                    'expected_workspace'),
            }]),

        Node(
            package='lzp_service',
            executable='route_navigation_manager',
            name='route_navigation_manager',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'route_db_path': LaunchConfiguration('route_db_path'),
                'navigation_mode_config_path': LaunchConfiguration(
                    'navigation_mode_config_path'),
                'inflation_radius_config_path': LaunchConfiguration(
                    'inflation_radius_config_path'),
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
                'goal_preflight_pose_max_age_sec': LaunchConfiguration(
                    'goal_preflight_pose_max_age_sec'),
                'goal_preflight_tolerance_m': LaunchConfiguration(
                    'goal_preflight_tolerance_m'),
                'goal_preflight_start_tolerance_m': LaunchConfiguration(
                    'goal_preflight_start_tolerance_m'),
                'goal_preflight_wait_timeout_sec': LaunchConfiguration(
                    'goal_preflight_wait_timeout_sec'),
                'goal_preflight_nomotion_interval_sec': LaunchConfiguration(
                    'goal_preflight_nomotion_interval_sec'),
                'goal_preflight_nomotion_max_requests': LaunchConfiguration(
                    'goal_preflight_nomotion_max_requests'),
                'recovery_costmap_clear_timeout_sec': LaunchConfiguration(
                    'recovery_costmap_clear_timeout_sec'),
            }]),
        Node(
            package='lzp_service',
            executable='inspection_task_manager',
            name='inspection_task_manager',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'route_db_path': LaunchConfiguration('route_db_path'),
                'inspection_db_path': LaunchConfiguration(
                    'inspection_db_path'),
                'inspection_evidence_root': LaunchConfiguration(
                    'inspection_evidence_root'),
                'environment_alert_config_path': LaunchConfiguration(
                    'environment_alert_config_path'),
                'camera_api_url': LaunchConfiguration('camera_api_url'),
            }]),
        Node(
            package='lzp_service',
            executable='terrain_navigation_service',
            name='terrain_navigation_service',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'campus_topology_file': LaunchConfiguration(
                    'campus_topology_file'),
                # 17 cm is a measured mechanical maximum. Keep a 2 cm
                # product margin until continuous stairs and loaded runs pass.
                'max_up_step_m': 0.15,
                'max_down_step_m': 0.12,
                'max_up_slope_deg': 20.0,
                'max_down_slope_deg': 18.0,
                'max_cross_slope_deg': 12.0,
                'min_terrain_confidence': 0.70,
            }]),
        Node(
            package='lzp_service',
            executable='campus_mission_manager',
            name='campus_mission_manager',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'campus_topology_file': LaunchConfiguration(
                    'campus_topology_file'),
                'stage_timeout_sec': 90.0,
                'connector_lateral_limit_m': 0.25,
                'connector_yaw_limit_deg': 10.0,
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
            package='lzp_service',
            executable='system_power_service',
            name='system_power_service',
            output='screen',
            parameters=[{
                'enabled': LaunchConfiguration('system_power_enabled'),
                'execution_delay_sec': LaunchConfiguration(
                    'system_power_delay_sec'),
            }]),
        Node(package='lzp_service', executable='navigation_startup_monitor',
             name='navigation_startup_monitor', output='screen'),
        # launch_manager_service owns the single selected body worker.
        # Do not launch another dog_remote_bridge alongside it.
        Node(
            condition=IfCondition(LaunchConfiguration('m701_enabled')),
            package='car_vel',
            executable='m701_air_quality_bridge',
            name='m701_air_quality_bridge',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'port': LaunchConfiguration('m701_port'),
                'address': LaunchConfiguration('m701_address'),
                'poll_interval_sec': LaunchConfiguration(
                    'm701_poll_interval_sec'),
                'baudrate': 9600,
                'response_timeout_sec': 0.35,
                # The product notes 2 minutes for TVOC and at least 5 minutes
                # on first use; use the conservative value in the UI.
                'warmup_sec': 300.0,
            }]),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('thermal_enabled')),
            cmd=[
                '/bin/bash',
                PathJoinSubstitution([
                    FindPackageShare('car_vel'),
                    'scripts',
                    'run_thermal_camera.sh',
                ]),
                LaunchConfiguration('camera_python'),
                '-u',
                PathJoinSubstitution([
                    FindPackageShare('car_vel'),
                    'hikvision_camera',
                    'thermal_web.py',
                ]),
                '--bundle-dir', LaunchConfiguration('thermal_bundle_dir'),
                '--device-index', LaunchConfiguration('thermal_device_index'),
                '--serial', LaunchConfiguration('thermal_serial'),
                '--default-threshold-c',
                LaunchConfiguration('thermal_default_threshold_c'),
                '--listen', '127.0.0.1',
                '--port', LaunchConfiguration('thermal_web_port'),
            ],
            name='thermal_camera_web',
            output='screen'),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('camera_enabled')),
            cmd=[
                '/bin/bash',
                PathJoinSubstitution([
                    FindPackageShare('car_vel'),
                    'scripts',
                    'run_usb_camera.sh',
                ]),
                LaunchConfiguration('camera_python'),
                '-u',
                PathJoinSubstitution([
                    FindPackageShare('car_vel'),
                    'hikvision_camera',
                    'camera_web.py',
                ]),
                '--camera-type', 'usb',
                '--video-backend', 'v4l2',
                '--usb-device', LaunchConfiguration('camera_device'),
                '--usb-width', LaunchConfiguration('camera_width'),
                '--usb-height', LaunchConfiguration('camera_height'),
                '--usb-fps', LaunchConfiguration('camera_fps'),
                '--usb-fourcc', 'MJPG',
                '--listen', '0.0.0.0',
                '--web-port', LaunchConfiguration('camera_web_port'),
                '--yolo-device', 'cpu',
                '--yolo-imgsz', LaunchConfiguration('yolo_imgsz'),
                '--yolo-inspection-imgsz',
                LaunchConfiguration('yolo_inspection_imgsz'),
                '--yolo-aux-transit-fps',
                LaunchConfiguration('yolo_aux_transit_fps'),
                '--yolo-aux-manual-fps',
                LaunchConfiguration('yolo_aux_manual_fps'),
                '--yolo-conda-env', 'current',
                '--yolo-capture-backend', 'mjpeg',
                '--yolo-voice-output', 'local',
                '--inspection-evidence-root',
                LaunchConfiguration('inspection_evidence_root'),
                '--thermal-url', LaunchConfiguration('thermal_api_url'),
                '--c12-visible-url', LaunchConfiguration('c12_visible_url'),
                '--c12-infrared-url', LaunchConfiguration('c12_infrared_url'),
            ],
            name='usb_camera_web',
            output='screen'),
        Node(
            package='car_vel', executable='lidar_network_bridge',
            name='lidar_network_bridge', output='screen'),
        Node(
            package='car_vel', executable='web_cloud_preview',
            name='web_cloud_preview', output='screen', parameters=[{
                'saved_map_port': LaunchConfiguration('saved_map_port'),
                'saved_map_directory': LaunchConfiguration('saved_map_directory'),
            }]),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                # /switch_map may legitimately wait up to 60 seconds for the
                # lifecycle map server. Keep rosbridge finite but above that
                # backend deadline; browser calls use shorter per-call limits.
                'default_call_service_timeout': 75.0,
                'call_services_in_new_thread': True,
                'send_action_goals_in_new_thread': True,
            }]),
    ])
