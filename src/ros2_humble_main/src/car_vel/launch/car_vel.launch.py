import os
from car_vel.lidar_network import runtime_config_path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Locate the URDF installed by robot_navigation2.
    package_path = get_package_share_directory('robot_navigation2')
    urdf_file_path = os.path.join(package_path, 'urdf', 'base.xacro')
    nav2_params_file = os.path.join(package_path, 'config', 'nav2_params.yaml')

    # The URDF owns only the fixed base_link -> livox_frame transform.
    # Small Point-LIO publishes odom -> base_footprint -> base_link dynamically.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file_path],
    )

    # Keep the 2-D scan origin at the measured horizontal LiDAR position while
    # excluding body roll/pitch from AMCL and the Nav2 costmaps.  Keep z=0 so
    # LaserScan points remain at the planar Nav2 height; the measured floor
    # offset is handled by pointcloud_to_laserscan's height limits.
    planar_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='planar_laser_tf',
        arguments=[
            '--x', '0.23', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'laser_2d',
        ],
    )

    # The persistent selected body worker consumes /cmd_vel_safe. Starting a
    # second UDP bridge here would bypass body selection and auto-stand Xingji.

    velocity_arbiter = Node(
        package='car_vel',
        executable='velocity_arbiter',
        name='velocity_arbiter',
        parameters=[{
            'command_timeout_sec': 0.3,
            'terrain_guard_timeout_sec': 0.5,
            # Local Xingji 3D navigation and patrol use terrain as telemetry.
            'xingji_3d_terrain_stop_enabled': False,
            # Standard Nav2 Humble publishes the velocity smoother output on
            # /cmd_vel. Isolate the hardware-facing output so the smoother and
            # arbiter can never race each other at the robot bridge.
            'navigation_input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_safe',
        }],
    )

    # Parameter ownership is independent of velocity ownership. Only an idle
    # Xingji 3-D scope may reduce MPPI's candidate count; leaving it restores
    # the actual original value. No command is republished or made fresh here.
    nav2_timing_profile = Node(
        package='car_vel', executable='nav2_timing_profile',
        name='nav2_timing_profile', output='screen',
        respawn=True, respawn_delay=1.0,
        parameters=[{
            'baseline_params_file': nav2_params_file,
            'enabled': ParameterValue(
                LaunchConfiguration('nav2_timing_profile_enabled'), value_type=bool),
            'optimized_batch_size': ParameterValue(
                LaunchConfiguration('nav2_mppi_optimized_batch_size'), value_type=int),
        }],
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('livox_ros_driver2'),
            '/launch_ROS2/',
            'msg_MID360s_launch.py',
        ]),
        launch_arguments={'user_config_path': runtime_config_path()}.items(),
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('small_point_lio'),
            '/launch/',
            'small_point_lio.launch.py',
        ]),
        # Persistent localization-map accumulation is mapping-only.
        launch_arguments={'enable_localization_map': 'false'}.items(),
    )

    pointcloud_to_laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pointcloud_to_laserscan'),
            '/launch/',
            'pointcloud_to_laserscan_launch.py',
        ])
    )

    # Event-triggered NDT/GICP refinement. It returns an action result only;
    # Scoped authority owns map -> odom in confirmed Xingji 3D; legacy scopes retain AMCL.
    relocalization_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('icp_registration'),
            '/launch/',
            'icp.launch.py',
        ])
    )

    relocalization_coordinator = Node(
        package='car_vel',
        executable='relocalization_coordinator',
        name='relocalization_coordinator',
        output='screen',
        parameters=[{
            'search_xy': 3.0,
            'search_yaw_deg': 90.0,
            'relocalize_timeout_sec': 30.0,
            'action_result_grace_sec': 1.5,
            'cancel_confirmation_timeout_sec': 12.0,
            'amcl_confirmation_timeout_sec': 10.0,
        }],
    )

    # Convert the validated saved global localization PCD into the same 2.5-D
    # terrain contract used in mapping mode. This does not replace the live
    # /scan collision layer or claim autonomous stair execution.
    terrain_grid = Node(
        package='lzp_service',
        executable='terrain_grid_node',
        name='terrain_grid_node',
        output='screen',
        parameters=[{
            'input_topic': '/global_localization_map',
            'saved_pcd_directory': '/home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps',
            'resolution': 0.25,
            'maximum_cells': 400000,
            'minimum_update_seconds': 5.0,
            # Same-map Nav2 may plan through terrain inside the conservative
            # bidirectional envelope. Anything above it remains SPECIAL and
            # therefore lethal unless a calibrated connector owns velocity.
            'caution_slope_deg': 12.0,
            'special_slope_deg': 18.0,
            'blocked_slope_deg': 28.0,
            'caution_step_m': 0.08,
            'special_step_m': 0.12,
            'blocked_step_m': 0.18,
            'roughness_caution_m': 0.05,
            'minimum_confidence': 0.35,
        }],
    )

    terrain_costmap = Node(
        package='lzp_service',
        executable='terrain_costmap_node',
        name='terrain_costmap_node',
        output='screen',
        parameters=[{
            'base_map_topic': '/map',
            'terrain_grid_topic': '/terrain_grid',
            'output_topic': '/terrain_costmap',
            'occupied_threshold': 65,
            # SPECIAL means above the automatic 18 deg / 12 cm envelope.
            # It stays lethal; only a calibrated high-risk connector may own
            # velocity there. Safe slopes and low steps remain traversable.
            'special_cost': 100,
        }],
    )

    # Lift Nav2's 2-D /plan onto the matching saved terrain for browser
    # visualization only. The controller continues to consume its original
    # full-resolution plan inside Nav2; this output is never fed back.
    terrain_path = Node(
        package='lzp_service',
        executable='terrain_path_node',
        name='terrain_path_node',
        output='screen',
        parameters=[{
            'clearance_m': 0.08,
            'minimum_spacing_m': 0.10,
            'maximum_points': 4000,
        }],
    )

    terrain_observer = Node(
        package='lzp_service',
        executable='terrain_observer_node',
        name='terrain_observer_node',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered',
            'output_topic': '/terrain_observation',
            # Existing measured MID360 height. Used only by Xingji 3D;
            # verify this after changing the sensor mounting or body height.
            'nominal_sensor_height_m': 0.52,
            'minimum_update_seconds': 0.20,
            'maximum_points': 60000,
            'forward_min': 0.10,
            'forward_max': 1.30,
            'half_width': 0.45,
        }],
    )

    automatic_terrain_navigation = Node(
        package='lzp_service',
        executable='automatic_terrain_navigation_node',
        name='automatic_terrain_navigation_node',
        output='screen',
        parameters=[{
            'enabled': True,
            # Continuous stairs deliberately remain manual/high-risk.
            'enabled_modes': ['rough_ground', 'obstacle', 'ramp'],
            'observation_timeout_sec': 0.75,
            # Xingji 3-D only. Keep unknown/height/hazard checks and all
            # other bodies' confidence policies; true restores the old gate.
            'xingji_3d_confidence_check_enabled': False,
            'xingji_3d_fixed_obstacle_enabled': True,
            'enter_confirmations': 3,
            'exit_confirmations': 5,
        }],
    )

    # Reset/stand-up moves the body during the first few seconds. Start the
    # LiDAR/LIO pipeline only after that motion has settled.
    delayed_perception = TimerAction(
        period=5.0,
        actions=[
            livox_driver,
            pointlio,
            pointcloud_to_laserscan,
            relocalization_server,
            terrain_grid,
            terrain_path,
            terrain_costmap,
            terrain_observer,
        ],
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_navigation2'),
            '/launch/',
            'navigation2.launch.py',
        ]),
        launch_arguments={'params_file': nav2_params_file}.items(),
    )

    delayed_navigation = TimerAction(
        period=12.0,
        actions=[navigation2],
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav2_timing_profile_enabled', default_value='true'),
        DeclareLaunchArgument('nav2_mppi_optimized_batch_size', default_value='400'),
        robot_state_publisher,
        planar_laser_tf,
        delayed_perception,
        velocity_arbiter,
        nav2_timing_profile,
        Node(package='car_vel', executable='global_pose_authority',
             name='global_pose_authority', output='screen', respawn=True, respawn_delay=1.0),
        automatic_terrain_navigation,
        relocalization_coordinator,
        delayed_navigation,
    ])
