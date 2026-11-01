import os
from car_vel.mapping_lio_stop import mapping_lio_topics
from car_vel.lidar_network import runtime_config_path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lio_stop_topic, lio_resume_topic = mapping_lio_topics()
    dynamic_filter_mode_arg = DeclareLaunchArgument(
        'dynamic_filter_mode',
        default_value='protected',
        description=(
            'Unified mapping filter mode: protected (recommended), bypass '
            '(controlled static-scene mapping only), or strict.'
        ),
    )
    dynamic_filter_mode = LaunchConfiguration('dynamic_filter_mode')

    # Locate the URDF installed by robot_navigation2.
    package_path = get_package_share_directory('robot_navigation2')
    urdf_file_path = os.path.join(package_path, 'urdf', 'base.xacro')

    # OctoMap continuously projects the registered mapping cloud into a 2-D map.
    octomapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('octomap_server2'),
            '/launch/',
            'octomap_server_launch.py',
        ]),
        # OctoMap receives only the mapping-specific, dynamically
        # ground-relative obstacle cloud. Navigation continues to use the
        # untouched /cloud_registered topic.
        launch_arguments={
            'input_cloud_topic': '/cloud_mapping_filtered',
            # The pre-filter normalizes its detected floor to z=0. OctoMap
            # uses those floor returns only for free-space ray clearing.
            'filter_ground': 'True',
            'ground_filter/plane_distance': '0.08',
            'pointcloud_min_height': '-0.08',
            'pointcloud_max_height': '0.85',
        }.items(),
    )

    mapping_cloud_filter = Node(
        package='octomap_server2',
        executable='mapping_cloud_filter',
        name='mapping_cloud_filter',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered',
            'output_topic': '/cloud_mapping_filtered',
            'dynamic_output_topic': '/cloud_mapping_dynamic',
            'world_frame': 'odom',
            'sensor_frame': 'livox_frame',
            # Measured with the dog standing normally.
            'nominal_sensor_height': 0.52,
            'min_obstacle_height': 0.10,
            'max_obstacle_height': 0.85,
            'ground_clearance_thickness': 0.06,
            'floor_search_window': 0.35,
            'floor_bin_size': 0.02,
            'min_floor_points': 80,
            'min_floor_peak_fraction': 0.04,
            'floor_smoothing_alpha': 0.70,
            # Reject isolated vertical jumps. A persistent new floor level is
            # accepted automatically after five consistent scans.
            'max_floor_jump': 0.10,
            'max_floor_rate': 0.45,
            'recovery_confirmation_frames': 5,
            # A voxel must remain spatially stable over several distinct
            # frames before it may enter OctoMap/projected_map. New or moving
            # voxels remain available on /cloud_mapping_dynamic while the
            # untouched /cloud_registered continues feeding local avoidance.
            'temporal_voxel_size': 0.15,
            'static_confirmation_observations': 8,
            'static_confirmation_seconds': 1.5,
            # protected is the safe default. Use bypass only while mapping a
            # controlled static scene; strict disables the ground fast path.
            'dynamic_filter_mode': ParameterValue(
                dynamic_filter_mode,
                value_type=str,
            ),
            # Reliable floor points still require multiple frames, but they
            # no longer wait for the slower obstacle/static-object threshold.
            'ground_preservation_enabled': True,
            'ground_confirmation_observations': 3,
            'ground_confirmation_seconds': 0.2,
            'max_observation_gap_frames': 3,
            'candidate_stale_frames': 30,
            'confirmed_stale_frames': 1800,
            'temporal_prune_interval_frames': 30,
            'max_tracked_voxels': 500000,
        }],
    )

    terrain_grid = Node(
        package='lzp_service',
        executable='terrain_grid_node',
        name='terrain_grid_node',
        output='screen',
        parameters=[{
            'input_topic': '/global_mapping_cloud',
            'resolution': 0.25,
            'maximum_cells': 400000,
            'minimum_update_seconds': 5.0,
            # Keep mapping preview and navigation interpretation identical.
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
        # Keep the compact repeated-observation voxel accumulator for PCD save.
        launch_arguments={
            'enable_localization_map': 'true',
            'localization_pcd_filter_mode': dynamic_filter_mode,
            # Recoverable events keep mapping's release-to-rearm behavior.
            # Confirmed faults use the global latched zero-speed stop channel;
            # only transient recovery events are remapped for mapping teleop.
            'recoverable_stop_topic': lio_stop_topic,
            'recoverable_resume_topic': lio_resume_topic,
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file_path],
    )

    # Mapping teleop stays behind the fail-safe velocity arbiter; it never
    # publishes directly to the hardware-facing /cmd_vel topic.
    # The persistent selected body worker also owns mapping teleop output.

    velocity_arbiter = Node(
        package='car_vel',
        executable='velocity_arbiter',
        name='velocity_arbiter',
        parameters=[{'command_timeout_sec': 0.3}],
    )

    delayed_mapping_pipeline = TimerAction(
        period=5.0,
        actions=[
            livox_driver,
            pointlio,
            mapping_cloud_filter,
            octomapping,
            terrain_grid,
        ],
    )

    return LaunchDescription([
        dynamic_filter_mode_arg,
        robot_state_publisher,
        velocity_arbiter,
        # Let reset/stand-up motion settle before LIO estimates gravity.
        delayed_mapping_pipeline,
    ])
