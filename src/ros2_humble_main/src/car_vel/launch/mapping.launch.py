import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        }],
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('livox_ros_driver2'),
            '/launch_ROS2/',
            'msg_MID360s_launch.py',
        ])
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('small_point_lio'),
            '/launch/',
            'small_point_lio.launch.py',
        ]),
        # Keep the compact repeated-observation voxel accumulator for PCD save.
        launch_arguments={'enable_localization_map': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file_path],
    )

    # Mapping mode still forwards browser /cmd_vel commands to the dog.
    dog_cmdvel_bridge = Node(
        package='car_vel',
        executable='cmdvel_bridge_ros2',
        name='cmdvel_bridge_ros2',
    )

    delayed_mapping_pipeline = TimerAction(
        period=5.0,
        actions=[
            livox_driver,
            pointlio,
            mapping_cloud_filter,
            octomapping,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        dog_cmdvel_bridge,
        # Let reset/stand-up motion settle before LIO estimates gravity.
        delayed_mapping_pipeline,
    ])
