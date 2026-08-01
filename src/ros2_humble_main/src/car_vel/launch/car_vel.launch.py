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

    dog_cmdvel_bridge = Node(
        package='car_vel',
        executable='cmdvel_bridge_ros2',
        name='cmdvel_bridge_ros2',
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
    # AMCL remains the sole map -> odom TF publisher.
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
            'search_xy': 1.0,
            'search_yaw_deg': 30.0,
            'relocalize_timeout_sec': 10.0,
            'action_result_grace_sec': 1.5,
            'cancel_confirmation_timeout_sec': 12.0,
            'amcl_confirmation_timeout_sec': 10.0,
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
        ],
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_navigation2'),
            '/launch/',
            'navigation2.launch.py',
        ])
    )

    delayed_navigation = TimerAction(
        period=12.0,
        actions=[navigation2],
    )

    return LaunchDescription([
        robot_state_publisher,
        planar_laser_tf,
        delayed_perception,
        dog_cmdvel_bridge,
        relocalization_coordinator,
        delayed_navigation,
    ])
