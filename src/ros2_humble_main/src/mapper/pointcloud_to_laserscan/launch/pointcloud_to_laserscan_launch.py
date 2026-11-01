from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0.22', '--y', '0', '--z', '0.11',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'base_link', '--child-frame-id' , 'livox_frame'
        #     ]
        # ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/cloud_registered'),   # 直接订阅真实点云
                ('scan', '/scan')  # 输出 /scanner/scan
            ],
            parameters=[{
                # Generate the polar scan around the robot.  Using odom here
                # makes range/angle relative to the world origin, so front
                # obstacle checks become wrong after the dog moves or turns.
                # laser_2d is a yaw-only child of base_footprint located at the
                # LiDAR's measured x/y mounting position.  This preserves the
                # 0.23 m forward sensor offset without injecting body tilt.
                'target_frame': 'laser_2d',
                'transform_tolerance': 0.2,
                # Collision avoidance values freshness over backlog: if TF or
                # LIO briefly stalls, discard superseded clouds instead of
                # replaying old obstacles after the robot has already moved.
                'queue_size': 1,
                # The MID360 centre is 0.52 m above the floor and its current
                # gravity-levelled TF is about 0.04 m above laser_2d, placing
                # the floor near z=-0.48 m in this frame.  Keep points from
                # 10 cm through 85 cm above the floor, i.e. z=-0.38..0.37 m
                # in laser_2d. Projecting them into a
                # z=0 LaserScan also avoids Nav2's negative-height filtering.
                'min_height': -0.38,
                'max_height': 0.37,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan',
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
