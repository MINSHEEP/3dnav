from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_localization_map_arg = DeclareLaunchArgument(
        "enable_localization_map",
        default_value="false",
        description=(
            "Accumulate the filtered localization PCD. Enable only while "
            "mapping; leave disabled during navigation to conserve RAM."
        ),
    )

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "config",
                    "mid360.yaml",
                ]
            ),
            {
                "enable_localization_map": ParameterValue(
                    LaunchConfiguration("enable_localization_map"),
                    value_type=bool,
                )
            },
        ],
    )

    # base_link -> livox_frame is published once by robot_state_publisher from
    # robot_navigation2/urdf/base.xacro.  Do not define a second, conflicting
    # static transform here.
    return LaunchDescription(
        [
            enable_localization_map_arg,
            small_point_lio_node,
        ]
    )

