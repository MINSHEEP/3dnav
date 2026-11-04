from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_stop_topic_arg = DeclareLaunchArgument(
        'motion_stop_topic', default_value='/motion_stop')
    # Retain the old argument for older callers, not for native hard stopping.
    emergency_stop_topic_arg = DeclareLaunchArgument(
        'emergency_stop_topic', default_value='/emergency_stop')
    clear_emergency_stop_topic_arg = DeclareLaunchArgument(
        'clear_emergency_stop_topic', default_value='/clear_emergency_stop')
    recoverable_stop_topic_arg = DeclareLaunchArgument(
        'recoverable_stop_topic', default_value='/lio/recoverable_stop')
    recoverable_resume_topic_arg = DeclareLaunchArgument(
        'recoverable_resume_topic', default_value='/lio/recoverable_resume')
    enable_localization_map_arg = DeclareLaunchArgument(
        "enable_localization_map",
        default_value="false",
        description=(
            "Accumulate the filtered localization PCD. Enable only while "
            "mapping; leave disabled during navigation to conserve RAM."
        ),
    )
    localization_pcd_filter_mode_arg = DeclareLaunchArgument(
        "localization_pcd_filter_mode",
        default_value="protected",
        description=(
            "Localization PCD temporal filter mode: protected (default), "
            "bypass (controlled static mapping), or strict."
        ),
    )

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        remappings=[
            ('/motion_stop', LaunchConfiguration('motion_stop_topic')),
            ('/emergency_stop', LaunchConfiguration('emergency_stop_topic')),
            ('/clear_emergency_stop', LaunchConfiguration('clear_emergency_stop_topic')),
            ('/lio/recoverable_stop', LaunchConfiguration('recoverable_stop_topic')),
            ('/lio/recoverable_resume', LaunchConfiguration('recoverable_resume_topic')),
        ],
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
                ),
                "localization_pcd_filter_mode": ParameterValue(
                    LaunchConfiguration("localization_pcd_filter_mode"),
                    value_type=str,
                ),
            },
        ],
    )

    # base_link -> livox_frame is published once by robot_state_publisher from
    # robot_navigation2/urdf/base.xacro.  Do not define a second, conflicting
    # static transform here.
    return LaunchDescription(
        [
            motion_stop_topic_arg,
            emergency_stop_topic_arg,
            clear_emergency_stop_topic_arg,
            recoverable_stop_topic_arg,
            recoverable_resume_topic_arg,
            enable_localization_map_arg,
            localization_pcd_filter_mode_arg,
            small_point_lio_node,
        ]
    )

