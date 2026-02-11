from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_panda_arg = DeclareLaunchArgument(
        'start_panda',
        default_value='true',
        description='Start Panda simulation bringup from the panda package.',
    )
    start_delay_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='6.0',
        description='Delay before the demo sends the first trajectory.',
    )
    marker_preview_arg = DeclareLaunchArgument(
        'marker_preview_sec',
        default_value='1.0',
        description='Delay between marker publish and trajectory publish.',
    )
    trajectory_offset_arg = DeclareLaunchArgument(
        'trajectory_start_offset_sec',
        default_value='0.8',
        description='JointTrajectory time_from_start offset for the first point.',
    )
    trajectory_topic_arg = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/joint_trajectory_controller/joint_trajectory',
        description='JointTrajectory command topic for Panda controller.',
    )

    panda_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('motion'), 'launch', 'panda_bringup.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('start_panda')),
    )

    return LaunchDescription(
        [
            start_panda_arg,
            start_delay_arg,
            marker_preview_arg,
            trajectory_offset_arg,
            trajectory_topic_arg,
            panda_bringup,
            Node(
                package='motion',
                executable='motion_02_straight_line',
                name='motion_02_straight_line',
                output='screen',
                parameters=[
                    {
                        'start_delay_sec': LaunchConfiguration('start_delay_sec'),
                        'marker_preview_sec': LaunchConfiguration('marker_preview_sec'),
                        'trajectory_start_offset_sec': LaunchConfiguration(
                            'trajectory_start_offset_sec'
                        ),
                        'trajectory_topic': LaunchConfiguration('trajectory_topic'),
                    }
                ],
            )
        ]
    )
