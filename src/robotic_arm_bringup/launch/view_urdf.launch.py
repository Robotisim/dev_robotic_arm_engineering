from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from xacro import process_file


def _launch_setup(context, *_, **__):
    urdf_path = LaunchConfiguration('urdf').perform(context)
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robotic_arm_bringup'), 'rviz', 'demo.rviz']
    )
    if Path(urdf_path).suffix == '.xacro':
        robot_description = process_file(urdf_path).toxml()
    else:
        with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
            robot_description = urdf_file.read()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'urdf',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('robotic_arm_description'), 'urdf', 'demo', 'demo_collision.urdf']
                ),
                description='Absolute path to the URDF or Xacro file to visualize in RViz.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulated clock.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
