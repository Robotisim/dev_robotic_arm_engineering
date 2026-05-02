from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *_, **__):
    robot = LaunchConfiguration('robot').perform(context)
    if robot != 'panda':
        raise RuntimeError("MoveIt bringup is currently available for robot:='panda'")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('robotic_arm_bringup'), 'launch', 'panda_ik_gazebo.launch.py']
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'rviz': LaunchConfiguration('rviz').perform(context),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot',
                default_value='panda',
                description='Robot model to launch with MoveIt. Currently supported: panda.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Reserved namespace for multi-arm teaching setups.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulated clock.',
            ),
            DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
            DeclareLaunchArgument(
                'control',
                default_value='true',
                description='Reserved flag; Panda MoveIt bringup starts controllers.',
            ),
            DeclareLaunchArgument(
                'moveit',
                default_value='true',
                description='Reserved flag; this launch starts MoveIt.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
