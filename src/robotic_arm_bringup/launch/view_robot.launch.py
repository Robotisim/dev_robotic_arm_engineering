from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include_bringup_launch(launch_file, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robotic_arm_bringup'), 'launch', launch_file])
        ),
        launch_arguments=launch_arguments.items(),
    )


def _launch_setup(context, *_, **__):
    robot = LaunchConfiguration('robot').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)

    if robot == 'stick_arm':
        return [
            _include_bringup_launch(
                'stick_arm_rviz.launch.py',
                {'use_sim_time': use_sim_time, 'rviz': rviz},
            )
        ]
    if robot == 'panda':
        return [
            _include_bringup_launch(
                'panda_view.launch.py',
                {'use_sim_time': use_sim_time, 'rviz': rviz},
            )
        ]

    raise RuntimeError("robot must be 'stick_arm' or 'panda'")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot',
                default_value='stick_arm',
                description='Robot model to view: stick_arm or panda.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Reserved namespace for multi-arm teaching setups.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulated clock.',
            ),
            DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
