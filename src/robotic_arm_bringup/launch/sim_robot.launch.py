from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _as_gazebo_world_name(world):
    if world in ('empty', ''):
        return 'empty.sdf'
    if world.endswith(('.sdf', '.world')):
        return world
    return f'{world}.sdf'


def _include_bringup_launch(launch_file, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robotic_arm_bringup'), 'launch', launch_file])
        ),
        launch_arguments=launch_arguments.items(),
    )


def _launch_setup(context, *_, **__):
    robot = LaunchConfiguration('robot').perform(context)
    world = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    control = LaunchConfiguration('control').perform(context).lower()

    if robot == 'stick_arm':
        return [
            _include_bringup_launch(
                'stick_arm_gz_control.launch.py',
                {'world': _as_gazebo_world_name(world), 'rviz': rviz},
            )
        ]

    if robot == 'panda':
        launch_file = 'panda_sim_control.launch.py' if control == 'true' else 'panda_sim.launch.py'
        return [
            _include_bringup_launch(
                launch_file,
                {'use_sim_time': use_sim_time},
            )
        ]

    raise RuntimeError("robot must be 'stick_arm' or 'panda'")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot',
                default_value='stick_arm',
                description='Robot model to simulate: stick_arm or panda.',
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
            DeclareLaunchArgument('rviz', default_value='true', description='Start RViz when supported.'),
            DeclareLaunchArgument(
                'control',
                default_value='true',
                description='Start ros2_control controllers when supported.',
            ),
            DeclareLaunchArgument(
                'moveit',
                default_value='false',
                description='Reserved flag for launch stacks that add MoveIt.',
            ),
            DeclareLaunchArgument(
                'world',
                default_value='empty',
                description='World: empty, cubes, multi_object, restricted_pillars, restricted_chicane, or vla.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
