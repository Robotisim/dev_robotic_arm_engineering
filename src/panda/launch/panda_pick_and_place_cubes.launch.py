from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cube_count = LaunchConfiguration('cube_count')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cube_count',
                default_value='3',
                description='Number of cubes to spawn (valid values: 1 to 5).',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('panda'), 'launch', 'panda_pick_and_place.launch.py']
                    )
                ),
                launch_arguments={
                    'world_file': PathJoinSubstitution(
                        [FindPackageShare('panda'), 'worlds', 'pick_and_place_cubes_base.sdf']
                    ),
                    'cube_count': cube_count,
                    'bridge_external_camera': 'false',
                }.items(),
            )
        ]
    )
