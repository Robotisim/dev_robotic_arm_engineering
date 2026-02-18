from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('panda'), 'launch', 'panda_pick_and_place.launch.py']
                    )
                ),
                launch_arguments={
                    'world_file': PathJoinSubstitution(
                        [FindPackageShare('panda'), 'worlds', 'restricted_pillars_env.sdf']
                    ),
                    'bridge_external_camera': 'false',
                }.items(),
            )
        ]
    )
