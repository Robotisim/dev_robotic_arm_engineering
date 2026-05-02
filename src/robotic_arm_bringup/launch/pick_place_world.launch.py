from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


WORLD_LAUNCH_FILES = {
    'cubes': 'panda_pick_and_place_cubes.launch.py',
    'multi_object': 'panda_multi_object_sequence.launch.py',
    'restricted_pillars': 'panda_restricted_pillars.launch.py',
    'restricted_chicane': 'panda_restricted_chicane.launch.py',
    'vla': 'panda_vla_pipeline.launch.py',
}


def _launch_setup(context, *_, **__):
    world = LaunchConfiguration('world').perform(context)
    if world == 'empty':
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('robotic_arm_bringup'), 'launch', 'panda_pick_and_place.launch.py']
                    )
                ),
                launch_arguments={
                    'world_file': PathJoinSubstitution(
                        [FindPackageShare('robotic_arm_sim'), 'worlds', 'empty.world']
                    ),
                    'cube_count': '0',
                    'bridge_external_camera': 'false',
                }.items(),
            )
        ]

    launch_file = WORLD_LAUNCH_FILES.get(world)
    if launch_file is None:
        raise RuntimeError(
            "world must be empty, cubes, multi_object, restricted_pillars, restricted_chicane, or vla"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('robotic_arm_bringup'), 'launch', launch_file])
            )
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='cubes',
                description='World: empty, cubes, multi_object, restricted_pillars, restricted_chicane, or vla.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Reserved namespace for multi-arm teaching setups.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Reserved clock setting for teaching launch consistency.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Reserved RViz setting for teaching launch consistency.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
