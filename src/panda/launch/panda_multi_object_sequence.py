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
                        [FindPackageShare('panda'), 'worlds', 'multi_object_sequential_env.sdf']
                    ),
                    'bridge_external_camera': 'false',
                    # External RGB-D camera positioned in front of the robot and table,
                    # elevated and angled down so it sees the full workspace: all three
                    # target objects, the corridor walls/barriers, and the Panda arm.
                    'spawn_external_rgbd_camera': 'true',
                    'external_camera_x': '1.10',
                    'external_camera_y': '1.80',
                    'external_camera_z': '2.10',
                    'external_camera_roll': '0.0',
                    'external_camera_pitch': '0.70',
                    'external_camera_yaw': '3.14159',
                }.items(),
            )
        ]
    )
