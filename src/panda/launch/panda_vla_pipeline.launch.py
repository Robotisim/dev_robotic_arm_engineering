from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """VLA sort-and-place environment.

    Table layout (top-down, robot at x=0 y=1.70 facing +x):

        obj_sphere  (red ball)    – (0.35, 1.72)
        obj_puck    (orange disc) – (0.45, 1.65)
        obj_brick   (green block) – (0.30, 1.90)
        obj_cylinder(blue can)   – (0.42, 1.88)

        basket_red   – (0.55, 1.62)   ← place red objects here
        basket_blue  – (0.55, 1.80)   ← place blue objects here
        basket_green – (0.55, 1.98)   ← place green objects here

    External RGB-D camera: x=1.10, y=1.80, z=2.10, pitch=0.70, yaw=π
      → positioned directly in front of the arm+table, elevated and angled
        down so it captures all objects, all baskets, and the Panda end-effector.

    Wrist-eye camera: built into Panda URDF → /wrist_eye/image_raw
                                              /wrist_eye/depth/image_raw
    """
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
                        [FindPackageShare('panda'), 'worlds', 'vla_pipeline_env.sdf']
                    ),
                    # Cube spawner disabled – objects are embedded in the world SDF.
                    'cube_count': '0',
                    # Wrist camera is always bridged by panda_pick_and_place.
                    # External in-world camera (rgbd_camera topic) is not used here.
                    'bridge_external_camera': 'false',
                    # Spawn a dedicated front-view RGB-D camera at run-time so
                    # its pose can be tuned without editing the world SDF.
                    'spawn_external_rgbd_camera': 'true',
                    # Camera sits in front of the robot and table, looking back
                    # at roughly 40° downward (pitch=0.70 rad) to see the full
                    # table surface including objects and baskets.
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
