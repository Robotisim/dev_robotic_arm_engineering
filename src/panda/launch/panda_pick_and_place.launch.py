import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

import xacro
import yaml


def _load_yaml(file_path):
    with open(file_path, encoding='utf-8') as file:
        return yaml.safe_load(file)


def _resolve_panda_file(*relative_parts):
    panda_share_path = get_package_share_directory('panda')
    installed_path = os.path.join(panda_share_path, *relative_parts)
    if os.path.exists(installed_path):
        return installed_path

    # Fallback for source-tree execution before rebuilding/installing the package.
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', *relative_parts))


def _resolve_panda_config(file_name):
    return _resolve_panda_file('config', file_name)


_MAX_CUBE_COUNT = 5
_CUBE_LAYOUT = [
    {'name': 'cube_red', 'pose': (0.430, 1.740, 1.040), 'rgba': (1.0, 0.0, 0.0, 1.0)},
    {'name': 'cube_green', 'pose': (0.515, 1.800, 1.040), 'rgba': (0.0, 1.0, 0.0, 1.0)},
    {'name': 'cube_blue', 'pose': (0.350, 1.865, 1.040), 'rgba': (0.0, 0.0, 1.0, 1.0)},
    {'name': 'cube_yellow', 'pose': (0.440, 1.900, 1.040), 'rgba': (1.0, 0.9, 0.0, 1.0)},
    {'name': 'cube_magenta', 'pose': (0.260, 1.760, 1.040), 'rgba': (1.0, 0.0, 1.0, 1.0)},
]


def _build_cube_sdf(name, rgba):
    r, g, b, a = rgba

    return f"""
<sdf version='1.9'>
  <model name='{name}'>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>0.12</mass>
        <inertia>
          <ixx>0.00005</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.00005</iyy>
          <iyz>0.0</iyz>
          <izz>0.00005</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          <specular>0.05 0.05 0.05 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


def _spawn_cube_models(context):
    cube_count_value = LaunchConfiguration('cube_count').perform(context).strip()
    try:
        cube_count = int(cube_count_value)
    except ValueError as exc:
        raise RuntimeError(
            "Invalid cube_count "
            f"'{cube_count_value}'. Use an integer in [1, {_MAX_CUBE_COUNT}] (or 0 to disable)."
        ) from exc

    if cube_count == 0:
        return []

    if not 1 <= cube_count <= _MAX_CUBE_COUNT:
        raise RuntimeError(
            f"Invalid cube_count '{cube_count}'. Use an integer in [1, {_MAX_CUBE_COUNT}] (or 0 to disable)."
        )

    cube_spawners = []
    for cube in _CUBE_LAYOUT[:cube_count]:
        cube_x, cube_y, cube_z = cube['pose']
        cube_spawners.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-string',
                    _build_cube_sdf(cube['name'], cube['rgba']),
                    '-name',
                    cube['name'],
                    '-allow_renaming',
                    'true',
                    '-x',
                    f'{cube_x:.3f}',
                    '-y',
                    f'{cube_y:.3f}',
                    '-z',
                    f'{cube_z:.3f}',
                    '-R',
                    '0.0',
                    '-P',
                    '0.0',
                    '-Y',
                    '0.0',
                ],
            )
        )

    return cube_spawners


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    seed_ready_pose = LaunchConfiguration('seed_ready_pose')
    start_moveit_delay_sec = LaunchConfiguration('start_moveit_delay_sec')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    bridge_external_camera = LaunchConfiguration('bridge_external_camera')
    spawn_external_rgbd_camera = LaunchConfiguration('spawn_external_rgbd_camera')
    external_camera_x = LaunchConfiguration('external_camera_x')
    external_camera_y = LaunchConfiguration('external_camera_y')
    external_camera_z = LaunchConfiguration('external_camera_z')
    external_camera_roll = LaunchConfiguration('external_camera_roll')
    external_camera_pitch = LaunchConfiguration('external_camera_pitch')
    external_camera_yaw = LaunchConfiguration('external_camera_yaw')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    panda_share_path = get_package_share_directory('panda')
    xacro_file = _resolve_panda_file('urdf', 'panda.xacro.urdf')
    panda_srdf_file = _resolve_panda_config('panda.srdf.xacro')
    panda_kinematics_file = _resolve_panda_config('kinematics.yaml')
    panda_joint_limits_file = _resolve_panda_config('joint_limits.yaml')

    resource_paths = [
        panda_share_path,
        os.path.join(panda_share_path, 'models'),
    ]
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_resource_path:
        resource_paths.append(existing_gz_resource_path)
    existing_ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_ign_resource_path:
        resource_paths.append(existing_ign_resource_path)

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(dict.fromkeys(resource_paths)),
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join(dict.fromkeys(resource_paths)),
    )

    with open(xacro_file, encoding='utf-8') as file:
        doc = xacro.parse(file)
    xacro.process_doc(doc)

    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            doc.toxml(),
            '-name',
            'Panda',
            '-allow_renaming',
            'true',
            '-x',
            spawn_x,
            '-y',
            spawn_y,
            '-z',
            spawn_z,
            '-R',
            spawn_roll,
            '-P',
            spawn_pitch,
            '-Y',
            spawn_yaw,
        ],
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    load_gripper_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    ready_pose_command = ExecuteProcess(
        cmd=[
            '/bin/bash',
            '-lc',
            (
                "ros2 topic pub --once /joint_trajectory_controller/joint_trajectory "
                "trajectory_msgs/msg/JointTrajectory "
                "\"{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, "
                "panda_joint5, panda_joint6, panda_joint7], "
                "points: [{positions: [0.0, -0.7853981633974483, 0.0, -2.356194490192345, "
                "0.0, 1.5707963267948966, 0.7853981633974483], time_from_start: {sec: 3}}]}\""
            ),
        ],
        output='screen',
        condition=IfCondition(seed_ready_pose),
    )

    wrist_eye_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrist_eye_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wrist_eye/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/wrist_eye/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_eye/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/wrist_eye/camera_info', '/wrist_eye/depth/camera_info'),
            ('/wrist_eye/depth_image', '/wrist_eye/depth/image_raw'),
            ('/wrist_eye/image', '/wrist_eye/image_raw'),
        ],
        output='screen',
    )

    external_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='external_rgbd_bridge',
        arguments=[
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/rgbd_camera/camera_info', '/camera/depth/camera_info'),
            ('/rgbd_camera/depth_image', '/camera/depth/image_raw'),
            ('/rgbd_camera/image', '/camera/image_raw'),
        ],
        output='screen',
        condition=IfCondition(bridge_external_camera),
    )

    external_camera_sdf = """
<sdf version='1.9'>
  <model name='external_rgbd_camera_rig'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.08 0.08 0.08</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.08 0.08 0.08</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1.0</ambient>
          <diffuse>0.1 0.1 0.1 1.0</diffuse>
        </material>
      </visual>
      <sensor name='external_rgbd_camera' type='rgbd_camera'>
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <visualize>true</visualize>
        <topic>external_rgbd_camera</topic>
        <camera>
          <horizontal_fov>1.0472</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
""".strip()

    spawn_external_camera = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            external_camera_sdf,
            '-name',
            'external_rgbd_camera_rig',
            '-allow_renaming',
            'true',
            '-x',
            external_camera_x,
            '-y',
            external_camera_y,
            '-z',
            external_camera_z,
            '-R',
            external_camera_roll,
            '-P',
            external_camera_pitch,
            '-Y',
            external_camera_yaw,
        ],
        condition=IfCondition(spawn_external_rgbd_camera),
    )

    spawned_external_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='spawned_external_rgbd_bridge',
        arguments=[
            '/external_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/external_rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/external_rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/external_rgbd_camera/camera_info', '/external_camera/depth/camera_info'),
            ('/external_rgbd_camera/depth_image', '/external_camera/depth/image_raw'),
            ('/external_rgbd_camera/image', '/external_camera/image_raw'),
        ],
        output='screen',
        condition=IfCondition(spawn_external_rgbd_camera),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                )
            ]
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items(),
    )

    moveit_controller_config = _load_yaml(
        _resolve_panda_config('moveit_controller_manager.yaml')
    )

    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path=panda_srdf_file)
        .robot_description_kinematics(file_path=panda_kinematics_file)
        .joint_limits(file_path=panda_joint_limits_file)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            moveit_config.to_dict(),
            moveit_controller_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_controller_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='If true, use simulated clock.',
            ),
            DeclareLaunchArgument(
                'seed_ready_pose',
                default_value='true',
                description='If true, command Panda to a collision-free ready pose after controllers load.',
            ),
            DeclareLaunchArgument(
                'start_moveit_delay_sec',
                default_value='12.0',
                description='Delay before launching MoveIt so Gazebo controllers are ready and seeded pose is applied.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='If true, start RViz with MoveIt MotionPlanning panel.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('moveit_resources_panda_moveit_config'), 'launch', 'moveit.rviz']
                ),
                description='RViz config for MoveIt.',
            ),
            DeclareLaunchArgument(
                'world_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('panda'), 'worlds', 'simple_pick_and_place_collision.sdf']
                ),
                description='SDF world file for Gazebo.',
            ),
            DeclareLaunchArgument(
                'cube_count',
                default_value='0',
                description='Number of cubes to spawn for pick-and-place (set 1 to 5, or 0 to disable).',
            ),
            DeclareLaunchArgument(
                'bridge_external_camera',
                default_value='true',
                description='If true, bridge external world RGB-D camera topics to ROS.',
            ),
            DeclareLaunchArgument(
                'spawn_external_rgbd_camera',
                default_value='true',
                description='If true, spawn an additional external RGB-D camera from another view.',
            ),
            DeclareLaunchArgument(
                'external_camera_x',
                default_value='0.95',
                description='Spawned external camera x position.',
            ),
            DeclareLaunchArgument(
                'external_camera_y',
                default_value='1.05',
                description='Spawned external camera y position.',
            ),
            DeclareLaunchArgument(
                'external_camera_z',
                default_value='1.75',
                description='Spawned external camera z position.',
            ),
            DeclareLaunchArgument(
                'external_camera_roll',
                default_value='0.0',
                description='Spawned external camera roll.',
            ),
            DeclareLaunchArgument(
                'external_camera_pitch',
                default_value='0.55',
                description='Spawned external camera pitch.',
            ),
            DeclareLaunchArgument(
                'external_camera_yaw',
                default_value='2.70',
                description='Spawned external camera yaw.',
            ),
            DeclareLaunchArgument('spawn_x', default_value='0.0', description='Panda spawn x position.'),
            DeclareLaunchArgument('spawn_y', default_value='1.70', description='Panda spawn y position.'),
            DeclareLaunchArgument('spawn_z', default_value='1.0', description='Panda spawn z position.'),
            DeclareLaunchArgument('spawn_roll', default_value='0.0', description='Panda spawn roll.'),
            DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='Panda spawn pitch.'),
            DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Panda spawn yaw.'),
            set_gz_resource_path,
            set_ign_resource_path,
            gazebo,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[OpaqueFunction(function=_spawn_cube_models), load_joint_state_broadcaster],
                ),
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_trajectory_controller],
                ),
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_trajectory_controller,
                    on_exit=[load_gripper_trajectory_controller],
                ),
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_gripper_trajectory_controller,
                    on_exit=[ready_pose_command],
                ),
            ),
            spawn_entity,
            spawn_external_camera,
            robot_state_publisher,
            wrist_eye_bridge,
            external_camera_bridge,
            spawned_external_camera_bridge,
            TimerAction(
                period=start_moveit_delay_sec,
                actions=[move_group_node, rviz_node],
            ),
        ]
    )
