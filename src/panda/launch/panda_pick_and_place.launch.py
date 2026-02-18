import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
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


def _resolve_panda_config(file_name):
    panda_share_path = get_package_share_directory('panda')
    installed_path = os.path.join(panda_share_path, 'config', file_name)
    if os.path.exists(installed_path):
        return installed_path

    # Fallback for source-tree execution before rebuilding/installing the package.
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', file_name))


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    seed_ready_pose = LaunchConfiguration('seed_ready_pose')
    start_moveit_delay_sec = LaunchConfiguration('start_moveit_delay_sec')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    bridge_external_camera = LaunchConfiguration('bridge_external_camera')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    panda_share_path = get_package_share_directory('panda')
    xacro_file = os.path.join(panda_share_path, 'urdf', 'panda.xacro.urdf')

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
        _resolve_panda_config('moveit_controller_manager_arm_only.yaml')
    )

    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path='config/panda.urdf.xacro')
        .robot_description_semantic(file_path='config/panda.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
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
                'bridge_external_camera',
                default_value='true',
                description='If true, bridge external world RGB-D camera topics to ROS.',
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
                    on_exit=[load_joint_state_broadcaster],
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
                    on_exit=[ready_pose_command],
                ),
            ),
            spawn_entity,
            robot_state_publisher,
            wrist_eye_bridge,
            external_camera_bridge,
            TimerAction(
                period=start_moveit_delay_sec,
                actions=[move_group_node, rviz_node],
            ),
        ]
    )
