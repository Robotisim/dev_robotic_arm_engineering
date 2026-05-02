import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    seed_ready_pose = LaunchConfiguration('seed_ready_pose')

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
        arguments=['-string', doc.toxml(), '-name', 'Panda', '-allow_renaming', 'true'],
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
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
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
                default_value='false',
                description='If true, command Panda to a collision-free ready pose after controllers load.',
            ),
            set_gz_resource_path,
            set_ign_resource_path,
            gazebo,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_trajectory_controller,
                    on_exit=[load_gripper_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_gripper_trajectory_controller,
                    on_exit=[ready_pose_command],
                )
            ),
            robot_state_publisher,
            spawn_entity,
            wrist_eye_bridge,
        ]
    )
