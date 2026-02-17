import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    panda_share_path = get_package_share_directory('panda')
    xacro_file = os.path.join(panda_share_path, 'urdf', 'panda.xacro.urdf')
    custom_world = os.path.join(panda_share_path, 'worlds', 'simple_pick_and_place_collision.sdf')
    rviz_config = os.path.join(panda_share_path, 'rviz', 'view_rgbd.rviz')

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
        parameters=[robot_description],
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
            '0.3',
            '-y',
            '0.9',
            '-z',
            '1.2',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_eye/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/wrist_eye/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_eye/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/rgbd_camera/camera_info', '/camera/depth/camera_info'),
            ('/rgbd_camera/depth_image', '/camera/depth/image_raw'),
            ('/rgbd_camera/image', '/camera/image_raw'),
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
        launch_arguments={'gz_args': f'-r -v 4 {custom_world}'}.items(),
    )

    return LaunchDescription(
        [
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
            spawn_entity,
            robot_state_publisher,
            rviz_node,
            bridge_node,
        ]
    )
