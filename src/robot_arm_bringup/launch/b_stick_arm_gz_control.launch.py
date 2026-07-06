import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_arm_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "stick_arm.urdf.xacro",
    )

    controllers_file = os.path.join(
        pkg_share,
        "config",
        "stick_arm_controllers.yaml",
    )

    rviz_config = os.path.join(
        pkg_share,
        "config",
        "stick_arm.rviz",
    )

    world_file = os.path.join(
        pkg_share,
        "worlds",
        "empty_arm_world.sdf",
    )

    # Create URDF from Xacro and pass its required parameters.
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            "name": "stick_arm",
            "controllers_file": controllers_file,
        },
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 3 {world_file}",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "stick_arm",
            "-string",
            robot_description,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.15",
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "stick_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            rviz_config,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            clock_bridge,
            robot_state_publisher,

            # Basic delays so Gazebo and controller_manager can start.
            TimerAction(
                period=3.0,
                actions=[spawn_robot],
            ),
            TimerAction(
                period=6.0,
                actions=[joint_state_broadcaster],
            ),
            TimerAction(
                period=8.0,
                actions=[arm_controller],
            ),
            TimerAction(
                period=4.0,
                actions=[rviz],
            ),
        ]
    )