import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_arm_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "roboflex_robotic_arm.urdf.xacro",
    )

    world_file = os.path.join(
        pkg_share,
        "worlds",
        "empty_arm_world.sdf",
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    pkg_share = get_package_share_directory("robot_arm_bringup")

    gz_resource_path = os.pathsep.join(
        [
            os.path.dirname(pkg_share),
            pkg_share,
            os.path.join(pkg_share, "models"),
            os.path.join(pkg_share, "meshes"),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
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

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "roboflex",
            "-string",
            robot_description,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                gz_resource_path,
            ),
            gazebo,
            clock_bridge,
            robot_state_publisher,

            # Wait for Gazebo, then spawn robot.
            TimerAction(
                period=3.0,
                actions=[spawn_robot],
            ),
        ]
    )