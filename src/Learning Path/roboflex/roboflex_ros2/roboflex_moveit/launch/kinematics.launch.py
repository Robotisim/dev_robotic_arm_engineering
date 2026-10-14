"""Inspect hardware-model kinematics and plan in MoveIt without motor control."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():
    description_share = get_package_share_directory("roboflex_description")
    moveit_share = get_package_share_directory("roboflex_moveit")
    hardware_model = os.path.join(description_share, "urdf", "real_hardware.urdf.xacro")

    # Keep the calibrated-model geometry, but remove every hardware backend.
    # This launch intentionally has no controller manager or command sender.
    robot = ET.fromstring(xacro.process_file(hardware_model).toxml())
    for control in robot.findall("ros2_control"):
        robot.remove(control)
    description = {"robot_description": ET.tostring(robot, encoding="unicode")}

    config = (
        MoveItConfigsBuilder("roboflex_description", package_name="roboflex_moveit")
        .robot_description(file_path=hardware_model)
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    config.robot_description = description

    initial_positions = {
        "zeros.joint_1": 0.0349066,
        "zeros.joint_2": -0.3839724,
        "zeros.joint_3": -0.9250245,
        "zeros.joint_4": -0.3665191,
        "zeros.joint_gripper": 0.0,
    }

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[description, {"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[
                    description,
                    initial_positions,
                    {
                        "use_sim_time": False,
                        "rate": 30,
                        "source_list": ["/visualization/joint_targets"],
                    },
                ],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[
                    config.to_dict(),
                    {"use_sim_time": False, "allow_trajectory_execution": False},
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(moveit_share, "config", "kinematics.rviz")],
                parameters=[
                    config.robot_description,
                    config.robot_description_semantic,
                    config.robot_description_kinematics,
                    config.joint_limits,
                    config.planning_pipelines,
                    {"use_sim_time": False},
                ],
                output="screen",
            ),
        ]
    )
