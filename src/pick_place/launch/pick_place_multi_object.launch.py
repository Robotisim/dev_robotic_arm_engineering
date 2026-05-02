#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pick_place"), "config", "params.yaml"]
        ),
        description="Path to pick_place params yaml",
    )

    multi_executor_node = Node(
        package="pick_place",
        executable="pick_place_multi_executor_node",
        name="pick_place_multi_executor",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, multi_executor_node])
