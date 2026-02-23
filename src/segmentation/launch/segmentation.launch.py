#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare("cube_segmentation")

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "segmentation_params.yaml"]
        ),
        description="Path to the configuration file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    # Color segmentation node
    color_segmentation_node = Node(
        package="cube_segmentation",
        executable="color_segmentation_node",
        name="color_segmentation_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            # Input topics
            ("/wrist_eye/image_raw", "/wrist_eye/image_raw"),
            # Output topics
            ("~/annotated_image", "/segmentation/annotated_image"),
            ("~/detection_markers", "/segmentation/detection_markers"),
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            config_file_arg,
            use_sim_time_arg,
            LogInfo(msg="Starting Color Segmentation Node (Phase 1)"),
            color_segmentation_node,
        ]
    )
