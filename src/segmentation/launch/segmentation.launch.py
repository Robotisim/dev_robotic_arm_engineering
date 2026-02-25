#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import math


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

    # Static transform: Create optical frame from camera link
    # The camera link has: X=down, Y=left, Z=forward (gripper perspective)
    # Optical frame needs: X=right, Y=down, Z=forward (depth forward = down to table)
    #
    # Transform chain:
    # 1. Rotate 90° around Z to swap X and Y: X=left, Y=down, Z=forward
    # 2. Rotate 180° around Y to flip X: X=right, Y=down, Z=forward
    # Combined: quat from (0, 90, 180) = (-0.5, 0.5, 0.5, 0.5)
    #
    # Alternative interpretation (if above doesn't work):
    # Camera X=down → Optical Z
    # Camera Y=left → Optical Y
    # Camera Z=forward → Optical X
    # This is: 90° around Y = (0, 0.7071068, 0, 0.7071068)
    camera_optical_frame_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_frame_publisher",
        arguments=[
            "0",
            "0",
            "0",  # translation (x, y, z)
            # Option 1: 90° around Y (current)
            "0",
            "0.7071068",
            "0",
            "0.7071068",
            # Option 2: Try (-0.5, 0.5, 0.5, 0.5) if positions still wrong
            "panda_wrist_eye_link",
            "panda_wrist_eye_optical_frame",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
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
            LogInfo(msg="Starting Color Segmentation Node with Optical Frame"),
            camera_optical_frame_publisher,
            color_segmentation_node,
        ]
    )
