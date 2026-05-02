#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="panda_wrist_eye_sensor",
        description="Non-optical camera frame that depth images originate from",
    )

    camera_header_frame_arg = DeclareLaunchArgument(
        "camera_header_frame",
        default_value="Panda/panda_link7/panda_wrist_eye_sensor",
        description="Frame ID found in incoming image headers",
    )

    publish_header_alias_tf_arg = DeclareLaunchArgument(
        "publish_header_alias_tf",
        default_value="true",
        description="Publish identity TF from camera_header_frame to camera_frame",
    )

    camera_optical_frame_arg = DeclareLaunchArgument(
        "camera_optical_frame",
        default_value="camera_optical_frame",
        description="Optical frame used by segmentation for depth deprojection",
    )

    publish_optical_tf_arg = DeclareLaunchArgument(
        "publish_optical_tf",
        default_value="true",
        description="Publish static TF from non-optical camera frame to optical frame",
    )

    camera_header_alias_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_header_alias_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            LaunchConfiguration("camera_header_frame"),
            LaunchConfiguration("camera_frame"),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("publish_header_alias_tf")),
    )

    # REP-103 optical frame convention: x-right, y-down, z-forward.
    camera_optical_frame_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_frame_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "-0.5",
            "0.5",
            "-0.5",
            "0.5",
            LaunchConfiguration("camera_frame"),
            LaunchConfiguration("camera_optical_frame"),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("publish_optical_tf")),
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
            {"source_frame": LaunchConfiguration("camera_optical_frame")},
        ],
        remappings=[
            # Output topics
            ("~/annotated_image", "/segmentation/annotated_image"),
            ("~/detection_markers", "/segmentation/detection_markers"),
            ("~/object_poses", "/segmentation/object_poses"),
            ("~/detections", "/segmentation/detections"),
            ("~/detected_objects_cloud", "/segmentation/detected_objects_cloud"),
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            config_file_arg,
            use_sim_time_arg,
            camera_frame_arg,
            camera_header_frame_arg,
            publish_header_alias_tf_arg,
            camera_optical_frame_arg,
            publish_optical_tf_arg,
            camera_header_alias_publisher,
            camera_optical_frame_publisher,
            color_segmentation_node,
        ]
    )
