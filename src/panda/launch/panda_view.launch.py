#!/usr/bin/env -S ros2 launch
"""Visualisation of URDF model for panda in RViz2"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    description_package = LaunchConfiguration('description_package')
    description_filepath = LaunchConfiguration('description_filepath')
    name = LaunchConfiguration('name')
    prefix = LaunchConfiguration('prefix')
    gripper = LaunchConfiguration('gripper')
    collision_arm = LaunchConfiguration('collision_arm')
    collision_gripper = LaunchConfiguration('collision_gripper')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_position_margin = LaunchConfiguration('safety_position_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    safety_k_velocity = LaunchConfiguration('safety_k_velocity')
    ros2_control = LaunchConfiguration('ros2_control')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin')
    ros2_control_command_interface = LaunchConfiguration('ros2_control_command_interface')
    gazebo_preserve_fixed_joint = LaunchConfiguration('gazebo_preserve_fixed_joint')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), description_filepath]),
            ' ',
            'name:=',
            name,
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'gripper:=',
            gripper,
            ' ',
            'collision_arm:=',
            collision_arm,
            ' ',
            'collision_gripper:=',
            collision_gripper,
            ' ',
            'safety_limits:=',
            safety_limits,
            ' ',
            'safety_position_margin:=',
            safety_position_margin,
            ' ',
            'safety_k_position:=',
            safety_k_position,
            ' ',
            'safety_k_velocity:=',
            safety_k_velocity,
            ' ',
            'ros2_control:=',
            ros2_control,
            ' ',
            'ros2_control_plugin:=',
            ros2_control_plugin,
            ' ',
            'ros2_control_command_interface:=',
            ros2_control_command_interface,
            ' ',
            'gazebo_preserve_fixed_joint:=',
            gazebo_preserve_fixed_joint,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='log',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['--display-config', rviz_config, '--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='log',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'description_package',
            default_value='panda',
            description='Custom package with robot description.',
        ),
        DeclareLaunchArgument(
            'description_filepath',
            default_value=path.join('urdf', 'panda.xacro.urdf'),
            description='Path to xacro or URDF description of the robot, relative to share of description_package.',
        ),
        DeclareLaunchArgument('name', default_value='panda', description='Name of the robot.'),
        DeclareLaunchArgument(
            'prefix',
            default_value='panda_',
            description='Prefix for all robot entities. If modified, update controller joint names as well.',
        ),
        DeclareLaunchArgument('gripper', default_value='true', description='Enable default gripper.'),
        DeclareLaunchArgument(
            'collision_arm',
            default_value='true',
            description='Enable collision geometry for manipulator arm.',
        ),
        DeclareLaunchArgument(
            'collision_gripper',
            default_value='true',
            description='Enable collision geometry for manipulator gripper.',
        ),
        DeclareLaunchArgument(
            'safety_limits',
            default_value='true',
            description='Enable safety limits controllers on manipulator joints.',
        ),
        DeclareLaunchArgument(
            'safety_position_margin',
            default_value='0.15',
            description='Lower and upper margin for safety controller position limits.',
        ),
        DeclareLaunchArgument(
            'safety_k_position',
            default_value='100.0',
            description='Parametric k-position factor of safety controllers.',
        ),
        DeclareLaunchArgument(
            'safety_k_velocity',
            default_value='40.0',
            description='Parametric k-velocity factor of safety controllers.',
        ),
        DeclareLaunchArgument(
            'ros2_control',
            default_value='true',
            description='Enable ros2_control for manipulator.',
        ),
        DeclareLaunchArgument(
            'ros2_control_plugin',
            default_value='gz',
            description='ros2_control plugin to load (fake, gz, real, or custom).',
        ),
        DeclareLaunchArgument(
            'ros2_control_command_interface',
            default_value='effort',
            description='Output command interface (position, velocity, effort, or combinations).',
        ),
        DeclareLaunchArgument(
            'gazebo_preserve_fixed_joint',
            default_value='false',
            description='Preserve fixed joints when generating SDF for Gazebo.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=path.join(get_package_share_directory('panda'), 'rviz', 'view.rviz'),
            description='Path to RViz2 config file.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulated clock.'),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS 2 log level for launched nodes.',
        ),
    ]
