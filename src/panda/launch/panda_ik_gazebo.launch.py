import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def _load_yaml(file_path):
    with open(file_path, encoding='utf-8') as file:
        return yaml.safe_load(file)


def _resolve_panda_config(file_name):
    panda_share_path = get_package_share_directory('panda')
    installed_path = os.path.join(panda_share_path, 'config', file_name)
    if os.path.exists(installed_path):
        return installed_path

    # Fallback for source-tree execution before rebuilding/installing the package.
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', file_name))


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_moveit_delay_sec = LaunchConfiguration('start_moveit_delay_sec')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    moveit_controller_config = _load_yaml(
        _resolve_panda_config('moveit_controller_manager.yaml')
    )

    panda_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('panda'), 'launch', 'panda_sim_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'seed_ready_pose': 'true',
        }.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path='config/panda.urdf.xacro')
        .robot_description_semantic(file_path='config/panda.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            moveit_config.to_dict(),
            moveit_controller_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_controller_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='If true, use simulated clock.',
            ),
            DeclareLaunchArgument(
                'start_moveit_delay_sec',
                default_value='10.0',
                description='Delay before launching MoveIt so Gazebo controllers are ready and seeded pose is applied.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='If true, start RViz with MoveIt MotionPlanning panel.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('moveit_resources_panda_moveit_config'), 'launch', 'moveit.rviz']
                ),
                description='RViz config for MoveIt.',
            ),
            panda_sim_control,
            TimerAction(
                period=start_moveit_delay_sec,
                actions=[move_group_node, rviz_node],
            ),
        ]
    )
