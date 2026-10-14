"""Depth perception begins the simulation-only half of the course."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('panda_learning_sim'), 'launch', 'workcell.launch.py')),
                                 launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': LaunchConfiguration('rviz')}.items(), condition=IfCondition(LaunchConfiguration('start_sim'))),
        Node(package='arm_module_04', executable='depth_objects', parameters=[{'use_sim_time': True}], output='screen')])
