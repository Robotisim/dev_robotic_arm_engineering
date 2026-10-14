"""Reuse the Panda depth workcell for sorting, stacking, and grasp comparisons."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'), DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('arm_module_04'), 'launch', 'module_04.launch.py')),
                                 launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': LaunchConfiguration('rviz')}.items())])
