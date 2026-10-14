"""Standalone C++ position IK with the canonical RoboFlex model; no MoveIt."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    backend = LaunchConfiguration('backend').perform(context)
    xml = xacro.process_file(os.path.join(share('roboflex_description'), 'urdf', 'real_hardware.urdf.xacro')).toxml()
    actions = []
    if LaunchConfiguration('start_backend').perform(context) == 'true':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('arm_module_01'), 'launch', 'module_01.launch.py')), launch_arguments={key: LaunchConfiguration(key).perform(context) for key in ['backend', 'firmware_ip', 'rviz', 'headless']}.items()))
    actions.append(Node(package='arm_module_02', executable='kinematics_node', parameters=[{'robot_description': xml, 'use_sim_time': backend == 'gazebo', 'dh_report': LaunchConfiguration('dh_report').perform(context) == 'true'}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend', default_value='mock'), DeclareLaunchArgument('start_backend', default_value='true'), DeclareLaunchArgument('firmware_ip', default_value=''), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('headless', default_value='false'), DeclareLaunchArgument('dh_report', default_value='false'), OpaqueFunction(function=build)])
