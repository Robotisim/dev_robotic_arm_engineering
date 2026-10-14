"""Run one C++ joint/workspace lab against an already running backend."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    xml = xacro.process_file(os.path.join(share('roboflex_description'), 'urdf', 'real_hardware.urdf.xacro')).toxml()
    return [Node(package='arm_module_01', executable='joint_lab', parameters=[{
        'robot_description': xml, 'operation': value('operation'), 'joint': value('joint'),
        'angle': float(value('angle')), 'execute': value('execute') == 'true',
        'use_sim_time': value('use_sim_time') == 'true'}], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default) for name, default in [
        ('operation', 'inspect'), ('joint', 'joint_1'), ('angle', '0.05'),
        ('execute', 'false'), ('use_sim_time', 'false')]] + [OpaqueFunction(function=build)])
