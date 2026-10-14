"""C++ demonstration recording, native training, or native policy inference."""
import os
import xacro
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'record':
        return [Node(package='arm_module_07', executable='episode_recorder', parameters=[{'use_sim_time': True, 'episode': LaunchConfiguration('episode').perform(context), 'colour': LaunchConfiguration('colour').perform(context)}], output='screen')]
    if mode == 'train':
        return [Node(package='arm_module_07', executable='policy_train', parameters=[{'dataset': LaunchConfiguration('dataset').perform(context), 'output': LaunchConfiguration('checkpoint').perform(context)}], output='screen')]
    if mode != 'run':
        raise ValueError('mode must be record, train, or run')
    model = ET.fromstring(xacro.process_file(os.path.join(share('moveit_resources_panda_description'), 'urdf', 'panda.urdf.xacro')).toxml())
    ET.SubElement(model, 'link', name='panda_hand_tcp')
    joint = ET.SubElement(model, 'joint', name='tcp_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='panda_hand')
    ET.SubElement(joint, 'child', link='panda_hand_tcp')
    ET.SubElement(joint, 'origin', xyz='0 0 0.1034', rpy='0 0 0')
    return [Node(package='arm_module_07', executable='policy_run', parameters=[{'use_sim_time': True, 'robot_description': ET.tostring(model, encoding='unicode'), 'steps': int(LaunchConfiguration('steps').perform(context)), 'checkpoint': LaunchConfiguration('checkpoint').perform(context), 'report': LaunchConfiguration('report').perform(context), 'colour': LaunchConfiguration('colour').perform(context), 'execute': LaunchConfiguration('execute').perform(context) == 'true'}], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='record'), DeclareLaunchArgument('episode', default_value='/tmp/arm_course_dataset/episode_001'),
        DeclareLaunchArgument('dataset', default_value='/tmp/arm_course_dataset'), DeclareLaunchArgument('checkpoint', default_value='/tmp/course_policy.json'),
        DeclareLaunchArgument('report', default_value='/tmp/policy_rollout.csv'),
        DeclareLaunchArgument('steps', default_value='50'), DeclareLaunchArgument('colour', default_value='red'), DeclareLaunchArgument('execute', default_value='false'), OpaqueFunction(function=build)])
