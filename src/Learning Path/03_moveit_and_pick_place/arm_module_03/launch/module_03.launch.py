"""Real-model MoveIt scripts. Attach to an existing hardware stack or start mock/Gazebo."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build(context):
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in ('mock', 'gazebo', 'hardware'):
        raise ValueError('Module 3 backend must be mock, gazebo, or hardware')
    xml = os.path.join(share('roboflex_description'), 'urdf', 'real_hardware.urdf.xacro')
    cfg = MoveItConfigsBuilder('roboflex_description', package_name='roboflex_moveit').robot_description(file_path=xml).planning_pipelines(pipelines=['ompl']).to_moveit_configs()
    common = {'use_sim_time': backend == 'gazebo'}
    actions = []
    if LaunchConfiguration('start_backend').perform(context) == 'true':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('arm_module_01'), 'launch', 'module_01.launch.py')), launch_arguments={key: LaunchConfiguration(key).perform(context) for key in ['backend', 'firmware_ip', 'rviz', 'headless']}.items()))
    if LaunchConfiguration('start_moveit').perform(context) == 'true':
        actions.append(Node(package='moveit_ros_move_group', executable='move_group', parameters=[cfg.to_dict(), common], output='screen'))
    actions.append(Node(package='arm_module_03', executable='motion_server', parameters=[cfg.to_dict(), common, {'allow_execution': LaunchConfiguration('allow_execution').perform(context) == 'true', 'pose_store': LaunchConfiguration('pose_store').perform(context)}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mock'), DeclareLaunchArgument('start_backend', default_value='true'), DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('allow_execution', default_value='false'), DeclareLaunchArgument('firmware_ip', default_value=''), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('pose_store', default_value='/tmp/roboflex_course_poses.yaml'), OpaqueFunction(function=build)])
