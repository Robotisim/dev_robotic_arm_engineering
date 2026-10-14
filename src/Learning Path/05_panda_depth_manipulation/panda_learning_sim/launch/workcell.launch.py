"""Panda physics workcell. No RoboFlex transport or hardware process is started."""
import os
import tempfile
import uuid
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build(context):
    root = share('panda_learning_sim')
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    allow = LaunchConfiguration('allow_execution').perform(context) == 'true'
    partition = LaunchConfiguration('partition').perform(context)
    if partition == 'auto':
        partition = 'arm_course_' + uuid.uuid4().hex
    model = ET.fromstring(xacro.process_file(os.path.join(share('moveit_resources_panda_description'), 'urdf', 'panda.urdf.xacro')).toxml())
    ET.SubElement(model, 'link', name='world')
    joint = ET.SubElement(model, 'joint', name='world_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='world')
    ET.SubElement(joint, 'child', link='panda_link0')
    ET.SubElement(model, 'link', name='panda_hand_tcp')
    joint = ET.SubElement(model, 'joint', name='tcp_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='panda_hand')
    ET.SubElement(joint, 'child', link='panda_hand_tcp')
    ET.SubElement(joint, 'origin', xyz='0 0 0.1034', rpy='0 0 0')
    control = ET.SubElement(model, 'ros2_control', name='PandaGazebo', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    initial = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    # Keep the fingers off the exact DART joint stops; velocity-actuated
    # position control can stick at an exact hard-stop configuration.
    for name, value in [(f'panda_joint{i + 1}', q) for i, q in enumerate(initial)] + [('panda_finger_joint1', 0.038), ('panda_finger_joint2', 0.038)]:
        j = ET.SubElement(control, 'joint', name=name)
        if name != 'panda_finger_joint2':
            ET.SubElement(j, 'command_interface', name='position')
        position = ET.SubElement(j, 'state_interface', name='position')
        ET.SubElement(position, 'param', name='initial_value').text = str(value)
        ET.SubElement(j, 'state_interface', name='velocity')
    gazebo = ET.SubElement(model, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='gz_ros2_control-system', name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = os.path.join(root, 'config', 'controllers.yaml')
    for finger in ['panda_leftfinger', 'panda_rightfinger']:
        g = ET.SubElement(model, 'gazebo', reference=finger)
        ET.SubElement(g, 'mu1').text = '2.0'
        ET.SubElement(g, 'mu2').text = '2.0'
    xml = ET.tostring(model, encoding='unicode')
    fd, urdf_path = tempfile.mkstemp(prefix='panda_course_', suffix='.urdf')
    with os.fdopen(fd, 'w') as stream:
        stream.write(xml)
    cfg = (MoveItConfigsBuilder('panda', package_name='moveit_resources_panda_moveit_config')
           .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    cfg.robot_description = {'robot_description': xml}
    srdf = ET.fromstring(cfg.robot_description_semantic['robot_description_semantic'])
    for v in srdf.findall('virtual_joint'):
        srdf.remove(v)
    cfg.robot_description_semantic = {'robot_description_semantic': ET.tostring(srdf, encoding='unicode')}
    cfg.trajectory_execution['moveit_simple_controller_manager'] = {
        'controller_names': ['panda_arm_controller'],
        'panda_arm_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory', 'default': True, 'joints': [f'panda_joint{i}' for i in range(1, 8)]}}
    common = {'use_sim_time': True}
    actions = [
        SetEnvironmentVariable('GZ_PARTITION', partition),
        LogInfo(msg='Gazebo transport partition: ' + partition),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                                 launch_arguments={'gz_args': ('-r -s --headless-rendering ' if headless else '-r ') + os.path.join(root, 'worlds', 'course.sdf')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[cfg.robot_description, common]),
        Node(package='ros_gz_sim', executable='create', arguments=['-world', 'course', '-name', 'panda', '-file', urdf_path], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': os.path.join(root, 'config', 'bridge.yaml')}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/world/course/set_pose@ros_gz_interfaces/srv/SetEntityPose'], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x', '0.48', '--y', '0', '--z', '1.15', '--pitch', '1.57079632679', '--frame-id', 'world', '--child-frame-id', 'camera_link'], parameters=[common]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--roll', '-1.57079632679', '--yaw', '-1.57079632679', '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame'], parameters=[common]),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[cfg.to_dict(), common], output='screen'),
        Node(package='panda_learning_sim', executable='scene_manager', parameters=[common], output='screen'),
        Node(package='arm_module_03', executable='motion_server', parameters=[cfg.to_dict(), common, {
            'robot': 'panda', 'arm_group': 'panda_arm', 'base_frame': 'panda_link0', 'tool_frame': 'panda_hand_tcp',
            'gripper_joint': 'panda_finger_joint1', 'gripper_action': '/panda_hand_controller/follow_joint_trajectory',
            'allow_execution': allow, 'pose_store': '/tmp/panda_course_poses.yaml'}], output='screen')]
    for name in ['joint_state_broadcaster', 'panda_arm_controller', 'panda_hand_controller']:
        actions.append(Node(package='controller_manager', executable='spawner', arguments=[name, '--controller-manager-timeout', '60'], parameters=[common], output='screen'))
    if use_rviz:
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share('moveit_resources_panda_moveit_config'), 'launch', 'moveit.rviz')], parameters=[cfg.to_dict(), common]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('allow_execution', default_value='true'),
        DeclareLaunchArgument('partition', default_value='auto'),
        OpaqueFunction(function=build)])
