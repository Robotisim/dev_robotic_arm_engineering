"""RoboFlex model and state source; hardware retains the repository's controller path."""
import os
import tempfile
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in ('display', 'mock', 'gazebo', 'hardware'):
        raise ValueError('backend must be display, mock, gazebo, or hardware')
    model_file = os.path.join(share('roboflex_description'), 'urdf', 'real_hardware.urdf.xacro')
    model = ET.fromstring(xacro.process_file(model_file).toxml())
    actions = []
    if backend == 'hardware':
        ip = LaunchConfiguration('firmware_ip').perform(context)
        if not ip or ip == '255.255.255.255':
            raise ValueError('Specify the actual firmware_ip for hardware mode')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('roboflex_control'), 'launch', 'real_hardware.launch.py')), launch_arguments={'firmware_ip': ip, 'use_sim_time': 'false'}.items()))
        return actions
    for c in model.findall('ros2_control'):
        model.remove(c)
    sim = backend == 'gazebo'
    if backend in ('mock', 'gazebo'):
        control = ET.SubElement(model, 'ros2_control', name='CourseRoboFlex', type='system')
        hardware = ET.SubElement(control, 'hardware')
        ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem' if sim else 'mock_components/GenericSystem'
        initial = [0.0349066, -0.3839724, -0.9250245, -0.3665191, 0.0]
        for name, value in zip(['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_gripper'], initial):
            joint = ET.SubElement(control, 'joint', name=name)
            ET.SubElement(joint, 'command_interface', name='position')
            position = ET.SubElement(joint, 'state_interface', name='position')
            ET.SubElement(position, 'param', name='initial_value').text = str(value)
    controllers = os.path.join(share('roboflex_control'), 'config', 'ros2_controllers.yaml')
    if sim:
        # This early RoboFlex lab teaches kinematics/angle control. Disable
        # gravity in the generated simulation model; real dynamics are not
        # identified for the angle-only servos. Panda later uses gravity.
        for link in model.findall('link'):
            g = ET.SubElement(model, 'gazebo', reference=link.get('name'))
            ET.SubElement(g, 'gravity').text = 'false'
        ET.SubElement(model, 'link', name='world')
        fixed = ET.SubElement(model, 'joint', name='world_fixed', type='fixed')
        ET.SubElement(fixed, 'parent', link='world')
        ET.SubElement(fixed, 'child', link='base_link')
        for name in ['gripper_drive_link', 'gripper_actuator_link']:
            link = model.find("link[@name='%s']" % name)
            if link.find('inertial') is None:
                inertia = ET.SubElement(link, 'inertial')
                ET.SubElement(inertia, 'mass', value='0.001')
                ET.SubElement(inertia, 'inertia', ixx='1e-7', iyy='1e-7', izz='1e-7', ixy='0', ixz='0', iyz='0')
        for joint in model.findall('joint'):
            mimic = joint.find('mimic')
            if mimic is not None:
                multiplier = float(mimic.get('multiplier', '1'))
                limit = joint.find('limit')
                limit.set('lower', str(min(0.0, multiplier * 0.523333)))
                limit.set('upper', str(max(0.0, multiplier * 0.523333)))
                element = ET.SubElement(control, 'joint', name=joint.get('name'))
                ET.SubElement(element, 'state_interface', name='position')
        gazebo = ET.SubElement(model, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin', filename='gz_ros2_control-system', name='gz_ros2_control::GazeboSimROS2ControlPlugin')
        ET.SubElement(plugin, 'parameters').text = controllers
    xml = ET.tostring(model, encoding='unicode')
    description = {'robot_description': xml, 'use_sim_time': sim}
    actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[description]))
    if backend == 'display':
        actions.append(Node(package='arm_module_01', executable='joint_lab', parameters=[description, {'operation': 'display'}], output='screen'))
    elif backend == 'mock':
        actions.append(Node(package='controller_manager', executable='ros2_control_node', parameters=[description, controllers], output='screen'))
    else:
        fd, path = tempfile.mkstemp(prefix='roboflex_course_', suffix='.urdf')
        with os.fdopen(fd, 'w') as stream:
            stream.write(xml)
        world = os.path.join(share('arm_module_01'), 'config', 'empty.sdf')
        headless = LaunchConfiguration('headless').perform(context) == 'true'
        actions += [SetEnvironmentVariable('GZ_PARTITION', 'roboflex_kinematics'),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')), launch_arguments={'gz_args': ('-r -s ' if headless else '-r ') + world}.items()),
                    Node(package='ros_gz_sim', executable='create', arguments=['-world', 'roboflex_course', '-name', 'roboflex', '-file', path], output='screen'),
                    Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/world/roboflex_course/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], remappings=[('/world/roboflex_course/clock', '/clock')])]
    if backend in ('mock', 'gazebo'):
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']:
            actions.append(Node(package='controller_manager', executable='spawner', arguments=[name, '--controller-manager-timeout', '60'], parameters=[{'use_sim_time': sim}], output='screen'))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share('roboflex_moveit'), 'config', 'kinematics.rviz')], parameters=[description]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend', default_value='display'), DeclareLaunchArgument('firmware_ip', default_value=''), DeclareLaunchArgument('headless', default_value='false'), DeclareLaunchArgument('rviz', default_value='true'), OpaqueFunction(function=build)])
