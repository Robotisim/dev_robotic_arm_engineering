from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('arm_sim_bringup')
    xacro_file = PathJoinSubstitution([package_share, 'urdf', 'stick_arm_4dof.urdf.xacro'])
    controllers_file = PathJoinSubstitution([package_share, 'config', 'stick_arm_controllers.yaml'])
    rviz_config = PathJoinSubstitution([package_share, 'rviz', 'stick_arm.rviz'])

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                xacro_file,
                ' ',
                'name:=stick_arm',
                ' ',
                'controllers_file:=',
                controllers_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='stick_arm_robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': False,
            }
        ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='stick_arm_joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz_node,
        ]
    )
