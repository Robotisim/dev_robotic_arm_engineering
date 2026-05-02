from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated clock.',
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz')

    description_share = FindPackageShare('robotic_arm_description')
    control_share = FindPackageShare('robotic_arm_control')
    bringup_share = FindPackageShare('robotic_arm_bringup')
    xacro_file = PathJoinSubstitution(
        [description_share, 'urdf', 'stick_arm', 'stick_arm_4dof.urdf.xacro']
    )
    controllers_file = PathJoinSubstitution(
        [control_share, 'config', 'stick_arm', 'controllers.yaml']
    )
    rviz_config = PathJoinSubstitution([bringup_share, 'rviz', 'stick_arm.rviz'])

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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz_node,
        ]
    )
