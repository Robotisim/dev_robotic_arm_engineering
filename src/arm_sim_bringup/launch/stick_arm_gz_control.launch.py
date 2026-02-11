from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('arm_sim_bringup')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz')
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf', description='Gazebo Harmonic world'
    )

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

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', LaunchConfiguration('world')],
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='stick_arm_robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_stick_arm',
        arguments=[
            '-name',
            'stick_arm',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.15',
        ],
        output='screen',
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='stick_arm_jsb_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='stick_arm_controller_spawner',
        arguments=[
            'stick_arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[load_joint_state_broadcaster])
    )
    controller_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_arm_controller])
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
            rviz_arg,
            world_arg,
            gz_sim,
            robot_state_publisher,
            clock_bridge,
            spawn_robot,
            jsb_after_spawn,
            controller_after_jsb,
            rviz_node,
        ]
    )
