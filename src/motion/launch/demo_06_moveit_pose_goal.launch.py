from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_panda_arg = DeclareLaunchArgument(
        'start_panda',
        default_value='true',
        description='Start Panda Gazebo + MoveIt bringup before sending the pose goal.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock.',
    )
    start_moveit_delay_arg = DeclareLaunchArgument(
        'start_moveit_delay_sec',
        default_value='10.0',
        description='Delay before move_group starts inside panda bringup.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start MoveIt RViz from panda bringup.',
    )
    start_delay_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='13.0',
        description='Delay before this node sends the MoveIt action goal.',
    )

    target_x_arg = DeclareLaunchArgument('target_x', default_value='0.45')
    target_y_arg = DeclareLaunchArgument('target_y', default_value='0.00')
    target_z_arg = DeclareLaunchArgument('target_z', default_value='0.42')
    target_qx_arg = DeclareLaunchArgument('target_qx', default_value='0.0')
    target_qy_arg = DeclareLaunchArgument('target_qy', default_value='0.0')
    target_qz_arg = DeclareLaunchArgument('target_qz', default_value='0.0')
    target_qw_arg = DeclareLaunchArgument('target_qw', default_value='1.0')

    planning_group_arg = DeclareLaunchArgument('planning_group', default_value='panda_arm')
    planning_frame_arg = DeclareLaunchArgument('planning_frame', default_value='panda_link0')
    target_link_arg = DeclareLaunchArgument('target_link', default_value='panda_link8')
    position_tolerance_arg = DeclareLaunchArgument('goal_position_tolerance', default_value='0.01')
    orientation_tolerance_arg = DeclareLaunchArgument(
        'goal_orientation_tolerance', default_value='0.05'
    )
    planning_time_arg = DeclareLaunchArgument('allowed_planning_time', default_value='5.0')
    attempts_arg = DeclareLaunchArgument('num_planning_attempts', default_value='5')
    velocity_scale_arg = DeclareLaunchArgument('max_velocity_scaling_factor', default_value='0.2')
    acceleration_scale_arg = DeclareLaunchArgument(
        'max_acceleration_scaling_factor', default_value='0.2'
    )
    plan_only_arg = DeclareLaunchArgument(
        'plan_only',
        default_value='false',
        description='Only plan if true. If false, MoveIt plans and executes.',
    )
    pipeline_id_arg = DeclareLaunchArgument('pipeline_id', default_value='')
    planner_id_arg = DeclareLaunchArgument('planner_id', default_value='')
    move_action_name_arg = DeclareLaunchArgument('move_action_name', default_value='/move_action')
    server_timeout_arg = DeclareLaunchArgument('server_wait_timeout_sec', default_value='30.0')
    log_current_pose_arg = DeclareLaunchArgument(
        'log_current_pose_before_goal',
        default_value='true',
        description='Log current end-effector pose from TF before sending the MoveIt goal.',
    )
    current_pose_timeout_arg = DeclareLaunchArgument(
        'current_pose_wait_timeout_sec',
        default_value='2.0',
        description='How long to wait for current TF pose.',
    )

    panda_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('panda'), 'launch', 'panda_ik_gazebo.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('start_panda')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'start_moveit_delay_sec': LaunchConfiguration('start_moveit_delay_sec'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    moveit_goal_node = Node(
        package='motion',
        executable='motion_06_moveit_pose_goal',
        name='motion_06_moveit_pose_goal',
        output='screen',
        parameters=[
            {
                'start_delay_sec': LaunchConfiguration('start_delay_sec'),
                'move_action_name': LaunchConfiguration('move_action_name'),
                'planning_group': LaunchConfiguration('planning_group'),
                'planning_frame': LaunchConfiguration('planning_frame'),
                'target_link': LaunchConfiguration('target_link'),
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_z': LaunchConfiguration('target_z'),
                'target_qx': LaunchConfiguration('target_qx'),
                'target_qy': LaunchConfiguration('target_qy'),
                'target_qz': LaunchConfiguration('target_qz'),
                'target_qw': LaunchConfiguration('target_qw'),
                'goal_position_tolerance': LaunchConfiguration('goal_position_tolerance'),
                'goal_orientation_tolerance': LaunchConfiguration('goal_orientation_tolerance'),
                'allowed_planning_time': LaunchConfiguration('allowed_planning_time'),
                'num_planning_attempts': LaunchConfiguration('num_planning_attempts'),
                'max_velocity_scaling_factor': LaunchConfiguration('max_velocity_scaling_factor'),
                'max_acceleration_scaling_factor': LaunchConfiguration(
                    'max_acceleration_scaling_factor'
                ),
                'plan_only': LaunchConfiguration('plan_only'),
                'pipeline_id': LaunchConfiguration('pipeline_id'),
                'planner_id': LaunchConfiguration('planner_id'),
                'server_wait_timeout_sec': LaunchConfiguration('server_wait_timeout_sec'),
                'log_current_pose_before_goal': LaunchConfiguration('log_current_pose_before_goal'),
                'current_pose_wait_timeout_sec': LaunchConfiguration(
                    'current_pose_wait_timeout_sec'
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            start_panda_arg,
            use_sim_time_arg,
            start_moveit_delay_arg,
            rviz_arg,
            start_delay_arg,
            target_x_arg,
            target_y_arg,
            target_z_arg,
            target_qx_arg,
            target_qy_arg,
            target_qz_arg,
            target_qw_arg,
            planning_group_arg,
            planning_frame_arg,
            target_link_arg,
            position_tolerance_arg,
            orientation_tolerance_arg,
            planning_time_arg,
            attempts_arg,
            velocity_scale_arg,
            acceleration_scale_arg,
            plan_only_arg,
            pipeline_id_arg,
            planner_id_arg,
            move_action_name_arg,
            server_timeout_arg,
            log_current_pose_arg,
            current_pose_timeout_arg,
            panda_bringup,
            moveit_goal_node,
        ]
    )
