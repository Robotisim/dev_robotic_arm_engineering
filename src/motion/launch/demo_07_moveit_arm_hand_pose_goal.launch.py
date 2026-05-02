from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_delay_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='1.0',
        description='Delay before this node sends the MoveIt action goal.',
    )

    target_x_arg = DeclareLaunchArgument('target_x', default_value='0.45')
    target_y_arg = DeclareLaunchArgument('target_y', default_value='0.00')
    target_z_arg = DeclareLaunchArgument('target_z', default_value='0.42')
    target_qx_arg = DeclareLaunchArgument('target_qx', default_value='0.0')
    target_qy_arg = DeclareLaunchArgument('target_qy', default_value='0.0')
    target_qz_arg = DeclareLaunchArgument('target_qz', default_value='0.0')
    target_qw_arg = DeclareLaunchArgument('target_qw', default_value='1.0')

    planning_group_arg = DeclareLaunchArgument('planning_group', default_value='panda_arm_hand')
    planning_frame_arg = DeclareLaunchArgument('planning_frame', default_value='panda_link0')
    target_link_arg = DeclareLaunchArgument('target_link', default_value='panda_hand_tcp')
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
    use_finger_joint_constraint_arg = DeclareLaunchArgument(
        'use_finger_joint_constraint',
        default_value='false',
        description='If true, keep a finger joint opening target while sending pose goals.',
    )
    finger_joint_name_arg = DeclareLaunchArgument(
        'finger_joint_name',
        default_value='panda_finger_joint1',
        description='Finger joint to constrain.',
    )
    finger_joint_target_arg = DeclareLaunchArgument(
        'finger_joint_target',
        default_value='0.035',
        description='Finger opening target (0.035=open, 0.0=closed).',
    )
    finger_joint_tolerance_above_arg = DeclareLaunchArgument(
        'finger_joint_tolerance_above',
        default_value='0.002',
        description='Allowed positive deviation for finger opening target.',
    )
    finger_joint_tolerance_below_arg = DeclareLaunchArgument(
        'finger_joint_tolerance_below',
        default_value='0.002',
        description='Allowed negative deviation for finger opening target.',
    )
    finger_joint_weight_arg = DeclareLaunchArgument(
        'finger_joint_weight',
        default_value='1.0',
        description='Constraint weight for finger opening target.',
    )
    gripper_command_arg = DeclareLaunchArgument(
        'gripper_command',
        default_value='none',
        description='Optional gripper action before pose goal: none | open | close.',
    )
    gripper_target_arg = DeclareLaunchArgument(
        'gripper_target',
        default_value='-1.0',
        description='Direct gripper opening target (e.g. 0.02). Use -1.0 to disable and use gripper_command.',
    )
    gripper_joint_name_arg = DeclareLaunchArgument(
        'gripper_joint_name',
        default_value='panda_finger_joint1',
        description='Gripper joint name used by the gripper controller.',
    )
    gripper_open_position_arg = DeclareLaunchArgument(
        'gripper_open_position',
        default_value='0.035',
        description='Open position used for gripper_command:=open.',
    )
    gripper_closed_position_arg = DeclareLaunchArgument(
        'gripper_closed_position',
        default_value='0.0',
        description='Closed position used for gripper_command:=close.',
    )
    gripper_motion_time_sec_arg = DeclareLaunchArgument(
        'gripper_motion_time_sec',
        default_value='1.0',
        description='Duration for gripper controller trajectory point.',
    )
    gripper_wait_after_command_sec_arg = DeclareLaunchArgument(
        'gripper_wait_after_command_sec',
        default_value='0.0',
        description='Optional wait time after gripper command before pose goal.',
    )

    moveit_goal_node = Node(
        package='motion',
        executable='motion_07_moveit_arm_hand_pose_goal',
        name='motion_07_moveit_arm_hand_pose_goal',
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
                'use_finger_joint_constraint': LaunchConfiguration('use_finger_joint_constraint'),
                'finger_joint_name': LaunchConfiguration('finger_joint_name'),
                'finger_joint_target': LaunchConfiguration('finger_joint_target'),
                'finger_joint_tolerance_above': LaunchConfiguration(
                    'finger_joint_tolerance_above'
                ),
                'finger_joint_tolerance_below': LaunchConfiguration(
                    'finger_joint_tolerance_below'
                ),
                'finger_joint_weight': LaunchConfiguration('finger_joint_weight'),
                'gripper_command': LaunchConfiguration('gripper_command'),
                'gripper_target': LaunchConfiguration('gripper_target'),
                'gripper_joint_name': LaunchConfiguration('gripper_joint_name'),
                'gripper_open_position': LaunchConfiguration('gripper_open_position'),
                'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
                'gripper_motion_time_sec': LaunchConfiguration('gripper_motion_time_sec'),
                'gripper_wait_after_command_sec': LaunchConfiguration(
                    'gripper_wait_after_command_sec'
                ),
            }
        ],
    )

    return LaunchDescription(
        [
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
            use_finger_joint_constraint_arg,
            finger_joint_name_arg,
            finger_joint_target_arg,
            finger_joint_tolerance_above_arg,
            finger_joint_tolerance_below_arg,
            finger_joint_weight_arg,
            gripper_command_arg,
            gripper_target_arg,
            gripper_joint_name_arg,
            gripper_open_position_arg,
            gripper_closed_position_arg,
            gripper_motion_time_sec_arg,
            gripper_wait_after_command_sec_arg,
            moveit_goal_node,
        ]
    )
