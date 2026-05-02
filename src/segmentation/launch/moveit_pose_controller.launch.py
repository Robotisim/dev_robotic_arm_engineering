"""
Launch file for MoveIt2 pose controller node.

This node accepts goal poses and sends them to MoveIt2's move_group action server.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    move_action_name = LaunchConfiguration("move_action_name")
    planning_group = LaunchConfiguration("planning_group")
    planning_frame = LaunchConfiguration("planning_frame")
    end_effector_link = LaunchConfiguration("end_effector_link")

    # Goal pose arguments
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    goal_qx = LaunchConfiguration("goal_qx")
    goal_qy = LaunchConfiguration("goal_qy")
    goal_qz = LaunchConfiguration("goal_qz")
    goal_qw = LaunchConfiguration("goal_qw")

    # Planning parameters
    allowed_planning_time = LaunchConfiguration("allowed_planning_time")
    num_planning_attempts = LaunchConfiguration("num_planning_attempts")
    max_velocity_scaling = LaunchConfiguration("max_velocity_scaling")
    max_acceleration_scaling = LaunchConfiguration("max_acceleration_scaling")
    planner_id = LaunchConfiguration("planner_id")
    plan_only = LaunchConfiguration("plan_only")

    return LaunchDescription(
        [
            # MoveIt action and planning group configuration
            DeclareLaunchArgument(
                "move_action_name",
                default_value="/move_action",
                description="MoveIt move group action server name",
            ),
            DeclareLaunchArgument(
                "planning_group",
                default_value="panda_arm",
                description="MoveIt planning group name",
            ),
            DeclareLaunchArgument(
                "planning_frame",
                default_value="panda_link0",
                description="Planning frame (base frame)",
            ),
            DeclareLaunchArgument(
                "end_effector_link",
                default_value="panda_link8",
                description="End effector link name",
            ),
            # Goal pose configuration
            DeclareLaunchArgument(
                "goal_x",
                default_value="0.4",
                description="Goal position X coordinate (meters)",
            ),
            DeclareLaunchArgument(
                "goal_y",
                default_value="0.0",
                description="Goal position Y coordinate (meters)",
            ),
            DeclareLaunchArgument(
                "goal_z",
                default_value="0.4",
                description="Goal position Z coordinate (meters)",
            ),
            DeclareLaunchArgument(
                "goal_qx",
                default_value="0.0",
                description="Goal orientation quaternion X",
            ),
            DeclareLaunchArgument(
                "goal_qy",
                default_value="0.0",
                description="Goal orientation quaternion Y",
            ),
            DeclareLaunchArgument(
                "goal_qz",
                default_value="0.0",
                description="Goal orientation quaternion Z",
            ),
            DeclareLaunchArgument(
                "goal_qw",
                default_value="1.0",
                description="Goal orientation quaternion W",
            ),
            # Planning parameters
            DeclareLaunchArgument(
                "allowed_planning_time",
                default_value="5.0",
                description="Maximum time allowed for planning (seconds)",
            ),
            DeclareLaunchArgument(
                "num_planning_attempts",
                default_value="5",
                description="Number of planning attempts",
            ),
            DeclareLaunchArgument(
                "max_velocity_scaling",
                default_value="0.2",
                description="Maximum velocity scaling factor (0.0-1.0)",
            ),
            DeclareLaunchArgument(
                "max_acceleration_scaling",
                default_value="0.2",
                description="Maximum acceleration scaling factor (0.0-1.0)",
            ),
            DeclareLaunchArgument(
                "planner_id",
                default_value="RRTConnectkConfigDefault",
                description="MoveIt planner ID",
            ),
            DeclareLaunchArgument(
                "plan_only",
                default_value="false",
                description="If true, only plan without executing",
            ),
            # MoveIt pose controller node
            Node(
                package="cube_segmentation",
                executable="moveit_pose_controller_node",
                name="moveit_pose_controller",
                output="screen",
                parameters=[
                    {
                        "move_action_name": move_action_name,
                        "planning_group": planning_group,
                        "planning_frame": planning_frame,
                        "end_effector_link": end_effector_link,
                        "goal_x": goal_x,
                        "goal_y": goal_y,
                        "goal_z": goal_z,
                        "goal_qx": goal_qx,
                        "goal_qy": goal_qy,
                        "goal_qz": goal_qz,
                        "goal_qw": goal_qw,
                        "allowed_planning_time": allowed_planning_time,
                        "num_planning_attempts": num_planning_attempts,
                        "max_velocity_scaling": max_velocity_scaling,
                        "max_acceleration_scaling": max_acceleration_scaling,
                        "planner_id": planner_id,
                        "plan_only": plan_only,
                    }
                ],
            ),
        ]
    )
