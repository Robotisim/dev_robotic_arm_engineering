from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Demo06MoveItPoseGoal(Node):
    def __init__(self, node_name: str = 'motion_06_moveit_pose_goal') -> None:
        super().__init__(node_name)

        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('planning_group', 'panda_arm')
        self.declare_parameter('planning_frame', 'panda_link0')
        self.declare_parameter('target_link', 'panda_link8')
        self.declare_parameter('target_x', 0.45)
        self.declare_parameter('target_y', 0.00)
        self.declare_parameter('target_z', 0.42)
        self.declare_parameter('target_qx', 0.0)
        self.declare_parameter('target_qy', 0.0)
        self.declare_parameter('target_qz', 0.0)
        self.declare_parameter('target_qw', 1.0)
        self.declare_parameter('goal_position_tolerance', 0.01)
        self.declare_parameter('goal_orientation_tolerance', 0.05)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('max_velocity_scaling_factor', 0.2)
        self.declare_parameter('max_acceleration_scaling_factor', 0.2)
        self.declare_parameter('pipeline_id', '')
        self.declare_parameter('planner_id', '')
        self.declare_parameter('plan_only', False)
        self.declare_parameter('server_wait_timeout_sec', 30.0)
        self.declare_parameter('start_delay_sec', 2.0)
        self.declare_parameter('log_current_pose_before_goal', True)
        self.declare_parameter('current_pose_wait_timeout_sec', 2.0)
        self.declare_parameter('use_finger_joint_constraint', False)
        self.declare_parameter('finger_joint_name', 'panda_finger_joint1')
        self.declare_parameter('finger_joint_target', 0.035)
        self.declare_parameter('finger_joint_tolerance_above', 0.002)
        self.declare_parameter('finger_joint_tolerance_below', 0.002)
        self.declare_parameter('finger_joint_weight', 1.0)
        self.declare_parameter('gripper_command', 'none')  # one of: none, open, close
        self.declare_parameter(
            'gripper_target', -1.0
        )  # numeric override; if >= 0, used directly and gripper_command is ignored
        self.declare_parameter('gripper_joint_name', 'panda_finger_joint1')
        self.declare_parameter('gripper_open_position', 0.035)
        self.declare_parameter('gripper_closed_position', 0.0)
        self.declare_parameter('gripper_motion_time_sec', 1.0)
        self.declare_parameter('gripper_wait_after_command_sec', 0.0)

        action_name = str(self.get_parameter('move_action_name').value)
        self._client = ActionClient(self, MoveGroup, action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_trajectory_controller/joint_trajectory', 10
        )
        self._finished = False
        self._send_future = None
        self._result_future = None
        self._start_timer = self.create_timer(
            max(0.0, float(self.get_parameter('start_delay_sec').value)),
            self._send_goal_once,
        )

        self.get_logger().info(
            'Prepared MoveIt pose-goal node: action=%s group=%s frame=%s link=%s'
            % (
                action_name,
                str(self.get_parameter('planning_group').value),
                str(self.get_parameter('planning_frame').value),
                str(self.get_parameter('target_link').value),
            )
        )

    def _send_goal_once(self) -> None:
        if self._finished:
            return

        self._start_timer.cancel()
        timeout = float(self.get_parameter('server_wait_timeout_sec').value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                'MoveIt action server not available on "%s" within %.1fs'
                % (str(self.get_parameter('move_action_name').value), timeout)
            )
            self._finish()
            return

        self._maybe_send_gripper_command()

        if self._get_bool_param('log_current_pose_before_goal'):
            self._log_current_pose()

        goal = self._build_goal()
        target_link = str(self.get_parameter('target_link').value)
        planning_group = str(self.get_parameter('planning_group').value)
        self._log_pose_block(
            title='GOAL POSE',
            planning_group=planning_group,
            planning_frame=str(self.get_parameter('planning_frame').value),
            target_link=target_link,
            x=float(self.get_parameter('target_x').value),
            y=float(self.get_parameter('target_y').value),
            z=float(self.get_parameter('target_z').value),
            qx=float(self.get_parameter('target_qx').value),
            qy=float(self.get_parameter('target_qy').value),
            qz=float(self.get_parameter('target_qz').value),
            qw=float(self.get_parameter('target_qw').value),
        )
        if self._get_bool_param('use_finger_joint_constraint'):
            self.get_logger().info(
                'Applying finger joint constraint: %s=%.4f (+%.4f / -%.4f)'
                % (
                    str(self.get_parameter('finger_joint_name').value),
                    float(self.get_parameter('finger_joint_target').value),
                    float(self.get_parameter('finger_joint_tolerance_above').value),
                    float(self.get_parameter('finger_joint_tolerance_below').value),
                )
            )
        self._send_future = self._client.send_goal_async(goal)
        self._send_future.add_done_callback(self._on_goal_response)

    def _maybe_send_gripper_command(self) -> None:
        gripper_target = float(self.get_parameter('gripper_target').value)
        command = str(self.get_parameter('gripper_command').value).strip().lower()

        if gripper_target >= 0.0:
            target = gripper_target
            command_label = f'set({target:.4f})'
        else:
            if command in {'', 'none'}:
                return

            if command == 'open':
                target = float(self.get_parameter('gripper_open_position').value)
            elif command == 'close':
                target = float(self.get_parameter('gripper_closed_position').value)
            else:
                self.get_logger().warn(
                    'Unknown gripper_command "%s". Use: none | open | close. Skipping gripper command.'
                    % command
                )
                return
            command_label = command

        gripper_joint_name = str(self.get_parameter('gripper_joint_name').value)
        motion_time = max(0.05, float(self.get_parameter('gripper_motion_time_sec').value))

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [gripper_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start = DurationMsg(
            sec=int(motion_time),
            nanosec=int((motion_time - int(motion_time)) * 1.0e9),
        )
        traj_msg.points = [point]

        self._gripper_pub.publish(traj_msg)
        self.get_logger().info(
            'Published gripper command: %s -> %s=%.4f in %.2fs'
            % (command_label, gripper_joint_name, target, motion_time)
        )

        wait_after = max(0.0, float(self.get_parameter('gripper_wait_after_command_sec').value))
        if wait_after > 0.0:
            self.get_logger().info(
                'Waiting %.2fs after gripper command before sending pose goal.' % wait_after
            )
            time.sleep(wait_after)

    def _build_goal(self) -> MoveGroup.Goal:
        planning_frame = str(self.get_parameter('planning_frame').value)
        target_link = str(self.get_parameter('target_link').value)

        target_x = float(self.get_parameter('target_x').value)
        target_y = float(self.get_parameter('target_y').value)
        target_z = float(self.get_parameter('target_z').value)
        target_qx = float(self.get_parameter('target_qx').value)
        target_qy = float(self.get_parameter('target_qy').value)
        target_qz = float(self.get_parameter('target_qz').value)
        target_qw = float(self.get_parameter('target_qw').value)

        quat_norm = math.sqrt(
            target_qx * target_qx
            + target_qy * target_qy
            + target_qz * target_qz
            + target_qw * target_qw
        )
        if quat_norm < 1.0e-9:
            self.get_logger().warn('Target quaternion norm is 0; defaulting to identity orientation.')
            target_qx, target_qy, target_qz, target_qw = 0.0, 0.0, 0.0, 1.0
        else:
            target_qx /= quat_norm
            target_qy /= quat_norm
            target_qz /= quat_norm
            target_qw /= quat_norm

        position_tolerance = max(1.0e-4, float(self.get_parameter('goal_position_tolerance').value))
        orientation_tolerance = max(
            1.0e-4, float(self.get_parameter('goal_orientation_tolerance').value)
        )

        goal = MoveGroup.Goal()
        goal.request.group_name = str(self.get_parameter('planning_group').value)
        goal.request.allowed_planning_time = float(self.get_parameter('allowed_planning_time').value)
        goal.request.num_planning_attempts = int(self.get_parameter('num_planning_attempts').value)
        goal.request.max_velocity_scaling_factor = float(
            self.get_parameter('max_velocity_scaling_factor').value
        )
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter('max_acceleration_scaling_factor').value
        )

        pipeline_id = str(self.get_parameter('pipeline_id').value).strip()
        planner_id = str(self.get_parameter('planner_id').value).strip()
        if pipeline_id:
            goal.request.pipeline_id = pipeline_id
        if planner_id:
            goal.request.planner_id = planner_id

        now = self.get_clock().now().to_msg()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = planning_frame
        position_constraint.header.stamp = now
        position_constraint.link_name = target_link
        position_constraint.weight = 1.0

        position_region = SolidPrimitive()
        position_region.type = SolidPrimitive.BOX
        position_region.dimensions = [
            2.0 * position_tolerance,
            2.0 * position_tolerance,
            2.0 * position_tolerance,
        ]

        region_pose = Pose()
        region_pose.position.x = target_x
        region_pose.position.y = target_y
        region_pose.position.z = target_z
        region_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(position_region)
        position_constraint.constraint_region.primitive_poses.append(region_pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = planning_frame
        orientation_constraint.header.stamp = now
        orientation_constraint.link_name = target_link
        orientation_constraint.orientation.x = target_qx
        orientation_constraint.orientation.y = target_qy
        orientation_constraint.orientation.z = target_qz
        orientation_constraint.orientation.w = target_qw
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        if self._get_bool_param('use_finger_joint_constraint'):
            finger_joint_constraint = JointConstraint()
            finger_joint_constraint.joint_name = str(self.get_parameter('finger_joint_name').value)
            finger_joint_constraint.position = float(self.get_parameter('finger_joint_target').value)
            finger_joint_constraint.tolerance_above = max(
                0.0, float(self.get_parameter('finger_joint_tolerance_above').value)
            )
            finger_joint_constraint.tolerance_below = max(
                0.0, float(self.get_parameter('finger_joint_tolerance_below').value)
            )
            finger_joint_constraint.weight = max(
                0.0, float(self.get_parameter('finger_joint_weight').value)
            )
            constraints.joint_constraints.append(finger_joint_constraint)

        goal.request.goal_constraints = [constraints]

        goal.planning_options.plan_only = self._get_bool_param('plan_only')
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def _on_goal_response(self, future) -> None:
        if self._finished:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Failed to send goal: {exc}')
            self._finish()
            return

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal was rejected.')
            self._finish()
            return

        self.get_logger().info('MoveIt accepted goal; waiting for execution result...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        if self._finished:
            return

        try:
            wrapped_result = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Failed to get MoveIt result: {exc}')
            self._finish()
            return

        result = wrapped_result.result
        error_code = result.error_code
        planning_time = float(result.planning_time)

        if error_code.val == MoveItErrorCodes.SUCCESS:
            if self._get_bool_param('plan_only'):
                self.get_logger().info(
                    f'Planning succeeded in {planning_time:.3f}s (plan_only=true, no execution).'
                )
            else:
                self.get_logger().info(f'Planning and execution succeeded in {planning_time:.3f}s.')
        else:
            self.get_logger().error(
                'MoveIt failed: code=%d message="%s" source="%s"'
                % (error_code.val, error_code.message, error_code.source)
            )

        self._finish()

    def _finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self.get_logger().info('%s node is shutting down.' % self.get_name())
        if rclpy.ok():
            rclpy.shutdown()

    def _log_current_pose(self) -> None:
        planning_frame = str(self.get_parameter('planning_frame').value)
        target_link = str(self.get_parameter('target_link').value)
        wait_timeout = max(0.0, float(self.get_parameter('current_pose_wait_timeout_sec').value))
        timeout = Duration(seconds=wait_timeout)

        try:
            if not self._tf_buffer.can_transform(
                planning_frame,
                target_link,
                Time(),
                timeout=timeout,
            ):
                self.get_logger().warn(
                    'Could not read current pose for "%s" in "%s" within %.2fs'
                    % (target_link, planning_frame, wait_timeout)
                )
                return

            transform = self._tf_buffer.lookup_transform(
                planning_frame,
                target_link,
                Time(),
                timeout=timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(f'Could not read current pose from TF: {exc}')
            return

        p = transform.transform.translation
        q = transform.transform.rotation
        self._log_pose_block(
            title='CURRENT POSE',
            planning_group=None,
            planning_frame=planning_frame,
            target_link=target_link,
            x=p.x,
            y=p.y,
            z=p.z,
            qx=q.x,
            qy=q.y,
            qz=q.z,
            qw=q.w,
        )

    def _log_pose_block(
        self,
        *,
        title: str,
        planning_group: str | None,
        planning_frame: str,
        target_link: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        lines = [
            '',
            f'-------------------- {title} --------------------',
            f'  Frame : {planning_frame}',
            f'  Link  : {target_link}',
        ]
        if planning_group:
            lines.append(f'  Group : {planning_group}')
        lines.extend(
            [
                f'  Pos   : x={x: .4f}   y={y: .4f}   z={z: .4f}',
                f'  Quat  : qx={qx: .4f}  qy={qy: .4f}  qz={qz: .4f}  qw={qw: .4f}',
                '------------------------------------------------------',
            ]
        )
        self.get_logger().info('\n'.join(lines))

    def _get_bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)


def _run(node_name: str) -> None:
    rclpy.init()
    node = Demo06MoveItPoseGoal(node_name=node_name)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    _run('motion_06_moveit_pose_goal')


def main_arm_hand() -> None:
    _run('motion_07_moveit_arm_hand_pose_goal')
