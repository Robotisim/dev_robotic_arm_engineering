from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
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


class Demo06MoveItPoseGoal(Node):
    def __init__(self) -> None:
        super().__init__('motion_06_moveit_pose_goal')

        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('planning_group', 'panda')
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

        action_name = str(self.get_parameter('move_action_name').value)
        self._client = ActionClient(self, MoveGroup, action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
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

        if self._get_bool_param('log_current_pose_before_goal'):
            self._log_current_pose()

        goal = self._build_goal()
        self.get_logger().info(
            'Sending pose goal xyz=(%.3f, %.3f, %.3f)'
            % (
                float(self.get_parameter('target_x').value),
                float(self.get_parameter('target_y').value),
                float(self.get_parameter('target_z').value),
            )
        )
        self._send_future = self._client.send_goal_async(goal)
        self._send_future.add_done_callback(self._on_goal_response)

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
        self.get_logger().info('motion_06_moveit_pose_goal node is shutting down.')
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
        self.get_logger().info(
            'Current pose (%s in %s): pos=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f)'
            % (
                target_link,
                planning_frame,
                p.x,
                p.y,
                p.z,
                q.x,
                q.y,
                q.z,
                q.w,
            )
        )

    def _get_bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)


def main() -> None:
    rclpy.init()
    node = Demo06MoveItPoseGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
