from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray

from .panda_kinematics import (
    DEFAULT_WORKSPACE,
    JOINT_NAMES,
    NEUTRAL_JOINT_POSITIONS,
    PandaKinematics,
    check_workspace_path,
)
from .trajectory_utils import build_joint_trajectory, make_path_markers


class TrajectoryDemoBase(Node, ABC):
    def __init__(self, node_name: str, path_name: str, point_dt: float) -> None:
        super().__init__(node_name)

        self.path_name = path_name
        self.point_dt = float(point_dt)
        self.workspace_bounds = DEFAULT_WORKSPACE
        self.kinematics = PandaKinematics()

        self.declare_parameter(
            'trajectory_topic',
            '/joint_trajectory_controller/joint_trajectory',
        )
        trajectory_topic = str(self.get_parameter('trajectory_topic').value)

        self._joint_state_cache: Dict[str, float] = {}
        self._has_run = False
        self._pending_trajectory: JointTrajectory | None = None
        self._pending_waypoint_count = 0
        self._publish_timer = None

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = QoSReliabilityPolicy.RELIABLE
        marker_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.trajectory_publisher = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/motion/trajectory_markers', marker_qos
        )
        self.status_publisher = self.create_publisher(String, '/motion/status', 10)

        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 20)

        self.declare_parameter('start_delay_sec', 2.5)
        self.declare_parameter('marker_preview_sec', 1.0)
        self.declare_parameter('trajectory_start_offset_sec', 0.8)

        start_delay = float(self.get_parameter('start_delay_sec').value)
        self._run_timer = self.create_timer(start_delay, self._run_once)

    def _joint_state_callback(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            self._joint_state_cache[name] = float(position)

    def _current_joint_positions(self) -> np.ndarray:
        if all(name in self._joint_state_cache for name in JOINT_NAMES):
            return np.array([self._joint_state_cache[name] for name in JOINT_NAMES], dtype=np.float64)
        return NEUTRAL_JOINT_POSITIONS.copy()

    @abstractmethod
    def build_cartesian_path(self, current_q: np.ndarray) -> np.ndarray:
        """Return [N,3] Cartesian waypoints in frame panda_link0."""

    def _run_once(self) -> None:
        if self._has_run:
            return
        self._has_run = True
        self._run_timer.cancel()

        current_q = self._current_joint_positions()
        cartesian_path = np.asarray(self.build_cartesian_path(current_q), dtype=np.float64)

        if cartesian_path.ndim != 2 or cartesian_path.shape[1] != 3 or cartesian_path.shape[0] == 0:
            self._publish_status(f'{self.path_name}: invalid path shape {cartesian_path.shape}')
            self.get_logger().error('build_cartesian_path must return [N,3] with N>0')
            return

        is_valid_workspace, workspace_reason = check_workspace_path(cartesian_path, self.workspace_bounds)
        stamp = self.get_clock().now().to_msg()

        self.marker_publisher.publish(
            make_path_markers(
                points=cartesian_path,
                frame_id='panda_link0',
                stamp=stamp,
                namespace=self.path_name,
                accepted=is_valid_workspace,
            )
        )

        if not is_valid_workspace:
            message = f'{self.path_name}: rejected - {workspace_reason}'
            self._publish_status(message)
            self.get_logger().warn(message)
            return

        solutions = []
        seed = current_q
        for index, waypoint in enumerate(cartesian_path):
            q_solution, success, iterations, error_norm = self.kinematics.solve_position_ik(
                waypoint, seed=seed
            )
            if not success:
                message = (
                    f'{self.path_name}: rejected - IK failed at waypoint {index} '
                    f'(iters={iterations}, err={error_norm:.4f} m)'
                )
                self.marker_publisher.publish(
                    make_path_markers(
                        points=cartesian_path,
                        frame_id='panda_link0',
                        stamp=stamp,
                        namespace=self.path_name,
                        accepted=False,
                    )
                )
                self._publish_status(message)
                self.get_logger().error(message)
                return

            solutions.append(q_solution)
            seed = q_solution

        trajectory_start_offset = float(self.get_parameter('trajectory_start_offset_sec').value)
        trajectory = build_joint_trajectory(
            joint_names=JOINT_NAMES,
            q_points=np.asarray(solutions, dtype=np.float64),
            point_dt=self.point_dt,
            start_delay=max(0.0, trajectory_start_offset),
            start_q=current_q,
        )

        self._schedule_trajectory_publish(trajectory, waypoint_count=len(solutions))

    def _schedule_trajectory_publish(self, trajectory: JointTrajectory, waypoint_count: int) -> None:
        marker_preview_sec = float(self.get_parameter('marker_preview_sec').value)

        self._pending_trajectory = trajectory
        self._pending_waypoint_count = int(waypoint_count)

        if marker_preview_sec <= 0.0:
            self._publish_pending_trajectory()
            return

        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self.destroy_timer(self._publish_timer)

        self._publish_status(
            f'{self.path_name}: markers published, waiting {marker_preview_sec:.2f}s before motion command'
        )
        self.get_logger().info(
            f'{self.path_name}: published markers first, command in {marker_preview_sec:.2f}s'
        )

        self._publish_timer = self.create_timer(marker_preview_sec, self._publish_pending_trajectory)

    def _publish_pending_trajectory(self) -> None:
        if self._pending_trajectory is None:
            return

        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self.destroy_timer(self._publish_timer)
            self._publish_timer = None

        self.trajectory_publisher.publish(self._pending_trajectory)
        message = (
            f'{self.path_name}: accepted - {self._pending_waypoint_count} waypoints published '
            'after marker preview'
        )
        self._publish_status(message)
        self.get_logger().info(message)

        self._pending_trajectory = None
        self._pending_waypoint_count = 0

    def _publish_status(self, text: str) -> None:
        status = String()
        status.data = text
        self.status_publisher.publish(status)


def spin_demo(node_factory):
    rclpy.init()
    node = node_factory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
