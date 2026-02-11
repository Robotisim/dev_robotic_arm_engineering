from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import numpy as np


JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]

JOINT_LOWER_LIMITS = np.array(
    [-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671], dtype=np.float64
)
JOINT_UPPER_LIMITS = np.array(
    [2.9671, 1.8326, 2.9671, 0.0873, 2.9671, 3.8223, 2.9671], dtype=np.float64
)

NEUTRAL_JOINT_POSITIONS = np.array([0.0, -0.785, 0.0, -2.2, 0.0, 1.7, 0.78], dtype=np.float64)


@dataclass(frozen=True)
class WorkspaceBounds:
    x_min: float = 0.20
    x_max: float = 0.80
    y_min: float = -0.55
    y_max: float = 0.55
    z_min: float = 0.05
    z_max: float = 0.85
    radius_min: float = 0.20
    radius_max: float = 0.90

    def check(self, point: Sequence[float]) -> Tuple[bool, str]:
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        radius = math.sqrt(x * x + y * y + z * z)

        if x < self.x_min or x > self.x_max:
            return False, f'x={x:.3f} outside [{self.x_min:.2f}, {self.x_max:.2f}]'
        if y < self.y_min or y > self.y_max:
            return False, f'y={y:.3f} outside [{self.y_min:.2f}, {self.y_max:.2f}]'
        if z < self.z_min or z > self.z_max:
            return False, f'z={z:.3f} outside [{self.z_min:.2f}, {self.z_max:.2f}]'
        if radius < self.radius_min or radius > self.radius_max:
            return (
                False,
                f'radius={radius:.3f} outside [{self.radius_min:.2f}, {self.radius_max:.2f}]',
            )
        return True, 'inside workspace'


DEFAULT_WORKSPACE = WorkspaceBounds()


def check_workspace_path(
    points: np.ndarray,
    bounds: WorkspaceBounds = DEFAULT_WORKSPACE,
) -> Tuple[bool, str]:
    for index, point in enumerate(points):
        is_valid, reason = bounds.check(point)
        if not is_valid:
            return False, f'waypoint {index} rejected: {reason}'
    return True, 'all waypoints are inside workspace'


class PandaKinematics:
    """Position FK/Jacobian and damped least-squares IK for Panda based on panda_sim URDF."""

    def __init__(self) -> None:
        joint_origins = (
            ((0.0, 0.0, 0.333), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
            ((0.0, -0.316, 0.0), (math.pi / 2.0, 0.0, 0.0)),
            ((0.0825, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
            ((-0.0825, 0.384, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
            ((0.088, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
        )

        self._origin_transforms = tuple(
            self._homogeneous(self._rpy_to_rotation(*rpy), np.array(xyz, dtype=np.float64))
            for xyz, rpy in joint_origins
        )
        self._tool_transform = self._homogeneous(np.eye(3), np.array([0.0, 0.0, 0.107], dtype=np.float64))
        self._joint_axis_local = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    @staticmethod
    def _homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        return transform

    @staticmethod
    def _rot_x(theta: float) -> np.ndarray:
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, c, -s],
                [0.0, s, c],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _rot_y(theta: float) -> np.ndarray:
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array(
            [
                [c, 0.0, s],
                [0.0, 1.0, 0.0],
                [-s, 0.0, c],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _rot_z(theta: float) -> np.ndarray:
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    @classmethod
    def _rpy_to_rotation(cls, roll: float, pitch: float, yaw: float) -> np.ndarray:
        return cls._rot_z(yaw) @ cls._rot_y(pitch) @ cls._rot_x(roll)

    @classmethod
    def _rot_z_homogeneous(cls, angle: float) -> np.ndarray:
        return cls._homogeneous(cls._rot_z(angle), np.zeros(3, dtype=np.float64))

    @staticmethod
    def _as_joint_vector(joint_positions: Sequence[float]) -> np.ndarray:
        q = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
        if q.shape[0] != 7:
            raise ValueError(f'Expected 7 joints, got {q.shape[0]}')
        return np.clip(q, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

    def forward_kinematics_and_jacobian(self, joint_positions: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        q = self._as_joint_vector(joint_positions)
        transform = np.eye(4, dtype=np.float64)

        joint_positions_world = np.zeros((7, 3), dtype=np.float64)
        joint_axes_world = np.zeros((7, 3), dtype=np.float64)

        for index, origin_transform in enumerate(self._origin_transforms):
            joint_frame = transform @ origin_transform
            joint_positions_world[index, :] = joint_frame[:3, 3]
            joint_axes_world[index, :] = joint_frame[:3, :3] @ self._joint_axis_local
            transform = joint_frame @ self._rot_z_homogeneous(q[index])

        end_frame = transform @ self._tool_transform
        end_position = end_frame[:3, 3]

        jacobian = np.zeros((3, 7), dtype=np.float64)
        for index in range(7):
            jacobian[:, index] = np.cross(
                joint_axes_world[index], end_position - joint_positions_world[index]
            )

        return end_position, jacobian

    def forward_position(self, joint_positions: Sequence[float]) -> np.ndarray:
        end_position, _ = self.forward_kinematics_and_jacobian(joint_positions)
        return end_position

    def solve_position_ik(
        self,
        target_position: Sequence[float],
        seed: Sequence[float] | None = None,
        max_iterations: int = 120,
        tolerance: float = 0.003,
        damping: float = 0.05,
        step_scale: float = 0.85,
    ) -> Tuple[np.ndarray, bool, int, float]:
        target = np.asarray(target_position, dtype=np.float64).reshape(3)
        q = self._as_joint_vector(seed if seed is not None else NEUTRAL_JOINT_POSITIONS)

        for iteration in range(max_iterations):
            current, jacobian = self.forward_kinematics_and_jacobian(q)
            error = target - current
            error_norm = float(np.linalg.norm(error))
            if error_norm <= tolerance:
                return q, True, iteration + 1, error_norm

            system_matrix = jacobian @ jacobian.T + (damping * damping) * np.eye(3, dtype=np.float64)
            delta_q = jacobian.T @ np.linalg.solve(system_matrix, error)

            delta_norm = float(np.linalg.norm(delta_q))
            if delta_norm > 0.25:
                delta_q *= 0.25 / delta_norm

            q = np.clip(q + step_scale * delta_q, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

        final_error = float(np.linalg.norm(target - self.forward_position(q)))
        return q, False, max_iterations, final_error
