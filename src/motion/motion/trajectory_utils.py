from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


def line_path(start: Sequence[float], end: Sequence[float], num_points: int) -> np.ndarray:
    if num_points < 2:
        raise ValueError('num_points must be >= 2')
    start_vec = np.asarray(start, dtype=np.float64)
    end_vec = np.asarray(end, dtype=np.float64)
    alphas = np.linspace(0.0, 1.0, num_points, dtype=np.float64)
    return start_vec[None, :] + (end_vec - start_vec)[None, :] * alphas[:, None]


def circle_bump_path(
    center: Sequence[float],
    radius: float,
    bump_height: float,
    num_points: int,
) -> np.ndarray:
    if num_points < 4:
        raise ValueError('num_points must be >= 4')

    center_vec = np.asarray(center, dtype=np.float64)
    angles = np.linspace(0.0, 2.0 * math.pi, num_points, endpoint=True, dtype=np.float64)

    x_values = center_vec[0] + radius * np.cos(angles)
    y_values = center_vec[1] + radius * np.sin(angles)
    z_values = center_vec[2] + bump_height * np.sin(angles) ** 2

    return np.stack([x_values, y_values, z_values], axis=1)


def rectangle_path(corners: Sequence[Sequence[float]], points_per_edge: int) -> np.ndarray:
    if len(corners) != 4:
        raise ValueError('rectangle_path expects exactly 4 corners')
    if points_per_edge < 2:
        raise ValueError('points_per_edge must be >= 2')

    corners_np = [np.asarray(corner, dtype=np.float64) for corner in corners]
    path_segments = []

    for edge_index in range(4):
        start = corners_np[edge_index]
        end = corners_np[(edge_index + 1) % 4]
        segment = line_path(start, end, points_per_edge)
        if edge_index > 0:
            segment = segment[1:, :]
        path_segments.append(segment)

    path = np.vstack(path_segments)
    if np.linalg.norm(path[0] - path[-1]) > 1e-9:
        path = np.vstack([path, path[0]])
    return path


def _duration_from_seconds(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def build_joint_trajectory(
    joint_names: Sequence[str],
    q_points: np.ndarray,
    point_dt: float,
    start_delay: float,
    start_q: np.ndarray | None,
) -> JointTrajectory:
    q_points_array = np.asarray(q_points, dtype=np.float64)
    if q_points_array.ndim != 2:
        raise ValueError('q_points must be a 2D array [N, joints]')

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)

    elapsed = float(start_delay)

    if start_q is not None:
        start_point = JointTrajectoryPoint()
        start_point.positions = [float(value) for value in np.asarray(start_q, dtype=np.float64)]
        start_point.time_from_start = _duration_from_seconds(elapsed)
        trajectory.points.append(start_point)

    for q in q_points_array:
        elapsed += float(point_dt)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in q]
        point.time_from_start = _duration_from_seconds(elapsed)
        trajectory.points.append(point)

    return trajectory


def _rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def make_path_markers(
    points: np.ndarray,
    frame_id: str,
    stamp,
    namespace: str,
    accepted: bool,
) -> MarkerArray:
    points_np = np.asarray(points, dtype=np.float64)

    if accepted:
        line_color = _rgba(0.0, 0.9, 0.2, 1.0)
        point_color = _rgba(0.0, 0.55, 1.0, 1.0)
    else:
        line_color = _rgba(1.0, 0.2, 0.1, 1.0)
        point_color = _rgba(1.0, 0.6, 0.1, 1.0)

    line_marker = Marker()
    line_marker.header.frame_id = frame_id
    line_marker.header.stamp = stamp
    line_marker.ns = namespace
    line_marker.id = 0
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.scale.x = 0.01
    line_marker.color = line_color
    line_marker.points = [
        Point(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in points_np
    ]

    waypoint_marker = Marker()
    waypoint_marker.header.frame_id = frame_id
    waypoint_marker.header.stamp = stamp
    waypoint_marker.ns = namespace
    waypoint_marker.id = 1
    waypoint_marker.type = Marker.SPHERE_LIST
    waypoint_marker.action = Marker.ADD
    waypoint_marker.pose.orientation.w = 1.0
    waypoint_marker.scale.x = 0.02
    waypoint_marker.scale.y = 0.02
    waypoint_marker.scale.z = 0.02
    waypoint_marker.color = point_color
    waypoint_marker.points = [
        Point(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in points_np
    ]

    return MarkerArray(markers=[line_marker, waypoint_marker])
