import numpy as np

from .demo_base import TrajectoryDemoBase, spin_demo
from .trajectory_utils import rectangle_path


class Demo04RectangleShape(TrajectoryDemoBase):
    def __init__(self) -> None:
        super().__init__(
            node_name='motion_04_rectangle_shape',
            path_name='04_rectangle_shape',
            point_dt=0.18,
        )

    def build_cartesian_path(self, current_q: np.ndarray) -> np.ndarray:
        _ = current_q
        corners = [
            [0.45, -0.20, 0.35],
            [0.65, -0.20, 0.35],
            [0.65, 0.20, 0.35],
            [0.45, 0.20, 0.35],
        ]
        return rectangle_path(corners=corners, points_per_edge=20)


def main() -> None:
    spin_demo(Demo04RectangleShape)
