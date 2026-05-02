import numpy as np

from .demo_base import TrajectoryDemoBase, spin_demo
from .trajectory_utils import line_path


class Demo02StraightLine(TrajectoryDemoBase):
    def __init__(self) -> None:
        super().__init__(
            node_name='motion_02_straight_line',
            path_name='02_straight_line',
            point_dt=0.20,
        )

    def build_cartesian_path(self, current_q: np.ndarray) -> np.ndarray:
        _ = current_q
        return line_path(start=[0.40, -0.25, 0.40], end=[0.66, 0.25, 0.40], num_points=40)


def main() -> None:
    spin_demo(Demo02StraightLine)
