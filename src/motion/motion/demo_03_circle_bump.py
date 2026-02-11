import numpy as np

from .demo_base import TrajectoryDemoBase, spin_demo
from .trajectory_utils import circle_bump_path


class Demo03CircleBump(TrajectoryDemoBase):
    def __init__(self) -> None:
        super().__init__(
            node_name='motion_03_circle_bump',
            path_name='03_circle_bump',
            point_dt=0.18,
        )

    def build_cartesian_path(self, current_q: np.ndarray) -> np.ndarray:
        _ = current_q
        return circle_bump_path(
            center=[0.55, 0.00, 0.30],
            radius=0.10,
            bump_height=0.10,
            num_points=72,
        )


def main() -> None:
    spin_demo(Demo03CircleBump)
