import numpy as np

from .demo_base import TrajectoryDemoBase, spin_demo


class Demo01PointToPoint(TrajectoryDemoBase):
    def __init__(self) -> None:
        super().__init__(
            node_name='motion_01_point_to_point',
            path_name='01_point_to_point',
            point_dt=1.2,
        )

    def build_cartesian_path(self, current_q: np.ndarray) -> np.ndarray:
        _ = current_q
        return np.array(
            [
                [0.48, -0.18, 0.42],
                [0.60, 0.20, 0.36],
            ],
            dtype=np.float64,
        )


def main() -> None:
    spin_demo(Demo01PointToPoint)
