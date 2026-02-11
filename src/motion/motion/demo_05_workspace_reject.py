import numpy as np

from .demo_base import TrajectoryDemoBase, spin_demo
from .trajectory_utils import line_path


class Demo05WorkspaceReject(TrajectoryDemoBase):
    def __init__(self) -> None:
        super().__init__(
            node_name='motion_05_workspace_reject',
            path_name='05_workspace_reject',
            point_dt=0.20,
        )

    def build_cartesian_path(self, current_q: np.ndarray) -> np.ndarray:
        _ = current_q
        # Intentionally outside x/radius bounds to demonstrate task-space rejection.
        return line_path(start=[0.72, 0.10, 0.30], end=[1.02, 0.25, 0.30], num_points=24)


def main() -> None:
    spin_demo(Demo05WorkspaceReject)
