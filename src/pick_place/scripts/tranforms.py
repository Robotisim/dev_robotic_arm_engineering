#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose


class PoseComparator(Node):
    def __init__(self):
        super().__init__("pose_comparator")

        self.target_frame = "panda_link0"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Ground truth cubes in panda_link0 (from launch spawn/world offsets)
        self.gt = {
            "cube_red": (0.430, 0.040, 0.040),
            "cube_green": (0.515, 0.100, 0.040),
            "cube_blue": (0.350, 0.165, 0.040),
            "cube_yellow": (0.440, 0.200, 0.040),
            "cube_magenta": (0.260, 0.060, 0.040),
        }

        self.sub = self.create_subscription(
            PoseStamped, "/pick_place/object_pose", self.cb, 10
        )

        self.get_logger().info("Listening on /pick_place/object_pose")

    def cb(self, msg: PoseStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time()
            )
            p = do_transform_pose(msg.pose, tf)
            x, y, z = p.position.x, p.position.y, p.position.z
        except TransformException as ex:
            self.get_logger().warn(f"TF failed: {ex}")
            return

        best_name = None
        best_dist = float("inf")
        best_err = (0.0, 0.0, 0.0)

        for name, (gx, gy, gz) in self.gt.items():
            ex = x - gx
            ey = y - gy
            ez = z - gz
            d = math.sqrt(ex * ex + ey * ey + ez * ez)
            if d < best_dist:
                best_dist = d
                best_name = name
                best_err = (ex, ey, ez)

        self.get_logger().info(
            f"det@{self.target_frame}: ({x:.3f}, {y:.3f}, {z:.3f}) | "
            f"nearest={best_name} | err=({best_err[0]:+.3f}, {best_err[1]:+.3f}, {best_err[2]:+.3f}) m | "
            f"norm={best_dist:.3f} m"
        )


def main():
    rclpy.init()
    node = PoseComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
