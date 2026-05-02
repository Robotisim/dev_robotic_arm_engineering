#!/usr/bin/env python3

import re

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class PaligemmaPoseFromImage(Node):
    def __init__(self):
        super().__init__("paligemma_pose_from_image")
        self.bridge = CvBridge()

        self.latest_depth_msg = None
        self.latest_camera_info_msg = None
        self.last_pose_msg = None

        self.loc_pattern = re.compile(r"<loc(\d{4})>")
        self.loc_bins = 1024

        self.detections_sub = self.create_subscription(
            String,
            "/vlm_pose_pipeline/detections",
            self.detections_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            "/wrist_eye/depth/image_raw",
            self.depth_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/wrist_eye/depth/camera_info",
            self.camera_info_callback,
            10,
        )

        self.point_pub = self.create_publisher(
            PoseStamped,
            "/pick_place/object_pose",
            10,
        )
        self.republish_timer = self.create_timer(0.1, self.republish_last_pose)

        self.get_logger().info("Subscribed: /vlm_pose_pipeline/detections")
        self.get_logger().info("Subscribed: /wrist_eye/depth/image_raw")
        self.get_logger().info("Subscribed: /wrist_eye/depth/camera_info")
        self.get_logger().info("Publishing: /pick_place/object_pose")

    def depth_callback(self, msg):
        self.latest_depth_msg = msg

    def camera_info_callback(self, msg):
        self.latest_camera_info_msg = msg

    def republish_last_pose(self):
        if self.last_pose_msg is not None:
            self.point_pub.publish(self.last_pose_msg)

    def detections_callback(self, msg):
        if self.latest_depth_msg is None or self.latest_camera_info_msg is None:
            self.get_logger().warn(
                "Skipping detection: waiting for depth and camera info"
            )
            return

        loc_values = self.parse_loc_tokens(msg.data)
        if loc_values is None:
            self.get_logger().warn(f"No <loc####> tokens found in: {msg.data}")
            return

        depth_img_m = self.depth_msg_to_meters(self.latest_depth_msg)
        height, width = depth_img_m.shape

        # PaLI-Gemma detect format is typically y_min, x_min, y_max, x_max in [0, 1023].
        y_min, x_min, y_max, x_max = loc_values
        u_min = self.quantized_to_pixel(x_min, width)
        u_max = self.quantized_to_pixel(x_max, width)
        v_min = self.quantized_to_pixel(y_min, height)
        v_max = self.quantized_to_pixel(y_max, height)

        u0 = max(0, min(u_min, u_max))
        u1 = min(width - 1, max(u_min, u_max))
        v0 = max(0, min(v_min, v_max))
        v1 = min(height - 1, max(v_min, v_max))
        u_center = int((u0 + u1) / 2)
        v_center = int((v0 + v1) / 2)

        depth_m = self.robust_depth_at(depth_img_m, u_center, v_center, window=5)
        if depth_m is None:
            self.get_logger().warn(
                f"No valid depth near detection center at pixel ({u_center}, {v_center})"
            )
            return

        camera_info = self.latest_camera_info_msg
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error("Invalid camera intrinsics: fx/fy must be > 0")
            return

        x_m = ((u_center - cx) / fx) * depth_m
        y_m = ((v_center - cy) / fy) * depth_m
        z_m = depth_m

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_wrist_eye_optical_frame"
        pose_msg.pose.position.x = float(x_m)
        pose_msg.pose.position.y = float(y_m)
        pose_msg.pose.position.z = float(z_m)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.last_pose_msg = pose_msg
        self.point_pub.publish(pose_msg)

        self.get_logger().info(
            "Detection pixel bbox=(%d,%d)-(%d,%d), center=(%d,%d), point=(%.3f, %.3f, %.3f) m"
            % (u0, v0, u1, v1, u_center, v_center, x_m, y_m, z_m)
        )

    def parse_loc_tokens(self, text):
        matches = self.loc_pattern.findall(text)
        if len(matches) < 4:
            return None
        return [int(matches[0]), int(matches[1]), int(matches[2]), int(matches[3])]

    def quantized_to_pixel(self, loc_value, image_size):
        loc_value = max(0, min(loc_value, self.loc_bins - 1))
        return int(round((loc_value / float(self.loc_bins - 1)) * (image_size - 1)))

    def depth_msg_to_meters(self, depth_msg):
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth_img.ndim == 3:
            depth_img = depth_img[:, :, 0]

        if np.issubdtype(depth_img.dtype, np.integer):
            return depth_img.astype(np.float32) / 1000.0
        return depth_img.astype(np.float32)

    def robust_depth_at(self, depth_img_m, u, v, window=5):
        half = window // 2
        h, w = depth_img_m.shape
        u0 = max(0, u - half)
        u1 = min(w, u + half + 1)
        v0 = max(0, v - half)
        v1 = min(h, v + half + 1)

        patch = depth_img_m[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))


def main(args=None):
    rclpy.init(args=args)
    node = PaligemmaPoseFromImage()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
