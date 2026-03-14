#!/usr/bin/env python3
"""
Diagnostic tool to verify camera calibration and depth alignment.
Run this to identify why point cloud positions don't match Gazebo.
Checks if RGB detection coordinates match depth coordinates.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class CameraCalibrationCheck(Node):
    def __init__(self):
        super().__init__("camera_calibration_check")

        # CRITICAL: Enable simulation time for Gazebo
        self.declare_parameter("use_sim_time", True)

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_rgb = None
        self.latest_depth = None

        # TF for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera frame to use - try optical frame first, fall back to camera link
        self.camera_frame = "panda_wrist_eye_optical_frame"
        self.camera_frame_fallback = "panda_wrist_eye_link"
        self.base_frame = "panda_link0"

        # Color ranges for detection (same as main node)
        self.color_ranges = {
            "red_low": ((0, 100, 50), (10, 255, 255), (0, 0, 255)),
            "red_high": ((170, 100, 50), (180, 255, 255), (0, 0, 255)),
            "blue": ((100, 100, 50), (130, 255, 255), (255, 0, 0)),
            "green": ((40, 100, 50), (80, 255, 255), (0, 255, 0)),
            "yellow": ((20, 100, 50), (35, 255, 255), (0, 255, 255)),
        }

        # Subscriptions
        self.rgb_sub = self.create_subscription(
            Image, "/wrist_eye/image_raw", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/wrist_eye/depth/image_raw", self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/wrist_eye/depth/camera_info", self.camera_info_callback, 10
        )

        # Timer for diagnostics
        self.create_timer(2.0, self.run_diagnostics)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Camera RGB/Depth Alignment Checker")
        self.get_logger().info("This will detect colored objects in RGB and verify")
        self.get_logger().info("if depth values exist at the same pixel coordinates")
        self.get_logger().info("=" * 60)

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("=" * 60)
            self.get_logger().info("CAMERA INTRINSICS:")
            self.get_logger().info(f"  Resolution: {msg.width}x{msg.height}")
            self.get_logger().info(f"  fx: {msg.k[0]:.2f}")
            self.get_logger().info(f"  fy: {msg.k[4]:.2f}")
            self.get_logger().info(f"  cx: {msg.k[2]:.2f}")
            self.get_logger().info(f"  cy: {msg.k[5]:.2f}")
            self.get_logger().info("=" * 60)

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        # Handle both encodings
        if msg.encoding == "16UC1":
            depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.latest_depth = depth.astype(np.float32) / 1000.0  # mm to meters
        elif msg.encoding == "32FC1":
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        else:
            self.get_logger().warn(f"Unknown depth encoding: {msg.encoding}")

    def detect_colored_objects(self, rgb_image):
        """Detect colored objects in RGB image using HSV filtering"""
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        detections = []

        for color_name, (lower, upper, bgr_color) in self.color_ranges.items():
            # Create mask
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if 500 < area < 50000:  # Same thresholds as main node
                    # Get moments for centroid
                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Get bounding box
                        x, y, w, h = cv2.boundingRect(contour)

                        detections.append(
                            {
                                "color": color_name.replace("_low", "").replace(
                                    "_high", ""
                                ),
                                "centroid": (cx, cy),
                                "bbox": (x, y, w, h),
                                "area": area,
                                "bgr_color": bgr_color,
                            }
                        )

        return detections

    def run_diagnostics(self):
        if (
            self.latest_rgb is None
            or self.latest_depth is None
            or self.camera_info is None
        ):
            self.get_logger().warn("Waiting for RGB, depth, and camera_info...")
            return

        self.get_logger().info("=" * 80)
        self.get_logger().info("RGB/DEPTH ALIGNMENT CHECK")
        self.get_logger().info("=" * 80)

        # Detect colored objects in RGB
        detections = self.detect_colored_objects(self.latest_rgb)
        self.get_logger().info(
            f"Detected {len(detections)} colored objects in RGB image"
        )

        if len(detections) == 0:
            self.get_logger().warn(
                "No colored objects detected. Make sure objects are visible!"
            )
            return

        # Create visualization images
        rgb_vis = self.latest_rgb.copy()
        depth_vis = cv2.normalize(self.latest_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.cvtColor(depth_vis.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        # Camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx_cam = self.camera_info.k[2]
        cy_cam = self.camera_info.k[5]

        # Check each detection
        for i, det in enumerate(detections):
            color_name = det["color"]
            cx, cy = det["centroid"]
            x, y, w, h = det["bbox"]
            bgr = det["bgr_color"]

            self.get_logger().info(f"\nObject {i+1} ({color_name}):")
            self.get_logger().info(f"  RGB centroid: ({cx}, {cy})")

            # Check depth at RGB centroid
            if (
                0 <= cy < self.latest_depth.shape[0]
                and 0 <= cx < self.latest_depth.shape[1]
            ):
                depth_at_rgb = self.latest_depth[cy, cx]

                # Sample 5x5 region for robustness
                y_min, y_max = max(0, cy - 2), min(self.latest_depth.shape[0], cy + 3)
                x_min, x_max = max(0, cx - 2), min(self.latest_depth.shape[1], cx + 3)
                depth_region = self.latest_depth[y_min:y_max, x_min:x_max]
                valid_depths = depth_region[
                    ~np.isnan(depth_region) & (depth_region > 0)
                ]

                if len(valid_depths) > 0:
                    median_depth = np.median(valid_depths)
                    self.get_logger().info(
                        f"  Depth at RGB centroid: {depth_at_rgb:.3f} m"
                    )
                    self.get_logger().info(
                        f"  Median depth (5x5): {median_depth:.3f} m"
                    )

                    try:
                        # Deproject to 3D
                        Z = median_depth
                        X_cam = (cx - cx_cam) * Z / fx
                        Y_cam = (cy - cy_cam) * Z / fy

                        self.get_logger().info(
                            f"  3D in camera frame: X={X_cam:.3f}, Y={Y_cam:.3f}, Z={Z:.3f}"
                        )

                        # Transform to base frame - try optical frame first, then camera link
                        source_frame = None
                        point_cam = PointStamped()
                        point_cam.header.stamp = self.get_clock().now().to_msg()
                        point_cam.point.x = X_cam
                        point_cam.point.y = Y_cam
                        point_cam.point.z = Z

                        try:
                            # Try optical frame first
                            point_cam.header.frame_id = self.camera_frame
                            point_base = self.tf_buffer.transform(
                                point_cam,
                                self.base_frame,
                                timeout=rclpy.duration.Duration(seconds=0.5),
                            )
                            source_frame = self.camera_frame
                        except Exception as e1:
                            try:
                                # Fall back to camera link frame with adjusted coordinates
                                # panda_wrist_eye_link has X=down, Y=left, Z=forward
                                # Our calculated X,Y,Z assume optical frame (X=right, Y=down, Z=forward)
                                # So we need to transform: X_link=Z, Y_link=Y, Z_link=-X
                                point_cam.header.frame_id = self.camera_frame_fallback
                                point_cam.point.x = (
                                    Z  # Depth forward becomes X in camera link
                                )
                                point_cam.point.y = Y_cam
                                point_cam.point.z = (
                                    -X_cam
                                )  # Down becomes -Z in camera link
                                point_base = self.tf_buffer.transform(
                                    point_cam,
                                    self.base_frame,
                                    timeout=rclpy.duration.Duration(seconds=0.5),
                                )
                                source_frame = self.camera_frame_fallback
                                self.get_logger().info(
                                    f"  (Using {self.camera_frame_fallback} - optical frame not available)"
                                )
                            except Exception as e2:
                                self.get_logger().warn(
                                    f"  Transform failed with both frames:"
                                )
                                self.get_logger().warn(
                                    f"    {self.camera_frame}: {str(e1)}"
                                )
                                self.get_logger().warn(
                                    f"    {self.camera_frame_fallback}: {str(e2)}"
                                )

                        if source_frame:
                            self.get_logger().info(
                                f"  3D in base frame ({self.base_frame}):"
                            )
                            self.get_logger().info(
                                f"    X={point_base.point.x:.3f}, "
                                f"Y={point_base.point.y:.3f}, Z={point_base.point.z:.3f}"
                            )

                            # Draw on visualizations
                            cv2.rectangle(rgb_vis, (x, y), (x + w, y + h), bgr, 2)
                            cv2.circle(rgb_vis, (cx, cy), 5, bgr, -1)
                            cv2.putText(
                                rgb_vis,
                                f"{color_name}",
                                (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                bgr,
                                2,
                            )
                            cv2.putText(
                                rgb_vis,
                                f"({cx},{cy})",
                                (x, y + h + 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.4,
                                bgr,
                                1,
                            )

                            cv2.rectangle(depth_vis, (x, y), (x + w, y + h), bgr, 2)
                            cv2.circle(depth_vis, (cx, cy), 5, bgr, -1)
                            cv2.putText(
                                depth_vis,
                                f"{median_depth:.2f}m",
                                (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                bgr,
                                2,
                            )
                    except Exception as e:
                        self.get_logger().error(f"  Unexpected error: {str(e)}")
                else:
                    self.get_logger().warn(f"  No valid depth at RGB centroid!")
                    cv2.rectangle(rgb_vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.circle(rgb_vis, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(
                        rgb_vis,
                        f"{color_name} - NO DEPTH",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                    )

        # Save visualization images
        cv2.imwrite("/tmp/rgb_detections.jpg", rgb_vis)
        cv2.imwrite("/tmp/depth_overlay.jpg", depth_vis)
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Visualization saved:")
        self.get_logger().info("  RGB with detections: /tmp/rgb_detections.jpg")
        self.get_logger().info("  Depth with overlay: /tmp/depth_overlay.jpg")
        self.get_logger().info("=" * 80)

        # Show images if display available
        try:
            cv2.imshow("RGB Detections", rgb_vis)
            cv2.imshow("Depth Overlay", depth_vis)
            cv2.waitKey(1)
        except:
            pass  # Headless environment


def main():
    rclpy.init()

    # Create node with simulation time enabled
    node = CameraCalibrationCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
