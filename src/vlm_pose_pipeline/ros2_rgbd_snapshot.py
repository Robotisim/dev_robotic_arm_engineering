#!/usr/bin/env python3
"""Capture one RGB-D snapshot plus intrinsics from ROS2 topics.

Intended use: play a ros2 bag, run this script, and it will save a single frame
that can be consumed by paligemma_pose_from_image.py.

Outputs in the target directory:
- rgb.png
- depth.png (uint16 millimeters when possible)
- depth.npy (float32 meters)
- intrinsics.json
- metadata.json
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one RGB-D + intrinsics snapshot from ROS2 topics."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory to store snapshot files",
    )
    parser.add_argument(
        "--rgb-topic",
        default="/wrist_eye/image_raw",
        help="RGB image topic",
    )
    parser.add_argument(
        "--depth-topic",
        default="/wrist_eye/depth/image_raw",
        help="Depth image topic",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/wrist_eye/depth/camera_info",
        help="CameraInfo topic",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=20.0,
        help="Max wait time before failing",
    )
    return parser.parse_args()


def msg_stamp_to_float(msg: Image | CameraInfo) -> float:
    sec = float(msg.header.stamp.sec)
    nsec = float(msg.header.stamp.nanosec)
    return sec + (nsec * 1e-9)


class SnapshotNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ros2_rgbd_snapshot")
        self.args = args
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_rgb: Image | None = None
        self.latest_depth: Image | None = None
        self.latest_info: CameraInfo | None = None

        self.create_subscription(Image, args.rgb_topic, self._rgb_cb, qos)
        self.create_subscription(Image, args.depth_topic, self._depth_cb, qos)
        self.create_subscription(CameraInfo, args.camera_info_topic, self._info_cb, qos)

        self.done = False
        self.success = False
        self.error: str | None = None

        self.get_logger().info(
            "Waiting for data on "
            f"rgb={args.rgb_topic}, depth={args.depth_topic}, info={args.camera_info_topic}"
        )

    def _rgb_cb(self, msg: Image) -> None:
        self.latest_rgb = msg
        self._try_capture()

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth = msg
        self._try_capture()

    def _info_cb(self, msg: CameraInfo) -> None:
        self.latest_info = msg
        self._try_capture()

    def _try_capture(self) -> None:
        if self.done:
            return
        if (
            self.latest_rgb is None
            or self.latest_depth is None
            or self.latest_info is None
        ):
            return

        try:
            self._save_snapshot()
            self.success = True
            self.done = True
        except Exception as exc:  # pragma: no cover - safety path
            self.error = str(exc)
            self.done = True

    def _extract_intrinsics(self, msg: CameraInfo) -> Intrinsics:
        if len(msg.k) != 9:
            raise ValueError("CameraInfo.k must contain 9 values")
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("Invalid intrinsics: fx/fy must be > 0")
        return Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy)

    def _depth_to_meters(self, depth_img: np.ndarray) -> np.ndarray:
        if depth_img.ndim == 3:
            depth_img = depth_img[:, :, 0]
        if np.issubdtype(depth_img.dtype, np.integer):
            return depth_img.astype(np.float32) / 1000.0
        return depth_img.astype(np.float32)

    def _save_snapshot(self) -> None:
        assert self.latest_rgb is not None
        assert self.latest_depth is not None
        assert self.latest_info is not None

        out_dir = self.args.output_dir
        out_dir.mkdir(parents=True, exist_ok=True)

        rgb_msg = self.latest_rgb
        depth_msg = self.latest_depth
        info_msg = self.latest_info

        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_m = self._depth_to_meters(depth_raw)
        intr = self._extract_intrinsics(info_msg)

        # Persist RGB.
        rgb_path = out_dir / "rgb.png"
        ok_rgb = cv2.imwrite(str(rgb_path), rgb_img)
        if not ok_rgb:
            raise RuntimeError(f"Failed to write {rgb_path}")

        # Persist depth in two formats: original-scale PNG and meters NPY.
        depth_png_path = out_dir / "depth.png"
        depth_npy_path = out_dir / "depth.npy"

        if np.issubdtype(depth_raw.dtype, np.integer):
            depth_png = depth_raw
        else:
            depth_png = np.clip(depth_m * 1000.0, 0, np.iinfo(np.uint16).max).astype(
                np.uint16
            )

        ok_depth = cv2.imwrite(str(depth_png_path), depth_png)
        if not ok_depth:
            raise RuntimeError(f"Failed to write {depth_png_path}")

        np.save(depth_npy_path, depth_m)

        intrinsics_path = out_dir / "intrinsics.json"
        intrinsics_payload: dict[str, float] = {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
        }
        intrinsics_path.write_text(
            json.dumps(intrinsics_payload, indent=2) + "\n", encoding="utf-8"
        )

        metadata_path = out_dir / "metadata.json"
        metadata: dict[str, Any] = {
            "rgb_topic": self.args.rgb_topic,
            "depth_topic": self.args.depth_topic,
            "camera_info_topic": self.args.camera_info_topic,
            "rgb_stamp_sec": msg_stamp_to_float(rgb_msg),
            "depth_stamp_sec": msg_stamp_to_float(depth_msg),
            "camera_info_stamp_sec": msg_stamp_to_float(info_msg),
            "depth_encoding": depth_msg.encoding,
            "rgb_encoding": rgb_msg.encoding,
            "files": {
                "rgb": rgb_path.name,
                "depth_png": depth_png_path.name,
                "depth_npy": depth_npy_path.name,
                "intrinsics": intrinsics_path.name,
            },
        }
        metadata_path.write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

        self.get_logger().info(f"Snapshot written to {out_dir}")
        self.get_logger().info(f"Use RGB: {rgb_path}")
        self.get_logger().info(f"Use depth: {depth_npy_path}")
        self.get_logger().info(f"Use intrinsics: {intrinsics_path}")


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = SnapshotNode(args)

    start_ns = node.get_clock().now().nanoseconds
    timeout_ns = int(args.timeout_sec * 1e9)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds - start_ns > timeout_ns:
                node.error = f"Timed out after {args.timeout_sec:.1f} seconds"
                node.done = True

        if not node.success:
            msg = node.error or "Capture failed"
            raise RuntimeError(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
