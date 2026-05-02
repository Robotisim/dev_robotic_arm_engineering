#!/usr/bin/env python3
"""
PCD Snapshot Capture Node

This node captures point cloud data from an RGB-D camera and saves it as PCD files.
It can be triggered via a ROS2 service or automatically at intervals.

Usage:
    ros2 run util pcd_snapshot.py --ros-args -p camera_name:=wrist_eye
    
Service call to capture:
    ros2 service call /capture_pcd std_srvs/srv/Trigger
"""

import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d


class PCDSnapshotNode(Node):
    def __init__(self):
        super().__init__('pcd_snapshot_node')
        
        # Parameters
        self.declare_parameter('camera_name', 'wrist_eye')
        self.declare_parameter('output_dir', '/tmp/pcd_snapshots')
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('capture_interval', 5.0)  # seconds
        self.declare_parameter('min_depth', 0.1)  # meters
        self.declare_parameter('max_depth', 3.0)  # meters
        
        self.camera_name = self.get_parameter('camera_name').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.auto_capture = self.get_parameter('auto_capture').value
        self.capture_interval = self.get_parameter('capture_interval').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        
        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Saving PCD files to: {self.output_dir}')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Storage for synchronized data
        self.latest_depth_image = None
        self.latest_rgb_image = None
        self.latest_camera_info = None
        self.data_ready = False
        
        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        depth_topic = f'/{self.camera_name}/depth/image_raw'
        rgb_topic = f'/{self.camera_name}/image_raw'
        camera_info_topic = f'/{self.camera_name}/depth/camera_info'
        
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile
        )
        
        self.rgb_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )
        
        # Service for manual capture
        self.capture_service = self.create_service(
            Trigger,
            'capture_pcd',
            self.capture_service_callback
        )
        
        # Timer for auto capture
        if self.auto_capture:
            self.capture_timer = self.create_timer(
                self.capture_interval,
                self.auto_capture_callback
            )
            self.get_logger().info(
                f'Auto-capture enabled. Capturing every {self.capture_interval} seconds.'
            )
        
        self.get_logger().info(f'PCD Snapshot Node initialized for camera: {self.camera_name}')
        self.get_logger().info(f'Listening to: {depth_topic}, {rgb_topic}, {camera_info_topic}')
        self.get_logger().info('Call /capture_pcd service to save a snapshot')
    
    def depth_callback(self, msg):
        """Store latest depth image."""
        self.latest_depth_image = msg
        self._check_data_ready()
    
    def rgb_callback(self, msg):
        """Store latest RGB image."""
        self.latest_rgb_image = msg
        self._check_data_ready()
    
    def camera_info_callback(self, msg):
        """Store latest camera info."""
        self.latest_camera_info = msg
        self._check_data_ready()
    
    def _check_data_ready(self):
        """Check if all required data is available."""
        self.data_ready = (
            self.latest_depth_image is not None and
            self.latest_rgb_image is not None and
            self.latest_camera_info is not None
        )
    
    def capture_service_callback(self, request, response):
        """Handle service call to capture PCD."""
        if not self.data_ready:
            response.success = False
            response.message = 'Camera data not ready. Waiting for depth, RGB, and camera_info.'
            self.get_logger().warn(response.message)
            return response
        
        try:
            filename = self.capture_pcd()
            response.success = True
            response.message = f'PCD saved to: {filename}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to capture PCD: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def auto_capture_callback(self):
        """Automatically capture PCD at intervals."""
        if self.data_ready:
            try:
                filename = self.capture_pcd()
                self.get_logger().info(f'Auto-captured: {filename}')
            except Exception as e:
                self.get_logger().error(f'Auto-capture failed: {str(e)}')
    
    def capture_pcd(self):
        """Capture and save point cloud data."""
        # Convert images
        depth_image = self.bridge.imgmsg_to_cv2(
            self.latest_depth_image,
            desired_encoding='passthrough'
        )
        
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                self.latest_rgb_image,
                desired_encoding='rgb8'
            )
        except Exception:
            # If RGB conversion fails, use grayscale
            rgb_image = self.bridge.imgmsg_to_cv2(
                self.latest_rgb_image,
                desired_encoding='passthrough'
            )
            if len(rgb_image.shape) == 2:
                rgb_image = np.stack([rgb_image] * 3, axis=-1)
        
        # Get camera intrinsics
        K = np.array(self.latest_camera_info.k).reshape(3, 3)
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        
        # Convert depth to meters if needed
        if depth_image.dtype == np.uint16:
            depth_image = depth_image.astype(np.float32) / 1000.0
        elif depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
            # Already in float, assume it's in meters
            depth_image = depth_image.astype(np.float32)
        
        # Filter depth values
        valid_mask = (depth_image > self.min_depth) & (depth_image < self.max_depth)
        
        # Generate point cloud
        height, width = depth_image.shape
        
        # Create pixel coordinate grids
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        
        # Calculate 3D coordinates
        z = depth_image
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Stack coordinates
        points = np.stack([x, y, z], axis=-1)
        
        # Flatten and filter
        points = points.reshape(-1, 3)
        colors = rgb_image.reshape(-1, 3).astype(np.float32) / 255.0
        valid_mask = valid_mask.flatten()
        
        points = points[valid_mask]
        colors = colors[valid_mask]
        
        if len(points) == 0:
            raise ValueError('No valid points in depth image')
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Generate filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = self.output_dir / f'snapshot_{self.camera_name}_{timestamp}.pcd'
        
        # Save PCD file
        o3d.io.write_point_cloud(str(filename), pcd, write_ascii=False)
        
        self.get_logger().info(f'Saved {len(points)} points to {filename}')
        
        return str(filename)


def main(args=None):
    rclpy.init(args=args)
    node = PCDSnapshotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
