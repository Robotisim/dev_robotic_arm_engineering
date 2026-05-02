import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import torch
import requests
from PIL import Image as PILImage
from transformers import AutoProcessor, PaliGemmaForConditionalGeneration


class RGBDSubscriber(Node):
    def __init__(self):
        super().__init__("rgbd_subscriber")

        self.bridge = CvBridge()
        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.latest_camera_info_msg = None

        # Subscribe to RGB image
        self.rgb_subscription = self.create_subscription(
            Image, "/wrist_eye/image_raw", self.rgb_callback, 10
        )

        # Subscribe to depth image
        self.depth_subscription = self.create_subscription(
            Image, "/wrist_eye/depth/image_raw", self.depth_callback, 10
        )

        # Subscribe to camera info
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/wrist_eye/depth/camera_info", self.camera_info_callback, 10
        )

        self.trigger_service = self.create_service(
            Trigger, "/vlm_pose_pipeline/trigger", self.trigger_callback
        )
        self.detection_publisher = self.create_publisher(
            String, "/vlm_pose_pipeline/detections", 10
        )

        self.get_logger().info("RGBD subscriber node initialized")
        self.get_logger().info("Service ready: /vlm_pose_pipeline/trigger")
        self.get_logger().info("Publisher ready: /vlm_pose_pipeline/detections")

        model = PaliGemmaForConditionalGeneration.from_pretrained(
            "google/paligemma2-3b-mix-224",
            dtype=torch.bfloat16,
            device_map="auto",
            attn_implementation="sdpa",
        )
        processor = AutoProcessor.from_pretrained(
            "google/paligemma2-3b-mix-224",
        )
        self.model = model
        self.processor = processor

    def rgb_callback(self, msg):
        self.latest_rgb_msg = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Process RGB image here

    def depth_callback(self, msg):
        self.latest_depth_msg = msg
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        # Process depth image here

    def camera_info_callback(self, msg):
        self.latest_camera_info_msg = msg
        # self.get_logger().info(f"Camera info received: {msg.width}x{msg.height}")
        # Use camera intrinsics here

    def trigger_callback(self, request, response):
        del request
        success, message = self.run_inference_once()
        response.success = success
        response.message = message
        if success:
            detection_msg = String()
            detection_msg.data = message
            self.detection_publisher.publish(detection_msg)
        return response

    def run_inference_once(self):
        if self.latest_rgb_msg is None:
            return False, "Missing /wrist_eye/image_raw"
        if self.latest_depth_msg is None:
            return False, "Missing /wrist_eye/depth/image_raw"
        if self.latest_camera_info_msg is None:
            return False, "Missing /wrist_eye/depth/camera_info"

        try:
            self.get_logger().info("Performing inference with latest RGB-D data")
            rgb_image = self.bridge.imgmsg_to_cv2(
                self.latest_rgb_msg, desired_encoding="bgr8"
            )

            prompt = "<image> detect red ball"
            pil_rgb_image = PILImage.fromarray(rgb_image[:, :, ::-1])
            inputs = self.processor(
                images=pil_rgb_image, text=prompt, return_tensors="pt"
            ).to(self.model.device)
            output = self.model.generate(
                **inputs, max_new_tokens=50, cache_implementation="static"
            )
            inference_text = self.processor.decode(output[0], skip_special_tokens=True)
            self.get_logger().info(inference_text)
            return True, inference_text
        except Exception as exc:
            error_msg = f"Inference failed: {exc}"
            self.get_logger().error(error_msg)
            return False, error_msg


def main(args=None):
    rclpy.init(args=args)
    node = RGBDSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
