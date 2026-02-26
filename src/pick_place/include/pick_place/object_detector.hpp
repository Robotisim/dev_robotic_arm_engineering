#ifndef PICK_PLACE__OBJECT_DETECTOR_HPP_
#define PICK_PLACE__OBJECT_DETECTOR_HPP_

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace pick_place {

	class ObjectDetector : public rclcpp::Node {
	public:
		explicit ObjectDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

	private:
		// Callback for depth image
		void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

		// Callback for camera info
		void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
		double selectAxisByOrder(char axis_char, double x, double y, double z) const;

		// Subscribers
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

		// Publishers
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_pub_;
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_centroid_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;

		// Camera model for backprojection
		image_geometry::PinholeCameraModel camera_model_;
		bool camera_info_received_;

		// Parameters for depth-based segmentation
		double depth_min_;                  // minimum depth in meters
		double depth_max_;                  // maximum depth in meters
		int64_t min_contour_area_;          // minimum contour area in pixels
		int64_t morph_kernel_size_;         // kernel size for morphological operations
		int64_t depth_sample_region_;       // N×N region size for depth averaging
		double axis_sign_x_;                // sign correction for backprojected X
		double axis_sign_y_;                // sign correction for backprojected Y
		double axis_sign_z_;                // sign correction for backprojected Z
		std::string axis_order_;            // permutation for XYZ axes: xyz, xzy, yxz, yzx, zxy, zyx
		std::string output_frame_override_; // optional frame id override for published pose
	};

} // namespace pick_place

#endif // PICK_PLACE__OBJECT_DETECTOR_HPP_
