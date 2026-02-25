#ifndef PICK_PLACE__OBJECT_DETECTOR_HPP_
#define PICK_PLACE__OBJECT_DETECTOR_HPP_

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pick_place {

	class ObjectDetector : public rclcpp::Node {
	public:
		explicit ObjectDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

	private:
		// Callback for depth image
		void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

		// Subscribers
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

		// Publishers
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_pub_;

		// Parameters for depth-based segmentation
		double depth_min_;         // minimum depth in meters
		double depth_max_;         // maximum depth in meters
		int64_t min_contour_area_; // minimum contour area in pixels
	};

} // namespace pick_place

#endif // PICK_PLACE__OBJECT_DETECTOR_HPP_
