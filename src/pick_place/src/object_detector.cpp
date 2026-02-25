#include "pick_place/object_detector.hpp"

namespace pick_place {

	ObjectDetector::ObjectDetector(rclcpp::NodeOptions const& options) : Node("object_detector", options) {
		RCLCPP_INFO(this->get_logger(), "Object Detector Node initialized");

		// Declare and get parameters for depth-based segmentation
		this->declare_parameter("object_detector.depth_min", 0.1);
		this->declare_parameter("object_detector.depth_max", 2.0);
		this->declare_parameter("object_detector.min_contour_area", 500);

		depth_min_ = this->get_parameter("object_detector.depth_min").as_double();
		depth_max_ = this->get_parameter("object_detector.depth_max").as_double();
		min_contour_area_ = this->get_parameter("object_detector.min_contour_area").as_int();

		RCLCPP_INFO(this->get_logger(), "Depth range: %.2f m to %.2f m", depth_min_, depth_max_);
		RCLCPP_INFO(this->get_logger(), "Min Contour Area: %ld", min_contour_area_);

		// Subscribe to depth image topic
		depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		    "/wrist_eye/depth/image_raw", 10, std::bind(&ObjectDetector::depthCallback, this, std::placeholders::_1));

		// Publisher for debug mask
		debug_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/pick_place/debug_mask", 10);

		RCLCPP_INFO(this->get_logger(), "Subscribed to /wrist_eye/depth/image_raw");
		RCLCPP_INFO(this->get_logger(), "Publishing debug mask to /pick_place/debug_mask");
	}

	void ObjectDetector::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
		try {
			// Convert ROS depth image to OpenCV format
			// Depth images can be 32FC1 (float) or 16UC1 (uint16)
			cv_bridge::CvImagePtr cv_ptr;
			if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			} else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
				// Convert millimeters to meters for 16UC1 encoding
				cv_ptr->image.convertTo(cv_ptr->image, CV_32FC1, 0.001);
			} else {
				RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
				return;
			}

			cv::Mat depth_image = cv_ptr->image;

			// For wrist-mounted camera pointing down:
			// Find the dominant depth (table surface) using histogram
			// Then detect objects CLOSER than the table (raised above it)

			// Compute histogram to find most common depth (table surface)
			std::map<int, int> depth_histogram;
			int valid_pixels = 0;
			for (int r = 0; r < depth_image.rows; r++) {
				for (int c = 0; c < depth_image.cols; c++) {
					float depth = depth_image.at<float>(r, c);
					if (!std::isnan(depth) && !std::isinf(depth) && depth > 0) {
						int depth_mm = static_cast<int>(depth * 1000); // Convert to mm for histogram
						depth_histogram[depth_mm]++;
						valid_pixels++;
					}
				}
			}

			// Find the most common depth (mode) - this is likely the table
			int mode_depth_mm = 0;
			int max_count = 0;
			for (auto const& entry : depth_histogram) {
				if (entry.second > max_count) {
					max_count = entry.second;
					mode_depth_mm = entry.first;
				}
			}
			double table_depth = mode_depth_mm / 1000.0; // Convert back to meters

			// Detect objects CLOSER than table (box height above table)
			// Typical box height: 5-15cm
			double object_height_threshold = 0.03; // Objects at least 3cm above table
			double dynamic_min = 0.0;
			double dynamic_max = table_depth - object_height_threshold;

			// Create binary mask: pixels closer than table surface
			cv::Mat mask = cv::Mat::zeros(depth_image.size(), CV_8UC1);
			for (int r = 0; r < depth_image.rows; r++) {
				for (int c = 0; c < depth_image.cols; c++) {
					float depth = depth_image.at<float>(r, c);
					if (!std::isnan(depth) && !std::isinf(depth) && depth > dynamic_min && depth <= dynamic_max) {
						mask.at<uint8_t>(r, c) = 255;
					}
				}
			}

			int nonzero_pixels = cv::countNonZero(mask);

			RCLCPP_INFO_THROTTLE(this->get_logger(),
			                     *this->get_clock(),
			                     1000,
			                     "Table depth: %.3fm | Detecting objects closer than %.3fm | Mask: %d/%d pixels",
			                     table_depth,
			                     dynamic_max,
			                     nonzero_pixels,
			                     depth_image.rows * depth_image.cols);

			// Convert mask back to ROS image message and publish
			cv_bridge::CvImage mask_msg;
			mask_msg.header = msg->header;
			mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
			mask_msg.image = mask;

			debug_mask_pub_->publish(*mask_msg.toImageMsg());

		} catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		} catch (cv::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
		}
	}

} // namespace pick_place
