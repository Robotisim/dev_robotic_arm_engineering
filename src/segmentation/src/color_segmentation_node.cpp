#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

struct ColorRange {
	std::string name;
	cv::Scalar lower_hsv;
	cv::Scalar upper_hsv;
	cv::Scalar visualization_color; // BGR for drawing
};

struct DetectedObject {
	std::string color;
	cv::Point2d centroid;
	cv::Rect bounding_box;
	double area;
	int pixel_count;
};

class ColorSegmentationNode : public rclcpp::Node {
public:
	ColorSegmentationNode() : Node("color_segmentation_node") {
		// Declare parameters
		this->declare_parameter("min_area", 500.0);
		this->declare_parameter("max_area", 50000.0);
		this->declare_parameter("min_aspect_ratio", 0.5);
		this->declare_parameter("max_aspect_ratio", 2.0);
		this->declare_parameter("morph_kernel_size", 5);
		this->declare_parameter("visualization_enabled", true);
		this->declare_parameter("debug_images", false);

		// Get parameters
		min_area_ = this->get_parameter("min_area").as_double();
		max_area_ = this->get_parameter("max_area").as_double();
		min_aspect_ratio_ = this->get_parameter("min_aspect_ratio").as_double();
		max_aspect_ratio_ = this->get_parameter("max_aspect_ratio").as_double();
		morph_kernel_size_ = this->get_parameter("morph_kernel_size").as_int();
		visualization_enabled_ = this->get_parameter("visualization_enabled").as_bool();
		debug_images_ = this->get_parameter("debug_images").as_bool();

		// Initialize color ranges for different cubes
		initializeColorRanges();

		// Create subscribers
		image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		    "/wrist_eye/image_raw", 10, std::bind(&ColorSegmentationNode::imageCallback, this, std::placeholders::_1));

		// Create publishers
		annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/annotated_image", 10);

		markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/detection_markers", 10);

		RCLCPP_INFO(this->get_logger(), "Color Segmentation Node initialized");
		RCLCPP_INFO(this->get_logger(), "Subscribing to: /wrist_eye/image_raw");
		RCLCPP_INFO(this->get_logger(), "Publishing to: ~/annotated_image, ~/detection_markers");
	}

private:
	void initializeColorRanges() {
		// Red cube - Note: Red wraps around in HSV, so we need two ranges
		color_ranges_.push_back({
		    "red_low",
		    cv::Scalar(0, 100, 50),   // Lower HSV
		    cv::Scalar(10, 255, 255), // Upper HSV
		    cv::Scalar(0, 0, 255)     // Red in BGR
		});

		color_ranges_.push_back({"red_high", cv::Scalar(170, 100, 50), cv::Scalar(180, 255, 255), cv::Scalar(0, 0, 255)});

		// Blue cube
		color_ranges_.push_back({
		    "blue", cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255), cv::Scalar(255, 0, 0) // Blue in BGR
		});

		// Green cube
		color_ranges_.push_back({
		    "green", cv::Scalar(40, 100, 50), cv::Scalar(80, 255, 255), cv::Scalar(0, 255, 0) // Green in BGR
		});

		// Yellow cube (optional)
		color_ranges_.push_back({
		    "yellow", cv::Scalar(20, 100, 50), cv::Scalar(35, 255, 255), cv::Scalar(0, 255, 255) // Yellow in BGR
		});
	}

	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
		// Convert ROS image to OpenCV
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat image = cv_ptr->image;
		cv::Mat annotated_image = image.clone();

		// Convert to HSV
		cv::Mat hsv_image;
		cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

		// Detect objects for each color
		std::vector<DetectedObject> all_detections;

		for (auto const& color_range : color_ranges_) {
			auto detections = detectColorBlobs(hsv_image, color_range);
			all_detections.insert(all_detections.end(), detections.begin(), detections.end());
		}

		// Merge red detections (low and high ranges)
		mergeRedDetections(all_detections);

		// Visualize detections
		if (visualization_enabled_) {
			visualizeDetections(annotated_image, all_detections);
		}

		// Publish annotated image
		publishAnnotatedImage(annotated_image, msg->header);

		// Publish markers for RViz
		publishMarkers(all_detections, msg->header);

		RCLCPP_INFO(this->get_logger(), "Detected %zu objects", all_detections.size());
	}

	std::vector<DetectedObject> detectColorBlobs(cv::Mat const& hsv_image, ColorRange const& color_range) {
		std::vector<DetectedObject> detections;

		// Create binary mask for this color
		cv::Mat mask;
		cv::inRange(hsv_image, color_range.lower_hsv, color_range.upper_hsv, mask);

		// Apply morphological operations
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_size_, morph_kernel_size_));

		// Opening: remove noise
		cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

		// Closing: fill gaps
		cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

		// Debug images disabled to avoid highgui dependency
		// Enable by publishing mask topics if needed for debugging

		// Find contours
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// Process each contour
		for (auto const& contour : contours) {
			double area = cv::contourArea(contour);

			// Filter by area
			if (area < min_area_ || area > max_area_) {
				continue;
			}

			// Get bounding rectangle
			cv::Rect bbox = cv::boundingRect(contour);

			// Filter by aspect ratio
			double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
			if (aspect_ratio < min_aspect_ratio_ || aspect_ratio > max_aspect_ratio_) {
				continue;
			}

			// Calculate centroid using moments
			cv::Moments m = cv::moments(contour);
			cv::Point2d centroid(m.m10 / m.m00, m.m01 / m.m00);

			// Create detection
			DetectedObject detection;
			detection.color = color_range.name;
			detection.centroid = centroid;
			detection.bounding_box = bbox;
			detection.area = area;
			detection.pixel_count = cv::countNonZero(mask(bbox));

			detections.push_back(detection);
		}

		return detections;
	}

	void mergeRedDetections(std::vector<DetectedObject>& detections) {
		// Combine red_low and red_high into single "red" label
		for (auto& detection : detections) {
			if (detection.color == "red_low" || detection.color == "red_high") {
				detection.color = "red";
			}
		}
	}

	void visualizeDetections(cv::Mat& image, std::vector<DetectedObject> const& detections) {
		for (auto const& detection : detections) {
			// Determine color for visualization
			cv::Scalar color;
			if (detection.color == "red") {
				color = cv::Scalar(0, 0, 255);
			} else if (detection.color == "blue") {
				color = cv::Scalar(255, 0, 0);
			} else if (detection.color == "green") {
				color = cv::Scalar(0, 255, 0);
			} else if (detection.color == "yellow") {
				color = cv::Scalar(0, 255, 255);
			} else {
				color = cv::Scalar(255, 255, 255);
			}

			// Draw bounding box
			cv::rectangle(image, detection.bounding_box, color, 2);

			// Draw centroid
			cv::circle(image, detection.centroid, 5, color, -1);
			cv::circle(image, detection.centroid, 7, cv::Scalar(255, 255, 255), 2);

			// Add label
			std::string label = detection.color;
			cv::putText(image,
			            label,
			            cv::Point(detection.bounding_box.x, detection.bounding_box.y - 10),
			            cv::FONT_HERSHEY_SIMPLEX,
			            0.6,
			            color,
			            2);

			// Add centroid coordinates
			std::stringstream ss;
			ss << "(" << static_cast<int>(detection.centroid.x) << "," << static_cast<int>(detection.centroid.y) << ")";
			cv::putText(image,
			            ss.str(),
			            cv::Point(detection.bounding_box.x, detection.bounding_box.y + detection.bounding_box.height + 20),
			            cv::FONT_HERSHEY_SIMPLEX,
			            0.4,
			            color,
			            1);
		}

		// Add info text
		std::string info = "Detected: " + std::to_string(detections.size()) + " objects";
		cv::putText(image, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
	}

	void publishAnnotatedImage(cv::Mat const& image, std_msgs::msg::Header const& header) {
		cv_bridge::CvImage out_msg;
		out_msg.header = header;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = image;

		annotated_image_pub_->publish(*out_msg.toImageMsg());
	}

	void publishMarkers(std::vector<DetectedObject> const& detections, std_msgs::msg::Header const& header) {
		visualization_msgs::msg::MarkerArray marker_array;

		int id = 0;
		for (auto const& detection : detections) {
			visualization_msgs::msg::Marker marker;
			marker.header = header;
			marker.header.frame_id = "wrist_eye_optical_frame";
			marker.ns = "detections_2d";
			marker.id = id++;
			marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
			marker.action = visualization_msgs::msg::Marker::ADD;

			// For now, just use pixel coordinates (will be proper 3D in Phase 2)
			marker.pose.position.x = detection.centroid.x / 1000.0; // Scale for visualization
			marker.pose.position.y = detection.centroid.y / 1000.0;
			marker.pose.position.z = 0.0;

			marker.pose.orientation.w = 1.0;

			marker.scale.z = 0.05;

			// Color based on detection
			if (detection.color == "red") {
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
			} else if (detection.color == "blue") {
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0;
			} else if (detection.color == "green") {
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
			} else if (detection.color == "yellow") {
				marker.color.r = 1.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
			}
			marker.color.a = 1.0;

			marker.text = detection.color;
			marker.lifetime = rclcpp::Duration::from_seconds(0.5);

			marker_array.markers.push_back(marker);
		}

		markers_pub_->publish(marker_array);
	}

	// Subscribers and Publishers
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

	// Parameters
	double min_area_;
	double max_area_;
	double min_aspect_ratio_;
	double max_aspect_ratio_;
	int morph_kernel_size_;
	bool visualization_enabled_;
	bool debug_images_;

	// Color ranges
	std::vector<ColorRange> color_ranges_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ColorSegmentationNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
