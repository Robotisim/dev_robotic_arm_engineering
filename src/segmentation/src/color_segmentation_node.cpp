#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <image_transport/image_transport.hpp>>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

struct ColorRange {
	std::string name;
	cv::Scalar lower_hsv;
	cv::Scalar upper_hsv;
	cv::Scalar visualization_color; // BGR for drawing
};

struct DetectedObject {
	std::string color;
	cv::Point2d centroid; // 2D pixel coordinates
	cv::Rect bounding_box;
	double area;
	int pixel_count;
	// Phase 2: 3D information
	cv::Point3d position_3d;      // 3D position in camera frame (meters)
	bool has_valid_depth = false; // Whether depth was successfully obtained
	double depth_value = 0.0;     // Raw depth value (meters)
	// Phase 3: Transformed position
	geometry_msgs::msg::Point position_base; // 3D position in robot base frame (meters)
	bool has_transform = false;              // Whether transform was successful
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

		// Phase 2 parameters
		this->declare_parameter("depth_processing_enabled", true);
		this->declare_parameter("depth_min_valid", 0.1);   // Minimum valid depth (meters)
		this->declare_parameter("depth_max_valid", 3.0);   // Maximum valid depth (meters)
		this->declare_parameter("depth_sample_radius", 2); // Radius for median depth sampling

		// Phase 3 parameters
		this->declare_parameter("transform_enabled", true);
		this->declare_parameter("target_frame", "panda_link0");          // Robot base frame
		this->declare_parameter("source_frame", "panda_wrist_eye_link"); // Camera frame
		this->declare_parameter("use_image_frame", false);               // Use frame from image header
		this->declare_parameter("tf_timeout", 0.1);                      // TF lookup timeout (seconds)

		// Get parameters
		min_area_ = this->get_parameter("min_area").as_double();
		max_area_ = this->get_parameter("max_area").as_double();
		min_aspect_ratio_ = this->get_parameter("min_aspect_ratio").as_double();
		max_aspect_ratio_ = this->get_parameter("max_aspect_ratio").as_double();
		morph_kernel_size_ = this->get_parameter("morph_kernel_size").as_int();
		visualization_enabled_ = this->get_parameter("visualization_enabled").as_bool();
		debug_images_ = this->get_parameter("debug_images").as_bool();

		// Phase 2 parameters
		depth_processing_enabled_ = this->get_parameter("depth_processing_enabled").as_bool();
		depth_min_valid_ = this->get_parameter("depth_min_valid").as_double();
		depth_max_valid_ = this->get_parameter("depth_max_valid").as_double();
		depth_sample_radius_ = this->get_parameter("depth_sample_radius").as_int();

		// Phase 3 parameters
		transform_enabled_ = this->get_parameter("transform_enabled").as_bool();
		target_frame_ = this->get_parameter("target_frame").as_string();
		source_frame_ = this->get_parameter("source_frame").as_string();
		use_image_frame_ = this->get_parameter("use_image_frame").as_bool();
		tf_timeout_ = this->get_parameter("tf_timeout").as_double();

		// Initialize color ranges for different cubes
		initializeColorRanges();

		// Create subscribers
		image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		    "/wrist_eye/image_raw", 10, std::bind(&ColorSegmentationNode::imageCallback, this, std::placeholders::_1));

		// Phase 2: Subscribe to depth and camera_info
		if (depth_processing_enabled_) {
			depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
			    "/wrist_eye/depth/image_raw",
			    10,
			    std::bind(&ColorSegmentationNode::depthCallback, this, std::placeholders::_1));

			camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			    "/wrist_eye/depth/camera_info",
			    10,
			    std::bind(&ColorSegmentationNode::cameraInfoCallback, this, std::placeholders::_1));
		}

		// Create publishers
		annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/annotated_image", 10);
		markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/detection_markers", 10);
		detections_pub_ = this->create_publisher<std_msgs::msg::String>("~/detections", 10);

		// Phase 2: Publish point cloud
		if (depth_processing_enabled_) {
			pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/detected_objects_cloud", 10);
		}

		// Phase 3: Publish pose array and initialize TF
		if (transform_enabled_) {
			pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/object_poses_base", 10);
			tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		}

		RCLCPP_INFO(this->get_logger(), "Color Segmentation Node initialized");
		RCLCPP_INFO(this->get_logger(), "Subscribing to: /wrist_eye/image_raw");
		if (depth_processing_enabled_) {
			RCLCPP_INFO(this->get_logger(), "Depth processing enabled");
			RCLCPP_INFO(this->get_logger(), "Subscribing to: /wrist_eye/depth/image_raw, /wrist_eye/depth/camera_info");
		}
		if (transform_enabled_) {
			RCLCPP_INFO(this->get_logger(), "Transform enabled: camera frame → %s", target_frame_.c_str());
		}
		RCLCPP_INFO(this->get_logger(), "Publishing to: ~/annotated_image, ~/detection_markers, ~/detections");
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

	std::string formatDetectionsForDebug(std::vector<DetectedObject> const& detections) {
		if (detections.empty()) {
			return "[none]";
		}

		std::stringstream ss;
		ss << "[";
		for (size_t i = 0; i < detections.size(); i++) {
			auto const& det = detections[i];
			if (i > 0) ss << "; ";

			ss << det.color << " @ pixel(" << static_cast<int>(det.centroid.x) << "," << static_cast<int>(det.centroid.y)
			   << ")";

			if (det.has_valid_depth) {
				ss << " cam(" << std::fixed << std::setprecision(3) << det.position_3d.x << "," << det.position_3d.y << ","
				   << det.position_3d.z << ")m";
			}

			if (det.has_transform) {
				ss << " base(" << std::fixed << std::setprecision(3) << det.position_base.x << "," << det.position_base.y << ","
				   << det.position_base.z << ")m";
			}
		}
		ss << "]";
		return ss.str();
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

		// Phase 2: Process depth for 3D localization
		if (depth_processing_enabled_ && has_depth_image_ && has_camera_info_) {
			processDepthForDetections(all_detections);
		}

		// Visualize detections
		if (visualization_enabled_) {
			visualizeDetections(annotated_image, all_detections);
		}

		// Publish annotated image
		publishAnnotatedImage(annotated_image, msg->header);

		// Publish markers for RViz
		publishMarkers(all_detections, msg->header);

		// Phase 2: Publish point cloud
		if (depth_processing_enabled_ && has_depth_image_ && has_camera_info_) {
			publishPointCloud(all_detections, msg->header);
		}

		// Phase 3: Transform to base frame and publish poses
		if (transform_enabled_ && depth_processing_enabled_) {
			transformDetectionsToBaseFrame(all_detections, msg->header);
			publishPoseArray(all_detections, msg->header);
		}

		// Publish structured detections
		publishDetections(all_detections, msg->header);

		RCLCPP_INFO(this->get_logger(), "Detected %zu objects", all_detections.size());
		if (!all_detections.empty()) {
			RCLCPP_INFO(this->get_logger(), "Detections: %s", formatDetectionsForDebug(all_detections).c_str());
		}
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

	// Phase 2: Depth processing callbacks and methods

	void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
		// Store depth image for synchronized processing
		try {
			// Depth images are typically 16UC1 (millimeters) or 32FC1 (meters)
			if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
				auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
				// Convert from millimeters to meters
				cv_ptr->image.convertTo(current_depth_image_, CV_32F, 0.001);
			} else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
				auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
				current_depth_image_ = cv_ptr->image;
			} else {
				RCLCPP_WARN_THROTTLE(
				    this->get_logger(), *this->get_clock(), 5000, "Unsupported depth encoding: %s", msg->encoding.c_str());
				return;
			}
			has_depth_image_ = true;
		} catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in depth callback: %s", e.what());
		}
	}

	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
		if (!has_camera_info_) {
			camera_model_.fromCameraInfo(msg);
			has_camera_info_ = true;
			RCLCPP_INFO(this->get_logger(),
			            "Camera info received. fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
			            camera_model_.fx(),
			            camera_model_.fy(),
			            camera_model_.cx(),
			            camera_model_.cy());
		}
	}

	double lookupDepthAtPoint(cv::Point2d const& point) {
		// Sample depth using median filter over small region for robustness
		if (current_depth_image_.empty()) {
			return 0.0;
		}

		int u = static_cast<int>(point.x);
		int v = static_cast<int>(point.y);

		// Check bounds
		if (u < 0 || u >= current_depth_image_.cols || v < 0 || v >= current_depth_image_.rows) {
			return 0.0;
		}

		// Collect depth samples in a small window
		std::vector<float> samples;
		int radius = depth_sample_radius_;

		for (int dv = -radius; dv <= radius; dv++) {
			for (int du = -radius; du <= radius; du++) {
				int sample_u = u + du;
				int sample_v = v + dv;

				if (sample_u >= 0 && sample_u < current_depth_image_.cols && sample_v >= 0 &&
				    sample_v < current_depth_image_.rows) {
					float depth = current_depth_image_.at<float>(sample_v, sample_u);

					// Filter valid depths
					if (std::isfinite(depth) && depth >= depth_min_valid_ && depth <= depth_max_valid_) {
						samples.push_back(depth);
					}
				}
			}
		}

		if (samples.empty()) {
			return 0.0;
		}

		// Return median depth for robustness
		std::sort(samples.begin(), samples.end());
		return samples[samples.size() / 2];
	}

	cv::Point3d deprojectPixelTo3D(cv::Point2d const& pixel, double depth) {
		if (!has_camera_info_ || depth <= 0.0) {
			return cv::Point3d(0, 0, 0);
		}

		// Use image_geometry to deproject
		// Formula: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy, Z = depth
		double x = (pixel.x - camera_model_.cx()) * depth / camera_model_.fx();
		double y = (pixel.y - camera_model_.cy()) * depth / camera_model_.fy();
		double z = depth;

		return cv::Point3d(x, y, z);
	}

	void processDepthForDetections(std::vector<DetectedObject>& detections) {
		if (detections.empty()) {
			return;
		}

		RCLCPP_DEBUG(this->get_logger(), "Processing depth for %zu detections", detections.size());
		size_t success_count = 0;

		for (auto& detection : detections) {
			// Look up depth at centroid
			double depth = lookupDepthAtPoint(detection.centroid);

			if (depth > 0.0) {
				// Deproject to 3D
				detection.position_3d = deprojectPixelTo3D(detection.centroid, depth);
				detection.depth_value = depth;
				detection.has_valid_depth = true;
				success_count++;
				RCLCPP_DEBUG(this->get_logger(),
				             "Depth OK: %s @ pixel(%.0f,%.0f) -> depth=%.3fm, 3D=(%.3f,%.3f,%.3f)",
				             detection.color.c_str(),
				             detection.centroid.x,
				             detection.centroid.y,
				             depth,
				             detection.position_3d.x,
				             detection.position_3d.y,
				             detection.position_3d.z);
			} else {
				detection.has_valid_depth = false;
				RCLCPP_WARN_THROTTLE(this->get_logger(),
				                     *this->get_clock(),
				                     2000,
				                     "No valid depth for %s object at (%.0f, %.0f)",
				                     detection.color.c_str(),
				                     detection.centroid.x,
				                     detection.centroid.y);
			}
		}

		RCLCPP_INFO(this->get_logger(),
		            "Depth processing: %zu/%zu objects got valid 3D positions",
		            success_count,
		            detections.size());
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
			marker.header.frame_id = source_frame_; // Use configured camera frame
			marker.ns = "detections";
			marker.id = id++;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;

			// Phase 2: Use 3D position if available, otherwise use scaled pixel coordinates
			if (detection.has_valid_depth) {
				marker.pose.position.x = detection.position_3d.x;
				marker.pose.position.y = detection.position_3d.y;
				marker.pose.position.z = detection.position_3d.z;
				marker.scale.x = 0.03; // 3cm sphere
				marker.scale.y = 0.03;
				marker.scale.z = 0.03;
			} else {
				// Fallback to 2D visualization (scaled pixels)
				marker.pose.position.x = detection.centroid.x / 1000.0;
				marker.pose.position.y = detection.centroid.y / 1000.0;
				marker.pose.position.z = 0.0;
				marker.scale.x = 0.01;
				marker.scale.y = 0.01;
				marker.scale.z = 0.01;
			}

			marker.pose.orientation.w = 1.0;

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

			marker.lifetime = rclcpp::Duration::from_seconds(0.5);

			marker_array.markers.push_back(marker);

			// Add text label
			visualization_msgs::msg::Marker text_marker;
			text_marker.header = marker.header;
			text_marker.ns = "labels";
			text_marker.id = id++;
			text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
			text_marker.action = visualization_msgs::msg::Marker::ADD;
			text_marker.pose.position = marker.pose.position;
			text_marker.pose.position.z += 0.05; // Offset above sphere
			text_marker.scale.z = 0.03;
			text_marker.color = marker.color;
			text_marker.color.a = 1.0;

			if (detection.has_valid_depth) {
				text_marker.text = detection.color + "\n" + "(" +
				                   std::to_string(static_cast<int>(detection.position_3d.x * 1000)) + "," +
				                   std::to_string(static_cast<int>(detection.position_3d.y * 1000)) + "," +
				                   std::to_string(static_cast<int>(detection.position_3d.z * 1000)) + ")mm";
			} else {
				text_marker.text = detection.color + " (no depth)";
			}

			text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
			marker_array.markers.push_back(text_marker);
		}

		markers_pub_->publish(marker_array);
	}

	void publishDetections(std::vector<DetectedObject> const& detections, std_msgs::msg::Header const& header) {
		std_msgs::msg::String msg;
		std::stringstream json;

		json << "{";
		json << "\"timestamp\": " << header.stamp.sec << "." << std::setfill('0') << std::setw(9) << header.stamp.nanosec
		     << ",";
		json << "\"frame_id\": \"" << header.frame_id << "\",";
		json << "\"count\": " << detections.size() << ",";
		json << "\"detections\": [";

		for (size_t i = 0; i < detections.size(); i++) {
			auto const& det = detections[i];
			if (i > 0) json << ",";

			json << "{";
			json << "\"color\": \"" << det.color << "\",";
			json << "\"pixel\": {";
			json << "\"x\": " << static_cast<int>(det.centroid.x) << ",";
			json << "\"y\": " << static_cast<int>(det.centroid.y);
			json << "},";
			json << "\"area\": " << det.area;

			if (det.has_valid_depth) {
				json << ",\"camera_frame\": {";
				json << "\"x\": " << std::fixed << std::setprecision(4) << det.position_3d.x << ",";
				json << "\"y\": " << det.position_3d.y << ",";
				json << "\"z\": " << det.position_3d.z;
				json << "},";
				json << "\"depth\": " << det.depth_value;
			}

			if (det.has_transform) {
				json << ",\"base_frame\": {";
				json << "\"x\": " << std::fixed << std::setprecision(4) << det.position_base.x << ",";
				json << "\"y\": " << det.position_base.y << ",";
				json << "\"z\": " << det.position_base.z;
				json << "}";
			}

			json << "}";
		}

		json << "]";
		json << "}";

		msg.data = json.str();
		detections_pub_->publish(msg);
	}

	void publishPointCloud(std::vector<DetectedObject> const& detections, std_msgs::msg::Header const& header) {
		// Create point cloud message
		sensor_msgs::msg::PointCloud2 cloud_msg;
		cloud_msg.header = header;
		cloud_msg.header.frame_id = source_frame_; // Use configured camera frame

		// Define fields: x, y, z, rgb
		cloud_msg.height = 1;
		cloud_msg.width = 0; // Will be set based on number of valid detections

		sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
		modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

		// Count valid detections
		size_t valid_count = 0;
		for (auto const& detection : detections) {
			if (detection.has_valid_depth) {
				valid_count++;
			}
		}

		if (valid_count == 0) {
			return; // No valid points to publish
		}

		modifier.resize(valid_count);
		cloud_msg.width = valid_count;

		// Create iterators for point cloud data
		sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

		// Fill point cloud data
		for (auto const& detection : detections) {
			if (detection.has_valid_depth) {
				*iter_x = detection.position_3d.x;
				*iter_y = detection.position_3d.y;
				*iter_z = detection.position_3d.z;

				// Set color based on detection color
				if (detection.color == "red") {
					*iter_r = 255;
					*iter_g = 0;
					*iter_b = 0;
				} else if (detection.color == "blue") {
					*iter_r = 0;
					*iter_g = 0;
					*iter_b = 255;
				} else if (detection.color == "green") {
					*iter_r = 0;
					*iter_g = 255;
					*iter_b = 0;
				} else if (detection.color == "yellow") {
					*iter_r = 255;
					*iter_g = 255;
					*iter_b = 0;
				} else {
					*iter_r = 255;
					*iter_g = 255;
					*iter_b = 255;
				}

				++iter_x;
				++iter_y;
				++iter_z;
				++iter_r;
				++iter_g;
				++iter_b;
			}
		}

		pointcloud_pub_->publish(cloud_msg);
	}

	// Phase 3: Transform detections to base frame
	void transformDetectionsToBaseFrame(std::vector<DetectedObject>& detections, std_msgs::msg::Header const& header) {
		// Count how many detections have valid depth to transform
		size_t valid_depth_count = 0;
		for (auto const& det : detections) {
			if (det.has_valid_depth) {
				valid_depth_count++;
			}
		}

		if (valid_depth_count == 0) {
			RCLCPP_DEBUG(this->get_logger(), "No detections with valid depth to transform");
			return;
		}

		// Determine source frame: use image header or configured parameter
		std::string source_frame;
		if (use_image_frame_ && !header.frame_id.empty()) {
			source_frame = header.frame_id;
		} else {
			source_frame = source_frame_; // Use configured camera frame
		}

		RCLCPP_DEBUG(this->get_logger(),
		             "Transforming %zu detections from %s to %s",
		             valid_depth_count,
		             source_frame.c_str(),
		             target_frame_.c_str());

		size_t success_count = 0;

		for (auto& detection : detections) {
			if (!detection.has_valid_depth) {
				continue; // Skip objects without 3D position
			}

			try {
				// Create PointStamped in camera frame
				geometry_msgs::msg::PointStamped point_camera;
				point_camera.header = header;
				point_camera.header.frame_id = source_frame;
				point_camera.point.x = detection.position_3d.x;
				point_camera.point.y = detection.position_3d.y;
				point_camera.point.z = detection.position_3d.z;

				// Transform to base frame
				geometry_msgs::msg::PointStamped point_base;
				point_base = tf_buffer_->transform(point_camera, target_frame_, tf2::durationFromSec(tf_timeout_));

				// Store transformed position
				detection.position_base = point_base.point;
				detection.has_transform = true;
				success_count++;

				RCLCPP_DEBUG(this->get_logger(),
				             "Transform OK: %s cam(%.3f,%.3f,%.3f) -> base(%.3f,%.3f,%.3f)",
				             detection.color.c_str(),
				             detection.position_3d.x,
				             detection.position_3d.y,
				             detection.position_3d.z,
				             detection.position_base.x,
				             detection.position_base.y,
				             detection.position_base.z);

			} catch (tf2::TransformException const& ex) {
				RCLCPP_WARN_THROTTLE(this->get_logger(),
				                     *this->get_clock(),
				                     2000,
				                     "Could not transform %s to %s: %s",
				                     source_frame.c_str(),
				                     target_frame_.c_str(),
				                     ex.what());
				detection.has_transform = false;
			}
		}

		RCLCPP_INFO(this->get_logger(),
		            "Transform processing: %zu/%zu objects transformed to %s frame",
		            success_count,
		            valid_depth_count,
		            target_frame_.c_str());
	}

	// Phase 3: Publish PoseArray for detected objects in base frame
	void publishPoseArray(std::vector<DetectedObject> const& detections, std_msgs::msg::Header const& header) {
		geometry_msgs::msg::PoseArray pose_array;
		pose_array.header.stamp = header.stamp;
		pose_array.header.frame_id = target_frame_;

		for (auto const& detection : detections) {
			if (detection.has_transform) {
				geometry_msgs::msg::Pose pose;
				pose.position = detection.position_base;

				// Default orientation (identity quaternion - no rotation)
				pose.orientation.x = 0.0;
				pose.orientation.y = 0.0;
				pose.orientation.z = 0.0;
				pose.orientation.w = 1.0;

				pose_array.poses.push_back(pose);
			}
		}

		if (!pose_array.poses.empty()) {
			pose_array_pub_->publish(pose_array);
		}
	}

	// Subscribers and Publishers
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;            // Phase 2
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_; // Phase 2
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_; // Phase 2
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_; // Phase 3

	// Parameters
	double min_area_;
	double max_area_;
	double min_aspect_ratio_;
	double max_aspect_ratio_;
	int morph_kernel_size_;
	bool visualization_enabled_;
	bool debug_images_;

	// Phase 2 parameters
	bool depth_processing_enabled_;
	double depth_min_valid_;
	double depth_max_valid_;
	int depth_sample_radius_;

	// Phase 3 parameters
	bool transform_enabled_;
	std::string target_frame_;
	std::string source_frame_;
	bool use_image_frame_;
	double tf_timeout_;

	// Phase 2 state
	cv::Mat current_depth_image_;
	image_geometry::PinholeCameraModel camera_model_;
	bool has_depth_image_ = false;
	bool has_camera_info_ = false;

	// Phase 3 state
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
