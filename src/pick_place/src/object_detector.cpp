#include "pick_place/object_detector.hpp"

namespace pick_place {

	ObjectDetector::ObjectDetector(rclcpp::NodeOptions const& options)
	    : Node("object_detector", options), camera_info_received_(false) {
		RCLCPP_INFO(this->get_logger(), "Object Detector Node initialized");

		// Declare and get parameters for depth-based segmentation
		this->declare_parameter("object_detector.depth_min", 0.1);
		this->declare_parameter("object_detector.depth_max", 2.0);
		this->declare_parameter("object_detector.min_contour_area", 500);
		this->declare_parameter("object_detector.morph_kernel_size", 5);
		this->declare_parameter("object_detector.depth_sample_region", 5);
		this->declare_parameter("object_detector.axis_sign_x", 1.0);
		this->declare_parameter("object_detector.axis_sign_y", 1.0);
		this->declare_parameter("object_detector.axis_sign_z", 1.0);
		this->declare_parameter("object_detector.axis_order", "xyz");
		this->declare_parameter("object_detector.output_frame_override", "");
		this->declare_parameter("object_detector.depth_topic", "/wrist_eye/depth/image_raw");
		this->declare_parameter("object_detector.camera_info_topic", "/wrist_eye/depth/camera_info");

		depth_min_ = this->get_parameter("object_detector.depth_min").as_double();
		depth_max_ = this->get_parameter("object_detector.depth_max").as_double();
		min_contour_area_ = this->get_parameter("object_detector.min_contour_area").as_int();
		morph_kernel_size_ = this->get_parameter("object_detector.morph_kernel_size").as_int();
		depth_sample_region_ = this->get_parameter("object_detector.depth_sample_region").as_int();
		axis_sign_x_ = this->get_parameter("object_detector.axis_sign_x").as_double();
		axis_sign_y_ = this->get_parameter("object_detector.axis_sign_y").as_double();
		axis_sign_z_ = this->get_parameter("object_detector.axis_sign_z").as_double();
		axis_order_ = this->get_parameter("object_detector.axis_order").as_string();
		output_frame_override_ = this->get_parameter("object_detector.output_frame_override").as_string();
		std::string depth_topic = this->get_parameter("object_detector.depth_topic").as_string();
		std::string camera_info_topic = this->get_parameter("object_detector.camera_info_topic").as_string();
		if (axis_order_.size() != 3) {
			RCLCPP_WARN(this->get_logger(), "Invalid axis_order '%s'. Falling back to 'xyz'.", axis_order_.c_str());
			axis_order_ = "xyz";
		}

		RCLCPP_INFO(this->get_logger(), "Depth range: %.2f m to %.2f m", depth_min_, depth_max_);
		RCLCPP_INFO(this->get_logger(), "Min Contour Area: %ld", min_contour_area_);
		RCLCPP_INFO(this->get_logger(), "Morph Kernel Size: %ld", morph_kernel_size_);
		RCLCPP_INFO(this->get_logger(), "Depth Sample Region: %ldx%ld pixels", depth_sample_region_, depth_sample_region_);
		RCLCPP_INFO(
		    this->get_logger(), "Axis sign correction: x=%.1f y=%.1f z=%.1f", axis_sign_x_, axis_sign_y_, axis_sign_z_);
		RCLCPP_INFO(this->get_logger(), "Axis order mapping: %s", axis_order_.c_str());
		if (!output_frame_override_.empty()) {
			RCLCPP_INFO(this->get_logger(), "Object pose frame override enabled: %s", output_frame_override_.c_str());
		}

		// Subscribe to depth image and camera info topics
		depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		    depth_topic, 10, std::bind(&ObjectDetector::depthCallback, this, std::placeholders::_1));

		camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		    camera_info_topic, 10, std::bind(&ObjectDetector::cameraInfoCallback, this, std::placeholders::_1));

		// Publishers for debug outputs
		debug_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/pick_place/debug_mask", 10);
		debug_centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/pick_place/debug_centroid", 10);
		object_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pick_place/object_pose", 10);

		RCLCPP_INFO(this->get_logger(), "Subscribed to %s", depth_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Subscribed to %s", camera_info_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Publishing debug mask to /pick_place/debug_mask");
		RCLCPP_INFO(this->get_logger(), "Publishing debug centroid to /pick_place/debug_centroid");
		RCLCPP_INFO(this->get_logger(), "Publishing object pose to /pick_place/object_pose");
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

			// Apply morphological operations to clean up noise
			// Opening: removes small noise (erosion then dilation)
			// Closing: fills small holes (dilation then erosion)
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_size_, morph_kernel_size_));

			cv::Mat cleaned_mask;
			cv::morphologyEx(mask, cleaned_mask, cv::MORPH_OPEN, kernel);          // Remove noise
			cv::morphologyEx(cleaned_mask, cleaned_mask, cv::MORPH_CLOSE, kernel); // Fill holes

			// Find contours in the cleaned mask
			std::vector<std::vector<cv::Point>> contours;
			cv::findContours(cleaned_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			// Find the largest contour above minimum area threshold
			double max_area = 0;
			int largest_contour_idx = -1;
			for (size_t i = 0; i < contours.size(); i++) {
				double area = cv::contourArea(contours[i]);
				if (area > max_area && area >= min_contour_area_) {
					max_area = area;
					largest_contour_idx = static_cast<int>(i);
				}
			}

			// If a valid contour is found, calculate its centroid
			if (largest_contour_idx >= 0) {
				cv::Moments m = cv::moments(contours[largest_contour_idx]);
				if (m.m00 != 0) {            // Avoid division by zero
					double cx = m.m10 / m.m00; // centroid x in pixels
					double cy = m.m01 / m.m00; // centroid y in pixels

					// Step 2.6: Look up depth value at centroid
					// OpenCV uses (row, col) indexing which is (y, x)
					int centroid_x = static_cast<int>(std::round(cx));
					int centroid_y = static_cast<int>(std::round(cy));

					// Step 2.7: Sample depth region around centroid and compute median
					int half_region = depth_sample_region_ / 2;
					std::vector<float> valid_depths;

					// Collect valid depth values from N×N region around centroid
					for (int dy = -half_region; dy <= half_region; dy++) {
						for (int dx = -half_region; dx <= half_region; dx++) {
							int sample_x = centroid_x + dx;
							int sample_y = centroid_y + dy;

							// Check bounds
							if (sample_x >= 0 && sample_x < depth_image.cols && sample_y >= 0 && sample_y < depth_image.rows) {

								float depth = depth_image.at<float>(sample_y, sample_x);

								// Filter: valid, non-zero, within range
								if (!std::isnan(depth) && !std::isinf(depth) && depth > 0 && depth >= depth_min_ &&
								    depth <= depth_max_) {
									valid_depths.push_back(depth);
								}
							}
						}
					}

					// Compute median depth from valid samples
					if (!valid_depths.empty()) {
						std::sort(valid_depths.begin(), valid_depths.end());
						float median_depth;

						if (valid_depths.size() % 2 == 0) {
							// Even number of samples: average of middle two
							size_t mid = valid_depths.size() / 2;
							median_depth = (valid_depths[mid - 1] + valid_depths[mid]) / 2.0f;
						} else {
							// Odd number: middle value
							median_depth = valid_depths[valid_depths.size() / 2];
						}

						RCLCPP_INFO(this->get_logger(),
						            "Median depth at centroid (%.1f, %.1f): %.3f m (%zu valid samples)",
						            cx,
						            cy,
						            median_depth,
						            valid_depths.size());

						// Step 2.8: Backproject to 3D using camera intrinsics
						if (camera_info_received_) {
							// Get camera intrinsics
							double fx = camera_model_.fx();
							double fy = camera_model_.fy();
							double cx_cam = camera_model_.cx();
							double cy_cam = camera_model_.cy();

							// Backproject to 3D camera frame
							// X = (u - cx) * Z / fx
							// Y = (v - cy) * Z / fy
							// Z = depth
							double raw_x = (cx - cx_cam) * median_depth / fx;
							double raw_y = (cy - cy_cam) * median_depth / fy;
							double raw_z = median_depth;

							double X = selectAxisByOrder(axis_order_[0], raw_x, raw_y, raw_z) * axis_sign_x_;
							double Y = selectAxisByOrder(axis_order_[1], raw_x, raw_y, raw_z) * axis_sign_y_;
							double Z = selectAxisByOrder(axis_order_[2], raw_x, raw_y, raw_z) * axis_sign_z_;

							RCLCPP_INFO(this->get_logger(), "3D position in camera frame: (X=%.3f, Y=%.3f, Z=%.3f) m", X, Y, Z);

							// Create and publish PoseStamped message
							geometry_msgs::msg::PoseStamped pose_msg;
							pose_msg.header = msg->header;
							if (!output_frame_override_.empty()) {
								pose_msg.header.frame_id = output_frame_override_;
							} else if (camera_info_received_ && !camera_model_.tfFrame().empty()) {
								pose_msg.header.frame_id = camera_model_.tfFrame();
							}
							pose_msg.pose.position.x = X;
							pose_msg.pose.position.y = Y;
							pose_msg.pose.position.z = Z;

							// Identity quaternion (no orientation information from depth)
							pose_msg.pose.orientation.x = 0.0;
							pose_msg.pose.orientation.y = 0.0;
							pose_msg.pose.orientation.z = 0.0;
							pose_msg.pose.orientation.w = 1.0; // w=1 for identity quaternion

							object_pose_pub_->publish(pose_msg);
						} else {
							RCLCPP_WARN_THROTTLE(this->get_logger(),
							                     *this->get_clock(),
							                     1000,
							                     "Camera info not received yet, cannot backproject to 3D");
						}
					} else {
						RCLCPP_WARN(this->get_logger(),
						            "No valid depth samples in %dx%d region around centroid",
						            depth_sample_region_,
						            depth_sample_region_);
					}

					RCLCPP_INFO_THROTTLE(this->get_logger(),
					                     *this->get_clock(),
					                     1000,
					                     "Object detected | Centroid: (%.1f, %.1f) px | Area: %.0f px² | Table: %.3fm",
					                     cx,
					                     cy,
					                     max_area,
					                     table_depth);

					// Publish centroid as Point message
					geometry_msgs::msg::Point centroid_msg;
					centroid_msg.x = cx;
					centroid_msg.y = cy;
					centroid_msg.z = 0.0; // Will be filled with depth in later steps
					debug_centroid_pub_->publish(centroid_msg);
				}
			} else {
				RCLCPP_INFO_THROTTLE(this->get_logger(),
				                     *this->get_clock(),
				                     1000,
				                     "No object detected | Table: %.3fm | Found %zu contours",
				                     table_depth,
				                     contours.size());
			}

			// Convert mask back to ROS image message and publish
			cv_bridge::CvImage mask_msg;
			mask_msg.header = msg->header;
			mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
			mask_msg.image = cleaned_mask;

			debug_mask_pub_->publish(*mask_msg.toImageMsg());

		} catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		} catch (cv::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
		}
	}

	void ObjectDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
		if (!camera_info_received_) {
			camera_model_.fromCameraInfo(msg);
			camera_info_received_ = true;

			RCLCPP_INFO(this->get_logger(), "Camera info received");
			RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", msg->width, msg->height);
			RCLCPP_INFO(this->get_logger(),
			            "  Intrinsics fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
			            camera_model_.fx(),
			            camera_model_.fy(),
			            camera_model_.cx(),
			            camera_model_.cy());
		}
	}

	double ObjectDetector::selectAxisByOrder(char axis_char, double x, double y, double z) const {
		switch (axis_char) {
			case 'x':
				return x;
			case 'y':
				return y;
			case 'z':
				return z;
			default:
				return 0.0;
		}
	}

} // namespace pick_place
