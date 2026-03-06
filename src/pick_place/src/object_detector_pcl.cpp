#include "pick_place/object_detector_pcl.hpp"

#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pick_place {

	ObjectDetectorPcl::ObjectDetectorPcl(rclcpp::NodeOptions const& options) : Node("object_detector_pcl", options) {
		RCLCPP_INFO(this->get_logger(), "Object Detector PCL Node initialized");

		this->declare_parameter("object_detector_pcl.cloud_topic", "/camera/depth/color/points");
		this->declare_parameter("object_detector_pcl.depth_min", 0.1);
		this->declare_parameter("object_detector_pcl.depth_max", 2.0);
		this->declare_parameter("object_detector_pcl.voxel_leaf_size", 0.005);
		this->declare_parameter("object_detector_pcl.plane_distance_threshold", 0.01);
		this->declare_parameter("object_detector_pcl.cluster_tolerance", 0.02);
		this->declare_parameter("object_detector_pcl.cluster_min_size", 100);
		this->declare_parameter("object_detector_pcl.cluster_max_size", 25000);
		this->declare_parameter("object_detector_pcl.axis_sign_x", 1.0);
		this->declare_parameter("object_detector_pcl.axis_sign_y", 1.0);
		this->declare_parameter("object_detector_pcl.axis_sign_z", 1.0);
		this->declare_parameter("object_detector_pcl.axis_order", "xyz");
		this->declare_parameter("object_detector_pcl.output_frame_override", "");
		this->declare_parameter("object_detector_pcl.publish_debug_clouds", true);

		std::string cloud_topic = this->get_parameter("object_detector_pcl.cloud_topic").as_string();
		depth_min_ = this->get_parameter("object_detector_pcl.depth_min").as_double();
		depth_max_ = this->get_parameter("object_detector_pcl.depth_max").as_double();
		voxel_leaf_size_ = this->get_parameter("object_detector_pcl.voxel_leaf_size").as_double();
		plane_distance_threshold_ = this->get_parameter("object_detector_pcl.plane_distance_threshold").as_double();
		cluster_tolerance_ = this->get_parameter("object_detector_pcl.cluster_tolerance").as_double();
		cluster_min_size_ = this->get_parameter("object_detector_pcl.cluster_min_size").as_int();
		cluster_max_size_ = this->get_parameter("object_detector_pcl.cluster_max_size").as_int();
		axis_sign_x_ = this->get_parameter("object_detector_pcl.axis_sign_x").as_double();
		axis_sign_y_ = this->get_parameter("object_detector_pcl.axis_sign_y").as_double();
		axis_sign_z_ = this->get_parameter("object_detector_pcl.axis_sign_z").as_double();
		axis_order_ = this->get_parameter("object_detector_pcl.axis_order").as_string();
		output_frame_override_ = this->get_parameter("object_detector_pcl.output_frame_override").as_string();
		publish_debug_clouds_ = this->get_parameter("object_detector_pcl.publish_debug_clouds").as_bool();

		if (axis_order_.size() != 3) {
			RCLCPP_WARN(this->get_logger(), "Invalid axis_order '%s'. Falling back to 'xyz'.", axis_order_.c_str());
			axis_order_ = "xyz";
		}

		cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		    cloud_topic, 10, std::bind(&ObjectDetectorPcl::cloudCallback, this, std::placeholders::_1));

		debug_centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/pick_place/debug_centroid", 10);
		object_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pick_place/object_pose", 10);
		debug_filtered_cloud_pub_ =
		    this->create_publisher<sensor_msgs::msg::PointCloud2>("/pick_place/debug_cloud_filtered", 10);
		debug_objects_cloud_pub_ =
		    this->create_publisher<sensor_msgs::msg::PointCloud2>("/pick_place/debug_cloud_objects", 10);
		debug_largest_cluster_cloud_pub_ =
		    this->create_publisher<sensor_msgs::msg::PointCloud2>("/pick_place/debug_cloud_largest_cluster", 10);

		RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cloud_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Depth range: %.2f m to %.2f m", depth_min_, depth_max_);
		RCLCPP_INFO(this->get_logger(), "Voxel leaf size: %.4f m", voxel_leaf_size_);
		RCLCPP_INFO(this->get_logger(), "Plane distance threshold: %.4f m", plane_distance_threshold_);
		RCLCPP_INFO(this->get_logger(), "Cluster tolerance: %.4f m", cluster_tolerance_);
		RCLCPP_INFO(this->get_logger(), "Cluster size range: %ld to %ld", cluster_min_size_, cluster_max_size_);
		RCLCPP_INFO(
		    this->get_logger(), "Axis sign correction: x=%.1f y=%.1f z=%.1f", axis_sign_x_, axis_sign_y_, axis_sign_z_);
		RCLCPP_INFO(this->get_logger(), "Axis order mapping: %s", axis_order_.c_str());
		RCLCPP_INFO(this->get_logger(), "Publish debug clouds: %s", publish_debug_clouds_ ? "true" : "false");
		if (!output_frame_override_.empty()) {
			RCLCPP_INFO(this->get_logger(), "Object pose frame override enabled: %s", output_frame_override_.c_str());
		}
	}

	void ObjectDetectorPcl::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *cloud);

		if (cloud->empty()) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received empty point cloud");
			return;
		}

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(depth_min_, depth_max_);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pass.filter(*cloud_filtered);

		if (cloud_filtered->empty()) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No points after depth passthrough filter");
			return;
		}

		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud_filtered);
		voxel.setLeafSize(static_cast<float>(voxel_leaf_size_),
		                  static_cast<float>(voxel_leaf_size_),
		                  static_cast<float>(voxel_leaf_size_));

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
		voxel.filter(*cloud_downsampled);

		if (publish_debug_clouds_) {
			sensor_msgs::msg::PointCloud2 debug_filtered_msg;
			pcl::toROSMsg(*cloud_downsampled, debug_filtered_msg);
			debug_filtered_msg.header = msg->header;
			debug_filtered_cloud_pub_->publish(debug_filtered_msg);
		}

		if (cloud_downsampled->empty()) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No points after voxel downsampling");
			return;
		}

		pcl::SACSegmentation<pcl::PointXYZ> segmentation;
		segmentation.setOptimizeCoefficients(true);
		segmentation.setModelType(pcl::SACMODEL_PLANE);
		segmentation.setMethodType(pcl::SAC_RANSAC);
		segmentation.setDistanceThreshold(plane_distance_threshold_);
		segmentation.setInputCloud(cloud_downsampled);

		pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
		segmentation.segment(*plane_inliers, *plane_coefficients);

		if (plane_inliers->indices.empty()) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No table plane detected");
			return;
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_downsampled);
		extract.setIndices(plane_inliers);
		extract.setNegative(true);

		pcl::PointCloud<pcl::PointXYZ>::Ptr object_points(new pcl::PointCloud<pcl::PointXYZ>);
		extract.filter(*object_points);

		if (publish_debug_clouds_) {
			sensor_msgs::msg::PointCloud2 debug_objects_msg;
			pcl::toROSMsg(*object_points, debug_objects_msg);
			debug_objects_msg.header = msg->header;
			debug_objects_cloud_pub_->publish(debug_objects_msg);
		}

		if (object_points->empty()) {
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No points above table plane");
			return;
		}

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(object_points);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
		cluster_extractor.setClusterTolerance(cluster_tolerance_);
		cluster_extractor.setMinClusterSize(static_cast<int>(cluster_min_size_));
		cluster_extractor.setMaxClusterSize(static_cast<int>(cluster_max_size_));
		cluster_extractor.setSearchMethod(tree);
		cluster_extractor.setInputCloud(object_points);
		cluster_extractor.extract(cluster_indices);

		if (cluster_indices.empty()) {
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No object clusters found");
			return;
		}

		pcl::PointIndices const* largest_cluster = nullptr;
		size_t max_points = 0;
		for (auto const& cluster : cluster_indices) {
			if (cluster.indices.size() > max_points) {
				max_points = cluster.indices.size();
				largest_cluster = &cluster;
			}
		}

		if (largest_cluster == nullptr || largest_cluster->indices.empty()) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Largest cluster invalid");
			return;
		}

		if (publish_debug_clouds_) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			largest_cluster_cloud->reserve(largest_cluster->indices.size());
			for (int index : largest_cluster->indices) {
				if (index >= 0 && static_cast<size_t>(index) < object_points->points.size()) {
					largest_cluster_cloud->push_back(object_points->points[static_cast<size_t>(index)]);
				}
			}

			sensor_msgs::msg::PointCloud2 debug_cluster_msg;
			pcl::toROSMsg(*largest_cluster_cloud, debug_cluster_msg);
			debug_cluster_msg.header = msg->header;
			debug_largest_cluster_cloud_pub_->publish(debug_cluster_msg);
		}

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*object_points, largest_cluster->indices, centroid);

		double raw_x = static_cast<double>(centroid[0]);
		double raw_y = static_cast<double>(centroid[1]);
		double raw_z = static_cast<double>(centroid[2]);

		double X = selectAxisByOrder(axis_order_[0], raw_x, raw_y, raw_z) * axis_sign_x_;
		double Y = selectAxisByOrder(axis_order_[1], raw_x, raw_y, raw_z) * axis_sign_y_;
		double Z = selectAxisByOrder(axis_order_[2], raw_x, raw_y, raw_z) * axis_sign_z_;

		geometry_msgs::msg::PoseStamped pose_msg;
		pose_msg.header = msg->header;
		if (!output_frame_override_.empty()) {
			pose_msg.header.frame_id = output_frame_override_;
		}
		pose_msg.pose.position.x = X;
		pose_msg.pose.position.y = Y;
		pose_msg.pose.position.z = Z;
		pose_msg.pose.orientation.x = 0.0;
		pose_msg.pose.orientation.y = 0.0;
		pose_msg.pose.orientation.z = 0.0;
		pose_msg.pose.orientation.w = 1.0;
		object_pose_pub_->publish(pose_msg);

		geometry_msgs::msg::Point centroid_msg;
		centroid_msg.x = X;
		centroid_msg.y = Y;
		centroid_msg.z = Z;
		debug_centroid_pub_->publish(centroid_msg);

		RCLCPP_INFO_THROTTLE(this->get_logger(),
		                     *this->get_clock(),
		                     1000,
		                     "Object cluster detected | Centroid: (%.3f, %.3f, %.3f) m | Cluster size: %zu pts",
		                     X,
		                     Y,
		                     Z,
		                     max_points);
	}

	double ObjectDetectorPcl::selectAxisByOrder(char axis_char, double x, double y, double z) const {
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
