#include "pick_place/object_detector_pcl.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <limits>
#include <sstream>

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
		this->declare_parameter("object_detector_pcl.publish_all_cluster_centroids", true);
		this->declare_parameter("object_detector_pcl.log_all_clusters", true);
		this->declare_parameter("object_detector_pcl.enable_target_tracking", false);
		this->declare_parameter("object_detector_pcl.target_init_x", 0.0);
		this->declare_parameter("object_detector_pcl.target_init_y", 0.0);
		this->declare_parameter("object_detector_pcl.target_init_z", 0.0);
		this->declare_parameter("object_detector_pcl.target_match_max_distance", 0.10);
		this->declare_parameter("object_detector_pcl.target_prefilter_enabled", true);
		this->declare_parameter("object_detector_pcl.target_prefilter_radius", 0.12);
		this->declare_parameter("object_detector_pcl.target_prefilter_min_points", 80);
		this->declare_parameter("object_detector_pcl.target_fallback_on_no_match", true);
		this->declare_parameter("object_detector_pcl.target_fallback_max_distance", 0.20);
		this->declare_parameter("object_detector_pcl.target_anchor_to_init", true);
		this->declare_parameter("object_detector_pcl.target_anchor_max_distance", 0.06);
		this->declare_parameter("object_detector_pcl.target_update_on_fallback", false);
		this->declare_parameter("object_detector_pcl.target_reject_large_clusters", true);
		this->declare_parameter("object_detector_pcl.target_max_cluster_extent_x", 0.14);
		this->declare_parameter("object_detector_pcl.target_max_cluster_extent_y", 0.14);
		this->declare_parameter("object_detector_pcl.target_max_cluster_extent_z", 0.14);
		this->declare_parameter("object_detector_pcl.target_max_cluster_points", 5000.0);

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
		publish_all_cluster_centroids_ = this->get_parameter("object_detector_pcl.publish_all_cluster_centroids").as_bool();
		log_all_clusters_ = this->get_parameter("object_detector_pcl.log_all_clusters").as_bool();
		enable_target_tracking_ = this->get_parameter("object_detector_pcl.enable_target_tracking").as_bool();
		target_init_x_ = this->get_parameter("object_detector_pcl.target_init_x").as_double();
		target_init_y_ = this->get_parameter("object_detector_pcl.target_init_y").as_double();
		target_init_z_ = this->get_parameter("object_detector_pcl.target_init_z").as_double();
		target_match_max_distance_ = this->get_parameter("object_detector_pcl.target_match_max_distance").as_double();
		target_prefilter_enabled_ = this->get_parameter("object_detector_pcl.target_prefilter_enabled").as_bool();
		target_prefilter_radius_ = this->get_parameter("object_detector_pcl.target_prefilter_radius").as_double();
		target_prefilter_min_points_ = this->get_parameter("object_detector_pcl.target_prefilter_min_points").as_int();
		target_fallback_on_no_match_ = this->get_parameter("object_detector_pcl.target_fallback_on_no_match").as_bool();
		target_fallback_max_distance_ = this->get_parameter("object_detector_pcl.target_fallback_max_distance").as_double();
		target_anchor_to_init_ = this->get_parameter("object_detector_pcl.target_anchor_to_init").as_bool();
		target_anchor_max_distance_ = this->get_parameter("object_detector_pcl.target_anchor_max_distance").as_double();
		target_update_on_fallback_ = this->get_parameter("object_detector_pcl.target_update_on_fallback").as_bool();
		target_reject_large_clusters_ = this->get_parameter("object_detector_pcl.target_reject_large_clusters").as_bool();
		target_max_cluster_extent_x_ = this->get_parameter("object_detector_pcl.target_max_cluster_extent_x").as_double();
		target_max_cluster_extent_y_ = this->get_parameter("object_detector_pcl.target_max_cluster_extent_y").as_double();
		target_max_cluster_extent_z_ = this->get_parameter("object_detector_pcl.target_max_cluster_extent_z").as_double();
		target_max_cluster_points_ = this->get_parameter("object_detector_pcl.target_max_cluster_points").as_double();

		has_tracked_target_ = false;
		tracked_target_x_ = 0.0;
		tracked_target_y_ = 0.0;
		tracked_target_z_ = 0.0;

		if (axis_order_.size() != 3) {
			RCLCPP_WARN(this->get_logger(), "Invalid axis_order '%s'. Falling back to 'xyz'.", axis_order_.c_str());
			axis_order_ = "xyz";
		}

		cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		    cloud_topic, 10, std::bind(&ObjectDetectorPcl::cloudCallback, this, std::placeholders::_1));

		debug_centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/pick_place/debug_centroid", 10);
		debug_all_cluster_centroids_pub_ =
		    this->create_publisher<geometry_msgs::msg::PoseArray>("/pick_place/debug_all_cluster_centroids", 10);
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
		RCLCPP_INFO(this->get_logger(),
		            "Publish all-cluster centroids: %s | Log all clusters: %s",
		            publish_all_cluster_centroids_ ? "true" : "false",
		            log_all_clusters_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "Target tracking enabled: %s", enable_target_tracking_ ? "true" : "false");
		if (enable_target_tracking_) {
			RCLCPP_INFO(
			    this->get_logger(),
			    "Target init seed: (%.5f, %.5f, %.5f) m | Match gate: %.3f m | Prefilter: %s (r=%.3f, min_pts=%ld) | "
			    "Fallback: %s (%.3f m) | Anchor init: %s (%.3f m) | Update on fallback: %s | Reject large clusters: %s",
			    target_init_x_,
			    target_init_y_,
			    target_init_z_,
			    target_match_max_distance_,
			    target_prefilter_enabled_ ? "on" : "off",
			    target_prefilter_radius_,
			    target_prefilter_min_points_,
			    target_fallback_on_no_match_ ? "on" : "off",
			    target_fallback_max_distance_,
			    target_anchor_to_init_ ? "on" : "off",
			    target_anchor_max_distance_,
			    target_update_on_fallback_ ? "true" : "false",
			    target_reject_large_clusters_ ? "on" : "off");
		}
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

		if (publish_all_cluster_centroids_ || log_all_clusters_) {
			pcl::search::KdTree<pcl::PointXYZ>::Ptr all_tree(new pcl::search::KdTree<pcl::PointXYZ>);
			all_tree->setInputCloud(object_points);

			std::vector<pcl::PointIndices> all_cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> all_cluster_extractor;
			all_cluster_extractor.setClusterTolerance(cluster_tolerance_);
			all_cluster_extractor.setMinClusterSize(static_cast<int>(cluster_min_size_));
			all_cluster_extractor.setMaxClusterSize(static_cast<int>(cluster_max_size_));
			all_cluster_extractor.setSearchMethod(all_tree);
			all_cluster_extractor.setInputCloud(object_points);
			all_cluster_extractor.extract(all_cluster_indices);

			if (publish_all_cluster_centroids_) {
				geometry_msgs::msg::PoseArray centroid_array_msg;
				centroid_array_msg.header = msg->header;
				if (!output_frame_override_.empty()) {
					centroid_array_msg.header.frame_id = output_frame_override_;
				}

				for (auto const& cluster : all_cluster_indices) {
					if (cluster.indices.empty()) {
						continue;
					}

					Eigen::Vector4f cluster_centroid;
					pcl::compute3DCentroid(*object_points, cluster.indices, cluster_centroid);

					double raw_x = static_cast<double>(cluster_centroid[0]);
					double raw_y = static_cast<double>(cluster_centroid[1]);
					double raw_z = static_cast<double>(cluster_centroid[2]);

					geometry_msgs::msg::Pose pose;
					pose.position.x = selectAxisByOrder(axis_order_[0], raw_x, raw_y, raw_z) * axis_sign_x_;
					pose.position.y = selectAxisByOrder(axis_order_[1], raw_x, raw_y, raw_z) * axis_sign_y_;
					pose.position.z = selectAxisByOrder(axis_order_[2], raw_x, raw_y, raw_z) * axis_sign_z_;
					pose.orientation.w = 1.0;
					centroid_array_msg.poses.push_back(pose);
				}

				debug_all_cluster_centroids_pub_->publish(centroid_array_msg);
			}

			if (log_all_clusters_) {
				std::ostringstream summary;
				summary << "All object clusters: " << all_cluster_indices.size();

				size_t max_report = 8;
				for (size_t cluster_id = 0; cluster_id < all_cluster_indices.size() && cluster_id < max_report; ++cluster_id) {
					auto const& cluster = all_cluster_indices[cluster_id];
					if (cluster.indices.empty()) {
						continue;
					}

					Eigen::Vector4f cluster_centroid;
					pcl::compute3DCentroid(*object_points, cluster.indices, cluster_centroid);

					double raw_x = static_cast<double>(cluster_centroid[0]);
					double raw_y = static_cast<double>(cluster_centroid[1]);
					double raw_z = static_cast<double>(cluster_centroid[2]);
					double mapped_x = selectAxisByOrder(axis_order_[0], raw_x, raw_y, raw_z) * axis_sign_x_;
					double mapped_y = selectAxisByOrder(axis_order_[1], raw_x, raw_y, raw_z) * axis_sign_y_;
					double mapped_z = selectAxisByOrder(axis_order_[2], raw_x, raw_y, raw_z) * axis_sign_z_;

					summary << " | #" << cluster_id << " n=" << cluster.indices.size() << " c=(" << mapped_x << "," << mapped_y
					        << "," << mapped_z << ")";
				}

				RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", summary.str().c_str());
			}
		}

		double target_x = target_init_x_;
		double target_y = target_init_y_;
		double target_z = target_init_z_;
		if (has_tracked_target_) {
			target_x = tracked_target_x_;
			target_y = tracked_target_y_;
			target_z = tracked_target_z_;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr clustering_input = object_points;
		if (enable_target_tracking_ && target_prefilter_enabled_) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr nearby_points(new pcl::PointCloud<pcl::PointXYZ>);
			nearby_points->reserve(object_points->points.size());
			for (auto const& point : object_points->points) {
				double raw_x = static_cast<double>(point.x);
				double raw_y = static_cast<double>(point.y);
				double raw_z = static_cast<double>(point.z);

				double mapped_x = selectAxisByOrder(axis_order_[0], raw_x, raw_y, raw_z) * axis_sign_x_;
				double mapped_y = selectAxisByOrder(axis_order_[1], raw_x, raw_y, raw_z) * axis_sign_y_;
				double mapped_z = selectAxisByOrder(axis_order_[2], raw_x, raw_y, raw_z) * axis_sign_z_;

				double distance = distance3d(mapped_x, mapped_y, mapped_z, target_x, target_y, target_z);
				if (distance <= target_prefilter_radius_) {
					nearby_points->push_back(point);
				}
			}

			if (static_cast<int64_t>(nearby_points->points.size()) >= target_prefilter_min_points_) {
				clustering_input = nearby_points;
			}
		}

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(clustering_input);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
		cluster_extractor.setClusterTolerance(cluster_tolerance_);
		cluster_extractor.setMinClusterSize(static_cast<int>(cluster_min_size_));
		cluster_extractor.setMaxClusterSize(static_cast<int>(cluster_max_size_));
		cluster_extractor.setSearchMethod(tree);
		cluster_extractor.setInputCloud(clustering_input);
		cluster_extractor.extract(cluster_indices);

		if (cluster_indices.empty()) {
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No object clusters found");
			return;
		}

		pcl::PointIndices const* selected_cluster = nullptr;
		size_t selected_cluster_size = 0;
		double selected_x = 0.0;
		double selected_y = 0.0;
		double selected_z = 0.0;
		bool selected_is_fallback = false;

		double best_distance = std::numeric_limits<double>::max();
		double nearest_distance = std::numeric_limits<double>::max();
		pcl::PointIndices const* nearest_cluster = nullptr;
		size_t nearest_cluster_size = 0;
		double nearest_x = 0.0;
		double nearest_y = 0.0;
		double nearest_z = 0.0;
		for (auto const& cluster : cluster_indices) {
			if (cluster.indices.empty()) {
				continue;
			}

			if (target_reject_large_clusters_) {
				if (static_cast<double>(cluster.indices.size()) > target_max_cluster_points_) {
					continue;
				}

				pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				cluster_cloud->reserve(cluster.indices.size());
				for (int index : cluster.indices) {
					if (index >= 0 && static_cast<size_t>(index) < clustering_input->points.size()) {
						cluster_cloud->push_back(clustering_input->points[static_cast<size_t>(index)]);
					}
				}

				if (cluster_cloud->empty()) {
					continue;
				}

				pcl::PointXYZ min_pt;
				pcl::PointXYZ max_pt;
				pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

				double ext_raw_x = static_cast<double>(max_pt.x - min_pt.x);
				double ext_raw_y = static_cast<double>(max_pt.y - min_pt.y);
				double ext_raw_z = static_cast<double>(max_pt.z - min_pt.z);

				double ext_x = std::abs(selectAxisByOrder(axis_order_[0], ext_raw_x, ext_raw_y, ext_raw_z));
				double ext_y = std::abs(selectAxisByOrder(axis_order_[1], ext_raw_x, ext_raw_y, ext_raw_z));
				double ext_z = std::abs(selectAxisByOrder(axis_order_[2], ext_raw_x, ext_raw_y, ext_raw_z));

				if (ext_x > target_max_cluster_extent_x_ || ext_y > target_max_cluster_extent_y_ ||
				    ext_z > target_max_cluster_extent_z_) {
					continue;
				}
			}

			Eigen::Vector4f cluster_centroid;
			pcl::compute3DCentroid(*clustering_input, cluster.indices, cluster_centroid);

			double raw_x = static_cast<double>(cluster_centroid[0]);
			double raw_y = static_cast<double>(cluster_centroid[1]);
			double raw_z = static_cast<double>(cluster_centroid[2]);

			double mapped_x = selectAxisByOrder(axis_order_[0], raw_x, raw_y, raw_z) * axis_sign_x_;
			double mapped_y = selectAxisByOrder(axis_order_[1], raw_x, raw_y, raw_z) * axis_sign_y_;
			double mapped_z = selectAxisByOrder(axis_order_[2], raw_x, raw_y, raw_z) * axis_sign_z_;
			double distance_to_init =
			    distance3d(mapped_x, mapped_y, mapped_z, target_init_x_, target_init_y_, target_init_z_);
			if (enable_target_tracking_ && target_anchor_to_init_ && distance_to_init > target_anchor_max_distance_) {
				continue;
			}

			if (enable_target_tracking_) {
				double distance = distance3d(mapped_x, mapped_y, mapped_z, target_x, target_y, target_z);
				if (distance < nearest_distance) {
					nearest_distance = distance;
					nearest_cluster = &cluster;
					nearest_cluster_size = cluster.indices.size();
					nearest_x = mapped_x;
					nearest_y = mapped_y;
					nearest_z = mapped_z;
				}
				if (distance <= target_match_max_distance_ && distance < best_distance) {
					best_distance = distance;
					selected_cluster = &cluster;
					selected_cluster_size = cluster.indices.size();
					selected_x = mapped_x;
					selected_y = mapped_y;
					selected_z = mapped_z;
				}
			} else {
				if (cluster.indices.size() > selected_cluster_size) {
					selected_cluster = &cluster;
					selected_cluster_size = cluster.indices.size();
					selected_x = mapped_x;
					selected_y = mapped_y;
					selected_z = mapped_z;
				}
			}
		}

		if (enable_target_tracking_ && (selected_cluster == nullptr || selected_cluster->indices.empty()) &&
		    target_fallback_on_no_match_ && nearest_cluster != nullptr &&
		    nearest_distance <= target_fallback_max_distance_) {
			selected_cluster = nearest_cluster;
			selected_cluster_size = nearest_cluster_size;
			selected_x = nearest_x;
			selected_y = nearest_y;
			selected_z = nearest_z;
			selected_is_fallback = true;
			RCLCPP_WARN_THROTTLE(this->get_logger(),
			                     *this->get_clock(),
			                     1000,
			                     "No in-gate match; using nearest cluster at distance %.3f m (fallback <= %.3f m)",
			                     nearest_distance,
			                     target_fallback_max_distance_);
		}

		if (selected_cluster == nullptr || selected_cluster->indices.empty()) {
			if (enable_target_tracking_) {
				RCLCPP_WARN_THROTTLE(this->get_logger(),
				                     *this->get_clock(),
				                     1000,
				                     "No cluster matched target gate %.3f m around (%.3f, %.3f, %.3f)",
				                     target_match_max_distance_,
				                     target_x,
				                     target_y,
				                     target_z);
			} else {
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Largest cluster invalid");
			}
			return;
		}

		if (publish_debug_clouds_) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			largest_cluster_cloud->reserve(selected_cluster->indices.size());
			for (int index : selected_cluster->indices) {
				if (index >= 0 && static_cast<size_t>(index) < clustering_input->points.size()) {
					largest_cluster_cloud->push_back(clustering_input->points[static_cast<size_t>(index)]);
				}
			}

			sensor_msgs::msg::PointCloud2 debug_cluster_msg;
			pcl::toROSMsg(*largest_cluster_cloud, debug_cluster_msg);
			debug_cluster_msg.header = msg->header;
			debug_largest_cluster_cloud_pub_->publish(debug_cluster_msg);
		}

		double X = selected_x;
		double Y = selected_y;
		double Z = selected_z;

		if (enable_target_tracking_) {
			if (!selected_is_fallback || target_update_on_fallback_) {
				has_tracked_target_ = true;
				tracked_target_x_ = X;
				tracked_target_y_ = Y;
				tracked_target_z_ = Z;
			}
		}

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
		                     selected_cluster_size);
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

	double ObjectDetectorPcl::distance3d(double x1, double y1, double z1, double x2, double y2, double z2) const {
		double dx = x1 - x2;
		double dy = y1 - y2;
		double dz = z1 - z2;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

} // namespace pick_place
