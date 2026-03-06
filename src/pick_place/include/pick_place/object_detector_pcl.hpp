#ifndef PICK_PLACE__OBJECT_DETECTOR_PCL_HPP_
#define PICK_PLACE__OBJECT_DETECTOR_PCL_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace pick_place {

	class ObjectDetectorPcl : public rclcpp::Node {
	public:
		explicit ObjectDetectorPcl(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

	private:
		void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		double selectAxisByOrder(char axis_char, double x, double y, double z) const;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_centroid_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_filtered_cloud_pub_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_objects_cloud_pub_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_largest_cluster_cloud_pub_;

		double depth_min_;
		double depth_max_;
		double voxel_leaf_size_;
		double plane_distance_threshold_;
		double cluster_tolerance_;
		int64_t cluster_min_size_;
		int64_t cluster_max_size_;
		double axis_sign_x_;
		double axis_sign_y_;
		double axis_sign_z_;
		std::string axis_order_;
		std::string output_frame_override_;
		bool publish_debug_clouds_;
	};

} // namespace pick_place

#endif // PICK_PLACE__OBJECT_DETECTOR_PCL_HPP_
